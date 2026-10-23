/******************************************************************************
 * @file   lte_sm.c
 * @brief  LTE modem state machine (boot -> attach -> PDP -> TCP -> MQTT -> RUN)
 *
 * HOW THE MCU AND THE MODEM TALK
 * ------------------------------
 * The modem is a separate computer. The MCU sends text commands (AT commands)
 * over a UART and the modem answers with text lines:
 *
 *    - final result : "OK" or "ERROR" (or "+CME ERROR: <n>") ends a command
 *    - info line    : e.g. "+CPIN: READY", comes BEFORE the final "OK"
 *    - URC          : "unsolicited result code", e.g. "+CEREG: 1". The modem
 *                     sends it at any time. It belongs to no command.
 *
 * AT replies carry no ID, so an "OK" does not say which command it answers.
 * The only safe rule is:
 *
 *      ONE COMMAND IN FLIGHT AT A TIME.
 *      Send a command, wait for its final result, then send the next one.
 *
 * This file follows that rule everywhere, including the boot sequence.
 *
 * STATE FLOW (happy path)
 * -----------------------
 *   INIT -> POWER_ON -> WAIT_BOOT -> CHECK_SIM -> ATTACH -> PDP
 *        -> OPEN_SOCKET -> MQTT_CONNECT -> MQTT_SUBSCRIBE -> RUN
 *
 *   WAIT_BOOT and CHECK_SIM each contain small SUB-STEPS (see section 9),
 *   so no new states were needed:
 *       WAIT_BOOT : AT        -> ATE0
 *       CHECK_SIM : AT+CPIN?  -> AT+CGDCONT (APN) -> AT+CEREG=2 -> attach
 *
 * FAILURE FLOW
 * ------------
 *   command ERROR / timeout -> retry counter +1
 *       counter < MAX_RETRIES  -> go to the matching RE... state, resend
 *       counter >= MAX_RETRIES -> LTE_SM_ERROR -> REBOOT_MODEM -> start again
 *
 * WHAT CHANGED COMPARED TO THE PREVIOUS VERSION
 * ---------------------------------------------
 *   1. Boot sequence sends ONE command at a time (a stray "OK" can no longer
 *      be mistaken for "attach succeeded").
 *   2. AT is re-sent until the modem answers (modems need seconds to boot).
 *   3. Recovery and ERROR states now also run on the periodic tick, not only
 *      when a line arrives from the modem (the modem may be silent).
 *   4. A timeout clears the active command and counts as a retry.
 *   5. Retries exhausted -> modem reboot, instead of looping forever.
 *   6. "+CME ERROR" / "+CMS ERROR" are treated as errors.
 *   7. +CEREG parser handles both the URC form and the query form.
 *   8. APN is configured (AT+CGDCONT) before attach.
 *   9. Handlers share two helpers (command_succeeded / command_failed)
 *      instead of repeating the same code five times.
 *  10. Test flags can really be honoured (see LTE_SM_HONOR_TEST_FLAGS).
 *  11. LAYERING: this file no longer includes or calls the hardware layer
 *      (lte_hw). Power, reset and time all go through lte_driver.
 *
 * @note Public API and state names are unchanged, so lte_driver, lte_tcp,
 *       lte_mqtt, the application and the test harness keep working.
 ******************************************************************************/

#include "lte_sm.h"
#include "lte_driver.h"
#include "lte_tcp.h"
#include "lte_mqtt.h"
#include "lte_types.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
 * 0. BUILD OPTIONS
 * ============================================================================
 */

/* APN of your SIM card. Without it many networks refuse the data session.
 * Override from the build system: -DLTE_APN=\"my.apn\" */
#ifndef LTE_APN
#define LTE_APN "internet"
#endif

/*
 * LTE_SM_HONOR_TEST_FLAGS
 *
 *   0 = behave exactly like the previous version: the three flags
 *       (sm_forward_enabled, sm_boot_enabled, sm_attach_enabled) are
 *       stored but NOT checked. Your existing tests keep their behaviour.
 *   1 = the flags really disable the forward path, the boot sequence and the
 *       attach handler, as the old comments promised.
 *
 * Why it defaults to 0: the old code never checked the flags, so the tests
 * may silently depend on the forward path running (for example to set the
 * "current command" before an injected line). Switch to 1 only when you are
 * ready to adjust the tests.
 */
#ifndef LTE_SM_HONOR_TEST_FLAGS
#define LTE_SM_HONOR_TEST_FLAGS 0
#endif

/* All logging goes through one macro so it can be silenced in one place. */
#define SM_LOG(...) printf(__VA_ARGS__)

/* ============================================================================
 * 1. STATE MACHINE VARIABLES
 * ============================================================================
 *
 * s_state      current state
 * s_prev_state state we came from (needed to choose the recovery state)
 * s_state_ts   time when the state was entered, or the active command was
 *              sent (used by the command timeout watchdog)
 */
static LteSmState_t s_state      = LTE_SM_INIT;
static LteSmState_t s_prev_state = LTE_SM_INVALID;
static uint32_t     s_state_ts   = 0;

/* Test-mode flags. Public, because the test harness toggles them.
 * They are only checked when LTE_SM_HONOR_TEST_FLAGS == 1. */
bool sm_forward_enabled = true;
bool sm_boot_enabled    = true;
bool sm_attach_enabled  = true;

/* ============================================================================
 * 2. COMMAND TRACKING
 * ============================================================================
 *
 * s_current_cmd is the ONE command that is currently in flight.
 *   - set when the command is sent
 *   - cleared when its final result arrives (OK / ERROR) or it times out
 *   - while it is set, no other command is sent (send_cmd_once)
 *   - while it is set, incoming lines are routed to that command's handler
 *
 * Boot commands (AT, ATE0, AT+CPIN?, ...) are tracked separately by the boot
 * sub-step variables in section 9.
 */
static LteCommand_t s_current_cmd = LTE_CMD_NONE;

/* ============================================================================
 * 3. RETRY COUNTERS
 * ============================================================================
 *
 * One counter per command. A failure (ERROR or timeout) increments it, a
 * success resets it. At MAX_RETRIES the state machine gives up on the simple
 * retry and goes to LTE_SM_ERROR, which escalates to a modem reboot.
 */
static int retry_attach    = 0;
static int retry_pdp       = 0;
static int retry_tcp       = 0;
static int retry_mqtt_conn = 0;
static int retry_mqtt_sub  = 0;

#define MAX_RETRIES 3

/* ============================================================================
 * 4. TIMEOUTS (milliseconds)
 * ============================================================================
 *
 * If the modem does not finish a command within its timeout, the command is
 * counted as failed. Attach is long because on a weak network the modem may
 * search for a cell for tens of seconds.
 */
#define TIMEOUT_ATTACH       30000
#define TIMEOUT_PDP          10000
#define TIMEOUT_TCP_OPEN      8000
#define TIMEOUT_MQTT_CONNECT  8000
#define TIMEOUT_MQTT_SUB      8000

/* Boot sequence: how long to wait for each reply before sending it again,
 * and how many sends are allowed before the modem is rebooted. */
#define BOOT_STEP_TIMEOUT_MS  2000
#define BOOT_MAX_SENDS        15

/* ============================================================================
 * 5. SMALL HELPERS
 * ============================================================================
 */

/* State name for logging. */
static const char *stateToString(LteSmState_t state)
{
    switch (state) {
    case LTE_SM_IDLE:             return "LTE_SM_IDLE";
    case LTE_SM_INIT:             return "LTE_SM_INIT";
    case LTE_SM_POWER_ON:         return "LTE_SM_POWER_ON";
    case LTE_SM_WAIT_BOOT:        return "LTE_SM_WAIT_BOOT";
    case LTE_SM_CHECK_SIM:        return "LTE_SM_CHECK_SIM";
    case LTE_SM_ATTACH:           return "LTE_SM_ATTACH";
    case LTE_SM_PDP:              return "LTE_SM_PDP";
    case LTE_SM_OPEN_SOCKET:      return "LTE_SM_OPEN_SOCKET";
    case LTE_SM_MQTT_CONNECT:     return "LTE_SM_MQTT_CONNECT";
    case LTE_SM_MQTT_SUBSCRIBE:   return "LTE_SM_MQTT_SUBSCRIBE";
    case LTE_SM_RUN:              return "LTE_SM_RUN";
    case LTE_SM_REBOOT_MODEM:     return "LTE_SM_REBOOT_MODEM";
    case LTE_SM_REATTACH:         return "LTE_SM_REATTACH";
    case LTE_SM_REPDP:            return "LTE_SM_REPDP";
    case LTE_SM_REOPEN_SOCKET:    return "LTE_SM_REOPEN_SOCKET";
    case LTE_SM_MQTT_RECONNECT:   return "LTE_SM_MQTT_RECONNECT";
    case LTE_SM_MQTT_RESUBSCRIBE: return "LTE_SM_MQTT_RESUBSCRIBE";
    case LTE_SM_ERROR:            return "LTE_SM_ERROR";
    case LTE_SM_INVALID:          return "LTE_SM_INVALID";
    default:                      return "UNKNOWN_STATE";
    }
}

/* Timeout for the active command (0 = no timeout known). */
static uint32_t get_cmd_timeout(LteCommand_t cmd)
{
    switch (cmd) {
    case LTE_CMD_ATTACH:         return TIMEOUT_ATTACH;
    case LTE_CMD_PDP:            return TIMEOUT_PDP;
    case LTE_CMD_TCP_OPEN:       return TIMEOUT_TCP_OPEN;
    case LTE_CMD_MQTT_CONNECT:   return TIMEOUT_MQTT_CONNECT;
    case LTE_CMD_MQTT_SUBSCRIBE: return TIMEOUT_MQTT_SUB;
    default:                     return 0;
    }
}

/* True for the states that exist only to repair a failure.
 * Explicit list instead of "state >= A && state <= B", which silently breaks
 * if someone reorders the enum in lte_types.h. */
static bool is_recovery_state(LteSmState_t st)
{
    switch (st) {
    case LTE_SM_REBOOT_MODEM:
    case LTE_SM_REATTACH:
    case LTE_SM_REPDP:
    case LTE_SM_REOPEN_SOCKET:
    case LTE_SM_MQTT_RECONNECT:
    case LTE_SM_MQTT_RESUBSCRIBE:
        return true;
    default:
        return false;
    }
}

/* True when the network connection is expected to exist (we were already
 * past attach). Used to decide whether a "+CEREG: 0" really means "lost". */
static bool network_expected(void)
{
    switch (s_state) {
    case LTE_SM_PDP:
    case LTE_SM_OPEN_SOCKET:
    case LTE_SM_MQTT_CONNECT:
    case LTE_SM_MQTT_SUBSCRIBE:
    case LTE_SM_RUN:
    case LTE_SM_REPDP:
    case LTE_SM_REOPEN_SOCKET:
    case LTE_SM_MQTT_RECONNECT:
    case LTE_SM_MQTT_RESUBSCRIBE:
        return true;
    default:
        return false;
    }
}

/* Final-result line recognisers.
 * Modems often answer "+CME ERROR: 30" instead of a bare "ERROR". */
static bool is_ok_line(const char *line)
{
    return strcmp(line, "OK") == 0;
}

static bool is_error_line(const char *line)
{
    return strcmp(line, "ERROR") == 0
        || strncmp(line, "+CME ERROR", 10) == 0
        || strncmp(line, "+CMS ERROR", 10) == 0;
}

/* Time since 'then'. Unsigned subtraction stays correct when the 32-bit
 * millisecond counter wraps (every ~49 days). */
static uint32_t elapsed_ms(uint32_t now, uint32_t then)
{
    return (uint32_t)(now - then);
}

/* ============================================================================
 * 6. RETRY COUNTERS: increment / reset / check
 * ============================================================================
 */

/* Called when a command fails. Returns the new counter value. */
static int increment_retry(LteCommand_t cmd)
{
    switch (cmd) {
    case LTE_CMD_ATTACH:         return ++retry_attach;
    case LTE_CMD_PDP:            return ++retry_pdp;
    case LTE_CMD_TCP_OPEN:       return ++retry_tcp;
    case LTE_CMD_MQTT_CONNECT:   return ++retry_mqtt_conn;
    case LTE_CMD_MQTT_SUBSCRIBE: return ++retry_mqtt_sub;
    default:
        SM_LOG("unexpected command: %d\n", (int)cmd);
        return 0;
    }
}

/* Called when a command succeeds. */
static void reset_retry(LteCommand_t cmd)
{
    switch (cmd) {
    case LTE_CMD_ATTACH:         retry_attach    = 0; break;
    case LTE_CMD_PDP:            retry_pdp       = 0; break;
    case LTE_CMD_TCP_OPEN:       retry_tcp       = 0; break;
    case LTE_CMD_MQTT_CONNECT:   retry_mqtt_conn = 0; break;
    case LTE_CMD_MQTT_SUBSCRIBE: retry_mqtt_sub  = 0; break;
    default:
        SM_LOG("unexpected command: %d\n", (int)cmd);
        break;
    }
}

static void clear_all_retries(void)
{
    retry_attach = retry_pdp = retry_tcp = retry_mqtt_conn = retry_mqtt_sub = 0;
}

/* True when ANY command used up all its retries. Used to decide between
 * "retry the same step" and "reboot the modem". */
static bool retries_exhausted(void)
{
    return retry_attach    >= MAX_RETRIES
        || retry_pdp       >= MAX_RETRIES
        || retry_tcp       >= MAX_RETRIES
        || retry_mqtt_conn >= MAX_RETRIES
        || retry_mqtt_sub  >= MAX_RETRIES;
}

/* ============================================================================
 * 7. STATE TRANSITION HELPER
 * ============================================================================
 *
 * lte_sm_set_state() is the ONLY place that changes s_state.
 * It saves the previous state, stores the new one, timestamps the change and
 * logs it.
 *
 * Two safety rules for LTE_SM_ERROR:
 *   - entering ERROR abandons the active command, so s_current_cmd is cleared.
 *     Otherwise the next line from the modem would still be routed to the
 *     old command's handler instead of the recovery logic.
 *   - setting ERROR while already in ERROR is ignored. Otherwise s_prev_state
 *     would become ERROR and the recovery mapping would lose track of where
 *     the failure really happened.
 */
void lte_sm_set_state(LteSmState_t st)
{
    if (st == LTE_SM_ERROR && s_state == LTE_SM_ERROR) {
        return;
    }

    s_prev_state = s_state;
    s_state      = st;
    s_state_ts   = lte_driver_now_ms();

    if (st == LTE_SM_ERROR) {
        s_current_cmd = LTE_CMD_NONE;
    }

    SM_LOG("STATE -> %s (prev=%s)\n",
           stateToString(s_state), stateToString(s_prev_state));
}

/* ============================================================================
 * 8. SENDING A COMMAND: send_cmd_once()
 * ============================================================================
 *
 * Sends an AT command only if no other command is in flight, and marks it as
 * the active command. Returns true if the command was really sent.
 *
 * This is the "one command at a time" rule in code form.
 */
static bool send_cmd_once(LteCommand_t cmd, const char *at)
{
    if (s_current_cmd != LTE_CMD_NONE) {
        return false;                    /* modem busy with another command */
    }
    s_current_cmd = cmd;
    lte_driver_send_cmd(at);
    s_state_ts = lte_driver_now_ms();    /* start of this command's timeout */
    return true;
}

/* ============================================================================
 * 9. BOOT SEQUENCE (sub-steps inside WAIT_BOOT and CHECK_SIM)
 * ============================================================================
 *
 * Every boot command waits for its own reply before the next one is sent.
 * Replies are only accepted from RX lines (never from a stale "last line"),
 * so an old "OK" cannot advance the sequence by mistake.
 *
 *   step        command            moves on when            state
 *   ---------   ----------------   ----------------------   -----------
 *   AT          AT                 "OK"                     WAIT_BOOT
 *   ATE0        ATE0               "OK"                     WAIT_BOOT
 *   CPIN        AT+CPIN?           "+CPIN: READY" then "OK" CHECK_SIM
 *   APN         AT+CGDCONT=...     "OK" (or ERROR, goes on) CHECK_SIM
 *   CEREG       AT+CEREG=2         "OK" (or ERROR, goes on) CHECK_SIM
 *   DONE        AT+CGATT=1 (attach is started)              ATTACH
 *
 * If no usable reply comes within BOOT_STEP_TIMEOUT_MS, the same command is
 * sent again. After BOOT_MAX_SENDS sends the modem is considered stuck and
 * LTE_SM_ERROR starts a reboot.
 *
 * Only CEREG is configured (not CREG/CGREG): this is an LTE design and CEREG
 * is the LTE (EPS) registration report. Add the others back only if the
 * modem must also fall back to 2G/3G.
 */
typedef enum {
    BOOT_STEP_AT = 0,
    BOOT_STEP_ATE0,
    BOOT_STEP_CPIN,
    BOOT_STEP_APN,
    BOOT_STEP_CEREG,
    BOOT_STEP_DONE
} BootStep_t;

static BootStep_t s_boot_step       = BOOT_STEP_AT;
static uint32_t   s_boot_sent_ts    = 0;      /* when the step was last sent   */
static uint8_t    s_boot_sends      = 0;      /* sends of the current step     */
static bool       s_boot_cpin_ready = false;  /* "+CPIN: READY" seen this try  */

static void boot_reset(void)
{
    s_boot_step       = BOOT_STEP_AT;
    s_boot_sent_ts    = 0;
    s_boot_sends      = 0;
    s_boot_cpin_ready = false;
}

/* Send the command that belongs to the current boot step. */
static void boot_send_step(void)
{
    static char apn_cmd[64];             /* static: driver may send later */
    const char *cmd = NULL;

    switch (s_boot_step) {
    case BOOT_STEP_AT:    cmd = "AT\r\n";            break;
    case BOOT_STEP_ATE0:  cmd = "ATE0\r\n";          break;  /* echo off */
    case BOOT_STEP_CPIN:  cmd = "AT+CPIN?\r\n";      break;  /* SIM status */
    case BOOT_STEP_APN:
        snprintf(apn_cmd, sizeof(apn_cmd),
                 "AT+CGDCONT=1,\"IP\",\"%s\"\r\n", LTE_APN);
        cmd = apn_cmd;
        break;
    case BOOT_STEP_CEREG: cmd = "AT+CEREG=2\r\n";    break;  /* reg. reports */
    default:              return;
    }

    s_boot_cpin_ready = false;
    s_boot_sends++;
    s_boot_sent_ts = lte_driver_now_ms();
    lte_driver_send_cmd(cmd);
}

/* Move to the next boot step, update the state, and send its command.
 * After the last step the network attach is started. */
static void boot_advance(BootStep_t next)
{
    s_boot_step  = next;
    s_boot_sends = 0;

    switch (next) {
    case BOOT_STEP_ATE0:
        boot_send_step();
        break;

    case BOOT_STEP_CPIN:
        SM_LOG("Modem boot complete, checking SIM...\n");
        boot_send_step();
        lte_sm_set_state(LTE_SM_CHECK_SIM);
        break;

    case BOOT_STEP_APN:
    case BOOT_STEP_CEREG:
        boot_send_step();
        break;

    case BOOT_STEP_DONE:
        SM_LOG("Registration reports enabled, attaching to network...\n");
        send_cmd_once(LTE_CMD_ATTACH, "AT+CGATT=1\r\n");
        lte_sm_set_state(LTE_SM_ATTACH);
        break;

    default:
        break;
    }
}

/* The step that follows the current one. */
static BootStep_t boot_next_step(void)
{
    switch (s_boot_step) {
    case BOOT_STEP_AT:    return BOOT_STEP_ATE0;
    case BOOT_STEP_ATE0:  return BOOT_STEP_CPIN;
    case BOOT_STEP_CPIN:  return BOOT_STEP_APN;
    case BOOT_STEP_APN:   return BOOT_STEP_CEREG;
    default:              return BOOT_STEP_DONE;
    }
}

/* A line arrived from the modem while booting. */
static void boot_on_line(const char *line)
{
    if (s_boot_step == BOOT_STEP_DONE) {
        return;
    }

    /* SIM check has an information line before the final OK. */
    if (s_boot_step == BOOT_STEP_CPIN) {
        if (strncmp(line, "+CPIN:", 6) == 0) {
            s_boot_cpin_ready = (strstr(line, "READY") != NULL);
            if (!s_boot_cpin_ready) {
                SM_LOG("SIM not ready: %s\n", line);
            }
            return;
        }
        if (is_ok_line(line)) {
            if (s_boot_cpin_ready) {
                SM_LOG("SIM is ready\n");
                boot_advance(boot_next_step());
            }
            /* OK without READY: wait, the timeout will ask again. */
        }
        return;   /* errors and other lines: the timeout will ask again */
    }

    /* All other steps: OK moves on. */
    if (is_ok_line(line)) {
        boot_advance(boot_next_step());
        return;
    }

    if (is_error_line(line)) {
        if (s_boot_step == BOOT_STEP_APN || s_boot_step == BOOT_STEP_CEREG) {
            /* Not critical: log it and carry on with the attach. */
            SM_LOG("Boot step %d returned an error, continuing\n",
                   (int)s_boot_step);
            boot_advance(boot_next_step());
        }
        /* AT / ATE0 errors: the timeout will resend. */
    }

    /* Anything else (echo of our own command, URCs) is ignored. */
}

/* Called on every tick while booting: resend on timeout, or give up. */
static void boot_tick(void)
{
    if (s_boot_step == BOOT_STEP_DONE) {
        return;
    }
    if (elapsed_ms(lte_driver_now_ms(), s_boot_sent_ts) < BOOT_STEP_TIMEOUT_MS) {
        return;
    }
    if (s_boot_sends >= BOOT_MAX_SENDS) {
        SM_LOG("Boot step %d: no usable reply after %u tries, rebooting modem\n",
               (int)s_boot_step, (unsigned)s_boot_sends);
        lte_sm_set_state(LTE_SM_ERROR);   /* prev = boot state -> REBOOT */
        return;
    }
    boot_send_step();                     /* ask again */
}

/* ============================================================================
 * 10. COMMAND RESULT HELPERS
 * ============================================================================
 *
 * Every tracked command ends in one of two ways. These helpers hold the
 * common code so each handler is only a few lines.
 */

/* The command finished with OK: forget retries, free the modem, move on. */
static void command_succeeded(LteCommand_t cmd, LteSmState_t next_state)
{
    reset_retry(cmd);
    s_current_cmd = LTE_CMD_NONE;
    lte_sm_set_state(next_state);
}

/* The command failed (ERROR): count it. Below the limit go to the retry
 * state; at the limit go to LTE_SM_ERROR (which escalates to a reboot). */
static void command_failed(LteCommand_t cmd, LteSmState_t retry_state)
{
    s_current_cmd = LTE_CMD_NONE;
    if (increment_retry(cmd) >= MAX_RETRIES) {
        lte_sm_set_state(LTE_SM_ERROR);
    } else {
        lte_sm_set_state(retry_state);
    }
}

/* Handler shape shared by PDP, TCP, MQTT connect and MQTT subscribe:
 * "OK" -> next state, error -> retry state.
 *
 * NOTE for TCP and MQTT: on many modems "OK" only means "request accepted"
 * and the real result comes later as a URC (for example "+QIOPEN: 0,0").
 * This works as long as lte_tcp / lte_mqtt turn that final result into an
 * "OK" or "ERROR" line for the state machine. If they do not, extend those
 * layers; do not treat the early "OK" as success. */
static void handle_simple_command(LteCommand_t cmd, const char *line,
                                  LteSmState_t ok_state,
                                  LteSmState_t retry_state)
{
    if (is_ok_line(line)) {
        command_succeeded(cmd, ok_state);
    } else if (is_error_line(line)) {
        command_failed(cmd, retry_state);
    }
    /* any other line (info lines, echo) is ignored */
}

/* ============================================================================
 * 11. COMMAND HANDLERS
 * ============================================================================
 *
 * Called by the event handler with every line received while the matching
 * command is in flight.
 */

/* ATTACH - AT+CGATT=1
 *   OK       -> attach done, go to PDP
 *   ERROR    -> retry (REATTACH) or give up (ERROR)
 *   "+CGATT: 0" is the answer to a QUERY (AT+CGATT?), not to CGATT=1. It is
 *   kept for compatibility with the tests and treated as a failed attach.
 *   "+CGATT: 1" changes nothing; the final OK decides. */
static void lte_cmd_attach_handle(const char *line)
{
#if LTE_SM_HONOR_TEST_FLAGS
    if (!sm_attach_enabled) {
        return;               /* test mode: attach handler switched off */
    }
#endif

    if (strncmp(line, "+CGATT:", 7) == 0) {
        if (strcmp(line, "+CGATT: 0") == 0) {
            command_failed(LTE_CMD_ATTACH, LTE_SM_REATTACH);
        }
        return;
    }

    handle_simple_command(LTE_CMD_ATTACH, line, LTE_SM_PDP, LTE_SM_REATTACH);
}

/* PDP - AT+CGACT=1,1       OK -> OPEN_SOCKET, error -> REPDP */
static void lte_cmd_pdp_handle(const char *line)
{
    handle_simple_command(LTE_CMD_PDP, line,
                          LTE_SM_OPEN_SOCKET, LTE_SM_REPDP);
}

/* TCP OPEN                  OK -> MQTT_CONNECT, error -> REOPEN_SOCKET */
static void lte_cmd_tcp_handle(const char *line)
{
    handle_simple_command(LTE_CMD_TCP_OPEN, line,
                          LTE_SM_MQTT_CONNECT, LTE_SM_REOPEN_SOCKET);
}

/* MQTT CONNECT              OK -> MQTT_SUBSCRIBE, error -> MQTT_RECONNECT */
static void lte_cmd_mqtt_conn_handle(const char *line)
{
    handle_simple_command(LTE_CMD_MQTT_CONNECT, line,
                          LTE_SM_MQTT_SUBSCRIBE, LTE_SM_MQTT_RECONNECT);
}

/* MQTT SUBSCRIBE            OK -> RUN, error -> MQTT_RESUBSCRIBE */
static void lte_cmd_mqtt_sub_handle(const char *line)
{
    handle_simple_command(LTE_CMD_MQTT_SUBSCRIBE, line,
                          LTE_SM_RUN, LTE_SM_MQTT_RESUBSCRIBE);
}

/* ============================================================================
 * 12. STATE MACHINE INITIALISATION
 * ============================================================================
 *
 * Puts everything back to a clean start. Called at boot and by the test
 * harness before every scenario.
 */
void lte_sm_init(void)
{
    s_state       = LTE_SM_INIT;
    s_prev_state  = LTE_SM_INVALID;
    s_state_ts    = lte_driver_now_ms();
    s_current_cmd = LTE_CMD_NONE;

    clear_all_retries();
    boot_reset();
}

/* ============================================================================
 * 13. TIMEOUT WATCHDOG
 * ============================================================================
 *
 * Called on every event. If the active command has been in flight longer
 * than its timeout:
 *   1. count it as a failure (retry counter +1),
 *   2. enter LTE_SM_ERROR (this also clears the active command),
 *   3. the recovery mapping then picks retry or reboot.
 *
 * Known limitation: after a timeout the modem might still answer the old
 * command later. That late "OK" could be credited to the retry. If you see
 * this on real hardware, wait about a second (or send a plain "AT" and wait
 * for its OK) before resending.
 */
static void check_timeout(void)
{
    if (s_current_cmd == LTE_CMD_NONE) {
        return;
    }

    uint32_t timeout = get_cmd_timeout(s_current_cmd);
    if (timeout == 0) {
        return;
    }

    if (elapsed_ms(lte_driver_now_ms(), s_state_ts) > timeout) {
        LteCommand_t timed_out = s_current_cmd;
        SM_LOG("TIMEOUT on command %d\n", (int)timed_out);
        increment_retry(timed_out);
        lte_sm_set_state(LTE_SM_ERROR);   /* also clears s_current_cmd */
    }
}

/* ============================================================================
 * 14. ERROR -> RECOVERY MAPPING
 * ============================================================================
 *
 * In LTE_SM_ERROR we choose what to do from the state where the failure
 * happened (s_prev_state):
 *
 *      ATTACH         -> REATTACH
 *      PDP            -> REPDP
 *      OPEN_SOCKET    -> REOPEN_SOCKET
 *      MQTT_CONNECT   -> MQTT_RECONNECT
 *      MQTT_SUBSCRIBE -> MQTT_RESUBSCRIBE
 *      anything else  -> REBOOT_MODEM (boot problems, failure in RUN, ...)
 *
 * ESCALATION: if any command already used all its retries, simple retrying
 * has failed, so the whole modem is rebooted. Without this the machine would
 * loop between ERROR and the same retry state forever.
 */
static void map_error_to_recovery(void)
{
    SM_LOG("ERROR (prev=%s)\n", stateToString(s_prev_state));

    if (retries_exhausted()) {
        SM_LOG("Retries exhausted, rebooting modem\n");
        lte_sm_set_state(LTE_SM_REBOOT_MODEM);
        return;
    }

    switch (s_prev_state) {
    case LTE_SM_ATTACH:         lte_sm_set_state(LTE_SM_REATTACH);         break;
    case LTE_SM_PDP:            lte_sm_set_state(LTE_SM_REPDP);            break;
    case LTE_SM_OPEN_SOCKET:    lte_sm_set_state(LTE_SM_REOPEN_SOCKET);    break;
    case LTE_SM_MQTT_CONNECT:   lte_sm_set_state(LTE_SM_MQTT_RECONNECT);   break;
    case LTE_SM_MQTT_SUBSCRIBE: lte_sm_set_state(LTE_SM_MQTT_RESUBSCRIBE); break;
    default:
        SM_LOG("No specific recovery for %s, rebooting modem\n",
               stateToString(s_prev_state));
        lte_sm_set_state(LTE_SM_REBOOT_MODEM);
        break;
    }
}

/* ============================================================================
 * 15. +CEREG REGISTRATION REPORTS
 * ============================================================================
 *
 * +CEREG reports the LTE registration status. <stat> values:
 *     0 not registered, not searching     1 registered, home
 *     2 searching                         3 registration denied
 *     4 unknown (e.g. out of coverage)    5 registered, roaming
 *
 * Two line formats exist and the old parser handled only the first:
 *   query form : +CEREG: <n>,<stat>[,<tac>,<ci>,<act>]     e.g. "+CEREG: 2,1"
 *   URC form   : +CEREG: <stat>[,"<tac>","<ci>",<act>]     e.g. "+CEREG: 1"
 *                                                          or   +CEREG: 1,"00A1","0123ABCD",7
 * They are told apart by the second field: a plain number means the first
 * field was <n> (query form); a quoted string or nothing means the first
 * field is already <stat> (URC form).
 */
static bool parse_cereg_stat(const char *line, int *stat)
{
    const char *p = line + 7;            /* skip "+CEREG:" */
    char *end = NULL;

    long first = strtol(p, &end, 10);
    if (end == p) {
        return false;                    /* no number found */
    }

    if (*end == ',') {
        const char *q = end + 1;
        while (*q == ' ') {
            q++;
        }
        if (*q >= '0' && *q <= '9') {    /* query form: second field is <stat> */
            *stat = (int)strtol(q, &end, 10);
            return true;
        }
    }

    *stat = (int)first;                  /* URC form */
    return true;
}

static void lte_handle_cereg_urc(const char *line)
{
    int stat = -1;

    if (!parse_cereg_stat(line, &stat)) {
        SM_LOG("Could not parse: %s\n", line);
        return;
    }

    SM_LOG("+CEREG status=%d\n", stat);

    switch (stat) {

    case 1:   /* registered, home    */
    case 5:   /* registered, roaming */
        SM_LOG("URC: network registered\n");
        break;

    case 2:   /* searching */
    case 4:   /* unknown   */
        SM_LOG("URC: searching or unknown, no action\n");
        break;

    case 0:   /* not registered */
        /* Before attach has finished, "0" is normal and the attach command
         * timeout covers real failures. Only react if we had a network. */
        if (network_expected()) {
            SM_LOG("URC: network lost, reattaching\n");
            s_current_cmd = LTE_CMD_NONE;           /* drop active command */
            lte_sm_set_state(LTE_SM_REATTACH);
        } else {
            SM_LOG("URC: not registered yet\n");
        }
        break;

    case 3:   /* registration denied */
        if (s_state != LTE_SM_ERROR) {
            SM_LOG("URC: registration denied\n");
            s_current_cmd = LTE_CMD_NONE;
            increment_retry(LTE_CMD_ATTACH);        /* counts towards reboot */
            lte_sm_set_state(LTE_SM_ERROR);
        }
        break;

    default:
        SM_LOG("URC: unknown CEREG status %d, ignored\n", stat);
        break;
    }
}

/* ============================================================================
 * 16. RECOVERY STATE HANDLERS
 * ============================================================================
 *
 * Each recovery state resends its command and returns to the normal state,
 * where the matching handler waits for the result.
 *
 * Now also run from the periodic tick (see handle_line_forward), because
 * after a failure the modem may be silent and no line would ever arrive.
 */
static void handle_recovery(void)
{
    switch (s_state) {

    /* Full modem reset, then start the boot sequence again from "AT". */
    case LTE_SM_REBOOT_MODEM:
        SM_LOG("Resetting modem...\n");
        lte_driver_modem_reset();
        clear_all_retries();
        s_current_cmd = LTE_CMD_NONE;
        boot_reset();
        boot_send_step();                 /* "AT"; boot_tick resends it */
        lte_sm_set_state(LTE_SM_WAIT_BOOT);
        break;

    case LTE_SM_REATTACH:
        send_cmd_once(LTE_CMD_ATTACH, "AT+CGATT=1\r\n");
        lte_sm_set_state(LTE_SM_ATTACH);
        break;

    case LTE_SM_REPDP:
        send_cmd_once(LTE_CMD_PDP, "AT+CGACT=1,1\r\n");
        lte_sm_set_state(LTE_SM_PDP);
        break;

    case LTE_SM_REOPEN_SOCKET:
        s_current_cmd = LTE_CMD_TCP_OPEN;
        lte_tcp_open();
        lte_sm_set_state(LTE_SM_OPEN_SOCKET);
        break;

    case LTE_SM_MQTT_RECONNECT:
        s_current_cmd = LTE_CMD_MQTT_CONNECT;
        lte_mqtt_connect("broker.hivemq.com", 1883);
        lte_sm_set_state(LTE_SM_MQTT_CONNECT);
        break;

    case LTE_SM_MQTT_RESUBSCRIBE:
        s_current_cmd = LTE_CMD_MQTT_SUBSCRIBE;
        lte_mqtt_subscribe("device/commands");
        lte_sm_set_state(LTE_SM_MQTT_SUBSCRIBE);
        break;

    default:
        lte_sm_set_state(LTE_SM_ERROR);
        break;
    }
}

/* ============================================================================
 * 17. FORWARD PATH (decides what to send next)
 * ============================================================================
 *
 * Called in two ways:
 *   - response == NULL : periodic tick from the main loop
 *   - response != NULL : a line arrived and no tracked command wanted it
 *
 * What it does depends on the state:
 *   boot states      : run the boot sub-steps (lines advance, ticks resend)
 *   PDP/SOCKET/MQTT  : send the command if none is in flight
 *   ERROR / RE...    : run the recovery logic (so a silent modem still
 *                      recovers)
 *   ATTACH / RUN     : nothing to send; replies are handled elsewhere
 */
static bool boot_allowed(void)
{
#if LTE_SM_HONOR_TEST_FLAGS
    return sm_boot_enabled;
#else
    return true;
#endif
}

static void handle_line_forward(const char *response)
{
    switch (s_state) {

    /* ---- Boot --------------------------------------------------------- */

    case LTE_SM_INIT:
        if (!boot_allowed()) break;
        SM_LOG("Powering on modem...\n");
        lte_driver_power_on();
        lte_sm_set_state(LTE_SM_POWER_ON);
        break;

    case LTE_SM_POWER_ON:
        if (!boot_allowed()) break;
        boot_reset();
        boot_send_step();                 /* first "AT" */
        lte_sm_set_state(LTE_SM_WAIT_BOOT);
        SM_LOG("Waiting for modem boot...\n");
        break;

    case LTE_SM_WAIT_BOOT:
    case LTE_SM_CHECK_SIM:
        if (!boot_allowed()) break;
        if (response != NULL) {
            boot_on_line(response);       /* a reply arrived */
        } else {
            boot_tick();                  /* time passed: resend or give up */
        }
        break;

    /* ---- Normal bring-up ---------------------------------------------- */

    case LTE_SM_ATTACH:
        /* The attach command is already in flight; the handler decides. */
        break;

    case LTE_SM_PDP:
        if (send_cmd_once(LTE_CMD_PDP, "AT+CGACT=1,1\r\n")) {
            SM_LOG("Activating PDP context...\n");
        }
        break;

    case LTE_SM_OPEN_SOCKET:
        if (s_current_cmd == LTE_CMD_NONE) {
            SM_LOG("Opening TCP socket...\n");
            s_current_cmd = LTE_CMD_TCP_OPEN;
            lte_tcp_open();
            s_state_ts = lte_driver_now_ms();
        }
        break;

    case LTE_SM_MQTT_CONNECT:
        if (s_current_cmd == LTE_CMD_NONE) {
            SM_LOG("Connecting to MQTT broker...\n");
            s_current_cmd = LTE_CMD_MQTT_CONNECT;
            lte_mqtt_connect("broker.hivemq.com", 1883);
            s_state_ts = lte_driver_now_ms();
        }
        break;

    case LTE_SM_MQTT_SUBSCRIBE:
        if (s_current_cmd == LTE_CMD_NONE) {
            SM_LOG("Subscribing to MQTT topic...\n");
            s_current_cmd = LTE_CMD_MQTT_SUBSCRIBE;
            lte_mqtt_subscribe("device/commands");
            s_state_ts = lte_driver_now_ms();
        }
        break;

    /* ---- Running ------------------------------------------------------ */

    case LTE_SM_RUN:
        /* Normal operation. Application traffic is sent by the application.
         * Incoming MQTT messages and socket-closed URCs would be handled
         * here in a future version. */
        break;

    case LTE_SM_IDLE:
        break;

    /* ---- Failure handling (now also runs on ticks) -------------------- */

    case LTE_SM_ERROR:
        map_error_to_recovery();
        break;

    case LTE_SM_REBOOT_MODEM:
    case LTE_SM_REATTACH:
    case LTE_SM_REPDP:
    case LTE_SM_REOPEN_SOCKET:
    case LTE_SM_MQTT_RECONNECT:
    case LTE_SM_MQTT_RESUBSCRIBE:
        handle_recovery();
        break;

    default:
        SM_LOG("Unexpected state %d\n", (int)s_state);
        lte_sm_set_state(LTE_SM_ERROR);
        break;
    }
}

/* ============================================================================
 * 18. TEST SUPPORT
 * ============================================================================
 */

/* Put the state machine directly into a state (tests only).
 * Deliberately does NOT touch s_prev_state, the timestamp or the active
 * command, and prints nothing. */
void lte_sm_force_state(LteSmState_t st)
{
    s_state = st;
}

void lte_sm_disable_forward_path(void) { sm_forward_enabled = false; }
void lte_sm_disable_boot_sequence(void){ sm_boot_enabled    = false; }
void lte_sm_disable_attach_handler(void){ sm_attach_enabled = false; }
void lte_sm_enable_forward_path(void)  { sm_forward_enabled = true;  }
void lte_sm_enable_boot_sequence(void) { sm_boot_enabled    = true;  }
void lte_sm_enable_attach_handler(void){ sm_attach_enabled  = true;  }

/* ============================================================================
 * 19. PUBLIC API
 * ============================================================================
 */

/* Current state. Read-only, safe to call at any time. */
LteSmState_t lte_sm_get_state(void)
{
    return s_state;
}

/* Periodic tick (production): decides what to send next. */
void lte_sm_forward_path(void)
{
#if LTE_SM_HONOR_TEST_FLAGS
    if (!sm_forward_enabled) {
        return;
    }
#endif
    handle_line_forward(NULL);
}

/* ============================================================================
 * 20. EVENT HANDLER (single entry point)
 * ============================================================================
 *
 * Everything enters here:
 *   ev == NULL                    periodic tick
 *   LTE_EVENT_RX_LINE             one line from the modem
 *   LTE_EVENT_TIMEOUT / ERROR     external failure notification
 *
 * Order of work for an RX line:
 *   A. +CEREG report        -> URC handler (never belongs to a command)
 *   B. a command in flight  -> that command's handler gets the line
 *   C. state is ERROR       -> choose a recovery state
 *   D. state is RE...       -> run the recovery action
 *   E. otherwise            -> forward path (boot steps use the line)
 */
void lte_sm_handle_event(const LteEvent_t *ev)
{
    /* Always check the command timeout first. */
    check_timeout();

    /* ---- Tick ---------------------------------------------------------- */
    if (!ev) {
        lte_sm_forward_path();
        return;
    }

    /* ---- Line from the modem ------------------------------------------- */
    if (ev->type == LTE_EVENT_RX_LINE) {

        SM_LOG("RX: %s  [state=%s cmd=%d]\n",
               ev->line, stateToString(s_state), (int)s_current_cmd);

        /* A. Registration reports first: they may announce network loss. */
        if (strncmp(ev->line, "+CEREG:", 7) == 0) {
            lte_handle_cereg_urc(ev->line);
            return;
        }

        /* B. Route to the command that is in flight. */
        switch (s_current_cmd) {
        case LTE_CMD_ATTACH:         lte_cmd_attach_handle(ev->line);    return;
        case LTE_CMD_PDP:            lte_cmd_pdp_handle(ev->line);       return;
        case LTE_CMD_TCP_OPEN:       lte_cmd_tcp_handle(ev->line);       return;
        case LTE_CMD_MQTT_CONNECT:   lte_cmd_mqtt_conn_handle(ev->line); return;
        case LTE_CMD_MQTT_SUBSCRIBE: lte_cmd_mqtt_sub_handle(ev->line);  return;
        case LTE_CMD_NONE:
        default:
            break;                    /* no command in flight */
        }

        /* C. Failure state: choose where to recover. */
        if (s_state == LTE_SM_ERROR) {
            map_error_to_recovery();
            return;
        }

        /* D. Recovery state: resend the command. */
        if (is_recovery_state(s_state)) {
            handle_recovery();
            return;
        }

        /* E. Everything else (boot steps, idle, run). */
#if LTE_SM_HONOR_TEST_FLAGS
        if (!sm_forward_enabled) {
            return;
        }
#endif
        handle_line_forward(ev->line);
        return;
    }

    /* ---- External failure notification --------------------------------- */
    if (ev->type == LTE_EVENT_TIMEOUT || ev->type == LTE_EVENT_ERROR) {
        lte_sm_set_state(LTE_SM_ERROR);
        return;
    }
}

/******************************************************************************
 * END OF lte_sm.c
 ******************************************************************************/