#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "lte_sm.h"
#include "lte_driver.h"
#include "lte_hw.h"

/* --------------------------------------------------------------------------
 * LTE Test Harness
 *
 * This file contains unit tests for the LTE modem state machine and driver.
 * It simulates modem responses and checks that the state machine transitions
 * correctly through various scenarios, including successful attach, PDP context
 * activation, socket opening, and error handling.
 *
 * The tests are designed to be run in a controlled environment where the LTE
 * hardware layer can be mocked or simulated.
 * 
 * Note: The tests assume that the LTE driver and state machine are initialized
 * and that the hardware layer provides a way to simulate time and modem responses. 
 * -------------------------------------------------------------------------- */

/* Simple helper: get current state (add this in lte_sm.c if not present) */
extern LteSmState_t lte_sm_get_state(void);

/* -------------------------------
   Test event injection helpers
   ------------------------------- */

/* Inject a full modem line as if it came from UART */
static void inject_line(const char *line)
{
    size_t len = strlen(line);

    for (size_t i = 0; i < len; i++)
        lte_driver_on_rx_byte((uint8_t)line[i]);

    /* Modem lines are usually terminated with \r\n */
    lte_driver_on_rx_byte('\r');
    lte_driver_on_rx_byte('\n');
}

/* Run one iteration of the state machine:
 * - drain events from driver
 * - feed them to state machine
 * - run forward path (NULL event)
 */
static void step_sm(void)
{
    LteEvent_t ev;

    while (lte_driver_get_next_event(&ev))
        lte_sm_handle_event(&ev);

    /* Forward path (no event) */
    lte_sm_handle_event(NULL);
}

/* Advance "time" for timeout tests.
 * On the host the fake clock only moves when lte_hw_delay_ms() is called,
 * so each loop pass advances it by 1 ms and then lets the state machine
 * run, so it can notice that a timeout has expired. */
static void advance_time_ms(uint32_t ms)
{
    uint32_t start = lte_hw_ms();

    while ((lte_hw_ms() - start) < ms) {
        lte_hw_delay_ms(1);   /* move the fake clock forward 1 ms */
        step_sm();            /* let the state machine see the new time */
    }
}

/* Inject one line and let the state machine handle ONLY that event, with no
 * forward-path tick afterwards. step_sm() ticks too, and the tick makes the
 * machine recover at once (for example REATTACH -> ATTACH), so the state we
 * want to check would already be gone. */
static void deliver_line(const char *line)
{
    LteEvent_t ev;

    inject_line(line);
    while (lte_driver_get_next_event(&ev))
        lte_sm_handle_event(&ev);
}

/* Reset the state machine, then put it in ATTACH or PDP with its command
 * already sent, so an injected reply is routed to the right handler. */
static void sm_start_in_attach(void)
{
    lte_sm_init();
    lte_sm_force_state(LTE_SM_REATTACH);
    step_sm();                       /* sends AT+CGATT=1, state -> ATTACH */
}

static void sm_start_in_pdp(void)
{
    lte_sm_init();
    lte_sm_force_state(LTE_SM_REPDP);
    step_sm();                       /* sends AT+CGACT=1,1, state -> PDP */
}

static void sm_start_in_socket(void)
{
    lte_sm_init();
    lte_sm_force_state(LTE_SM_REOPEN_SOCKET);
    step_sm();                       /* opens TCP, state -> OPEN_SOCKET */
}

static void sm_start_in_mqtt_connect(void)
{
    lte_sm_init();
    lte_sm_force_state(LTE_SM_MQTT_RECONNECT);
    step_sm();                       /* sends MQTT connect, state -> MQTT_CONNECT */
}

static void sm_start_in_mqtt_subscribe(void)
{
    lte_sm_init();
    lte_sm_force_state(LTE_SM_MQTT_RESUBSCRIBE);
    step_sm();                       /* sends subscribe, state -> MQTT_SUBSCRIBE */
}

/* A modem reply followed by a tick: what the real main loop does. */
static void modem_reply(const char *line)
{
    deliver_line(line);
    step_sm();
}

/* Power on the machine and move it to WAIT_BOOT (AT sent). */
static void sm_start_boot(void)
{
    lte_sm_init();
    step_sm();                       /* INIT -> POWER_ON */
    step_sm();                       /* POWER_ON -> WAIT_BOOT */
}

typedef void (*start_fn_t)(void);

/* Compare the current state with the expected one and print the verdict. */
static int check_state(LteSmState_t expect, const char *what)
{
    LteSmState_t st = lte_sm_get_state();

    if (st == expect) {
        printf("PASS: %s (state = %d)\n", what, st);
        return 1;
    }
    printf("FAIL: %s: state = %d (expected %d)\n", what, st, expect);
    return 0;
}

/* -------------------------------
   Individual test cases
   ------------------------------- */

// Test: Successful attach and PDP activation   
static int test_attach_ok(void)
{
    printf("TEST: Attach OK -> PDP\n");
    sm_start_in_attach();

    /* Simulate modem responses for attach */
    inject_line("+CGATT:1");
    step_sm();

    inject_line("OK");
    step_sm();

    LteSmState_t st = lte_sm_get_state();
    // expected state is LTE_SM_PDP after successful attach 
    if (st == LTE_SM_PDP) {
        printf("PASS: state = LTE_SM_PDP\n");
        return 1;
    } else {
        printf("FAIL: state = %d (expected LTE_SM_PDP)\n", st);
        return 0;
    }
}

// Test: Attach error leads to REATTACH or ERROR state
static int test_attach_error_retry(void)
{
    printf("TEST: Attach ERROR -> REATTACH\n");
    sm_start_in_attach();

    deliver_line("ERROR");   /* modem rejects AT+CGATT=1 */
    
    // After an attach error, the state machine should transition to REATTACH or ERROR  
    LteSmState_t st = lte_sm_get_state();
    if (st == LTE_SM_REATTACH || st == LTE_SM_ERROR) {
        printf("PASS: state = %d (REATTACH/ERROR acceptable)\n", st);
        return 1;
    } else {
        printf("FAIL: state = %d (expected REATTACH/ERROR)\n", st);
        return 0;
    }
}

// Test: Successful PDP activation leads to OPEN_SOCKET state   
static int test_pdp_ok(void)
{
    printf("TEST: PDP OK -> OPEN_SOCKET\n");
    sm_start_in_pdp();

    inject_line("OK");  /* Assume we are in LTE_SM_PDP and modem replies OK */
    step_sm();

    LteSmState_t st = lte_sm_get_state();
    // expected state is LTE_SM_OPEN_SOCKET after successful PDP activation 
    if (st == LTE_SM_OPEN_SOCKET) {
        printf("PASS: state = LTE_SM_OPEN_SOCKET\n");
        return 1;
    } else {
        printf("FAIL: state = %d (expected LTE_SM_OPEN_SOCKET)\n", st);
        return 0;
    }
}

// Test: +CEREG:0 indicates network lost, should transition to ERROR or REATTACH    
static int test_cereg_network_lost(void)
{
    printf("TEST: +CEREG:0 -> ERROR/REATTACH\n");
    sm_start_in_pdp();       /* a state where the network is expected */

    deliver_line("+CEREG: 0,0");
    
    // After receiving +CEREG:0, the state machine should transition to ERROR or REATTACH   
    LteSmState_t st = lte_sm_get_state();
    if (st == LTE_SM_ERROR || st == LTE_SM_REATTACH) {
        printf("PASS: state = %d (ERROR/REATTACH)\n", st);
        return 1;
    } else {
        printf("FAIL: state = %d (expected ERROR/REATTACH)\n", st);
        return 0;
    }
}

// Test: Timeout during attach should lead to ERROR state   
static int test_timeout_attach(void)
{
    printf("TEST: Attach timeout -> ERROR\n");
    sm_start_in_attach();    /* AT+CGATT=1 sent, no reply will come */

    /* TIMEOUT_ATTACH is 30000 ms and the watchdog fires when MORE than that
     * has passed, so advance exactly 30001 ms. Any further and the machine
     * would already have recovered and sent the command again. */
    advance_time_ms(30001);

    /* On timeout the machine enters ERROR and, in the same tick, maps it to
     * the recovery state REATTACH. Either one proves the timeout fired. */
    LteSmState_t st = lte_sm_get_state();
    if (st == LTE_SM_ERROR || st == LTE_SM_REATTACH) {
        printf("PASS: state = %d (timeout -> ERROR/REATTACH)\n", st);
        return 1;
    } else {
        printf("FAIL: state = %d (expected ERROR/REATTACH)\n", st);
        return 0;
    }
}

// Test: complete bring-up from INIT to RUN, one modem reply at a time
static int test_full_path_to_run(void)
{
    printf("TEST: full path INIT -> RUN\n");

    /* Each row is one modem reply and the state expected right after it.
     * The command the machine has just sent is in the comment. */
    static const struct {
        const char  *reply;
        LteSmState_t expect;
    } steps[] = {
        { "OK",            LTE_SM_WAIT_BOOT      },  /* reply to AT       -> sends ATE0       */
        { "OK",            LTE_SM_CHECK_SIM      },  /* reply to ATE0     -> sends AT+CPIN?   */
        { "+CPIN: READY",  LTE_SM_CHECK_SIM      },  /* SIM info line (no state change)       */
        { "OK",            LTE_SM_CHECK_SIM      },  /* end of CPIN       -> sends APN        */
        { "OK",            LTE_SM_CHECK_SIM      },  /* reply to APN      -> sends CEREG=2    */
        { "OK",            LTE_SM_ATTACH         },  /* reply to CEREG    -> sends CGATT=1    */
        { "OK",            LTE_SM_PDP            },  /* attach done       -> sends CGACT      */
        { "OK",            LTE_SM_OPEN_SOCKET    },  /* PDP active        -> opens TCP        */
        { "OK",            LTE_SM_MQTT_CONNECT   },  /* TCP open          -> MQTT connect     */
        { "OK",            LTE_SM_MQTT_SUBSCRIBE },  /* MQTT connected    -> subscribe        */
        { "OK",            LTE_SM_RUN            },  /* subscribed        -> running          */
    };

    lte_sm_init();

    /* Two ticks: INIT -> POWER_ON, then POWER_ON -> WAIT_BOOT (sends AT). */
    step_sm();
    step_sm();

    if (lte_sm_get_state() != LTE_SM_WAIT_BOOT) {
        printf("FAIL: after power-on state = %d (expected WAIT_BOOT)\n",
               lte_sm_get_state());
        return 0;
    }

    for (size_t i = 0; i < sizeof(steps) / sizeof(steps[0]); i++) {
        deliver_line(steps[i].reply);   /* the reply is handled...            */
        step_sm();                      /* ...then a tick sends what comes next */

        LteSmState_t st = lte_sm_get_state();
        if (st != steps[i].expect) {
            printf("FAIL: step %u (reply \"%s\"): state = %d (expected %d)\n",
                   (unsigned)(i + 1), steps[i].reply, st, steps[i].expect);
            return 0;
        }
    }

    printf("PASS: reached LTE_SM_RUN\n");
    return 1;
}

// Test: the same command failing MAX_RETRIES times must reboot the modem
static int test_retries_exhausted_reboot(void)
{
    /* Same value as MAX_RETRIES in lte_sm.c. If you change it there,
     * change it here too. */
    const int max_retries = 3;

    printf("TEST: retries exhausted -> modem reboot\n");
    sm_start_in_attach();            /* AT+CGATT=1 sent */

    /* Fail the attach until the retry limit is reached. Below the limit
     * the machine goes to REATTACH and a tick sends the command again. */
    for (int i = 1; i <= max_retries; i++) {
        deliver_line("ERROR");

        if (i < max_retries) {
            if (lte_sm_get_state() != LTE_SM_REATTACH) {
                printf("FAIL: failure %d: state = %d (expected REATTACH)\n",
                       i, lte_sm_get_state());
                return 0;
            }
            step_sm();               /* REATTACH -> ATTACH, command resent */
        }
    }

    /* The last failure reaches the limit: the machine must give up. */
    if (lte_sm_get_state() != LTE_SM_ERROR) {
        printf("FAIL: after %d failures state = %d (expected ERROR)\n",
               max_retries, lte_sm_get_state());
        return 0;
    }

    /* The next tick sees "retries exhausted" and chooses a reboot. */
    step_sm();
    if (lte_sm_get_state() != LTE_SM_REBOOT_MODEM) {
        printf("FAIL: state = %d (expected REBOOT_MODEM)\n",
               lte_sm_get_state());
        return 0;
    }

    /* One more tick performs the reset and restarts the boot sequence. */
    step_sm();
    if (lte_sm_get_state() != LTE_SM_WAIT_BOOT) {
        printf("FAIL: after reboot state = %d (expected WAIT_BOOT)\n",
               lte_sm_get_state());
        return 0;
    }

    printf("PASS: ERROR -> REBOOT_MODEM -> WAIT_BOOT\n");
    return 1;
}

// Test: a modem that never answers AT is retried, then rebooted
static int test_boot_silence_reboot(void)
{
    /* Same values as BOOT_STEP_TIMEOUT_MS and BOOT_MAX_SENDS in lte_sm.c.
     * If you change them there, change them here too. */
    const uint32_t step_timeout_ms = 2000;
    const uint32_t max_sends       = 15;
    const uint32_t expected_ms     = step_timeout_ms * max_sends;  /* 30000 */

    printf("TEST: boot silence -> retries -> reboot\n");

    lte_sm_init();
    step_sm();                       /* INIT -> POWER_ON              */
    step_sm();                       /* POWER_ON -> WAIT_BOOT, AT sent */

    if (lte_sm_get_state() != LTE_SM_WAIT_BOOT) {
        printf("FAIL: state = %d (expected WAIT_BOOT)\n", lte_sm_get_state());
        return 0;
    }

    /* The modem stays silent. Move time forward 1 ms at a time until the
     * machine stops waiting (cap at 40 s so a bug cannot hang the test). */
    uint32_t waited = 0;
    while (lte_sm_get_state() == LTE_SM_WAIT_BOOT && waited < 40000) {
        lte_hw_delay_ms(1);
        step_sm();
        waited++;
    }

    /* It must give up with ERROR, and only after about 15 tries of 2 s
     * each. Giving up much earlier would mean too few retries. */
    if (lte_sm_get_state() != LTE_SM_ERROR) {
        printf("FAIL: after %u ms state = %d (expected ERROR)\n",
               (unsigned)waited, lte_sm_get_state());
        return 0;
    }
    if (waited < expected_ms - step_timeout_ms ||
        waited > expected_ms + step_timeout_ms) {
        printf("FAIL: gave up after %u ms (expected about %u ms)\n",
               (unsigned)waited, (unsigned)expected_ms);
        return 0;
    }

    /* A boot failure has no specific recovery state, so it must reboot. */
    step_sm();
    if (lte_sm_get_state() != LTE_SM_REBOOT_MODEM) {
        printf("FAIL: state = %d (expected REBOOT_MODEM)\n",
               lte_sm_get_state());
        return 0;
    }

    step_sm();                       /* reset the modem, restart boot */
    if (lte_sm_get_state() != LTE_SM_WAIT_BOOT) {
        printf("FAIL: after reboot state = %d (expected WAIT_BOOT)\n",
               lte_sm_get_state());
        return 0;
    }

    printf("PASS: gave up after %u ms -> REBOOT_MODEM -> WAIT_BOOT\n",
           (unsigned)waited);
    return 1;
}

/* ============================================================================
   Recovery tests: every command has its own failure path
   ============================================================================ */

/* A failure reply in a state must lead to that command's recovery state. */
static int test_reply_recovery(const char *name, start_fn_t start,
                               const char *reply, LteSmState_t expect)
{
    printf("TEST: %s\n", name);
    start();
    deliver_line(reply);
    return check_state(expect, name);
}

/* No reply for longer than the command timeout must also lead to recovery.
 * The timeout fires when MORE than timeout_ms has passed, so we advance
 * timeout_ms + 1 and stop right there. */
static int test_timeout_recovery(const char *name, start_fn_t start,
                                 uint32_t timeout_ms, LteSmState_t expect)
{
    printf("TEST: %s\n", name);
    start();
    advance_time_ms(timeout_ms + 1);
    return check_state(expect, name);
}

/* The same command failing the maximum number of times must reboot the
 * modem (see test_retries_exhausted_reboot for the attach version). */
static int test_retries_exhausted_for(const char *name, start_fn_t start,
                                      LteSmState_t recovery)
{
    const int max_retries = 3;       /* same as MAX_RETRIES in lte_sm.c */

    printf("TEST: %s\n", name);
    start();

    for (int i = 1; i <= max_retries; i++) {
        deliver_line("ERROR");
        if (i < max_retries) {
            if (lte_sm_get_state() != recovery) {
                printf("FAIL: %s: failure %d: state = %d (expected %d)\n",
                       name, i, lte_sm_get_state(), recovery);
                return 0;
            }
            step_sm();               /* recovery state -> command resent */
        }
    }

    if (lte_sm_get_state() != LTE_SM_ERROR) {
        printf("FAIL: %s: after %d failures state = %d (expected ERROR)\n",
               name, max_retries, lte_sm_get_state());
        return 0;
    }
    step_sm();
    if (lte_sm_get_state() != LTE_SM_REBOOT_MODEM) {
        printf("FAIL: %s: state = %d (expected REBOOT_MODEM)\n",
               name, lte_sm_get_state());
        return 0;
    }
    step_sm();
    return check_state(LTE_SM_WAIT_BOOT, name);
}

// Test: "+CME ERROR" is recognised as a failure, like a plain ERROR
static int test_cme_error_is_failure(void)
{
    return test_reply_recovery("+CME ERROR on attach -> REATTACH",
                               sm_start_in_attach, "+CME ERROR: 30",
                               LTE_SM_REATTACH);
}

// Test: one failed attach, then the retry succeeds
static int test_attach_fail_once_then_ok(void)
{
    printf("TEST: attach fails once, retry succeeds\n");
    sm_start_in_attach();

    modem_reply("ERROR");            /* fail; the tick resends the command */
    if (lte_sm_get_state() != LTE_SM_ATTACH) {
        printf("FAIL: state = %d (expected ATTACH after retry)\n",
               lte_sm_get_state());
        return 0;
    }

    deliver_line("OK");
    return check_state(LTE_SM_PDP, "retry succeeded");
}

// Test: a success clears the retry counter, so old failures do not add up
static int test_retry_counter_reset(void)
{
    printf("TEST: success resets the retry counter\n");
    sm_start_in_attach();

    modem_reply("ERROR");            /* failure 1 */
    modem_reply("ERROR");            /* failure 2 */
    deliver_line("OK");              /* success: counter must go back to 0 */
    if (lte_sm_get_state() != LTE_SM_PDP) {
        printf("FAIL: state = %d (expected PDP)\n", lte_sm_get_state());
        return 0;
    }

    /* Start a new attach and fail once. If the counter was NOT cleared this
     * would be failure number 3 and the machine would go to ERROR. */
    lte_sm_force_state(LTE_SM_REATTACH);
    step_sm();
    deliver_line("ERROR");
    return check_state(LTE_SM_REATTACH, "one failure after a success");
}

/* ============================================================================
   Boot tests
   ============================================================================ */

// Test: a SIM that is not ready blocks the boot, and it continues once ready
static int test_sim_not_ready(void)
{
    printf("TEST: SIM not ready blocks boot, then recovers\n");
    sm_start_boot();
    modem_reply("OK");               /* AT   -> sends ATE0     */
    modem_reply("OK");               /* ATE0 -> sends AT+CPIN? */

    modem_reply("+CPIN: SIM PIN");   /* SIM locked */
    modem_reply("OK");
    modem_reply("OK");
    modem_reply("OK");               /* if boot wrongly moved on, this would be ATTACH */

    if (lte_sm_get_state() != LTE_SM_CHECK_SIM) {
        printf("FAIL: state = %d (expected CHECK_SIM, boot must wait)\n",
               lte_sm_get_state());
        return 0;
    }

    /* After the step timeout the machine asks again; now the SIM is ready. */
    advance_time_ms(2000);
    modem_reply("+CPIN: READY");
    modem_reply("OK");               /* CPIN done -> sends APN   */
    modem_reply("OK");               /* APN done  -> sends CEREG */
    modem_reply("OK");               /* CEREG done -> attach     */
    return check_state(LTE_SM_ATTACH, "boot continued after SIM ready");
}

// Test: errors on the APN and CEREG steps are not critical
static int test_boot_noncritical_errors(void)
{
    printf("TEST: APN and CEREG errors do not stop boot\n");
    sm_start_boot();
    modem_reply("OK");               /* AT   */
    modem_reply("OK");               /* ATE0 */
    modem_reply("+CPIN: READY");
    modem_reply("OK");               /* CPIN done -> sends APN */

    modem_reply("ERROR");            /* APN failed   -> carry on */
    modem_reply("ERROR");            /* CEREG failed -> carry on */
    return check_state(LTE_SM_ATTACH, "boot went on to ATTACH");
}

/* ============================================================================
   Registration reports (+CEREG)
   ============================================================================ */

// Test: "not registered" is normal before the attach has finished
static int test_cereg_lost_before_attach_ignored(void)
{
    printf("TEST: +CEREG 0 while attaching is ignored\n");
    sm_start_in_attach();
    deliver_line("+CEREG: 0,0");
    return check_state(LTE_SM_ATTACH, "still attaching");
}

// Test: "registered" changes nothing
static int test_cereg_registered_no_change(void)
{
    printf("TEST: +CEREG 1 (registered) changes nothing\n");
    sm_start_in_pdp();
    deliver_line("+CEREG: 1");
    return check_state(LTE_SM_PDP, "still in PDP");
}

// Test: registration denied is a failure and leads to a re-attach
static int test_cereg_denied(void)
{
    printf("TEST: +CEREG 3 (denied) -> ERROR -> REATTACH\n");
    sm_start_in_attach();
    deliver_line("+CEREG: 3");

    if (!check_state(LTE_SM_ERROR, "registration denied")) {
        return 0;
    }
    step_sm();
    return check_state(LTE_SM_REATTACH, "recovery after denial");
}

/* ============================================================================
   Running state
   ============================================================================ */

// Test: an external failure while running reboots the modem
static int test_failure_in_run_reboots(void)
{
    LteEvent_t ev;

    printf("TEST: failure in RUN -> reboot\n");
    lte_sm_init();
    lte_sm_force_state(LTE_SM_RUN);

    memset(&ev, 0, sizeof(ev));
    ev.type = LTE_EVENT_ERROR;
    lte_sm_handle_event(&ev);

    if (!check_state(LTE_SM_ERROR, "failure reported")) {
        return 0;
    }
    step_sm();                       /* RUN has no specific recovery */
    return check_state(LTE_SM_REBOOT_MODEM, "reboot chosen");
}

// Test: a stray OK when nothing is pending must not disturb the machine
static int test_stray_ok_in_run(void)
{
    printf("TEST: stray OK in RUN is ignored\n");
    lte_sm_init();
    lte_sm_force_state(LTE_SM_RUN);
    deliver_line("OK");
    return check_state(LTE_SM_RUN, "still RUN");
}

/* -------------------------------
   Test runner
   ------------------------------- */

/* Runs every test and returns how many FAILED (0 = all passed). */
int lte_run_tests(void)
{
    int pass = 0;  // Count of passed tests
    int total = 0;  // Count of total tests

    /* Initialize hardware layer in test mode if needed */
    lte_hw_init();

    /* You may want to reset state machine to INIT before each test */
    
    total++; pass += test_attach_ok();
    
    total++; pass += test_attach_error_retry();
 
    total++; pass += test_pdp_ok();
 
    total++; pass += test_cereg_network_lost();
  
    total++; pass += test_timeout_attach();

    total++; pass += test_full_path_to_run();

    total++; pass += test_retries_exhausted_reboot();

    total++; pass += test_boot_silence_reboot();

    /* Failure replies: each command goes to its own recovery state */
    total++; pass += test_reply_recovery("PDP error -> REPDP",
                         sm_start_in_pdp, "ERROR", LTE_SM_REPDP);
    total++; pass += test_reply_recovery("TCP open error -> REOPEN_SOCKET",
                         sm_start_in_socket, "ERROR", LTE_SM_REOPEN_SOCKET);
    total++; pass += test_reply_recovery("MQTT connect error -> MQTT_RECONNECT",
                         sm_start_in_mqtt_connect, "ERROR", LTE_SM_MQTT_RECONNECT);
    total++; pass += test_reply_recovery("MQTT subscribe error -> MQTT_RESUBSCRIBE",
                         sm_start_in_mqtt_subscribe, "ERROR", LTE_SM_MQTT_RESUBSCRIBE);
    total++; pass += test_cme_error_is_failure();

    /* Silence: each command has its own timeout */
    total++; pass += test_timeout_recovery("PDP timeout -> REPDP",
                         sm_start_in_pdp, 10000, LTE_SM_REPDP);
    total++; pass += test_timeout_recovery("TCP open timeout -> REOPEN_SOCKET",
                         sm_start_in_socket, 8000, LTE_SM_REOPEN_SOCKET);
    total++; pass += test_timeout_recovery("MQTT connect timeout -> MQTT_RECONNECT",
                         sm_start_in_mqtt_connect, 8000, LTE_SM_MQTT_RECONNECT);
    total++; pass += test_timeout_recovery("MQTT subscribe timeout -> MQTT_RESUBSCRIBE",
                         sm_start_in_mqtt_subscribe, 8000, LTE_SM_MQTT_RESUBSCRIBE);

    /* Escalation: too many failures reboot the modem */
    total++; pass += test_retries_exhausted_for("PDP retries exhausted -> reboot",
                         sm_start_in_pdp, LTE_SM_REPDP);
    total++; pass += test_retries_exhausted_for("TCP retries exhausted -> reboot",
                         sm_start_in_socket, LTE_SM_REOPEN_SOCKET);
    total++; pass += test_retries_exhausted_for("MQTT connect retries exhausted -> reboot",
                         sm_start_in_mqtt_connect, LTE_SM_MQTT_RECONNECT);
    total++; pass += test_retries_exhausted_for("MQTT subscribe retries exhausted -> reboot",
                         sm_start_in_mqtt_subscribe, LTE_SM_MQTT_RESUBSCRIBE);

    /* Retry bookkeeping */
    total++; pass += test_attach_fail_once_then_ok();
    total++; pass += test_retry_counter_reset();

    /* Boot sequence details */
    total++; pass += test_sim_not_ready();
    total++; pass += test_boot_noncritical_errors();

    /* Registration reports */
    total++; pass += test_cereg_lost_before_attach_ignored();
    total++; pass += test_cereg_registered_no_change();
    total++; pass += test_cereg_denied();

    /* Running state */
    total++; pass += test_failure_in_run_reboots();
    total++; pass += test_stray_ok_in_run();

    printf("\nTEST SUMMARY: %d/%d passed\n", pass, total);
    return total - pass;
}

/* Kept so existing callers (for example the ESP32 app) still work. */
void lte_run_all_tests(void)
{
    (void)lte_run_tests();
}