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

    printf("\nTEST SUMMARY: %d/%d passed\n", pass, total);
    return total - pass;
}

/* Kept so existing callers (for example the ESP32 app) still work. */
void lte_run_all_tests(void)
{
    (void)lte_run_tests();
}