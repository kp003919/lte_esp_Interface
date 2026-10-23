/******************************************************************************
 * @file   app.c
 * @brief  Water leak detection application on top of the LTE state machine.
 *
 * DESIGN IDEAS (read these first, the code follows them)
 * ------------------------------------------------------
 * 1. DETECTION NEVER WAITS FOR THE MODEM.
 *    Sensors are read on every loop, even when the modem is reattaching or
 *    reconnecting. A leak that starts while the modem is busy is still seen.
 *
 * 2. AN ALARM IS "PENDING" UNTIL THE BROKER CONFIRMS IT.
 *    When an alarm starts or ends, a message is placed in a small queue.
 *    It is removed from the queue ONLY when the MQTT layer reports success.
 *    If sending fails, the message stays and is sent again later.
 *
 * 3. ONLY ONE PUBLISH IS IN FLIGHT AT A TIME.
 *    The application never calls the modem twice in a row. It sends one
 *    message, waits for the result (success / failure / timeout), then the
 *    next one. This avoids mixing up replies from the modem.
 *
 * 4. ALARMS HAVE PRIORITY OVER TELEMETRY.
 *    If an alarm and a heartbeat are both waiting, the alarm goes first.
 *
 * 5. NO RAW TCP SEND ON THE MQTT SOCKET.
 *    MQTT runs on top of the TCP socket. Writing raw text into the same
 *    socket corrupts the MQTT stream and the broker drops the connection.
 *    MQTT already has its own keep-alive, so the TCP heartbeat is removed.
 *    (Set ENABLE_TCP_HEARTBEAT to 1 only if you use a SEPARATE TCP socket.)
 *
 * ASSUMPTIONS ABOUT YOUR OTHER MODULES (adapt if yours differ)
 * ------------------------------------------------------------
 *  - lte_mqtt_publish(topic, payload) returns non-zero if the request was
 *    accepted for sending, and 0 if it could not be started.
 *  - When the final result of that publish is known (broker confirmed with
 *    PUBACK, or the modem returned an error), the MQTT layer calls
 *      void lte_app_on_publish_result(bool ok);
 *    which is defined at the bottom of this file. Declare it in a header
 *    (for example lte_mqtt.h) so lte_mqtt.c can call it.
 *  - lte_app_on_publish_result() must be called from the same task that
 *    runs lte_app_loop(), or be protected by a mutex.
 ******************************************************************************/

#include "lte_sm.h"
#include "lte_driver.h"
#include "lte_hw.h"
#include "lte_test_harness.h"
#include "lte_mqtt.h"
#include "lte_tcp.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ============================================================================
 * 1. BUILD OPTIONS
 * ============================================================================
 */

/* 1 = run the LTE test harness and stop. 0 = run the real application. */
#define LTE_TEST_MODE 1

/* 1 = also send a raw TCP heartbeat. Keep 0 when MQTT uses the same socket. */
#define ENABLE_TCP_HEARTBEAT 0

/* ============================================================================
 * 2. ALARM THRESHOLDS AND MQTT TOPICS
 * ============================================================================
 *
 * Hysteresis: once an alarm is active, the value must go back PAST the
 * threshold by a margin before the alarm is cleared. Without this, a sensor
 * hovering around the threshold would raise and clear the alarm many times.
 *
 * Example (level): alarm raised above 80.0, cleared only below 78.0.
 */
#define MAX_LEVEL        80.0f   /* Alarm if water level is above this        */
#define MIN_PRESSURE     30.0f   /* Alarm if pressure is below this           */
#define LEVEL_HYST        2.0f   /* Margin below MAX_LEVEL to clear the alarm */
#define PRESSURE_HYST     2.0f   /* Margin above MIN_PRESSURE to clear it     */

/* A new alarm state must stay true for this long before it is accepted.
 * This filters out one-sample glitches from the sensors. */
#define ALARM_CONFIRM_MS 1000

#define TOPIC_ALARM      "water/alarms"
#define TOPIC_TELEMETRY  "device/telemetry"

/* ============================================================================
 * 3. TIMING AND QUEUE SETTINGS
 * ============================================================================
 */
#define TELEMETRY_PERIOD_MS   15000  /* Heartbeat every 15 s                    */
#define TCP_HEARTBEAT_MS      10000  /* Only used if ENABLE_TCP_HEARTBEAT == 1  */
#define PUBLISH_TIMEOUT_MS    10000  /* Max wait for one publish result         */
#define RETRY_DELAY_MS         5000  /* Wait after a failure before retrying    */
#define LOOP_DELAY_MS            20  /* Main loop sleep (lets idle task run)    */

#define ALARM_QUEUE_SIZE          4  /* Alarm messages waiting to be confirmed  */
#define ALARM_PAYLOAD_SIZE       96  /* Max length of one alarm message         */

/* ============================================================================
 * 4. TIME HELPER
 * ============================================================================
 *
 * lte_hw_ms() is a 32-bit millisecond counter that wraps after ~49 days.
 * Subtracting unsigned values (now - then) stays correct across the wrap,
 * so we always compare time like this: elapsed(now, then) >= period.
 */
static inline uint32_t elapsed_ms(uint32_t now, uint32_t then)
{
    return (uint32_t)(now - then);
}

/* ============================================================================
 * 5. SENSOR READING (MOCK)
 * ============================================================================
 *
 * In the real product these read the hardware (ADC, GPIO, ...).
 * They are called on EVERY loop, regardless of the modem state.
 */
static float read_water_level_sensor(void) { return 50.0f; }
static float read_pressure_sensor(void)    { return 40.0f; }
static int   read_leak_sensor(void)        { return 0; }

/* ============================================================================
 * 6. ALARM MESSAGE QUEUE (the "pending" list)
 * ============================================================================
 *
 * A small ring buffer of alarm messages that have NOT been confirmed by the
 * broker yet. The message at the head is always the next one to send.
 *
 * The payload text is built at the moment of detection, so it contains the
 * sensor values from that moment, not from the moment it is finally sent.
 *
 * If the queue is full (a very long outage with many alarm changes), the
 * OLDEST message is dropped and counted, so the newest state is never lost.
 */
typedef struct {
    char payload[ALARM_PAYLOAD_SIZE];
} AlarmMsg_t;

static AlarmMsg_t s_alarm_q[ALARM_QUEUE_SIZE];
static uint8_t    s_alarm_head  = 0;   /* Index of the oldest message */
static uint8_t    s_alarm_count = 0;   /* Number of messages waiting  */
static uint32_t   s_alarm_dropped = 0; /* Messages lost to overflow   */

static void alarm_queue_push(const char *payload)
{
    /* Queue full: drop the oldest message to make room. */
    if (s_alarm_count == ALARM_QUEUE_SIZE) {
        s_alarm_head = (uint8_t)((s_alarm_head + 1) % ALARM_QUEUE_SIZE);
        s_alarm_count--;
        s_alarm_dropped++;
        printf("WARNING: alarm queue full, oldest dropped (total %lu)\n",
               (unsigned long)s_alarm_dropped);
    }

    uint8_t tail = (uint8_t)((s_alarm_head + s_alarm_count) % ALARM_QUEUE_SIZE);
    strncpy(s_alarm_q[tail].payload, payload, ALARM_PAYLOAD_SIZE - 1);
    s_alarm_q[tail].payload[ALARM_PAYLOAD_SIZE - 1] = '\0';
    s_alarm_count++;
}

static const char *alarm_queue_peek(void)
{
    return (s_alarm_count > 0) ? s_alarm_q[s_alarm_head].payload : NULL;
}

static void alarm_queue_pop(void)
{
    if (s_alarm_count > 0) {
        s_alarm_head = (uint8_t)((s_alarm_head + 1) % ALARM_QUEUE_SIZE);
        s_alarm_count--;
    }
}

/* ============================================================================
 * 7. ALARM DETECTION
 * ============================================================================
 *
 * Runs on every loop, whatever the modem is doing.
 *
 * Steps:
 *   1. Read all sensors.
 *   2. Decide what the alarm state SHOULD be (with hysteresis).
 *   3. If it differs from the current state, wait ALARM_CONFIRM_MS to be sure.
 *   4. When confirmed, change the state and put a message in the queue.
 *
 * Two messages exist: STATE=RAISED (alarm began) and STATE=CLEARED (alarm
 * ended). The user's display needs both.
 */
static bool     s_alarm_active    = false; /* Confirmed alarm state            */
static bool     s_change_pending  = false; /* A different state is being timed */
static uint32_t s_change_start_ms = 0;     /* When that change was first seen  */

static void alarm_detect(uint32_t now)
{
    float level    = read_water_level_sensor();
    float pressure = read_pressure_sensor();
    int   leak     = read_leak_sensor();

    /* Step 2: what should the alarm state be right now? */
    bool want_active;
    if (!s_alarm_active) {
        /* Currently OK: raise as soon as any threshold is crossed. */
        want_active = (level > MAX_LEVEL) || leak || (pressure < MIN_PRESSURE);
    } else {
        /* Currently in alarm: stay in alarm until EVERYTHING is back to
         * normal by more than the hysteresis margin. */
        bool all_clear = (level    < (MAX_LEVEL    - LEVEL_HYST))
                      && (!leak)
                      && (pressure > (MIN_PRESSURE + PRESSURE_HYST));
        want_active = !all_clear;
    }

    /* Step 3: no change wanted -> cancel any change being timed. */
    if (want_active == s_alarm_active) {
        s_change_pending = false;
        return;
    }

    /* A change is wanted. Start timing it the first time we see it. */
    if (!s_change_pending) {
        s_change_pending  = true;
        s_change_start_ms = now;
        return;
    }

    /* Still waiting for the confirmation time to pass. */
    if (elapsed_ms(now, s_change_start_ms) < ALARM_CONFIRM_MS) {
        return;
    }

    /* Step 4: change confirmed. Update the state and queue a message. */
    s_alarm_active   = want_active;
    s_change_pending = false;

    char payload[ALARM_PAYLOAD_SIZE];
    snprintf(payload, sizeof(payload),
             "ALARM:STATE=%s;LEVEL=%.1f;LEAK=%d;PRESS=%.2f",
             s_alarm_active ? "RAISED" : "CLEARED",
             level, leak, pressure);

    printf("Alarm %s -> queued: %s\n",
           s_alarm_active ? "RAISED" : "CLEARED", payload);

    alarm_queue_push(payload);
}

/* ============================================================================
 * 8. PUBLISH MANAGER (one message in flight at a time)
 * ============================================================================
 *
 * s_inflight says what we are currently waiting for:
 *   NONE      - nothing sent, the modem is free for a new message
 *   ALARM     - the alarm at the head of the queue was sent, waiting result
 *   TELEMETRY - a heartbeat was sent, waiting result
 *
 * While s_inflight != NONE we never start another publish.
 *
 * After a failure we wait RETRY_DELAY_MS before trying again, so we do not
 * hammer a modem that is busy recovering.
 */
typedef enum {
    INFLIGHT_NONE = 0,
    INFLIGHT_ALARM,
    INFLIGHT_TELEMETRY
} Inflight_t;

static Inflight_t s_inflight        = INFLIGHT_NONE;
static uint32_t   s_inflight_ms     = 0;     /* When the publish was started   */
static bool       s_backoff_active  = false; /* Waiting before the next try    */
static uint32_t   s_backoff_start   = 0;     /* When the backoff began         */
static uint32_t   s_last_telemetry  = 0;     /* Last heartbeat start time      */

/* Called whenever a publish failed, timed out, or the modem left RUN.
 * The alarm (if any) is NOT removed from the queue, so it will be retried. */
static void publish_failed(uint32_t now, const char *reason)
{
    printf("Publish failed (%s). In flight was %d. Alarms still pending: %u\n",
           reason, (int)s_inflight, (unsigned)s_alarm_count);

    s_inflight       = INFLIGHT_NONE;
    s_backoff_active = true;
    s_backoff_start  = now;

    /* If the modem still thinks it is fine, tell the state machine that
     * something is wrong so it runs its recovery path. If it has already
     * left RUN, it is recovering by itself. */
    if (lte_sm_get_state() == LTE_SM_RUN) {
        lte_sm_set_state(LTE_SM_ERROR);
    }
}

static void publish_manager(uint32_t now)
{
    /* --- A. Something is in flight: check for loss of link or timeout ---- */
    if (s_inflight != INFLIGHT_NONE) {

        /* The modem left RUN while we were waiting. We can never be sure
         * the message arrived, so treat it as failed. For an alarm this means
         * it is sent again (the broker may see it twice, which is much better
         * than never seeing it). */
        if (lte_sm_get_state() != LTE_SM_RUN) {
            publish_failed(now, "modem left RUN");
            return;
        }

        if (elapsed_ms(now, s_inflight_ms) >= PUBLISH_TIMEOUT_MS) {
            publish_failed(now, "timeout");
        }
        return; /* Still waiting: never start a second publish. */
    }

    /* --- B. Nothing in flight, but the modem must be ready --------------- */
    if (lte_sm_get_state() != LTE_SM_RUN) {
        return;
    }

    /* --- C. Respect the retry delay after a failure ---------------------- */
    if (s_backoff_active) {
        if (elapsed_ms(now, s_backoff_start) < RETRY_DELAY_MS) {
            return;
        }
        s_backoff_active = false;
    }

    /* --- D. Alarms first (priority) -------------------------------------- */
    const char *alarm_payload = alarm_queue_peek();
    if (alarm_payload != NULL) {
        if (lte_mqtt_publish(TOPIC_ALARM, alarm_payload)) {
            s_inflight    = INFLIGHT_ALARM;
            s_inflight_ms = now;
            printf("Alarm publish started: %s\n", alarm_payload);
        } else {
            publish_failed(now, "alarm publish could not start");
        }
        return;
    }

    /* --- E. Then the periodic heartbeat ---------------------------------- */
    if (elapsed_ms(now, s_last_telemetry) >= TELEMETRY_PERIOD_MS) {
        if (lte_mqtt_publish(TOPIC_TELEMETRY, "Hello from Muhsin")) {
            s_inflight       = INFLIGHT_TELEMETRY;
            s_inflight_ms    = now;
            s_last_telemetry = now;
        } else {
            publish_failed(now, "telemetry publish could not start");
        }
    }
}

/* ============================================================================
 * 9. RESULT CALLBACK (called by the MQTT layer)
 * ============================================================================
 *
 * The MQTT layer calls this when the FINAL result of the publish is known:
 *   ok == true   the broker confirmed (for example PUBACK with QoS 1)
 *   ok == false  the modem reported an error
 *
 * This is the ONLY place where an alarm is removed from the queue.
 */
void lte_app_on_publish_result(bool ok)
{
    uint32_t now = lte_hw_ms();

    /* A result with nothing in flight is a late answer (for example after
     * our timeout already gave up). Ignore it, because we cannot know which
     * message it belongs to. */
    if (s_inflight == INFLIGHT_NONE) {
        printf("Ignoring late publish result (%d)\n", (int)ok);
        return;
    }

    if (!ok) {
        publish_failed(now, "MQTT layer reported failure");
        return;
    }

    /* Success. */
    if (s_inflight == INFLIGHT_ALARM) {
        printf("Alarm confirmed by broker, removed from queue\n");
        alarm_queue_pop();
    }
    s_inflight = INFLIGHT_NONE;
}

/* ============================================================================
 * 10. OPTIONAL RAW TCP HEARTBEAT
 * ============================================================================
 *
 * Disabled by default (see design idea 5). Only enable it with a separate
 * TCP socket, and only when no MQTT publish is in flight.
 */
#if ENABLE_TCP_HEARTBEAT
static uint32_t s_last_tcp = 0;

static void tcp_heartbeat(uint32_t now)
{
    if (lte_sm_get_state() != LTE_SM_RUN)         return;
    if (s_inflight != INFLIGHT_NONE)              return;
    if (elapsed_ms(now, s_last_tcp) < TCP_HEARTBEAT_MS) return;

    lte_tcp_send("Hello From ESP32");
    s_last_tcp = now;
}
#endif

/* ============================================================================
 * 11. APPLICATION LOOP
 * ============================================================================
 *
 * Called repeatedly from app_main().
 *
 *   1. Detect alarms     (always, independent of the modem)
 *   2. Send messages     (only when the modem is ready, one at a time)
 *
 * Handling incoming TCP data and incoming MQTT messages is not part of this
 * file yet. That belongs in the RUN state of the modem layer.
 */
static void lte_app_loop(void)
{
    uint32_t now = lte_hw_ms();

    /* Print a message only when the modem state changes, instead of
     * printing the same line thousands of times per second. */
    static LteSmState_t last_state = LTE_SM_INVALID;
    LteSmState_t state = lte_sm_get_state();
    if (state != last_state) {
        printf("Application sees modem state %d (%s)\n", (int)state,
               (state == LTE_SM_RUN) ? "ready" : "not ready");
        last_state = state;
    }

    /* 1. Detection: always runs. */
    alarm_detect(now);

    /* 2. Sending: only moves forward when the modem is in RUN. */
    publish_manager(now);

#if ENABLE_TCP_HEARTBEAT
    tcp_heartbeat(now);
#endif
}

/* ============================================================================
 * 12. MAIN ENTRY POINT
 * ============================================================================
 *
 * Modes:
 *   LTE_TEST_MODE = 1  Run the LTE test harness, then idle forever.
 *   LTE_TEST_MODE = 0  Normal firmware: process modem events and run the app.
 */
void app_main(void)
{
#if LTE_TEST_MODE
    printf("Running LTE modem test harness...\n");
    lte_run_all_tests();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

#else
    LteEvent_t ev;
    printf("Starting LTE modem firmware...\n");

    for (;;)
    {
        /* 1. Drain all pending modem events (lines from the modem). */
        while (lte_driver_get_next_event(&ev)) {
            lte_sm_handle_event(&ev);
        }

        /* 2. Forward-path tick: lets the state machine send its next command. */
        lte_sm_handle_event(NULL);

        /* 3. Application logic: detect alarms, send messages. */
        lte_app_loop();

        /* 4. Sleep briefly. Without this the loop spins at full speed, starves
         *    the idle task (watchdog reset) and wastes power. */
        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }
#endif
}