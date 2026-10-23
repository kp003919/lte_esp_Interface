#include "lte_driver.h"
#include "lte_hw.h"
#include <string.h>
#include <stdint.h>

#define LTE_EVENT_QUEUE_SIZE 16
#define LTE_LINE_BUF_SIZE    256

/* ---- Event queue: single producer (UART path), single consumer (task) ----
 * Each slot owns its line text, so queued events cannot overwrite each
 * other. event.line points into the slot and stays valid until the queue
 * wraps back to that slot (LTE_EVENT_QUEUE_SIZE - 1 later events). Consume
 * a line before fetching many more events. */
static LteEvent_t    s_events[LTE_EVENT_QUEUE_SIZE];
static char          s_event_lines[LTE_EVENT_QUEUE_SIZE][LTE_LINE_BUF_SIZE];
static volatile int  s_head = 0;
static volatile int  s_tail = 0;
static volatile uint32_t s_dropped = 0;

/* Deprecated helpers read this copy of the most recent line. */
static char s_last_line[LTE_LINE_BUF_SIZE];

/* Line assembly buffer used by the byte handler. */
static char rx_buf[LTE_LINE_BUF_SIZE];
static int  rx_pos = 0;

static void clear_state(void)
{
    s_head = 0;
    s_tail = 0;
    s_dropped = 0;
    s_last_line[0] = '\0';
    rx_pos = 0;
    rx_buf[0] = '\0';
}

/* Push an event. If the queue is full the event is dropped and counted. */
static void push_event(LteEventType_t type, const char *line)
{
    int head = s_head;
    int next = (head + 1) % LTE_EVENT_QUEUE_SIZE;

    if (next == s_tail) {
        s_dropped++;
        return;
    }

    s_events[head].type = type;
    if (line != NULL) {
        strncpy(s_event_lines[head], line, LTE_LINE_BUF_SIZE - 1);
        s_event_lines[head][LTE_LINE_BUF_SIZE - 1] = '\0';
        s_events[head].line = s_event_lines[head];
    } else {
        s_events[head].line = NULL;
    }

    s_head = next;               /* publish last */
}

/* ---- lifecycle ---- */

void lte_driver_init(void)  { clear_state(); }
void lte_driver_reset(void) { clear_state(); }

/* ---- hardware wrappers ---- */

void     lte_driver_power_on(void)    { lte_hw_power_on_sequence(); }
void     lte_driver_modem_reset(void) { lte_hw_reset_modem(); }
uint32_t lte_driver_now_ms(void)      { return lte_hw_ms(); }

/* ---- transmit ---- */

void lte_driver_send_cmd(const char *cmd)
{
    if (cmd == NULL) {
        return;
    }
    lte_hw_uart_write(cmd);
}

/* ---- receive ---- */

/* Accumulate bytes until '\n'. '\r' is ignored. Empty lines (the blank line
 * modems send before every response) are skipped. Over-long lines are
 * truncated. */
void lte_driver_on_rx_byte(uint8_t b)
{
    if (b == '\n') {
        if (rx_pos > 0) {
            rx_buf[rx_pos] = '\0';

            strncpy(s_last_line, rx_buf, LTE_LINE_BUF_SIZE - 1);
            s_last_line[LTE_LINE_BUF_SIZE - 1] = '\0';

            push_event(LTE_EVENT_RX_LINE, rx_buf);

            rx_pos = 0;
            rx_buf[0] = '\0';
        }
    } else if (b != '\r') {
        if (rx_pos < LTE_LINE_BUF_SIZE - 1) {
            rx_buf[rx_pos++] = (char)b;
        }
    }
}

int lte_driver_get_next_event(LteEvent_t *ev)
{
    int tail = s_tail;

    if (ev == NULL || s_head == tail) {
        return 0;
    }
    *ev = s_events[tail];
    s_tail = (tail + 1) % LTE_EVENT_QUEUE_SIZE;
    return 1;
}

uint32_t lte_driver_get_dropped_count(void) { return s_dropped; }

/* ---- timeout / error notifications ---- */

void lte_driver_push_timeout_event(void) { push_event(LTE_EVENT_TIMEOUT, NULL); }
void lte_driver_push_error_event(void)   { push_event(LTE_EVENT_ERROR,   NULL); }

/* ---- DEPRECATED last-line helpers ---- */

int lte_driver_last_line_contains(const char *substr)
{
    return substr != NULL && strstr(s_last_line, substr) != NULL;
}

int lte_driver_last_line_starts_with(const char *prefix)
{
    return prefix != NULL && strncmp(s_last_line, prefix, strlen(prefix)) == 0;
}

const char *lte_driver_get_last_line(void)
{
    return s_last_line;
}