#ifndef LTE_DRIVER_H
#define LTE_DRIVER_H

#include "lte_types.h"
#include <stdint.h>

/**
 * @file lte_driver.h
 * @brief LTE modem driver: UART bytes in, line events out.
 *
 * Depends only on the hardware layer (lte_hw). Knows nothing about AT
 * command meaning: it sends strings and splits received bytes into lines.
 */

/* ---- lifecycle ---- */
void lte_driver_init(void);
void lte_driver_reset(void);          /* clear driver state (tests only) */

/* ---- modem power / reset / time (wrap the hardware layer) ---- */
void     lte_driver_power_on(void);
void     lte_driver_modem_reset(void);
uint32_t lte_driver_now_ms(void);

/* ---- transmit ---- */
void lte_driver_send_cmd(const char *cmd);

/* ---- receive ---- */
void lte_driver_on_rx_byte(uint8_t byte);       /* from the UART path */
int  lte_driver_get_next_event(LteEvent_t *ev); /* 1 = event, 0 = empty */
uint32_t lte_driver_get_dropped_count(void);    /* events lost, queue full */

/* ---- event injection (timeout / error notifications) ---- */
void lte_driver_push_timeout_event(void);
void lte_driver_push_error_event(void);

/* ---- DEPRECATED: raw last-line access, to be removed ----
 * Higher layers should use events instead of reading the buffer. */
int         lte_driver_last_line_contains(const char *substr);
int         lte_driver_last_line_starts_with(const char *prefix);
const char *lte_driver_get_last_line(void);

#endif /* LTE_DRIVER_H */