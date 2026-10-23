#pragma once
#include <stdint.h>

/**
 * @file lte_hw.h
 * @brief LTE hardware interface: UART, modem power/reset pins and time.
 *
 * This is the lowest layer. It knows nothing about the driver, AT commands
 * or the state machine above it. Received bytes are handed upward through a
 * callback registered with lte_hw_set_rx_callback().
 */

/**
 * @brief Initialises the LTE hardware interface (UART and its receive task).
 *
 * Call once at startup. Register the receive callback BEFORE calling this
 * (lte_driver_init() does that), otherwise early bytes are dropped.
 */
void lte_hw_init(void);

/**
 * @brief Writes a null-terminated string to the modem via UART.
 *
 * @param data String to send.
 * @note The UART must be initialised first (lte_hw_init).
 */
void lte_hw_uart_write(const char *data);

/** Function that receives each byte read from the modem UART. */
typedef void (*lte_hw_rx_cb_t)(uint8_t byte);

/**
 * @brief Registers the function that receives each byte from the modem.
 *
 * Called from the UART receive task context. Pass NULL to unregister.
 */
void lte_hw_set_rx_callback(lte_hw_rx_cb_t cb);

/**
 * @brief Milliseconds since start, from the millisecond tick counter.
 *
 * @note The counter only advances when lte_hw_tick_1ms() is called every 1 ms.
 */
uint32_t lte_hw_ms(void);

/**
 * @brief Executes the modem power-on sequence (PWRKEY and RESET pins).
 */
void lte_hw_power_on_sequence(void);

/**
 * @brief Hardware reset of the modem by pulsing the reset pin.
 */
void lte_hw_reset_modem(void);

/**
 * @brief Increments the millisecond tick counter.
 *
 * Must be called every 1 ms (timer callback or RTOS tick hook).
 */
void lte_hw_tick_1ms(void);

/**
 * @brief Blocks the calling task for the given number of milliseconds.
 *
 * Uses the RTOS delay, so other tasks keep running. Must not be called
 * from an interrupt handler.
 */
void lte_hw_delay_ms(uint32_t ms);