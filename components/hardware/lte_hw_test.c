#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "lte_hw.h"

static uint32_t fake_ms = 0;

static char last_cmd[128];
static int  power_on_count = 0;
static int  reset_count    = 0;

void lte_hw_init(void)
{
    fake_ms = 0;
    printf("[TEST HW INIT]\n");
}

uint32_t lte_hw_ms(void)
{
    return fake_ms;
}

void lte_hw_delay_ms(uint32_t ms)
{   
    
    fake_ms += ms;
}



void lte_hw_uart_write(const char *data)
{
    strncpy(last_cmd, data, sizeof(last_cmd) - 1);
    last_cmd[sizeof(last_cmd) - 1] = '\0';
    printf("[TEST HW TX] %s", data);
}

void lte_hw_power_on_sequence(void)
{
    power_on_count++;
    printf("[TEST HW] power-on sequence\n");
}

void lte_hw_reset_modem(void)
{
    reset_count++;
    printf("[TEST HW] modem reset\n");
}

/* Test support: what was the last command sent to the modem? */
const char *lte_hw_last_cmd(void)
{
    return last_cmd;
}

/* Test support: forget the last command, so a test cannot see
 * a command left over from an earlier test. */
void lte_hw_clear_last_cmd(void)
{
    last_cmd[0] = '\0';
}