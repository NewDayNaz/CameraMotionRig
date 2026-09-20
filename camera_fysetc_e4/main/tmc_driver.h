/**
 * @file tmc_driver.h
 * @brief TMC2209 UART configuration for FYSETC E4 (shared UART1)
 *
 * Sets current and microsteps. Pan/tilt use spreadCycle. Zoom uses stealthChop
 * so StallGuard4 can see load (TMC2209 SG is invalid in spreadCycle).
 */

#ifndef TMC_DRIVER_H
#define TMC_DRIVER_H

#include <stdbool.h>
#include <stdint.h>
#include "board.h"

typedef struct {
    uint8_t axis;
    uint8_t address;
    bool uart_ok;          /* GCONF or IOIN read succeeded */
    bool write_ack;        /* IFCNT incremented after a dummy write */
    uint8_t ic_version;    /* IOIN[31:24], TMC2209 = 0x21 */
    uint32_t gconf;
    uint32_t gstat;
    uint32_t ifcnt_before;
    uint32_t ifcnt_after;
    uint32_t ioin;
    uint32_t tstep;
    uint32_t tcoolthrs;
    uint32_t sgthrs;
    uint32_t sg_result;
    uint32_t chopconf;
    uint32_t ihold_irun;
    uint8_t ihold;
    uint8_t irun;
    uint8_t pwm_scale_sum; /* PWM_SCALE_SUM; 0 = stealthChop not running, 255 = can't hold current */
    int rx_len;            /* raw bytes seen on the GCONF read (0 = RX silent) */
    char rx_hex[48];       /* first raw RX bytes, space-separated hex */
} tmc_diag_t;

typedef struct {
    int tx_len;
    int rx_len;
    char rx_hex[48];
    int loopback_rx;       /* UART1 internal loopback, no GPIO */
    int hw_rxfifo;         /* UART1 RX FIFO count after TX, before drain */
    bool tx_done;
    int gpio21_idle;
    int gpio22_idle;
    int gpio21_tx_low;     /* GPIO21 while bit-banged GPIO22 = 0 */
    int gpio21_tx_high;
    bool pins_tied;        /* GPIO21 followed GPIO22 */
    int out_sel_tx;        /* GPIO matrix out_sel on GPIO22; UART1 TX = 17 */
    int in_sel_rx;         /* GPIO matrix in_sel for UART1 RX; should be 21 */
} tmc_loopback_t;

bool tmc_driver_init(void);
bool tmc_driver_is_ready(void);
bool tmc_driver_uart_installed(void);

bool tmc_driver_read_sg_result(uint8_t axis, uint16_t *sg_result);
bool tmc_driver_read_sg_tstep(uint8_t axis, uint16_t *sg_result, uint32_t *tstep);
bool tmc_driver_read_sg_tstep_pwm(uint8_t axis, uint16_t *sg_result, uint32_t *tstep, uint8_t *pwm_sum);
/** Last zoom SG/TSTEP/PWM from the background poll task (never call from the 1 ms step loop). */
bool tmc_driver_get_cached_sg_tstep(uint16_t *sg_result, uint32_t *tstep);
bool tmc_driver_get_cached_sg_tstep_pwm(uint16_t *sg_result, uint32_t *tstep, uint8_t *pwm_sum);
bool tmc_driver_diagnose_axis(uint8_t axis, tmc_diag_t *out);
bool tmc_driver_bus_echo(tmc_loopback_t *out);
bool tmc_driver_reconfigure(void);

/** RUN currents (motion) vs extra-low standstill after long idle. */
void tmc_driver_set_standby(bool standby);
uint8_t tmc_driver_get_irun(uint8_t axis);
bool tmc_driver_set_irun(uint8_t axis, uint8_t cs);

#endif // TMC_DRIVER_H
