/**
 * @file tmc_driver.h
 * @brief TMC2209 UART configuration for FYSETC E4 (shared UART1)
 *
 * Sets current, microsteps, and spreadCycle. Zoom stallGuard is enabled for
 * sensorless homing and live crash-stop against the lens hard stop. Pan/tilt
 * stallGuard stays off (magnetic sensors are the origin).
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
    int rx_len;            /* raw bytes seen on the GCONF read (0 = RX silent) */
    char rx_hex[48];       /* first raw RX bytes, space-separated hex */
} tmc_diag_t;

typedef struct {
    int tx_len;
    int rx_len;
    char rx_hex[48];
} tmc_loopback_t;

bool tmc_driver_init(void);
bool tmc_driver_is_ready(void);
bool tmc_driver_uart_installed(void);

bool tmc_driver_read_sg_result(uint8_t axis, uint16_t *sg_result);
bool tmc_driver_read_sg_tstep(uint8_t axis, uint16_t *sg_result, uint32_t *tstep);
bool tmc_driver_diagnose_axis(uint8_t axis, tmc_diag_t *out);
bool tmc_driver_bus_echo(tmc_loopback_t *out);
bool tmc_driver_reconfigure(void);

/** RUN currents (motion) vs extra-low standstill after long idle. */
void tmc_driver_set_standby(bool standby);

#endif // TMC_DRIVER_H
