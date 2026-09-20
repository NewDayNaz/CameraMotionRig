/**
 * @file zoom_cal.h
 * @brief NVS zoom calibration: free-run vs wide/tele stallGuard bands + soft range
 */

#ifndef ZOOM_CAL_H
#define ZOOM_CAL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t sg_min;
    uint16_t sg_max;
    uint16_t sg_avg_x10;
    uint8_t pwm_min;
    uint8_t pwm_max;
    uint8_t samples_ok;
    uint8_t captured;
} zoom_cal_band_t;

typedef struct {
    uint32_t magic;
    uint8_t valid;
    uint8_t sg_usable;
    uint16_t sg_stall_max;
    int32_t wide_pos;
    int32_t tele_pos;
    int32_t span_steps;
    int32_t soft_min;
    int32_t soft_max;
    zoom_cal_band_t free_run;
    zoom_cal_band_t wide;
    zoom_cal_band_t tele;
    uint8_t pwm_usable;
    uint8_t pwm_thresh;
} zoom_cal_t;

typedef enum {
    ZOOM_CAL_FREE = 0,
    ZOOM_CAL_WIDE,
    ZOOM_CAL_TELE,
} zoom_cal_phase_t;

void zoom_cal_init(void);
const zoom_cal_t *zoom_cal_get(void);

void zoom_cal_set_band(zoom_cal_phase_t phase, const zoom_cal_band_t *band, int32_t position);
void zoom_cal_recompute(void);

bool zoom_cal_save(char *err, int err_sz);
bool zoom_cal_clear(void);
/** Shift stored wide/tele so the wide end is 0. Returns the subtracted origin. */
int32_t zoom_cal_rebase_wide(void);
/** PWM_SCALE_SUM trip for homing / auto-mark (calibrated, else default 81). */
uint8_t zoom_cal_pwm_thresh(void);

#endif /* ZOOM_CAL_H */
