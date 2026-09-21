/**
 * @file zoom_cal.c
 * @brief Persist zoom stallGuard bands and derived soft travel limits
 */

#include "zoom_cal.h"
#include "stepper_limits.h"

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <stdio.h>
#include <string.h>

static const char *TAG = "zoom_cal";
static const char *NVS_NS = "zoom_cal";
static const char *NVS_KEY = "zcal";
#define ZOOM_CAL_MAGIC 0x5A43414Cu

static zoom_cal_t s_cal;

static void cal_zero(void)
{
    memset(&s_cal, 0, sizeof(s_cal));
    s_cal.magic = ZOOM_CAL_MAGIC;
    s_cal.sg_stall_max = HOME_ZOOM_SG_STALL_MAX;
    s_cal.soft_min = 0;
    s_cal.soft_max = MAX_ZOOM_RANGE_STEPS;
    s_cal.pwm_thresh = HOME_ZOOM_PWM_THRESH;
}

static bool nvs_write(const zoom_cal_t *cal)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return false;
    }
    esp_err_t err = nvs_set_blob(h, NVS_KEY, cal, sizeof(*cal));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err == ESP_OK;
}

void zoom_cal_init(void)
{
    cal_zero();

    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    zoom_cal_t loaded;
    size_t sz = 0;
    esp_err_t peek = nvs_get_blob(h, NVS_KEY, NULL, &sz);
    if (peek == ESP_ERR_NVS_NOT_FOUND || sz == 0 || sz > sizeof(loaded)) {
        nvs_close(h);
        return;
    }
    memset(&loaded, 0, sizeof(loaded));
    esp_err_t err = nvs_get_blob(h, NVS_KEY, &loaded, &sz);
    nvs_close(h);
    if (err != ESP_OK || loaded.magic != ZOOM_CAL_MAGIC) {
        return;
    }
    s_cal = loaded;
    zoom_cal_recompute();
    if (loaded.valid) {
        s_cal.valid = 1;
    }
    ESP_LOGI(TAG, "loaded valid=%u sg_usable=%u pwm_thresh=%u span=%ld soft=%ld..%ld stall_max=%u",
             (unsigned)s_cal.valid, (unsigned)s_cal.sg_usable,
             (unsigned)s_cal.pwm_thresh,
             (long)s_cal.span_steps, (long)s_cal.soft_min, (long)s_cal.soft_max,
             (unsigned)s_cal.sg_stall_max);
}

const zoom_cal_t *zoom_cal_get(void)
{
    return &s_cal;
}

void zoom_cal_recompute(void)
{
    s_cal.sg_usable = 0;
    s_cal.sg_stall_max = HOME_ZOOM_SG_STALL_MAX;
    s_cal.pwm_usable = 0;
    s_cal.pwm_thresh = HOME_ZOOM_PWM_THRESH;
    s_cal.span_steps = 0;
    s_cal.soft_min = 0;
    s_cal.soft_max = MAX_ZOOM_RANGE_STEPS;

    if (s_cal.free_run.captured) {
        uint8_t free_hi = s_cal.free_run.pwm_max;
        s_cal.pwm_thresh = (uint8_t)(free_hi + HOME_ZOOM_CAL_PWM_GAP);
        s_cal.pwm_usable = 1;
    }

    if (!s_cal.wide.captured || !s_cal.tele.captured) {
        return;
    }

    int32_t lo = s_cal.wide_pos;
    int32_t hi = s_cal.tele_pos;
    if (hi < lo) {
        int32_t tmp = lo;
        lo = hi;
        hi = tmp;
    }
    s_cal.span_steps = hi - lo;
    if (s_cal.span_steps < HOME_ZOOM_CAL_MIN_RANGE) {
        return;
    }

    /* Preview and saved limits are in the post-Save frame (wide = 0). */
    s_cal.soft_min = HOME_ZOOM_CAL_MARGIN;
    s_cal.soft_max = s_cal.span_steps - HOME_ZOOM_CAL_MARGIN;
    if (s_cal.soft_max <= s_cal.soft_min + 50) {
        s_cal.soft_min = 0;
        s_cal.soft_max = s_cal.span_steps;
    }

    if (!s_cal.free_run.captured) {
        return;
    }

    uint16_t end_max = s_cal.wide.sg_max;
    if (s_cal.tele.sg_max > end_max) {
        end_max = s_cal.tele.sg_max;
    }
    int gap = (int)s_cal.free_run.sg_min - (int)end_max;
    if (gap >= HOME_ZOOM_CAL_SG_GAP) {
        s_cal.sg_usable = 1;
        s_cal.sg_stall_max = (uint16_t)(end_max + gap / 2);
    }

    uint8_t free_hi = s_cal.free_run.pwm_max;
    /* Home to the wide rubber, not the midpoint of free vs ends (that tripped in air). */
    uint8_t wide_lo = s_cal.wide.pwm_min;
    if (s_cal.wide.captured && wide_lo > free_hi + 1u) {
        s_cal.pwm_thresh = wide_lo;
        s_cal.pwm_usable = 1;
    } else {
        uint8_t end_lo = wide_lo;
        if (s_cal.tele.captured && s_cal.tele.pwm_min < end_lo) {
            end_lo = s_cal.tele.pwm_min;
        }
        if (end_lo > free_hi + 1u) {
            s_cal.pwm_thresh = end_lo;
            s_cal.pwm_usable = 1;
        }
    }
}

void zoom_cal_set_band(zoom_cal_phase_t phase, const zoom_cal_band_t *band, int32_t position)
{
    if (band == NULL) {
        return;
    }
    switch (phase) {
    case ZOOM_CAL_FREE:
        s_cal.free_run = *band;
        break;
    case ZOOM_CAL_WIDE:
        s_cal.wide = *band;
        s_cal.wide_pos = position;
        break;
    case ZOOM_CAL_TELE:
        s_cal.tele = *band;
        s_cal.tele_pos = position;
        break;
    default:
        return;
    }
    s_cal.valid = 0;
    zoom_cal_recompute();
}

bool zoom_cal_save(char *err, int err_sz)
{
    zoom_cal_recompute();
    if (!s_cal.free_run.captured || !s_cal.wide.captured || !s_cal.tele.captured) {
        if (err && err_sz > 0) {
            snprintf(err, (size_t)err_sz, "Capture free-run, wide, and tele first");
        }
        return false;
    }
    if (s_cal.span_steps < HOME_ZOOM_CAL_MIN_RANGE) {
        if (err && err_sz > 0) {
            snprintf(err, (size_t)err_sz,
                     "Ends are only %ld steps apart (need %d)",
                     (long)s_cal.span_steps, HOME_ZOOM_CAL_MIN_RANGE);
        }
        return false;
    }
    s_cal.magic = ZOOM_CAL_MAGIC;
    s_cal.valid = 1;
    if (!nvs_write(&s_cal)) {
        s_cal.valid = 0;
        if (err && err_sz > 0) {
            snprintf(err, (size_t)err_sz, "NVS write failed");
        }
        return false;
    }
    ESP_LOGI(TAG, "saved span=%ld soft=%ld..%ld sg_usable=%u pwm_thresh=%u stall_max=%u",
             (long)s_cal.span_steps, (long)s_cal.soft_min, (long)s_cal.soft_max,
             (unsigned)s_cal.sg_usable, (unsigned)s_cal.pwm_thresh,
             (unsigned)s_cal.sg_stall_max);
    return true;
}

int32_t zoom_cal_rebase_wide(void)
{
    int32_t lo = s_cal.wide_pos;
    if (s_cal.tele_pos < lo) {
        lo = s_cal.tele_pos;
    }
    if (lo != 0) {
        s_cal.wide_pos -= lo;
        s_cal.tele_pos -= lo;
        zoom_cal_recompute();
    }
    return lo;
}

uint8_t zoom_cal_pwm_thresh(void)
{
    /* Prefer the measured wide rubber, even if NVS still has the old midpoint. */
    if (s_cal.wide.captured && s_cal.free_run.captured &&
        s_cal.wide.pwm_min > s_cal.free_run.pwm_max + 1u &&
        s_cal.wide.pwm_min >= 40u && s_cal.wide.pwm_min <= 200u) {
        return s_cal.wide.pwm_min;
    }
    if (s_cal.pwm_thresh >= 40u && s_cal.pwm_thresh <= 200u) {
        return s_cal.pwm_thresh;
    }
    return HOME_ZOOM_PWM_THRESH;
}

bool zoom_cal_clear(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, NVS_KEY);
        nvs_commit(h);
        nvs_close(h);
    }
    cal_zero();
    return true;
}
