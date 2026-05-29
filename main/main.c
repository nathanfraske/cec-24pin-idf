/*
 * CEC 24-pin Module Firmware - ESP-IDF port (prototype v2 sensor block)
 *
 * Brings up the I2C bus and the four INA226 power monitors (12V @0x40,
 * 5V @0x41, 3V3 @0x44, 5VSB @0x45), reading bus voltage and software-
 * computed current (shunt uV / R_shunt) from each. Samples at 50 Hz,
 * runs each channel through a fast EMA, feeds the state classifier and
 * detection layers, and emits TelePlot series at 10 Hz with a 1 Hz INFO
 * summary line.
 *
 * Burst capture reconfigures the 3 main-rail INA226s to fast mode and
 * captures current at 1 kHz (voltage decimated to ~100 Hz) for the HS
 * window.
 *
 * NTC temperature (ADC1_CH6) and CAN (TWAI, RX moved to GPIO15) arrive
 * with the shared daughterboard and are not wired in this build.
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_psram.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"

#include "ina226.h"
/* cec_adc / thermistor remain in the cec_sensors component for the NTC
 * channel that arrives with the daughterboard; not used yet in v2. */
#include "cec_filters.h"
#include "cec_state.h"
#include "cec_layer1.h"
#include "cec_layer2.h"
#include "cec_layer3.h"
#include "cec_swing.h"
#include "cec_nvs.h"
#include "cec_capture.h"
#include "cec_cli.h"
#include "cec_teleplot.h"

static const char *TAG = "cec_main";

/* I2C bus pins, matching v0.5.9 wiring on the Lonely Binary ESP32-S3 N16R8 */
#define I2C_PIN_SDA       8
#define I2C_PIN_SCL       9
#define I2C_PORT_NUM      I2C_NUM_0

/* v2 sensor block: four INA226 monitors share I2C0 (SDA 8 / SCL 9).
 * Firmware keys off I2C address; physical slot is irrelevant (the modules
 * were rearranged on the board). Address->rail->shunt is authoritative.
 *
 *   addr  rail    R_shunt   full-scale   (CAL computes to 2048 for all)
 *   0x40  +12V    0.002 R   40.96 A
 *   0x41  +3.3V   0.025 R    3.28 A      (was 5V in the v2 spec; address
 *                                         swapped here to match how this
 *                                         board was physically jumpered)
 *   0x44  +5V     0.010 R    8.19 A      (was 3.3V in the v2 spec; swap)
 *   0x45  +5VSB   0.025 R    3.28 A      (moved from 0x40 in v1)
 *
 * Current is read in software (shunt uV / R_shunt) so the non-standard
 * shunts need no per-LSB bookkeeping. Voltage is the INA226 bus register
 * (1.25 mV LSB) at the PSU side of each Kelvin shunt. Trims start at 1.0;
 * adjust per-rail against a meter if bench calibration shows drift. */
#define INA226_ADDR_12V      0x40
#define INA226_ADDR_3V3      0x41   /* board-specific: 3.3V module is at 0x41 */
#define INA226_ADDR_5V       0x44   /* board-specific: 5V module is at 0x44 */
#define INA226_ADDR_5VSB     0x45

#define INA226_SHUNT_12V     0.002f
#define INA226_SHUNT_5V      0.010f
#define INA226_SHUNT_3V3     0.025f
#define INA226_SHUNT_5VSB    0.025f

#define INA226_IMAX_12V      40.96f
#define INA226_IMAX_5V        8.19f
#define INA226_IMAX_3V3       3.28f
#define INA226_IMAX_5VSB      3.28f

/* Per-device I2C clock. 400 kHz per the v2 spec; drop to 100000 if the
 * four parallel breakout pull-ups make the bus marginal (spec section 6). */
#define INA226_SCL_HZ        400000

/* NTC thermistor lives on ADC1_CH6 (GPIO7) on the shared daughterboard,
 * which is not built yet. The ADC subsystem stays uninitialized until it
 * lands; temperature reads as unavailable until then. v2 has no other ADC
 * sensor - rail voltage now comes from the INA226 bus register, so the
 * old resistor-divider taps are gone. */

/* Loop cadence. Sample at 50 Hz to match v0.5.9; emit TelePlot at 10 Hz
 * (every 5th iteration); log an INFO summary at 1 Hz (every 50th). */
#define SAMPLE_PERIOD_MS  20
#define TELEPLOT_DIVIDER  5
#define LOG_DIVIDER       50

/* EMA smoothing. At 50 Hz, alpha=0.02 gives a ~1 s time constant, matching
 * the v0.5.9 EMA_ALPHA_FAST. */
#define EMA_ALPHA_FAST    0.02f

/* I2C bus handle (shared across components later) */
static i2c_master_bus_handle_t s_i2c_bus = NULL;

/* INA226 instances, one per rail. All four share the I2C0 bus. The 3
 * main rails (12V/5V/3V3) are reconfigured to fast mode during an HS
 * burst; 5VSB stays in steady mode. */
static ina226_handle_t s_ina226_12v  = NULL;
static ina226_handle_t s_ina226_5v   = NULL;
static ina226_handle_t s_ina226_3v3  = NULL;
static ina226_handle_t s_ina226_5vsb = NULL;

/* Filter state */
static ema_t s_v_5vsb_ema, s_i_5vsb_ema;
static ema_t s_v_12v_ema, s_v_5v_ema, s_v_3v3_ema;
static ema_t s_i_12v_ema, s_i_5v_ema, s_i_3v3_ema;
static ema_t s_temp_ema;

/* State machine */
static cec_state_t s_state = CEC_STATE_OFF;
static int64_t s_state_entered_us = 0;

/* Layer 1 static-threshold detectors per rail. Bands carry forward from
 * v0.5.9: 5%/10% on main rails, 10%/20% on 5VSB. */
#define L1_CRIT_CONSECUTIVE 3

/* Mask Layer 1 for this long after every state transition. Covers the
 * PSU inrush (most visibly on 5VSB OFF -> STANDBY ramp, where the rail
 * passes through 1..5 V on its way up and would otherwise sustain
 * CRITICAL for the consecutive-sample window). 500 ms is loose enough
 * for typical PSU ramp times and tight enough not to miss real faults. */
#define LAYER1_SETTLE_US  (500LL * 1000)

static cec_layer1_detector_t s_l1_12v, s_l1_5v, s_l1_3v3, s_l1_5vsb;
static cec_severity_t s_last_sev_12v  = CEC_SEV_NONE;
static cec_severity_t s_last_sev_5v   = CEC_SEV_NONE;
static cec_severity_t s_last_sev_3v3  = CEC_SEV_NONE;
static cec_severity_t s_last_sev_5vsb = CEC_SEV_NONE;

/* Layer 2 adaptive transient detectors. Per-rail min-thresholds and
 * k_sigma carry forward from v0.5.9. The fire condition is
 * |instant - ema| > max(min, k*std) for 3 consecutive samples. */
#define LAYER2_K_SIGMA       5.0f
#define LAYER2_CONSECUTIVE   3
#define L2_MIN_V_12V         0.50f
#define L2_MIN_V_5V          0.20f
#define L2_MIN_V_3V3         0.15f
#define L2_MIN_V_5VSB        0.30f
#define L2_MIN_I_12V         1.00f
#define L2_MIN_I_5V          0.50f
#define L2_MIN_I_3V3         0.30f
static cec_layer2_detector_t s_l2_v_12v, s_l2_v_5v, s_l2_v_3v3, s_l2_v_5vsb;
static cec_layer2_detector_t s_l2_i_12v, s_l2_i_5v, s_l2_i_3v3;

/* Layer 3 per-(state, rail) running profile. Z-score threshold 4.0
 * (v0.5.9 default). Adapt rate 0.0005 gives an effective window of
 * roughly 2000 samples (40 s at 50 Hz) once the profile is warm. */
#define LAYER3_Z_THRESHOLD   4.0f
#define PROFILE_ADAPT_RATE   0.0005f
typedef enum {
    PROF_V_12V = 0, PROF_V_5V,  PROF_V_3V3,  PROF_V_5VSB,
    PROF_I_12V,     PROF_I_5V,  PROF_I_3V3,  PROF_I_5VSB,
    PROF_TEMP,      PROF_COUNT
} prof_idx_t;
static cec_rail_profile_t s_profiles[CEC_STATE_COUNT][PROF_COUNT];
static bool s_z_above_last = false;

/* Power swing: 250-sample rolling window (~5 s at 50 Hz). Adaptive
 * threshold = max(8 W, 25% * window_mean), 2 consecutive bad samples
 * (~40 ms) before firing. From v0.5.9. */
#define POWER_SWING_WINDOW_SIZE        250
#define POWER_SWING_CONSECUTIVE        2
#define POWER_SWING_MIN_THRESHOLD_W    8.0f
#define POWER_SWING_FRACTION           0.25f
static cec_swing_detector_t s_power_swing;
static float s_power_swing_buf[POWER_SWING_WINDOW_SIZE];

/* Current swing: per-rail 250-sample rolling window with fixed
 * thresholds. 3 consecutive bad samples (~60 ms) before firing. From
 * v0.5.9. Only one rail fires per iteration (priority 12V > 5V > 3V3). */
#define CURRENT_SWING_WINDOW_SIZE      250
#define CURRENT_SWING_CONSECUTIVE      3
#define CURRENT_SWING_THRESH_I_12V     0.30f
#define CURRENT_SWING_THRESH_I_5V      0.50f
#define CURRENT_SWING_THRESH_I_3V3     0.30f
static cec_swing_detector_t s_i_swing_12v, s_i_swing_5v, s_i_swing_3v3;
static float s_i_swing_12v_buf[CURRENT_SWING_WINDOW_SIZE];
static float s_i_swing_5v_buf [CURRENT_SWING_WINDOW_SIZE];
static float s_i_swing_3v3_buf[CURRENT_SWING_WINDOW_SIZE];

/* NVS persistence for Layer 3 profiles. The magic prefix lets a future
 * firmware revision reject an old blob cleanly if the layout changes. */
#define NVS_PROFILES_KEY     "profiles"
#define NVS_PROFILES_MAGIC   0xCEC30001U
#define NVS_SETTINGS_KEY     "settings"
#define NVS_SETTINGS_MAGIC   0xCEC50001U
#define NVS_SAVE_INTERVAL_US (5LL * 60 * 1000 * 1000)   /* 5 minutes */
static bool    s_profiles_dirty = false;
static int64_t s_last_nvs_save_us = 0;

/* Shutdown detection: 1-second rate-of-change window on v_12v_ema.
 * Triggers when 12V is dropping faster than 0.5 V/s from a nominal-ish
 * starting point (> 5 V) — i.e. the PSU is in the middle of going down.
 * Fires CEC_TRIG_SHUTDOWN (bypasses cooldown so a real shutdown still
 * captures even if a recent burst is in cooldown) and asserts a 30 s
 * mute window during which all detector layers go quiet so we don't
 * spam triggers on collapsing rails. The mute clears either by timeout
 * or by the state classifier landing on OFF or STANDBY, whichever
 * comes first.
 *
 * The 5VSB-defined STANDBY in cec_state_classify is what makes this
 * clean: STANDBY now explicitly means "main rails off, 5VSB up",
 * which is the canonical post-shutdown stable state, so reaching it
 * is a positive signal that the shutdown has completed. */
#define V_12V_RATE_HISTORY_SIZE        50           /* 1 s at 50 Hz */
#define V_12V_SHUTDOWN_RATE_THRESHOLD  (-0.5f)      /* V/s, negative */
#define V_12V_SHUTDOWN_MIN_ARMED_V     5.0f         /* don't trigger if 12V was already low */
#define SHUTDOWN_MUTE_DURATION_US      (30LL * 1000 * 1000)
static float   s_v_12v_history[V_12V_RATE_HISTORY_SIZE];
static size_t  s_v_12v_hist_idx = 0;
static size_t  s_v_12v_hist_count = 0;
static bool    s_shutting_down = false;
static int64_t s_shutdown_start_us = 0;

/* Runtime-toggleable layer enables, persisted to NVS so they survive
 * reboots. Defaults are all-on. Toggle via the serial CLI. */
typedef struct {
    bool layer1;
    bool layer2;
    bool layer3;
    bool swing_power;
    bool swing_current;
} cec_settings_t;
static cec_settings_t s_settings = {
    .layer1 = true, .layer2 = true, .layer3 = true,
    .swing_power = true, .swing_current = true,
};
static bool s_settings_dirty = false;

static void init_i2c_bus(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT_NUM,
        .scl_io_num = I2C_PIN_SCL,
        .sda_io_num = I2C_PIN_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_i2c_bus));
    ESP_LOGI(TAG, "I2C bus: SDA=GPIO%d, SCL=GPIO%d", I2C_PIN_SDA, I2C_PIN_SCL);
}

/* Probe the whole 7-bit address space and log what ACKs. Run once at
 * boot, before INA226 bring-up, so a dead/miswired bus is obvious and we
 * can tell "nothing responds" (power/wiring/pull-ups) from "responds at
 * unexpected addresses" (jumper error). Recommended by the v2 spec's
 * "scan before trusting data" note. */
static void i2c_bus_scan(void)
{
    int found = 0;
    ESP_LOGI(TAG, "I2C scan (expecting 0x40/0x41/0x44/0x45):");
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_master_probe(s_i2c_bus, addr, 50) == ESP_OK) {
            ESP_LOGI(TAG, "  found device @ 0x%02X", addr);
            found++;
        }
    }
    if (found == 0) {
        ESP_LOGW(TAG, "  no I2C devices responded - check INA226 power (VS pin), "
                      "SDA/SCL wiring on GPIO%d/%d, and bus pull-ups",
                 I2C_PIN_SDA, I2C_PIN_SCL);
    } else {
        ESP_LOGI(TAG, "  %d device(s) responded", found);
    }
}

static esp_err_t init_one_ina226(uint8_t addr, float shunt, float imax,
                                 float current_trim, ina226_handle_t *out)
{
    ina226_config_t cfg = INA226_CONFIG_DEFAULT();
    cfg.bus_handle    = s_i2c_bus;
    cfg.i2c_addr      = addr;
    cfg.shunt_ohms    = shunt;
    cfg.max_current_a = imax;             /* with the v2 shunts, CAL -> 2048 */
    cfg.config_value  = INA226_CONFIG_STEADY;
    cfg.scl_speed_hz  = INA226_SCL_HZ;
    cfg.voltage_trim  = 1.0f;
    cfg.current_trim  = current_trim;
    return ina226_create(&cfg, out);
}

/* All four shunt terminals on this board are wired backwards relative to
 * the INA226 IN+/IN- convention, so every rail gets current_trim = -1.0
 * to flip the sign at the driver - the inversion is invisible to every
 * downstream consumer (EMA, layers, swings, p_total, burst capture). On a
 * future board where a rail is wired correctly, set its row back to +1.0. */
#define INA226_ITRIM_12V    (-1.0f)
#define INA226_ITRIM_5V     (-1.0f)
#define INA226_ITRIM_3V3    (-1.0f)
#define INA226_ITRIM_5VSB   (-1.0f)

/* Bring up all four INA226 monitors. Returns the count that enumerated
 * so the caller can warn on a partial bus. A missing address is almost
 * always an address-jumper error on a module, not a firmware bug. */
static int init_ina226_all(void)
{
    int ok = 0;
    struct { uint8_t addr; float shunt; float imax; float itrim;
             ina226_handle_t *out; const char *name; } rails[] = {
        { INA226_ADDR_12V,  INA226_SHUNT_12V,  INA226_IMAX_12V,  INA226_ITRIM_12V,  &s_ina226_12v,  "12V"  },
        { INA226_ADDR_5V,   INA226_SHUNT_5V,   INA226_IMAX_5V,   INA226_ITRIM_5V,   &s_ina226_5v,   "5V"   },
        { INA226_ADDR_3V3,  INA226_SHUNT_3V3,  INA226_IMAX_3V3,  INA226_ITRIM_3V3,  &s_ina226_3v3,  "3V3"  },
        { INA226_ADDR_5VSB, INA226_SHUNT_5VSB, INA226_IMAX_5VSB, INA226_ITRIM_5VSB, &s_ina226_5vsb, "5VSB" },
    };
    for (size_t i = 0; i < sizeof(rails) / sizeof(rails[0]); i++) {
        esp_err_t err = init_one_ina226(rails[i].addr, rails[i].shunt,
                                        rails[i].imax, rails[i].itrim,
                                        rails[i].out);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "INA226 %s @ 0x%02X ready (R_shunt=%.3f, itrim=%+.1f)",
                     rails[i].name, rails[i].addr, rails[i].shunt, rails[i].itrim);
            ok++;
        } else {
            ESP_LOGE(TAG, "INA226 %s @ 0x%02X init failed: %s",
                     rails[i].name, rails[i].addr, esp_err_to_name(err));
        }
    }
    return ok;
}

static void init_layer1(void)
{
    static const cec_rail_spec_t SPEC_12V  = { 12.0f, 0.05f, 0.10f };
    static const cec_rail_spec_t SPEC_5V   = {  5.0f, 0.05f, 0.10f };
    static const cec_rail_spec_t SPEC_3V3  = {  3.3f, 0.05f, 0.10f };
    static const cec_rail_spec_t SPEC_5VSB = {  5.0f, 0.10f, 0.20f };
    cec_layer1_init(&s_l1_12v,  &SPEC_12V,  L1_CRIT_CONSECUTIVE);
    cec_layer1_init(&s_l1_5v,   &SPEC_5V,   L1_CRIT_CONSECUTIVE);
    cec_layer1_init(&s_l1_3v3,  &SPEC_3V3,  L1_CRIT_CONSECUTIVE);
    cec_layer1_init(&s_l1_5vsb, &SPEC_5VSB, L1_CRIT_CONSECUTIVE);
}

static void init_layer2(void)
{
    cec_layer2_init(&s_l2_v_12v,  L2_MIN_V_12V,  LAYER2_K_SIGMA, LAYER2_CONSECUTIVE);
    cec_layer2_init(&s_l2_v_5v,   L2_MIN_V_5V,   LAYER2_K_SIGMA, LAYER2_CONSECUTIVE);
    cec_layer2_init(&s_l2_v_3v3,  L2_MIN_V_3V3,  LAYER2_K_SIGMA, LAYER2_CONSECUTIVE);
    cec_layer2_init(&s_l2_v_5vsb, L2_MIN_V_5VSB, LAYER2_K_SIGMA, LAYER2_CONSECUTIVE);
    cec_layer2_init(&s_l2_i_12v,  L2_MIN_I_12V,  LAYER2_K_SIGMA, LAYER2_CONSECUTIVE);
    cec_layer2_init(&s_l2_i_5v,   L2_MIN_I_5V,   LAYER2_K_SIGMA, LAYER2_CONSECUTIVE);
    cec_layer2_init(&s_l2_i_3v3,  L2_MIN_I_3V3,  LAYER2_K_SIGMA, LAYER2_CONSECUTIVE);
}

static void init_layer3_profiles(void)
{
    for (int s = 0; s < CEC_STATE_COUNT; s++) {
        for (int r = 0; r < PROF_COUNT; r++) {
            cec_rail_profile_init(&s_profiles[s][r]);
        }
    }
}

/* Clear Layer 2 consecutive-fire counters on a "state up" transition so
 * a pending fire from STANDBY ramp values doesn't survive into IDLE. */
static void reset_layer2_counters(void)
{
    cec_layer2_reset(&s_l2_v_12v);
    cec_layer2_reset(&s_l2_v_5v);
    cec_layer2_reset(&s_l2_v_3v3);
    cec_layer2_reset(&s_l2_v_5vsb);
    cec_layer2_reset(&s_l2_i_12v);
    cec_layer2_reset(&s_l2_i_5v);
    cec_layer2_reset(&s_l2_i_3v3);
}

static void init_swing_detectors(void)
{
    cec_swing_detector_init(&s_power_swing, s_power_swing_buf,
                            POWER_SWING_WINDOW_SIZE, POWER_SWING_CONSECUTIVE);
    cec_swing_detector_init(&s_i_swing_12v, s_i_swing_12v_buf,
                            CURRENT_SWING_WINDOW_SIZE, CURRENT_SWING_CONSECUTIVE);
    cec_swing_detector_init(&s_i_swing_5v,  s_i_swing_5v_buf,
                            CURRENT_SWING_WINDOW_SIZE, CURRENT_SWING_CONSECUTIVE);
    cec_swing_detector_init(&s_i_swing_3v3, s_i_swing_3v3_buf,
                            CURRENT_SWING_WINDOW_SIZE, CURRENT_SWING_CONSECUTIVE);
}

/* On state-up the rails go from ~0 V to nominal in milliseconds; the
 * swing windows had been collecting near-zero samples and the new
 * "huge swing" would otherwise fire on the very first IDLE sample.
 * Empty the windows so they re-fill cleanly from in-state values. */
static void reset_swing_windows(void)
{
    cec_swing_detector_reset_empty(&s_power_swing);
    cec_swing_detector_reset_empty(&s_i_swing_12v);
    cec_swing_detector_reset_empty(&s_i_swing_5v);
    cec_swing_detector_reset_empty(&s_i_swing_3v3);
}

/* Same idea for the v_12v rate-of-change history that drives shutdown
 * detection: clear it on state-up so the first 1 s of in-state
 * sampling refills it cleanly instead of being polluted by 0 V values
 * from STANDBY/OFF. */
static void reset_v_12v_history(void)
{
    memset(s_v_12v_history, 0, sizeof(s_v_12v_history));
    s_v_12v_hist_idx = 0;
    s_v_12v_hist_count = 0;
}

/* Load profiles from NVS if the stored blob is valid; otherwise leave
 * the (already-zeroed) profile array alone. Logs the outcome so it's
 * obvious from the boot log whether we picked up warm state. */
static void load_profiles_from_nvs(void)
{
    esp_err_t err = cec_nvs_load_blob(NVS_PROFILES_KEY, NVS_PROFILES_MAGIC,
                                      s_profiles, sizeof(s_profiles));
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "NVS: loaded profiles (%u bytes)",
                 (unsigned)sizeof(s_profiles));
    } else if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGI(TAG, "NVS: no saved profiles, starting cold");
    } else if (err == ESP_ERR_INVALID_VERSION || err == ESP_ERR_INVALID_SIZE) {
        ESP_LOGW(TAG, "NVS: stored profiles unusable (%s), clearing and starting cold",
                 esp_err_to_name(err));
        cec_nvs_clear_blob(NVS_PROFILES_KEY);
    } else {
        ESP_LOGW(TAG, "NVS: load failed (%s), profiles will warm from cold",
                 esp_err_to_name(err));
    }
}

static void load_settings_from_nvs(void)
{
    cec_settings_t loaded;
    esp_err_t err = cec_nvs_load_blob(NVS_SETTINGS_KEY, NVS_SETTINGS_MAGIC,
                                      &loaded, sizeof(loaded));
    if (err == ESP_OK) {
        s_settings = loaded;
        ESP_LOGI(TAG, "NVS: loaded settings (L1=%d L2=%d L3=%d sp=%d sc=%d)",
                 s_settings.layer1, s_settings.layer2, s_settings.layer3,
                 s_settings.swing_power, s_settings.swing_current);
    } else if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGI(TAG, "NVS: no saved settings, using defaults (all-on)");
    } else if (err == ESP_ERR_INVALID_VERSION || err == ESP_ERR_INVALID_SIZE) {
        ESP_LOGW(TAG, "NVS: stored settings unusable (%s), clearing",
                 esp_err_to_name(err));
        cec_nvs_clear_blob(NVS_SETTINGS_KEY);
    }
}

static void save_settings_to_nvs(void)
{
    esp_err_t err = cec_nvs_save_blob(NVS_SETTINGS_KEY, NVS_SETTINGS_MAGIC,
                                      &s_settings, sizeof(s_settings));
    if (err == ESP_OK) {
        s_settings_dirty = false;
        ESP_LOGI(TAG, "NVS: saved settings");
    } else {
        ESP_LOGW(TAG, "NVS: settings save failed: %s", esp_err_to_name(err));
    }
}

/* Re-prime EMAs to the next sample on transitions UP into the
 * main-rails-on regime. Without this, the EMAs would carry their
 * pre-transition (near-zero) values into the new state and produce
 * misleading instant-vs-EMA deviations until they catch up. v0.5.9
 * does the same in reset_filters_on_state_up(). */
static void reset_emas_on_state_up(void)
{
    ema_reset(&s_v_5vsb_ema); ema_reset(&s_i_5vsb_ema);
    ema_reset(&s_v_12v_ema);  ema_reset(&s_v_5v_ema);  ema_reset(&s_v_3v3_ema);
    ema_reset(&s_i_12v_ema);  ema_reset(&s_i_5v_ema);  ema_reset(&s_i_3v3_ema);
    ema_reset(&s_temp_ema);
}

typedef struct {
    cec_severity_t sev;
    bool entered_critical;   /* true on the iteration where sev steps to CRITICAL */
} layer1_step_result_t;

/* Update one rail's Layer 1 detector, log on severity transitions, and
 * report whether this is the iteration on which the rail just stepped
 * into CRITICAL — the caller uses that to fire a burst trigger exactly
 * once per fault entry. */
static layer1_step_result_t layer1_step(const char *rail_name,
                                        cec_layer1_detector_t *d,
                                        cec_severity_t *last_sev,
                                        float v_rail)
{
    cec_severity_t prev = *last_sev;
    cec_severity_t sev = cec_layer1_update(d, v_rail);
    if (sev != prev) {
        if (sev != CEC_SEV_NONE) {
            ESP_LOGW(TAG, "L1: %s %s (v=%.3f, nominal=%.2f)",
                     rail_name, cec_severity_name(sev), v_rail, d->spec.nominal);
        } else {
            ESP_LOGI(TAG, "L1: %s recovered (v=%.3f)", rail_name, v_rail);
        }
        *last_sev = sev;
    }
    return (layer1_step_result_t){
        .sev = sev,
        .entered_critical = (prev != CEC_SEV_CRITICAL && sev == CEC_SEV_CRITICAL),
    };
}

/* Burst capture hooks (run on the capture task, Core 1).
 *
 * setup: switch the 3 main-rail INA226s into fast shunt+bus continuous
 *        mode (~280 us/pair, ~3.5 kHz) so the 1 kHz HS loop sees fresh
 *        samples. 5VSB is left in steady mode (not part of HS).
 * teardown: restore steady (16-avg) mode after the capture.
 */
static void hs_setup_fn(void)
{
    if (s_ina226_12v) ina226_set_config(s_ina226_12v, INA226_CONFIG_HS);
    if (s_ina226_5v)  ina226_set_config(s_ina226_5v,  INA226_CONFIG_HS);
    if (s_ina226_3v3) ina226_set_config(s_ina226_3v3, INA226_CONFIG_HS);
}

static void hs_teardown_fn(void)
{
    if (s_ina226_12v) ina226_set_config(s_ina226_12v, INA226_CONFIG_STEADY);
    if (s_ina226_5v)  ina226_set_config(s_ina226_5v,  INA226_CONFIG_STEADY);
    if (s_ina226_3v3) ina226_set_config(s_ina226_3v3, INA226_CONFIG_STEADY);
}

/* HS sample callback at 1 kHz on the capture task. Current (shunt) on the
 * 3 main rails every sample; bus voltage only when want_voltage (~100 Hz)
 * so the I2C budget stays inside 1 ms at 400 kHz. Returns a per-rail
 * success bitmask — the engine carries forward each unset bit separately
 * so one device's NACK can't void healthy rails. */
static uint32_t hs_sample_fn(cec_capture_hs_sample_t *out, bool want_voltage)
{
    uint32_t ok = 0;
    if (ina226_read_current(s_ina226_12v, &out->i_12v) == ESP_OK) ok |= CEC_HS_OK_I_12V;
    if (ina226_read_current(s_ina226_5v,  &out->i_5v)  == ESP_OK) ok |= CEC_HS_OK_I_5V;
    if (ina226_read_current(s_ina226_3v3, &out->i_3v3) == ESP_OK) ok |= CEC_HS_OK_I_3V3;
    if (want_voltage) {
        if (ina226_read_bus_voltage(s_ina226_12v, &out->v_12v) == ESP_OK) ok |= CEC_HS_OK_V_12V;
        if (ina226_read_bus_voltage(s_ina226_5v,  &out->v_5v)  == ESP_OK) ok |= CEC_HS_OK_V_5V;
        if (ina226_read_bus_voltage(s_ina226_3v3, &out->v_3v3) == ESP_OK) ok |= CEC_HS_OK_V_3V3;
    }
    return ok;
}

/* ---------------------------- CLI handlers ---------------------------- */

/* Trigger a manual burst with optional caller-supplied annotation text.
 * Tokens argv[1..argc-1] are space-joined into a single annotation. If
 * no text is supplied the burst still fires with reason=MANUAL but
 * without an annotation line. */
static int cli_cmd_burst(int argc, char **argv)
{
    char text[96];
    text[0] = '\0';
    if (argc >= 2) {
        size_t pos = 0;
        for (int i = 1; i < argc; i++) {
            size_t avail = sizeof(text) - pos - 1;
            if (avail == 0) break;
            if (i > 1) { text[pos++] = ' '; if (pos >= sizeof(text) - 1) break; avail--; }
            size_t n = strlen(argv[i]);
            if (n > avail) n = avail;
            memcpy(text + pos, argv[i], n);
            pos += n;
            text[pos] = '\0';
        }
    }
    esp_err_t err = cec_capture_trigger_with_text(CEC_TRIG_MANUAL,
                                                   text[0] ? text : NULL);
    if (err == ESP_OK) {
        printf("burst triggered (manual%s%s)\n",
               text[0] ? ", annotation: " : "",
               text[0] ? text : "");
        return 0;
    }
    if (err == ESP_ERR_NOT_FINISHED) {
        printf("error: capture already running\n");
    } else if (err == ESP_ERR_INVALID_STATE) {
        printf("error: within cooldown window\n");
    } else {
        printf("error: %s\n", esp_err_to_name(err));
    }
    return 1;
}

static bool *settings_flag_for(const char *name)
{
    if (strcmp(name, "layer1") == 0 || strcmp(name, "l1") == 0)
        return &s_settings.layer1;
    if (strcmp(name, "layer2") == 0 || strcmp(name, "l2") == 0)
        return &s_settings.layer2;
    if (strcmp(name, "layer3") == 0 || strcmp(name, "l3") == 0)
        return &s_settings.layer3;
    if (strcmp(name, "swing_power") == 0 || strcmp(name, "sp") == 0)
        return &s_settings.swing_power;
    if (strcmp(name, "swing_current") == 0 || strcmp(name, "sc") == 0)
        return &s_settings.swing_current;
    return NULL;
}

static int cli_cmd_set(int argc, char **argv)
{
    if (argc != 3) {
        printf("usage: set <layer1|layer2|layer3|swing_power|swing_current> <on|off>\n");
        return 1;
    }
    bool *flag = settings_flag_for(argv[1]);
    if (flag == NULL) {
        printf("error: unknown setting '%s'\n", argv[1]);
        return 1;
    }
    bool new_value;
    if (strcmp(argv[2], "on") == 0 || strcmp(argv[2], "1") == 0)        new_value = true;
    else if (strcmp(argv[2], "off") == 0 || strcmp(argv[2], "0") == 0)  new_value = false;
    else {
        printf("error: value must be 'on' or 'off'\n");
        return 1;
    }
    *flag = new_value;
    s_settings_dirty = true;
    save_settings_to_nvs();
    printf("%s = %s\n", argv[1], new_value ? "on" : "off");
    return 0;
}

static int cli_cmd_status(int argc, char **argv)
{
    bool json = (argc >= 2 && strcmp(argv[1], "json") == 0);
    int64_t now_us = esp_timer_get_time();
    int64_t dwell_ms = (now_us - s_state_entered_us) / 1000;

    if (json) {
        printf("{\"state\":\"%s\",\"dwell_ms\":%lld,",
               cec_state_name(s_state), (long long)dwell_ms);
        printf("\"v\":{\"12v\":%.3f,\"5v\":%.3f,\"3v3\":%.3f,\"5vsb\":%.3f},",
               ema_value(&s_v_12v_ema), ema_value(&s_v_5v_ema),
               ema_value(&s_v_3v3_ema), ema_value(&s_v_5vsb_ema));
        printf("\"i\":{\"12v\":%.3f,\"5v\":%.3f,\"3v3\":%.3f,\"5vsb\":%.4f},",
               ema_value(&s_i_12v_ema), ema_value(&s_i_5v_ema),
               ema_value(&s_i_3v3_ema), ema_value(&s_i_5vsb_ema));
        printf("\"temp_c\":%.2f,", ema_value(&s_temp_ema));
        printf("\"layers\":{\"l1\":%s,\"l2\":%s,\"l3\":%s,\"swing_power\":%s,\"swing_current\":%s},",
               s_settings.layer1 ? "true" : "false",
               s_settings.layer2 ? "true" : "false",
               s_settings.layer3 ? "true" : "false",
               s_settings.swing_power ? "true" : "false",
               s_settings.swing_current ? "true" : "false");
        printf("\"profile_warm\":{");
        for (int s = 0; s < CEC_STATE_COUNT; s++) {
            printf("%s\"%s\":%s",
                   s > 0 ? "," : "",
                   cec_state_name((cec_state_t)s),
                   cec_rail_profile_is_warm(&s_profiles[s][PROF_V_12V]) ? "true" : "false");
        }
        printf("},\"shutting_down\":%s,\"nvs\":{\"profiles_dirty\":%s}}\n",
               s_shutting_down ? "true" : "false",
               s_profiles_dirty ? "true" : "false");
        return 0;
    }

    printf("state:    %s  (dwell %lld ms)%s\n",
           cec_state_name(s_state), (long long)dwell_ms,
           s_shutting_down ? "  [shutdown muted]" : "");
    printf("V:        12=%.3f  5=%.3f  3V3=%.3f  5SB=%.3f\n",
           ema_value(&s_v_12v_ema), ema_value(&s_v_5v_ema),
           ema_value(&s_v_3v3_ema), ema_value(&s_v_5vsb_ema));
    printf("I:        12=%.3f  5=%.3f  3V3=%.3f  5SB=%.4f\n",
           ema_value(&s_i_12v_ema), ema_value(&s_i_5v_ema),
           ema_value(&s_i_3v3_ema), ema_value(&s_i_5vsb_ema));
    printf("temp:     %.2f C\n", ema_value(&s_temp_ema));
    printf("layers:   L1=%s L2=%s L3=%s  swing/power=%s  swing/current=%s\n",
           s_settings.layer1 ? "on" : "off",
           s_settings.layer2 ? "on" : "off",
           s_settings.layer3 ? "on" : "off",
           s_settings.swing_power ? "on" : "off",
           s_settings.swing_current ? "on" : "off");
    printf("profiles: ");
    for (int s = 0; s < CEC_STATE_COUNT; s++) {
        printf("%s=%s  ", cec_state_name((cec_state_t)s),
               cec_rail_profile_is_warm(&s_profiles[s][PROF_V_12V]) ? "warm" : "cold");
    }
    printf("\n");
    printf("nvs:      profiles_dirty=%s\n", s_profiles_dirty ? "yes" : "no");
    return 0;
}

static const cec_cli_command_t s_cli_commands[] = {
    { "burst",  "burst now [reason text...] — fire a manual burst capture",  cli_cmd_burst  },
    { "set",    "set <layer1|layer2|layer3|swing_power|swing_current> <on|off>", cli_cmd_set },
    { "status", "status [json] — current state, EMA readings, layer enables, profile warmth", cli_cmd_status },
};

static void log_hardware_info(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "Chip: %s rev v%d.%d, %d core(s)",
             CONFIG_IDF_TARGET,
             chip_info.revision / 100, chip_info.revision % 100,
             chip_info.cores);

    uint32_t flash_size;
    esp_flash_get_size(NULL, &flash_size);
    ESP_LOGI(TAG, "Flash: %" PRIu32 " MB", flash_size / (1024 * 1024));

    if (esp_psram_is_initialized()) {
        size_t psram_size = esp_psram_get_size();
        ESP_LOGI(TAG, "PSRAM: %u MB", psram_size / (1024 * 1024));
    } else {
        ESP_LOGW(TAG, "PSRAM: not initialized");
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "CEC 24-pin Module Firmware (ESP-IDF port)");
    ESP_LOGI(TAG, "Version: 0.6.0-dev (INA226 bringup)");
    ESP_LOGI(TAG, "===========================================");

    log_hardware_info();

    /* Bring up the dedicated TelePlot transport on UART0/GPIO 43-44
     * (CH340K USB-C). All teleplot_* output goes here; CLI input,
     * ESP_LOG, and command responses stay on the JTAG USB-C. If the
     * UART init fails (cable not plugged in, etc.) the helpers fall
     * back to stdout so capture-tool workflows still work over JTAG. */
    if (cec_telemetry_init_uart() != ESP_OK) {
        ESP_LOGW(TAG, "telemetry UART unavailable; TelePlot will share the JTAG USB-C with logs");
    }

    init_i2c_bus();
    i2c_bus_scan();

    int ina_ok = init_ina226_all();
    if (ina_ok != 4) {
        ESP_LOGW(TAG, "only %d/4 INA226 enumerated - check address jumpers; "
                      "missing rails will read unavailable", ina_ok);
    }

    ema_init(&s_v_5vsb_ema, EMA_ALPHA_FAST);
    ema_init(&s_i_5vsb_ema, EMA_ALPHA_FAST);
    ema_init(&s_v_12v_ema,  EMA_ALPHA_FAST);
    ema_init(&s_v_5v_ema,   EMA_ALPHA_FAST);
    ema_init(&s_v_3v3_ema,  EMA_ALPHA_FAST);
    ema_init(&s_i_12v_ema,  EMA_ALPHA_FAST);
    ema_init(&s_i_5v_ema,   EMA_ALPHA_FAST);
    ema_init(&s_i_3v3_ema,  EMA_ALPHA_FAST);
    ema_init(&s_temp_ema,   EMA_ALPHA_FAST);

    init_layer1();
    init_layer2();
    init_layer3_profiles();
    init_swing_detectors();

    if (cec_nvs_init() == ESP_OK) {
        load_profiles_from_nvs();
        load_settings_from_nvs();
    } else {
        ESP_LOGW(TAG, "NVS init failed; profiles will not persist across boots");
    }

    esp_err_t cli_err = cec_cli_init(s_cli_commands,
                                     sizeof(s_cli_commands) / sizeof(s_cli_commands[0]));
    if (cli_err != ESP_OK) {
        ESP_LOGW(TAG, "cec_cli_init failed: %s — serial commands unavailable",
                 esp_err_to_name(cli_err));
    }

    const cec_capture_config_t cap_cfg = {
        .sample_fn   = hs_sample_fn,
        .setup_fn    = hs_setup_fn,
        .teardown_fn = hs_teardown_fn,
    };
    esp_err_t err = cec_capture_init(&cap_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "cec_capture_init failed: %s", esp_err_to_name(err));
        ESP_LOGW(TAG, "Continuing without burst capture");
    }

    /* Sample at 50 Hz, emit TelePlot at 10 Hz, log an INFO summary at 1 Hz.
     * Read failures don't get spammed per-iteration; the divided cadence
     * makes them visible as gaps in TelePlot and bad values in the summary. */
    ESP_LOGI(TAG, "Entering main loop (sample=50 Hz, teleplot=10 Hz, log=1 Hz)");
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t iter = 0;
    while (1) {
        float v_5vsb = 0.0f, i_5vsb = 0.0f;
        float v_12v = 0.0f, v_5v = 0.0f, v_3v3 = 0.0f;
        float i_12v = 0.0f, i_5v = 0.0f, i_3v3 = 0.0f;
        float temp_c = 0.0f;
        bool ok_5vsb = false, ok_temp = false;
        bool ok_v_12v = false, ok_v_5v = false, ok_v_3v3 = false;
        bool ok_i_12v = false, ok_i_5v = false, ok_i_3v3 = false;

        /* Capture-phase gating. During CAPTURING (HS window): the 3 main
         * rails are in fast mode and the HS task owns I2C, so coast on
         * held EMAs. During DUMPING: the I2C bus is idle - reads, EMAs,
         * and detectors all run normally; only steady-state TelePlot
         * emission stays silent because the capture task owns the UART.
         * Splitting the gate this way recovers ~5 s of monitoring
         * coverage per burst vs. blanking on any cec_capture_is_busy(). */
        bool capturing = cec_capture_is_capturing();
        bool dumping   = cec_capture_is_dumping();
        if (!capturing && s_ina226_12v) {
            ok_v_12v = (ina226_read_bus_voltage(s_ina226_12v, &v_12v) == ESP_OK);
            ok_i_12v = (ina226_read_current(s_ina226_12v, &i_12v) == ESP_OK);
        }
        if (!capturing && s_ina226_5v) {
            ok_v_5v = (ina226_read_bus_voltage(s_ina226_5v, &v_5v) == ESP_OK);
            ok_i_5v = (ina226_read_current(s_ina226_5v, &i_5v) == ESP_OK);
        }
        if (!capturing && s_ina226_3v3) {
            ok_v_3v3 = (ina226_read_bus_voltage(s_ina226_3v3, &v_3v3) == ESP_OK);
            ok_i_3v3 = (ina226_read_current(s_ina226_3v3, &i_3v3) == ESP_OK);
        }
        if (!capturing && s_ina226_5vsb) {
            ok_5vsb = (ina226_read_bus_voltage(s_ina226_5vsb, &v_5vsb) == ESP_OK &&
                       ina226_read_current(s_ina226_5vsb, &i_5vsb) == ESP_OK);
        }
        /* NTC temperature arrives with the daughterboard; not wired in v2. */
        ok_temp = false;

        /* Hold the raw instantaneous value at the last-good EMA on a failed
         * or skipped read. This keeps the raw-vs-EMA pair consistent so an
         * I2C glitch can't feed a full-scale deviation into Layer 2 (which
         * compares instant against EMA). With every rail now on I2C this
         * matters for all of them, not just the old 5VSB. */
        if (!ok_v_12v) v_12v = ema_value(&s_v_12v_ema);
        if (!ok_i_12v) i_12v = ema_value(&s_i_12v_ema);
        if (!ok_v_5v)  v_5v  = ema_value(&s_v_5v_ema);
        if (!ok_i_5v)  i_5v  = ema_value(&s_i_5v_ema);
        if (!ok_v_3v3) v_3v3 = ema_value(&s_v_3v3_ema);
        if (!ok_i_3v3) i_3v3 = ema_value(&s_i_3v3_ema);
        if (!ok_5vsb) { v_5vsb = ema_value(&s_v_5vsb_ema); i_5vsb = ema_value(&s_i_5vsb_ema); }

        /* EMA: on a good read advance the filter; otherwise hold last value. */
        float v_5vsb_ema = ok_5vsb  ? ema_update(&s_v_5vsb_ema, v_5vsb) : ema_value(&s_v_5vsb_ema);
        float i_5vsb_ema = ok_5vsb  ? ema_update(&s_i_5vsb_ema, i_5vsb) : ema_value(&s_i_5vsb_ema);
        float v_12v_ema  = ok_v_12v ? ema_update(&s_v_12v_ema,  v_12v)  : ema_value(&s_v_12v_ema);
        float v_5v_ema   = ok_v_5v  ? ema_update(&s_v_5v_ema,   v_5v)   : ema_value(&s_v_5v_ema);
        float v_3v3_ema  = ok_v_3v3 ? ema_update(&s_v_3v3_ema,  v_3v3)  : ema_value(&s_v_3v3_ema);
        float i_12v_ema  = ok_i_12v ? ema_update(&s_i_12v_ema,  i_12v)  : ema_value(&s_i_12v_ema);
        float i_5v_ema   = ok_i_5v  ? ema_update(&s_i_5v_ema,   i_5v)   : ema_value(&s_i_5v_ema);
        float i_3v3_ema  = ok_i_3v3 ? ema_update(&s_i_3v3_ema,  i_3v3)  : ema_value(&s_i_3v3_ema);
        float temp_ema   = ok_temp  ? ema_update(&s_temp_ema,   temp_c) : ema_value(&s_temp_ema);

        /* Total main-rail power, EMA-smoothed. 5VSB is standby and not
         * included by design (matches v0.5.9). */
        float p_total = (v_12v_ema * i_12v_ema)
                      + (v_5v_ema  * i_5v_ema)
                      + (v_3v3_ema * i_3v3_ema);

        /* Update 12V rate-of-change history (1 s window). The "oldest"
         * sample is whatever sits at the slot we're about to overwrite. */
        float v_12v_oldest = s_v_12v_history[s_v_12v_hist_idx];
        s_v_12v_history[s_v_12v_hist_idx] = v_12v_ema;
        s_v_12v_hist_idx = (s_v_12v_hist_idx + 1) % V_12V_RATE_HISTORY_SIZE;
        if (s_v_12v_hist_count < V_12V_RATE_HISTORY_SIZE) s_v_12v_hist_count++;
        bool rate_valid = (s_v_12v_hist_count >= V_12V_RATE_HISTORY_SIZE);
        float v_12v_rate = rate_valid ? (v_12v_ema - v_12v_oldest) : 0.0f;

        /* Shutdown detection: 12V was nominal-ish and is now falling
         * fast. Fire the burst and assert the mute. The capture path
         * has SHUTDOWN-bypasses-cooldown built in, so this always
         * captures even if a previous burst was recent. */
        /* Arm shutdown detection only from a running state (IDLE/ACTIVE/
         * PEAK). You can't "begin shutting down" from a rail that's
         * already at/near zero — without this guard, 12V decaying through
         * the 5..10.5 V band while already in STANDBY re-asserts
         * s_shutting_down every iteration, and since the mute-clear is a
         * state-transition edge it never fires (no new edge), leaving the
         * mute stuck for the whole STANDBY dwell. Gating arming on a
         * running state kills that re-assert at the source. */
        bool shutdown_armable = (s_state == CEC_STATE_IDLE
                                 || s_state == CEC_STATE_ACTIVE
                                 || s_state == CEC_STATE_PEAK);
        if (rate_valid && !s_shutting_down && shutdown_armable
            && v_12v_ema > V_12V_SHUTDOWN_MIN_ARMED_V
            && v_12v_rate < V_12V_SHUTDOWN_RATE_THRESHOLD) {
            s_shutting_down = true;
            s_shutdown_start_us = esp_timer_get_time();
            ESP_LOGW(TAG, "SHUTDOWN DETECTED (12V dropping %.2f V/s); muting detectors for %ds",
                     v_12v_rate, (int)(SHUTDOWN_MUTE_DURATION_US / 1000000));
            esp_err_t terr = cec_capture_trigger(CEC_TRIG_SHUTDOWN);
            if (terr == ESP_OK) {
                ESP_LOGW(TAG, "burst trigger: SHUTDOWN");
            }
        }
        /* Clear the mute on either (a) timeout, or (b) the system having
         * settled into STANDBY/OFF — checked as a level, not just the
         * transition edge, so a re-assert mid-STANDBY can't leave it
         * stuck. Reaching STANDBY/OFF means the shutdown resolved. */
        if (s_shutting_down) {
            bool timed_out = (esp_timer_get_time() - s_shutdown_start_us) > SHUTDOWN_MUTE_DURATION_US;
            bool rails_down = (s_state == CEC_STATE_STANDBY || s_state == CEC_STATE_OFF);
            if (timed_out || rails_down) {
                s_shutting_down = false;
                ESP_LOGI(TAG, "shutdown mute cleared (%s)",
                         timed_out ? "timeout" : "reached STANDBY/OFF");
            }
        }

        cec_state_t next_state = cec_state_classify(v_12v_ema, v_5vsb_ema, p_total, s_state);
        if (next_state != s_state) {
            int64_t now_us = esp_timer_get_time();
            int64_t dwell_ms = (now_us - s_state_entered_us) / 1000;
            ESP_LOGI(TAG, "state: %s -> %s (dwell=%lld ms, p_total=%.1f W)",
                     cec_state_name(s_state), cec_state_name(next_state),
                     (long long)dwell_ms, p_total);
            /* Transition UP from below IDLE: rails were 0 V or ramping,
             * the EMAs / Layer-2 variance estimators carry pre-transition
             * state, and Layer-2 counters might be mid-accumulating from
             * the ramp. Reset them so the new state starts clean. Matches
             * v0.5.9 reset_filters_on_state_up. */
            if (next_state >= CEC_STATE_IDLE && s_state < CEC_STATE_IDLE) {
                reset_emas_on_state_up();
                reset_layer2_counters();
                reset_swing_windows();
                reset_v_12v_history();
                s_z_above_last = false;
            }
            /* 5VSB powers up at OFF -> {STANDBY,IDLE+}, not at the main-
             * rails-up transition. Re-prime its EMA pair so the next
             * sample jumps from 0 V to the actual ~5 V instead of crawling
             * up at alpha=0.02 (~1 s tau) for seconds and tripping Layer 1
             * CRITICAL halfway through. The full reset above already
             * covers 5VSB on STANDBY -> IDLE, so this only matters for
             * the OFF -> STANDBY edge but is also harmlessly redundant on
             * a direct OFF -> IDLE. */
            if (s_state == CEC_STATE_OFF && next_state != CEC_STATE_OFF) {
                ema_reset(&s_v_5vsb_ema);
                ema_reset(&s_i_5vsb_ema);
                cec_layer1_reset(&s_l1_5vsb);
                s_last_sev_5vsb = CEC_SEV_NONE;
            }
            /* (Shutdown-mute clear is handled by the level check above,
             * which fires whenever s_state is STANDBY/OFF — not just on
             * this transition edge — so a re-assert mid-STANDBY can't
             * leave it stuck.) */
            s_state = next_state;
            s_state_entered_us = now_us;
        }

        /* Layer 1 gating:
         *   - Main rails (12V/5V/3V3) only check in IDLE/ACTIVE/PEAK,
         *     since OFF/STANDBY has them at 0 V and the ramp passes
         *     through out-of-band values.
         *   - 5VSB checks any time the rail is supposed to be present
         *     (everything except OFF).
         *   - Both wait LAYER1_SETTLE_US after every state transition
         *     so the PSU's inrush ramp doesn't trip a false CRITICAL.
         *     Detectors are reset during the settle window so the
         *     consecutive-sample counter doesn't accumulate from the
         *     ramp.
         */
        bool l1_settled = (esp_timer_get_time() - s_state_entered_us) > LAYER1_SETTLE_US;
        bool main_rails_armed = l1_settled
            && (s_state == CEC_STATE_IDLE || s_state == CEC_STATE_ACTIVE || s_state == CEC_STATE_PEAK);
        bool sb_rail_armed = l1_settled && (s_state != CEC_STATE_OFF);
        /* "active" = state-armed AND runtime-enabled AND not in the
         * shutdown mute window. The settings flag lets the operator
         * silence a layer at runtime; the shutdown mute keeps every
         * detector quiet while the rails are collapsing so the only
         * trigger that fires during a shutdown is CEC_TRIG_SHUTDOWN. */
        bool armed = !s_shutting_down;
        bool l1_main_active = s_settings.layer1 && main_rails_armed && armed;
        bool l1_sb_active   = s_settings.layer1 && sb_rail_armed   && armed;
        bool l2_active      = s_settings.layer2 && main_rails_armed && armed;
        bool l3_active      = s_settings.layer3 && main_rails_armed && armed;
        bool sp_active      = s_settings.swing_power   && main_rails_armed && armed;
        bool sc_active      = s_settings.swing_current && main_rails_armed && armed;

        cec_severity_t sev_12v = CEC_SEV_NONE;
        cec_severity_t sev_5v  = CEC_SEV_NONE;
        cec_severity_t sev_3v3 = CEC_SEV_NONE;
        cec_severity_t sev_5vsb = CEC_SEV_NONE;
        bool any_entered_crit = false;

        if (l1_main_active) {
            layer1_step_result_t r;
            r = layer1_step("12V", &s_l1_12v, &s_last_sev_12v, v_12v_ema);
            sev_12v = r.sev; any_entered_crit |= r.entered_critical;
            r = layer1_step("5V",  &s_l1_5v,  &s_last_sev_5v,  v_5v_ema);
            sev_5v  = r.sev; any_entered_crit |= r.entered_critical;
            r = layer1_step("3V3", &s_l1_3v3, &s_last_sev_3v3, v_3v3_ema);
            sev_3v3 = r.sev; any_entered_crit |= r.entered_critical;
        } else {
            cec_layer1_reset(&s_l1_12v); s_last_sev_12v = CEC_SEV_NONE;
            cec_layer1_reset(&s_l1_5v);  s_last_sev_5v  = CEC_SEV_NONE;
            cec_layer1_reset(&s_l1_3v3); s_last_sev_3v3 = CEC_SEV_NONE;
        }
        if (l1_sb_active) {
            layer1_step_result_t r5sb = layer1_step("5VSB", &s_l1_5vsb, &s_last_sev_5vsb, v_5vsb_ema);
            sev_5vsb = r5sb.sev;
            any_entered_crit |= r5sb.entered_critical;
        } else {
            cec_layer1_reset(&s_l1_5vsb); s_last_sev_5vsb = CEC_SEV_NONE;
        }

        if (any_entered_crit) {
            esp_err_t terr = cec_capture_trigger(CEC_TRIG_STATIC_CRIT);
            if (terr == ESP_OK) {
                ESP_LOGW(TAG, "burst trigger: STATIC_CRIT");
            } else if (terr == ESP_ERR_NOT_FINISHED) {
                ESP_LOGW(TAG, "burst trigger: STATIC_CRIT skipped (capture busy)");
            } else if (terr == ESP_ERR_INVALID_STATE) {
                ESP_LOGW(TAG, "burst trigger: STATIC_CRIT skipped (cooldown)");
            }
        }

        /* Layers 2 and 3 share the rail-state gating with Layer 1's main
         * rails, then layer with their own enable flag so the operator
         * can silence either layer at runtime. */
        bool l2_fired = false;
        if (l2_active) {
            l2_fired |= cec_layer2_update(&s_l2_v_12v,  v_12v,  v_12v_ema);
            l2_fired |= cec_layer2_update(&s_l2_v_5v,   v_5v,   v_5v_ema);
            l2_fired |= cec_layer2_update(&s_l2_v_3v3,  v_3v3,  v_3v3_ema);
            l2_fired |= cec_layer2_update(&s_l2_v_5vsb, v_5vsb, v_5vsb_ema);
            l2_fired |= cec_layer2_update(&s_l2_i_12v,  i_12v,  i_12v_ema);
            l2_fired |= cec_layer2_update(&s_l2_i_5v,   i_5v,   i_5v_ema);
            l2_fired |= cec_layer2_update(&s_l2_i_3v3,  i_3v3,  i_3v3_ema);
        } else {
            reset_layer2_counters();
        }
        if (l2_fired) {
            esp_err_t terr = cec_capture_trigger(CEC_TRIG_TRANSIENT);
            if (terr == ESP_OK) {
                ESP_LOGW(TAG, "burst trigger: TRANSIENT");
            }
            /* NOT_FINISHED / cooldown skips are silent for L2/L3 — they
             * fire often enough that logging each skip would be noisy. */
        }

        /* Layer 3: update the current state's profiles, then compute
         * the max |z| across the six main rails. Single-sample trigger
         * gated on transition (z stayed below, now crossed above)
         * so a sustained anomaly doesn't spam the trigger path. */
        float z_max = 0.0f;
        if (l3_active) {
            cec_rail_profile_t *prof = s_profiles[s_state];
            cec_rail_profile_update(&prof[PROF_V_12V],  v_12v_ema,  PROFILE_ADAPT_RATE);
            cec_rail_profile_update(&prof[PROF_V_5V],   v_5v_ema,   PROFILE_ADAPT_RATE);
            cec_rail_profile_update(&prof[PROF_V_3V3],  v_3v3_ema,  PROFILE_ADAPT_RATE);
            cec_rail_profile_update(&prof[PROF_V_5VSB], v_5vsb_ema, PROFILE_ADAPT_RATE);
            cec_rail_profile_update(&prof[PROF_I_12V],  i_12v_ema,  PROFILE_ADAPT_RATE);
            cec_rail_profile_update(&prof[PROF_I_5V],   i_5v_ema,   PROFILE_ADAPT_RATE);
            cec_rail_profile_update(&prof[PROF_I_3V3],  i_3v3_ema,  PROFILE_ADAPT_RATE);
            cec_rail_profile_update(&prof[PROF_I_5VSB], i_5vsb_ema, PROFILE_ADAPT_RATE);
            cec_rail_profile_update(&prof[PROF_TEMP],   temp_ema,   PROFILE_ADAPT_RATE);
            s_profiles_dirty = true;

            if (cec_rail_profile_is_warm(&prof[PROF_V_12V])) {
                float z;
                z = fabsf(cec_rail_profile_z_score(&prof[PROF_V_12V], v_12v_ema)); if (z > z_max) z_max = z;
                z = fabsf(cec_rail_profile_z_score(&prof[PROF_V_5V],  v_5v_ema));  if (z > z_max) z_max = z;
                z = fabsf(cec_rail_profile_z_score(&prof[PROF_V_3V3], v_3v3_ema)); if (z > z_max) z_max = z;
                z = fabsf(cec_rail_profile_z_score(&prof[PROF_I_12V], i_12v_ema)); if (z > z_max) z_max = z;
                z = fabsf(cec_rail_profile_z_score(&prof[PROF_I_5V],  i_5v_ema));  if (z > z_max) z_max = z;
                z = fabsf(cec_rail_profile_z_score(&prof[PROF_I_3V3], i_3v3_ema)); if (z > z_max) z_max = z;
            }
        }
        bool z_above = (z_max > LAYER3_Z_THRESHOLD);
        if (z_above && !s_z_above_last) {
            esp_err_t terr = cec_capture_trigger(CEC_TRIG_ANOMALY);
            if (terr == ESP_OK) {
                ESP_LOGW(TAG, "burst trigger: ANOMALY (z_max=%.2f)", z_max);
            }
        }
        s_z_above_last = z_above;

        /* Power swing: adaptive threshold against the 5-second window
         * mean. Reset baseline on fire so the post-event behavior gets
         * its own debounce window. */
        float p_window_mean = cec_swing_detector_mean(&s_power_swing);
        float p_swing_thresh = fmaxf(POWER_SWING_MIN_THRESHOLD_W,
                                     POWER_SWING_FRACTION * p_window_mean);
        if (sp_active) {
            if (cec_swing_detector_update(&s_power_swing, p_total, p_swing_thresh)) {
                ESP_LOGW(TAG, "POWER SWING: now=%.1f W, mean=%.1f W, swing=%+.1f W, thr=%.1f W",
                         p_total, p_window_mean, p_total - p_window_mean, p_swing_thresh);
                esp_err_t terr = cec_capture_trigger(CEC_TRIG_POWER_SWING);
                if (terr == ESP_OK) {
                    ESP_LOGW(TAG, "burst trigger: POWER_SWING");
                }
                cec_swing_detector_reset_to(&s_power_swing, p_total);
            }
        } else {
            cec_swing_detector_reset_empty(&s_power_swing);
        }

        /* Per-rail current swing. Fire on the first rail to trip, with
         * 12V > 5V > 3V3 priority so the log message identifies one
         * specific cause. Re-baseline all three windows on fire. */
        if (sc_active) {
            bool f12 = cec_swing_detector_update(&s_i_swing_12v, i_12v_ema, CURRENT_SWING_THRESH_I_12V);
            bool f5  = cec_swing_detector_update(&s_i_swing_5v,  i_5v_ema,  CURRENT_SWING_THRESH_I_5V);
            bool f3v3 = cec_swing_detector_update(&s_i_swing_3v3, i_3v3_ema, CURRENT_SWING_THRESH_I_3V3);
            const char *fired_rail = NULL;
            float fired_val = 0.0f, fired_mean = 0.0f;
            if (f12)       { fired_rail = "12V"; fired_val = i_12v_ema; fired_mean = cec_swing_detector_mean(&s_i_swing_12v); }
            else if (f5)   { fired_rail = "5V";  fired_val = i_5v_ema;  fired_mean = cec_swing_detector_mean(&s_i_swing_5v);  }
            else if (f3v3) { fired_rail = "3V3"; fired_val = i_3v3_ema; fired_mean = cec_swing_detector_mean(&s_i_swing_3v3); }
            if (fired_rail != NULL) {
                ESP_LOGW(TAG, "CURRENT SWING on %s: now=%.3f A, mean=%.3f A, swing=%+.3f A",
                         fired_rail, fired_val, fired_mean, fired_val - fired_mean);
                esp_err_t terr = cec_capture_trigger(CEC_TRIG_CURRENT_SWING);
                if (terr == ESP_OK) {
                    ESP_LOGW(TAG, "burst trigger: CURRENT_SWING");
                }
                cec_swing_detector_reset_to(&s_i_swing_12v, i_12v_ema);
                cec_swing_detector_reset_to(&s_i_swing_5v,  i_5v_ema);
                cec_swing_detector_reset_to(&s_i_swing_3v3, i_3v3_ema);
            }
        } else {
            cec_swing_detector_reset_empty(&s_i_swing_12v);
            cec_swing_detector_reset_empty(&s_i_swing_5v);
            cec_swing_detector_reset_empty(&s_i_swing_3v3);
        }

        /* Periodic NVS save. Only run if profiles got dirty since the
         * last save, and only every NVS_SAVE_INTERVAL_US to limit flash
         * wear. Never during ANY burst phase: the flash write disables
         * cache on both cores and would wreck either HS capture timing
         * (capturing) or UART streaming (dumping). */
        if (!capturing && !dumping && s_profiles_dirty
            && (esp_timer_get_time() - s_last_nvs_save_us) > NVS_SAVE_INTERVAL_US) {
            esp_err_t err = cec_nvs_save_blob(NVS_PROFILES_KEY, NVS_PROFILES_MAGIC,
                                              s_profiles, sizeof(s_profiles));
            if (err == ESP_OK) {
                s_profiles_dirty = false;
                s_last_nvs_save_us = esp_timer_get_time();
                ESP_LOGI(TAG, "NVS: saved profiles (%u bytes)",
                         (unsigned)sizeof(s_profiles));
            } else {
                ESP_LOGW(TAG, "NVS: save failed: %s", esp_err_to_name(err));
                /* Back off until the next interval so we don't spam on
                 * persistent errors. */
                s_last_nvs_save_us = esp_timer_get_time();
            }
        }

        /* Push a pre-trigger sample every iteration so the ring buffer
         * always holds the last ~20 s of filtered telemetry. Skip during
         * CAPTURING (EMAs are held/stale, no fresh data) and during
         * DUMPING (the capture task is reading the ring; we'd overwrite
         * the captured context). */
        if (!capturing && !dumping) {
            cec_capture_sample_t pre = {
                .ts_ms  = (uint32_t)(esp_timer_get_time() / 1000),
                .state  = (uint8_t)s_state,
                .v_12v  = v_12v_ema,  .i_12v  = i_12v_ema,
                .v_5v   = v_5v_ema,   .i_5v   = i_5v_ema,
                .v_3v3  = v_3v3_ema,  .i_3v3  = i_3v3_ema,
                .v_5vsb = v_5vsb_ema, .i_5vsb = i_5vsb_ema,
                .temp_c = temp_ema,
            };
            cec_capture_push(&pre);
        }

        /* Steady-state TelePlot stays gated on any burst phase. During
         * CAPTURING the EMAs hold their pre-capture values (sensor reads
         * were skipped above) — emitting those for 4 s would be flatlined
         * stale rows that misrepresent freshness. During DUMPING the
         * capture task owns the UART. */
        if (!capturing && !dumping && iter % TELEPLOT_DIVIDER == 0) {
            /* Use the device-side millisecond clock for every TelePlot
             * row so the slow-loop output shares a time base with the
             * burst-capture b_*hs_* streams. Without this TelePlot
             * auto-stamps with the host wallclock and the two streams
             * can't be aligned. */
            int64_t now_ms = esp_timer_get_time() / 1000;
            if (ok_5vsb) {
                teleplot_emit_t("v_5vsb",     now_ms, v_5vsb);
                teleplot_emit_t("v_5vsb_ema", now_ms, v_5vsb_ema);
                teleplot_emit_t("i_5vsb_raw", now_ms, i_5vsb);
                teleplot_emit_t("i_5vsb_ema", now_ms, i_5vsb_ema);
            }
            if (ok_v_12v) { teleplot_emit_t("v_12v", now_ms, v_12v); teleplot_emit_t("v_12v_ema", now_ms, v_12v_ema); }
            if (ok_v_5v)  { teleplot_emit_t("v_5v",  now_ms, v_5v);  teleplot_emit_t("v_5v_ema",  now_ms, v_5v_ema);  }
            if (ok_v_3v3) { teleplot_emit_t("v_3v3", now_ms, v_3v3); teleplot_emit_t("v_3v3_ema", now_ms, v_3v3_ema); }
            if (ok_i_12v) { teleplot_emit_t("i_12v", now_ms, i_12v); teleplot_emit_t("i_12v_ema", now_ms, i_12v_ema); }
            if (ok_i_5v)  { teleplot_emit_t("i_5v",  now_ms, i_5v);  teleplot_emit_t("i_5v_ema",  now_ms, i_5v_ema);  }
            if (ok_i_3v3) { teleplot_emit_t("i_3v3", now_ms, i_3v3); teleplot_emit_t("i_3v3_ema", now_ms, i_3v3_ema); }
            if (ok_temp)  { teleplot_emit_t("temp_c", now_ms, temp_c); teleplot_emit_t("temp_c_ema", now_ms, temp_ema); }
            teleplot_emit_t("p_total", now_ms, p_total);
            teleplot_emit_t("state",   now_ms, (float)s_state);
            teleplot_emit_t("sev_12v",  now_ms, (float)sev_12v);
            teleplot_emit_t("sev_5v",   now_ms, (float)sev_5v);
            teleplot_emit_t("sev_3v3",  now_ms, (float)sev_3v3);
            teleplot_emit_t("sev_5vsb", now_ms, (float)sev_5vsb);
            teleplot_emit_t("z_max",    now_ms, z_max);
            teleplot_emit_t("shutting_down", now_ms, s_shutting_down ? 1.0f : 0.0f);
            if (cec_swing_detector_is_full(&s_power_swing)) {
                teleplot_emit_t("p_window_mean", now_ms, p_window_mean);
                teleplot_emit_t("p_swing_thr",   now_ms, p_swing_thresh);
            }
        }

        if (iter % LOG_DIVIDER == 0) {
            ESP_LOGI(TAG, "[%s] V: 12=%.3f 5=%.3f 3V3=%.3f 5SB=%.3f | "
                          "I: 12=%.2f 5=%.2f 3V3=%.2f 5SB=%.4f | "
                          "P=%.1fW T=%.1fC",
                     cec_state_name(s_state),
                     v_12v_ema, v_5v_ema, v_3v3_ema, v_5vsb_ema,
                     i_12v_ema, i_5v_ema, i_3v3_ema, i_5vsb_ema,
                     p_total, temp_ema);
        }

        iter++;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}
