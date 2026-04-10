/*
 * MPPT Solar Charger (Perturb & Observe) + 1602A LCD via PCF8574
 * ESP-IDF  |  ESP32
 *
 * ADC pin map:
 *   ADC1_CH0   →  Solar panel voltage  (via resistor divider)
 *   ADC1_CH3   →  Solar panel current  (ACS712 or similar)
 *   ADC2_CH9   →  Buck converter output voltage
 *
 * PWM:
 *   MCPWM Unit 0  100 kHz  →  Buck High-side gate (GPIO 4)  [complementary + dead time]
 *                              Buck Low-side  gate (GPIO 27)
 *
 * Synchronous Buck — two MOSFETs, complementary PWM, dead time on both edges:
 *   High-side : goes HIGH at timer ZERO, goes LOW at comparator match
 *   Low-side  : exact inverse of high-side, dead-time guard on every transition
 *   Dead time : 400 ns  →  prevents shoot-through (both MOSFETs ON simultaneously)
 *
 * I2C:
 *   SDA = GPIO 21,  SCL = GPIO 22
 *   PCF8574 address = 0x27  (change to 0x3F if your module differs)
 *
 * Dependencies in CMakeLists.txt REQUIRES:
 *   esp_adc driver esp_driver_mcpwm esp_driver_i2c esp_driver_gpio
 */

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <math.h>


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "driver/i2c.h"
#include "driver/mcpwm_prelude.h"
#include "freertos/queue.h"
#include "hal/adc_types.h"
#include "esp_rom_sys.h"


/* ================================================================
 *  CONFIGURATION — adjust these to match your hardware
 * ================================================================ */

/* --- ADC sampling --- */
#define NUM_SAMPLES     5

/* --- Buck converter PWM (MCPWM Unit 0, 100 kHz, complementary + dead time) ---
 *
 *   BUCK_HIGH_GPIO      : High-side MOSFET gate pin
 *   BUCK_LOW_GPIO       : Low-side  MOSFET gate pin
 *   BUCK_TIMER_RES_HZ   : MCPWM internal timer clock  (10 MHz → 100 ns per tick)
 *   BUCK_FREQ_HZ        : Switching frequency          (100 kHz)
 *   BUCK_PERIOD_TICKS   : Ticks per PWM period = 10 000 000 / 100 000 = 100 ticks
 *   BUCK_DEAD_TIME_NS   : Dead time between transitions (200 ns = 2 ticks @ 10 MHz)
 *                         Increase if your MOSFET / gate driver needs more margin.
 *   BUCK_DUTY_MIN_TICKS : Minimum compare value (10% on-time)
 *   BUCK_DUTY_MAX_TICKS : Maximum compare value (90% on-time)
 *   BUCK_STEP_TICKS     : P&O perturbation step size per iteration
 */

#define BUCK_GPIO           27
#define BUCK_TIMER_RES_HZ       10000000UL                            /* 10 MHz            */
#define BUCK_FREQ_HZ            100000                                /* 100 kHz           */
#define BUCK_PERIOD_TICKS       (BUCK_TIMER_RES_HZ / BUCK_FREQ_HZ)   /* 100 ticks         */
#define BUCK_DUTY_MIN_TICKS     (BUCK_PERIOD_TICKS * 10 / 100)       /* 10%  =  10 ticks  */
#define BUCK_DUTY_MAX_TICKS     (BUCK_PERIOD_TICKS * 50 / 100)       /* 50%  =  50 ticks  */
#define BUCK_STEP_TICKS         1                                     /* P&O step          */

/* --- Voltage / current scaling ---
 *   RATIO           : resistor-divider ratio  (e.g. 0.2 means Vreal = Vadc / 0.2)
 *   CURRENT_OFFSET  : sensor zero-current output in mV (ACS712 = 2500 mV)
 *   CURRENT_SENS    : sensor sensitivity in mV/A        (ACS712-5A = 185, 20A = 100, 30A = 66)
 */
#define RATIO               0.2f
#define CURRENT_OFFSET_MV   2500
#define CURRENT_SENS_MV_A   100     

/* --- I2C / LCD --- */
#define I2C_MASTER_SDA      21
#define I2C_MASTER_SCL      22
#define I2C_MASTER_PORT     I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  100000
#define LCD_ADDR            0x27    /* try 0x3F if 0x27 does not work */

/* PCF8574 → HD44780 pin mapping (standard 1602 I2C backpack) */
#define LCD_RS  (1 << 0)
#define LCD_RW  (1 << 1)
#define LCD_EN  (1 << 2)
#define LCD_BL  (1 << 3)   /* backlight */
#define LCD_D4  (1 << 4)
#define LCD_D5  (1 << 5)
#define LCD_D6  (1 << 6)
#define LCD_D7  (1 << 7)

/* ================================================================
 *  GLOBALS
 * ================================================================ */

static const char *TAG = "MPPT";

/* ADC handles */
adc_oneshot_unit_handle_t adc1_handle;
adc_oneshot_unit_handle_t adc2_handle = NULL;
adc_cali_handle_t cali_handle1 = NULL;
adc_cali_handle_t cali_handle2 = NULL;
bool adc1_cali_enabled = false;
bool adc2_cali_enabled = false;

/* Queues */
QueueHandle_t mppt_queue    = NULL;
QueueHandle_t display_queue = NULL;

/* MCPWM comparator handle — set in app_main, used in MPPTtask to update duty cycle */
mcpwm_cmpr_handle_t buck_comparator = NULL;

/* ================================================================
 *  QUEUE DATA TYPES
 * ================================================================ */

typedef struct {
    int mppt_voltage;    /* solar panel voltage  [mV] */
    int mppt_current;    /* solar panel current  [mA] */
    int output_voltage;  /* buck output voltage  [mV] */
} mppt_data_t;

typedef struct {
    float power_in_mW;    /* input power to display  */
    int   buck_duty_pct;  /* buck duty cycle 0-100 % */
} display_data_t;

/* ================================================================
 *  HELPER: ADC averaged read  (BUG FIX: braces were wrong before)
 * ================================================================ */

static int adc_read_avg(adc_oneshot_unit_handle_t handle,
                        adc_channel_t channel,
                        int num_samples,
                        int delay_msec)
{
    int raw, sum = 0;
    for (int i = 0; i < num_samples; i++) {
        ESP_ERROR_CHECK(adc_oneshot_read(handle, channel, &raw));
        sum += raw;
    }
    vTaskDelay(pdMS_TO_TICKS(delay_msec));
    return (sum / num_samples);
}

/* ================================================================
 *  LCD / PCF8574 DRIVER
 * ================================================================ */

static uint8_t lcd_bl = LCD_BL;   /* backlight flag, on by default */

/* Write one byte to PCF8574 over I2C */
static esp_err_t pcf8574_write(uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* Pulse the Enable pin to latch data */
static void lcd_pulse_en(uint8_t data)
{
    pcf8574_write(data | LCD_EN);
    esp_rom_delay_us(600);
    pcf8574_write(data & ~LCD_EN);
    esp_rom_delay_us(60);
}

/* Send one nibble (4 bits) */
static void lcd_write_nibble(uint8_t nibble, uint8_t mode)
{
    uint8_t d = lcd_bl | mode;
    d &= ~(LCD_D4 | LCD_D5 | LCD_D6 | LCD_D7);
    if (nibble & 0x01) d |= LCD_D4;
    if (nibble & 0x02) d |= LCD_D5;
    if (nibble & 0x04) d |= LCD_D6;
    if (nibble & 0x08) d |= LCD_D7;
    lcd_pulse_en(d);
}

/* Send one full byte as two nibbles */
static void lcd_write_byte(uint8_t byte, uint8_t mode)
{
    lcd_write_nibble(byte >> 4,   mode);  /* high nibble first */
    lcd_write_nibble(byte & 0x0F, mode);  /* low  nibble       */
}

/* Send a command byte (RS = 0) */
static void lcd_cmd(uint8_t cmd)
{
    lcd_write_byte(cmd, 0);
    /* Clear and home need extra time */
    if (cmd == 0x01 || cmd == 0x02)
        vTaskDelay(pdMS_TO_TICKS(5));
    else
        esp_rom_delay_us(50);
}

/* Send a data byte (RS = 1) */
static void lcd_data(uint8_t ch)
{
    lcd_write_byte(ch, LCD_RS);
    esp_rom_delay_us(40);
}

/* Initialise the 1602A in 4-bit mode */
static void lcd_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));

    lcd_write_nibble(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(5));

    lcd_write_nibble(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(2));

    lcd_write_nibble(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(2));

    lcd_write_nibble(0x02, 0);
    vTaskDelay(pdMS_TO_TICKS(2));

    lcd_cmd(0x28);
    lcd_cmd(0x08);
    lcd_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_cmd(0x06);
    lcd_cmd(0x0C);
    vTaskDelay(pdMS_TO_TICKS(2));

    ESP_LOGI(TAG, "LCD initialised");
}
/* Position cursor: col 0–15, row 0–1 */
static void lcd_set_cursor(uint8_t col, uint8_t row)
{
    static const uint8_t row_offset[] = {0x00, 0x40};
    lcd_cmd(0x80 | (col + row_offset[row & 1]));
}

/* Print a null-terminated string */
static void lcd_print(const char *s)
{
    while (*s) lcd_data((uint8_t)*s++);
}

/* Clear display and home cursor */
static void lcd_clear(void)
{
    lcd_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(2));
}

/* ================================================================
 *  TASK: ADCtask — power sensing only
 * ================================================================ */

void ADCtask(void *pvParameters)
{
    mppt_data_t    calibrated = {0};
    mppt_data_t    mppt_data  = {0};

    while (1) {
        /* ---------- Voltage / current / output readings ---------- */
        int v_raw  = adc_read_avg(adc1_handle, ADC_CHANNEL_0, NUM_SAMPLES, 5);
        int i_raw  = adc_read_avg(adc1_handle, ADC_CHANNEL_3, NUM_SAMPLES, 5);
        int vo_raw = adc_read_avg(adc2_handle, ADC_CHANNEL_9, NUM_SAMPLES, 5);

        if (adc1_cali_enabled) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle1, v_raw,  &calibrated.mppt_voltage));
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle1, i_raw,  &calibrated.mppt_current));
        }
        else {
              calibrated.mppt_voltage = v_raw;   // use raw as fallback
              calibrated.mppt_current = i_raw;
              }
        if (adc2_cali_enabled) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle2, vo_raw, &calibrated.output_voltage));
        }
        else{
			calibrated.output_voltage = vo_raw;
		}

        /*
         * Scale ADC millivolt readings to real-world values:
         *   voltage [mV]  =  calibrated_mV / RATIO          (resistor divider)
         *   current [mA]  =  (calibrated_mV - offset_mV) * 1000 / sensitivity_mV_per_A
         */
        mppt_data.mppt_voltage   = (int)(calibrated.mppt_voltage  / RATIO);
        mppt_data.mppt_current   = (int)((calibrated.mppt_current - CURRENT_OFFSET_MV)
                                          * 1000 / CURRENT_SENS_MV_A);   /* mA */
        mppt_data.output_voltage = (int)(calibrated.output_voltage / RATIO);

        xQueueOverwrite(mppt_queue, &mppt_data);

        vTaskDelay(pdMS_TO_TICKS(10));   /* 100 Hz sampling */
    }
}

/* ================================================================
 *  TASK: MPPTtask — Perturb & Observe algorithm
 *
 *  Principle:
 *    1. Measure panel voltage V and current I  →  P = V × I
 *    2. Compare P with previous power P_prev
 *    3. If ΔP > 0  the perturbation moved us toward MPP → keep direction
 *    4. If ΔP < 0  we moved away from MPP            → reverse direction
 *    5. Clamp duty within safe limits and apply to buck PWM channel
 * ================================================================ */

void MPPTtask(void *pvParameters)
{
    mppt_data_t    mppt;
    display_data_t disp;

    float    prev_power_mW = 0.0f;
    int buck_duty     = (BUCK_DUTY_MIN_TICKS + BUCK_DUTY_MAX_TICKS) / 2; /* start at 50% */
    int      direction     = BUCK_STEP_TICKS;   /* positive → increase duty */

    /* Apply initial duty to comparator — hardware starts switching immediately */
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(buck_comparator, buck_duty));

    while (1) {
        if (xQueueReceive(mppt_queue, &mppt, pdMS_TO_TICKS(200))) {

            float V_mV      = (float)mppt.mppt_voltage;   /* mV */
            float I_mA      = (float)mppt.mppt_current;   /* mA */
            float power_mW  = (V_mV * I_mA) / 1000.0f;   /* mW = mV × mA / 1000 */

            /* Ignore negative power (sensor noise / no panel connected) */
            if (power_mW < 0.0f) power_mW = 0.0f;

            float delta_P = power_mW - prev_power_mW;

            /*
             * Only perturb if the power change exceeds the noise floor (1 mW).
             * This prevents endless oscillation in steady-state conditions.
             */
            if (fabsf(delta_P) > 1.0f) {
                if (delta_P > 0.0f) {
                    /* Moving in the right direction — continue */
                    buck_duty += direction;
                } else {
                    /* Overshot — reverse */
                    direction = -direction;
                    buck_duty += direction;
                }
            }

            /* Safety clamp — never leave the 10%–90% window */
            if (buck_duty > BUCK_DUTY_MAX_TICKS) buck_duty = BUCK_DUTY_MAX_TICKS;
            if (buck_duty < BUCK_DUTY_MIN_TICKS) buck_duty = BUCK_DUTY_MIN_TICKS;

            /*
             * Single call updates both MOSFETs simultaneously.
             * The new value takes effect at the next timer-zero crossing so
             * there is never a mid-period glitch or shoot-through risk.
             */
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(buck_comparator, buck_duty));

            prev_power_mW = power_mW;

            /* Prepare display data */
            disp.power_in_mW   = power_mW;
            disp.buck_duty_pct = (int)(buck_duty * 100 / BUCK_PERIOD_TICKS);
            xQueueOverwrite(display_queue, &disp);

            ESP_LOGI(TAG, "MPPT | Vin=%4dmV  Iin=%4dmA  Pin=%7.1fmW  Duty=%3d%%",
                     mppt.mppt_voltage, mppt.mppt_current,
                     power_mW, disp.buck_duty_pct);
        }
    }
}

/* ================================================================
 *  TASK: LCDtask — refresh 1602A display at 2 Hz
 *
 *  Line 1:  "Pin: 1234.5  mW"
 *  Line 2:  "Duty:     75  %"
 * ================================================================ */

void LCDtask(void *pvParameters)
{
    display_data_t disp = {0};
    char line1[17], line2[17];

    lcd_init();

    /* Splash screen */
    lcd_set_cursor(0, 0); lcd_print("  MPPT Charger  ");
    lcd_set_cursor(0, 1); lcd_print("  Initialising  ");
    vTaskDelay(pdMS_TO_TICKS(2000));
    lcd_clear();

    while (1) {
        /* Block up to 500 ms for fresh data; if nothing arrives reuse old values */
        xQueueReceive(display_queue, &disp, pdMS_TO_TICKS(500));

        /*
         * Format into exactly 16 characters per line so we never need to
         * clear the display between updates (avoids visible flicker).
         *
         * Line 1 example: "Pin: 1234.5 mW  "
         * Line 2 example: "Duty:      75 % "
         */
        snprintf(line1, sizeof(line1), "Pin:%8.1f mW ", disp.power_in_mW);
        snprintf(line2, sizeof(line2), "Duty:      %3d%% ", disp.buck_duty_pct);

        lcd_set_cursor(0, 0);
        lcd_print(line1);
        lcd_set_cursor(0, 1);
        lcd_print(line2);

        vTaskDelay(pdMS_TO_TICKS(500));   /* 2 Hz refresh */
    }
}

/* ================================================================
 *  app_main
 * ================================================================ */

void app_main(void)
{
    /* ----------------------------------------------------------------
     *  I2C — master mode for PCF8574 / LCD
     * ---------------------------------------------------------------- */
    i2c_config_t i2c_conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_MASTER_SDA,
        .scl_io_num       = I2C_MASTER_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_PORT, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_PORT, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGI(TAG, "I2C master ready  SDA=%d  SCL=%d", I2C_MASTER_SDA, I2C_MASTER_SCL);

    /* ----------------------------------------------------------------
     *  ADC Unit 1
     * ---------------------------------------------------------------- */
    adc_oneshot_unit_init_cfg_t u1_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&u1_cfg, &adc1_handle));

    adc_oneshot_chan_cfg_t ch_cfg1 = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &ch_cfg1));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_3, &ch_cfg1));

    /* ----------------------------------------------------------------
     *  ADC Unit 2
     * ---------------------------------------------------------------- */
    adc_oneshot_unit_init_cfg_t u2_cfg = {
        .unit_id  = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&u2_cfg, &adc2_handle));

    adc_oneshot_chan_cfg_t ch_cfg2 = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12,
    };
    /* BUG FIX: was adc1_handle — must use adc2_handle for ADC2 channels */
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_9, &ch_cfg2));

    /* ----------------------------------------------------------------
     *  ADC Calibration
     * ---------------------------------------------------------------- */
    ESP_LOGI(TAG, "ADC calibration scheme: Line Fitting");

    adc_cali_line_fitting_config_t cal1 = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_line_fitting(&cal1, &cali_handle1) == ESP_OK) {
        adc1_cali_enabled = true;
        ESP_LOGI(TAG, "ADC1 calibration enabled");
    } else {
        ESP_LOGW(TAG, "ADC1 calibration not supported — raw values will be used");
    }

    adc_cali_line_fitting_config_t cal2 = {
        .unit_id  = ADC_UNIT_2,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_line_fitting(&cal2, &cali_handle2) == ESP_OK) {
        adc2_cali_enabled = true;
        ESP_LOGI(TAG, "ADC2 calibration enabled");
    } else {
        ESP_LOGW(TAG, "ADC2 calibration not supported — raw values will be used");
    }

    /* ----------------------------------------------------------------
     *  MCPWM — Synchronous Buck Converter (100 kHz, complementary + dead time)
     *
     *  Object hierarchy (must be created in this exact order):
     *    Timer → Operator → Comparator → Generator A → Generator B → Dead time → Start
     *
     *  Timer: counts UP from 0 → BUCK_PERIOD_TICKS (100 ticks) then wraps.
     *         At 10 MHz that is one tick per 100 ns → period = 10 µs = 100 kHz.
     *
     *  Comparator: stores the duty boundary (in ticks).
     *              When timer count == comparator value, generators react.
     *              update_cmp_on_tez = true  →  new value latched only at
     *              timer-zero so there is never a mid-period glitch.
     *
     *  Generator A (high-side, GPIO 25):
     *    timer ZERO event   → output HIGH   (start of on-time)
     *    comparator event   → output LOW    (end of on-time)
     *
     *  Generator B (low-side, GPIO 26):
     *    No timer/compare actions set directly.
     *    Its waveform is produced entirely by the dead-time block:
     *    input = Generator A, output = inverted A + rising-edge delay.
     *
     *  Dead time (200 ns = 2 ticks on each edge):
     *
     *    High-side gate:  __DT|‾‾‾‾‾‾‾‾‾‾‾|DT__
     *    Low-side  gate:  ‾‾‾‾|___________|‾‾‾‾
     *                         ↑           ↑
     *                     dead gaps — both MOSFETs OFF — no shoot-through
     * ---------------------------------------------------------------- */

    /* 1. Timer */
    ESP_LOGI(TAG, "Create timer and operator");
    
    mcpwm_timer_handle_t buck_timer_h = NULL;
    mcpwm_timer_config_t buck_timer_cfg = {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = BUCK_TIMER_RES_HZ,          /* 10 MHz                    */
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,   /* edge-aligned              */
        .period_ticks  = BUCK_PERIOD_TICKS,           /* 100 ticks = 100 kHz       */
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&buck_timer_cfg, &buck_timer_h));

    /* 2. Operator — owns comparators and generators, must use same group as timer */
    mcpwm_oper_handle_t buck_oper = NULL;
    mcpwm_operator_config_t buck_oper_cfg = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&buck_oper_cfg, &buck_oper));
    
    ESP_LOGI(TAG, "Connect timer and operator");
    
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(buck_oper, buck_timer_h));
    
    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    
    /* 3. Comparator — duty boundary; shared with MPPTtask via buck_comparator global */
    mcpwm_comparator_config_t buck_cmp_cfg = {
        .flags.update_cmp_on_tez = true,   /* latch new value at timer-zero only */
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(buck_oper, &buck_cmp_cfg, &buck_comparator));

    


    /* 4. Generator A → HIGH-SIDE MOSFET (GPIO 25) */
    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_cfg = {
        .gen_gpio_num = BUCK_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(buck_oper, &generator_cfg, &generator));

    /* Set starting duty to 35% */
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(buck_comparator,
                    (BUCK_DUTY_MIN_TICKS + BUCK_DUTY_MAX_TICKS) / 2));

    /* HIGH goes HIGH on timer-zero, LOW on comparator match */
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       buck_comparator,
                                       MCPWM_GEN_ACTION_LOW)));

    

    /* 7. Enable timer and start — PWM outputs begin immediately */
    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(buck_timer_h));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(buck_timer_h, MCPWM_TIMER_START_NO_STOP));

   ESP_LOGI(TAG, "Boost MCPWM ready | GPIO=%d | %d Hz | period=%lu ticks",
         BUCK_GPIO,
         BUCK_FREQ_HZ,
         (unsigned long)BUCK_PERIOD_TICKS);
    /* ----------------------------------------------------------------
     *  Queues  (depth 1 + overwrite = always freshest data)
     * ---------------------------------------------------------------- */
    mppt_queue    = xQueueCreate(1, sizeof(mppt_data_t));
    display_queue = xQueueCreate(1, sizeof(display_data_t));

    if (!mppt_queue)    ESP_LOGE(TAG, "mppt_queue creation failed");
    if (!display_queue) ESP_LOGE(TAG, "display_queue creation failed");

    /* ----------------------------------------------------------------
     *  Tasks
     *  Priorities:  ADC(5) > MPPT(4) > LCD(3)
     * ---------------------------------------------------------------- */
    xTaskCreate(ADCtask,   "ADC_TASK",   4096, NULL, 5, NULL);
    xTaskCreate(MPPTtask,  "MPPT_TASK",  4096, NULL, 4, NULL);
    xTaskCreate(LCDtask,   "LCD_TASK",   3072, NULL, 3, NULL);
}
