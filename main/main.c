#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/ledc.h"


_Static_assert(configTICK_RATE_HZ >= 1000, "Set CONFIG_FREERTOS_HZ=1000");
static const char *TAG = "sentinel";

// Pin map ------------------------------------------------------------------------------------------
//   Stepper IN1-IN4 : GPIO 12, 13, 14, 27
//   HC-SR04 TRIG    : GPIO 25
//   HC-SR04 ECHO    : GPIO 26 (through the 5V -> 3.3V divider)
//   LCD SDA / SCL   : GPIO 32 / 33
//   Joystick VRx    : GPIO 35 (ADC1 channel 7, joystick powered from 3V3)
//   Buzzer +        : GPIO 23

#define SENTINEL_NO_ECHO  0xFFFF
#define SENSOR_TIMEOUT_MS  500    // sensor data older than this is treated as lost

// Shared state -------------------------------------------------------------------------------------

typedef struct {
    int16_t  angle_deg;
    uint16_t distance_cm;
    int64_t  updated_us;          // when the ultrasonic task last reported
} scan_state_t;

static scan_state_t      g_state = { 0, SENTINEL_NO_ECHO, 0 };
static SemaphoreHandle_t g_state_mutex;

static void state_set_angle(int16_t a) {
    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_state.angle_deg = a;
        xSemaphoreGive(g_state_mutex);
    }
}

static void state_set_distance(uint16_t d) {
    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_state.distance_cm = d;
        g_state.updated_us = esp_timer_get_time();
        xSemaphoreGive(g_state_mutex);
    }
}

static scan_state_t state_get(void) {
    scan_state_t copy = { 0, SENTINEL_NO_ECHO, 0 };
    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        copy = g_state;                      // copy under the lock...
        xSemaphoreGive(g_state_mutex);       // ...then release immediately
    }
    return copy;
}

static bool sensor_fresh(const scan_state_t *s) {
    return s->updated_us != 0 &&
           (esp_timer_get_time() - s->updated_us) / 1000 <= SENSOR_TIMEOUT_MS;
}

// Ultrasonic sensor control ------------------------------------------------------------------------

#define TRIG_PIN           GPIO_NUM_25
#define ECHO_PIN           GPIO_NUM_26
#define ECHO_TIMEOUT_MS    30     // no echo within this = nothing in range
#define MEASURE_PERIOD_MS  60     // let old echoes die out before re-triggering

static TaskHandle_t ultrasonic_task_handle = NULL;
static volatile int64_t  echo_rise_us  = 0;
static volatile uint32_t echo_width_us = 0;

static void IRAM_ATTR echo_isr(void *arg) {
    int64_t now = esp_timer_get_time();            // microsecond timestamp
    if (gpio_get_level(ECHO_PIN)) {
        echo_rise_us = now;                         // rising edge: start of pulse
    } else {
        echo_width_us = (uint32_t)(now - echo_rise_us);   // falling edge: pulse width
        if (ultrasonic_task_handle != NULL) {
            BaseType_t woken = pdFALSE;
            vTaskNotifyGiveFromISR(ultrasonic_task_handle, &woken);
            portYIELD_FROM_ISR(woken);              // switch to the task right away if needed
        }
    }
}

static void ultrasonic_gpio_init(void) {
    gpio_config_t trig = {
        .pin_bit_mask = 1ULL << TRIG_PIN,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&trig));
    gpio_set_level(TRIG_PIN, 0);

    gpio_config_t echo = {
        .pin_bit_mask = 1ULL << ECHO_PIN,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE,             // interrupt on rising AND falling
    };
    ESP_ERROR_CHECK(gpio_config(&echo));

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ECHO_PIN, echo_isr, NULL));
}

static void ultrasonic_task(void *arg) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, 0);                // clear any stale notification

        gpio_set_level(TRIG_PIN, 1);                // 10 us trigger pulse
        esp_rom_delay_us(10);
        gpio_set_level(TRIG_PIN, 0);

        uint16_t cm = SENTINEL_NO_ECHO;
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ECHO_TIMEOUT_MS)) > 0) {
            uint32_t c = echo_width_us / 58;
            if (c >= 2 && c <= 400) cm = (uint16_t)c;   // HC-SR04 range is about 2 to 400 cm
      //  } else {
      //      ESP_LOGW(TAG, "no echo, ECHO pin level=%d", gpio_get_level(ECHO_PIN));
        }
        state_set_distance(cm);
        vTaskDelay(pdMS_TO_TICKS(MEASURE_PERIOD_MS));
    }
}

// Stepper motor control ----------------------------------------------------------------------------

#define STEPS_PER_REV  4096                          // half-steps per output shaft rev (28BYJ-48)
#define ARC_DEG        90                            // total sweep width
#define ARC_STEPS      ((STEPS_PER_REV * ARC_DEG) / 360)
#define STEP_DELAY_MS  2

static const gpio_num_t coil_pins[4] = {GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_27};

// 8-phase half-step sequence: which of the 4 coils are on in each phase
static const uint8_t half_step[8][4] = {
    {1,0,0,0}, {1,1,0,0}, {0,1,0,0}, {0,1,1,0},
    {0,0,1,0}, {0,0,1,1}, {0,0,0,1}, {1,0,0,1},
};

static int phase = 0;
static int32_t step_pos = 0;                         // 0 .. ARC_STEPS

static void stepper_init(void) {
    for (int i = 0; i < 4; i++) {
        gpio_reset_pin(coil_pins[i]);
        gpio_set_direction(coil_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(coil_pins[i], 0);
    }
}

static void stepper_step(int dir) {                  // dir = +1 or -1
    phase = (phase + dir + 8) % 8;                   // wrap around the 8-entry table
    for (int i = 0; i < 4; i++) {
        gpio_set_level(coil_pins[i], half_step[phase][i]);
    }
}

static void sweep_task(void *arg) {
    int dir = +1;
    for (;;) {
        stepper_step(dir);
        step_pos += dir;
        if (step_pos >= ARC_STEPS)      dir = -1;
        else if (step_pos <= 0)         dir = +1;

        int16_t angle = (int16_t)((step_pos * ARC_DEG) / ARC_STEPS) - ARC_DEG / 2;
        state_set_angle(angle);
        vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));
    }
}

// Joystick (alert threshold) and buzzer ------------------------------------------------------------

#define BUZZER_PIN        GPIO_NUM_23
#define JOY_ADC_CHANNEL   ADC_CHANNEL_7      // GPIO35 on ADC1
#define THRESH_MIN_CM     5
#define THRESH_MAX_CM     100
#define JOY_DEADZONE  350

static volatile uint32_t g_threshold_cm = 30;        // single writer (input_task), so no mutex needed

static adc_oneshot_unit_handle_t adc1_handle;

static void joystick_init(void) {
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc1_handle));
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOY_ADC_CHANNEL, &chan_cfg));
}

static int joystick_read(void) {
    int sum = 0, v = 0;
    for (int i = 0; i < 8; i++) {
        if (adc_oneshot_read(adc1_handle, JOY_ADC_CHANNEL, &v) == ESP_OK) sum += v;
    }
    return sum / 8;                                   // average 8 samples to tame ADC noise
}

static int joy_center = 2048;

static bool joy_ok = true;

static void joystick_calibrate(void) {
    vTaskDelay(pdMS_TO_TICKS(200));
    long sum = 0;
    int lo = 4095, hi = 0;
    for (int i = 0; i < 16; i++) {
        int v = joystick_read();
        sum += v;
        if (v < lo) lo = v;
        if (v > hi) hi = v;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    joy_center = (int)(sum / 16);
    ESP_LOGI(TAG, "joystick center = %d (min %d, max %d)", joy_center, lo, hi);
    if (hi - lo > 150 || joy_center < 300 || joy_center > 3900) {
        ESP_LOGW(TAG, "joystick unusable at rest (unstable or pinned) - input disabled");
        joy_ok = false;
    }
}

static void input_task(void *arg) {
    for (;;) {
        int raw = joystick_read();
        //ESP_LOGI(TAG, "joy raw=%d", raw);
        int t = (int)g_threshold_cm;
        static int n = 0;
        //if (++n % 5 == 0) ESP_LOGI(TAG, "joy raw=%d center=%d", raw, joy_center);
        int up_thr   = joy_center + (4095 - joy_center) / 2;
        int down_thr = joy_center / 2;
        if (joy_ok && raw > up_thr && t < THRESH_MAX_CM)        t++;
        else if (joy_ok && raw < down_thr && t > THRESH_MIN_CM) t--;
        g_threshold_cm = (uint32_t)t;
        vTaskDelay(pdMS_TO_TICKS(100));                   // 10 cm/s ramp rate
    }
}

static void buzzer_init(void) {
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 2500,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    ledc_channel_config_t ch = {
        .gpio_num = BUZZER_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));
}

static void buzzer_set(bool on) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, on ? 512 : 0);   // 512/1024 = 50% duty
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void alert_task(void *arg) {
    for (;;) {
        scan_state_t s = state_get();

        if (sensor_fresh(&s) && s.distance_cm != SENTINEL_NO_ECHO &&
            s.distance_cm < g_threshold_cm) {
            buzzer_set(true);
            vTaskDelay(pdMS_TO_TICKS(60));                        // fixed beep length
            buzzer_set(false);
            vTaskDelay(pdMS_TO_TICKS(40 + s.distance_cm * 6));    // closer = faster beeps
        } else {
            buzzer_set(false);                      // silent: safe default
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

// I2C LCD (HD44780 via PCF8574 backpack) -----------------------------------------------------------

#define I2C_SDA_PIN  GPIO_NUM_32
#define I2C_SCL_PIN  GPIO_NUM_33

#define LCD_RS  0x01
#define LCD_EN  0x04
#define LCD_BL  0x08

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t lcd_dev;

static void pcf_write(uint8_t b) {
    (void)i2c_master_transmit(lcd_dev, &b, 1, 50);
}

static void lcd_write_nibble(uint8_t nibble, uint8_t rs) {
    uint8_t d = (uint8_t)(((nibble & 0x0F) << 4) | LCD_BL | (rs ? LCD_RS : 0));
    pcf_write(d | LCD_EN);
    esp_rom_delay_us(1);
    pcf_write(d & (uint8_t)~LCD_EN);
    esp_rom_delay_us(50);
}

static void lcd_send(uint8_t byte, uint8_t rs) {
    lcd_write_nibble(byte >> 4, rs);
    lcd_write_nibble(byte & 0x0F, rs);
}

static void lcd_print_line(int row, const char *s) {
    lcd_send(0x80 | (row ? 0x40 : 0x00), 0);
    bool end = false;
    for (int i = 0; i < 16; i++) {
        if (!end && s[i] == '\0') end = true;
        lcd_send(end ? ' ' : (uint8_t)s[i], 1);
    }
}

static bool lcd_init(void) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    uint8_t addr = 0;
    for (uint8_t a = 0x03; a < 0x78; a++) {
        if (i2c_master_probe(i2c_bus, a, 20) == ESP_OK) {
            ESP_LOGI(TAG, "I2C device found at 0x%02X", a);
            if (addr == 0) addr = a;
        }
    }
    if (addr == 0) {
        ESP_LOGE(TAG, "No I2C device found - check SDA/SCL/VCC/GND wiring");
        return false;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, &lcd_dev));

    pcf_write(LCD_BL);
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_write_nibble(0x03, 0); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(0x03, 0); vTaskDelay(pdMS_TO_TICKS(2));
    lcd_write_nibble(0x03, 0); vTaskDelay(pdMS_TO_TICKS(2));
    lcd_write_nibble(0x02, 0);
    lcd_send(0x28, 0);
    lcd_send(0x0C, 0);
    lcd_send(0x06, 0);
    lcd_send(0x01, 0);
    vTaskDelay(pdMS_TO_TICKS(3));
    return true;
}

static void display_task(void *arg) {
    char l0[24], l1[24];
    for (;;) {
        scan_state_t s = state_get();

        if (!sensor_fresh(&s)) {
            snprintf(l0, sizeof l0, "NO SIGNAL");
            l1[0] = '\0';
        } else {
            snprintf(l0, sizeof l0, "Angle: %4d deg", (int)s.angle_deg);
            if (s.distance_cm == SENTINEL_NO_ECHO) snprintf(l1, sizeof l1, "Dist:--- Lim:%3u", (unsigned)g_threshold_cm);
            else snprintf(l1, sizeof l1, "Dist:%3u Lim:%3u", (unsigned)s.distance_cm, (unsigned)g_threshold_cm);
        }
        lcd_print_line(0, l0);
        lcd_print_line(1, l1);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// main ---------------------------------------------------------------------------------------------

static void stream_task(void *arg) {
    int64_t last = 0;
    for (;;) {
        scan_state_t s = state_get();
        if (s.updated_us != last) {                 // only when there is a new measurement
            last = s.updated_us;
            printf("R,%d,%u,%u\n", (int)s.angle_deg, (unsigned)s.distance_cm, (unsigned)g_threshold_cm);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    g_state_mutex = xSemaphoreCreateMutex();        // create BEFORE any task uses it
    configASSERT(g_state_mutex != NULL);

    stepper_init();
    ultrasonic_gpio_init();
    joystick_init();
    joystick_calibrate();
    buzzer_init();

    xTaskCreate(stream_task,     "stream",  2048, NULL, 7, NULL);
    xTaskCreate(sweep_task,      "sweep",   2048, NULL, 5, NULL);
    xTaskCreate(ultrasonic_task, "ultra",   3072, NULL, 6, &ultrasonic_task_handle);
    xTaskCreate(alert_task,      "alert",   3072, NULL, 4, NULL);
    xTaskCreate(input_task,      "input",   3072, NULL, 2, NULL);
    if (lcd_init()) {
        xTaskCreate(display_task, "display", 4096, NULL, 3, NULL);
    }
}