#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_log_level.h"
#include "esp_rom_sys.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "onewire_bus.h"
#include "onewire_device.h"
#include "ultrasonic.h"
#include <stdbool.h>
#include <stdio.h>

// Pin
#define PUMP_PIN 2
#define TRIG_PIN 13
#define ECHO_PIN 12
#define TRIG_PIN_TANK 14
#define ECHO_PIN_TANK 27
#define ONEWIRE_GPIO 0
#define ONEWIRE_MAX_DEVICES 2

#define PUMP_TANK_MAX_DISTANCE 30
#define PUMP_WARNING_LIMIT 3 // Số lần lỗi cho phép trước khi tắt bơm
#define PUMP_TANK_ON_DISTANCE_CM 5
#define PUMP_TANK_OFF_DISTANCE_CM 15
#define PUMP_TANK_BASE_AREA_CM2 600 // m^2

#define TANK_SENSOR_MAX_CM 105
#define TANK_HEIGHT_CM 100
#define TANK_SENSOR_OFFSET_CM 5
#define TANK_ALERT_MIN 80 // %
#define TANK_ALERT_MAX 95 // %
#define TANK_AVG_BUF_SIZE 5
#define TANK_VOLLUME_L 160

const static char *TAG = "RAS";
const static char *PUMP_SENSOR_TAG = "Pump sensor";
const static char *TANK_LEVEL_TAG = "Tank";

static esp_err_t pump_err;
static esp_err_t tank_err;
static bool isPumpOn = false;
static int pump_distance;
static float flow_rate_lm = 15;
uint8_t tank_level_pct;

static ultrasonic_t pump_sensor = {.trig_pin = TRIG_PIN, .echo_pin = ECHO_PIN};
static ultrasonic_t tank_level_sensor = {.trig_pin = TRIG_PIN_TANK,
                                         .echo_pin = ECHO_PIN_TANK};

typedef struct {
    int *buf;
    uint8_t size;
    uint8_t index;
    uint8_t count;
    int32_t sum;
} avg_filter_t;

int avg_add(avg_filter_t *f, int new_value) {
    if (f->count < f->size) {
        f->buf[f->index] = new_value;
        f->sum += new_value;
        f->count++;
    } else {
        f->sum -= f->buf[f->index];
        f->buf[f->index] = new_value;
        f->sum += new_value;
    }

    f->index++;
    if (f->index >= f->size)
        f->index = 0;

    return f->sum / f->count;
}

typedef struct {
    const char *msg;
    int value;
    esp_log_level_t level;
} log_package_t;

static QueueHandle_t xLogQueue = NULL;

void log_worker_task(void *arg) {
    log_package_t item;
    const char *TAG = "SYSTEM_LOG";

    while (true) {
        if (xQueueReceive(xLogQueue, &item, portMAX_DELAY) == pdTRUE) {
            switch (item.level) {
            case ESP_LOG_ERROR:
                ESP_LOGE(TAG, "%s: %d", item.msg, item.value);
                break;
            case ESP_LOG_WARN:
                ESP_LOGW(TAG, "%s: %d", item.msg, item.value);
                break;
            case ESP_LOG_INFO:
                ESP_LOGI(TAG, "%s: %d", item.msg, item.value);
                break;
            default:
                ESP_LOGI(TAG, "%s: %d", item.msg, item.value);
                break;
            }
        }
    }
}

void log_push(const char *message, int value, esp_log_level_t level) {
    if (xLogQueue == NULL)
        return;

    log_package_t item;
    item.msg = message;
    item.value = value;
    item.level = level;

    xQueueSend(xLogQueue, &item, 0);
}

void pump_control(void *pvParameters) {
    uint8_t error_count = 0;
    TickType_t pump_off_tick = xTaskGetTickCount();

    while (true) {
        pump_err = ultrasonic_measure(&pump_sensor, PUMP_TANK_MAX_DISTANCE,
                                      &pump_distance);

        if (pump_err != ESP_OK) {
            error_count++;
            if (error_count > PUMP_WARNING_LIMIT) {
                isPumpOn = false;
                gpio_set_level(PUMP_PIN, 0);
                log_push("Lỗi khi đọc giá trị bể bơm", int value, esp_log_level_t level);
                vTaskDelay(pdMS_TO_TICKS(200));
                error_count = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        error_count = 0;

        if (!isPumpOn && (pump_distance <= PUMP_TANK_ON_DISTANCE_CM)) {
            isPumpOn = true;
            gpio_set_level(PUMP_PIN, isPumpOn);
            uint32_t time_off_ms =
                (xTaskGetTickCount() - pump_off_tick) * portTICK_PERIOD_MS;

            if (time_off_ms > 0)
                flow_rate_lm =
                    PUMP_TANK_BASE_AREA_CM2 *
                    (PUMP_TANK_OFF_DISTANCE_CM - PUMP_TANK_ON_DISTANCE_CM) *
                    60.0f / time_off_ms;

        } else if (isPumpOn && (pump_distance >= PUMP_TANK_OFF_DISTANCE_CM)) {
            isPumpOn = false;
            gpio_set_level(PUMP_PIN, isPumpOn);
            pump_off_tick = xTaskGetTickCount();
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void log_pump_data() {
    while (1) {
        if (pump_err != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        ESP_LOGI(PUMP_SENSOR_TAG, "%d cm | %d", pump_distance, isPumpOn);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void check_tank_level(void *pvParameters) {
    int tank_level_buf[TANK_AVG_BUF_SIZE];
    avg_filter_t tank_level_filter = {.buf = tank_level_buf,
                                      .size = TANK_AVG_BUF_SIZE,
                                      .index = 0,
                                      .count = 0,
                                      .sum = 0};

    while (true) {
        int distance;
        tank_err = ultrasonic_measure(&tank_level_sensor, TANK_SENSOR_MAX_CM,
                                      &distance);
        if (tank_err != ESP_OK) {
            ESP_LOGE(TANK_LEVEL_TAG, "Error code %#x", tank_err);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        int avg_dis = avg_add(&tank_level_filter, distance);

        int tank_level_cm = avg_dis - TANK_SENSOR_OFFSET_CM;
        tank_level_pct = (uint8_t)(((TANK_HEIGHT_CM - tank_level_cm) * 100) /
                                   TANK_HEIGHT_CM);

        int time_ex_tank_min;
        if (flow_rate_lm > 0)
            time_ex_tank_min =
                (TANK_VOLLUME_L * tank_level_pct * 0.01f) / flow_rate_lm;

        esp_log_level_t log_level = ESP_LOG_INFO;
        if (tank_level_pct <= TANK_ALERT_MIN ||
            tank_level_pct >= TANK_ALERT_MAX) {
            log_level = ESP_LOG_ERROR;
        }

        ESP_LOG_LEVEL_LOCAL(log_level, TANK_LEVEL_TAG,
                            "RAW:%d cm | AVG:%d cm | LEVEL:%d", distance,
                            avg_dis, tank_level_pct);

        ESP_LOGI(TANK_LEVEL_TAG, "Tank replace time: %d minus",
                 time_ex_tank_min);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void) {
    gpio_config_t io_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };

    xLogQueue = xQueueCreate(10, sizeof(log_package_t));
    xTaskCreate(log_worker_task, "log_worker", 2048, NULL, 5, NULL);

    // TRIG output
    io_cfg.mode = GPIO_MODE_OUTPUT;
    io_cfg.pin_bit_mask = (1ULL << PUMP_PIN);
    gpio_config(&io_cfg);

    ultrasonic_init(&pump_sensor);
    ultrasonic_init(&tank_level_sensor);

    xTaskCreate(pump_control, "pump_control", 4096, NULL, 10, NULL);
    xTaskCreate(log_pump_data, "log_pump_data", 4096, NULL, 5, NULL);
    xTaskCreate(check_tank_level, "check_tank_level", 4096, &tank_level_sensor,
                5, NULL);
}
