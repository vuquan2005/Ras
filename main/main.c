#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "ultrasonic.h"
#include <stdbool.h>
#include <stdio.h>

// Pin
#define PUMP_PIN 2
#define TRIG_PIN 13
#define ECHO_PIN 12
#define TRIG_PIN_TANK 13
#define ECHO_PIN_TANK 12

#define PUMP_MAX_DISTANCE 30
#define PUMP_WARNIG_LIMIT 3 // Số lần lỗi cho phép trước khi cảnh báo
#define PUMP_ON_DISTANCE 5
#define PUMP_OFF_DISTANCE 25
#define PUMP_BASE_AREA 0.12 // m^2

#define TANK_HEIGHT 100
#define TANK_SENSOR_OFFSET 5
#define TANK_ALERT_MIN 80
#define TANK_ALERT_MAX 95
#define TANK_AVG_BUF_SIZE 5
#define TANK_BASE_AREA 0.12 // m^2

const static char *TAG = "RAS";
const static char *PUMP_SENSOR_TAG = "Pump sensor";
const static char *TANK_LEVEL_TAG = "Tank";

static esp_err_t pump_err;
static bool isPumpOn = false;
static int pump_distance;

static ultrasonic_t pump_sen = {.trig_pin = TRIG_PIN, .echo_pin = ECHO_PIN};
static ultrasonic_t tank_level_sen = {.trig_pin = TRIG_PIN_TANK,
                                      .echo_pin = ECHO_PIN_TANK};

typedef struct {
    int *buf;
    uint8_t size;
    uint8_t index;
    uint8_t count;
    int32_t sum;
} avg_filter_t;

int avg_add(avg_filter_t *f, int value) {
    if (f->count < f->size) {
        f->buf[f->index] = value;
        f->sum += value;
        f->count++;
    } else {
        f->sum -= f->buf[f->index];
        f->buf[f->index] = value;
        f->sum += value;
    }

    f->index++;
    if (f->index >= f->size)
        f->index = 0;

    return f->sum / f->count;
}

void pump_control(void *pvParameters) {
    uint8_t error_count = 0;
    while (true) {
        pump_err =
            ultrasonic_measure(&pump_sen, PUMP_MAX_DISTANCE, &pump_distance);

        if (pump_err != ESP_OK) {
            error_count++;
            if (error_count > PUMP_WARNIG_LIMIT) {
                gpio_set_level(PUMP_PIN, 0);
                vTaskDelay(pdMS_TO_TICKS(200));
                error_count = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        error_count = 0;

        if (!isPumpOn & (pump_distance <= PUMP_ON_DISTANCE)) {
            isPumpOn = true;
        } else if (isPumpOn & (pump_distance >= PUMP_OFF_DISTANCE)) {
            isPumpOn = false;
        }
        gpio_set_level(PUMP_PIN, isPumpOn);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void log_pump_data() {
    while (1) {
        if (pump_err != ESP_OK) {
            ESP_LOGW(PUMP_SENSOR_TAG, "Error code %#x", pump_err);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        ESP_LOGI(PUMP_SENSOR_TAG, "%d cm | %d", pump_distance, isPumpOn);

        // Tốc độ thay nước

        vTaskDelay(pdMS_TO_TICKS(1000));
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
        pump_err = ultrasonic_measure(&tank_level_sen, TANK_HEIGHT, &distance);
        if (pump_err != ESP_OK) {
            ESP_LOGW(TANK_LEVEL_TAG, "Error code %#x", pump_err);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        int avg_dis = avg_add(&tank_level_filter, distance);

        int water_level_cm = distance - TANK_SENSOR_OFFSET;
        char water_level =
            (char)(((TANK_HEIGHT - water_level_cm) * 100) / TANK_HEIGHT);

        esp_log_level_t log_level = ESP_LOG_INFO;
        if (water_level <= TANK_ALERT_MIN || water_level >= TANK_ALERT_MAX) {
            log_level = ESP_LOG_WARN;
        }

        ESP_LOG_LEVEL_LOCAL(log_level, TANK_LEVEL_TAG,
                            "RAW:%d cm | AVG:%d cm | LEVEL:%d", distance,
                            avg_dis, water_level);

        // Gửi dữ liệu lên Blynk (comming soon)
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void) {
    ultrasonic_init(&pump_sen);
    ultrasonic_init(&tank_level_sen);

    // xTaskCreate(pump_control, "pump_control", 4096, NULL, 10, NULL);
    // xTaskCreate(log_pump_data, "log_pump_data", 4069, NULL, 5, NULL);
    xTaskCreate(check_tank_level, "check_tank_level", 4096, &tank_level_sen, 5,
                NULL);
}
