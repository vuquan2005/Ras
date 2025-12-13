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

#define PUMP_PIN 2
#define TRIG_PIN 13
#define ECHO_PIN 12
#define TRIG_PIN_TANK 0
#define ECHO_PIN_TANK 0

#define PUMP_ON_DISTANCE 5
#define PUMP_OFF_DISTANCE 25
#define PUMP_MAX_DISTANCE 30
#define Tank_MAX_DISTANCE 100

const static char *TAG = "RAS";
const char *PUMP_SENSOR_TAG = "Pump sensor";
const char *TANK_LEVEL_TAG = "Tank sensor";


static esp_err_t pump_err;
static bool isPumpOn = false;
static int pump_distance;

static ultrasonic_t pump_sen = {.trig_pin = TRIG_PIN, .echo_pin = ECHO_PIN};
static ultrasonic_t tank_level_sen = {.trig_pin = TRIG_PIN_TANK,
                                      .echo_pin = ECHO_PIN_TANK};

void pump_control(void *pvParameters) {
    while (true) {
        pump_err =
            ultrasonic_measure(&pump_sen, PUMP_MAX_DISTANCE, &pump_distance);

        if (pump_err == ESP_OK) {
            if (pump_distance <= PUMP_ON_DISTANCE) {
                isPumpOn = true;
            } else if (pump_distance >= PUMP_OFF_DISTANCE) {
                isPumpOn = false;
            }
            gpio_set_level(PUMP_PIN, isPumpOn);
        } else {
            gpio_set_level(PUMP_PIN, 0);
        }

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

        ESP_LOGI(PUMP_SENSOR_TAG, "%d  %d", pump_distance, isPumpOn);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void check_tank_level(void *pvParameters) {
    while (true) {
        int distance;
        pump_err = ultrasonic_measure(&tank_level_sen, PUMP_MAX_DISTANCE, &distance);
        if (pump_err != ESP_OK) {
            ESP_LOGW(TANK_LEVEL_TAG, "Error code %#x", pump_err);
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void) {
    ultrasonic_init(&pump_sen);
    ultrasonic_init(&tank_level_sen);

    xTaskCreate(pump_control, "pump_control", 4096, NULL, 10, NULL);
    xTaskCreate(log_pump_data, "log_pump_data", 4069, NULL, 5, NULL);
    // xTaskCreate(check_tank_level, "check_tank_level", 4096,
    // &tank_level_sen, 5,
    //             NULL);
}
