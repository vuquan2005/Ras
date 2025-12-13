#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <stdio.h>

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

#define PUMP_PIN GPIO_NUM_2
#define TRIG_PIN GPIO_NUM_13
#define ECHO_PIN GPIO_NUM_12

#define MIN_DISTANCE 5
#define MAX_DISTANCE 30
#define TRIG_PULSE_US 10

static const char *TAG = "ESP";
static bool isPumpOn = false;

typedef struct {
    int distance;
    bool pumpState;
} pump_data_t;

static QueueHandle_t pumpQueue;

/* Đo độ rộng xung echo (us) */
static int get_pulse_time_us(gpio_num_t pin) {
    int64_t start = 0, end = 0;
    int timeout = 30000;

    while (gpio_get_level(pin) == 0 && --timeout > 0) {
        esp_rom_delay_us(1);
    }
    if (timeout <= 0)
        return -1;

    start = esp_timer_get_time();

    timeout = 30000;
    while (gpio_get_level(pin) == 1 && --timeout > 0) {
        esp_rom_delay_us(1);
    }
    if (timeout <= 0)
        return -1;

    end = esp_timer_get_time();
    return (int)(end - start);
}

/* Đo khoảng cách (cm) */
static int get_distance(void) {
    gpio_set_level(TRIG_PIN, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(TRIG_PULSE_US);
    gpio_set_level(TRIG_PIN, 0);

    int pulseTime = get_pulse_time_us(ECHO_PIN);
    if (pulseTime < 0)
        return -1;

    return (int)(pulseTime / 58.0);
}

/* Task điều khiển bơm (50ms) */
static void pump_task(void *arg) {
    while (1) {
        int distance = get_distance();

        if (distance > 0) {
            if (distance <= MIN_DISTANCE) {
                isPumpOn = true;
            } else if (distance >= MAX_DISTANCE) {
                isPumpOn = false;
            }
            gpio_set_level(PUMP_PIN, isPumpOn);
        }

        pump_data_t data = {.distance = distance, .pumpState = isPumpOn};
        xQueueSend(pumpQueue, &data, 0);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* Task ghi log (500ms) */
static void log_task(void *arg) {
    pump_data_t data;
    while (1) {
        if (xQueueReceive(pumpQueue, &data, portMAX_DELAY)) {
            if (data.distance < 0) {
                ESP_LOGW(TAG, "Timeout!");
            } else {
                ESP_LOGI(TAG, "%d  %d", data.distance, data.pumpState);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main(void) {
    gpio_config_t io_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };

    // TRIG output
    io_cfg.mode = GPIO_MODE_OUTPUT;
    io_cfg.pin_bit_mask = (1ULL << TRIG_PIN);
    gpio_config(&io_cfg);

    // PUMP output
    io_cfg.pin_bit_mask = (1ULL << PUMP_PIN);
    gpio_config(&io_cfg);

    // ECHO input
    io_cfg.mode = GPIO_MODE_INPUT;
    io_cfg.pin_bit_mask = (1ULL << ECHO_PIN);
    gpio_config(&io_cfg);

    pumpQueue = xQueueCreate(1, sizeof(pump_data_t));

    xTaskCreate(pump_task, "pump_task", 4096, NULL, 7, NULL);
    xTaskCreate(log_task, "log_task", 4096, NULL, 5, NULL);
}
