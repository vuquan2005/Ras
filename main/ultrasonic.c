#include "ultrasonic.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

#define TIMEOUT_US 30000 // 30 ms
#define US_PER_CM 58

esp_err_t ultrasonic_init(const ultrasonic_t *device) {
    if (!device)
        return ESP_ERR_INVALID_ARG;

    gpio_config_t io_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };

    // TRIG output
    io_cfg.mode = GPIO_MODE_OUTPUT;
    io_cfg.pin_bit_mask = (1ULL << device->trig_pin);
    gpio_config(&io_cfg);

    // TRIG output
    io_cfg.mode = GPIO_MODE_INPUT;
    io_cfg.pin_bit_mask = (1ULL << device->echo_pin);
    gpio_config(&io_cfg);

    gpio_set_level(device->trig_pin, 0);

    return ESP_OK;
}

esp_err_t ultrasonic_measure(const ultrasonic_t *device, int max_distance,
                             int *distance) {
    if (!device || !distance)
        return ESP_ERR_INVALID_ARG;

    // Cấm ngắt
    portENTER_CRITICAL(&mux);

    /* Phát xung trigger */
    gpio_set_level(device->trig_pin, 0);
    esp_rom_delay_us(2);
    gpio_set_level(device->trig_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(device->trig_pin, 0);

    // Kiểm tra trig
    if (gpio_get_level(device->echo_pin)) {
        portEXIT_CRITICAL(&mux);
        return (ESP_ERR_ULTRASONIC_PING);
    }

    // Đợi echo lên HIGH
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(device->echo_pin) == 0) {
        if (esp_timer_get_time() - start > TIMEOUT_US) {
            portEXIT_CRITICAL(&mux);
            return ESP_ERR_ULTRASONIC_PING_TIMEOUT;
        }
    }

    // Đo thời gian echo
    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(device->echo_pin) == 1) {
        if (esp_timer_get_time() - echo_start >
            (int64_t)max_distance * US_PER_CM) {
            portEXIT_CRITICAL(&mux);
            return ESP_ERR_ULTRASONIC_ECHO_TIMEOUT;
        }
    }

    int64_t echo_end = esp_timer_get_time();
    uint32_t time_us = echo_end - echo_start;

    // Bỏ cấm ngắt
    portEXIT_CRITICAL(&mux);

    *distance = (int)(time_us / US_PER_CM);

    return ESP_OK;
}
