#pragma once

#include "driver/gpio.h"
#include "esp_err.h"
#include <stdint.h>

#define ESP_ERR_ULTRASONIC_PING         0x200
#define ESP_ERR_ULTRASONIC_PING_TIMEOUT 0x201
#define ESP_ERR_ULTRASONIC_ECHO_TIMEOUT 0x202

typedef struct {
    gpio_num_t trig_pin;
    gpio_num_t echo_pin;
} ultrasonic_t;

esp_err_t ultrasonic_init(const ultrasonic_t *device);

esp_err_t ultrasonic_measure(const ultrasonic_t *device,
                             int max_distance, int *distance);
