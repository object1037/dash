#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "led_strip.h"

typedef struct {
  uint8_t r, g, b;
} rgb_t;

esp_err_t blink_led(led_strip_handle_t* led_strip, rgb_t* color);
esp_err_t configure_led(led_strip_handle_t* led_strip, uint8_t pin);
