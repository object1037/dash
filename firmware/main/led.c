#include "led.h"
#include "esp_log.h"

esp_err_t blink_led(led_strip_handle_t* led_strip, rgb_t* color) {
  if (color->r || color->g || color->b) {
    /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
    led_strip_set_pixel(*led_strip, 0, color->r, color->g, color->b);
    /* Refresh the strip to send data */
    return led_strip_refresh(*led_strip);
  } else {
    /* Set all LED off to clear all pixels */
    return led_strip_clear(*led_strip);
  }
}

esp_err_t configure_led(led_strip_handle_t* led_strip, uint8_t pin) {
  /* LED strip initialization with the GPIO and pixels number*/
  led_strip_config_t strip_config = {
      .strip_gpio_num = pin,
      .max_leds = 1,  // at least one LED on board
  };

  led_strip_rmt_config_t rmt_config = {
      .resolution_hz = 10 * 1000 * 1000,  // 10MHz
      .flags.with_dma = false,
  };
  ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, led_strip));
  /* Set all LED off to clear all pixels */
  return led_strip_clear(*led_strip);
}
