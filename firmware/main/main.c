#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "led.h"
#include "esp_lcd_panel_ssd1680.h"
#include "ui.h"
#include "scd40.h"

static const char* TAG = "App";

#define LCD_HOST SPI2_HOST

#define PIN_NUM_LED 2
#define PIN_NUM_SCLK 6
#define PIN_NUM_MOSI 7
#define PIN_NUM_EPD_DC 4
#define PIN_NUM_EPD_CS 5
#define PIN_NUM_EPD_RST 8
#define PIN_NUM_EPD_BUSY 10
#define PIN_NUM_SCL 0
#define PIN_NUM_SDA 1

#define LCD_CMD_BITS 8
#define LCD_PARAM_BITS 8

led_strip_handle_t led_strip;
rgb_t color_white = {16, 16, 16};
rgb_t color_black = {0, 0, 0};
scd40_measurement_t measurement;

// --- display callbacks

IRAM_ATTR bool epaper_flush_ready_callback(const esp_lcd_panel_handle_t handle, const void* edata, void* user_data) {
  SemaphoreHandle_t* panel_refreshing_sem_ptr = user_data;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(*panel_refreshing_sem_ptr, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
    return true;
  }
  return false;
}

void app_main(void) {
  /* Configure the peripheral according to the LED type */
  ESP_ERROR_CHECK(configure_led(&led_strip, PIN_NUM_LED));

  ESP_LOGI(TAG, "Initialize SPI bus");
  spi_bus_config_t spi_buscfg = {
      .sclk_io_num = PIN_NUM_SCLK,
      .mosi_io_num = PIN_NUM_MOSI,
      .miso_io_num = -1,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = EPD_HEIGHT * EPD_WIDTH / 8,
  };
  ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &spi_buscfg, SPI_DMA_CH_AUTO));

  ESP_LOGI(TAG, "Install panel IO");
  esp_lcd_panel_io_handle_t io_handle = NULL;
  esp_lcd_panel_io_spi_config_t io_config = {
      .dc_gpio_num = PIN_NUM_EPD_DC,
      .cs_gpio_num = PIN_NUM_EPD_CS,
      .pclk_hz = 20 * 1000 * 1000,
      .lcd_cmd_bits = LCD_CMD_BITS,
      .lcd_param_bits = LCD_PARAM_BITS,
      .spi_mode = 0,
      .trans_queue_depth = 10,
      .on_color_trans_done = NULL,
  };
  // --- Attach the LCD to the SPI bus
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));
  esp_lcd_panel_handle_t panel_handle = NULL;

  esp_lcd_ssd1680_config_t epaper_ssd1680_config = {
      .busy_gpio_num = PIN_NUM_EPD_BUSY,
      .non_copy_mode = true,
  };
  esp_lcd_panel_dev_config_t panel_config = {
      .reset_gpio_num = PIN_NUM_EPD_RST,
      .flags.reset_active_high = false,
      .vendor_config = &epaper_ssd1680_config,
  };
  gpio_install_isr_service(0);
  ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1680(io_handle, &panel_config, &panel_handle));

  // --- Reset the display
  ESP_LOGI(TAG, "Resetting e-Paper display...");
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  vTaskDelay(100 / portTICK_PERIOD_MS);
  // --- Initialize panel
  ESP_LOGI(TAG, "Initializing e-Paper display...");
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
  vTaskDelay(100 / portTICK_PERIOD_MS);
  // --- Turn on display
  ESP_LOGI(TAG, "Turning e-Paper display on...");
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
  vTaskDelay(100 / portTICK_PERIOD_MS);

  static SemaphoreHandle_t panel_refreshing_sem;
  panel_refreshing_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(panel_refreshing_sem);

  // --- Register the e-Paper refresh done callback
  epaper_panel_callbacks_t cbs = {.on_epaper_refresh_done = epaper_flush_ready_callback};
  epaper_panel_register_event_callbacks(panel_handle, &cbs, &panel_refreshing_sem);

  uint8_t* draw_buf = init_ui_buffer();
  ESP_LOGI(TAG, "Draw UI");
  draw_ui(panel_handle, panel_refreshing_sem, draw_buf);

  ESP_ERROR_CHECK(blink_led(&led_strip, &color_white));

  vTaskDelay(pdMS_TO_TICKS(5000));
  ESP_ERROR_CHECK(blink_led(&led_strip, &color_black));
  ESP_LOGI(TAG, "Go to sleep mode...");

  ESP_LOGI(TAG, "Initialize I2C bus");
  i2c_master_bus_config_t i2c_buscfg = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = I2C_NUM_0,
      .scl_io_num = PIN_NUM_SCL,
      .sda_io_num = PIN_NUM_SDA,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = false,
  };
  i2c_master_bus_handle_t i2c_handle;
  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_buscfg, &i2c_handle));

  i2c_device_config_t scd40_dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = SCD40_ADDR,
      .scl_speed_hz = 100000,
  };

  i2c_master_dev_handle_t scd40_handle;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_handle, &scd40_dev_config, &scd40_handle));

  ESP_LOGI(TAG, "Start SCD40 low power measurement");
  ESP_ERROR_CHECK(scd40_start_lp_measurement(scd40_handle));

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (scd40_get_data_ready(scd40_handle)) {
      scd40_read_measurement(scd40_handle, &measurement);
      ESP_LOGI(TAG, "CO2: %d ppm, Temp: %.2f C, RH: %.2f %%", measurement.co2, measurement.temperature, measurement.humidity);
    }
  }
}
