#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <stdio.h>

#include "esp_lcd_panel_ssd1680.h"
#include "led.h"
#include "scd40.h"
#include "ui.h"

static const char *TAG = "App";

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

#define MEASURE_PER_MINUTE 2
#define FILTER_ALPHA 0.4f
#define GRAPH_WIDTH 240

#define TEMP_OFFSET -1.0f

led_strip_handle_t led_strip;
rgb_t color_white = {16, 16, 16};
rgb_t color_black = {0, 0, 0};
scd40_measurement_t meas_last_minutes[MEASURE_PER_MINUTE - 1] = {{
    .temperature = 0.0f,
    .humidity = 0.0f,
    .co2 = 0,
}};
scd40_measurement_t meas_digest[3] = {{
    .temperature = 0.0f,
    .humidity = 0.0f,
    .co2 = 0,
}};
scd40_measurement_t meas_history[GRAPH_WIDTH] = {{
    .temperature = 0.0f,
    .humidity = 0.0f,
    .co2 = 0,
}};
int counter = 0;
bool first_run = true;
bool first_quarter = true;

// --- display callbacks

IRAM_ATTR bool epaper_flush_ready_callback(const esp_lcd_panel_handle_t handle,
                                           const void *edata, void *user_data) {
  SemaphoreHandle_t *panel_refreshing_sem_ptr = user_data;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(*panel_refreshing_sem_ptr, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
    return true;
  }
  return false;
}

void get_minmax_meas(scd40_measurement_t *min_meas,
                     scd40_measurement_t *max_meas) {
  min_meas->temperature = 130.0f;
  min_meas->humidity = 100.0f;
  min_meas->co2 = 0xFFFF;
  max_meas->temperature = -45.0f;
  max_meas->humidity = 0.0f;
  max_meas->co2 = 0;
  for (int i = 0; i < GRAPH_WIDTH; i++) {
    scd40_measurement_t meas = meas_history[i];
    if (meas.temperature == 0.0f && meas.humidity == 0.0f && meas.co2 == 0) {
      continue;
    }
    if (meas.temperature < min_meas->temperature) {
      min_meas->temperature = meas.temperature;
    }
    if (meas.temperature > max_meas->temperature) {
      max_meas->temperature = meas.temperature;
    }
    if (meas.humidity < min_meas->humidity) {
      min_meas->humidity = meas.humidity;
    }
    if (meas.humidity > max_meas->humidity) {
      max_meas->humidity = meas.humidity;
    }
    if (meas.co2 < min_meas->co2) {
      min_meas->co2 = meas.co2;
    }
    if (meas.co2 > max_meas->co2) {
      max_meas->co2 = meas.co2;
    }
  }
}

void update_minmax_digest() {
  scd40_measurement_t min_meas, max_meas;
  get_minmax_meas(&min_meas, &max_meas);
  meas_digest[1] = min_meas;
  meas_digest[2] = max_meas;
}

void set_minute_digest(scd40_measurement_t *latest_meas) {
  float temp_sum = latest_meas->temperature;
  float rh_sum = latest_meas->humidity;
  uint32_t co2_sum = latest_meas->co2;
  for (int i = 0; i < MEASURE_PER_MINUTE - 1; i++) {
    temp_sum += meas_last_minutes[i].temperature;
    rh_sum += meas_last_minutes[i].humidity;
    co2_sum += meas_last_minutes[i].co2;
  }
  meas_digest[0].temperature = temp_sum / (float)(MEASURE_PER_MINUTE);
  meas_digest[0].humidity = rh_sum / (float)(MEASURE_PER_MINUTE);
  meas_digest[0].co2 = co2_sum / MEASURE_PER_MINUTE;
}

float filter(float prev_value, float new_meas) {
  return FILTER_ALPHA * prev_value + (1 - FILTER_ALPHA) * new_meas;
}

void add_history(int minutes) {
  scd40_measurement_t meas_prev =
      meas_history[(minutes + GRAPH_WIDTH - 1) % GRAPH_WIDTH];
  if (meas_prev.temperature == 0.0f && meas_prev.humidity == 0.0f &&
      meas_prev.co2 == 0) {
    meas_prev = meas_digest[0];
  }
  meas_history[minutes].temperature =
      filter(meas_prev.temperature, meas_digest[0].temperature);
  meas_history[minutes].humidity =
      filter(meas_prev.humidity, meas_digest[0].humidity);
  meas_history[minutes].co2 =
      (uint16_t)(filter(meas_prev.co2, meas_digest[0].co2));
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
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
                                           &io_config, &io_handle));
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
  ESP_ERROR_CHECK(
      esp_lcd_new_panel_ssd1680(io_handle, &panel_config, &panel_handle));

  // --- Reset the display
  ESP_LOGI(TAG, "Resetting e-Paper display...");
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  vTaskDelay(100 / portTICK_PERIOD_MS);
  // --- Initialize panel
  ESP_LOGI(TAG, "Initializing e-Paper display...");
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

  uint8_t *draw_buf = init_ui_buffer();

  static SemaphoreHandle_t panel_refreshing_sem;
  panel_refreshing_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(panel_refreshing_sem);

  // --- Register the e-Paper refresh done callback
  epaper_panel_callbacks_t cbs = {.on_epaper_refresh_done =
                                      epaper_flush_ready_callback};
  epaper_panel_register_event_callbacks(panel_handle, &cbs,
                                        &panel_refreshing_sem);

  // --- Initialize I2C and SCD40
  ESP_LOGI(TAG, "Initialize I2C bus");
  i2c_master_bus_config_t i2c_buscfg = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = I2C_NUM_0,
      .scl_io_num = PIN_NUM_SCL,
      .sda_io_num = PIN_NUM_SDA,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };
  i2c_master_bus_handle_t i2c_handle;
  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_buscfg, &i2c_handle));

  i2c_device_config_t scd40_dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = SCD40_ADDR,
      .scl_speed_hz = 100000,
  };

  i2c_master_dev_handle_t scd40_handle;
  ESP_ERROR_CHECK(
      i2c_master_bus_add_device(i2c_handle, &scd40_dev_config, &scd40_handle));
  vTaskDelay(pdMS_TO_TICKS(100));

  blink_led(&led_strip, &color_white);

  // Sleep for SCD40 bootup
  vTaskDelay(pdMS_TO_TICKS(1000));
  while (i2c_master_probe(i2c_handle, SCD40_ADDR, -1) != ESP_OK) {
    ESP_LOGI(TAG, "Waiting for SCD40 to boot...");
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  blink_led(&led_strip, &color_black);
  ESP_LOGI(TAG, "Start SCD40 periodic measurement");
  ESP_ERROR_CHECK(scd40_start_lp_measurement(scd40_handle));

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (!scd40_get_data_ready(scd40_handle)) {
      continue;
    }

    // Approx. every 30 seconds
    int minutes = counter / MEASURE_PER_MINUTE;
    int minute_idx = counter % MEASURE_PER_MINUTE;
    if (minute_idx != 0) {
      scd40_read_measurement(scd40_handle, &meas_last_minutes[minute_idx - 1],
                             TEMP_OFFSET);
    } else { // every minute
      scd40_measurement_t meas;
      scd40_read_measurement(scd40_handle, &meas, TEMP_OFFSET);

      if (first_run) {
        meas_digest[0] = meas;
      } else {
        set_minute_digest(&meas);
      }

      if (!first_quarter) {
        // Don't use the first 15 minutes of data
        add_history(minutes);
        update_minmax_digest();
      }

      ESP_LOGI(TAG, "(%d) CO2: %d ppm, Temp: %.2f C, RH: %.2f %%", counter,
               meas_digest[0].co2, meas_digest[0].temperature,
               meas_digest[0].humidity);

      first_run = false;
    }

    if (counter % (MEASURE_PER_MINUTE * 15) == 0) { // every 15 minutes
      ESP_LOGI(TAG, "[%d] Update Graph", minutes);
      draw_ui(meas_digest, meas_history, minutes, draw_buf);
      refresh_panel(panel_handle, panel_refreshing_sem, draw_buf);
      if (counter > 0) {
        first_quarter = false;
      }
    }

    counter++;
    if (counter == (MEASURE_PER_MINUTE * 60 * 4)) { // 4 hours
      counter = 0;
    }
  }
}
