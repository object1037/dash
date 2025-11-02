#include "ui.h"
#include "esp_err.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_ssd1680.h"
#include "esp_lcd_panel_vendor.h"
#include "fonts.h"
#include "freertos/projdefs.h"
#include "string.h"

uint8_t *init_ui_buffer() {
  uint8_t *draw_buf = heap_caps_malloc(BUF_SIZE, MALLOC_CAP_DMA);
  assert(draw_buf);
  memset(draw_buf, 0x00, BUF_SIZE);
  return draw_buf;
}

void draw_font(int start_x, int start_y, bool is_small, int idx, uint8_t *buf) {
  int font_w = FONT_W;
  int font_h = FONT_H;
  const uint8_t *font = FONTS[idx];
  if (is_small) {
    font_w = FONT_SMALL_W;
    font_h = FONT_SMALL_H;
    font = FONTS_SMALL[idx];
  }

  for (int offset_x = 0; offset_x < font_w; offset_x++) {
    for (int offset_y = 0; offset_y < font_h; offset_y++) {
      buf[TO_IDX(start_x + offset_x, start_y + offset_y)] |=
          font[offset_y * font_w + offset_x];
    }
  }
}

void draw_num(int start_x, int start_y, bool is_small, float number,
              uint8_t *buf) {
  int font_h = FONT_H - 2;
  int dot_offset_h = 0;
  if (is_small) {
    font_h = FONT_SMALL_H - 1;
    dot_offset_h = -1;
  }
  if (number < 100) {
    // XY.Z
    draw_font(start_x, start_y, is_small, (int)(number * 10) % 10, buf); // Z
    start_y += dot_offset_h;
    draw_font(start_x, start_y + font_h, is_small, FONT_DOT, buf); // dot
    start_y += dot_offset_h;
    draw_font(start_x, start_y + font_h * 2, is_small, (int)(number) % 10,
              buf); // Y
    draw_font(start_x, start_y + font_h * 3, is_small, (int)(number / 10),
              buf); // X
  } else {
    // WXYZ
    draw_font(start_x, start_y, is_small, (int)(number) % 10, buf); // Z
    draw_font(start_x, start_y + font_h, is_small, (int)(number / 10) % 10,
              buf); // Y
    draw_font(start_x, start_y + font_h * 2, is_small, (int)(number / 100) % 10,
              buf); // X
    if (number >= 1000) {
      draw_font(start_x, start_y, is_small, (int)(number / 1000) % 10,
                buf); // W
    }
  }
}

void draw_ui(esp_lcd_panel_handle_t panel_handle,
             scd40_measurement_t meas_data[3],
             SemaphoreHandle_t panel_refreshing_sem, uint8_t *buf) {
  for (int i = 0; i < BUF_SIZE; i++) {
    int x = i % COLS_BYTE;
    int y = i / COLS_BYTE;

    // Horizontal lines
    if (x == 5 || x == 10) {
      buf[i] |= 0x01;
      // tick marks
      if (y > 56 && (y - 56) % 60 == 0) {
        buf[i] |= 0x03;
      }
    }

    // Vertical lines
    if (y == 56) {
      buf[i] |= 0xFF;
    }
  }

  // Font render
  // Temperature
  draw_num(1, 3, false, meas_data[0].temperature, buf);
  draw_num(4, 0, true, meas_data[2].temperature, buf);  // max
  draw_font(4, 23, true, FONT_HYPHEN, buf);             // hyphen
  draw_num(4, 30, true, meas_data[1].temperature, buf); // min

  // Humidity
  draw_num(6, 3, false, meas_data[0].humidity, buf);
  draw_num(9, 0, true, meas_data[2].humidity, buf);  // max
  draw_font(9, 23, true, FONT_HYPHEN, buf);          // hyphen
  draw_num(9, 30, true, meas_data[1].humidity, buf); // min

  // CO2
  draw_num(11, 3, false, meas_data[0].co2, buf);
  draw_num(14, 0, true, meas_data[2].co2, buf);  // max
  draw_font(14, 23, true, FONT_HYPHEN, buf);     // hyphen
  draw_num(14, 30, true, meas_data[1].co2, buf); // min

  // Tests
  for (int y = 0; y < 11; y++) {
    draw_font(1, 106 + y * FONT_H, false, y, buf);
  }
  for (int y = 0; y < 11; y++) {
    draw_font(4, 107 + y * FONT_SMALL_H, true, y, buf);
  }

  // --- Turn on display
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
  vTaskDelay(pdMS_TO_TICKS(100));

  xSemaphoreTake(panel_refreshing_sem, portMAX_DELAY);
  ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, EPD_WIDTH,
                                            EPD_HEIGHT, buf));
  ESP_ERROR_CHECK(epaper_panel_refresh_screen(panel_handle));

  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, false));
}
