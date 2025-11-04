#include "ui.h"
#include "esp_err.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_ssd1680.h"
#include "esp_lcd_panel_vendor.h"
#include "fonts.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "scd40.h"
#include "string.h"
#include <math.h>
#include <stdint.h>

uint8_t *init_ui_buffer() {
  uint8_t *draw_buf = heap_caps_malloc(BUF_SIZE, MALLOC_CAP_DMA);
  assert(draw_buf);
  memset(draw_buf, 0x00, BUF_SIZE);
  return draw_buf;
}

static void draw_font(int start_x, int start_y, bool is_small, int idx,
                      uint8_t *buf) {
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

static void draw_num(int start_x, int start_y, bool is_small, float number,
                     uint8_t *buf) {
  int font_h = FONT_H - 2;
  int dot_offset_h = 0;
  if (is_small) {
    font_h = FONT_SMALL_H - 1;
    dot_offset_h = -1;
  }
  if (number < 100) {
    // XY.Z
    draw_font(start_x, start_y, is_small, (int)roundf(number * 10) % 10,
              buf); // Z
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
      draw_font(start_x, start_y + font_h * 3, is_small,
                (int)(number / 1000) % 10,
                buf); // W
    }
  }
}

static void draw_graph(int start_x, float trace_min, float trace_max, int type,
                       scd40_measurement_t meas_history[], int history_cursor,
                       uint8_t *buf) {
  float trace_step = 0.05f;
  float trace_scale = trace_max - trace_min;
  float trace_mid = (trace_max + trace_min) / 2.0f;

  if (trace_scale >= 4000) {
    trace_mid = trace_min + 2000.0f;
  }

  if (trace_scale >= 2000) {
    trace_step = 100;
  } else if (trace_scale >= 800) {
    trace_step = 50;
  } else if (trace_scale >= 400) {
    trace_step = 20;
  } else if (trace_scale >= 200) {
    trace_step = 10;
  } else if (trace_scale >= 80) {
    trace_step = 5;
  } else if (trace_scale >= 40) {
    trace_step = 2;
  } else if (trace_scale >= 20) {
    trace_step = 1;
  } else if (trace_scale >= 8) {
    trace_step = 0.5;
  } else if (trace_scale >= 4) {
    trace_step = 0.2;
  } else if (trace_scale >= 2) {
    trace_step = 0.1;
  }

  for (int y = 295; y > 56; y--) {
    int history_idx = (295 - y + history_cursor) % 240;
    scd40_measurement_t meas = meas_history[history_idx];

    float trace_offset;
    if (type == 0) {
      trace_offset = meas.temperature - trace_mid;
    } else if (type == 1) {
      trace_offset = meas.humidity - trace_mid;
    } else {
      trace_offset = meas.co2 - trace_mid;
    }

    if (trace_offset == -trace_mid) {
      continue; // no data
    }

    int steps = (int)(trace_offset / trace_step) + 20;
    if (steps < 0) {
      steps = 0;
    } else if (steps > 40) {
      steps = 40;
    }
    uint64_t tmp = 0;
    tmp |= ((uint64_t)1 << steps) - 1;

    for (int x_offset = 0; x_offset < 5; x_offset++) {
      buf[TO_IDX(start_x + x_offset, y)] |= (tmp >> (4 - x_offset) * 8) & 0xFF;
    }
  }
}

void refresh_panel(esp_lcd_panel_handle_t panel_handle,
                   SemaphoreHandle_t panel_refreshing_sem, uint8_t *buf) {
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
  vTaskDelay(pdMS_TO_TICKS(100));

  xSemaphoreTake(panel_refreshing_sem, portMAX_DELAY);
  ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, EPD_WIDTH,
                                            EPD_HEIGHT, buf));
  ESP_ERROR_CHECK(epaper_panel_refresh_screen(panel_handle));

  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, false));
}

void draw_ui(scd40_measurement_t meas_digest[3],
             scd40_measurement_t meas_history[], int minutes, uint8_t *buf) {
  memset(buf, 0x00, BUF_SIZE);
  for (int i = 0; i < BUF_SIZE; i++) {
    int x = i % COLS_BYTE;
    int y = i / COLS_BYTE;

    // Horizontal lines
    if (x == 5 || x == 10) {
      buf[i] |= 0x01;
      // tick marks
      if (y > 56 && (y - 56) % 60 == 0) {
        buf[i + 1] |= 0x80;
      }
    }

    // Vertical lines
    if (y == 56) {
      buf[i] |= 0xFF;
    }
  }

  // Font render
  // Temperature
  draw_num(1, 3, false, meas_digest[0].temperature, buf);
  draw_num(4, 0, true, meas_digest[2].temperature, buf);  // max
  draw_font(4, 23, true, FONT_HYPHEN, buf);               // hyphen
  draw_num(4, 30, true, meas_digest[1].temperature, buf); // min

  // Humidity
  draw_num(6, 3, false, meas_digest[0].humidity, buf);
  draw_num(9, 0, true, meas_digest[2].humidity, buf);  // max
  draw_font(9, 23, true, FONT_HYPHEN, buf);            // hyphen
  draw_num(9, 30, true, meas_digest[1].humidity, buf); // min

  // CO2
  draw_num(11, 3, false, meas_digest[0].co2, buf);
  draw_num(14, 0, true, meas_digest[2].co2, buf);  // max
  draw_font(14, 23, true, FONT_HYPHEN, buf);       // hyphen
  draw_num(14, 30, true, meas_digest[1].co2, buf); // min

  // Graphs
  draw_graph(1, meas_digest[1].temperature, meas_digest[2].temperature, 0,
             meas_history, minutes + 1, buf);
  draw_graph(6, meas_digest[1].humidity, meas_digest[2].humidity, 1,
             meas_history, minutes + 1, buf);
  draw_graph(11, meas_digest[1].co2, meas_digest[2].co2, 2, meas_history,
             minutes + 1, buf);

  // Tests
  // for (int y = 0; y < 11; y++) {
  //   draw_font(1, 106 + y * FONT_H, false, y, buf);
  // }
  // for (int y = 0; y < 11; y++) {
  //   draw_font(4, 107 + y * FONT_SMALL_H, true, y, buf);
  // }
}
