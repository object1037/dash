#include "ui.h"

uint8_t* init_ui_buffer() {
  uint8_t* draw_buf = heap_caps_malloc(BUF_SIZE, MALLOC_CAP_DMA);
  assert(draw_buf);
  memset(draw_buf, 0x00, BUF_SIZE);
  return draw_buf;
}

void draw_font(int start_x, int start_y, const uint8_t* font, uint8_t* buf) {
  for (int offset_x = 0; offset_x < 3; offset_x++) {
    for (int offset_y = 0; offset_y < 14; offset_y++) {
      buf[TO_IDX(start_x + offset_x, start_y + offset_y)] |= font[offset_y * 3 + offset_x];
    }
  }
}

void draw_ui(esp_lcd_panel_handle_t panel_handle, SemaphoreHandle_t panel_refreshing_sem, uint8_t* buf) {
  for (int i = 0; i < BUF_SIZE; i++) {
    int x = i % 16;
    int y = i / 16;

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

    // Font render
    for (int x = 0; x < 3; x++) {
      for (int y = 0; y < 4; y++) {
        draw_font(1 + x * 5, 3 + y * 12, FONTS[0], buf);
      }
    }

    for (int y = 0; y < 10; y++) {
      draw_font(1, 106 + y * 14, FONTS[y], buf);
    }
  }

  xSemaphoreTake(panel_refreshing_sem, portMAX_DELAY);
  ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, EPD_WIDTH, EPD_HEIGHT, buf));
  ESP_ERROR_CHECK(epaper_panel_refresh_screen(panel_handle));
}
