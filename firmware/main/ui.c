#include "ui.h"

uint8_t* init_ui_buffer() {
  uint8_t* draw_buf = heap_caps_malloc(BUF_SIZE, MALLOC_CAP_DMA);
  assert(draw_buf);
  memset(draw_buf, 0x00, BUF_SIZE);
  return draw_buf;
}

void draw_ui(esp_lcd_panel_handle_t panel_handle, SemaphoreHandle_t panel_refreshing_sem, uint8_t* draw_buf) {
  for (int i = 0; i < BUF_SIZE; i++) {
    int x = i % 16;
    int y = i / 16;

    // Horizontal lines
    if (x == 5) {
      draw_buf[i] |= 0x02;
    } else if (x == 10) {
      draw_buf[i] |= 0x01;
    }

    // Vertical lines
    if (y == 56) {
      draw_buf[i] |= 0xFF;
    }

    if (y > 56 && (y - 56) % 60 == 0) {
      if (x == 5) {
        draw_buf[i] |= 0x07;
      } else if (x == 10) {
        draw_buf[i] |= 0x03;
      } else if (x == 11) {
        draw_buf[i] |= 0x80;
      }
    }
  }

  xSemaphoreTake(panel_refreshing_sem, portMAX_DELAY);
  ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, EPD_WIDTH, EPD_HEIGHT, draw_buf));
  ESP_ERROR_CHECK(epaper_panel_refresh_screen(panel_handle));
}
