#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_lcd_panel_io.h"
#include "scd40.h"

#define EPD_WIDTH 128
#define EPD_HEIGHT 296
#define COLS_BYTE (EPD_WIDTH / 8)
#define BUF_SIZE (EPD_HEIGHT * EPD_WIDTH / 8)
#define TO_IDX(x, y) ((y) * COLS_BYTE + (x))

uint8_t *init_ui_buffer();
void draw_ui(esp_lcd_panel_handle_t panel_handle, scd40_measurement_t meas_data[3], SemaphoreHandle_t panel_refreshing_sem, uint8_t *draw_buf);
