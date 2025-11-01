#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "string.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_ssd1680.h"

#define EPD_WIDTH 128
#define EPD_HEIGHT 296
#define BUF_SIZE (EPD_HEIGHT * EPD_WIDTH / 8)

uint8_t* init_ui_buffer();
void draw_ui(esp_lcd_panel_handle_t panel_handle, SemaphoreHandle_t panel_refreshing_sem, uint8_t* draw_buf);
