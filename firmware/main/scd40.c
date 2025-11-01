#include "scd40.h"

void conv_cmd(uint16_t cmd, uint8_t* buf) {
  buf[0] = cmd >> 8;
  buf[1] = cmd & 0xFF;
}

esp_err_t scd40_start_lp_measurement(i2c_master_dev_handle_t dev_handle) {
  uint8_t cmd[2];
  conv_cmd(SCD40_CMD_START_LP_MEASUREMENT, cmd);
  return i2c_master_transmit(dev_handle, cmd, sizeof(cmd), -1);
}

bool scd40_get_data_ready(i2c_master_dev_handle_t dev_handle) {
  uint8_t cmd[2];
  conv_cmd(SCD40_CMD_GET_DATA_READY_STATUS, cmd);
  ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, cmd, sizeof(cmd), -1));
  vTaskDelay(pdMS_TO_TICKS(2));
  uint8_t response[3];
  ESP_ERROR_CHECK(i2c_master_receive(dev_handle, response, sizeof(response), -1));

  uint16_t response_value = (response[0] << 8) | response[1];
  if ((response_value & 0x07FF) == 0) {
    return false;
  } else {
    return true;
  }
}

void scd40_read_measurement(i2c_master_dev_handle_t dev_handle, scd40_measurement_t* measurement) {
  uint8_t cmd[2];
  conv_cmd(SCD40_CMD_READ_MEASUREMENT, cmd);
  ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, cmd, sizeof(cmd), -1));
  vTaskDelay(pdMS_TO_TICKS(2));
  uint8_t response[9];
  ESP_ERROR_CHECK(i2c_master_receive(dev_handle, response, sizeof(response), -1));

  measurement->co2 = (response[0] << 8) | response[1];
  uint16_t temp_raw = (response[3] << 8) | response[4];
  uint16_t rh_raw = (response[6] << 8) | response[7];

  measurement->temperature = -45.0f + 175.0f * ((float)temp_raw / 65535.0f);
  measurement->humidity = 100.0f * ((float)rh_raw / 65535.0f);
}
