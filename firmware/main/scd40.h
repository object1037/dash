#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
  uint16_t co2;
  float temperature;
  float humidity;
} scd40_measurement_t;

#define SCD40_ADDR 0x62

#define SCD40_CMD_START_PERIODIC_MEASUREMENT 0x21B1
#define SCD40_CMD_READ_MEASUREMENT 0xEC05
#define SCD40_CMD_STOP_PERIODIC_MEASUREMENT 0x3F86

#define SCD40_CMD_SET_TEMP_OFFSET 0x241D
#define SCD40_CMD_GET_TEMP_OFFSET 0x2318
#define SCD40_CMD_SET_ALTITUDE 0x2427
#define SCD40_CMD_GET_ALTITUDE 0x2322
#define SCD40_CMD_SET_PRESSURE 0xE000

#define SCD40_CMD_FORCE_RECALIBRATION 0x362F
#define SCD40_CMD_SET_AUTO_CALIBRATION 0x2416
#define SCD40_CMD_GET_AUTO_CALIBRATION 0x2313

#define SCD40_CMD_START_LP_MEASUREMENT 0x21AC
#define SCD40_CMD_GET_DATA_READY_STATUS 0xE4B8

#define SCD40_CMD_PERSIST_SETTINGS 0x3615
#define SCD40_CMD_SERIAL_NUMBER 0x3682
#define SCD40_CMD_SELF_TEST 0x3639
#define SCD40_CMD_FACTORY_RESET 0x3632
#define SCD40_CMD_REINIT 0x3646

esp_err_t scd40_start_lp_measurement(i2c_master_dev_handle_t dev_handle);
bool scd40_get_data_ready(i2c_master_dev_handle_t dev_handle);
void scd40_read_measurement(i2c_master_dev_handle_t dev_handle, scd40_measurement_t* measurement);
