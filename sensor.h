#ifndef AKIRA_FOC__SENSOR_H
#define AKIRA_FOC__SENSOR_H

#include "driver/i2c.h"
#include "esp_log.h"
#include "../../setting/pinSetting.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

struct Sensor_t
{
  Sensor_t() {}
  virtual ~Sensor_t() {};
  double AngShift = 0.;
  void initSensor() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = P_SDA,
        .scl_io_num = P_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {.clk_speed = 400000},
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0 , &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0 , conf.mode, 0, 0, 0));
  };
  
  void setAngBios(double bios){
    AngShift = bios;
  }

  double get_angle() {
    uint8_t data_to_send[1] = {0x0E};
    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_NUM_0, 0x36, data_to_send, sizeof(data_to_send), 10));
    uint8_t data_received[2];
    ESP_ERROR_CHECK(i2c_master_read_from_device(I2C_NUM_0, 0x36, data_received, sizeof(data_received), 10));
    uint16_t combined_data = (data_received[0] << 8) | (data_received[1] & 0xFF);
    double nowAngReal =  (double)combined_data/4096.*360 - AngShift;
    if (nowAngReal < 0.) {nowAngReal += 360.;}
    nowAngReal = nowAngReal*-1;
    return nowAngReal;
  }


};

#endif