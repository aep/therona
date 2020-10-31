/* Wi-Fi Provisioning Manager Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>

#include "esp_console.h"


#include "vl53l1_api.h"


static VL53L1_Dev_t sensor0 = {
    .I2cHandle  = I2C_NUM_0,
    .I2cDevAddr = 0x29,
    .comms_speed_khz = 100,
};
static VL53L1_Dev_t sensor1 = {
    .I2cHandle  = I2C_NUM_1,
    .I2cDevAddr = 0x29,
    .comms_speed_khz = 100,
};

static void init_i2c()
{
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0);

    i2c_config_t conf0 = {
        .mode               = I2C_MODE_MASTER,
        .sda_io_num         = 16,
        .sda_pullup_en      = GPIO_PULLUP_ENABLE,
        .scl_io_num         = 17,
        .scl_pullup_en      = GPIO_PULLUP_ENABLE,
        .master.clk_speed   = 100000,
    };
    esp_err_t e = i2c_param_config(sensor0.I2cHandle, &conf0);
    ESP_ERROR_CHECK(e);

    i2c_config_t conf1 = {
        .mode               = I2C_MODE_MASTER,
        .sda_io_num         = 32,
        .sda_pullup_en      = GPIO_PULLUP_ENABLE,
        .scl_io_num         = 33,
        .scl_pullup_en      = GPIO_PULLUP_ENABLE,
        .master.clk_speed   = 100000,
    };
    e = i2c_param_config(sensor1.I2cHandle, &conf1);
    ESP_ERROR_CHECK(e);

    VL53L1_Error ex;

    ex = VL53L1_software_reset(&sensor0);
    if (ex != 0) {
        ESP_LOGE("sensor0", "VL53L1_software_reset %d", ex);
    }
    ex = VL53L1_software_reset(&sensor1);
    if (ex != 0) {
        ESP_LOGE("sensor1", "VL53L1_software_reset %d", ex);
    }

    ex  = VL53L1_WaitDeviceBooted(&sensor0);
    if (ex != 0) {
        ESP_LOGE("sensor0", "VL53L1_WaitDeviceBooted %d", ex);
    }
    ex  = VL53L1_WaitDeviceBooted(&sensor1);
    if (ex != 0) {
        ESP_LOGE("sensor1", "VL53L1_WaitDeviceBooted %d", ex);
    }

    ex  = VL53L1_DataInit(&sensor0);
    if (ex != 0) {
        ESP_LOGE("sensor0", "VL53L1_DataInit %d", ex);
    }
    ex  = VL53L1_DataInit(&sensor1);
    if (ex != 0) {
        ESP_LOGE("sensor1", "VL53L1_DataInit %d", ex);
    }

    ex  = VL53L1_StaticInit(&sensor0);
    if (ex != 0) {
        ESP_LOGE("sensor0", "VL53L1_StaticInit %d", ex);
    }
    ex  = VL53L1_StaticInit(&sensor1);
    if (ex != 0) {
        ESP_LOGE("sensor1", "VL53L1_StaticInit %d", ex);
    }

    ex  = VL53L1_PerformRefSpadManagement(&sensor0);
    if (ex != 0) {
        ESP_LOGE("sensor0", "VL53L1_PerformRefSpadManagement %d", ex);
    }
    ex  = VL53L1_PerformRefSpadManagement(&sensor1);
    if (ex != 0) {
        ESP_LOGE("sensor1", "VL53L1_PerformRefSpadManagement %d", ex);
    }

    ex = VL53L1_SetXTalkCompensationEnable(&sensor0, 0);
    if (ex != 0) {
        ESP_LOGE("sensor0", "VL53L1_PerformRefSpadManagement %d", ex);
    }
    ex = VL53L1_SetXTalkCompensationEnable(&sensor1, 0);
    if (ex != 0) {
        ESP_LOGE("sensor1", "VL53L1_PerformRefSpadManagement %d", ex);
    }

    ex = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&sensor0, 20000);
    if (ex != 0) {
        ESP_LOGE("sensor0", "SetMeasurementTimingBudgetMicroSeconds %d", ex);
    }
    ex = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&sensor1, 20000);
    if (ex != 0) {
        ESP_LOGE("sensor1", "SetMeasurementTimingBudgetMicroSeconds %d", ex);
    }

    ex = VL53L1_SetInterMeasurementPeriodMilliSeconds(&sensor0,  25);
    if (ex != 0) {
        ESP_LOGE("sensor0", "SetInterMeasurementPeriodMilliSeconds %d", ex);
    }
    ex = VL53L1_SetInterMeasurementPeriodMilliSeconds(&sensor1,  25);
    if (ex != 0) {
        ESP_LOGE("sensor1", "SetInterMeasurementPeriodMilliSeconds %d", ex);
    }


    ex = VL53L1_SetDistanceMode(&sensor0, 2);
    if (ex != 0) {
        ESP_LOGE("sensor0", "VL53L1_SetDistanceMode %d", ex);
    }
    ex = VL53L1_SetDistanceMode(&sensor1, 2);
    if (ex != 0) {
        ESP_LOGE("sensor1", "VL53L1_SetDistanceMode %d", ex);
    }




    VL53L1_DeviceInfo_t devinfo;
    ex = VL53L1_GetDeviceInfo(&sensor0, &devinfo);
    if (ex != 0) {
        ESP_LOGE("sensor0", "VL53L1_GetDeviceInfo %d", ex);
    } else {
        ESP_LOGI("sensor0", " %s version: %d.%d",
            devinfo.ProductId,
            devinfo.ProductRevisionMajor,
            devinfo.ProductRevisionMinor
        );
    }

}
static void sensor_start() {
    VL53L1_Error ex;

    ex = VL53L1_StartMeasurement(&sensor0);
    if (ex != 0) {
        ESP_LOGE("sensor0", "VL53L1_StartMeasurement%d", ex);
    }
    ex = VL53L1_StartMeasurement(&sensor1);
    if (ex != 0) {
        ESP_LOGE("sensor1", "VL53L1_StartMeasurement%d ", ex);
    }
}

static void sensor_read(const char *tag, VL53L1_Dev_t *sensor) {
    VL53L1_Error ex;

    ex = VL53L1_WaitMeasurementDataReady(sensor);
    if (ex != 0) {
        ESP_LOGE(tag, "VL53L1_WaitMeasurementDataReady %d", ex);
    }

    VL53L1_RangingMeasurementData_t m = {0};
    ex = VL53L1_GetRangingMeasurementData(sensor, &m);
    if (ex != 0) {
        ESP_LOGE(tag, "VL53L1_GetRangingMeasurementData %d", ex);
    }

    VL53L1_ClearInterruptAndStartMeasurement(sensor);

    ESP_LOGI(tag, "range: %d %d", m.RangeStatus, m.RangeMilliMeter);

}

void app_main(void)
{
    init_i2c();

    sensor_start();

    for (;;) {
        sensor_read("sensor0", &sensor0);
        sensor_read("sensor1", &sensor1);
    }



}
