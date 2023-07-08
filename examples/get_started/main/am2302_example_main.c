/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "am2302_rmt.h"

// GPIO assignment
#define AM2302_GPIO  2

static const char *TAG = "example";

void app_main(void)
{
    am2302_config_t am2302_config = {
        .gpio_num = AM2302_GPIO,
    };
    am2302_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
    };
    am2302_handle_t sensor = NULL;
    ESP_ERROR_CHECK(am2302_new_sensor_rmt(&am2302_config, &rmt_config, &sensor));

    ESP_LOGI(TAG, "Start reading temperature and humidity from AM2302 sensor");
    float temperature = 0;
    float humidity = 0;
    while (1) {
        // the delay between each sensor read is required by the data sheet
        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP_ERROR_CHECK(am2302_read_temp_humi(sensor, &temperature, &humidity));
        ESP_LOGI(TAG, "Temperature: %.1f °C, Humidity: %.1f %%", temperature, humidity);
    }
}
