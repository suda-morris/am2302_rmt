/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/rmt_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Type of AM2302 sensor handle
 */
typedef struct am2302_t *am2302_handle_t;

/**
 * @brief AM2302 configuration
 */
typedef struct {
    int gpio_num; /*!< GPIO number that consumed by the sensor */
} am2302_config_t;

/**
 * @brief AM2302 RMT specific configuration
 */
typedef struct {
    rmt_clock_source_t clk_src; /*!< RMT clock source */
} am2302_rmt_config_t;

/**
 * @brief Create AM2302 sensor handle with RMT backend
 *
 * @note One sensor utilizes a pair of RMT TX and RX channels
 *
 * @param[in] am2302_config AM2303 specific configuration
 * @param[in] rmt_config RMT specific configuration
 * @param[out] ret_sensor Returned sensor handle
 * @return
 *      - ESP_OK: create sensor handle successfully
 *      - ESP_ERR_INVALID_ARG: create sensor handle failed because of invalid argument
 *      - ESP_ERR_NO_MEM: create sensor handle failed because of out of memory
 *      - ESP_FAIL: create sensor handle failed because some other error
 */
esp_err_t am2302_new_sensor_rmt(const am2302_config_t *am2302_config, const am2302_rmt_config_t *rmt_config, am2302_handle_t *ret_sensor);

/**
 * @brief Delete the sensor handle
 *
 * @param[in] sensor Sensor handle returned from `am2302_new_sensor`
 * @return
 *      - ESP_OK: delete sensor handle successfully
 *      - ESP_ERR_INVALID_ARG: delete sensor handle failed because of invalid argument
 *      - ESP_FAIL: delete sensor handle failed because some other error
 */
esp_err_t am2302_del_sensor(am2302_handle_t sensor);

/**
 * @brief Read temperature and humidity from the sensor
 *
 * @param[in] sensor Sensor handle returned from `am2302_new_sensor`
 * @param[out] temp Temperature in degree Celsius
 * @param[out] humi Humidity in percentage
 * @return
 *      - ESP_OK: read temperature and humidity successfully
 *      - ESP_ERR_INVALID_ARG: read temperature and humidity failed because of invalid argument
 *      - ESP_FAIL: read temperature and humidity failed because some other error
 */
esp_err_t am2302_read_temp_humi(am2302_handle_t sensor, float *temp, float *humi);

#ifdef __cplusplus
}
#endif
