/**
 * @file veml7700.h
 * 
 * @author Kristijan Grozdanovski (kgrozdanovski7@gmail.com)
 * 
 * @brief Vishay VEML7700 Light Sensor driver for integration with ESP-IDF framework.
 * 
 * @version 1
 * 
 * @date 2021-12-11
 * 
 * @copyright Copyright (c) 2021, Kristijan Grozdanovski
 * All rights reserved.
 * 
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree. 
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "driver/i2c.h"

#ifndef I2C_MASTER_NUM
#define I2C_MASTER_NUM I2C_NUM_0    	/*!< Port number for I2C master */
#endif

#define VEML7700_I2C_ADDR UINT8_C(0x10) /*!< Sensor slave I2C address */

#define VEML7700_ALS_CONFIG 0x00        /*!< Light configuration register */
#define VEML7700_ALS_THREHOLD_HIGH 0x01 /*!< Light high threshold for irq */
#define VEML7700_ALS_THREHOLD_LOW 0x02  /*!< Light low threshold for irq */
#define VEML7700_ALS_POWER_SAVE 0x03    /*!< Power save register */
#define VEML7700_ALS_DATA 0x04          /*!< The light data output */
#define VEML7700_WHITE_DATA 0x05        /*!< The white light data output */
#define VEML7700_INTERRUPTSTATUS 0x06   /*!< What IRQ (if any) */

#define VEML7700_INTERRUPT_HIGH 0x4000  /*!< Interrupt status for high threshold */
#define VEML7700_INTERRUPT_LOW 0x8000   /*!< Interrupt status for low threshold */

#define VEML7700_GAIN_2 0x01            /*!< ALS gain 2x */
#define VEML7700_GAIN_1 0x00            /*!< ALS gain 1x */
#define VEML7700_GAIN_1_8 0x02          /*!< ALS gain 1/8x */
#define VEML7700_GAIN_1_4 0x03          /*!< ALS gain 1/4x */

#define VEML7700_IT_800MS 0x03          /*!< ALS intetgration time 800ms */
#define VEML7700_IT_400MS 0x02          /*!< ALS intetgration time 400ms */
#define VEML7700_IT_200MS 0x01          /*!< ALS intetgration time 200ms */
#define VEML7700_IT_100MS 0x00          /*!< ALS intetgration time 100ms */
#define VEML7700_IT_50MS 0x08           /*!< ALS intetgration time 50ms */
#define VEML7700_IT_25MS 0x0C           /*!< ALS intetgration time 25ms */

#define VEML7700_PERS_1 0x00            /*!< ALS irq persisance 1 sample */
#define VEML7700_PERS_2 0x01            /*!< ALS irq persisance 2 samples */
#define VEML7700_PERS_4 0x02            /*!< ALS irq persisance 4 samples */
#define VEML7700_PERS_8 0x03            /*!< ALS irq persisance 8 samples */

#define VEML7700_POWERSAVE_MODE1 0x00   /*!< Power saving mode 1 */
#define VEML7700_POWERSAVE_MODE2 0x01   /*!< Power saving mode 2 */
#define VEML7700_POWERSAVE_MODE3 0x02   /*!< Power saving mode 3 */
#define VEML7700_POWERSAVE_MODE4 0x03   /*!< Power saving mode 4 */

#define VEML7700_GAIN_OPTIONS_COUNT 4	/*!< Possible gain values count */
#define VEML7700_IT_OPTIONS_COUNT 6		/*!< Possible integration time values count */

#define LUX_FC_COEFFICIENT 0.092903     /*!< Multiplier coefficient for lux-fc conversion */
#define FC_LUX_COEFFICIENT 10.7639      /*!< Multiplier coefficient for fc-lux conversion */

#define ARRAY_SIZE(arr)	(sizeof(arr) / sizeof((arr)[0]))

/**
 * @brief Represents sensor and I2C device configuration.
 * 
 * @note Can contain calculated values for active settings such
 * as 'resolution' and 'maximum lux'.
 * 
 */
static struct veml7700_config {
	uint16_t gain;				/*!< Sensor gain configuration */
	uint16_t integration_time;	/*!< Sensor integration time configuration */
	uint16_t persistance;		/*!< Last result persistance on-sensor configuration */
	uint16_t interrupt_enable;	/*!< Enable/disable interrupts */
	uint16_t shutdown;			/*!< Shutdown command configuration */
    float resolution;			/*!< Current resolution and multiplier */
    uint32_t maximum_lux;		/*!< Current maximum lux limit */
};

/**
 * @brief Initialize the sensor by configuring it with default settings.
 * 
 * @attention This function should be called only once and before any other
 * functions from this group.
 * 
 * @return esp_err_t 
 */
esp_err_t veml7700_initialize();

/**
 * @brief Write the sensor configuration to the device.
 * 
 * @param configuration The configuration to be written.
 * 
 * @return esp_err_t 
 */
esp_err_t veml7700_set_config(struct veml7700_config *configuration);

/**
 * @brief Read the ALS data once.
 * 
 * @param lux The ALS read result in lux.
 * 
 * @return esp_err_t 
 */
esp_err_t veml7700_read_als_lux(double* lux);

/**
 * @brief Read the ALS data once. Optimize resolution if neccessary.
 * 
 * @attention This function alters the sensor configuration as it sees fit.
 * 
 * @param lux The ALS read result in lux.
 * 
 * @return esp_err_t 
 */
esp_err_t veml7700_read_als_lux_auto(double* lux);

/**
 * @brief Read the White light data once.
 * 
 * @param lux The White light read result in lux.
 * 
 * @return esp_err_t 
 */
esp_err_t veml7700_read_white_lux(double* lux);

/**
 * @brief Read the White light data once. Optimize resolution if neccessary.
 * 
 * @attention This function alters the sensor configuration as it sees fit.
 * 
 * @param lux The White light read result in lux.
 * 
 * @return esp_err_t 
 */
esp_err_t veml7700_read_white_lux_auto(double* lux);

/**
 * @brief Read the currently configured and used sensor resolution.
 * 
 * @return float The resolution value.
 */
float veml7700_get_resolution();

#ifdef __cplusplus
}
#endif