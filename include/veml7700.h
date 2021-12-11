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
 * @brief Get the default sensor configuration.
 * 
 * @note Default values implemented are chosen per official recommendation.
 * 
 * @return struct veml7700_config 
 */
static struct veml7700_config veml7700_get_default_config();

/**
 * @brief Auto-resolution adjustment algorithm implementation for
 * VEML7700 light sensor.
 * 
 * @note  Does not match official recommended algorithm.
 * 
 * @param lux Luminocity value for which we are optimizing.
 * 
 * @return esp_err_t 
 */
static esp_err_t veml7700_optimize_configuration(uint32_t *lux);

/**
 * @brief Read the maximum lux for the currentl configuration.
 * 
 * @return uint32_t The maximum lux value.
 */
static uint32_t veml7700_get_current_maximum_lux();

/**
 * @brief Get the next smallest maximum lux limit value.
 * 
 * Used to identify if a better range is possible for the current 
 * light levels.
 * 
 * @param lux The referent lux value, usually the last result.
 * 
 * @return uint32_t The next smallest maximum lux value.
 */
static uint32_t veml7700_get_lower_maximum_lux(uint32_t* lux);

/**
 * @brief Get the smallest possible maximum lux value for this sensor.
 * 
 * @return uint32_t The smallest maximum lux value.
 */
static uint32_t veml7700_get_lowest_maximum_lux();

/**
 * @brief The maximum possible lux value for any configuration on this
 * sensor.
 * 
 * @return uint32_t The maximum lux value.
 */
static uint32_t veml7700_get_maximum_lux();

/**
 * @brief Get the index of the gain value within the list of possible
 * gain values.
 * 
 * @param gain The gain value to search for.
 * 
 * @return int The index within the array of possible gains.
 */
static int veml7700_get_gain_index(uint8_t gain);

/**
 * @brief Get the index of the gain value within the list of possible
 * integration time values.
 * 
 * @param gain The integration time value to search for.
 * 
 * @return int The index within the array of possible integration times.
 */
static int veml7700_get_it_index(uint8_t integration_time);

/**
 * @brief Find the index of a given element within an array.
 * 
 * This is a standard implementation of a commonly used function which can be
 * found online.
 * 
 * @param elm		Value of the element we are searching for
 * @param ar 		The array in which to search
 * @param len 		Length of the given array
 * 
 * @return uint8_t 
 * 		- n Index within the array
 * 		- -1 Element not found.
 */
static uint8_t indexOf(uint8_t elm, uint8_t *ar, uint8_t len);

/**
 * @brief Decrease either gain and/or integration time in configuration.
 * 
 * @note  Does not match official recommended algorithm.
 * 
 * @return void 
 */
static void decrease_resolution();

/**
 * @brief Increase either gain and/or integration time in configuration.
 * 
 * @note Does not match official recommended algorithm.
 * 
 * @return void 
 */
static void increase_resolution();

/**
 * @brief I2C read protocol implementation for VEML7700 IC.
 * 
 * @note Implementation as per official specification by Vishay found
 * on page 6 of the [VEML7700 Datasheet] (https://www.vishay.com/docs/84286/veml7700.pdf), rev. 1.5
 * 
 * @param dev_addr The address of the I2C slave device
 * @param reg_addr The register address from which to read
 * @param reg_data The data read from the desired register
 * @param len Length of data to be read in bytes
 * 
 * @return esp_err_t 
 */
static esp_err_t veml7700_i2c_read(uint8_t *dev_addr, uint8_t reg_addr, uint16_t *reg_data, uint8_t len);

/**
 * @brief I2C write protocol implementation for VEML7700 IC.
 * 
 * @note Implementation as per official specification by Vishay found
 * on page 6 of the [VEML7700 Datasheet] (https://www.vishay.com/docs/84286/veml7700.pdf), rev. 1.5
 * 
 * @param dev_addr The address of the I2C slave device
 * @param reg_addr The register address to which to write
 * @param reg_data The data to be written
 * @param len Length of data to be written in bytes
 * 
 * @return esp_err_t 
 */
static esp_err_t veml7700_i2c_write(uint8_t *dev_addr, uint8_t reg_addr, uint16_t *reg_data, uint8_t len);

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