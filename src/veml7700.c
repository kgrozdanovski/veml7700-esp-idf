/**
 * @file veml7700.c
 * 
 * @author Kristijan Grozdanovski (kgrozdanovski7@gmail.com)
 * 
 * @brief Vishay VEML7700 Light Sensor driver for integration with ESP-IDF framework.
 * 
 * @version 2
 * 
 * @date 2021-12-11
 * 
 * @copyright Copyright (c) 2022, Kristijan Grozdanovski
 * All rights reserved.
 * 
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree. 
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "veml7700.h"
#include "driver/i2c.h"

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

#define ARRAY_SIZE(arr)	(sizeof(arr) / sizeof((arr)[0]))


/**
 * @brief List of all possible values for configuring sensor gain.
 * 
 */
static const uint8_t gain_values[VEML7700_GAIN_OPTIONS_COUNT] = {
    VEML7700_GAIN_2,
    VEML7700_GAIN_1,
    VEML7700_GAIN_1_8,
    VEML7700_GAIN_1_4
};
/**
 * @brief List of all possible values for configuring sensor integration time.
 * 
 */
static const uint8_t integration_time_values[VEML7700_IT_OPTIONS_COUNT] = {
    VEML7700_IT_800MS,
    VEML7700_IT_400MS,
    VEML7700_IT_200MS,
    VEML7700_IT_100MS,
    VEML7700_IT_50MS,
    VEML7700_IT_25MS
};

/**
 * @brief Proper resolution multipliers mapped to gain-integration time combination.
 * 
 * @note Source: Official Vishay VEML7700 Application Note, rev. 20-Sep-2019
 * 
 * @link https://www.vishay.com/docs/84323/designingveml7700.pdf
 */
static const float resolution_map[VEML7700_IT_OPTIONS_COUNT][VEML7700_GAIN_OPTIONS_COUNT] = {
    {0.0036, 0.0072, 0.0288, 0.0576},
    {0.0072, 0.0144, 0.0576, 0.1152},
    {0.0144, 0.0288, 0.1152, 0.2304},
    {0.0288, 0.0576, 0.2304, 0.4608},
    {0.0576, 0.1152, 0.4608, 0.9216},
    {9.1152, 0.2304, 0.9216, 1.8432}
};
/**
 * @brief Maximum luminocity mapped to gain-integration time combination.
 * 
 * @note Source: Official Vishay VEML7700 Application Note, rev. 20-Sep-2019
 * 
 * @link https://www.vishay.com/docs/84323/designingveml7700.pdf
 */
static const uint32_t maximums_map[VEML7700_IT_OPTIONS_COUNT][VEML7700_GAIN_OPTIONS_COUNT] = {
    {236, 472, 1887, 3775},
    {472, 944, 3775, 7550},
    {944, 1887, 7550, 15099},
    {1887, 3775, 15099, 30199},
    {3775, 7550, 30199, 60398},
    {7550, 15099, 60398, 120796}
};

/**
 * @brief Component tag to be used for ESP LOG messages.
 * 
 */
static const char* VEML7700_TAG = "VEML7700";

/**
 * @brief The default I2C address of the sensor.
 * 
 * There's no capability to change this in hardware, so no need
 * to make this configurable.
 * 
 */
const uint8_t veml7700_device_address = VEML7700_I2C_ADDR;

/**
 * @brief Private data for one sensor.
 */
struct veml7700_privdata_t {
	struct veml7700_config configuration;
	int i2c_master_num;
	int addr;
};

//Forward declarations
static struct veml7700_config veml7700_get_default_config();
static esp_err_t veml7700_optimize_configuration(veml7700_handle_t dev, double *lux);
static uint32_t veml7700_get_current_maximum_lux();
static uint32_t veml7700_get_lower_maximum_lux(veml7700_handle_t dev, double* lux);
static uint32_t veml7700_get_lowest_maximum_lux();
static uint32_t veml7700_get_maximum_lux();
static int veml7700_get_gain_index(uint8_t gain);
static int veml7700_get_it_index(uint8_t integration_time);
static uint8_t indexOf(uint8_t elm, const uint8_t *ar, uint8_t len);
static void decrease_resolution(veml7700_handle_t dev);
static void increase_resolution(veml7700_handle_t dev);
static esp_err_t veml7700_i2c_read_reg(veml7700_handle_t dev, uint8_t reg_addr, uint16_t *reg_data);
static esp_err_t veml7700_i2c_write_reg(veml7700_handle_t dev, uint8_t reg_addr, uint16_t reg_data);
static esp_err_t veml7700_send_config(veml7700_handle_t dev);


/**
 * @brief Get the default sensor configuration.
 * 
 * @note Default values implemented are chosen per official recommendation.
 * 
 * @return struct veml7700_config 
 */
static struct veml7700_config veml7700_get_default_config()
{
	struct veml7700_config configuration;

	configuration.gain = VEML7700_GAIN_1_8;
	configuration.integration_time = VEML7700_IT_100MS;	// Start with 100ms to avoid any saturation effect
	configuration.persistance = VEML7700_PERS_1;
	configuration.interrupt_enable = false;
	configuration.shutdown = VEML7700_POWERSAVE_MODE1;

	return configuration;
}

/**
 * @brief Auto-resolution adjustment algorithm implementation for
 * VEML7700 light sensor.
 * 
 * @note  Does not match official recommended algorithm.
 * 
 * @param dev Handle for the device
 * @param lux Luminocity value for which we are optimizing.
 * 
 * @return esp_err_t 
 */
static esp_err_t veml7700_optimize_configuration(veml7700_handle_t dev, double *lux)
{
	// Make sure this isn't the smallest maximum
	if (dev->configuration.maximum_lux == veml7700_get_lowest_maximum_lux(dev)) {
		ESP_LOGD(VEML7700_TAG, "Already configured with maximum resolution.");
		return ESP_FAIL;
	}
	if (dev->configuration.maximum_lux == veml7700_get_maximum_lux(dev)) {
		ESP_LOGD(VEML7700_TAG, "Already configured for maximum luminocity.");
		return ESP_FAIL;
	}

	if (ceil(*lux) >= veml7700_get_current_maximum_lux(dev)) {
		// Decrease resolution
		decrease_resolution(dev);
	} else if (*lux < veml7700_get_lower_maximum_lux(dev, lux)) {
		// Increase resolution
		increase_resolution(dev);
	} else {
		ESP_LOGD(VEML7700_TAG, "Configuration already optimal.");
		return ESP_FAIL;
	}

	ESP_LOGD(VEML7700_TAG, "Configuration optimized.");
	return ESP_OK;
}

/**
 * @brief Read the maximum lux for the currentl configuration.
 * 
 * @param dev Handle for the device
 * @return uint32_t The maximum lux value.
 */
static uint32_t veml7700_get_current_maximum_lux(veml7700_handle_t dev)
{
	int gain_index = veml7700_get_gain_index(dev->configuration.gain);
	int it_index = veml7700_get_it_index(dev->configuration.integration_time);

	return maximums_map[it_index][gain_index];
}

/**
 * @brief Get the next smallest maximum lux limit value.
 * 
 * Used to identify if a better range is possible for the current 
 * light levels.
 * 
 * @param dev Handle for the device
 * @param lux The referent lux value, usually the last result.
 * 
 * @return uint32_t The next smallest maximum lux value.
 */
static uint32_t veml7700_get_lower_maximum_lux(veml7700_handle_t dev, double* lux)
{
	int gain_index = veml7700_get_gain_index(dev->configuration.gain);
	int it_index = veml7700_get_it_index(dev->configuration.integration_time);

	// Find the next smallest 'maximum' value in the mapped maximum luminocities
	if ((gain_index > 0) && (it_index > 0)) {
		if (maximums_map[it_index][gain_index - 1] >= maximums_map[it_index - 1][gain_index]) {
			return maximums_map[it_index][gain_index - 1];
		} else {
			return maximums_map[it_index - 1][gain_index];
		}
	} else if ((gain_index > 0) && (it_index == 0)) {
		return maximums_map[it_index][gain_index - 1];
	} else {
		return maximums_map[it_index - 1][gain_index];
	}
}

/**
 * @brief Get the smallest possible maximum lux value for this sensor.
 * 
 * @return uint32_t The smallest maximum lux value.
 */
static uint32_t veml7700_get_lowest_maximum_lux()
{
	return maximums_map[0][0];
}

/**
 * @brief The maximum possible lux value for any configuration on this
 * sensor.
 * 
 * @return uint32_t The maximum lux value.
 */
static uint32_t veml7700_get_maximum_lux()
{
	return maximums_map[VEML7700_IT_OPTIONS_COUNT - 1][VEML7700_GAIN_OPTIONS_COUNT - 1];
}

/**
 * @brief Get the index of the gain value within the list of possible
 * gain values.
 * 
 * @param gain The gain value to search for.
 * 
 * @return int The index within the array of possible gains.
 */
static int veml7700_get_gain_index(uint8_t gain)
{
	return indexOf(gain, gain_values, VEML7700_GAIN_OPTIONS_COUNT);
}

/**
 * @brief Get the index of the gain value within the list of possible
 * integration time values.
 * 
 * @param gain The integration time value to search for.
 * 
 * @return int The index within the array of possible integration times.
 */
static int veml7700_get_it_index(uint8_t integration_time)
{
	return indexOf(integration_time, integration_time_values, VEML7700_IT_OPTIONS_COUNT);
}

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
static uint8_t indexOf(uint8_t elm, const uint8_t *ar, uint8_t len)
{
    while (len--) { if (ar[len] == elm) { return len; } } return -1;
}

/**
 * @brief Decrease either gain and/or integration time in configuration.
 * 
 * @note  Does not match official recommended algorithm.
 * 
 * @param dev Handle for the device
 * @return void 
 */
static void decrease_resolution(veml7700_handle_t dev)
{
	// Identify the indexes of the currently configured values
	int gain_index = veml7700_get_gain_index(dev->configuration.gain);
	int it_index = veml7700_get_it_index(dev->configuration.integration_time);

	ESP_LOGD(VEML7700_TAG, "Decreasing sensor resolution...\n");

	// If this is the last gain or integration time setting, increment the other property
	if (gain_index == 3) {
		dev->configuration.integration_time = integration_time_values[it_index + 1];
	} else if (it_index == 5) {
		dev->configuration.gain = gain_values[gain_index + 1];
	} else {
		// Check which adjacent value is bigger than the current, but choose the smaller if both are
		if (resolution_map[it_index + 1][gain_index] > dev->configuration.resolution) {
			if (resolution_map[it_index + 1][gain_index] <= resolution_map[it_index][gain_index + 1]) {
				dev->configuration.integration_time = integration_time_values[it_index + 1];
			}
		} else if (resolution_map[it_index][gain_index + 1] > dev->configuration.resolution) {
			if (resolution_map[it_index][gain_index + 1] <= resolution_map[it_index + 1][gain_index]) {
				dev->configuration.gain = gain_values[gain_index + 1];
			}
		} else {
			ESP_LOGE(VEML7700_TAG, "Failed to decrease sensor resolution.");
		}
	}

	// Update the sensor configuration
	veml7700_send_config(dev);
}

/**
 * @brief Increase either gain and/or integration time in configuration.
 * 
 * @note Does not match official recommended algorithm.
 * 
 * @param dev Handle for the device
 * @return void 
 */
static void increase_resolution(veml7700_handle_t dev)
{
	int gain_index = veml7700_get_gain_index(dev->configuration.gain);
	int it_index = veml7700_get_it_index(dev->configuration.integration_time);

	if ((gain_index > 0) && (it_index > 0)) {
		if (maximums_map[it_index][gain_index - 1] >= maximums_map[it_index - 1][gain_index]) {
			dev->configuration.gain = gain_values[gain_index - 1];
		} else {
			dev->configuration.integration_time = integration_time_values[it_index - 1];
		}
	} else if ((gain_index > 0) && (it_index == 0)) {
		dev->configuration.gain = gain_values[gain_index - 1];
	} else {
		dev->configuration.integration_time = integration_time_values[it_index - 1];
	}

	// Update the sensor configuration
	veml7700_send_config(dev);
}

/**
 * @brief I2C register read protocol implementation for VEML7700 IC.
 * 
 * @note Implementation as per official specification by Vishay found
 * on page 6 of the [VEML7700 Datasheet] (https://www.vishay.com/docs/84286/veml7700.pdf), rev. 1.5
 * 
 * @param dev Handle for the device
 * @param reg_addr The register address from which to read
 * @param reg_data The data read from the desired register
 * 
 * @return esp_err_t 
 */
static esp_err_t veml7700_i2c_read_reg(veml7700_handle_t dev, uint8_t reg_addr, uint16_t *reg_data)
{
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev->addr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg_addr, true);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev->addr << 1) | I2C_MASTER_READ, true);
	
	uint8_t read_data[2];
	i2c_master_read(cmd, read_data, 2, I2C_MASTER_LAST_NACK);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(dev->i2c_master_num, cmd, 2000 / portTICK_PERIOD_MS);

	*reg_data=read_data[0]|(read_data[1]<<8);
	i2c_cmd_link_delete(cmd);

	return espRc;
}

/**
 * @brief I2C register write protocol implementation for VEML7700 IC.
 * 
 * @note Implementation as per official specification by Vishay found
 * on page 6 of the [VEML7700 Datasheet] (https://www.vishay.com/docs/84286/veml7700.pdf), rev. 1.5
 * 
 * @param dev Handle for the device
 * @param reg_addr The register address to which to write
 * @param reg_data The data to be written
 * 
 * @return esp_err_t 
 */
static esp_err_t veml7700_i2c_write_reg(veml7700_handle_t dev, uint8_t reg_addr, uint16_t reg_data)
{
	esp_err_t espRc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev->addr << 1) | I2C_MASTER_WRITE, false);
	i2c_master_write_byte(cmd, reg_addr, false);

	uint8_t write_data[2];
	write_data[0]=reg_data&0xff;
	write_data[1]=(reg_data>>8)&0xff;
	i2c_master_write(cmd, write_data, 2, false);
	
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(dev->i2c_master_num, cmd, 1000 / portTICK_PERIOD_MS);

	i2c_cmd_link_delete(cmd);

	return espRc;
}


esp_err_t veml7700_initialize(veml7700_handle_t *dev, int i2c_master_num)
{
	veml7700_privdata_t *rdev=calloc(sizeof(veml7700_privdata_t), 1);
	if (rdev==NULL) return ESP_ERR_NO_MEM;
	// Define the sensor configuration globally
	rdev->configuration = veml7700_get_default_config();
	rdev->i2c_master_num = i2c_master_num;
	rdev->addr = veml7700_device_address;

	*dev=rdev;
	return veml7700_send_config(rdev);
}

void veml7700_release(veml7700_handle_t dev) {
	//Nothing special to do with the device; simply free the handle memory.
	free(dev);
}

static esp_err_t veml7700_send_config(veml7700_handle_t dev)
{
	uint16_t config_data = ( 
		(dev->configuration.gain << 11) |
		(dev->configuration.integration_time << 6) |
		(dev->configuration.persistance << 4) |
		(dev->configuration.interrupt_enable << 1) |
		(dev->configuration.shutdown << 0)
	);

	// Set the resolution on the configuration struct
	dev->configuration.resolution = veml7700_get_resolution(dev);
	// Set the current maximum value on the configuration struct
	dev->configuration.maximum_lux = veml7700_get_current_maximum_lux(dev);

	return veml7700_i2c_write_reg(
		dev, 
		VEML7700_ALS_CONFIG, 
		config_data
	);
}

esp_err_t veml7700_set_config(veml7700_handle_t dev, struct veml7700_config *configuration)
{
	dev->configuration=*configuration;
	return veml7700_send_config(dev);
}

esp_err_t veml7700_read_als_lux(veml7700_handle_t dev, double* lux)
{
	esp_err_t i2c_result;
	uint16_t reg_data;
	
	i2c_result = veml7700_i2c_read_reg(dev, VEML7700_ALS_DATA, &reg_data);
	if (i2c_result != 0) {
		ESP_LOGW(VEML7700_TAG, "veml7700_i2c_read() returned %d", i2c_result);
		return i2c_result;;
	}

	*lux = reg_data * dev->configuration.resolution;

	return ESP_OK;
}

esp_err_t veml7700_read_als_lux_auto(veml7700_handle_t dev, double* lux)
{
	veml7700_read_als_lux(dev, lux);

	ESP_LOGD(VEML7700_TAG, "Configured maximum luminocity: %d\n", dev->configuration.maximum_lux);
	ESP_LOGD(VEML7700_TAG, "Configured resolution: %0.4f\n", dev->configuration.resolution);
	
	// Calculate and automatically reconfigure the optimal sensor configuration
	esp_err_t optimize = veml7700_optimize_configuration(dev, lux);
	if (optimize == ESP_OK) {
		// Read again
		return veml7700_read_als_lux(dev, lux);
	}

	return ESP_OK;
}

esp_err_t veml7700_read_white_lux(veml7700_handle_t dev, double* lux)
{
	esp_err_t i2c_result;
	uint16_t reg_data;
	
	i2c_result = veml7700_i2c_read_reg(dev, VEML7700_WHITE_DATA, &reg_data);
	if (i2c_result != 0) {
		ESP_LOGW(VEML7700_TAG, "veml7700_i2c_read() returned %d", i2c_result);
		return i2c_result;
	}

	*lux = reg_data * dev->configuration.resolution;

	return ESP_OK;
}

esp_err_t veml7700_read_white_lux_auto(veml7700_handle_t dev, double* lux)
{
	veml7700_read_white_lux(dev, lux);

	ESP_LOGD(VEML7700_TAG, "Configured maximum luminocity: %d\n", dev->configuration.maximum_lux);
	ESP_LOGD(VEML7700_TAG, "Configured resolution: %0.4f\n", dev->configuration.resolution);
	
	// Calculate and automatically reconfigure the optimal sensor configuration
	esp_err_t optimize = veml7700_optimize_configuration(dev, lux);
	if (optimize == ESP_OK) {
		// Read again
		return veml7700_read_white_lux(dev, lux);
	}

	return ESP_OK;
}

float veml7700_get_resolution(veml7700_handle_t dev)
{
	int gain_index = veml7700_get_gain_index(dev->configuration.gain);
	int it_index = veml7700_get_it_index(dev->configuration.integration_time);

	return resolution_map[it_index][gain_index];
}
