/**
 * @file veml7700_light_sensor.c
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
#include "esp_event.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/i2c.h"

#include "veml7700.h"

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

/**
 * Function prototypes
 */
void i2c_master_setup(void);
void task_veml7700_read(void *ignore);

/**
 * @brief Standard application entry point.
 * 
 */
void app_main()
{
	// Must call before wifi/http functions
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	// Initialize I2C
	i2c_master_setup();

	xTaskCreate(&task_read_veml7700, "task_read_veml7700",  4096, NULL, 6, NULL);
}

/**
 * @brief Configure the ESP host as an I2C master device.
 * 
 */
void i2c_master_setup(void)
{
	i2c_config_t conf;

	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = SDA_PIN;
	conf.scl_io_num = SCL_PIN;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
	conf.clk_flags = 0;

	i2c_param_config(I2C_MASTER_NUM, &conf);
	i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
}

/**
 * @brief FreeRTOS-compatible task to periodically read and print sensor data.
 * 
 * @param ignore Ignore
 */
void task_veml7700_read(void *ignore)
{ 
	veml7700_handle_t veml7700_dev;

	esp_err_t init_result = veml7700_initialize(&veml7700_dev, I2C_MASTER_NUM);
	if (init_result != ESP_OK) {
		ESP_LOGE("VEML7700", "Failed to initialize. Result: %d\n", init_result);
		return;
	}

	ESP_LOGI("VEML7700", "Reading data...\r\n");

	while (true) {
		double lux_als, lux_white, fc_als, fc_white;

		// Read the ALS data
		ESP_ERROR_CHECK( veml7700_read_als_lux_auto(veml7700_dev, &lux_als) );
		// Convert to foot candles
		fc_als = lux_als * LUX_FC_COEFFICIENT;

		// Read the White data
		ESP_ERROR_CHECK( veml7700_read_white_lux_auto(veml7700_dev, &lux_white) );
		// Convert to foot candles
		fc_white = lux_white * LUX_FC_COEFFICIENT;

		printf("VEML7700 measured ALS %0.4f lux or %0.4f fc \n", lux_als, fc_als);
		printf("VEML7700 measured White %0.4f lux or %0.4f fc \n\n", lux_white, fc_white);

		vTaskDelay(2000 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}