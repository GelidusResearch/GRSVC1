/*!
 *  @file dht_sensor.h
 *
 *  A C implementation for the DHT series temperature/humidity sensors for ESP-IDF.
 *  Converted from the original C++ DHT-ESP-IDF library by FoxKeys.
 *
 *  MIT license
 */

#ifndef DHT_SENSOR_H
#define DHT_SENSOR_H

#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_err.h"

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

// ( 1 + 40 ) * 2 => "Sensor response signal" + 40 data bits. Multiply on 2 because we measure low and high time of each bit
// Notice! Some sensors (AM2301 for example) sends more data bits (64). But, "extra" bits are zero, and we ignore it
#define DHT_EXPECTED_RESPONSE_SIZE ((1 + 40) * 2)

typedef enum {
    DHT_TYPE_DHT11 = 11,
    DHT_TYPE_DHT12 = 12,
    DHT_TYPE_DHT21 = 21,
    DHT_TYPE_DHT22 = 22,
    DHT_TYPE_AM2301 = 2301
} dht_type_t;

typedef struct {
    gpio_num_t pin;
    dht_type_t type;
    
    // Configuration parameters
    bool debug_enabled;
    uint8_t bus_release_time_us;
    uint32_t bus_max_read_time;
    uint32_t double_read_interval_ms;
    
    // Internal state
    bool service_installed;
    bool isr_handler_installed;
    bool reading_response;
    uint8_t response_timings[DHT_EXPECTED_RESPONSE_SIZE];
    uint8_t edges_read;
    int64_t prev_edge_time;
} dht_sensor_t;

/**
 * @brief Initialize a DHT sensor structure
 * 
 * @param sensor Pointer to DHT sensor structure
 * @param pin GPIO pin DHT connected to
 * @param type DHT sensor type
 */
void dht_init(dht_sensor_t *sensor, gpio_num_t pin, dht_type_t type);

/**
 * @brief Setup sensor GPIO pin and interrupt handlers
 * 
 * @param sensor Pointer to DHT sensor structure
 * @return esp_err_t 
 *          - ESP_OK Success
 *          - ESP_ERR_INVALID_ARG - GPIO parameters error
 *          - ESP_ERR_NO_MEM No memory to install isr service
 *          - ESP_ERR_NOT_FOUND No free interrupt found
 *          - ESP_ERR_INVALID_STATE Wrong state, ISR service not initialized
 */
esp_err_t dht_begin(dht_sensor_t *sensor);

/**
 * @brief Cleanup sensor resources
 * 
 * @param sensor Pointer to DHT sensor structure
 */
void dht_cleanup(dht_sensor_t *sensor);

/**
 * @brief Wakeup sensor and start new conversion (no read)
 * 
 * @param sensor Pointer to DHT sensor structure
 * @return esp_err_t 
 *          - ESP_OK Success
 *          - ESP_ERR_INVALID_STATE - begin() was not executed or failed
 *          - ESP_ERR_INVALID_ARG - GPIO parameters error
 */
esp_err_t dht_sensor_wakeup(dht_sensor_t *sensor);

/**
 * @brief Single read from sensor
 * 
 * @param sensor Pointer to DHT sensor structure
 * @param temperature Pointer to temperature variable (can be NULL)
 * @param humidity Pointer to humidity variable (can be NULL)
 * @return esp_err_t 
 *          - ESP_OK Success
 *          - ESP_ERR_INVALID_STATE - begin() was not executed or failed
 *          - ESP_ERR_INVALID_ARG - GPIO error
 *          - ESP_ERR_NOT_FOUND - no response from sensor
 *          - ESP_ERR_INVALID_RESPONSE - too short response from sensor
 *          - ESP_ERR_INVALID_CRC - wrong response checksum
 */
esp_err_t dht_single_read(dht_sensor_t *sensor, float *temperature, float *humidity);

/**
 * @brief Double read from sensor (ensures fresh data)
 * 
 * @param sensor Pointer to DHT sensor structure
 * @param temperature Pointer to temperature variable (can be NULL)
 * @param humidity Pointer to humidity variable (can be NULL)
 * @return esp_err_t (same as dht_single_read)
 */
esp_err_t dht_double_read(dht_sensor_t *sensor, float *temperature, float *humidity);

/**
 * @brief Convert Celsius to Fahrenheit
 * 
 * @param celsius Temperature in Celsius
 * @return float Temperature in Fahrenheit
 */
float dht_convert_c_to_f(float celsius);

#ifdef __cplusplus
}
#endif

#endif // DHT_SENSOR_H
