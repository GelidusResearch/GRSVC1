/*!
 *  @file dht_sensor.c
 *
 *  A C implementation for the DHT series temperature/humidity sensors for ESP-IDF.
 *  Converted from the original C++ DHT-ESP-IDF library by FoxKeys.
 *
 *  IMPORTANT NOTE: For optimal performance and ISR safety, consider enabling
 *  CONFIG_GPIO_CTRL_FUNC_IN_IRAM in sdkconfig. This allows gpio_set_level
 *  to be executed when Cache is disabled within ISR context.
 *
 *  MIT license
 */

#include "dht_sensor.h"

static const char DHT_TAG[] = "DHT";

// Global pointer for ISR handler (supports only one sensor instance)
static dht_sensor_t *g_dht_sensor = NULL;

// ISR handler - not used in polling approach but kept for potential future use
/*
static void IRAM_ATTR dht_gpio_isr_handler(void *arg) {
    dht_sensor_t *sensor = (dht_sensor_t *)arg;
    if (sensor->reading_response) {
        if (sensor->edges_read < DHT_EXPECTED_RESPONSE_SIZE) {
            int64_t curr_time = esp_timer_get_time();
            int64_t interval = curr_time - sensor->prev_edge_time;
            sensor->prev_edge_time = curr_time;

            sensor->response_timings[sensor->edges_read] = interval < 0xff ? interval : 0xff;
        } // extra edges ignored, but counted. Suitable for debugging
        sensor->edges_read++;
    }
}
*/

void dht_init(dht_sensor_t *sensor, gpio_num_t pin, dht_type_t type) {
    memset(sensor, 0, sizeof(dht_sensor_t));
    sensor->pin = pin;
    sensor->type = type;
    
    // Set default configuration values
    sensor->debug_enabled = false;
    sensor->bus_release_time_us = 30;      // 20-40μs wait after releasing bus
    sensor->bus_max_read_time = 8000;      // Max time to wait for complete response
    sensor->double_read_interval_ms = 60000; // DHT22 refresh interval - 60 seconds
    
    // Initialize state
    sensor->service_installed = false;
    sensor->isr_handler_installed = false;
    sensor->reading_response = false;
    sensor->edges_read = 0;
    sensor->prev_edge_time = 0;
    
    // Store global reference for ISR
    g_dht_sensor = sensor;
}

esp_err_t dht_begin(dht_sensor_t *sensor) {
    gpio_config_t io_conf = {0};
    
    // Configure GPIO as input with pull-up (no interrupts for now)
    io_conf.intr_type = GPIO_INTR_DISABLE;  // Disable interrupts, use polling instead
    io_conf.pin_bit_mask = 1ULL << sensor->pin;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    
    esp_err_t res = gpio_config(&io_conf);
    if (res != ESP_OK) {
        ESP_LOGE(DHT_TAG, "gpio_config() failed with error %d", res);
        return res;
    }

    // Mark as configured (no ISR service needed for polling approach)
    sensor->isr_handler_installed = true;  // Use this flag to indicate "ready"

    return ESP_OK;
}

void dht_cleanup(dht_sensor_t *sensor) {
    // No ISR cleanup needed for polling approach
    sensor->isr_handler_installed = false;
    sensor->service_installed = false;
    g_dht_sensor = NULL;
}

esp_err_t dht_sensor_wakeup(dht_sensor_t *sensor) {
    if (!sensor->isr_handler_installed) {
        ESP_LOGE(DHT_TAG, "ISR handler is not installed. begin() was not executed or was failed.");
        return ESP_ERR_INVALID_STATE;
    }
    
    // DHT22 Start Signal Protocol (per OCFreaks reference):
    // 1. MCU pulls line LOW for ~18ms (critical timing!)
    // 2. MCU releases line, pulled HIGH for ~20-40μs  
    // 3. DHT responds with ~80μs LOW, then ~80μs HIGH
    // 4. Data transmission begins (40 bits)
    
    // Set output level BEFORE switch to output mode (avoid glitches)
    esp_err_t res = gpio_set_level(sensor->pin, 1);
    if (res != ESP_OK) {
        ESP_LOGE(DHT_TAG, "gpio_set_level() failed with error 0x%x", res);
        return res;
    }
    
    res = gpio_set_direction(sensor->pin, GPIO_MODE_OUTPUT);
    if (res != ESP_OK) {
        ESP_LOGE(DHT_TAG, "gpio_set_direction() failed with error 0x%x", res);
        return res;
    }
    
    res = gpio_set_level(sensor->pin, 0); // Pull low to start communication
    if (res != ESP_OK) {
        ESP_LOGE(DHT_TAG, "gpio_set_level() failed with error 0x%x", res);
        return res;
    }
    
    // Hold low for appropriate time based on sensor type
    switch (sensor->type) {
    case DHT_TYPE_DHT11:
    case DHT_TYPE_DHT12:
        // DHT11/12: Low retention time can't be less than 18ms
        esp_rom_delay_us(20000); // Hold for 20ms
        break;
    case DHT_TYPE_DHT21:
    case DHT_TYPE_DHT22:
    case DHT_TYPE_AM2301:
        // DHT22: Start signal must be at least 18ms (per OCFreaks reference)
        // Previous 1ms was too short - this was the main timing issue!
        esp_rom_delay_us(18000); // Hold for 18ms (corrected from 1ms)
        break;
    }

    // Release the bus (this will produce first positive edge)
    res = gpio_set_direction(sensor->pin, GPIO_MODE_INPUT);
    if (res != ESP_OK) {
        ESP_LOGE(DHT_TAG, "gpio_set_direction() failed with error 0x%x", res);
        return res;
    }
    
    esp_rom_delay_us(sensor->bus_release_time_us); // Wait for bus release

    return ESP_OK;
}

esp_err_t dht_single_read(dht_sensor_t *sensor, float *temperature, float *humidity) {
    if (!sensor->isr_handler_installed) {
        ESP_LOGE(DHT_TAG, "ISR handler is not installed. begin() was not executed or was failed.");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Cleanup variables for read process
    memset(&sensor->response_timings, 0, sizeof(sensor->response_timings));
    sensor->edges_read = 0;
    if (temperature != NULL) {
        *temperature = NAN;
    }
    if (humidity != NULL) {
        *humidity = NAN;
    }

    // Send "Start" signal to sensor
    esp_err_t res = dht_sensor_wakeup(sensor);
    if (res != ESP_OK) {
        ESP_LOGE(DHT_TAG, "Sensor wakeup failed with error 0x%x", res);
        return res;
    }

    // Use polling approach instead of interrupts for more reliable timing
    // Wait for DHT response signal (sensor pulls low for ~80μs, then high for ~80μs)
    int timeout_counter = 0;
    
    // Wait for sensor to pull line low (response signal start)
    while (gpio_get_level(sensor->pin) == 1 && timeout_counter++ < 100) {
        esp_rom_delay_us(1);
    }
    if (timeout_counter >= 100) {
        ESP_LOGE(DHT_TAG, "No response from sensor - line never went low");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Wait for sensor to release line (response signal end)
    timeout_counter = 0;
    while (gpio_get_level(sensor->pin) == 0 && timeout_counter++ < 100) {
        esp_rom_delay_us(1);
    }
    if (timeout_counter >= 100) {
        ESP_LOGE(DHT_TAG, "Sensor response timeout - line stuck low");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Wait for sensor ready signal (high for ~80μs)
    timeout_counter = 0;
    while (gpio_get_level(sensor->pin) == 1 && timeout_counter++ < 100) {
        esp_rom_delay_us(1);
    }
    if (timeout_counter >= 100) {
        ESP_LOGE(DHT_TAG, "Sensor ready timeout - line stuck high");
        return ESP_ERR_NOT_FOUND;
    }

    // Now read the 40 data bits using polling
    uint8_t data[5] = {0};
    
    for (int i = 0; i < 40; i++) {
        // Wait for bit start (low pulse ~50μs)
        timeout_counter = 0;
        while (gpio_get_level(sensor->pin) == 0 && timeout_counter++ < 80) {
            esp_rom_delay_us(1);
        }
        if (timeout_counter >= 80) {
            ESP_LOGE(DHT_TAG, "Bit %d timeout waiting for high", i);
            return ESP_ERR_INVALID_RESPONSE;
        }
        
        // Measure high pulse duration (~26-28μs for '0', ~70μs for '1')
        uint32_t high_time = 0;
        while (gpio_get_level(sensor->pin) == 1 && high_time++ < 120) {
            esp_rom_delay_us(1);
        }
        
        if (high_time >= 120) {
            ESP_LOGE(DHT_TAG, "Bit %d timeout - high pulse too long", i);
            return ESP_ERR_INVALID_RESPONSE;
        }
        
        // Determine bit value: >40μs = '1', <=40μs = '0'
        bool bit = high_time > 40;
        
        if (sensor->debug_enabled && i < 8) {
            ESP_LOGI(DHT_TAG, "Bit %d: High time=%lu μs -> %d", i, high_time, bit);
        }
        
        // Store bit in data array
        data[i / 8] <<= 1;
        if (bit) {
            data[i / 8] |= 1;
        }
    }

    if (sensor->debug_enabled) {
        ESP_LOGI(DHT_TAG, "Decoded response bytes:");
        ESP_LOG_BUFFER_HEXDUMP(DHT_TAG, &data, sizeof(data), ESP_LOG_INFO);
    }

    // Verify checksum
    uint8_t calculated_checksum = data[0] + data[1] + data[2] + data[3];
    uint8_t received_checksum = data[4];
    
    if (sensor->debug_enabled) {
        ESP_LOGI(DHT_TAG, "Data bytes: [0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X]", 
                 data[0], data[1], data[2], data[3], data[4]);
        ESP_LOGI(DHT_TAG, "Calculated checksum: 0x%02X (%d)", calculated_checksum, calculated_checksum);
        ESP_LOGI(DHT_TAG, "Received checksum: 0x%02X (%d)", received_checksum, received_checksum);
    }

    // DHT22 checksum is only the lower 8 bits of the sum
    if (calculated_checksum != received_checksum) {
        ESP_LOGE(DHT_TAG, "Checksum mismatch. Calculated: 0x%02X, Received: 0x%02X", 
                 calculated_checksum, received_checksum);
        ESP_LOGE(DHT_TAG, "Raw data: H=0x%02X%02X, T=0x%02X%02X, CRC=0x%02X", 
                 data[0], data[1], data[2], data[3], data[4]);
        return ESP_ERR_INVALID_CRC;
    }

    // Parse data based on sensor type
    switch (sensor->type) {
    case DHT_TYPE_DHT11:
    case DHT_TYPE_DHT12:
        // DHT11/12: Integer and decimal parts
        if (humidity != NULL) {
            *humidity = data[0] + data[1] * 0.1f;
        }
        if (temperature != NULL) {
            *temperature = data[2] + (data[3] & 0x7F) * 0.1f;
            if (data[3] & 0x80) {
                *temperature *= -1.0f; // Negative temperature
            }
        }
        break;
        
    case DHT_TYPE_DHT21:
    case DHT_TYPE_DHT22:
    case DHT_TYPE_AM2301:
        // DHT21/22/AM2301: 16-bit values
        if (humidity != NULL) {
            *humidity = ((uint16_t)data[0] << 8 | data[1]) * 0.1f;
        }
        if (temperature != NULL) {
            *temperature = (((uint16_t)(data[2] & 0x7F)) << 8 | data[3]) * 0.1f;
            if (data[2] & 0x80) {
                *temperature *= -1.0f; // Negative temperature
            }
        }
        break;
    }

    ESP_LOGI(DHT_TAG, "Temperature: %.1f°C, Humidity: %.1f%%", 
             temperature ? *temperature : NAN, humidity ? *humidity : NAN);

    return ESP_OK;
}

esp_err_t dht_double_read(dht_sensor_t *sensor, float *temperature, float *humidity) {
    if (!sensor->isr_handler_installed) {
        ESP_LOGE(DHT_TAG, "ISR handler is not installed. begin() was not executed or was failed.");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Send first "Start" signal to sensor
    esp_err_t res = dht_sensor_wakeup(sensor);
    if (res != ESP_OK) {
        ESP_LOGE(DHT_TAG, "Sensor wakeup failed with error 0x%x", res);
        return res;
    }
    
    // Wait for the required interval between reads
    vTaskDelay(pdMS_TO_TICKS(sensor->double_read_interval_ms));
    
    // Perform the actual read
    return dht_single_read(sensor, temperature, humidity);
}

float dht_convert_c_to_f(float celsius) {
    return isnan(celsius) ? NAN : celsius * 1.8f + 32.0f;
}
