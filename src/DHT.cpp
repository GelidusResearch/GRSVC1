/*!
 *  @file DHT.cpp
 *
 *  An ESP-IDF library for the DHT series of low-cost temperature/humidity sensors.
 *
 *  Written by FoxKeys https://github.com/FoxKeys
 * 
 *  Repo: https://github.com/FoxKeys/DHT-ESP-IDF
 *
 *  MIT license, check LICENSE for more information
 */
#include "DHT.h"

/**
 * @brief Construct a new DHT::DHT object
 *          Notice! Creating few objects for same GPIO will produce errors!
 *          Be carefull
 * 
 * @param pin - GPIO pin DHT connected to
 * @param type - DHD type (enum)
 */
DHT::DHT( const gpio_num_t pin, const dht_type_t type ) : _pin( pin ), _type( type ) {
}

/**
 * @brief Destroy the DHT::DHT object
 *          We must free alocated ESP resources (ISR handler and service)
 *          Note! We uninstalling gpio isr service _only_ if it was installed by our object. 
 *          Otherwise leave untouched. This is requider when isr service used by another parts of code
 * 
 */
DHT::~DHT() {
    if ( this->_isr_handler_installed ) {
        gpio_isr_handler_remove( this->_pin );
    }
    if ( this->_service_installed ) {
        gpio_uninstall_isr_service();
    }
};

/**
 * @brief   Setup sensor GPIO pin and interrupts handlers
 * 
 * @return esp_err_t 
 *          - ESP_OK Success
 *          - ESP_ERR_INVALID_ARG - GPIO parameters error (wrong pin selected in constructor)
 *          - ESP_ERR_NO_MEM No memory to install isr service
 *          - ESP_ERR_NOT_FOUND No free interrupt found with the specified flags
 *          - ESP_ERR_INVALID_STATE Wrong state, the ISR service has not been initialized.
 */
esp_err_t DHT::begin() {
    gpio_config_t io_conf;
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = 1ULL << this->_pin;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    esp_err_t res = gpio_config( &io_conf );

    if ( res != ESP_OK ) {
        ESP_LOGE( DHT_TAG, "gpio_config() failed with error %d", res );
        return res;
    }

    //install gpio isr service
    res = gpio_install_isr_service( ESP_INTR_FLAG_IRAM ); // Using IRAM because of (relatively) high speed transmission
    if ( res == ESP_OK ) {
        this->_service_installed = true;
    } else if ( res != ESP_ERR_INVALID_STATE ) { // ESP_ERR_INVALID_STATE means service already installed, and no action needed
        ESP_LOGE( DHT_TAG, "gpio_install_isr_service() failed with error %d", res );
        return res;
    }

    //hook isr handler for specific gpio pin
    res = gpio_isr_handler_add( this->_pin, gpio_isr_handler, (void *)this );
    if ( res != ESP_OK ) {
        ESP_LOGE( DHT_TAG, "gpio_isr_handler_add() failed with error %d", res );
        return res;
    }
    this->_isr_handler_installed = true;

    return ESP_OK;
};

/**
 * @brief Wakeups sensor, starts new conversion but not read response.
 *          Again. No response readins here! If you *only* call "sensorWakeup" - response 
 *          will be completely ignored
 *          Usable for sensorWakeup() -> delay(2S) -> singleRead(...) working sequence
 * 
 * @return esp_err_t 
 *          - ESP_OK Success
 *          - ESP_ERR_INVALID_STATE - begin() was not executed or has been failed
 *          - ESP_ERR_INVALID_ARG - GPIO parameters error (wrong pin selected in constructor)
 */
esp_err_t DHT::sensorWakeup() {
    if ( !this->_isr_handler_installed ) {
        ESP_LOGE( DHT_TAG, "ISR handler is not installed. begin() was not executed or was failed." );
        return ESP_ERR_INVALID_STATE;
    }
    // Notice! At this moment input are in GPIO_MODE_INPUT mode!
    // We setting output level BEFORE switch to GPIO_MODE_OUTPUT (otherwise we can get short low-to-high glitches)
    esp_err_t res = gpio_set_level( this->_pin, 1 );
    if ( res != ESP_OK ) {
        ESP_LOGE( DHT_TAG, "gpio_set_level() failed with error 0x%x", res );
        return res;
    }
    res = gpio_set_direction( this->_pin, GPIO_MODE_OUTPUT ); // have "high" on pin now
    if ( res != ESP_OK ) {
        ESP_LOGE( DHT_TAG, "gpio_set_direction() failed with error 0x%x", res );
        return res;
    }
    res = gpio_set_level( this->_pin, 0 ); // making it "low"
    if ( res != ESP_OK ) {
        ESP_LOGE( DHT_TAG, "gpio_set_level() failed with error 0x%x", res );
        return res;
    }
    switch ( this->_type ) {
    case DHT_TYPE_DHT11:
    case DHT_TYPE_DHT12: {
        // DHT11: Microprocessor I / O output while the output is set to low, and low retention time can’t be less
        // than 18ms
        // DHT12: Host data bus (SDA) Down over time (18ms), Tells sensors to prepare data
        esp_rom_delay_us( 20000 ); // Hold for 20000uS == 20mS
        break;
    }
    case DHT_TYPE_DHT21:
    case DHT_TYPE_DHT22:
    case DHT_TYPE_AM2301: {
        // DHT21: When the bus is idle, the bus is high. When the communication starts, the host (MCU) pulls
        // the bus low for 500us and then releases the bus, with a delay of 20-40us, the host starts to detect
        // the response signal of the slave (DHT21).
        //
        // DHT22: When communication between MCU and DHT22 begin, program of MCU will transform data-bus's
        // voltage level from high to low level and this process must beyond at least 1ms to ensure DHT22 could
        // detect MCU's signal, then MCU will wait 20-40us for DHT22's response.
        //
        // AM2301: Microprocessor data bus (SDA) to bring down a period of time (at least 800μS) notify the
        // sensor to prepare the data
        esp_rom_delay_us( 1000 ); // Hold for 1000uS
        break;
    }
    };

    // Notice! This will produce first positive edge (by bus pullup)
    res = gpio_set_direction( this->_pin, GPIO_MODE_INPUT ); // Release the bus. Notice! This edge will be counted by isr, because _reading_response is true already!
    if ( res != ESP_OK ) {
        ESP_LOGE( DHT_TAG, "gpio_set_direction() failed with error 0x%x", res );
        return res;
    }
    esp_rom_delay_us( this->bus_release_time_us ); // Wait for bus release

    return ESP_OK;
};

/**
 * @brief   Single read from sensor. Can return "deprecated" value because of next note from datasheet:
 *          "Note: the temperature and humidity data read by the host from the AM2301 is always the last measured value, such as the
 *          two measurement interval is very long, continuous read twice to the second value of real-time temperature and humidity values,
 *          while two read take minimum time interval be 2S"
 *          Use doubleRead(...) to always get updated data (but, this function will take more than 2S) 
 *          or use sensorWakeup() -> delay(2S) -> singleRead(...) sequence with async tasks/sleep
 * 
 * @param t - pointer to temperature (if required)
 * @param h - pointer to humidity (if required)
 * @return esp_err_t 
 *          - ESP_OK Success
 *          - ESP_ERR_INVALID_STATE - begin() was not executed or has been failed
 *          - ESP_ERR_INVALID_ARG - GPIO error (wrong pin selected in constructor)
 *          - ESP_ERR_NOT_FOUND - no response from sensor
 *          - ESP_ERR_INVALID_RESPONSE - too short response from sensor
 *          - ESP_ERR_INVALID_CRC - wrong response checksum
 */
esp_err_t DHT::singleRead( float *t, float *h ) {
    if ( !this->_isr_handler_installed ) {
        ESP_LOGE( DHT_TAG, "ISR handler is not installed. begin() was not executed or was failed." );
        return ESP_ERR_INVALID_STATE;
    }
    // Cleanup variables required for read process
    memset( &this->_response_timings, 0, sizeof( this->_response_timings ) );
    this->_edges_read = 0;
    if ( t != nullptr ) {
        *t = NAN;
    }
    if ( h != nullptr ) {
        *h = NAN;
    }

    // Send "Start" signal to sensor
    esp_err_t res = this->sensorWakeup();
    if ( res != ESP_OK ) {
        ESP_LOGE( DHT_TAG, "Sensor wakeup failed with error 0x%x", res );
        return res;
    }

    // At this moment we are released bus after of "Start signal". Remeber start time and allow isr to measure pulses
    this->_prev_edge_time = esp_timer_get_time();
    this->_reading_response = true;
    esp_rom_delay_us( this->bus_max_read_time );
    this->_reading_response = false; // Stop reading data from bus

    // Check edge_count
    if ( this->_edges_read == 0 ) {
        ESP_LOGE( DHT_TAG, "No response from sensor" );
        return ESP_ERR_NOT_FOUND;
    } else if ( this->_edges_read < expected_response_size ) {
        ESP_LOGE( DHT_TAG, "Sensor response is too short. Expected %d edges, got %d", expected_response_size, this->_edges_read );
        return ESP_ERR_INVALID_RESPONSE;
    }

    if ( this->debug_enabled ) {
        ESP_LOGI( DHT_TAG, "Raw response (timings):" );
        ESP_LOG_BUFFER_HEXDUMP( DHT_TAG, &this->_response_timings, sizeof( this->_response_timings ), ESP_LOG_INFO );
    }

    uint8_t data[5] = { 0 };
    assert( ( sizeof( data ) * 16 + 3 ) > sizeof( this->_response_timings ) );

    for ( int i = 0; i < sizeof( data ) * 8; i++ ) {
        // offset +2 to skip first 2 edges ("Sensor response signal" ). +0 offset is "low" bit part, +1 offset is "high" bit part
        bool bit = this->_response_timings[2 + i * 2 + 1] > this->_response_timings[2 + i * 2]; // If "high" longer than "low" - then 1 and 0 otherwise

        data[i / 8] = data[i / 8] << 1;
        data[i / 8] = data[i / 8] | bit;
    }
    if ( this->debug_enabled ) {
        ESP_LOGI( DHT_TAG, "Decoded response bytes:" );
        ESP_LOG_BUFFER_HEXDUMP( DHT_TAG, &data, sizeof( data ), ESP_LOG_INFO );
    }

    uint8_t checksum = data[0] + data[1] + data[2] + data[3];
    if ( this->debug_enabled ) {
        ESP_LOGI( DHT_TAG, "Calculated checksum: %x", checksum );
    }

    if ( checksum != data[4] ) {
        return ESP_ERR_INVALID_CRC;
    }

    // Notice. All examples are from datasheets "as is". Mostly from chineese versions (translated by google)
    switch ( this->_type ) {
    case DHT_TYPE_DHT11:
    case DHT_TYPE_DHT12: {
        // DHT11
        // Example 1: The received 40-bit data is: 00110101 00000000 00011000 00000100 01010001
        // Check digit = 00110101 + 00000000 + 00011000 + 00000100 = 01010001 (The received data is correct)
        // Humidity: 00110101 (integer)=35H = 53%RH 00000000 (decimal)=00H=0.0%RH => 53%RH + 0.0%RH = 53.0%RH
        // Temperature: 00011000 (integer)=18H = 24 C 00000100 (decimal)=04H=0.4C => 24C + 0.4C = 24.4C
        //
        // When the temperature is lower than 0°C, the highest position of the lower 8 bits of the temperature data is 1
        // Example: -10.1C is expressed as 00001010 10000001
        // Temperature: 00001010 (integer)=0AH = 10C, 00000001 (decimal)=01H=0.1C => -(10C+0.1C)C = -10.1C
        // DHT12
        // Example: receiving 40 Data for: 00111000 00001000 00011010 00000110 01100000
        // 00111000+00001000+00011010+00000110=01100000 (Check digit)
        // Humidity: 00111000 (Binary) =>56 (Decimal) 00001000(Binary)=>8 (Decimal) => Humidity=56.8%RH
        // Temperature: 00011010 (Binary) =>26 (Decimal)00000110(Binary)=>6 (Decimal) => Temperature= 26.6 degrees
        // Celsius Example two: received 40 Data for:
        // 00111000 00001000 00011010 10000110 11100000
        // 00111000+00001000+00011010+10000110=11100000(Check digit)
        // Humidity:00111000 (Binary) =>56 (Decimal)00001000(Binary)=>8 (Decimal)=>Humidity=56.8%RH
        // Temperature: temperature low 8Bit 为 1 It indicates sampling the temperature to minus-temperature
        // 00011010 (Binary) =>26 (Decimal) 10000110 (Binary,Ignore 8Bit)=>6(Decimal) =>Temperature=-26.6 degrees Celsius
        if ( h != nullptr ) {
            *h = data[0] + data[1] * 0.1;
        }
        if ( t != nullptr ) {
            *t = data[2] + ( data[3] & 0x7F ) * 0.1;
            if ( data[3] & 0x80 ) {
                *t *= -1.0;
            }
        }

        break;
    }
    case DHT_TYPE_DHT21:
    case DHT_TYPE_DHT22:
    case DHT_TYPE_AM2301: {
        // DHT21
        // Note: The sampling period interval should not be less than 1.7 seconds (2 seconds recommended).
        // Data format: 40bit data=16bit humidity data+16bit temperature data+8bit checksum
        // Example: Receive 40bit data as follows:
        // 00000010 10001100 00000001 01011111 11101110
        // Humidity high 8 bits + humidity low 8 bits + temperature high 8 bits + temperature low 8 bits = last 8 bits = checksum
        // 00000010+10001100+00000001+01011111=11101110
        // Humidity=65.2%RH Temperature=35.1°C
        // The highest position of the temperature data when the temperature is below 0°C
        // For example: -10.1°C is expressed as 1000 0000 0110 0101
        // AM2301
        // Example 1：40 Data received
        // 00000010 10010010 00000001 00001101 10100010
        // Calculate: 00000010 + 10010010 + 00000001 + 00001101 = 10100010（Parity bit)
        // humidity：00000010 10010010 = 0292H (Hexadecimal)= 2×256 + 9×16 + 2 = 658 => Humidity = 65.8%R
        // Temp.：0000 0001 0000 1101 = 10DH(Hexadecimal) = 1×256 + 0×16 + 13 = 269 => Temp.= 26.9°C
        // When the temperature is below 0 °C, the highest position of the temperature data.
        // Example：-10.1 °C Expressed as 1 000 0000 0110 0101
        // Temp.：0000 0000 0110 0101 = 0065H(Hexadecimal)＝ 6×16 +5 = 101 => Temp. ＝ -10.1°C
        if ( h != nullptr ) {
            *h = ( (uint16_t)data[0] ) << 8 | data[1];
            *h *= 0.1;
        }
        if ( t != nullptr ) {
            *t = ( ( uint16_t )( data[2] & 0x7F ) ) << 8 | data[3];
            *t *= 0.1;
            if ( data[2] & 0x80 ) {
                *t *= -1.0;
            }
        }
        break;
    }
    };

    ESP_LOGI( DHT_TAG, "t: %f; h: %f", t ? *t : NAN, h ? *h : NAN );

    return ESP_OK;
};

/**
 * @brief   Double read from sensor. Double read can be required because of next note from datasheet:
 *          "Note: the temperature and humidity data read by the host from the AM2301 is always the last measured value, such as the
 *          two measurement interval is very long, continuous read twice to the second value of real-time temperature and humidity values,
 *          while two read take minimum time interval be 2S"
 *          So, to get fresh data - use doubleRead() or sensorWakeup() -> delay(2S) -> singleRead(...) sequence with async tasks/sleep
 * 
 * @param t - pointer to temperature (if required)
 * @param h - pointer to humidity (if required)
 * @return esp_err_t 
 *          - ESP_OK Success
 *          - ESP_ERR_INVALID_STATE - begin() was not executed or has been failed
 *          - ESP_ERR_INVALID_ARG - GPIO error (wrong pin selected in constructor)
 *          - ESP_ERR_NOT_FOUND - no response from sensor
 *          - ESP_ERR_INVALID_RESPONSE - too short response from sensor
 *          - ESP_ERR_INVALID_CRC - wrong response checksum
 */
esp_err_t DHT::doubleRead( float *t, float *h ) {
    if ( !this->_isr_handler_installed ) {
        ESP_LOGE( DHT_TAG, "ISR handler is not installed. begin() was not executed or was failed." );
        return ESP_ERR_INVALID_STATE;
    }
    // Send "Start" signal to sensor
    esp_err_t res = this->sensorWakeup();
    if ( res != ESP_OK ) {
        ESP_LOGE( DHT_TAG, "Sensor wakeup failed with error 0x%x", res );
        return res;
    }
    vTaskDelay( pdMS_TO_TICKS( this->double_read_interval_ms ) );
    return this->singleRead( t, h );
};

float DHT::convertCtoF( const float tC ) const {
    return tC == NAN ? NAN : tC * 1.8 + 32;
};

/**
 * @brief ISR handler. Private *static* IRAM method
 * 
 * @param arg - contains pointer to DHT *object*
 *              Because this is *static* method, it is *shared* between *objects* of DHT class
 */
void DHT::gpio_isr_handler( void *arg ) {
    DHT *self = reinterpret_cast<DHT *>( arg );
    if ( self->_reading_response ) {
        if ( self->_edges_read < expected_response_size ) {
            int64_t curr_time = esp_timer_get_time();
            int64_t interval = curr_time - self->_prev_edge_time;
            self->_prev_edge_time = curr_time;

            self->_response_timings[self->_edges_read] = interval < 0xff ? interval : 0xff;
        } // extra edges ignored, but counted. Suitable for debugging
        self->_edges_read++;
    }
};
