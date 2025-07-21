/*!
 *  @file DHT.h
 *
 *  An ESP-IDF library for the DHT series of low-cost temperature/humidity sensors.
 *
 *  Written by FoxKeys https://github.com/FoxKeys
 * 
 *  Repo: https://github.com/FoxKeys/DHT-ESP-IDF
 *
 *  MIT license, check LICENSE for more information
 */

#pragma once

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char DHT_TAG[] = "DHT";
// ( 1 + 40 ) * 2 => "Sensor response signal" + 40 data bits. Multiply on 2 because we measure low and high time of each bit
// Notice! Some sensors (AM2301 for example) sends more data bits (64). But, "extra" bits are zero, and we ignore it
static const uint8_t expected_response_size = ( 1 + 40 ) * 2;

typedef enum {
    DHT_TYPE_DHT11 = 11,
    DHT_TYPE_DHT12 = 12,
    DHT_TYPE_DHT21 = 21,
    DHT_TYPE_DHT22 = 22,
    DHT_TYPE_AM2301 = 2301
} dht_type_t;

class DHT {
public:
    /**
     * @brief Outputs response dumps to serial when set to "true"
     * 
     */
    bool debug_enabled = false;
    /**
     * @brief   time (in us) between bus release and readings start. 
     *          If this value too little - interrupt can "see" release pulse and all response bits will be shifted,
     *          breaking decode. If value is too big - we can lost first bit(s) of response. According to datasheed of AM2301
     *          Tgo - Bus master has released time is 20-200uS But, 15 is ok for my sensors. But, you can try another values 
     *          if you see CRC errors. Enable "extended_debug" to see response dumps. Two first bytes of Raw response expected 
     *          to be about 50/50. If you see something like 05 55 51 - increase bus_release_time_us. 
     *          If you see something like 55 31 1a - (first 55 is lost) - decrease bus_release_time_us
     */
    uint8_t bus_release_time_us = 15;
    /** 
     *  @brief  Max total time to read response (uS). According to "AM2301 Product Manual"
     *          Max response time = Trel + Treh + (Tlow + Th1)*40 + Ten
     *          Trel (max) = 85uS, Treh (max) = 85uS, Tlow (max) = 55uS, Th1 (max) = 75uS, Ten (max) = 55uS
     *          So, Max response time = 85 + 85 + (55 + 75)*40 + 55 = 5425uS
     *          Using a bit longer interval as default (extra bits will be ignored anyway)
     */
    uint32_t bus_max_read_time = 8000;
    /**
     * @brief Delay between two reads of "doubleRead" in mS. Double read is required because of this notice in datasheet:
     *        Note: the temperature and humidity data read by the host from the AM2301 is always the last measured value, such as the
     *        two measurement interval is very long, continuous read twice to the second value of real-time temperature and humidity values,
     *        while two read take minimum time interval be 2S
     * 
     */
    uint32_t double_read_interval_ms = 2001;

    DHT( const gpio_num_t pin, const dht_type_t type );
    virtual ~DHT();
    esp_err_t begin();
    esp_err_t sensorWakeup();
    esp_err_t singleRead( float *t = nullptr, float *h = nullptr );
    esp_err_t doubleRead( float *t = nullptr, float *h = nullptr );
    float convertCtoF( const float tC ) const;

private:
    gpio_num_t _pin;
    dht_type_t _type;

    bool _service_installed = false;
    bool _isr_handler_installed = false;
    bool _reading_response = false;
    uint8_t _response_timings[expected_response_size];
    uint8_t _edges_read = 0;
    int64_t _prev_edge_time = 0;

    static void IRAM_ATTR gpio_isr_handler( void *arg );
};