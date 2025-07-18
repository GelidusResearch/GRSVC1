#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "sdkconfig.h"

#include <host/ble_hs.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_partition.h"
#include "esp_system.h"

#include "dht_sensor.h"

#define DEVICE_NAME "GRSVC1"
#define FIRMWARE_VERSION "1.0.0"
#define MANUFACTURER_NAME "Gelidus Research Inc."
#define MODEL_NUMBER "GRSVC V1"
#define HARDWARE_REVISION "Rev 1.0"
#define SOFTWARE_REVISION "ESP-IDF v5.4.2"
#define FLOW_SENSOR_GPIO GPIO_NUM_26  // Flow GPIO pin
#define VALVE_1_GPIO GPIO_NUM_33  // Valve 1 GPIO pin
#define VALVE_2_GPIO GPIO_NUM_27  // Valve 2 GPIO pin
#define POWER_12V_GPIO GPIO_NUM_14  // 12V output GPIO pin
#define STATUS_LED_GPIO GPIO_NUM_4  // Status LED GPIO pin 
#define BATTERY_VOLTAGE_GPIO GPIO_NUM_12  // Battery Voltage ADC pin (ADC2_CH5)
#define DHT_SENSOR_GPIO GPIO_NUM_32  // DHT sensor GPIO pin

char *TAG = "GRSVC1";
uint8_t ble_addr_type;

// Globals
static char device_mac_string[18] = "00:00:00:00:00:00";
static volatile uint32_t pulse_count = 0;
static adc_oneshot_unit_handle_t adc2_handle;
static adc_cali_handle_t adc2_cali_chan5_handle;

// DHT sensor
static dht_sensor_t dht_sensor;
static float last_temperature = NAN;
static float last_humidity = NAN;

// Valve state tracking (since latching valves don't provide feedback)
static bool valve_1_state = false; // false = OFF, true = ON
static bool valve_2_state = false; // false = OFF, true = ON

// Connection state tracking
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool is_connected = false;
static bool is_advertising = false;

// OTA (Over-The-Air) update variables
static esp_ota_handle_t ota_handle = 0;
static const esp_partition_t *ota_partition = NULL;
static bool ota_in_progress = false;
static uint32_t ota_bytes_received = 0;
static uint32_t ota_total_size = 0;
static uint32_t ota_last_progress = 0;

// OTA update states
typedef enum {
    OTA_STATE_IDLE = 0,
    OTA_STATE_INIT,
    OTA_STATE_RECEIVING,
    OTA_STATE_VALIDATING,
    OTA_STATE_COMPLETE,
    OTA_STATE_ERROR
} ota_state_t;

static ota_state_t ota_state = OTA_STATE_IDLE;

// Initialize MAC address string (call once at startup)
void init_device_mac_string() {
    if (ble_hs_synced()) {
        uint8_t mac_addr[6];
        int mac_rc = ble_hs_id_copy_addr(ble_addr_type, mac_addr, NULL);
        if (mac_rc == 0) {
            snprintf(device_mac_string, sizeof(device_mac_string), "%02X:%02X:%02X:%02X:%02X:%02X",
                    mac_addr[5], mac_addr[4], mac_addr[3], 
                    mac_addr[2], mac_addr[1], mac_addr[0]);
            ESP_LOGI(TAG, "Device MAC address initialized: %s", device_mac_string);
        } else {
            ESP_LOGW(TAG, "Failed to read MAC address during initialization, using default");
        }
    } else {
        ESP_LOGW(TAG, "BLE stack not synced during MAC address initialization");
    }
}

// ISR handler increments pulse count on rising edge
static void IRAM_ATTR flow_sensor_isr_handler(void* arg) {
    pulse_count++;
}

void flow_sensor_init() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << FLOW_SENSOR_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);

    // Install ISR service and add handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(FLOW_SENSOR_GPIO, flow_sensor_isr_handler, NULL);
}

void ble_app_advertise(void);
void connection_monitor_task(void *param);
void force_advertising_restart(void);

// Command enumeration for switch-case
typedef enum {
    CMD_UNKNOWN = 0,
    CMD_VALVE_1_ON,
    CMD_VALVE_1_OFF,
    CMD_VALVE_2_ON,
    CMD_VALVE_2_OFF,
    CMD_12V_ON,
    CMD_12V_OFF,
    CMD_RESET_FLOW,
    CMD_STATUS,
    CMD_RESTART_ADV,
    CMD_OTA_INFO,
    CMD_BLE_STATUS,
    CMD_TEST_DHT,
    CMD_DHT_REINIT
} command_t;

// Function to parse command string to enum
static command_t parse_command(const char* cmd) {
    if (strcmp(cmd, "VALVE 1 ON") == 0) return CMD_VALVE_1_ON;
    if (strcmp(cmd, "VALVE 1 OFF") == 0) return CMD_VALVE_1_OFF;
    if (strcmp(cmd, "VALVE 2 ON") == 0) return CMD_VALVE_2_ON;
    if (strcmp(cmd, "VALVE 2 OFF") == 0) return CMD_VALVE_2_OFF;
    if (strcmp(cmd, "12V ON") == 0) return CMD_12V_ON;
    if (strcmp(cmd, "12V OFF") == 0) return CMD_12V_OFF;
    if (strcmp(cmd, "RESET FLOW") == 0) return CMD_RESET_FLOW;
    if (strcmp(cmd, "STATUS") == 0) return CMD_STATUS;
    if (strcmp(cmd, "RESTART ADV") == 0) return CMD_RESTART_ADV;
    if (strcmp(cmd, "OTA INFO") == 0) return CMD_OTA_INFO;
    if (strcmp(cmd, "BLE STATUS") == 0) return CMD_BLE_STATUS;
    if (strcmp(cmd, "TEST DHT") == 0) return CMD_TEST_DHT;
    if (strcmp(cmd, "DHT REINIT") == 0) return CMD_DHT_REINIT;
    return CMD_UNKNOWN;
}

// Write data to ESP32 defined as server - Control characteristic handler
static int device_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    char data[32];
    int data_len = ctxt->om->om_len;
    
    // Ensure null termination and prevent buffer overflow
    if (data_len >= sizeof(data)) {
        data_len = sizeof(data) - 1;
    }
    
    memcpy(data, ctxt->om->om_data, data_len);
    data[data_len] = '\0';
    
    ESP_LOGI(TAG, "Received command: %s", data);
    
    // Parse command and use switch-case for better performance and readability
    command_t cmd = parse_command(data);
    
    switch (cmd) {
        case CMD_VALVE_1_ON:
            ESP_LOGI(TAG, "VALVE 1 ON - Pulsing valve 1 for 50ms (GPIO %d)", VALVE_1_GPIO);
            
            // Check current GPIO state before pulse
            int initial_level = gpio_get_level(VALVE_1_GPIO);
            ESP_LOGI(TAG, "VALVE 1 - GPIO %d initial level: %d", VALVE_1_GPIO, initial_level);
            
            // Generate 50ms pulse to toggle valve
            esp_err_t ret = gpio_set_level(VALVE_1_GPIO, 1);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "VALVE 1 - Failed to set GPIO %d HIGH: %s", VALVE_1_GPIO, esp_err_to_name(ret));
                break;
            }
            
            ESP_LOGI(TAG, "VALVE 1 - Pulse HIGH, holding for 50ms...");
            vTaskDelay(pdMS_TO_TICKS(50)); // 50ms pulse
            
            ret = gpio_set_level(VALVE_1_GPIO, 0);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "VALVE 1 - Failed to set GPIO %d LOW: %s", VALVE_1_GPIO, esp_err_to_name(ret));
            } else {
                valve_1_state = true; // Update state tracking
                ESP_LOGI(TAG, "VALVE 1 - Pulse complete, valve toggled to ON state");
            }
            break;
            
        case CMD_VALVE_1_OFF:
            ESP_LOGI(TAG, "VALVE 1 OFF - Pulsing valve 1 for 50ms (GPIO %d)", VALVE_1_GPIO);
            
            // Check current GPIO state before pulse
            int initial_level_v1 = gpio_get_level(VALVE_1_GPIO);
            ESP_LOGI(TAG, "VALVE 1 - GPIO %d initial level: %d", VALVE_1_GPIO, initial_level_v1);
            
            // Generate 50ms pulse to toggle valve
            esp_err_t ret_v1 = gpio_set_level(VALVE_1_GPIO, 1);
            if (ret_v1 != ESP_OK) {
                ESP_LOGE(TAG, "VALVE 1 - Failed to set GPIO %d HIGH: %s", VALVE_1_GPIO, esp_err_to_name(ret_v1));
                break;
            }
            
            ESP_LOGI(TAG, "VALVE 1 - Pulse HIGH, holding for 50ms...");
            vTaskDelay(pdMS_TO_TICKS(50)); // 50ms pulse
            
            ret_v1 = gpio_set_level(VALVE_1_GPIO, 0);
            if (ret_v1 != ESP_OK) {
                ESP_LOGE(TAG, "VALVE 1 - Failed to set GPIO %d LOW: %s", VALVE_1_GPIO, esp_err_to_name(ret_v1));
            } else {
                valve_1_state = false; // Update state tracking
                ESP_LOGI(TAG, "VALVE 1 - Pulse complete, valve toggled to OFF state");
            }
            break;
            
        case CMD_VALVE_2_ON:
            ESP_LOGI(TAG, "VALVE 2 ON - Pulsing valve 2 for 50ms (GPIO %d)", VALVE_2_GPIO);
            
            // Generate 50ms pulse to toggle valve
            esp_err_t ret_v2_on = gpio_set_level(VALVE_2_GPIO, 1);
            if (ret_v2_on != ESP_OK) {
                ESP_LOGE(TAG, "VALVE 2 - Failed to set GPIO %d HIGH: %s", VALVE_2_GPIO, esp_err_to_name(ret_v2_on));
                break;
            }
            
            ESP_LOGI(TAG, "VALVE 2 - Pulse HIGH, holding for 50ms...");
            vTaskDelay(pdMS_TO_TICKS(50)); // 50ms pulse
            
            ret_v2_on = gpio_set_level(VALVE_2_GPIO, 0);
            if (ret_v2_on != ESP_OK) {
                ESP_LOGE(TAG, "VALVE 2 - Failed to set GPIO %d LOW: %s", VALVE_2_GPIO, esp_err_to_name(ret_v2_on));
            } else {
                valve_2_state = true; // Update state tracking
                ESP_LOGI(TAG, "VALVE 2 - Pulse complete, valve toggled to ON state");
            }
            break;
            
        case CMD_VALVE_2_OFF:
            ESP_LOGI(TAG, "VALVE 2 OFF - Pulsing valve 2 for 50ms (GPIO %d)", VALVE_2_GPIO);
            
            // Generate 50ms pulse to toggle valve
            esp_err_t ret_v2_off = gpio_set_level(VALVE_2_GPIO, 1);
            if (ret_v2_off != ESP_OK) {
                ESP_LOGE(TAG, "VALVE 2 - Failed to set GPIO %d HIGH: %s", VALVE_2_GPIO, esp_err_to_name(ret_v2_off));
                break;
            }
            
            ESP_LOGI(TAG, "VALVE 2 - Pulse HIGH, holding for 50ms...");
            vTaskDelay(pdMS_TO_TICKS(50)); // 50ms pulse
            
            ret_v2_off = gpio_set_level(VALVE_2_GPIO, 0);
            if (ret_v2_off != ESP_OK) {
                ESP_LOGE(TAG, "VALVE 2 - Failed to set GPIO %d LOW: %s", VALVE_2_GPIO, esp_err_to_name(ret_v2_off));
            } else {
                valve_2_state = false; // Update state tracking
                ESP_LOGI(TAG, "VALVE 2 - Pulse complete, valve toggled to OFF state");
            }
            break;
        case CMD_12V_ON:
            ESP_LOGI(TAG, "12V ON - Activating 12V output");
            gpio_set_level(POWER_12V_GPIO, 1);
            break;
        case CMD_12V_OFF:
            ESP_LOGI(TAG, "12V OFF - Deactivating 12V output");
            gpio_set_level(POWER_12V_GPIO, 0);
            break;
        case CMD_RESET_FLOW:
            ESP_LOGI(TAG, "RESET FLOW - Resetting flow counter");
            pulse_count = 0;
            break;
        case CMD_STATUS:
            ESP_LOGI(TAG, "STATUS - Requested device status");
            
            // Log current tracked valve states (for latching valves)
            ESP_LOGI(TAG, "=== VALVE STATES (TRACKED) ===");
            ESP_LOGI(TAG, "VALVE_1: %s", valve_1_state ? "ON" : "OFF");
            ESP_LOGI(TAG, "VALVE_2: %s", valve_2_state ? "ON" : "OFF");
            
            // Log current GPIO states for debugging (should normally be LOW for pulsed valves)
            ESP_LOGI(TAG, "=== GPIO STATES (CURRENT) ===");
            ESP_LOGI(TAG, "VALVE_1 (GPIO %d): %d (should be 0 - pulsed control)", VALVE_1_GPIO, gpio_get_level(VALVE_1_GPIO));
            ESP_LOGI(TAG, "VALVE_2 (GPIO %d): %d (should be 0 - pulsed control)", VALVE_2_GPIO, gpio_get_level(VALVE_2_GPIO));
            ESP_LOGI(TAG, "POWER_12V (GPIO %d): %d", POWER_12V_GPIO, gpio_get_level(POWER_12V_GPIO));
            ESP_LOGI(TAG, "FLOW_SENSOR (GPIO %d): %d", FLOW_SENSOR_GPIO, gpio_get_level(FLOW_SENSOR_GPIO));
            ESP_LOGI(TAG, "DHT_SENSOR (GPIO %d): %d", DHT_SENSOR_GPIO, gpio_get_level(DHT_SENSOR_GPIO));
            
            ESP_LOGI(TAG, "=== SYSTEM STATUS ===");
            ESP_LOGI(TAG, "Flow pulses: %lu", (unsigned long)pulse_count);
            ESP_LOGI(TAG, "Uptime: %lu seconds", (unsigned long)(esp_timer_get_time() / 1000000));
            ESP_LOGI(TAG, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
            
            // Status will be read via the status characteristic
            break;
        case CMD_RESTART_ADV:
            ESP_LOGI(TAG, "RESTART ADV - Forcing advertising restart");
            force_advertising_restart();
            break;
        case CMD_OTA_INFO:
            ESP_LOGI(TAG, "OTA INFO - Displaying comprehensive partition information");
            
            // Get current running partition
            const esp_partition_t *current = esp_ota_get_running_partition();
            const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
            
            ESP_LOGI(TAG, "=== CURRENT PARTITION INFO ===");
            if (current) {
                ESP_LOGI(TAG, "Running partition: %s", current->label);
                ESP_LOGI(TAG, "  Type: %d, Subtype: %d", current->type, current->subtype);
                ESP_LOGI(TAG, "  Address: 0x%lx, Size: %lu bytes (%.1f MB)", 
                         (unsigned long)current->address, (unsigned long)current->size,
                         (float)current->size / (1024*1024));
                
                // Check if running from factory partition
                if (current->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY) {
                    ESP_LOGW(TAG, "*** RUNNING FROM FACTORY PARTITION ***");
                    ESP_LOGW(TAG, "*** OTA UPDATES MAY NOT WORK PROPERLY ***");
                    ESP_LOGW(TAG, "*** CONSIDER FLASHING TO OTA_0 PARTITION ***");
                }
            } else {
                ESP_LOGE(TAG, "Failed to get current partition!");
            }
            
            ESP_LOGI(TAG, "=== OTA UPDATE PARTITION ===");
            if (next) {
                ESP_LOGI(TAG, "Next OTA partition: %s", next->label);
                ESP_LOGI(TAG, "  Type: %d, Subtype: %d", next->type, next->subtype);
                ESP_LOGI(TAG, "  Address: 0x%lx, Size: %lu bytes (%.1f MB)", 
                         (unsigned long)next->address, (unsigned long)next->size,
                         (float)next->size / (1024*1024));
                ESP_LOGI(TAG, "OTA update partition is AVAILABLE");
            } else {
                ESP_LOGE(TAG, "No OTA partition available!");
                ESP_LOGE(TAG, "This means:");
                ESP_LOGE(TAG, "  1. Partition table may not be flashed correctly");
                ESP_LOGE(TAG, "  2. Device may be using single-app partition table");
                ESP_LOGE(TAG, "  3. All OTA slots may be in use (unlikely)");
            }
            
            ESP_LOGI(TAG, "=== ALL APPLICATION PARTITIONS ===");
            esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, NULL);
            if (it) {
                int count = 0;
                while (it) {
                    const esp_partition_t *part = esp_partition_get(it);
                    count++;
                    const char* subtype_name = "UNKNOWN";
                    // Check partition subtype and assign human-readable name
                    if (part->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY) {
                        subtype_name = "FACTORY";
                    } else if (part->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0) {
                        subtype_name = "OTA_0";
                    } else if (part->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1) {
                        subtype_name = "OTA_1";
                    } else if (part->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_2) {
                        subtype_name = "OTA_2";
                    } else if (part->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_3) {
                        subtype_name = "OTA_3";
                    } else if (part->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_4) {
                        subtype_name = "OTA_4";
                    } else if (part->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_5) {
                        subtype_name = "OTA_5";
                    } else if (part->subtype == ESP_PARTITION_SUBTYPE_APP_TEST) {
                        subtype_name = "TEST";
                    } else {
                        subtype_name = "OTHER";
                    }
                    ESP_LOGI(TAG, "  %d. %s (%s): offset=0x%lx, size=%lu bytes", 
                             count, part->label, subtype_name,
                             (unsigned long)part->address, (unsigned long)part->size);
                    it = esp_partition_next(it);
                }
                esp_partition_iterator_release(it);
                ESP_LOGI(TAG, "Total APP partitions found: %d", count);
                
                if (count < 2) {
                    ESP_LOGE(TAG, "*** INSUFFICIENT PARTITIONS FOR OTA ***");
                    ESP_LOGE(TAG, "*** NEED AT LEAST 2 APP PARTITIONS FOR OTA ***");
                    ESP_LOGE(TAG, "*** CURRENT COUNT: %d ***", count);
                }
            } else {
                ESP_LOGE(TAG, "No APP partitions found at all!");
            }
            
            ESP_LOGI(TAG, "=== RECOMMENDATIONS ===");
            if (!next) {
                ESP_LOGI(TAG, "To fix OTA issues:");
                ESP_LOGI(TAG, "  1. Erase flash completely: 'idf.py erase-flash'");
                ESP_LOGI(TAG, "  2. Flash bootloader, partition table, and app: 'idf.py flash'");
                ESP_LOGI(TAG, "  3. Ensure sdkconfig has CONFIG_PARTITION_TABLE_TWO_OTA=y");
            }
            break;
        case CMD_BLE_STATUS:
            ESP_LOGI(TAG, "BLE STATUS - Displaying comprehensive BLE connection diagnostics");
            
            ESP_LOGI(TAG, "=== BLE CONNECTION STATUS ===");
            ESP_LOGI(TAG, "Connected: %s", is_connected ? "YES" : "NO");
            ESP_LOGI(TAG, "Advertising: %s", is_advertising ? "YES" : "NO");
            ESP_LOGI(TAG, "Connection Handle: %d", conn_handle);
            ESP_LOGI(TAG, "Device Name: %s", ble_svc_gap_device_name());
            
            // Get and display MAC address
            uint8_t own_addr[6];
            int mac_rc = ble_hs_id_copy_addr(ble_addr_type, own_addr, NULL);
            if (mac_rc == 0) {
                ESP_LOGI(TAG, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X", 
                         own_addr[5], own_addr[4], own_addr[3], 
                         own_addr[2], own_addr[1], own_addr[0]);
                
                // Also log as hex bytes to help debug web UI issues
                ESP_LOGI(TAG, "MAC as hex bytes: %02X %02X %02X %02X %02X %02X", 
                         own_addr[0], own_addr[1], own_addr[2], 
                         own_addr[3], own_addr[4], own_addr[5]);
            } else {
                ESP_LOGE(TAG, "Failed to get MAC address: %d", mac_rc);
            }
            
            ESP_LOGI(TAG, "=== BLE STACK STATUS ===");
            ESP_LOGI(TAG, "Address Type: %d", ble_addr_type);
            ESP_LOGI(TAG, "Host Sync Status: %s", ble_hs_synced() ? "SYNCED" : "NOT_SYNCED");
            
            ESP_LOGI(TAG, "=== CONNECTION HISTORY ===");
            ESP_LOGI(TAG, "Uptime: %lu seconds", (unsigned long)(esp_timer_get_time() / 1000000));
            ESP_LOGI(TAG, "Free Heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
            ESP_LOGI(TAG, "Min Free Heap: %lu bytes", (unsigned long)esp_get_minimum_free_heap_size());
            
            ESP_LOGI(TAG, "=== TROUBLESHOOTING ===");
            if (!is_connected && !is_advertising) {
                ESP_LOGW(TAG, "*** NOT CONNECTED AND NOT ADVERTISING ***");
                ESP_LOGW(TAG, "This indicates a BLE stack issue");
                ESP_LOGW(TAG, "Try: Send 'RESTART ADV' command");
            } else if (!is_connected && is_advertising) {
                ESP_LOGI(TAG, "Device is advertising and ready for connection");
                ESP_LOGI(TAG, "If web UI can't connect, try refreshing the browser page");
            } else if (is_connected) {
                ESP_LOGI(TAG, "Device is connected and functioning normally");
                ESP_LOGI(TAG, "Connection handle: %d", conn_handle);
            }
            
            ESP_LOGI(TAG, "=== RECOMMENDATIONS ===");
            ESP_LOGI(TAG, "- For connection issues: Send 'RESTART ADV' command");
            ESP_LOGI(TAG, "- For browser refresh issues: Wait 2-3 seconds before reconnecting");
            ESP_LOGI(TAG, "- Clear browser BLE cache if repeated issues occur");
            break;
        case CMD_TEST_DHT:
            ESP_LOGI(TAG, "TEST DHT - Running comprehensive DHT sensor diagnostic test");
            
            ESP_LOGI(TAG, "=== DHT SENSOR TEST ===");
            ESP_LOGI(TAG, "DHT Sensor GPIO: %d", DHT_SENSOR_GPIO);
            ESP_LOGI(TAG, "DHT Sensor Type: DHT22");
            
            // Enable debug mode for detailed output
            bool old_debug = dht_sensor.debug_enabled;
            dht_sensor.debug_enabled = true;
            
            // Test sensor initialization status
            if (dht_sensor.isr_handler_installed) {
                ESP_LOGI(TAG, "DHT ISR handler: INSTALLED");
            } else {
                ESP_LOGW(TAG, "DHT ISR handler: NOT INSTALLED");
                ESP_LOGW(TAG, "Try reinitializing with: dht_begin(&dht_sensor)");
            }
            
            // Check initial GPIO state
            ESP_LOGI(TAG, "=== GPIO %d STATUS (Before read) ===", DHT_SENSOR_GPIO);
            ESP_LOGI(TAG, "Initial GPIO level: %d", gpio_get_level(DHT_SENSOR_GPIO));
            
            // Perform multiple read attempts for reliability analysis
            ESP_LOGI(TAG, "=== MULTIPLE READ TEST (3 attempts) ===");
            int success_count = 0;
            float temp_readings[3] = {NAN, NAN, NAN};
            float hum_readings[3] = {NAN, NAN, NAN};
            
            for (int attempt = 1; attempt <= 3; attempt++) {
                ESP_LOGI(TAG, "DHT read attempt %d/3...", attempt);
                
                float temp, hum;
                esp_err_t read_err = dht_single_read(&dht_sensor, &temp, &hum);
                
                if (read_err == ESP_OK) {
                    ESP_LOGI(TAG, "  Attempt %d SUCCESS: T=%.2f°C, H=%.2f%%", attempt, temp, hum);
                    temp_readings[attempt-1] = temp;
                    hum_readings[attempt-1] = hum;
                    success_count++;
                    
                    // Update cached values with latest successful read
                    last_temperature = temp;
                    last_humidity = hum;
                } else {
                    ESP_LOGE(TAG, "  Attempt %d FAILED: %s", attempt, esp_err_to_name(read_err));
                }
                
                // Wait between attempts to avoid sensor overload
                if (attempt < 3) {
                    vTaskDelay(pdMS_TO_TICKS(2000)); // 2 second delay
                }
            }
            
            // Analyze results
            ESP_LOGI(TAG, "=== READ RESULTS ANALYSIS ===");
            ESP_LOGI(TAG, "Successful reads: %d/3 (%.1f%%)", success_count, (success_count * 100.0f) / 3.0f);
            
            if (success_count > 0) {
                // Calculate averages and ranges
                float temp_sum = 0, hum_sum = 0;
                float temp_min = 999, temp_max = -999;
                float hum_min = 999, hum_max = -999;
                int valid_temps = 0, valid_hums = 0;
                
                for (int i = 0; i < 3; i++) {
                    if (!isnan(temp_readings[i])) {
                        temp_sum += temp_readings[i];
                        temp_min = fminf(temp_min, temp_readings[i]);
                        temp_max = fmaxf(temp_max, temp_readings[i]);
                        valid_temps++;
                    }
                    if (!isnan(hum_readings[i])) {
                        hum_sum += hum_readings[i];
                        hum_min = fminf(hum_min, hum_readings[i]);
                        hum_max = fmaxf(hum_max, hum_readings[i]);
                        valid_hums++;
                    }
                }
                
                if (valid_temps > 0) {
                    float temp_avg = temp_sum / valid_temps;
                    ESP_LOGI(TAG, "Temperature: Avg=%.2f°C, Range=%.2f-%.2f°C, Span=%.2f°C", 
                             temp_avg, temp_min, temp_max, temp_max - temp_min);
                    ESP_LOGI(TAG, "Temperature (Fahrenheit): Avg=%.2f°F", dht_convert_c_to_f(temp_avg));
                }
                
                if (valid_hums > 0) {
                    float hum_avg = hum_sum / valid_hums;
                    ESP_LOGI(TAG, "Humidity: Avg=%.2f%%, Range=%.2f-%.2f%%, Span=%.2f%%", 
                             hum_avg, hum_min, hum_max, hum_max - hum_min);
                }
                
                // Check for reasonable values
                bool temp_reasonable = (valid_temps > 0 && temp_min > -40 && temp_max < 80);
                bool hum_reasonable = (valid_hums > 0 && hum_min >= 0 && hum_max <= 100);
                
                ESP_LOGI(TAG, "Data validation: Temperature %s, Humidity %s", 
                         temp_reasonable ? "REASONABLE" : "OUT OF RANGE",
                         hum_reasonable ? "REASONABLE" : "OUT OF RANGE");
                
            } else {
                ESP_LOGW(TAG, "No successful reads - sensor may be disconnected or faulty");
                
                // Display cached values if available
                if (!isnan(last_temperature) && !isnan(last_humidity)) {
                    ESP_LOGI(TAG, "Last known values:");
                    ESP_LOGI(TAG, "  Temperature: %.2f°C", last_temperature);
                    ESP_LOGI(TAG, "  Humidity: %.2f%%", last_humidity);
                } else {
                    ESP_LOGW(TAG, "No cached sensor values available");
                }
            }
            
            // Restore original debug setting
            dht_sensor.debug_enabled = old_debug;
            
            ESP_LOGI(TAG, "=== GPIO %d STATUS (After read) ===", DHT_SENSOR_GPIO);
            ESP_LOGI(TAG, "Final GPIO level: %d", gpio_get_level(DHT_SENSOR_GPIO));
            
            ESP_LOGI(TAG, "=== DHT TROUBLESHOOTING ===");
            if (success_count == 0) {
                ESP_LOGE(TAG, "All reads failed - check hardware:");
                ESP_LOGI(TAG, "1. DHT sensor wiring (VCC, GND, Data to GPIO %d)", DHT_SENSOR_GPIO);
                ESP_LOGI(TAG, "2. EXTERNAL 4.7kΩ pull-up resistor on data line (CRITICAL!)");
                ESP_LOGI(TAG, "   - Internal ESP32 pull-up (~45kΩ) may be insufficient");
                ESP_LOGI(TAG, "   - Connect 4.7kΩ resistor between Data pin and VCC");
                ESP_LOGI(TAG, "3. Power supply (3.3V or 5V depending on sensor model)");
                ESP_LOGI(TAG, "4. Sensor type matches (DHT22 configured)");
                ESP_LOGI(TAG, "5. GPIO pin not conflicting with other functions");
            } else if (success_count < 3) {
                ESP_LOGW(TAG, "Intermittent reads - possible issues:");
                ESP_LOGI(TAG, "1. Loose connections or poor solder joints");
                ESP_LOGI(TAG, "2. Insufficient pull-up resistor (try external 4.7kΩ) or long wires");
                ESP_LOGI(TAG, "3. Power supply noise or voltage drops");
                ESP_LOGI(TAG, "4. Sensor aging or environmental interference");
            } else {
                ESP_LOGI(TAG, "Sensor working well - all reads successful");
            }
            
            ESP_LOGI(TAG, "=== PULL-UP RESISTOR INFO ===");
            ESP_LOGI(TAG, "Current configuration: Internal ESP32 pull-up ENABLED (~45kΩ)");
            ESP_LOGI(TAG, "Recommended: EXTERNAL 4.7kΩ pull-up resistor for reliability");
            ESP_LOGI(TAG, "Wiring: Data pin -> 4.7kΩ resistor -> VCC (3.3V/5V)");
            ESP_LOGI(TAG, "Note: External resistor is especially important with:");
            ESP_LOGI(TAG, "  - Wire lengths > 20cm");
            ESP_LOGI(TAG, "  - Breadboard connections");
            ESP_LOGI(TAG, "  - Noisy electrical environments");
            
            ESP_LOGI(TAG, "=== ADDITIONAL TIPS ===");
            ESP_LOGI(TAG, "- Allow 2+ seconds between readings");
            ESP_LOGI(TAG, "- Sensor needs warm-up time after power-on");
            ESP_LOGI(TAG, "- Use short, good quality wires for data connection");
            ESP_LOGI(TAG, "- Check for electromagnetic interference sources");
            break;
        case CMD_DHT_REINIT:
            ESP_LOGI(TAG, "DHT REINIT - Reinitializing DHT sensor");
            
            ESP_LOGI(TAG, "=== DHT SENSOR REINITIALIZATION ===");
            
            // Cleanup existing sensor if initialized
            if (dht_sensor.isr_handler_installed || dht_sensor.service_installed) {
                ESP_LOGI(TAG, "Cleaning up existing DHT sensor configuration...");
                dht_cleanup(&dht_sensor);
                ESP_LOGI(TAG, "DHT cleanup completed");
                
                // Small delay to ensure cleanup is complete
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            
            // Reinitialize sensor
            ESP_LOGI(TAG, "Reinitializing DHT sensor on GPIO %d...", DHT_SENSOR_GPIO);
            dht_init(&dht_sensor, DHT_SENSOR_GPIO, DHT_TYPE_DHT22);
            
            esp_err_t dht_err = dht_begin(&dht_sensor);
            if (dht_err == ESP_OK) {
                ESP_LOGI(TAG, "DHT sensor reinitialized successfully");
                
                // Test with a single read
                ESP_LOGI(TAG, "Testing reinitialized sensor...");
                float temp, hum;
                esp_err_t read_err = dht_single_read(&dht_sensor, &temp, &hum);
                if (read_err == ESP_OK) {
                    last_temperature = temp;
                    last_humidity = hum;
                    ESP_LOGI(TAG, "Test read SUCCESS: Temperature=%.2f°C, Humidity=%.2f%%", temp, hum);
                } else {
                    ESP_LOGW(TAG, "Test read failed: %s", esp_err_to_name(read_err));
                    ESP_LOGW(TAG, "Sensor may need more time to stabilize");
                }
            } else {
                ESP_LOGE(TAG, "Failed to reinitialize DHT sensor: %s", esp_err_to_name(dht_err));
                ESP_LOGE(TAG, "Possible hardware issue or GPIO conflict");
            }
            
            ESP_LOGI(TAG, "=== REINITIALIZATION COMPLETE ===");
            ESP_LOGI(TAG, "Use 'TEST DHT' command to verify sensor functionality");
            break;
        case CMD_UNKNOWN:
        default:
            ESP_LOGW(TAG, "Unknown command: %s", data);
            return BLE_ATT_ERR_INVALID_PDU;
    }
    
    return 0;
}

// Read data from ESP32 defined as server - Status characteristic handler
static int device_read(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // Create status message with device information
    char status_msg[192];
    snprintf(status_msg, sizeof(status_msg), 
             "GRSVC1 Status: Flow=%lu pulses, Uptime=%lu ms, BLE Connected=%s, Advertising=%s", 
             (unsigned long)pulse_count, 
             (unsigned long)(esp_timer_get_time() / 1000),
             is_connected ? "YES" : "NO",
             is_advertising ? "YES" : "NO");
    
    os_mbuf_append(ctxt->om, status_msg, strlen(status_msg));
    return 0;
}

// Device Information Service characteristic access functions
static int device_info_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg) {
    uint16_t uuid = ble_uuid_u16(ctxt->chr->uuid);
    const char *info_string;
    
    switch (uuid) {
        case 0x2A29: // Manufacturer Name String
            info_string = MANUFACTURER_NAME;
            break;
        case 0x2A24: // Model Number String
            info_string = MODEL_NUMBER;
            break;
        case 0x2A26: // Firmware Revision String
            info_string = FIRMWARE_VERSION;
            break;
        case 0x2A27: // Hardware Revision String
            info_string = HARDWARE_REVISION;
            break;
        case 0x2A28: // Software Revision String
            info_string = SOFTWARE_REVISION;
            break;
        case 0xFF04: // Custom UUID for MAC Address (not blocklisted)
            ESP_LOGI(TAG, "=== MAC Address Characteristic Read Request ===");
            ESP_LOGI(TAG, "Returning pre-initialized MAC string: '%s'", device_mac_string);
            info_string = device_mac_string;
            break;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
    
    ESP_LOGI(TAG, "Device info access returning string: '%s' (length: %d)", info_string, strlen(info_string));
    
    int result = os_mbuf_append(ctxt->om, info_string, strlen(info_string));
    ESP_LOGI(TAG, "os_mbuf_append result: %d", result);
    
    return result == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

// Battery ADC initialization
void battery_adc_init() {
    // Configure ADC2 unit
    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

    // Configure ADC2 channel (GPIO12 = ADC2_CH5)
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_5, &config));

    // Try ADC calibration init (optional)
    adc2_cali_chan5_handle = NULL; // Start with no calibration
    ESP_LOGI(TAG, "ADC2 configured for battery monitoring (GPIO12)");
}

// Read battery voltage and convert to percentage (0-100%)
uint8_t read_battery_level() {
    uint32_t adc_reading = 0;
    int raw_value;
    int successful_reads = 0;
    
    // Take multiple samples for better accuracy
    for (int i = 0; i < 32; i++) {
        esp_err_t result = adc_oneshot_read(adc2_handle, ADC_CHANNEL_5, &raw_value);
        if (result == ESP_OK) {
            adc_reading += raw_value;
            successful_reads++;
        } else {
            ESP_LOGW(TAG, "ADC2 read failed, attempt %d", (int)i);
        }
    }
    
    // Check if we got any successful reads
    if (successful_reads == 0) {
        ESP_LOGW(TAG, "All ADC reads failed, returning default 50 percent");
        return 50; // Default to 50% if all reads fail
    }
    
    adc_reading /= successful_reads;
    
    // Convert to voltage (mV) - simple linear conversion
    // ADC range: 0-4095 for 12-bit, voltage range: 0-3300mV with 12dB attenuation
    uint32_t voltage = (adc_reading * 3300) / 4095;
    
    // Assuming voltage divider: Battery -> R1 -> ADC_PIN -> R2 -> GND
    // If using 2:1 voltage divider, multiply by 2
    voltage *= 2; // Adjust this based on your voltage divider circuit
    
    // Convert voltage to battery percentage (assuming 3.0V min, 4.2V max for Li-Ion)
    uint8_t battery_percentage;
    if (voltage >= 4200) {
        battery_percentage = 100;
    } else if (voltage <= 3000) {
        battery_percentage = 0;
    } else {
        // Linear mapping from 3.0V-4.2V to 0-100%
        battery_percentage = (uint8_t)((voltage - 3000) * 100 / 1200);
    }
    
    return battery_percentage;
}

// Battery level characteristic access function
static int battery_level_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            uint8_t battery_level = read_battery_level();
            ESP_LOGI(TAG, "Battery level read: %d percent", (int)battery_level);
            if (os_mbuf_append(ctxt->om, &battery_level, sizeof(battery_level)) == 0) {
                return 0; // Success
            } else {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED; // Read-only characteristic
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

// Flow volume characteristic access function
static int flow_volume_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            // Convert pulse count to flow volume
            float flow_volume = (float)pulse_count * 0.1; // Conversion factor
            if (os_mbuf_append(ctxt->om, &flow_volume, sizeof(float)) == 0) {
                return 0; // Success
            } else {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED; // Read-only characteristic
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

// Flow volume characteristic user description access function
static int flow_volume_desc_access(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_DSC:
            const char *description = "Current Flow Volume in Liters";
            if (os_mbuf_append(ctxt->om, description, strlen(description)) == 0) {
                return 0; // Success
            } else {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
        case BLE_GATT_ACCESS_OP_WRITE_DSC:
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED; // Read-only descriptor
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

// Device status characteristic user description access function
static int device_status_desc_access(uint16_t conn_handle, uint16_t attr_handle,
                                     struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_DSC:
            const char *description = "Device Status";
            if (os_mbuf_append(ctxt->om, description, strlen(description)) == 0) {
                return 0; // Success
            } else {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
        case BLE_GATT_ACCESS_OP_WRITE_DSC:
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED; // Read-only descriptor
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

// Control characteristic user description access function
static int control_desc_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_DSC:
            const char *description = "Device Control Commands";
            if (os_mbuf_append(ctxt->om, description, strlen(description)) == 0) {
                return 0; // Success
            } else {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
        case BLE_GATT_ACCESS_OP_WRITE_DSC:
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED; // Read-only descriptor
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

// Battery level characteristic user description access function
static int battery_desc_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_DSC:
            const char *description = "Battery Level Percentage";
            if (os_mbuf_append(ctxt->om, description, strlen(description)) == 0) {
                return 0; // Success
            } else {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
        case BLE_GATT_ACCESS_OP_WRITE_DSC:
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED; // Read-only descriptor
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

// URI characteristic access function
static int uri_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            const char *uri = "https://www.gelidus.ca/setup-guide-grsvc1";
            if (os_mbuf_append(ctxt->om, uri, strlen(uri)) == 0) {
                return 0; // Success
            } else {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED; // Read-only characteristic
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

// URI characteristic user description access function
static int uri_desc_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_DSC:
            const char *description = "Device Information URI";
            if (os_mbuf_append(ctxt->om, description, strlen(description)) == 0) {
                return 0; // Success
            } else {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
        case BLE_GATT_ACCESS_OP_WRITE_DSC:
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED; // Read-only descriptor
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

// Humidity characteristic access function
static int humidity_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR: {
            // Try to read fresh sensor data
            float temp, hum;
            esp_err_t err = dht_single_read(&dht_sensor, &temp, &hum);
            
            uint16_t humidity;
            if (err == ESP_OK && !isnan(hum)) {
                last_humidity = hum;
                humidity = (uint16_t)(hum * 100); // Convert to hundredths of percent
                ESP_LOGI(TAG, "DHT humidity read: %.2f%%", hum);
            } else {
                // Use last known value or default if sensor read fails
                if (!isnan(last_humidity)) {
                    humidity = (uint16_t)(last_humidity * 100);
                    ESP_LOGW(TAG, "Using last known humidity: %.2f%% (sensor error: %s)", 
                             last_humidity, esp_err_to_name(err));
                } else {
                    humidity = 4500; // Default 45.00% if no previous reading
                    ESP_LOGW(TAG, "Using default humidity: 45.00%% (sensor error: %s)", 
                             esp_err_to_name(err));
                }
            }
            
            if (os_mbuf_append(ctxt->om, &humidity, sizeof(humidity)) == 0) {
                return 0; // Success
            } else {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
        }
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED; // Read-only characteristic
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

// Humidity characteristic user description access function
static int humidity_desc_access(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_DSC:
            const char *description = "Relative Humidity Percentage";
            if (os_mbuf_append(ctxt->om, description, strlen(description)) == 0) {
                return 0; // Success
            } else {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
        case BLE_GATT_ACCESS_OP_WRITE_DSC:
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED; // Read-only descriptor
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

// Temperature characteristic access function
static int temperature_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR: {
            // Try to read fresh sensor data
            float temp, hum;
            esp_err_t err = dht_single_read(&dht_sensor, &temp, &hum);
            
            int16_t temperature;
            if (err == ESP_OK && !isnan(temp)) {
                last_temperature = temp;
                temperature = (int16_t)(temp * 100); // Convert to hundredths of degrees Celsius
                ESP_LOGI(TAG, "DHT temperature read: %.2f°C", temp);
            } else {
                // Use last known value or default if sensor read fails
                if (!isnan(last_temperature)) {
                    temperature = (int16_t)(last_temperature * 100);
                    ESP_LOGW(TAG, "Using last known temperature: %.2f°C (sensor error: %s)", 
                             last_temperature, esp_err_to_name(err));
                } else {
                    temperature = 2350; // Default 23.50°C if no previous reading
                    ESP_LOGW(TAG, "Using default temperature: 23.50°C (sensor error: %s)", 
                             esp_err_to_name(err));
                }
            }
            
            if (os_mbuf_append(ctxt->om, &temperature, sizeof(temperature)) == 0) {
                return 0; // Success
            } else {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
        }
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED; // Read-only characteristic
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

// Temperature characteristic user description access function
static int temperature_desc_access(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_DSC:
            const char *description = "Temperature in Celsius";
            if (os_mbuf_append(ctxt->om, description, strlen(description)) == 0) {
                return 0; // Success
            } else {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
        case BLE_GATT_ACCESS_OP_WRITE_DSC:
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED; // Read-only descriptor
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

// OTA Control characteristic access function
static int ota_control_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            // Return current OTA state
            uint8_t state_info[8];
            state_info[0] = (uint8_t)ota_state;
            state_info[1] = ota_in_progress ? 1 : 0;
            // Progress percentage (0-100)
            uint8_t progress = 0;
            if (ota_total_size > 0) {
                progress = (uint8_t)((ota_bytes_received * 100) / ota_total_size);
            }
            state_info[2] = progress;
            // OTA available flag (check if OTA partition exists)
            const esp_partition_t *next_ota = esp_ota_get_next_update_partition(NULL);
            state_info[3] = next_ota ? 1 : 0; // 1 if OTA available, 0 if not
            // Bytes received (32-bit, little-endian)
            state_info[4] = (ota_bytes_received >> 0) & 0xFF;
            state_info[5] = (ota_bytes_received >> 8) & 0xFF;
            state_info[6] = (ota_bytes_received >> 16) & 0xFF;
            state_info[7] = (ota_bytes_received >> 24) & 0xFF;
            
            if (os_mbuf_append(ctxt->om, state_info, sizeof(state_info)) == 0) {
                return 0;
            } else {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            // OTA Control commands
            if (ctxt->om->om_len < 5) {
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            
            uint8_t *data = ctxt->om->om_data;
            uint8_t command = data[0];
            uint32_t size = (data[4] << 24) | (data[3] << 16) | (data[2] << 8) | data[1];
            
            switch (command) {
                case 0x01: // Start OTA update
                    ESP_LOGI(TAG, "OTA: Starting update, expected size: %lu bytes", (unsigned long)size);
                    
                    if (ota_in_progress) {
                        ESP_LOGW(TAG, "OTA: Update already in progress");
                        return BLE_ATT_ERR_UNLIKELY;
                    }
                    
                    // Validate size is reasonable (not too small or too large)
                    if (size < 1024 || size > 2*1024*1024) { // 1KB to 2MB range
                        ESP_LOGE(TAG, "OTA: Invalid firmware size: %lu bytes", (unsigned long)size);
                        ota_state = OTA_STATE_ERROR;
                        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
                    }
                    
                    // Find next available OTA partition
                    ota_partition = esp_ota_get_next_update_partition(NULL);
                    if (!ota_partition) {
                        ESP_LOGE(TAG, "OTA: No available partition");
                        ESP_LOGE(TAG, "OTA: Current partition table does not support OTA updates");
                        ESP_LOGE(TAG, "OTA: Please reconfigure partition table to include OTA partitions");
                        
                        // List available partitions for debugging
                        esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, NULL);
                        if (it) {
                            ESP_LOGE(TAG, "OTA: Available APP partitions:");
                            while (it) {
                                const esp_partition_t *part = esp_partition_get(it);
                                ESP_LOGE(TAG, "OTA:   - %s: subtype=%d, size=%lu bytes", 
                                         part->label, part->subtype, (unsigned long)part->size);
                                it = esp_partition_next(it);
                            }
                            esp_partition_iterator_release(it);
                        }
                        
                        ota_state = OTA_STATE_ERROR;
                        return BLE_ATT_ERR_UNLIKELY;
                    }
                    
                    // Check if the partition is large enough for the firmware
                    if (size > ota_partition->size) {
                        ESP_LOGE(TAG, "OTA: Firmware size (%lu bytes) exceeds partition size (%lu bytes)", 
                                 (unsigned long)size, (unsigned long)ota_partition->size);
                        ota_state = OTA_STATE_ERROR;
                        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
                    }
                    
                    ESP_LOGI(TAG, "OTA: Using partition %s at offset 0x%lx, size: %lu bytes", 
                             ota_partition->label, (unsigned long)ota_partition->address,
                             (unsigned long)ota_partition->size);
                    
                    // Begin OTA update
                    esp_err_t err = esp_ota_begin(ota_partition, size, &ota_handle);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "OTA: Begin failed: %s", esp_err_to_name(err));
                        if (err == ESP_ERR_INVALID_ARG) {
                            ESP_LOGE(TAG, "OTA: Invalid argument - check partition and size");
                        } else if (err == ESP_ERR_NO_MEM) {
                            ESP_LOGE(TAG, "OTA: Out of memory");
                        } else if (err == ESP_ERR_OTA_PARTITION_CONFLICT) {
                            ESP_LOGE(TAG, "OTA: Partition conflict - already in use");
                        }
                        ota_state = OTA_STATE_ERROR;
                        return BLE_ATT_ERR_UNLIKELY;
                    }
                    
                    // Reset counters
                    ota_bytes_received = 0;
                    ota_total_size = size;
                    ota_in_progress = true;
                    ota_state = OTA_STATE_RECEIVING;
                    ota_last_progress = 0;
                    
                    // Stop advertising during OTA for stability
                    if (is_advertising) {
                        ble_gap_adv_stop();
                        is_advertising = false;
                        ESP_LOGI(TAG, "OTA: Stopped advertising for stability");
                    }
                    
                    ESP_LOGI(TAG, "OTA: Ready to receive firmware data");
                    break;
                    
                case 0x02: // Finish OTA update
                    ESP_LOGI(TAG, "OTA: Finishing update");
                    
                    if (!ota_in_progress) {
                        // Check if auto-finish already completed
                        if (ota_state == OTA_STATE_COMPLETE) {
                            ESP_LOGI(TAG, "OTA: Already completed by auto-finish");
                            return 0; // Success - already finished
                        }
                        ESP_LOGW(TAG, "OTA: No update in progress");
                        return BLE_ATT_ERR_UNLIKELY;
                    }
                    
                    // Check if all data was received
                    if (ota_bytes_received < ota_total_size) {
                        ESP_LOGW(TAG, "OTA: Not all data received yet (%lu/%lu bytes)", 
                                (unsigned long)ota_bytes_received, (unsigned long)ota_total_size);
                        return BLE_ATT_ERR_UNLIKELY;
                    }
                    
                    ota_state = OTA_STATE_VALIDATING;
                    
                    // End OTA update
                    esp_err_t end_err = esp_ota_end(ota_handle);
                    if (end_err != ESP_OK) {
                        ESP_LOGE(TAG, "OTA: End failed: %s", esp_err_to_name(end_err));
                        ota_state = OTA_STATE_ERROR;
                        ota_in_progress = false;
                        return BLE_ATT_ERR_UNLIKELY;
                    }
                    
                    // Set new boot partition
                    esp_err_t set_err = esp_ota_set_boot_partition(ota_partition);
                    if (set_err != ESP_OK) {
                        ESP_LOGE(TAG, "OTA: Set boot partition failed: %s", esp_err_to_name(set_err));
                        ota_state = OTA_STATE_ERROR;
                        ota_in_progress = false;
                        return BLE_ATT_ERR_UNLIKELY;
                    }
                    
                    ota_state = OTA_STATE_COMPLETE;
                    ota_in_progress = false;
                    
                    ESP_LOGI(TAG, "OTA: Update completed successfully! Restarting in 3 seconds...");
                    
                    // Schedule restart
                    vTaskDelay(pdMS_TO_TICKS(3000));
                    esp_restart();
                    break;
                    
                case 0x03: // Cancel OTA update
                    ESP_LOGI(TAG, "OTA: Canceling update");
                    
                    if (ota_in_progress && ota_handle) {
                        esp_ota_abort(ota_handle);
                    }
                    
                    ota_in_progress = false;
                    ota_state = OTA_STATE_IDLE;
                    ota_bytes_received = 0;
                    ota_total_size = 0;
                    
                    // Restart advertising after OTA cancellation
                    if (is_connected && !is_advertising) {
                        ESP_LOGI(TAG, "OTA: Restarting advertising after cancellation");
                        // Don't restart advertising if still connected - that's normal
                    }
                    ota_handle = 0;
                    break;
                    
                default:
                    ESP_LOGW(TAG, "OTA: Unknown command: 0x%02X", command);
                    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            
            return 0;
            
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

// OTA Data characteristic access function
static int ota_data_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            if (!ota_in_progress || ota_state != OTA_STATE_RECEIVING) {
                ESP_LOGW(TAG, "OTA: Data received but no update in progress");
                return BLE_ATT_ERR_UNLIKELY;
            }
            
            // Write firmware data to OTA partition
            size_t data_len = ctxt->om->om_len;
            uint8_t *data = ctxt->om->om_data;
            
            if (data_len == 0) {
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            
            // Check if we're not exceeding the expected size
            if (ota_bytes_received + data_len > ota_total_size) {
                ESP_LOGE(TAG, "OTA: Data exceeds expected size");
                ota_state = OTA_STATE_ERROR;
                return BLE_ATT_ERR_UNLIKELY;
            }
            
            // Write data to OTA partition
            esp_err_t err = esp_ota_write(ota_handle, data, data_len);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "OTA: Write failed: %s", esp_err_to_name(err));
                ota_state = OTA_STATE_ERROR;
                return BLE_ATT_ERR_UNLIKELY;
            }
            
            ota_bytes_received += data_len;
            
            // Log progress every 10%
            uint32_t progress = (ota_bytes_received * 100) / ota_total_size;
            if (progress >= ota_last_progress + 10) {
                ESP_LOGI(TAG, "OTA: Progress %lu%% (%lu/%lu bytes)", 
                         (unsigned long)progress, 
                         (unsigned long)ota_bytes_received, 
                         (unsigned long)ota_total_size);
                ota_last_progress = progress;
            }
            
            // Auto-finish OTA when all data is received
            if (ota_bytes_received >= ota_total_size) {
                ESP_LOGI(TAG, "OTA: All data received, auto-finishing update...");
                
                ota_state = OTA_STATE_VALIDATING;
                
                // End OTA update
                esp_err_t end_err = esp_ota_end(ota_handle);
                if (end_err != ESP_OK) {
                    ESP_LOGE(TAG, "OTA: Auto-finish end failed: %s", esp_err_to_name(end_err));
                    ota_state = OTA_STATE_ERROR;
                    ota_in_progress = false;
                    return BLE_ATT_ERR_UNLIKELY;
                }
                
                // Set new boot partition
                esp_err_t set_err = esp_ota_set_boot_partition(ota_partition);
                if (set_err != ESP_OK) {
                    ESP_LOGE(TAG, "OTA: Auto-finish set boot partition failed: %s", esp_err_to_name(set_err));
                    ota_state = OTA_STATE_ERROR;
                    ota_in_progress = false;
                    return BLE_ATT_ERR_UNLIKELY;
                }
                
                ota_state = OTA_STATE_COMPLETE;
                ota_in_progress = false;
                
                ESP_LOGI(TAG, "OTA: Auto-finish completed! Restarting in 2 seconds...");
                
                // Schedule restart (shorter delay for auto-finish)
                vTaskDelay(pdMS_TO_TICKS(2000));
                esp_restart();
            }
            
            return 0;
            
        case BLE_GATT_ACCESS_OP_READ_CHR:
            return BLE_ATT_ERR_READ_NOT_PERMITTED; // Write-only characteristic
            
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

// OTA Control characteristic user description access function
static int ota_control_desc_access(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_DSC:
            const char *description = "OTA Firmware Update Control";
            if (os_mbuf_append(ctxt->om, description, strlen(description)) == 0) {
                return 0;
            } else {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
        case BLE_GATT_ACCESS_OP_WRITE_DSC:
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

// OTA Data characteristic user description access function
static int ota_data_desc_access(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_DSC:
            const char *description = "OTA Firmware Update Data";
            if (os_mbuf_append(ctxt->om, description, strlen(description)) == 0) {
                return 0;
            } else {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
        case BLE_GATT_ACCESS_OP_WRITE_DSC:
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

// Array of pointers to other service definitions
// Custom GRSVC1 Multi-Function Service UUID + Device Information Service
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        // Custom GRSVC1 Multi-Function Service
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x1815),                // Automation IO service UUID for GRSVC1
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = BLE_UUID16_DECLARE(0xFF01),         // Custom Control characteristic (non-blocklisted)
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .access_cb = device_write,
                .descriptors = (struct ble_gatt_dsc_def[]){
                    {
                        .uuid = BLE_UUID16_DECLARE(0x2901),     // Characteristic User Description
                        .access_cb = control_desc_access,
                        .att_flags = BLE_ATT_F_READ,
                    },
                    {0} // End of descriptors
                }
            },
            {
                .uuid = BLE_UUID16_DECLARE(0x2B3C),             // Status
                .flags = BLE_GATT_CHR_F_READ,
                .access_cb = device_read,
                .descriptors = (struct ble_gatt_dsc_def[]){
                    {
                        .uuid = BLE_UUID16_DECLARE(0x2901),     // Characteristic User Description
                        .access_cb = device_status_desc_access,
                        .att_flags = BLE_ATT_F_READ,
                    },
                    {0} // End of descriptors
                }
            },
            {
                .uuid = BLE_UUID16_DECLARE(0x2B1B),         // Flow Volume characteristic
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .access_cb = flow_volume_chr_access,
                .descriptors = (struct ble_gatt_dsc_def[]){
                    {
                        .uuid = BLE_UUID16_DECLARE(0x2901),     // Characteristic User Description
                        .access_cb = flow_volume_desc_access,
                        .att_flags = BLE_ATT_F_READ,
                    },
                    {0} // End of descriptors
                }
            },
            {
                .uuid = BLE_UUID16_DECLARE(0x2A19),         // Battery Level characteristic
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .access_cb = battery_level_chr_access,
                .descriptors = (struct ble_gatt_dsc_def[]){
                    {
                        .uuid = BLE_UUID16_DECLARE(0x2901),     // Characteristic User Description
                        .access_cb = battery_desc_access,
                        .att_flags = BLE_ATT_F_READ,
                    },
                    {0} // End of descriptors
                }
            },
            {
                .uuid = BLE_UUID16_DECLARE(0x2A6F),         // Humidity characteristic
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .access_cb = humidity_chr_access,
                .descriptors = (struct ble_gatt_dsc_def[]){
                    {
                        .uuid = BLE_UUID16_DECLARE(0x2901),     // Characteristic User Description
                        .access_cb = humidity_desc_access,
                        .att_flags = BLE_ATT_F_READ,
                    },
                    {0} // End of descriptors
                }
            },
            {
                .uuid = BLE_UUID16_DECLARE(0x2AB6),         // URI characteristic
                .flags = BLE_GATT_CHR_F_READ,
                .access_cb = uri_chr_access,
                .descriptors = (struct ble_gatt_dsc_def[]){
                    {
                        .uuid = BLE_UUID16_DECLARE(0x2901),     // Characteristic User Description
                        .access_cb = uri_desc_access,
                        .att_flags = BLE_ATT_F_READ,
                    },
                    {0} // End of descriptors
                }
            },
            {
                .uuid = BLE_UUID16_DECLARE(0x2A6E),         // Temperature characteristic
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .access_cb = temperature_chr_access,
                .descriptors = (struct ble_gatt_dsc_def[]){
                    {
                        .uuid = BLE_UUID16_DECLARE(0x2901),     // Characteristic User Description
                        .access_cb = temperature_desc_access,
                        .att_flags = BLE_ATT_F_READ,
                    },
                    {0} // End of descriptors
                }
            },
            {
                .uuid = BLE_UUID16_DECLARE(0xFF02),         // OTA Control characteristic (custom UUID)
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .access_cb = ota_control_chr_access,
                .descriptors = (struct ble_gatt_dsc_def[]){
                    {
                        .uuid = BLE_UUID16_DECLARE(0x2901),     // Characteristic User Description
                        .access_cb = ota_control_desc_access,
                        .att_flags = BLE_ATT_F_READ,
                    },
                    {0} // End of descriptors
                }
            },
            {
                .uuid = BLE_UUID16_DECLARE(0xFF03),         // OTA Data characteristic (custom UUID)
                .flags = BLE_GATT_CHR_F_WRITE,
                .access_cb = ota_data_chr_access,
                .descriptors = (struct ble_gatt_dsc_def[]){
                    {
                        .uuid = BLE_UUID16_DECLARE(0x2901),     // Characteristic User Description
                        .access_cb = ota_data_desc_access,
                        .att_flags = BLE_ATT_F_READ,
                    },
                    {0} // End of descriptors
                }
            },
            {0} // End of characteristics
        }
    },
    {
        // Device Information Service
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180A),                // Device Information Service UUID
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = BLE_UUID16_DECLARE(0x2A29),         // Manufacturer Name String
                .flags = BLE_GATT_CHR_F_READ,
                .access_cb = device_info_access
            },
            {
                .uuid = BLE_UUID16_DECLARE(0x2A26),         // Firmware Revision String
                .flags = BLE_GATT_CHR_F_READ,
                .access_cb = device_info_access
            },
            {
                .uuid = BLE_UUID16_DECLARE(0x2A27),         // Hardware Revision String
                .flags = BLE_GATT_CHR_F_READ,
                .access_cb = device_info_access
            },
            {
                .uuid = BLE_UUID16_DECLARE(0x2A24),         // Model Number String
                .flags = BLE_GATT_CHR_F_READ,
                .access_cb = device_info_access
            },
            {
                .uuid = BLE_UUID16_DECLARE(0x2A28),         // Software Revision String
                .flags = BLE_GATT_CHR_F_READ,
                .access_cb = device_info_access
            },
            {
                .uuid = BLE_UUID16_DECLARE(0xFF04),         // Custom UUID for MAC address (not blocklisted)
                .flags = BLE_GATT_CHR_F_READ,
                .access_cb = device_info_access
            },
            {0} // End of characteristics
        }
    },
    {0} // End of services
};

// BLE event handling
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    // Advertise if connected
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!");
        if (event->connect.status == 0)
        {
            // Check if we're already connected - reject new connections
            if (is_connected && conn_handle != BLE_HS_CONN_HANDLE_NONE) {
                ESP_LOGW("GAP", "Already connected (handle: %d), rejecting new connection (handle: %d)", 
                         conn_handle, event->connect.conn_handle);
                // Disconnect the new connection immediately
                ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                return 0;
            }
            
            // Check if OTA is in progress - reject connections during OTA
            if (ota_in_progress) {
                ESP_LOGW("GAP", "OTA in progress, rejecting connection attempt");
                ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                return 0;
            }
            
            // Connection successful
            conn_handle = event->connect.conn_handle;
            is_connected = true;
            is_advertising = false;
            ESP_LOGI("GAP", "Connection established, handle: %d", conn_handle);
            
            // Read fresh DHT sensor data for immediate availability to connected client
            ESP_LOGI("GAP", "Reading fresh sensor data for connected client...");
            float temp, hum;
            esp_err_t read_err = dht_single_read(&dht_sensor, &temp, &hum);
            if (read_err == ESP_OK) {
                last_temperature = temp;
                last_humidity = hum;
                ESP_LOGI("GAP", "Fresh sensor data: Temperature=%.2f°C, Humidity=%.2f%%", temp, hum);
            } else {
                ESP_LOGW("GAP", "Failed to read fresh sensor data: %s", esp_err_to_name(read_err));
                ESP_LOGW("GAP", "Client will receive last known values: Temperature=%.2f°C, Humidity=%.2f%%", 
                         last_temperature, last_humidity);
            }
        }
        else
        {
            // Connection failed, restart advertising
            conn_handle = BLE_HS_CONN_HANDLE_NONE;
            is_connected = false;
            ESP_LOGE("GAP", "Connection failed with status: %d", event->connect.status);
            ble_app_advertise();
        }
        break;
    // Advertise again after completion of the event
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT DISCONNECTED - Reason: %d", event->disconnect.reason);
        
        // Update connection state immediately
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        is_connected = false;
        is_advertising = false;
        
        // Log disconnect reason for debugging
        const char* disconnect_reason = "Unknown";
        switch (event->disconnect.reason) {
            case BLE_HS_ENOTCONN: disconnect_reason = "Not connected"; break;
            case BLE_HS_ETIMEOUT: disconnect_reason = "Timeout"; break;
            case BLE_HS_EDONE: disconnect_reason = "Connection terminated by peer"; break;
            case BLE_HS_ECONTROLLER: disconnect_reason = "Controller error"; break;
            case 0x13: disconnect_reason = "Remote user terminated connection"; break;
            case 0x16: disconnect_reason = "Connection terminated by local host"; break;
            case 0x3d: disconnect_reason = "Unacceptable connection parameters"; break;
        }
        ESP_LOGI("GAP", "Disconnect reason: %s (0x%02X)", disconnect_reason, event->disconnect.reason);
        
        // If OTA was in progress, reset state
        if (ota_in_progress) {
            ESP_LOGW("GAP", "OTA interrupted by disconnect, resetting OTA state");
            if (ota_handle) {
                esp_ota_abort(ota_handle);
                ota_handle = 0;
            }
            ota_in_progress = false;
            ota_state = OTA_STATE_IDLE;
            ota_bytes_received = 0;
            ota_total_size = 0;
        }
        
        // Immediate advertising restart for better reconnection
        ESP_LOGI("GAP", "Immediately restarting advertising after disconnect");
        
        // Small delay to ensure proper cleanup before restart
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Force advertising restart
        ble_app_advertise();
        
        // If advertising didn't start immediately, try again after a short delay
        if (!is_advertising) {
            ESP_LOGW("GAP", "Initial advertising restart failed, retrying in 500ms");
            vTaskDelay(pdMS_TO_TICKS(500));
            ble_app_advertise();
        }
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI("GAP", "BLE GAP EVENT ADV_COMPLETE - Reason: %d", event->adv_complete.reason);
        is_advertising = false;
        
        // If not connected and advertising completed, restart advertising
        if (!is_connected) {
            ESP_LOGI("GAP", "Advertising completed, restarting...");
            ble_app_advertise();
        }
        break;
    default:
        ESP_LOGD("GAP", "BLE GAP EVENT: %d", event->type);
        break;
    }
    return 0;
}

// Define the BLE connection
void ble_app_advertise(void)
{
    // Don't start advertising if already connected
    if (is_connected) {
        ESP_LOGD(TAG, "Already connected, skipping advertising...");
        return;
    }
    
    // Don't advertise during OTA operations for stability
    if (ota_in_progress) {
        ESP_LOGD(TAG, "OTA in progress, skipping advertising...");
        return;
    }

    // If already advertising, log but don't return - might need to restart
    if (is_advertising) {
        ESP_LOGD(TAG, "Already advertising, but continuing to ensure proper state...");
    }

    // Get the device's own MAC address
    uint8_t own_addr[6];
    int rc = ble_hs_id_copy_addr(ble_addr_type, own_addr, NULL);
    if (rc == 0) {
        ESP_LOGD(TAG, "Device MAC Address: %02X:%02X:%02X:%02X:%02X:%02X", 
                 own_addr[5], own_addr[4], own_addr[3], 
                 own_addr[2], own_addr[1], own_addr[0]);
    } else {
        ESP_LOGE(TAG, "Failed to get MAC address: %d", rc);
        // Continue anyway, advertising without MAC in manufacturer data
    }

    // GAP - device name definition
    struct ble_hs_adv_fields fields;
    const char *device_name;
    memset(&fields, 0, sizeof(fields));
    device_name = ble_svc_gap_device_name(); // Read the BLE device name
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    // Add manufacturer specific data containing MAC address
    static uint8_t mfg_data[8]; // 2 bytes company ID + 6 bytes MAC
    mfg_data[0] = 0xFF; // Company ID LSB (0xFFFF = test/development)
    mfg_data[1] = 0xFF; // Company ID MSB
    if (rc == 0) {
        // Copy MAC address in reverse order (little-endian)
        memcpy(&mfg_data[2], own_addr, 6);
        fields.mfg_data = mfg_data;
        fields.mfg_data_len = sizeof(mfg_data);
    } else {
        // Don't include manufacturer data if MAC address is unavailable
        fields.mfg_data = NULL;
        fields.mfg_data_len = 0;
    }

    // Set advertising fields
    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set advertising fields: %d (%s)", rc, 
                 rc == BLE_HS_EBUSY ? "busy" : 
                 rc == BLE_HS_EINVAL ? "invalid" : "unknown");
        
        // Try to stop advertising first and retry
        ble_gap_adv_stop();
        vTaskDelay(pdMS_TO_TICKS(100));
        
        rc = ble_gap_adv_set_fields(&fields);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to set advertising fields after retry: %d", rc);
            return;
        }
    }

    // GAP - device connectivity definition
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // discoverable
    
    // Use faster advertising intervals for better reconnection
    adv_params.itvl_min = BLE_GAP_ADV_FAST_INTERVAL1_MIN;  // 20ms
    adv_params.itvl_max = BLE_GAP_ADV_FAST_INTERVAL1_MAX;  // 30ms
    
    // Start advertising
    rc = ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    if (rc == 0) {
        is_advertising = true;
        ESP_LOGI(TAG, "BLE advertising started successfully (fast intervals for quick reconnection)");
    } else {
        ESP_LOGE(TAG, "Failed to start BLE advertising: %d (%s)", rc,
                 rc == BLE_HS_EALREADY ? "already advertising" :
                 rc == BLE_HS_EBUSY ? "busy" :
                 rc == BLE_HS_EINVAL ? "invalid parameters" : "unknown");
        
        is_advertising = false;
        
        // If already advertising error, consider it success
        if (rc == BLE_HS_EALREADY) {
            ESP_LOGI(TAG, "Advertising already active, updating state");
            is_advertising = true;
        }
    }
}

// Force advertising restart - useful for debugging connection issues
void force_advertising_restart(void)
{
    ESP_LOGI(TAG, "Forcing advertising restart...");
    
    // Always stop current advertising first, regardless of state
    int rc = ble_gap_adv_stop();
    if (rc == 0) {
        ESP_LOGI(TAG, "Stopped current advertising");
    } else {
        ESP_LOGW(TAG, "Failed to stop advertising (may not have been active): %d", rc);
    }
    
    // Reset advertising state
    is_advertising = false;
    
    // Reset connection state if needed
    if (!is_connected) {
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
    }
    
    // Wait for proper cleanup
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Start advertising again with retry logic
    ESP_LOGI(TAG, "Attempting to restart advertising...");
    ble_app_advertise();
    
    // Wait a bit and verify advertising started
    vTaskDelay(pdMS_TO_TICKS(300));
    if (!is_advertising) {
        ESP_LOGW(TAG, "Advertising restart failed after multiple attempts");
    } else {
        ESP_LOGI(TAG, "Advertising restart successful");
    }
}

// Initialize output GPIOs for valves and 12V power
void output_gpio_init() {
    ESP_LOGI(TAG, "Initializing output GPIOs...");
    
    // Configure valve, power output, and status LED GPIOs
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << VALVE_1_GPIO) | (1ULL << VALVE_2_GPIO) | (1ULL << POWER_12V_GPIO) | (1ULL << STATUS_LED_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,   // Disable pull-down - may conflict with external hardware
        .pull_up_en = GPIO_PULLUP_DISABLE,       // Disable pull-up as well
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "GPIO configuration successful");
    
    // Set all outputs to low (inactive) state initially
    ESP_LOGI(TAG, "Setting initial GPIO states to LOW...");
    
    ret = gpio_set_level(VALVE_1_GPIO, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set VALVE_1_GPIO initial level: %s", esp_err_to_name(ret));
    }
    
    ret = gpio_set_level(VALVE_2_GPIO, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set VALVE_2_GPIO initial level: %s", esp_err_to_name(ret));
    }
    
    ret = gpio_set_level(POWER_12V_GPIO, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set POWER_12V_GPIO initial level: %s", esp_err_to_name(ret));
    }
    
    ret = gpio_set_level(STATUS_LED_GPIO, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set STATUS_LED_GPIO initial level: %s", esp_err_to_name(ret));
    }
    
    // Wait a moment for GPIOs to settle
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Verify the initial states
    ESP_LOGI(TAG, "Initial GPIO states after configuration:");
    ESP_LOGI(TAG, "  VALVE_1 (GPIO %d): %d", VALVE_1_GPIO, gpio_get_level(VALVE_1_GPIO));
    ESP_LOGI(TAG, "  VALVE_2 (GPIO %d): %d", VALVE_2_GPIO, gpio_get_level(VALVE_2_GPIO));
    ESP_LOGI(TAG, "  POWER_12V (GPIO %d): %d", POWER_12V_GPIO, gpio_get_level(POWER_12V_GPIO));
    ESP_LOGI(TAG, "  STATUS_LED (GPIO %d): %d", STATUS_LED_GPIO, gpio_get_level(STATUS_LED_GPIO));
    
    // Test setting each GPIO HIGH briefly to verify they work
    ESP_LOGI(TAG, "Testing GPIO functionality...");
    
    // Test VALVE_1
    ESP_LOGI(TAG, "Testing VALVE_1 (GPIO %d):", VALVE_1_GPIO);
    gpio_set_level(VALVE_1_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    int valve1_test = gpio_get_level(VALVE_1_GPIO);
    gpio_set_level(VALVE_1_GPIO, 0);
    ESP_LOGI(TAG, "  VALVE_1 test HIGH result: %d %s", valve1_test, valve1_test == 1 ? "(OK)" : "(FAILED)");
    
    // Test VALVE_2  
    ESP_LOGI(TAG, "Testing VALVE_2 (GPIO %d):", VALVE_2_GPIO);
    gpio_set_level(VALVE_2_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    int valve2_test = gpio_get_level(VALVE_2_GPIO);
    gpio_set_level(VALVE_2_GPIO, 0);
    ESP_LOGI(TAG, "  VALVE_2 test HIGH result: %d %s", valve2_test, valve2_test == 1 ? "(OK)" : "(FAILED)");
      // Test POWER_12V
    ESP_LOGI(TAG, "Testing POWER_12V (GPIO %d):", POWER_12V_GPIO);
    gpio_set_level(POWER_12V_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    int power12v_test = gpio_get_level(POWER_12V_GPIO);
    gpio_set_level(POWER_12V_GPIO, 0);
    ESP_LOGI(TAG, "  POWER_12V test HIGH result: %d %s", power12v_test, power12v_test == 1 ? "(OK)" : "(FAILED)");
    
    // Test STATUS_LED
    ESP_LOGI(TAG, "Testing STATUS_LED (GPIO %d):", STATUS_LED_GPIO);
    gpio_set_level(STATUS_LED_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    int status_led_test = gpio_get_level(STATUS_LED_GPIO);
    gpio_set_level(STATUS_LED_GPIO, 0);
    ESP_LOGI(TAG, "  STATUS_LED test HIGH result: %d %s", status_led_test, status_led_test == 1 ? "(OK)" : "(FAILED)");
    
    ESP_LOGI(TAG, "Output GPIOs initialized: VALVE_1=%d, VALVE_2=%d, POWER_12V=%d, STATUS_LED=%d", 
             VALVE_1_GPIO, VALVE_2_GPIO, POWER_12V_GPIO, STATUS_LED_GPIO);
             
    // GPIO hardware information
    ESP_LOGI(TAG, "GPIO Hardware Notes:");
    ESP_LOGI(TAG, "  GPIO33: Output-only pin, no input capability");
    ESP_LOGI(TAG, "  GPIO27: General purpose I/O pin");  
    ESP_LOGI(TAG, "  GPIO14: General purpose I/O pin");
    ESP_LOGI(TAG, "  GPIO4: Status LED pin");
    ESP_LOGI(TAG, "  GPIO5: DHT sensor pin (moved from GPIO4 due to LED conflict)");
    ESP_LOGI(TAG, "  If GPIOs fail to set HIGH, check for external pull-downs or hardware conflicts");
}

// The application
void ble_app_on_sync(void)
{
    ESP_LOGI(TAG, "BLE stack synchronized, starting services...");
    
    // Determine the best address type automatically
    int rc = ble_hs_id_infer_auto(0, &ble_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to infer address type: %d", rc);
    } else {
        ESP_LOGI(TAG, "Address type inferred: %d", ble_addr_type);
    }
    
    // Get and log our BLE address
    uint8_t own_addr[6];
    rc = ble_hs_id_copy_addr(ble_addr_type, own_addr, NULL);
    if (rc == 0) {
        ESP_LOGI(TAG, "BLE Address: %02X:%02X:%02X:%02X:%02X:%02X", 
                 own_addr[5], own_addr[4], own_addr[3], 
                 own_addr[2], own_addr[1], own_addr[0]);
    }
    
    // Initialize the MAC address string for device info characteristic
    init_device_mac_string();
    
    // Verify GATT services are available
    ESP_LOGI(TAG, "BLE services should now be discoverable");
    ESP_LOGI(TAG, "Expected services:");
    ESP_LOGI(TAG, "  - Automation IO (0x1815) with 9 characteristics");
    ESP_LOGI(TAG, "  - Device Information (0x180A) with 5 characteristics");
    ESP_LOGI(TAG, "Total expected characteristics: 14");
    
    // Start advertising
    ble_app_advertise();
}

// Connection monitoring task - ensures advertising is always active when not connected
void connection_monitor_task(void *param)
{
    static uint32_t last_status_log = 0;
    static uint32_t last_adv_attempt = 0;
    static bool prev_connected = false;
    static bool prev_advertising = false;
    static uint8_t retry_count = 0;
    static uint32_t disconnect_time = 0;
    
    ESP_LOGI(TAG, "Connection monitor task started");
    
    while (1) {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Track when we become disconnected
        if (prev_connected && !is_connected) {
            disconnect_time = current_time;
            ESP_LOGI(TAG, "Connection lost, starting aggressive reconnection mode");
        }
        
        // Check if we should be advertising but aren't
        if (!is_connected && !is_advertising) {
            uint32_t retry_interval = 1000; // Default 1 second
            
            // Use faster retry for the first 10 seconds after disconnect
            if (current_time - disconnect_time < 10000) {
                retry_interval = 500; // 500ms for quick reconnection
            } else if (current_time - disconnect_time < 30000) {
                retry_interval = 1000; // 1 second for medium-term
            } else {
                retry_interval = 2000; // 2 seconds for long-term
            }
            
            // Check if enough time has passed since last attempt
            if (current_time - last_adv_attempt > retry_interval) {
                ESP_LOGW(TAG, "Not connected and not advertising - starting advertising (retry %d, interval %dms)", 
                         retry_count, (int)retry_interval);
                
                // Force stop any existing advertising first
                ble_gap_adv_stop();
                vTaskDelay(pdMS_TO_TICKS(50));
                
                ble_app_advertise();
                last_adv_attempt = current_time;
                
                // Wait a bit and check if advertising started
                vTaskDelay(pdMS_TO_TICKS(200));
                if (!is_advertising) {
                    retry_count++;
                    ESP_LOGW(TAG, "Advertising start failed, attempt %d", retry_count);
                    
                    // Reset BLE stack if too many failures
                    if (retry_count > 5) {
                        ESP_LOGE(TAG, "Too many advertising failures, attempting BLE reset");
                        
                        // Stop advertising completely
                        ble_gap_adv_stop();
                        
                        // Wait a moment
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        
                        // Try to restart advertising
                        ble_app_advertise();
                        
                        retry_count = 0; // Reset counter after attempting recovery
                        vTaskDelay(pdMS_TO_TICKS(2000)); // Wait before next check
                    }
                } else {
                    retry_count = 0; // Reset on success
                    ESP_LOGI(TAG, "Advertising restarted successfully");
                }
            }
        } else {
            retry_count = 0; // Reset when connected or advertising
        }
        
        // Log state changes
        if (is_connected != prev_connected) {
            ESP_LOGI(TAG, "Connection: %s", is_connected ? "CONNECTED" : "DISCONNECTED");
            if (is_connected) {
                ESP_LOGI(TAG, "Connection established successfully, handle: %d", conn_handle);
            }
            prev_connected = is_connected;
        }
        
        if (is_advertising != prev_advertising) {
            ESP_LOGI(TAG, "Advertising: %s", is_advertising ? "ACTIVE" : "INACTIVE");
            prev_advertising = is_advertising;
        }
        
        // Periodic status log (every 30 seconds) - simplified
        if (current_time - last_status_log > 30000) {
            ESP_LOGI(TAG, "BLE Status - Conn:%s Adv:%s Handle:%d Uptime:%ds", 
                     is_connected ? "Y" : "N", 
                     is_advertising ? "Y" : "N",
                     conn_handle,
                     (int)(current_time / 1000));
            last_status_log = current_time;
        }
        
        // Check more frequently for better responsiveness
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check every 1 second instead of 5
    }
}

// The infinite task
void host_task(void *param)
{
    nimble_port_run(); // This function will return only when nimble_port_stop() is executed
}

void app_main()
{
    nvs_flash_init();                          // Initialize NVS flash using
    
    // Initialize flow sensor
    flow_sensor_init();
    ESP_LOGI(TAG, "Flow sensor initialized on GPIO %d", FLOW_SENSOR_GPIO);
    
    // Initialize output GPIOs for valves and 12V power
    output_gpio_init();
    ESP_LOGI(TAG, "Output GPIOs initialized with pull-down enabled");
    
    // Initialize DHT sensor
    ESP_LOGI(TAG, "Initializing DHT sensor on GPIO %d...", DHT_SENSOR_GPIO);
    dht_init(&dht_sensor, DHT_SENSOR_GPIO, DHT_TYPE_DHT22);
    esp_err_t dht_err = dht_begin(&dht_sensor);
    if (dht_err == ESP_OK) {
        ESP_LOGI(TAG, "DHT sensor initialized successfully");
        
        // Perform initial sensor reading to verify functionality
        float temp, hum;
        esp_err_t read_err = dht_single_read(&dht_sensor, &temp, &hum);
        if (read_err == ESP_OK) {
            last_temperature = temp;
            last_humidity = hum;
            ESP_LOGI(TAG, "Initial DHT reading: Temperature=%.2f°C, Humidity=%.2f%%", temp, hum);
        } else {
            ESP_LOGW(TAG, "Initial DHT reading failed: %s", esp_err_to_name(read_err));
            ESP_LOGW(TAG, "DHT sensor may not be connected or may need warming up");
        }
    } else {
        ESP_LOGE(TAG, "Failed to initialize DHT sensor: %s", esp_err_to_name(dht_err));
        ESP_LOGW(TAG, "Temperature and humidity readings will use default values");
    }
    
    // Initialize BLE stack
    ESP_LOGI(TAG, "Initializing BLE stack...");
    // esp_nimble_hci_and_controller_init();   // Initialize ESP controller
    nimble_port_init();                        // Initialize the host stack
    ble_svc_gap_device_name_set(DEVICE_NAME);  // Initialize NimBLE configuration - server name
    ble_svc_gap_init();                        // Initialize NimBLE configuration - gap service
    ble_svc_gatt_init();                       // Initialize NimBLE configuration - gatt service
    
    // Count and validate GATT services
    int rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to count GATT services: %d", rc);
    } else {
        ESP_LOGI(TAG, "GATT services counted successfully");
    }
    
    // Add GATT services
    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to add GATT services: %d", rc);
    } else {
        ESP_LOGI(TAG, "GATT services added successfully");
        
        // Log service details
        ESP_LOGI(TAG, "Registered BLE Services:");
        ESP_LOGI(TAG, "  1. Automation IO Service (0x1815) with %d characteristics:", 9);
        ESP_LOGI(TAG, "     - Control Point (0xFF01) - Read/Write");
        ESP_LOGI(TAG, "     - Status (0x2B3C) - Read");
        ESP_LOGI(TAG, "     - Volume Flow characteristic UUID: 0x2B1B (read flow data in L/min)");
        ESP_LOGI(TAG, "     - Battery Level characteristic UUID: 0x2A19 (read battery level)");
        ESP_LOGI(TAG, "     - Humidity characteristic UUID: 0x2A6F (read humidity level)");
        ESP_LOGI(TAG, "     - Temperature characteristic UUID: 0x2A6E (read temperature in Celsius)");
        ESP_LOGI(TAG, "     - OTA Control characteristic UUID: 0xFF02 (firmware update control)");
        ESP_LOGI(TAG, "     - OTA Data characteristic UUID: 0xFF03 (firmware update data)");
        ESP_LOGI(TAG, "  2. Device Information Service (0x180A) with %d characteristics:", 6);
        ESP_LOGI(TAG, "     - Manufacturer Name (0x2A29) - Read");
        ESP_LOGI(TAG, "     - Firmware Revision (0x2A26) - Read");
        ESP_LOGI(TAG, "     - Hardware Revision (0x2A27) - Read");
        ESP_LOGI(TAG, "     - Model Number (0x2A24) - Read");
        ESP_LOGI(TAG, "     - Software Revision (0x2A28) - Read");
        ESP_LOGI(TAG, "     - MAC Address (0xFF04) - Read");
    }
    
    ble_hs_cfg.sync_cb = ble_app_on_sync;      // Initialize application
    
    // Initialize battery ADC
    battery_adc_init();
    ESP_LOGI(TAG, "Battery ADC initialized");
    
    // Initialize OTA (Over-The-Air) update system
    ESP_LOGI(TAG, "Initializing OTA update system...");
    const esp_partition_t *current_partition = esp_ota_get_running_partition();
    const esp_partition_t *next_partition = esp_ota_get_next_update_partition(NULL);
    
    if (current_partition) {
        ESP_LOGI(TAG, "Current running partition: %s at offset 0x%lx, size: %lu bytes", 
                 current_partition->label, (unsigned long)current_partition->address,
                 (unsigned long)current_partition->size);
        ESP_LOGI(TAG, "Current partition type: %d, subtype: %d", 
                 current_partition->type, current_partition->subtype);
    } else {
        ESP_LOGE(TAG, "Failed to get current running partition!");
    }
    
    if (next_partition) {
        ESP_LOGI(TAG, "Next OTA partition: %s at offset 0x%lx, size: %lu bytes", 
                 next_partition->label, (unsigned long)next_partition->address,
                 (unsigned long)next_partition->size);
        ESP_LOGI(TAG, "OTA update system ready");
    } else {
        ESP_LOGW(TAG, "No OTA partition available - firmware updates disabled");
        ESP_LOGW(TAG, "To enable OTA updates, use 'idf.py menuconfig' and configure:");
        ESP_LOGW(TAG, "  1. Partition Table -> Partition table: Factory app, two OTA definitions");
        ESP_LOGW(TAG, "  2. Or create custom partition table with OTA partitions");
        
        // Check if we're running from factory partition
        if (current_partition && current_partition->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY) {
            ESP_LOGW(TAG, "Currently running from factory partition - this may limit OTA functionality");
            ESP_LOGW(TAG, "For full OTA support, consider flashing to an OTA partition");
        }
        
        // List all available partitions for debugging
        esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, NULL);
        if (it) {
            ESP_LOGI(TAG, "Available APP partitions:");
            while (it) {
                const esp_partition_t *part = esp_partition_get(it);
                ESP_LOGI(TAG, "  - %s: type=%d, subtype=%d, offset=0x%lx, size=%lu bytes", 
                         part->label, part->type, part->subtype,
                         (unsigned long)part->address, (unsigned long)part->size);
                it = esp_partition_next(it);
            }
            esp_partition_iterator_release(it);
        }
    }
    
    ESP_LOGI(TAG, "GRSVC1 Multi-Function Device initialized");
    ESP_LOGI(TAG, "Firmware Version: %s", FIRMWARE_VERSION);
    ESP_LOGI(TAG, "Manufacturer: %s", MANUFACTURER_NAME);
    ESP_LOGI(TAG, "Model: %s", MODEL_NUMBER);
    ESP_LOGI(TAG, "Hardware Revision: %s", HARDWARE_REVISION);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "BLE Services Available:");
    ESP_LOGI(TAG, "- Automation IO Service (UUID: 0x1815)");
    ESP_LOGI(TAG, "  * User Control Point characteristic UUID: 0xFF01 (write commands)");
    ESP_LOGI(TAG, "    Commands: VALVE 1/2 ON/OFF, 12V ON/OFF, RESET FLOW, STATUS, RESTART ADV, OTA INFO, BLE STATUS, TEST GPIO");
    ESP_LOGI(TAG, "  * Status characteristic UUID: 0x2B3C (read status)");
    ESP_LOGI(TAG, "  * Volume Flow characteristic UUID: 0x2B1B (read flow data in L/min)");
    ESP_LOGI(TAG, "  * Battery Level characteristic UUID: 0x2A19 (read battery level)");
    ESP_LOGI(TAG, "  * Humidity characteristic UUID: 0x2A6F (read humidity level)");
    ESP_LOGI(TAG, "  * Temperature characteristic UUID: 0x2A6E (read temperature in Celsius)");
    ESP_LOGI(TAG, "  * OTA Control characteristic UUID: 0xFF02 (firmware update control)");
    ESP_LOGI(TAG, "  * OTA Data characteristic UUID: 0xFF03 (firmware update data)");
    ESP_LOGI(TAG, "  * URI characteristic UUID: 0x2AB6 (device information link)");
    ESP_LOGI(TAG, "- Device Information Service (UUID: 0x180A)");
    ESP_LOGI(TAG, "  * Manufacturer Name, Model, Firmware Version, etc.");
    ESP_LOGI(TAG, "  * MAC Address characteristic UUID: 0xFF04 (device MAC address)");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "BLE Advertising Features:");
    ESP_LOGI(TAG, "- Device name: %s", DEVICE_NAME);
    ESP_LOGI(TAG, "- MAC address included in manufacturer data");
    ESP_LOGI(TAG, "- Connectable and discoverable mode");
    ESP_LOGI(TAG, "- Auto-restart advertising on disconnect");
    ESP_LOGI(TAG, "- Connection monitoring with periodic status checks");
    ESP_LOGI(TAG, "- Remote advertising restart via 'RESTART ADV' command");
    
    // Create connection monitoring task with larger stack
    xTaskCreate(connection_monitor_task, "conn_monitor", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Connection monitor task started");
    
    nimble_port_freertos_init(host_task);      // Run the thread
}
