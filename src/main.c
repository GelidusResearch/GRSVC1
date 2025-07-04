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

#define DEVICE_NAME "GRSVC1"
#define FIRMWARE_VERSION "1.0.0"
#define MANUFACTURER_NAME "Gelidus Research Inc."
#define MODEL_NUMBER "GRSVC V1"
#define HARDWARE_REVISION "Rev 1.0"
#define SOFTWARE_REVISION "ESP-IDF v5.4.2"

char *TAG = "GRSVC1";
uint8_t ble_addr_type;

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

#define FLOW_SENSOR_GPIO GPIO_NUM_26  // Flow GPIO pin
#define VALVE_1_GPIO GPIO_NUM_33  // Valve 1 GPIO pin
#define VALVE_2_GPIO GPIO_NUM_27  // Valve 2 GPIO pin
#define POWER_12V_GPIO GPIO_NUM_14  // 12V output GPIO pin
#define BATTERY_VOLTAGE_GPIO GPIO_NUM_12  // Battery Voltage ADC pin (ADC2_CH5)

static volatile uint32_t pulse_count = 0;
static adc_oneshot_unit_handle_t adc2_handle;
static adc_cali_handle_t adc2_cali_chan5_handle;

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
    CMD_OTA_INFO
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
            ESP_LOGI(TAG, "VALVE 1 ON - Activating valve 1");
            gpio_set_level(VALVE_1_GPIO, 1);
            break;
        case CMD_VALVE_1_OFF:
            ESP_LOGI(TAG, "VALVE 1 OFF - Deactivating valve 1");
            gpio_set_level(VALVE_1_GPIO, 0);
            break;
        case CMD_VALVE_2_ON:
            ESP_LOGI(TAG, "VALVE 2 ON - Activating valve 2");
            gpio_set_level(VALVE_2_GPIO, 1);
            break;
        case CMD_VALVE_2_OFF:
            ESP_LOGI(TAG, "VALVE 2 OFF - Deactivating valve 2");
            gpio_set_level(VALVE_2_GPIO, 0);
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
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
    
    return os_mbuf_append(ctxt->om, info_string, strlen(info_string)) == 0 ?
           0 : BLE_ATT_ERR_INSUFFICIENT_RES;
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
        case BLE_GATT_ACCESS_OP_READ_CHR:
            // Simulate humidity reading (replace with actual sensor reading)
            uint16_t humidity = 4500; // 45.00% humidity (in hundredths of percent)
            if (os_mbuf_append(ctxt->om, &humidity, sizeof(humidity)) == 0) {
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
        case BLE_GATT_ACCESS_OP_READ_CHR:
            // Simulate temperature reading (replace with actual sensor reading)
            int16_t temperature = 2350; // 23.50°C (in hundredths of degrees Celsius)
            if (os_mbuf_append(ctxt->om, &temperature, sizeof(temperature)) == 0) {
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
                    
                    ESP_LOGI(TAG, "OTA: Ready to receive firmware data");
                    break;
                    
                case 0x02: // Finish OTA update
                    ESP_LOGI(TAG, "OTA: Finishing update");
                    
                    if (!ota_in_progress) {
                        ESP_LOGW(TAG, "OTA: No update in progress");
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
                .uuid = BLE_UUID16_DECLARE(0x2A9F),         // User Control Point (Control characteristic)
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
                .uuid = BLE_UUID16_DECLARE(0x2A6C),         // OTA Control characteristic
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
                .uuid = BLE_UUID16_DECLARE(0x2A6B),         // OTA Data characteristic
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
            // Connection successful
            conn_handle = event->connect.conn_handle;
            is_connected = true;
            is_advertising = false;
            ESP_LOGI("GAP", "Connection established, handle: %d", conn_handle);
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
        
        // Update connection state
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        is_connected = false;
        is_advertising = false;
        
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
        
        // Restart advertising after disconnect
        ble_app_advertise();
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
    // Don't start advertising if already advertising or connected
    if (is_advertising) {
        ESP_LOGW(TAG, "Already advertising, skipping...");
        return;
    }
    
    if (is_connected) {
        ESP_LOGW(TAG, "Already connected, skipping advertising...");
        return;
    }

    // Get the device's own MAC address
    uint8_t own_addr[6];
    int rc = ble_hs_id_copy_addr(ble_addr_type, own_addr, NULL);
    if (rc == 0) {
        ESP_LOGI(TAG, "Device MAC Address: %02X:%02X:%02X:%02X:%02X:%02X", 
                 own_addr[5], own_addr[4], own_addr[3], 
                 own_addr[2], own_addr[1], own_addr[0]);
    } else {
        ESP_LOGE(TAG, "Failed to get MAC address: %d", rc);
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
    }
    fields.mfg_data = mfg_data;
    fields.mfg_data_len = sizeof(mfg_data);

    // Set advertising fields
    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set advertising fields: %d", rc);
        return;
    }

    // GAP - device connectivity definition
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable or non-connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // discoverable or non-discoverable
    
    // Start advertising
    rc = ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    if (rc == 0) {
        is_advertising = true;
        ESP_LOGI(TAG, "BLE advertising started successfully");
    } else {
        ESP_LOGE(TAG, "Failed to start BLE advertising: %d", rc);
        is_advertising = false;
        
        // Note: Retry will be handled by the connection monitor task
        // to prevent stack overflow from recursive calls
    }
}

// Force advertising restart - useful for debugging connection issues
void force_advertising_restart(void)
{
    ESP_LOGI(TAG, "Forcing advertising restart...");
    
    // Stop current advertising if active
    if (is_advertising) {
        int rc = ble_gap_adv_stop();
        if (rc == 0) {
            ESP_LOGI(TAG, "Stopped current advertising");
        } else {
            ESP_LOGW(TAG, "Failed to stop advertising: %d", rc);
        }
        is_advertising = false;
    }
    
    // Reset connection state if needed
    if (!is_connected) {
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
    }
    
    // Small delay to ensure cleanup
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Start advertising again
    ble_app_advertise();
}

// The application
void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type); // Determines the best address type automatically
    ble_app_advertise();                     // Define the BLE connection
}

// Connection monitoring task - ensures advertising is always active when not connected
void connection_monitor_task(void *param)
{
    static uint32_t last_status_log = 0;
    static uint32_t last_adv_attempt = 0;
    static bool prev_connected = false;
    static bool prev_advertising = false;
    static uint8_t retry_count = 0;
    
    ESP_LOGI(TAG, "Connection monitor task started");
    
    while (1) {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Check if we should be advertising but aren't
        if (!is_connected && !is_advertising) {
            // Limit advertising attempts to once every 2 seconds to prevent spam
            if (current_time - last_adv_attempt > 2000) {
                ESP_LOGW(TAG, "Not connected and not advertising - starting advertising (retry %d)", retry_count);
                ble_app_advertise();
                last_adv_attempt = current_time;
                
                // Wait a bit and check if advertising started
                vTaskDelay(pdMS_TO_TICKS(500));
                if (!is_advertising) {
                    retry_count++;
                    if (retry_count > 10) {
                        ESP_LOGE(TAG, "Advertising failed after 10 retries, waiting longer...");
                        retry_count = 0;
                        vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds before trying again
                    }
                } else {
                    retry_count = 0; // Reset on success
                }
            }
        } else {
            retry_count = 0; // Reset when connected or advertising
        }
        
        // Log state changes (reduced logging to save stack)
        if (is_connected != prev_connected) {
            ESP_LOGI(TAG, "Connection: %s", is_connected ? "CONNECTED" : "DISCONNECTED");
            prev_connected = is_connected;
        }
        
        if (is_advertising != prev_advertising) {
            ESP_LOGI(TAG, "Advertising: %s", is_advertising ? "ACTIVE" : "INACTIVE");
            prev_advertising = is_advertising;
        }
        
        // Periodic status log (every 30 seconds) - simplified
        if (current_time - last_status_log > 30000) {
            ESP_LOGI(TAG, "BLE Status - Conn:%s Adv:%s Handle:%d", 
                     is_connected ? "Y" : "N", 
                     is_advertising ? "Y" : "N",
                     conn_handle);
            last_status_log = current_time;
        }
        
        // Check every 5 seconds
        vTaskDelay(pdMS_TO_TICKS(5000));
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
    
    // Initialize BLE stack
    // esp_nimble_hci_and_controller_init();   // Initialize ESP controller
    nimble_port_init();                        // Initialize the host stack
    ble_svc_gap_device_name_set(DEVICE_NAME);  // Initialize NimBLE configuration - server name
    ble_svc_gap_init();                        // Initialize NimBLE configuration - gap service
    ble_svc_gatt_init();                       // Initialize NimBLE configuration - gatt service
    ble_gatts_count_cfg(gatt_svcs);            // Initialize NimBLE configuration - config gatt services
    ble_gatts_add_svcs(gatt_svcs);             // Initialize NimBLE configuration - queues gatt services.
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
    ESP_LOGI(TAG, "  * User Control Point characteristic UUID: 0x2A9F (write commands)");
    ESP_LOGI(TAG, "    Commands: VALVE 1/2 ON/OFF, 12V ON/OFF, RESET FLOW, STATUS, RESTART ADV, OTA INFO");
    ESP_LOGI(TAG, "  * Status characteristic UUID: 0x2B3C (read status)");
    ESP_LOGI(TAG, "  * Volume Flow characteristic UUID: 0x2B1B (read flow data in L/min)");
    ESP_LOGI(TAG, "  * Battery Level characteristic UUID: 0x2A19 (read battery level)");
    ESP_LOGI(TAG, "  * Humidity characteristic UUID: 0x2A6F (read humidity level)");
    ESP_LOGI(TAG, "  * Temperature characteristic UUID: 0x2A6E (read temperature in Celsius)");
    ESP_LOGI(TAG, "  * OTA Control characteristic UUID: 0x2A6C (firmware update control)");
    ESP_LOGI(TAG, "  * OTA Data characteristic UUID: 0x2A6B (firmware update data)");
    ESP_LOGI(TAG, "  * URI characteristic UUID: 0x2AB6 (device information link)");
    ESP_LOGI(TAG, "- Device Information Service (UUID: 0x180A)");
    ESP_LOGI(TAG, "  * Manufacturer Name, Model, Firmware Version, etc.");
    ESP_LOGI(TAG, "  * URI characteristic UUID: 0x2AB6 (device information link)");
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
