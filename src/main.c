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

#define FLOW_SENSOR_GPIO GPIO_NUM_26  // Flow GPIO pin
#define VALVE_1_GPIO GPIO_NUM_33  // Valve 1 GPIO pin
#define VALVE_2_GPIO GPIO_NUM_27  // Valve 2 GPIO pin
#define POWER_12V_GPIO GPIO_NUM_14  // 12V output GPIO pin
#define BATTERY_VOLTAGE_GPIO GPIO_NUM_12  // Battery Voltage ADC pin (ADC2_CH5)

static volatile uint32_t pulse_count = 0;
static adc_oneshot_unit_handle_t adc2_handle;
static adc_cali_handle_t adc2_cali_chan5_handle;

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
    CMD_STATUS
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
    char status_msg[128];
    snprintf(status_msg, sizeof(status_msg), 
             "GRSVC1 Status: Flow=%lu pulses, Uptime=%lu ms", 
             (unsigned long)pulse_count, 
             (unsigned long)(esp_timer_get_time() / 1000));
    
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
        if (event->connect.status != 0)
        {
            ble_app_advertise();
        }
        break;
    // Advertise again after completion of the event
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT DISCONNECTED");
        ble_app_advertise(); // Restart advertising after disconnect
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI("GAP", "BLE GAP EVENT");
        ble_app_advertise();
        break;
    default:
        break;
    }
    return 0;
}

// Define the BLE connection
void ble_app_advertise(void)
{
    // GAP - device name definition
    struct ble_hs_adv_fields fields;
    const char *device_name;
    memset(&fields, 0, sizeof(fields));
    device_name = ble_svc_gap_device_name(); // Read the BLE device name
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    // GAP - device connectivity definition
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable or non-connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // discoverable or non-discoverable
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

// The application
void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type); // Determines the best address type automatically
    ble_app_advertise();                     // Define the BLE connection
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
    
    ESP_LOGI(TAG, "GRSVC1 Multi-Function Device initialized");
    ESP_LOGI(TAG, "Firmware Version: %s", FIRMWARE_VERSION);
    ESP_LOGI(TAG, "Manufacturer: %s", MANUFACTURER_NAME);
    ESP_LOGI(TAG, "Model: %s", MODEL_NUMBER);
    ESP_LOGI(TAG, "Hardware Revision: %s", HARDWARE_REVISION);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "BLE Services Available:");
    ESP_LOGI(TAG, "- Automation IO Service (UUID: 0x1815)");
    ESP_LOGI(TAG, "  * Digital Output characteristic UUID: 0x2A56 (write commands)");
    ESP_LOGI(TAG, "  * Analog Output characteristic UUID: 0x2A58 (read status)");
    ESP_LOGI(TAG, "  * Volume Flow characteristic UUID: 0x5EE2 (read flow data in L/min)");
    ESP_LOGI(TAG, "  * Battery Level characteristic UUID: 0x2A19 (read battery level)");
    ESP_LOGI(TAG, "  * Humidity characteristic UUID: 0x2A6F (read humidity level)");
    ESP_LOGI(TAG, "  * Temperature characteristic UUID: 0x2A6E (read temperature in Celsius)");
    ESP_LOGI(TAG, "  * URI characteristic UUID: 0x2AB6 (device information link)");
    ESP_LOGI(TAG, "- Device Information Service (UUID: 0x180A)");
    ESP_LOGI(TAG, "  * Manufacturer Name, Model, Firmware Version, etc.");
    ESP_LOGI(TAG, "  * URI characteristic UUID: 0x2AB6 (device information link)");
    
    nimble_port_freertos_init(host_task);      // Run the thread
}
