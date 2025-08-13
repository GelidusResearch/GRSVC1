
// Modularized with all major functions ported from ESP32 main.c
#include <Arduino.h>
#include <DHT.h>  // Adafruit DHT library
#include "main_functions.h"

// Function declarations
void updateStatusLED();

// Pin definitions
// #define BATTERY_ADC      4    // P0.04 (AIN2)
// #define STATUS_LED       2    // P0.02
// #define EN_12V           5    // P0.05
// #define DHT_PIN          6    // P0.06
// #define FLOW_PULSES      8    // P0.08
// #define PUSH_BUTTON_1   13    // P0.13
// #define PUSH_BUTTON_2   15    // P0.15
// #define PUSH_BUTTON_3   16    // P0.16
// #define USB_DETECT      17    // P0.17
// #define HB_1A_VALVE_1   21    // P0.21
// #define HB_1B_VALVE_1   22    // P0.22
// #define HB_2A_VALVE_2   23    // P0.23
// #define HB_2B_VALVE_2   24    // P0.24

// Pin definitions
#define BATTERY_ADC      4    // P0.04
#define STATUS_LED      15    // P0.15
#define EN_12V           6    // P0.06
#define DHT_PIN          8    // P0.08
#define FLOW_PULSES     24    // P0.17
#define PUSH_BUTTON_1   17    // P0.20
#define PUSH_BUTTON_2   20    // P0.22
#define PUSH_BUTTON_3   22    // P0.24
#define USB_DETECT      11    // P0.11
#define HB_1A_VALVE_1   104   // P1.04
#define HB_1B_VALVE_1   106   // P1.06
#define HB_2A_VALVE_2   109   // P1.09
#define HB_2B_VALVE_2   110   // P1.10

#define DHTTYPE DHT22
DHT dht(DHT_PIN, DHTTYPE);

// Valve state tracking
bool valve1State = false;
bool valve2State = false;

// Flow pulse counting
volatile uint32_t pulseCount = 0;

// Cached sensor readings
float lastTemp = NAN;
float lastHum = NAN;
uint32_t lastSensorRead = 0;
uint8_t lastBatteryPct = 0;
float lastBatteryVoltage = 0;

// Button debounce
unsigned long lastBtn1Press = 0;
unsigned long lastBtn2Press = 0;
unsigned long lastBtn3Press = 0;
const unsigned long debounceMs = 200;

// Status LED timing
unsigned long lastLEDUpdate = 0;
bool ledPulseState = false;
const unsigned long ledPulseInterval = 1000; // 1 second pulse when disconnected

// Debug output timing to reduce BLE interference
unsigned long lastDebugOutput = 0;
const unsigned long debugOutputInterval = 5000; // 5 seconds between debug prints

// BLE notification timing to prevent overwhelming the connection
unsigned long lastBLEUpdate = 0;
const unsigned long bleUpdateInterval = 1000; // 1 second between BLE notifications

// BLE restart flag
bool restartAdvertising = false;
unsigned long advertisingRestartTime = 0;

// NimBLE Server and Services
NimBLEServer* pServer = nullptr;
NimBLEService* batteryService = nullptr;
NimBLEService* environmentalService = nullptr;
NimBLEService* flowService = nullptr;
NimBLEService* valveService = nullptr;
NimBLEService* deviceService = nullptr;
NimBLEService* otaService = nullptr;

// NimBLE Characteristics
NimBLECharacteristic* batteryLevelChar = nullptr;
NimBLECharacteristic* temperatureChar = nullptr;
NimBLECharacteristic* humidityChar = nullptr;
NimBLECharacteristic* flowRateChar = nullptr;
NimBLECharacteristic* valve1StateChar = nullptr;
NimBLECharacteristic* valve2StateChar = nullptr;
NimBLECharacteristic* deviceNameChar = nullptr;
NimBLECharacteristic* firmwareVersionChar = nullptr;
NimBLECharacteristic* otaControlChar = nullptr;
NimBLECharacteristic* otaDataChar = nullptr;

// OTA variables
bool otaInProgress = false;
uint32_t otaExpectedSize = 0;
uint32_t otaReceivedSize = 0;
uint8_t* otaBuffer = nullptr;
uint32_t otaBufferSize = 0;

void flowISR() { pulseCount++; }

void setup() {
  setupValves();
  setupBatteryMonitor();
  setupDHT();
  setupButtons();
  setupFlowSensor();
  setupStatusLED();
  setupUSBDetect();
  setupBLE();
  setupOTA(); // Uncomment if using OTA
  Serial.begin(115200);
}

void loop() {
  // Sensor readings
  float temp, hum;
  readDHT(temp, hum);
  float batteryVoltage = readBatteryVoltage();
  uint8_t batteryPct = batteryPercent(batteryVoltage);

  // Button handling
  handleButtons();

  // Status LED
  updateStatusLED();

  // Print debug info (reduced frequency to avoid BLE interference)
  unsigned long now = millis();
  if (now - lastDebugOutput >= debugOutputInterval) {
    Serial.print("Battery: "); Serial.print(batteryVoltage); Serial.print(" V ("); Serial.print(batteryPct); Serial.println("%)");
    Serial.print("Temp: "); Serial.println(temp);
    Serial.print("Hum: "); Serial.println(hum);
    Serial.print("Flow pulses: "); Serial.println(pulseCount);
    Serial.print("USB: "); Serial.println(isUSBConnected() ? "Connected" : "Disconnected");
    lastDebugOutput = now;
  }

  // BLE connection status and ensure advertising
  if (pServer) {
    Serial.print("BLE Connected clients: ");
    Serial.println(pServer->getConnectedCount());
    
    // Handle delayed advertising restart
    if (restartAdvertising && millis() >= advertisingRestartTime) {
      Serial.println("Executing delayed advertising restart...");
      NimBLEDevice::getAdvertising()->stop();
      delay(200); // Longer delay for clean stop
      
      // Reconfigure advertising for restart with enhanced name advertising
      NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
      pAdvertising->setMinInterval(32);   // 20ms
      pAdvertising->setMaxInterval(160);  // 100ms
      pAdvertising->setName("GRSVC1");    // Ensure name is set on restart
      
      // Enhanced name advertising for restart
      NimBLEAdvertisementData advertisementData;
      advertisementData.setName("GRSVC1");
      advertisementData.setCompleteServices(BLEUUID("180F"));
      pAdvertising->setAdvertisementData(advertisementData);
      
      NimBLEAdvertisementData scanResponseData;
      scanResponseData.setName("GRSVC1");
      pAdvertising->setScanResponseData(scanResponseData);
      
      bool started = NimBLEDevice::startAdvertising();
      Serial.print("Advertising restart result: ");
      Serial.println(started ? "SUCCESS" : "FAILED");
      if (started) {
        Serial.println("Device name 'GRSVC1' re-advertised with enhanced data");
      }
      restartAdvertising = false;
    }
    
    // If no clients connected, ensure we're advertising
    if (pServer->getConnectedCount() == 0 && !restartAdvertising) {
      if (!NimBLEDevice::getAdvertising()->isAdvertising()) {
        Serial.println("No advertising detected - restarting immediately...");
        NimBLEDevice::startAdvertising();
      }
    }
  }

  // BLE/OTA updates
  updateBLE(temp, hum, batteryVoltage, batteryPct);
  handleOTA();

  // Power management
  enterSleepIfIdle();
  delay(500); // Reduced from 1000ms for better BLE responsiveness
}

// --- Function implementations ---
void setupValves() {
  pinMode(HB_1A_VALVE_1, OUTPUT);
  pinMode(HB_1B_VALVE_1, OUTPUT);
  pinMode(HB_2A_VALVE_2, OUTPUT);
  pinMode(HB_2B_VALVE_2, OUTPUT);
  pinMode(EN_12V, OUTPUT);
  
  // Initialize all outputs to LOW (0+0 = sleep/High-Z state)
  digitalWrite(HB_1A_VALVE_1, LOW);
  digitalWrite(HB_1B_VALVE_1, LOW);
  digitalWrite(HB_2A_VALVE_2, LOW);
  digitalWrite(HB_2B_VALVE_2, LOW);
  digitalWrite(EN_12V, LOW);
  
  Serial.println("Valve H-bridge control initialized - Sleep state (0+0)");
}
// 0 + 0: Coast/Sleep (High-Z state)
// 0 + 1: Open valve
// 1 + 0: Close valve
// 1 + 1: Brake (not used)

// Generic valve control function with valve number parameter
void setValve(uint8_t valveNum, bool open) {
  if (valveNum < 1 || valveNum > 2) {
    Serial.print("Invalid valve number: ");
    Serial.println(valveNum);
    return;
  }
  
  Serial.print("Setting Valve ");
  Serial.print(valveNum);
  Serial.print(" to: ");
  Serial.println(open ? "OPEN" : "CLOSE");
  
  // Determine pin assignments based on valve number
  uint8_t pinA = (valveNum == 1) ? HB_1A_VALVE_1 : HB_2A_VALVE_2;
  uint8_t pinB = (valveNum == 1) ? HB_1B_VALVE_1 : HB_2B_VALVE_2;
  
  digitalWrite(EN_12V, HIGH);  // Enable 12V supply
  delay(10);                   // Small delay for power stabilization
  
  if (open) {
    // Open valve (0 + 1)
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
  } else {
    // Close valve (1 + 0)
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
  }
  
  delay(50);  // 50ms pulse width for valve actuation
  
  // Sleep H-bridge (0 + 0) - Coast to High-Z state
  digitalWrite(pinA, LOW);
  digitalWrite(pinB, LOW);
  
  digitalWrite(EN_12V, LOW);   // Disable 12V supply
  
  // Update state tracking
  if (valveNum == 1) {
    valve1State = open;
    // Immediately notify BLE clients of valve state change
    if (pServer && pServer->getConnectedCount() > 0 && valve1StateChar) {
      uint8_t v1State = valve1State ? 1 : 0;
      valve1StateChar->setValue(&v1State, 1);
      valve1StateChar->notify();
    }
  } else {
    valve2State = open;
    // Immediately notify BLE clients of valve state change
    if (pServer && pServer->getConnectedCount() > 0 && valve2StateChar) {
      uint8_t v2State = valve2State ? 1 : 0;
      valve2StateChar->setValue(&v2State, 1);
      valve2StateChar->notify();
    }
  }
  
  Serial.print("Valve ");
  Serial.print(valveNum);
  Serial.println(" operation complete");
}

// Toggle function with valve number parameter
void toggleValve(uint8_t valveNum) {
  if (valveNum == 1) {
    setValve(1, !valve1State);
  } else if (valveNum == 2) {
    setValve(2, !valve2State);
  } else {
    Serial.print("Invalid valve number for toggle: ");
    Serial.println(valveNum);
  }
}

// Set valve state only if different
void setValveState(uint8_t valveNum, bool open) {
  bool currentState = (valveNum == 1) ? valve1State : valve2State;
  if (currentState != open) {
    setValve(valveNum, open);
  }
}

// Direct open/close functions with valve number parameter
void openValve(uint8_t valveNum) {
  setValve(valveNum, true);
}

void closeValve(uint8_t valveNum) {
  setValve(valveNum, false);
}

void stopAllValves() {
  // Emergency stop - set all H-bridge outputs to sleep state (0 + 0)
  digitalWrite(HB_1A_VALVE_1, LOW);
  digitalWrite(HB_1B_VALVE_1, LOW);
  digitalWrite(HB_2A_VALVE_2, LOW);
  digitalWrite(HB_2B_VALVE_2, LOW);
  digitalWrite(EN_12V, LOW);
  Serial.println("All valves set to sleep state (High-Z)");
}
void setupBatteryMonitor() {
  pinMode(BATTERY_ADC, INPUT);
}
float readBatteryVoltage() {
  int raw = analogRead(BATTERY_ADC);
  float v_adc = (raw / 1023.0) * 3.3;
  return v_adc * 2.341;
}
uint8_t batteryPercent(float voltage) {
  if (voltage >= 4.1) return 100;
  if (voltage <= 3.0) return 0;
  return (uint8_t)((voltage - 2.9) * 100 / 1.2);
}
void setupDHT() {
  dht.begin();
}
void readDHT(float &temp, float &hum) {
  temp = dht.readTemperature();
  hum = dht.readHumidity();
}
void setupButtons() {
  pinMode(PUSH_BUTTON_1, INPUT_PULLUP);
  pinMode(PUSH_BUTTON_2, INPUT_PULLUP);
  pinMode(PUSH_BUTTON_3, INPUT_PULLUP);
}
void handleButtons() {
  unsigned long now = millis();
  if (!digitalRead(PUSH_BUTTON_1) && now - lastBtn1Press > debounceMs) {
    toggleValve(1);
    lastBtn1Press = now;
  }
  if (!digitalRead(PUSH_BUTTON_2) && now - lastBtn2Press > debounceMs) {
    toggleValve(2);
    lastBtn2Press = now;
  }
  // Add PUSH_BUTTON_3 logic as needed
}
void setupFlowSensor() {
  pinMode(FLOW_PULSES, INPUT);
  attachInterrupt(digitalPinToInterrupt(FLOW_PULSES), flowISR, RISING);
}
void setupStatusLED() {
  pinMode(STATUS_LED, OUTPUT);
}
void updateStatusLED() {
  // Check if we have BLE clients connected
  bool hasConnectedClients = (pServer && pServer->getConnectedCount() > 0);
  
  if (hasConnectedClients) {
    // Steady 10% PWM when connected
    analogWrite(STATUS_LED, 25); // 10% of 255 = 25
  } else {
    // Pulse once per second when disconnected (10% on, 90% off)
    unsigned long now = millis();
    if (now - lastLEDUpdate >= ledPulseInterval) {
      ledPulseState = !ledPulseState;
      if (ledPulseState) {
        analogWrite(STATUS_LED, 25); // 10% PWM for brief pulse
      } else {
        analogWrite(STATUS_LED, 0);  // Off
      }
      lastLEDUpdate = now;
    }
  }
}
void setupUSBDetect() {
  pinMode(USB_DETECT, INPUT);
}
bool isUSBConnected() {
  return digitalRead(USB_DETECT);
}
void enterSleepIfIdle() {
  // Implement sleep logic if needed
}

// BLE Implementation with NimBLE
class ServerCallbacks: public NimBLEServerCallbacks {
public:
    void onConnect(NimBLEServer* pServer) {
        Serial.println("Client connected");
        Serial.print("Connected count: ");
        Serial.println(pServer->getConnectedCount());
        Serial.println("Connection established - service discovery should begin");
    }

    void onDisconnect(NimBLEServer* pServer) {
        Serial.println("Client disconnected - Scheduling advertising restart...");
        restartAdvertising = true;
        advertisingRestartTime = millis() + 500; // restart after 500ms delay
    }
    
    void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) {
        Serial.print("MTU updated: ");
        Serial.println(MTU);
    }
};

class ValveCallbacks: public NimBLECharacteristicCallbacks {
public:
    void onWrite(NimBLECharacteristic* pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            uint8_t value = rxValue[0];
            
            if (pCharacteristic->getUUID().equals(NimBLEUUID("12345678-1234-1234-1234-123456789ABF"))) {
                // Valve 1
                Serial.print("Valve 1 BLE command: ");
                Serial.println(value);
                bool openState = (value == 1);
                setValve(1, openState);
                Serial.print("Valve 1 set to ");
                Serial.println(openState ? "OPEN" : "CLOSE");
            } else if (pCharacteristic->getUUID().equals(NimBLEUUID("12345678-1234-1234-1234-123456789AC0"))) {
                // Valve 2
                Serial.print("Valve 2 BLE command: ");
                Serial.println(value);
                bool openState = (value == 1);
                setValve(2, openState);
                Serial.print("Valve 2 set to ");
                Serial.println(openState ? "OPEN" : "CLOSE");
            }
        }
    }
};

class OTACallbacks: public NimBLECharacteristicCallbacks {
public:
    void onWrite(NimBLECharacteristic* pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        
        if (pCharacteristic->getUUID().equals(NimBLEUUID("12345678-1234-1234-1234-123456789AC1"))) {
            // OTA Control characteristic
            if (rxValue.length() >= 5) {
                uint8_t command = rxValue[0];
                
                if (command == 0x01) { // Start OTA
                    // Extract expected size (little endian)
                    otaExpectedSize = (uint32_t)rxValue[1] | 
                                    ((uint32_t)rxValue[2] << 8) | 
                                    ((uint32_t)rxValue[3] << 16) | 
                                    ((uint32_t)rxValue[4] << 24);
                    
                    Serial.print("OTA Start - Expected size: ");
                    Serial.println(otaExpectedSize);
                    
                    // Allocate buffer for firmware
                    if (otaBuffer != nullptr) {
                        free(otaBuffer);
                    }
                    otaBuffer = (uint8_t*)malloc(otaExpectedSize);
                    
                    if (otaBuffer == nullptr) {
                        Serial.println("OTA Error: Failed to allocate buffer");
                        otaInProgress = false;
                        return;
                    }
                    
                    otaReceivedSize = 0;
                    otaInProgress = true;
                    Serial.println("OTA started successfully");
                    
                } else if (command == 0x02) { // Finish OTA
                    Serial.println("OTA Finish command received");
                    
                    if (otaInProgress && otaReceivedSize == otaExpectedSize) {
                        Serial.println("OTA completed successfully - preparing to restart");
                        
                        // Here you would typically write the firmware to flash
                        // For nRF52840, this would involve using the DFU bootloader
                        Serial.println("Firmware update would be applied here");
                        
                        // Cleanup
                        if (otaBuffer != nullptr) {
                            free(otaBuffer);
                            otaBuffer = nullptr;
                        }
                        
                        otaInProgress = false;
                        
                        // Restart device after a delay
                        delay(1000);
                        Serial.println("Restarting device...");
                        NVIC_SystemReset();
                        
                    } else {
                        Serial.print("OTA Error: Size mismatch. Expected: ");
                        Serial.print(otaExpectedSize);
                        Serial.print(", Received: ");
                        Serial.println(otaReceivedSize);
                        otaInProgress = false;
                        
                        if (otaBuffer != nullptr) {
                            free(otaBuffer);
                            otaBuffer = nullptr;
                        }
                    }
                }
            }
            
        } else if (pCharacteristic->getUUID().equals(NimBLEUUID("12345678-1234-1234-1234-123456789AC2"))) {
            // OTA Data characteristic
            if (otaInProgress && rxValue.length() > 0) {
                uint32_t dataSize = rxValue.length();
                
                if (otaReceivedSize + dataSize <= otaExpectedSize) {
                    // Copy data to buffer
                    memcpy(otaBuffer + otaReceivedSize, rxValue.data(), dataSize);
                    otaReceivedSize += dataSize;
                    
                    // Progress reporting every 1KB
                    if (otaReceivedSize % 1024 == 0 || otaReceivedSize == otaExpectedSize) {
                        Serial.print("OTA Progress: ");
                        Serial.print(otaReceivedSize);
                        Serial.print("/");
                        Serial.print(otaExpectedSize);
                        Serial.print(" bytes (");
                        Serial.print((otaReceivedSize * 100) / otaExpectedSize);
                        Serial.println("%)");
                    }
                } else {
                    Serial.println("OTA Error: Data overflow");
                    otaInProgress = false;
                    
                    if (otaBuffer != nullptr) {
                        free(otaBuffer);
                        otaBuffer = nullptr;
                    }
                }
            }
        }
    }
};

void setupBLE() {
    NimBLEDevice::init("GRSVC1");
    
    // Set BLE power level to maximum for better range (nRF52840 supports up to +8dBm)
    NimBLEDevice::setPower(8); // +8dBm max for nRF52840
    
    // Create BLE Server
    pServer = NimBLEDevice::createServer();
    
    // Set server callbacks for connection handling
    pServer->setCallbacks(new ServerCallbacks());
    
    // Create Battery Service
    batteryService = pServer->createService("180F");
    batteryLevelChar = batteryService->createCharacteristic(
        "2A19", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    
    // Create Environmental Service
    environmentalService = pServer->createService("181A");
    temperatureChar = environmentalService->createCharacteristic(
        "2A6E", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    humidityChar = environmentalService->createCharacteristic(
        "2A6F", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    
    // Create Flow Service
    flowService = pServer->createService("12345678-1234-1234-1234-123456789ABC");
    flowRateChar = flowService->createCharacteristic(
        "12345678-1234-1234-1234-123456789ABD", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    
    // Create Valve Service
    valveService = pServer->createService("12345678-1234-1234-1234-123456789ABE");
    valve1StateChar = valveService->createCharacteristic(
        "12345678-1234-1234-1234-123456789ABF", 
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
    );
    valve2StateChar = valveService->createCharacteristic(
        "12345678-1234-1234-1234-123456789AC0", 
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
    );
    
    // Create Device Information Service
    deviceService = pServer->createService("180A");
    deviceNameChar = deviceService->createCharacteristic("2A00", NIMBLE_PROPERTY::READ);
    firmwareVersionChar = deviceService->createCharacteristic("2A26", NIMBLE_PROPERTY::READ);
    
    // Create OTA Service
    otaService = pServer->createService("12345678-1234-1234-1234-123456789AC3");
    otaControlChar = otaService->createCharacteristic(
        "12345678-1234-1234-1234-123456789AC1", 
        NIMBLE_PROPERTY::WRITE
    );
    otaDataChar = otaService->createCharacteristic(
        "12345678-1234-1234-1234-123456789AC2", 
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    
    // Set initial values
    deviceNameChar->setValue("GRSVC1");
    firmwareVersionChar->setValue("1.0.0");
    
    uint8_t v1State = valve1State ? 1 : 0;
    uint8_t v2State = valve2State ? 1 : 0;
    valve1StateChar->setValue(&v1State, 1);
    valve2StateChar->setValue(&v2State, 1);
    
    // Set callbacks for valve characteristics
    ValveCallbacks* pCallbacks = new ValveCallbacks();
    valve1StateChar->setCallbacks(pCallbacks);
    valve2StateChar->setCallbacks(pCallbacks);
    
    // Set callbacks for OTA characteristics
    OTACallbacks* pOTACallbacks = new OTACallbacks();
    otaControlChar->setCallbacks(pOTACallbacks);
    otaDataChar->setCallbacks(pOTACallbacks);
    
    // Start services
    batteryService->start();
    environmentalService->start();
    flowService->start();
    valveService->start();
    deviceService->start();
    otaService->start();
    
    // Start advertising with device name
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    
    // Add services to advertising
    pAdvertising->addServiceUUID("180F"); // Battery Service
    pAdvertising->addServiceUUID("181A"); // Environmental Service
    pAdvertising->addServiceUUID("12345678-1234-1234-1234-123456789ABE"); // Valve Service
    pAdvertising->addServiceUUID("12345678-1234-1234-1234-123456789AC3"); // OTA Service
    
    // Set advertising intervals for better connection stability
    pAdvertising->setMinInterval(160);  // 100ms - slower for stability
    pAdvertising->setMaxInterval(240);  // 150ms - slower for stability
    
    // Enhanced name advertising for Web Bluetooth compatibility
    pAdvertising->setName("GRSVC1");
    
    // Force name to appear in advertising data (multiple methods)
    NimBLEAdvertisementData advertisementData;
    advertisementData.setName("GRSVC1");
    advertisementData.setCompleteServices(BLEUUID("180F")); // Battery service
    advertisementData.setAppearance(0x0000); // Generic appearance
    
    // Set both advertising and scan response data
    pAdvertising->setAdvertisementData(advertisementData);
    
    NimBLEAdvertisementData scanResponseData;
    scanResponseData.setName("GRSVC1"); // Name in scan response too
    pAdvertising->setScanResponseData(scanResponseData);
    
    // Make sure advertising starts
    bool advertisingStarted = NimBLEDevice::startAdvertising();
    Serial.print("BLE device active, advertising started: ");
    Serial.println(advertisingStarted ? "SUCCESS" : "FAILED");
    if (advertisingStarted) {
        Serial.println("Waiting for connections...");
        Serial.println("Device name 'GRSVC1' should now appear in Web Bluetooth device picker");
        Serial.println("Enhanced advertising: Name in both advertisement and scan response data");
    }
}

void updateBLE(float temp, float hum, float battery, uint8_t battery_pct) {
    if (!pServer) return;
    
    // Limit notification frequency to prevent overwhelming the connection
    unsigned long now = millis();
    bool shouldNotify = (now - lastBLEUpdate >= bleUpdateInterval);
    
    if (shouldNotify) {
        lastBLEUpdate = now;
    }
    
    // Update battery level
    batteryLevelChar->setValue(battery_pct);
    if (pServer->getConnectedCount() > 0 && shouldNotify) {
        batteryLevelChar->notify();
    }
    
    // Update temperature (in 0.01°C units)
    int16_t tempValue = (int16_t)(temp * 100);
    temperatureChar->setValue((uint8_t*)&tempValue, 2);
    if (pServer->getConnectedCount() > 0 && shouldNotify) {
        temperatureChar->notify();
    }
    
    // Update humidity (in 0.01% units)
    uint16_t humValue = (uint16_t)(hum * 100);
    humidityChar->setValue((uint8_t*)&humValue, 2);
    if (pServer->getConnectedCount() > 0 && shouldNotify) {
        humidityChar->notify();
    }
    
    // Update flow rate
    uint32_t currentPulseCount = pulseCount;
    flowRateChar->setValue((uint8_t*)&currentPulseCount, 4);
    if (pServer->getConnectedCount() > 0 && shouldNotify) {
        flowRateChar->notify();
    }
    
    // Update valve states (always update, but notify less frequently)
    uint8_t v1State = valve1State ? 1 : 0;
    uint8_t v2State = valve2State ? 1 : 0;
    valve1StateChar->setValue(&v1State, 1);
    valve2StateChar->setValue(&v2State, 1);
    if (pServer->getConnectedCount() > 0 && shouldNotify) {
        valve1StateChar->notify();
        valve2StateChar->notify();
    }
}

void valve1CharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
    // This function is no longer needed with NimBLE - handled in ValveCallbacks
}

void valve2CharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
    // This function is no longer needed with NimBLE - handled in ValveCallbacks
}

void setupOTA() {
    Serial.println("OTA support initialized");
    Serial.println("OTA Service UUID: 12345678-1234-1234-1234-123456789AC3");
    Serial.println("OTA Control UUID: 12345678-1234-1234-1234-123456789AC1");
    Serial.println("OTA Data UUID: 12345678-1234-1234-1234-123456789AC2");
}

void handleOTA() {
    // OTA handling is done through BLE callbacks
    // This function can be used for additional OTA maintenance if needed
    
    // Check for memory leaks or cleanup if needed
    if (!otaInProgress && otaBuffer != nullptr) {
        free(otaBuffer);
        otaBuffer = nullptr;
        Serial.println("Cleaned up abandoned OTA buffer");
    }
}
