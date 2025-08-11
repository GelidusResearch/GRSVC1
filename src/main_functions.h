// Header function prototypes for GRSVC1 Arduino/nRF52840
#include <NimBLEDevice.h>

// Valve control functions
void setupValves();
void setValve(uint8_t valveNum, bool open);
void toggleValve(uint8_t valveNum);
void setValveState(uint8_t valveNum, bool open);
void openValve(uint8_t valveNum);
void closeValve(uint8_t valveNum);
void stopAllValves();

// Battery monitoring
void setupBatteryMonitor();
float readBatteryVoltage();
uint8_t batteryPercent(float voltage);

// Environmental sensors
void setupDHT();
void readDHT(float &temp, float &hum);

// Button handling
void setupButtons();
void handleButtons();

// Flow sensor
void setupFlowSensor();
void flowISR();

// Status LED
void setupStatusLED();
void blinkStatusLED();

// USB detection
void setupUSBDetect();
bool isUSBConnected();

// BLE functions
void setupBLE();
void updateBLE(float temp, float hum, float battery, uint8_t battery_pct);

// OTA functions
void setupOTA();
void handleOTA();

// Power management
void enterSleepIfIdle();
