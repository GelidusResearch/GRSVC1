// Modular function prototypes for GRSVC1 Arduino/nRF52840
#include <NimBLEDevice.h>

void setupValves();
void toggleValve1();
void toggleValve2();
void setupBatteryMonitor();
float readBatteryVoltage();
uint8_t batteryPercent(float voltage);
void setupDHT();
void readDHT(float &temp, float &hum);
void setupButtons();
void handleButtons();
void setupFlowSensor();
void flowISR();
void setupStatusLED();
void blinkStatusLED();
void setupUSBDetect();
bool isUSBConnected();
void setupBLE();
void updateBLE(float temp, float hum, float battery, uint8_t battery_pct);
void setupOTA();
void handleOTA();
void enterSleepIfIdle();

// BLE functions
void setupBLE();
void updateBLE(float temp, float hum, float battery, uint8_t battery_pct);

// OTA functions
void setupOTA();
void handleOTA();
