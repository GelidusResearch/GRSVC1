H// void setupValves();

// Modern valve control functions with valve number parameter
void setValve(uint8_t valveNum, bool open);
void toggleValve(uint8_t valveNum);
void setValveState(uint8_t valveNum, bool open);
void openValve(uint8_t valveNum);
void closeValve(uint8_t valveNum);

void stopAllValves();r function prototypes for GRSVC1 Arduino/nRF52840
#include <NimBLEDevice.h>

void setupValves();
void toggleValve1();
void toggleValve2();
void setValve1State(bool open);
void setValve2State(bool open);
void openValve1();
void closeValve1();
void openValve2();
void closeValve2();
void stopAllValves();
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
