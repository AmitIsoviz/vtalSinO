// Code generated by Arduino IoT Cloud, DO NOT EDIT.

#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>

const char DEVICE_LOGIN_NAME[]  = "1e7d8106-c37d-4323-9cc5-49aaae25dc61";

const char SSID[]               = "123";    // Network SSID (name)
const char PASS[]               = "316432822";    // Network password (use for WPA, or use as key for WEP)
const char DEVICE_KEY[]  = "WABKOEC66ALLISZDH6FO";    // Secret device password

void onFlagChange();

float dist;
CloudTemperatureSensor temperature;
bool flag;

void initProperties(){

  ArduinoCloud.setBoardId(DEVICE_LOGIN_NAME);
  ArduinoCloud.setSecretDeviceKey(DEVICE_KEY);
  ArduinoCloud.addProperty(dist, READ, 1 * SECONDS, NULL);
  ArduinoCloud.addProperty(temperature, READ, 1 * SECONDS, NULL);
  ArduinoCloud.addProperty(flag, READWRITE, ON_CHANGE, onFlagChange);

}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);
