/* 
 * ESP32 NodeMCU DHT11 - Humidity Temperature Sensor Example
 * https://circuits4you.com
 * 
 * References
 * https://circuits4you.com/2017/12/31/nodemcu-pinout/
 * 
 */
//PushButton
const int PushButton = 27;

//Temp&Hum
#include "DHTesp.h"

#include "thingProperties.h" //addition
#include <TinyGPS++.h>
//#include <SoftwareSerial.h>
#include <math.h>
#define pi 3.14159265358979323846 //addition

#define DHTpin 12    //D15 of ESP32 DevKit
DHTesp dht;


static const int RXPin = 16, TXPin = 17;// Here we make pin 16 as RX of arduino & pin 17 as TX of arduino 
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;

double deg2rad(double);
double rad2deg(double);

double distance(double lat1, double lon1, double lat2, double lon2, char unit) {//addition
  double theta, dist;
  if ((lat1 == lat2) && (lon1 == lon2)) {
    return 0;
  }
  else {
    theta = lon1 - lon2;
    dist = sin(deg2rad(lat1)) * sin(deg2rad(lat2)) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
    dist = acos(dist);
    dist = rad2deg(dist);
    dist = dist * 60 * 1.1515;
    switch(unit) {
      case 'M':
        break;
      case 'K':
        dist = dist * 1.609344;
        break;
      case 'N':
        dist = dist * 0.8684;
        break;
    }
    return (dist);
  }
}

double deg2rad(double deg) {
  return (deg * pi / 180);
}

double rad2deg(double rad) {
  return (rad * 180 / pi);
}//addition


void setup()
{
  //PushButton
  pinMode(PushButton, INPUT);

  //Temp&Hum
  Serial.begin(115200);
  Serial.println();
  Serial.println("Status\tHumidity (%)\tTemperature (C)\t(F)\tHeatIndex (C)\t(F)");
  
  // Autodetect is not working reliable, don't use the following line
  // dht.setup(17);
  // use this instead: 
  dht.setup(DHTpin, DHTesp::DHT11); //for DHT11 Connect DHT sensor to GPIO 17
  //dht.setup(DHTpin, DHTesp::DHT22); //for DHT22 Connect DHT sensor to GPIO 17


  //addional
  // Initialize serial and wait for port to open:
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  delay(1500); 
WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
  // Defined in thingProperties.h
  initProperties();

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  
  /*
     The following function allows you to obtain more information
     related to the state of network and IoT Cloud connection and errors
     the higher number the more granular information you’ll get.
     The default is 0 (only errors).
     Maximum is 4
 */
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
}
//defining base coordinates to compare to
double lat = 0;
double lng = 0;
double latBase = 32.101743;
double lngBase = 34.860912;
double borderDistance = 0.04;//default as 100m, recommended to be 20m radius
double dst = 0;
void displayInfo()
{ 
  if (gps.location.isValid())
  {
    Serial.print(F("Location: "));
    //double latBase = 32.101743;
    //double lngBase = 34.860912;
    lat = gps.location.lat();
    lng = gps.location.lng();
    Serial.print(lat, 6);
    Serial.print(F(","));
    Serial.print(lng, 6);
    Serial.println();
    dst = distance(lat,lng,latBase,lngBase,'K');
    if (dst > borderDistance){
      flag = true;
      Serial.print(F("ALERT - SOLDIER HAS BEEN DISCONNECTED FROM THE GROUP"));
      Serial.println();
      Serial.print(F("Current distance from squad: "));
      Serial.print(dst,6);
    }
    else{
        flag=false;
    }
  }
  else
  {
    Serial.print(F("INVALID GPS"));
    }
    delay(1000);
    Serial.println();
  //addition


}

void loop()
{  
  delay(1000);
  //PushButton
  int Push_button_state = digitalRead(PushButton);
  if ( Push_button_state == HIGH )
  { 
  Serial.println("HELP !!");
  }

  //Temp&Hum
  //delay(dht.getMinimumSamplingPeriod());
  float humidity = dht.getHumidity();
  float temperature = dht.getTemperature();
  //Serial.print(dht.getStatusString());
  dht.getStatusString();
  //Serial.print("\t\t");
  Serial.print("humidity:");
  Serial.print("\t");
  Serial.print(humidity, 1);
  Serial.print("\t\t");
  Serial.print("Temperature:");
  Serial.print("\t");
  Serial.println(temperature, 1);

  delay(25);
  //Serial.print("\t\t");
  //Serial.print(dht.toFahrenheit(temperature), 1);
  //Serial.print("\t\t");
  //Serial.print(dht.computeHeatIndex(temperature, humidity, false), 1);
  //Serial.print("\t\t");
  //Serial.println(dht.computeHeatIndex(dht.toFahrenheit(temperature), humidity, true), 1);


  //addition
  ArduinoCloud.update();
  // Your code here 
    while (Serial2.available() > 0)
    if (gps.encode(Serial2.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
  //addition
}

void onFlagChange(){} //addition
/*
  Since Flag is READ_WRITE variable, onFlagChange() is
  executed every time a new value is received from IoT Cloud.
*/

/*
  Since Flag is READ_WRITE variable, onFlagChange() is
  executed every time a new value is received from IoT Cloud.
*/

/*
  Since Flag is READ_WRITE variable, onFlagChange() is
  executed every time a new value is received from IoT Cloud.
*/

/*
  Since Flag is READ_WRITE variable, onFlagChange() is
  executed every time a new value is received from IoT Cloud.
*/
