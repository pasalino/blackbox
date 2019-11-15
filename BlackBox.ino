#include <AWS_IOT.h>
#include <WiFi.h>
#include "TinyGPS++.h"
#include "SoftwareSerial.h"
#include <ArduinoJson.h>
#include <DHT.h>
#include <NTPClient.h>


#define SENSOR_NAME "box1"

// If is in debug mode
#define DEBUG 1
// If want print in serial the informations about sensors
#define PRINT_SERIAL 1
// Bypass GPS
#define BYPASS_GPS 1
#define DEFAULT_LAT 45.5252600
#define DEFAULT_LNG 45.5252600
#define DEFAULT_ALT 200


// ESP and Wifi Information
#define WIFI_SSID "SSDI"
#define WIFI_PASSWORD "PASSWORD"

// AWS IoT
#define HOST_ADDRESS "HOST AWS"
#define CLIENT_ID "box1"
#define TOPIC_NAME "$aws/things/box1/shadow/update"

#define SEND_INTERVAL 5000


// Temperature and humidity Sensor
#define PIN_DHT 27
#define DHT_TYPE DHT22

#define DHT_INTERVAL 1000


// Microphone Sensor
#define PIN_MICRO 14
#define MICRO_ELEMENTS 50

#define MICRO_INTERVAL 100


// GPS
#define GPS_UART_RX_PIN 22
#define GPS_UART_TX_PIN 23
#define GPS_BOUD_RATE 4800
#define GPS_INTERVAL 1000

// ##################### MACRO #########################


#if DEBUG==1
#define P_DEBUG(msg) Serial.print("#DEBUG# "); Serial.println(msg);
#else
#define P_DEBUG(msg)
#endif


#define BLOCK_INTERVAL(INTERVAL) \
  static unsigned long last = millis(); \
  unsigned long tick = millis(); \
  if(millis() - last <= INTERVAL){ \
    return; \
  } \
  last = tick;


// ##################### END MACRO #########################


DHT dht(PIN_DHT, DHT_TYPE);

SoftwareSerial gps_serial;
TinyGPSPlus gps;



AWS_IOT hornbill;
int status = WL_IDLE_STATUS;
int tick = 0, msgCount = 0, msgReceived = 0;
char payload[512];
char rcvdPayload[512];

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);



// Last sensors values
float humidity;    // Stores humidity value
float temperature; // Stores temperature value
bool valueMicrophone; // Stores microphone sensor 
bool gpsIsValid; // Check if GPS signal is valid
float latitude, longitude, altitude; // Stores gps information
int gpsAge; // Last valid value in GPS
unsigned long timeStamp;


void setup() {
  Serial.begin(112500);
  delay(1000);

  // Init devices
  Serial.println("ProjectWork IoT for Digital Innovation");
  Serial.println("Student: Pasqualino de Simone");

  Serial.println("Starting...");
  // Setup DHT
  dht.begin();
  // Setup Micro
  pinMode(PIN_MICRO, INPUT);

  //Setup GPS
  gps_serial.begin(9600, 22, 23, SWSERIAL_8N1, false, 256);

  //Setup WIFI
  while (status != WL_CONNECTED)
  {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(WIFI_SSID);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // wait 5 seconds for connection:
    delay(5000);
  }
  Serial.println("Connected to wifi");

  timeClient.begin();

  //Setup AWS IOT
  if (hornbill.connect(HOST_ADDRESS, CLIENT_ID) == 0)
  {
    Serial.println("Connected to AWS");
    delay(1000);
  }
  else
  {
    Serial.println("AWS connection failed, Check the HOST Address");
    while (1);
  }

  Serial.println("Setup completed!");
}

void loop() {
  timeClient.update();
  
  timeStamp = timeClient.getEpochTime();
  delay(1000);

  //Update Sensors
  updateGPS();

  if (gpsIsValid) {
    updateDHT();
    updateMicrophone();
    sendInformation();
  }


}

// ###### SEND INFORMATION #####
void sendInformation() {
  BLOCK_INTERVAL(SEND_INTERVAL);
  StaticJsonDocument<512> jsonDoc;
  JsonObject stateObj = jsonDoc.createNestedObject("state");
  JsonObject reportedObj = stateObj.createNestedObject("reported");
  JsonObject sensors = reportedObj.createNestedObject("sensors");
  JsonObject geolocation = reportedObj.createNestedObject("geolocation");
  JsonObject state = reportedObj.createNestedObject("state");

  // Write the temperature & humidity. Here you can use any C++ type (and you can refer to variables)
  reportedObj["timeStamp"] = timeStamp;
  reportedObj["name"] = SENSOR_NAME;
  
  sensors["temperature"] = temperature;
  sensors["humidity"] = humidity;
  sensors["valueMicrophone"] = valueMicrophone;
  sensors["gasAnalogValue"] = 513;
  
  state["wifiStrength"] = WiFi.RSSI();

  geolocation["latitude"] = latitude;
  geolocation["longitude"] = longitude;
  geolocation["altitude"] = altitude;
  geolocation["gpsAge"] = gpsAge;

  String jsonBuffer;
  serializeJson(jsonDoc, jsonBuffer);
  if (hornbill.publish(TOPIC_NAME, const_cast<char*>(jsonBuffer.c_str())) == 0)
  {
#if (PRINT_SERIAL == 1)
    Serial.println("SEND INFORMATION");
    serializeJson(jsonDoc, Serial);
    Serial.println("");
#endif
  }
  else
  {
    Serial.println("Publish failed");
  }
}

// ###### UPDATE GPS ######
void updateGPS() {
  BLOCK_INTERVAL(GPS_INTERVAL);

#if BYPASS_GPS==1
  gpsIsValid = true;
  latitude = DEFAULT_LAT;
  longitude = DEFAULT_LNG;
  altitude = DEFAULT_ALT;
  return;
#endif

  int satellites = gps.satellites.value();
  gpsIsValid = gps.location.isValid();
  gpsIsValid &= gps.altitude.isValid();
  gpsAge = gps.location.age();
  if (gpsIsValid) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    altitude = gps.altitude.meters();
  }
#if (PRINT_SERIAL == 1)
  if (gpsIsValid) {
    Serial.print("GPS - lat: ");
    Serial.print(latitude);
    Serial.print(" lng: ");
    Serial.print(longitude);
    Serial.print( " alt: ");
    Serial.print(altitude);
    Serial.print( " age: ");
    Serial.print( gpsAge);
    Serial.print( " satellites: ");
    Serial.print( satellites);
    Serial.println("");
  } else {
    Serial.print("GPS - NOT VALID age:");
    Serial.print(gpsAge);
    Serial.print( " satellites: ");
    Serial.print(satellites);
    Serial.println("");
  }
#endif
}


// ###### SENSORS ######
void updateDHT() {
  BLOCK_INTERVAL(DHT_INTERVAL);

  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

#if (PRINT_SERIAL == 1)
  String s_temp = String(temperature, 1);
  String s_humi = String(humidity, 1);
  Serial.println("DHT22 - Humidity:" + s_humi + "%, Temperature: " + s_temp + "°C");
#endif

}


void updateMicrophone() {
  BLOCK_INTERVAL(MICRO_INTERVAL);

  static int buffer[MICRO_ELEMENTS];
  static int index = 0;

  bool value = digitalRead(PIN_MICRO);
  buffer[index] = value ? 1 : 0;

  index++;

  if (index != MICRO_ELEMENTS) {
    return;
  }

  index = 0;

  int sum = 0;
  for (int i = 0; i < MICRO_ELEMENTS; i++) {
    sum += buffer[i];
  }

  float avg = sum / MICRO_ELEMENTS;
  valueMicrophone = avg > 0.5;

#if (PRINT_SERIAL == 1)
  Serial.print("MICROPHONE - Value Value: ");
  Serial.println(valueMicrophone ? "HIGH" : "LOW");
#endif
}
