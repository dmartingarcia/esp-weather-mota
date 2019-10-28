#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "Adafruit_Si7021.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "ESP8266httpUpdate.h"
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <FS.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>


#define ONE_WIRE_BUS 14
#define TEMPERATURE_PRECISION 10

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;

Adafruit_Si7021 sensor = Adafruit_Si7021();
Adafruit_BMP280 bmp;

bool si7021_found=false;
bool bmp280_found=false;
bool dallas_found=false;

void setup()
{
  Serial.begin(115200);

  WiFiManager wifiManager;
  wifiManager.setTimeout(180);

  if (!wifiManager.autoConnect("ESP-auto")) {
    Serial.println("failed to connect, we should reset as see if it connects");
    delay(1000);
    ESP.reset();
  }

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  if (sensor.begin()) {
    si7021_found = true;
    Serial.println("SI7021 found");
  }else{ 
    Serial.println("Did not find Si7021 sensor!");
  }

  if (bmp.begin()){
    bmp280_found=true;

    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
                  
    Serial.println("BMP280 Sensor found");
  }else{
    Serial.println("BMP280 Sensor missing");
  }
    
  if (!sensors.getAddress(insideThermometer, 0)){
    Serial.println("Dallas Device 0 found!");
    printAddress(insideThermometer);
    Serial.println();
    sensors.setResolution(insideThermometer, TEMPERATURE_PRECISION);

    dallas_found=true;
  }else{
    Serial.println("Unable to find Dallas address for Device 0");
  }
}

void loop() {
  t_httpUpdate_return ret = ESPhttpUpdate.update("senseapp.space", 443, "/esp/update/esp-mota.bin", "0");
  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.println("[update] Update failed.");
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("[update] Update no Update.");
      break;
    case HTTP_UPDATE_OK:
      Serial.println("[update] Update ok."); // may not be called since we reboot the ESP
      break;
  }

  si7021_sensor_routine();
  bmp280_sensor_routine();
  dallas_sensor_routine();

  Serial.print("Bye Bye");

  ESP.deepSleep(300e6);
  //ESP.deepSleep(10 * 1000000, WAKE_RF_DEFAULT);
}

void dallas_sensor_routine(){
  if(!dallas_found) return;
  
  Serial.print("Dallas Sensor - ");
  sensors.requestTemperatures();

  float tempC = sensors.getTempC(insideThermometer);
  Serial.print("Temp C: ");
  Serial.println(tempC);
}

void si7021_sensor_routine(){
  if(!si7021_found) return;
  
  Serial.print("SI7021 - Humidity:    ");
  Serial.print(sensor.readHumidity(), 2);
  Serial.print("\tTemperature: ");
  Serial.println(sensor.readTemperature(), 2);
}

void bmp280_sensor_routine(){
  if(!bmp280_found) return;

  Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
    Serial.println(" m");
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
