#include <Arduino.h>

#include <Wire.h>
#include <SSD1306Wire.h>
#include "Font_custom.h"

// Initialize the OLED display using Wire library
SSD1306Wire  display(0x3c, 5, 4);

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 14       // Digital pin connected to the DHT sensor 
#define DHTTYPE    DHT22     // DHT 22 (AM2302)
DHT_Unified dht(DHTPIN, DHTTYPE);


#include <Sds011.h>
#define SDS_PIN_RX 13
#define SDS_PIN_TX 15
HardwareSerial& serialSDS(Serial2);
Sds011Async< HardwareSerial > sds011(serialSDS);

bool is_SDS_running = true;

void start_SDS() {
    if (sds011.set_sleep(false)) { 
      is_SDS_running = true;
    } 
    else {
      Serial.println("Error waking SDS011");
    }
}

void stop_SDS() {
    if (sds011.set_sleep(true)) { 
      is_SDS_running = false; 
    } 
    else {
      Serial.println("Error sleeping SDS011");
    }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Air quality monitor starting...");

  dht.begin();

  display.init();
  display.flipScreenVertically();
  display.setFont(DejaVu_Sans_12);
  display.drawString(1, 15, "Starting Monitor.");
  display.drawString(1, 30, "Wait 30 seconds...");
  display.display();

  serialSDS.begin(9600, SERIAL_8N1, SDS_PIN_RX, SDS_PIN_TX);
  delay(100);
  Sds011::Report_mode report_mode;
  if (!sds011.get_data_reporting_mode(report_mode)) {
      Serial.println("Sds011::get_data_reporting_mode() failed");
  }
  if (Sds011::REPORT_ACTIVE != report_mode) {
      Serial.println("Turning on Sds011::REPORT_ACTIVE reporting mode");
      if (!sds011.set_data_reporting_mode(Sds011::REPORT_ACTIVE)) {
          Serial.println("Sds011::set_data_reporting_mode(Sds011::REPORT_ACTIVE) failed");
      }
  }else{
    Serial.println("Sds011 already in Active reporting mode");
  }
}


void update_screen(float pm2_5, float pm10, float temperature, float humidity)
{
  String str_pm2_5 = String(pm2_5, 2);
  String str_pm10 = String(pm10, 2);
  String str_temperature = String(temperature, 1);
  String str_humidity = String(humidity, 1);

  String line1 = String("PM2.5: " + str_pm2_5 + " ug/m3");
  String line2 = String("PM10 : " + str_pm10 + " ug/m3");
  String line3 = String("Temp: " + str_temperature + " C" );
  String line4 = String("Hum: " + str_humidity + "%" );
  display.clear();
  display.drawString(1, 0, line1);
  display.drawString(1, 15, line2);
  display.drawString(1, 30, line3);
  display.drawString(1, 45, line4);
  display.display();

  Serial.println(line1);
  Serial.println(line2);
  Serial.println(line3);
  Serial.println(line4);
}

void loop() {
  float temp = 0.0;
  float humid = 0.0;
  float p25 = 0.0;
  float p10 = 0.0;

  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else{
    temp = event.temperature;
  }

  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else{
    humid = event.relative_humidity;
  }

  constexpr uint32_t down_s = 30;
  int pm25,pm10;
  constexpr uint32_t duty_s = 30;
  uint32_t deadline = millis() + down_s * 1000;

  start_SDS();
  Serial.print("started SDS011 (is running = ");
  Serial.print(is_SDS_running);
  Serial.println(")");

  deadline = millis() + duty_s * 1000;
  while (static_cast<int32_t>(deadline - millis()) > 0) {
      delay(5000);
      Serial.println(static_cast<int32_t>(deadline - millis()) / 1000);
      // sds011.perform_work();
  }
  if(!sds011.query_data( pm25, pm10,10)){
    Serial.println("Query_data failed!");
    p25 = 0.0;
    p10 = 0.0;
  } else {
    p25 = float(pm25) / 10;
    p10 = float(pm10) / 10;
  }
  update_screen(p25,p10, temp, humid);
  
  stop_SDS();
  Serial.print("stopped SDS011 (is running = ");
  Serial.print(is_SDS_running);
  Serial.println(")");
  
  deadline = millis() + down_s * 1000;
  while (static_cast<int32_t>(deadline - millis()) > 0) {
      delay(2000);
      Serial.println(static_cast<int32_t>(deadline - millis()) / 1000);
      // sds011.perform_work();
  }
}
