#include <ESP8266WiFi.h>
#include <Adafruit_MAX31865.h>
#include "secrets.h"

// hardware pinout
int relayPin = 0; // D3

int maxCLK = 14; // D5
int maxSDO = 12; // D6
int maxSDI = 13; // D7
int maxCS = 4;   // D2

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(maxCS, maxSDI, maxSDO, maxCLK);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0

// Define the PID tuning Parameters
double Kp = 4.5;
double Ki = 0.125;
double Kd = 0.2;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");

  // set to 2WIRE or 4WIRE as necessary
  thermo.begin(MAX31865_3WIRE);

  // Wifi client mode
  String ssid = WIFI_SSID;
  String password = WIFI_PASSWORD;
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  unsigned long bootTime = millis();
  Serial.println("Booted after " + String(bootTime / 1000.0) + " seconds");
}

void loop() {
  uint16_t rtd = thermo.readRTD();

  Serial.print("RTD value: "); Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
  Serial.print("Ratio = "); Serial.println(ratio,8);
  Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  Serial.print("Temperature = "); Serial.println(thermo.temperature(RNOMINAL, RREF));

  // Check and print any faults
  uint8_t fault = thermo.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    thermo.clearFault();
  }
  Serial.println();
  delay(1000);

}
