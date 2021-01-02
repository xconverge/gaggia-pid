#include "secrets.h"

#include <Adafruit_MAX31865.h>
#include <ArduinoJson.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>

double Setpoint = 215;

// hardware pinout
int relayPin = 0; // D3

int maxCLK = 14; // D5
int maxSDO = 12; // D6
int maxSDI = 13; // D7
int maxCS = 4;   // D2

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(maxCS, maxSDI, maxSDO, maxCLK);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
float RREF = 430.0;
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
float RNOMINAL = 100.0;

// All timers reference the value of now
unsigned long now = millis(); //This variable is used to keep track of time
int runTimeMins;
long runTimeSecs;
unsigned long runTimeStart = now;

ESP8266WebServer server(80);

char jsonresult[512];

float getTemp()
{
  uint16_t rtd = thermo.readRTD();

  Serial.print("RTD value: ");
  Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
  Serial.print("Ratio = ");
  Serial.println(ratio, 8);
  Serial.print("Resistance = ");
  Serial.println(RREF * ratio, 8);
  Serial.print("Temperature = ");

  float tempActual = thermo.temperature(RNOMINAL, RREF);
  Serial.println(tempActual);

  // Check and print any faults
  uint8_t fault = thermo.readFault();
  if (fault)
  {
    Serial.print("Fault 0x");
    Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH)
    {
      Serial.println("RTD High Threshold");
    }
    if (fault & MAX31865_FAULT_LOWTHRESH)
    {
      Serial.println("RTD Low Threshold");
    }
    if (fault & MAX31865_FAULT_REFINLOW)
    {
      Serial.println("REFIN- > 0.85 x Bias");
    }
    if (fault & MAX31865_FAULT_REFINHIGH)
    {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_RTDINLOW)
    {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_OVUV)
    {
      Serial.println("Under/Over voltage");
    }
    thermo.clearFault();
  }
  Serial.println();

  return tempActual;
}

void keepTime(void)
{
  now = millis(); //Keep track of time
  runTimeSecs = (now - runTimeStart) / 1000;
  runTimeMins = (now - runTimeStart) / 60000;
}

char *genJSON()
{
  DynamicJsonDocument doc(256);
  doc["Uptime"] = now / 1000;
  doc["Runtime"] = runTimeSecs;
  doc["Setpoint"] = Setpoint;

  serializeJson(doc, jsonresult);
  return jsonresult;
}

void handleJSON()
{
  server.send(200, "application/json", String(genJSON()));
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting...");

  // Set the Relay to output mode and ensure the relay is off
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);

  thermo.begin(MAX31865_3WIRE);

  // Wifi client mode
  String ssid = WIFI_SSID;
  String password = WIFI_PASSWORD;
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/json", handleJSON);
  // Start server
  server.begin();
  Serial.println("HTTP server started");

  unsigned long bootTime = millis();
  Serial.println("Booted after " + String(bootTime / 1000.0) + " seconds");
}

void loop()
{
  keepTime();
  float currentTemp = getTemp();

  server.handleClient();
  delay(1000);
}
