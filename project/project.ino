#include "secrets.h"

#include <Adafruit_MAX31865.h>
#include <ArduinoJson.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <PID_v1.h>
#include <EEPROM.h>
#include <PID_AutoTune_v0.h>

// hardware pinout
int relayPin = 0; // D3

int maxCLK = 14; // D5
int maxSDO = 12; // D6
int maxSDI = 13; // D7
int maxCS = 4;   // D2

// This will be the current desired setpoint
double tempDesired = 215;

// This will be the latest actual reading
double tempActual = 0;

// PID PWM Window in milliseconds
const int WindowSize = 5000;

// Define the PID tuning Parameters
double Kp = 4.5;
double Ki = 0.125;
double Kd = 0.2;

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(maxCS, maxSDI, maxSDO, maxCLK);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
float RREF = 430.0;
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
float RNOMINAL = 100.0;

// All timers reference the value of now
unsigned long now = millis(); //This variable is used to keep track of time

ESP8266WebServer server(80);

char jsonresult[512];

// PID variables
// Using P_ON_M mode ( http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/ )
double Input, Output;
PID myPID(&Input, &Output, &tempDesired, Kp, Ki, Kd, P_ON_M, DIRECT);
double PWMOutput;
unsigned long windowStartTime;

// PID Autotune parameters
byte ATuneModeRemember = 2;
double aTuneStep = 50, aTuneNoise = 1, aTuneStartValue = 100;
unsigned int aTuneLookBack = 20;
boolean tuning = false;
PID_ATune aTune(&Input, &Output);

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

  float temp = thermo.temperature(RNOMINAL, RREF);
  // Convert from C to F
  temp = (9.0 / 5.0) * temp + 32.0;

  Serial.println(temp);

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

  return temp;
}

void enableAutoTune(boolean enable)
{
  tuning = enable;

  if (enable)
  {
    //Set the output to the desired starting frequency.
    Output = aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
  }
  else
  {
    //cancel autotune
    aTune.Cancel();
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if (start){
    ATuneModeRemember = myPID.GetMode();
  }
  else{
    myPID.SetMode(ATuneModeRemember);
  }
}

// Round down to 2 decimal places
double round2(double value)
{
  return (int)(value * 100 + 0.5) / 100.0;
}

char *genJSON()
{
  DynamicJsonDocument doc(256);
  doc["Uptime"] = now / 1000;
  doc["Setpoint"] = round2(tempDesired);
  doc["ActualTemp"] = round2(tempActual);
  doc["Kp"] = round2(Kp);
  doc["Ki"] = round2(Ki);
  doc["Kd"] = round2(Kd);
  doc["autotuning"] = tuning;

  serializeJson(doc, jsonresult);
  return jsonresult;
}

void handleJSON()
{
  server.send(200, "application/json", String(genJSON()));
}

void handleSetvals()
{
  String message;

  String setpoint_val = server.arg("setpoint");

  if (setpoint_val != NULL)
  {
    double setpoint_tmp = setpoint_val.toFloat();
    if (setpoint_tmp <= 221.1 && setpoint_tmp > 0.1)
    {
      tempDesired = setpoint_tmp;
      message += "Setpoint: " + setpoint_val;
      message += "\n";
    }
    else
    {
      message += "Setpoint: " + String(setpoint_val) + " is invalid\n";
    }
  }

  String kp_val = server.arg("kp");
  if (kp_val != NULL)
  {
    Kp = kp_val.toFloat();
    message += "Kp: " + kp_val;
  }

  String ki_val = server.arg("ki");
  if (ki_val != NULL)
  {
    Ki = ki_val.toFloat();
    message += "Ki: " + ki_val;
  }

  String kd_val = server.arg("kd");
  if (kd_val != NULL)
  {
    Kd = kd_val.toFloat();
    message += "Kd: " + kd_val;
  }

  server.send(200, "text/plain", message);
}

void handleSave()
{
  writeConfigValuesToEEPROM();

  server.send(200, "text/plain", "Wrote config to EEPROM.");
}

void handleAutotuneStart()
{
  enableAutoTune(true);
  server.send(200, "text/plain", "Autotune started.");
}

void handleAutotuneStop()
{
  enableAutoTune(false);
  server.send(200, "text/plain", "Autotune stopped.");
}

void controlRelay()
{
  // Provide the PID loop with the current temperature
  Input = tempActual;

  if (tuning)
  {
    byte val = (aTune.Runtime());
    if (val != 0)
    {
      tuning = false;
    }
    if (!tuning)
    {
      // We're done, set the tuning parameters
      Kp = aTune.GetKp();
      Ki = aTune.GetKi();
      Kd = aTune.GetKd();
      myPID.SetTunings(Kp, Ki, Kd);
      AutoTuneHelper(false);
    }
  }
  else
  {
    myPID.Compute();
  }

  // Starts a new PWM cycle every WindowSize milliseconds
  if (WindowSize < (now - windowStartTime))
  {
    windowStartTime += WindowSize;
  }

  // Calculate the number of milliseconds that have passed in the current PWM cycle.
  // If that is less than the Output value, the relay is turned ON
  // If that is greater than (or equal to) the Output value, the relay is turned OFF.
  PWMOutput = Output * (WindowSize / 100.00);
  if ((PWMOutput > 100) && (PWMOutput > (now - windowStartTime)))
  {
    digitalWrite(relayPin, HIGH);
  }
  else
  {
    digitalWrite(relayPin, LOW);
  }
}

void writeConfigValuesToEEPROM()
{
  // Store config values to eeprom
  EEPROM.begin(sizeof(double) * 4);

  int EEaddress = 0;

  EEPROM.put(EEaddress, tempDesired);
  EEaddress += sizeof(tempDesired);
  EEPROM.put(EEaddress, Kp);
  EEaddress += sizeof(Kp);
  EEPROM.put(EEaddress, Ki);
  EEaddress += sizeof(Ki);
  EEPROM.put(EEaddress, Kd);
  EEPROM.commit();
  EEPROM.end();
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

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/json", handleJSON);
  server.on("/set", HTTP_POST, handleSetvals);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/autotunestart", HTTP_POST, handleAutotuneStart);
  server.on("/autotunestop", HTTP_POST, handleAutotuneStop);

  // Start server
  server.begin();
  Serial.println("HTTP server started");

  // Initialize the target setpoint and PID parameters
  EEPROM.begin(sizeof(double) * 4);
  // Read configuration values from eeprom
  int EEaddress = 0;
  EEPROM.get(EEaddress, tempDesired);
  EEaddress += sizeof(tempDesired);
  EEPROM.get(EEaddress, Kp);
  EEaddress += sizeof(Kp);
  EEPROM.get(EEaddress, Ki);
  EEaddress += sizeof(Ki);
  EEPROM.get(EEaddress, Kd);
  EEPROM.end();

  // TODO SK: Validate the config values

  unsigned long bootTime = millis();
  Serial.println("Booted after " + String(bootTime / 1000.0) + " seconds");
}

void loop()
{
  now = millis();
  // tempActual = getTemp();

  // controlRelay();

  delay(0);
  server.handleClient();
}
