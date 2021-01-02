#include "secrets.h"

#include <Adafruit_MAX31865.h>
#include <ArduinoJson.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <PID_v1.h>

float tempActual = 0;

// Start heating the boiler when powered on
bool operMode = true;

const int maxRunTime = 180;  // Don't keep the boiler hot after this many minutes
double tempDesired = 215;    // This will be the current desired setpoint
const int WindowSize = 5000; // PID PWM Window in milliseconds

// Define the PID tuning Parameters
double Kp = 4.5;
double Ki = 0.125;
double Kd = 0.2;

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

// PID variables
// Using P_ON_M mode ( http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/ )
double Input, Output;
PID myPID(&Input, &Output, &tempDesired, Kp, Ki, Kd, P_ON_M, DIRECT);
double PWMOutput;
unsigned long windowStartTime;

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

// Round down a float to 2 decimal places
double round2(double value)
{
  return (int)(value * 100 + 0.5) / 100.0;
}

char *genJSON()
{
  DynamicJsonDocument doc(256);
  doc["Uptime"] = now / 1000;
  doc["Runtime"] = runTimeSecs;
  doc["Setpoint"] = round2(tempDesired);
  doc["ActualTemp"] = round2(tempActual);

  serializeJson(doc, jsonresult);
  return jsonresult;
}

void handleJSON()
{
  server.send(200, "application/json", String(genJSON()));
}

// unless true is specified, default to false
bool enablePID(bool enable = false)
{
  if (enable == false)
  {
    // de-activate the relay
    digitalWrite(relayPin, LOW);
    // set PID mode to manual mode
    myPID.SetMode(MANUAL);
    // Force PID output to 0
    Output = 0;
    // set operation mode to false
    operMode = false;
  }
  else if (enable == true)
  {
    myPID.SetMode(AUTOMATIC);
    runTimeStart = now;
    operMode = true;
  }
}

void controlRelay()
{
  // Provide the PID loop with the current temperature
  Input = tempActual;

  // If we've reached maxRunTime, disable the PID control
  if (runTimeMins >= maxRunTime && operMode)
  {
    enablePID(false);
  }
  else if (!myPID.GetMode() && operMode)
  {
    // Set the PID back to Automatic mode if operMode is true
    enablePID(true);
  }

  if (operMode)
  {
    myPID.Compute();

    // Starts a new PWM cycle every WindowSize milliseconds
    if (WindowSize < now - windowStartTime)
      windowStartTime += WindowSize;

    // Calculate the number of milliseconds that have passed in the current PWM cycle.
    // If that is less than the Output value, the relay is turned ON
    // If that is greater than (or equal to) the Output value, the relay is turned OFF.
    PWMOutput = Output * (WindowSize / 100.00);
    if ((PWMOutput > 100) && (PWMOutput > (now - windowStartTime)))
      digitalWrite(relayPin, HIGH);
    else
      digitalWrite(relayPin, LOW);
  }
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
  // Start server
  server.begin();
  Serial.println("HTTP server started");

  unsigned long bootTime = millis();
  Serial.println("Booted after " + String(bootTime / 1000.0) + " seconds");
}

void loop()
{
  keepTime();
  tempActual = getTemp();

  controlRelay();

  delay(0);
  server.handleClient();
}
