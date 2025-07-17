#include <Adafruit_MAX31865.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <MQTT.h>  // https://github.com/256dpi/arduino-mqtt
#include <PID_AutoTune_v0.h>
#include <PID_v1.h>

#include "secrets.h"

// hardware pinout
int relayPin = 0;  // D3

int maxCLK = 14;  // D5
int maxSDO = 12;  // D6
int maxSDI = 13;  // D7
int maxCS = 4;    // D2

// This will be the temporary steam setpoint
double temporary_steam_temp = 0;
// This will be the duration in milliseconds for the temporary steam setpoint
unsigned long temporary_steam_duration = 0;
// This will be the time in milliseconds when the temporary steam setpoint was
// set
unsigned long temporary_steam_start_time = 0;

// Bang-bang steam hysteresis (deg F)
// Heater turns ON when temp <= (steamTemp - STEAM_BANG_HYST_F)
// and OFF when temp >= steamTemp (one-sided band)
const double STEAM_BANG_HYST_F = 5.0;

// This will be the current desired setpoint
double tempDesired = 220;

// This is the actual PID setpoint, it is usually set to the tempDesired value
// but can be overridden temporarily with the temporary_steam_temp
double currentPIDSetpoint = tempDesired;

// This will be the latest actual reading
double tempActual = 0;

// Safety value, will always turn off the relay if this is exceeded
double maxBoilerTemp = 315;

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
unsigned long now = millis();  // This variable is used to keep track of time

ESP8266WebServer server(80);

// Try to connect to MQTT broker, will disable to false automatically if can't
// connect to broker
boolean useMQTT = true;
WiFiClient net;
MQTTClient mqttClient;
unsigned long previousMqttStatsMillis = now;
unsigned long mqttStatsInterval = 1000;
String mqttStatusTopicName = "espresso/status";

char jsonresult[512];

// PID variables
// Using P_ON_M mode (
// http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/
// )
double Input, Output;
PID myPID(&Input, &Output, &currentPIDSetpoint, Kp, Ki, Kd, P_ON_M, DIRECT);
double PWMOutput;
unsigned long windowStartTime;

// PID Autotune parameters
byte ATuneModeRemember = 2;
boolean tuning = false;
PID_ATune aTune(&Input, &Output);

float getTemp() {
  uint16_t rtd = thermo.readRTD();

  float temp = thermo.temperature(RNOMINAL, RREF);

  // Convert from C to F
  temp = (9.0 / 5.0) * temp + 32.0;

  bool debugPrints = false;
  if (debugPrints) {
    Serial.print("RTD value: ");
    Serial.println(rtd);
    float ratio = rtd;
    ratio /= 32768;
    Serial.print("Ratio = ");
    Serial.println(ratio, 8);
    Serial.print("Resistance = ");
    Serial.println(RREF * ratio, 8);
    Serial.print("Temperature = ");
    Serial.println(temp);
  }

  // Check and print any faults
  uint8_t fault = thermo.readFault();
  if (fault) {
    Serial.print("Fault 0x");
    Serial.println(fault, HEX);
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

    temp = 9999;
  }

  return temp;
}

void enableAutoTune(double step, double noise, int lookback) {
  tuning = true;

  aTune.SetNoiseBand(noise);
  aTune.SetOutputStep(step);
  aTune.SetLookbackSec(lookback);
  AutoTuneHelper(true);
}

void disableAutoTune() {
  tuning = false;

  // Cancel autotune
  aTune.Cancel();
  AutoTuneHelper(false);
}

void AutoTuneHelper(boolean start) {
  if (start) {
    ATuneModeRemember = myPID.GetMode();
  } else {
    myPID.SetMode(ATuneModeRemember);
  }
}

// Round down to 2 decimal places
double round2(double value) { return (int)(value * 100 + 0.5) / 100.0; }

char *genJSON() {
  DynamicJsonDocument doc(256);
  doc["Uptime"] = now / 1000;
  doc["Setpoint"] = round2(currentPIDSetpoint);
  doc["ActualTemp"] = round2(tempActual);
  doc["Kp"] = round2(Kp);
  doc["Ki"] = round2(Ki);
  doc["Kd"] = round2(Kd);
  doc["autotuning"] = tuning;

  serializeJson(doc, jsonresult);
  return jsonresult;
}

void handleJSON() { server.send(200, "application/json", String(genJSON())); }

void handleSetvals() {
  String message;

  String setpoint_val = server.arg("setpoint");

  if (setpoint_val != NULL) {
    double setpoint_tmp = setpoint_val.toFloat();
    if (setpoint_tmp <= 221.1 && setpoint_tmp > 0.1) {
      tempDesired = setpoint_tmp;
      message += "Setpoint: " + setpoint_val;
      message += "\n";
    } else {
      message += "Setpoint: " + String(setpoint_val) + " is invalid\n";
    }
  }

  String kp_val = server.arg("kp");
  if (kp_val != NULL) {
    Kp = kp_val.toFloat();
    message += "Kp: " + kp_val;
  }

  String ki_val = server.arg("ki");
  if (ki_val != NULL) {
    Ki = ki_val.toFloat();
    message += "Ki: " + ki_val;
  }

  String kd_val = server.arg("kd");
  if (kd_val != NULL) {
    Kd = kd_val.toFloat();
    message += "Kd: " + kd_val;
  }

  server.send(200, "text/plain", message);
}

void handleSave() {
  writeConfigValuesToEEPROM();

  server.send(200, "text/plain", "Wrote config to EEPROM.");
}

// If setpoint is set, it will set temporary_steam_temp, which will persist for
// the duration specified
void handleSteam() {
  if (!(server.hasArg("setpoint") && server.hasArg("duration"))) {
    server.send(400, "text/plain",
                "Error: setpoint and duration are both required.");
    return;
  }

  String setpoint_val = server.arg("setpoint");
  String duration_val = server.arg("duration");

  float setpoint_tmp = setpoint_val.toFloat();
  float duration_tmp = duration_val.toFloat();  // seconds

  String message;
  bool ok = true;

  // bounds
  if (!(setpoint_tmp > 0.1f && setpoint_tmp <= 305.0f)) {
    message += "Invalid steam setpoint: " + setpoint_val + "\n";
    ok = false;
  }
  // Max 20 min, min 0.1 s
  if (!(duration_tmp > 0.1f && duration_tmp <= (60.0f * 20.0f))) {
    message += "Invalid steam duration: " + duration_val + "\n";
    ok = false;
  }

  if (!ok) {
    server.send(400, "text/plain", message);
    return;
  }

  // Accept: convert to ms, arm new cycle
  temporary_steam_temp = setpoint_tmp;
  temporary_steam_duration = (unsigned long)(duration_tmp * 1000.0f);
  temporary_steam_start_time = 0;  // force fresh start next loop()

  message = "Steam setpoint: " + setpoint_val + "\n";
  message += "Steam duration: " + duration_val + "s\n";
  server.send(200, "text/plain", message);
}

void handleAutotuneStart() {
  double step = 750;
  String step_val = server.arg("step");
  if (step_val != NULL) {
    step = step_val.toFloat();
  }

  double noise = 1;
  String noise_val = server.arg("noise");
  if (noise_val != NULL) {
    noise = noise_val.toFloat();
  }

  unsigned int lookback = 20;
  String lookback_val = server.arg("lookback");
  if (lookback_val != NULL) {
    lookback = lookback_val.toInt();
  }

  String message = "Autotune started: noise(" + String(noise) + ") " + "step(" +
                   String(step) + ") lookback(" + String(lookback) + ")";
  enableAutoTune(step, noise, lookback);
  server.send(200, "text/plain", message);
}

void handleAutotuneStop() {
  disableAutoTune();
  server.send(200, "text/plain", "Autotune stopped.");
}

void controlRelay() {
  // Provide the PID loop with the current temperature
  Input = tempActual;

  // Safety to turn off if max temp is exceeded
  if (Input >= maxBoilerTemp) {
    digitalWrite(relayPin, LOW);
    return;
  }

  if (tuning) {
    if (aTune.Runtime()) {
      tuning = false;

      // We're done, set the tuning parameters
      Kp = aTune.GetKp();
      Ki = aTune.GetKi();
      Kd = aTune.GetKd();
      myPID.SetTunings(Kp, Ki, Kd);
      AutoTuneHelper(false);
    }
  } else {
    myPID.Compute();
  }

  // Starts a new PWM cycle every WindowSize milliseconds
  if (WindowSize < (now - windowStartTime)) {
    windowStartTime += WindowSize;
  }

  // Calculate the number of milliseconds that have passed in the current PWM
  // cycle. If that is less than the Output value, the relay is turned ON If
  // that is greater than (or equal to) the Output value, the relay is turned
  // OFF.
  PWMOutput = Output * (WindowSize / 100.00);
  if ((PWMOutput > 100) && (PWMOutput > (now - windowStartTime))) {
    digitalWrite(relayPin, HIGH);
  } else {
    digitalWrite(relayPin, LOW);
  }
}

void writeConfigValuesToEEPROM() {
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

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");

  // Set the Relay to output mode and ensure the relay is off
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);

  // PID Window
  windowStartTime = now;
  // PID output 0 - 100
  myPID.SetOutputLimits(0, 100);
  // PID Samples every 50ms
  myPID.SetSampleTime(50);
  // Enable control loop
  myPID.SetMode(AUTOMATIC);

  // Set 1 for PID or 0 for PI controller autotuning parameters
  aTune.SetControlType(1);

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
  server.on("/set", HTTP_POST, handleSetvals);
  server.on("/steam", HTTP_POST, handleSteam);
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

  // Set the PID setpoint to the desired temperature
  currentPIDSetpoint = tempDesired;

  // TODO SK: Validate the config values

  if (useMQTT) {
    // Start MQTT client
    mqttClient.begin(MQTT_HOST, net);
    mqttClient.onMessage(messageReceived);

    Serial.println("Trying to connect to MQTT broker...");
    unsigned long mqttConnectTimeStart = millis();
    mqttClient.connect("espresso");
    unsigned long mqttConnectElapsedTime = millis() - mqttConnectTimeStart;

    if (mqttClient.connected()) {
      Serial.println("Connected to MQTT broker after " +
                     String(mqttConnectElapsedTime / 1000.0) +
                     " seconds, enabling mqtt!");
      mqttClient.subscribe(mqttStatusTopicName);
    } else {
      Serial.println("Did not connect to MQTT broker after " +
                     String(mqttConnectElapsedTime / 1000.0) +
                     " seconds, disabling mqtt!");
      useMQTT = false;
    }
  }

  unsigned long bootTime = millis();
  Serial.println("Booted after " + String(bootTime / 1000.0) + " seconds");
}

void loop() {
  now = millis();
  tempActual = getTemp();
  // Provide the PID loop with the current temperature
  Input = tempActual;

  // Track whether a steam override is armed (nonzero duration & temp).
  const bool steamActive = (temporary_steam_duration != 0 && temporary_steam_temp != 0);

  // We received a temporary steam setpoint and duration so we need to apply it
  if (steamActive) {
    // On entering steam, take PID out of the loop (MANUAL) and record start
    if (myPID.GetMode() != MANUAL) {
      myPID.SetMode(MANUAL);
    }

    // Track current steam target for telemetry
    currentPIDSetpoint = temporary_steam_temp;

    if (temporary_steam_start_time == 0) {
      // If this is the first time we set the temporary steam setpoint, record
      // the start time
      temporary_steam_start_time = now;
    } else {
      // If we already have a start time, check if we need to update it
      if (now - temporary_steam_start_time >= temporary_steam_duration) {
        // If the duration has passed, reset the temporary steam setpoint
        temporary_steam_temp = 0;
        temporary_steam_duration = 0;
        temporary_steam_start_time = 0;
        // Reset to the original desired temperature
        currentPIDSetpoint = tempDesired;
        // Restore PID control for brew.
        myPID.SetMode(AUTOMATIC);
      }
    }
  } else {
    // If no temporary steam setpoint, use the original desired temperature
    if (currentPIDSetpoint != tempDesired) {
      // If the current PID setpoint is not the desired temperature, reset it
      // This is to ensure that we don't keep the temporary steam setpoint if it
      // was not set or if it has expired
      currentPIDSetpoint = tempDesired;
    }
  }

  if (currentPIDSetpoint > maxBoilerTemp - 1) {
    currentPIDSetpoint = maxBoilerTemp - 1;
  }

  // --- Steam bang-bang control ---------------------------------------------
  if (steamActive) {
    // Safety first: hard cutoff
    if (tempActual >= maxBoilerTemp) {
      digitalWrite(relayPin, LOW);
    } else if (tempActual <= (temporary_steam_temp - STEAM_BANG_HYST_F)) {
      // Below lower band: heater ON
      digitalWrite(relayPin, HIGH);
    } else if (tempActual >= temporary_steam_temp) {
      // At/above steam target: heater OFF
      digitalWrite(relayPin, LOW);
    }
  } else {
    // Ensure PID enabled during brew mode.
    if (myPID.GetMode() != AUTOMATIC) {
      myPID.SetMode(AUTOMATIC);
    }

    // Brew (PID) control
    controlRelay();
  }

  server.handleClient();

  if (useMQTT) {
    // Publish status via mqtt
    mqttClient.loop();
    if (now - previousMqttStatsMillis > mqttStatsInterval) {
      mqttClient.publish(mqttStatusTopicName, genJSON());
      previousMqttStatsMillis = now;
    }
  }

  delay(10);
}
