#include <Adafruit_AHTX0.h>
#include <Adafruit_LTR390.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
// #include <OneWire.h>
// #include <DallasTemperature.h>
#include "DFRobot_GDL.h"
#include <RTClib.h>

Adafruit_AHTX0 aht;
Adafruit_LTR390 ltr = Adafruit_LTR390();

#define PHPIN 6
#define TDSPIN 2
#define HIGH_LEVEL_SENSOR 18 // Water Lvl
#define LOW_LEVEL_SENSOR 3 // Water Lvl
#define WATER_PUMP_PIN 9
#define WATER_PUMP_BTN 7
#define NUTRIENT_PUMP1_PIN 4
#define NUTRIENT_PUMP2_PIN 5

#define TFT_DC 8
#define TFT_CS 1
#define TFT_RST 14
DFRobot_ST7735_128x160_HW_SPI screen(/*dc=*/TFT_DC, /*cs=*/TFT_CS, /*rst=*/TFT_RST);

// OneWire oneWire(DS18B20PIN);
// DallasTemperature ds18b20_sensor(&oneWire);

float tdsValue = 0;
float ecValue = 0;
float pHValue = 0;
unsigned long int avgValue; // Store the average value of the sensor feedback
float b;
int buf[10], tempBuf;
int ec_tds_buffer[10] = { 0 };
float temperature = 0, hum = 0, water_temp = 0, alsLux = 0, uvIndex = 0;
float alsRaw = 0, uvsRaw = 0;

// Timer durations in minutes
const int ON_DURATION = 10; // Duration the pump stays ON
const int OFF_DURATION = 20; // Duration the pump stays OFF

// Variables to track timing
RTC_DS3231 rtc;
DateTime lastToggleTime;
bool pumpState = false; // false = OFF, true = ON
bool waterLevelLow = false;

const int DATA_COLLECT_INTERVAL = 60;
DateTime dataLastCollectedTime;
int lastSecond = -1;

// Duration for nutrient dosing (in seconds) – pumps run for 14 seconds.
const unsigned long NUTRIENT_DISPENSE_DURATION_SEC = 14;

// Cooldown period after a dosing event (in seconds) – 5 minutes = 300 seconds.
const unsigned long COOLDOWN_PERIOD_SEC = 300;

// TDS thresholds (adjust as needed)
const float TDS_LOW_LIMIT = 600.0; // Below this, dosing is needed.
const float TDS_HIGH_LIMIT = 800.0; // At or above this, dosing is not allowed.

// Global variables for dosing control:
bool nutrientDispenseInProgress = false; // true if pumps are currently dosing
bool nutrientDosingDone = false; // true if a dosing event has completed (and cooldown is active)

// RTC-based timestamp variables for dosing:
DateTime nutrientDispenseStartTimeRtc; // When the dosing (pump on) began.
DateTime lastDosingEndTime; // When the dosing event ended (used for cooldown)

// Data sending
DateTime lastDataSentTime;

const char* mqtt_broker = "** REMOVED **.emqxsl.com";
const char* topic = "prod/test";
const char* command_topic = "devices/test/command";
const char* mqtt_username = "** REMOVED **";
const char* mqtt_password = "** REMOVED **";
const int mqtt_port = 8883;
const char* ca_cert = R"EOF(
-----BEGIN CERTIFICATE-----
** REMOVED **
-----END CERTIFICATE-----
)EOF";

WiFiClientSecure espClient;
PubSubClient client(espClient);

std::pair<float, float> calculateECTDS(int pin, float aref, float dynamicTemp, int buffer[], int bufferSize)
{
    static unsigned long analogSampleTimepoint = millis();
    static unsigned long printTimepoint = millis();
    static int bufferIndex = 0;
    float temperature = dynamicTemp;
    float adcRange = 4096.0; // 12-bit ADC range

    // Analog reading logic
    if (millis() - analogSampleTimepoint > 40U) {
        analogSampleTimepoint = millis();
        buffer[bufferIndex] = analogRead(pin); // Read and store the analog value
        bufferIndex++;
        if (bufferIndex == bufferSize) {
            bufferIndex = 0;
        }
    }

    // Calculation logic
    if (millis() - printTimepoint > 800U) {
        printTimepoint = millis();

        // Copy buffer for median calculation
        int tempBuffer[bufferSize];
        for (int i = 0; i < bufferSize; i++) {
            tempBuffer[i] = buffer[i];
        }

        // Calculate average voltage using the median filtering algorithm
        int medianValue = getMedianNum(tempBuffer, bufferSize);
        float averageVoltage = (medianValue * aref) / adcRange;

        // Temperature compensation
        float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
        // float compensationVoltage = averageVoltage / compensationCoefficient;
        float compensationVoltage = averageVoltage;
        // Serial.printf("Avg V: %.2f, Comp Coeff: %.2f, Comp V: %.2f", averageVoltage, compensationCoefficient,
        // compensationVoltage); Serial.println();
        if (temperature <= 0) {
            compensationCoefficient = 1.0; // Avoid unrealistic adjustments for negative temperatures
        }

        // EC calculation in µS/cm (microSiemens per centimeter)
        float ecValue = compensationVoltage * 1000.0; // Conversion factor for EC

        // TDS calculation
        float tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage
                             - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage)
            * 0.5;

        // Return EC and TDS as a pair
        return { ecValue, tdsValue };
    }

    // Return 0 for both EC and TDS if not ready to calculate
    return { 0.0, 0.0 };
}

int getMedianNum(int bArray[], int iFilterLen)
{
    int bTab[iFilterLen];
    for (int i = 0; i < iFilterLen; i++) {
        bTab[i] = bArray[i];
    }
    for (int j = 0; j < iFilterLen - 1; j++) {
        for (int i = 0; i < iFilterLen - j - 1; i++) {
            if (bTab[i] > bTab[i + 1]) {
                int bTemp = bTab[i];
                bTab[i] = bTab[i + 1];
                bTab[i + 1] = bTemp;
            }
        }
    }
    if (iFilterLen % 2 == 1) {
        return bTab[iFilterLen / 2];
    } else {
        return (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
    }
}

void setup()
{
    Serial.begin(115200);

    pinMode(PHPIN, INPUT);

    pinMode(HIGH_LEVEL_SENSOR, INPUT_PULLUP);
    pinMode(LOW_LEVEL_SENSOR, INPUT_PULLUP);

    setupWifi();
    espClient.setCACert(ca_cert);
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(handleMqttCallback);

    // Initialize RTC
    if (!rtc.begin()) {
        Serial.println("Couldn't find RTC");
        while (1)
            ;
    }
    if (rtc.lostPower()) {
        Serial.println("RTC lost power, setting the time!");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Set RTC to compile time
    }

    // Setup nutrient pump relay pins
    pinMode(NUTRIENT_PUMP1_PIN, OUTPUT);
    pinMode(NUTRIENT_PUMP2_PIN, OUTPUT);
    pinMode(WATER_PUMP_BTN, INPUT);
    pinMode(WATER_PUMP_PIN, OUTPUT);

    digitalWrite(NUTRIENT_PUMP1_PIN, HIGH);
    digitalWrite(NUTRIENT_PUMP2_PIN, HIGH);
    digitalWrite(WATER_PUMP_PIN, LOW);

    // Initialize variables
    lastToggleTime = rtc.now();
    lastDataSentTime = rtc.now();
    lastDosingEndTime = rtc.now();
    nutrientDispenseStartTimeRtc = rtc.now();
    dataLastCollectedTime = rtc.now();

    if (!aht.begin()) {
        Serial.println("Could not find AHT? Check wiring");
        while (1)
            delay(10);
    }
    Serial.println("AHT10 or AHT20 found");
    initializeLTRSensor();

    setupDisplay();

    reconnectMqtt();
}

void loop()
{
    // digitalWrite(led, HIGH);

    if (!client.connected())
        reconnectMqtt();
    client.loop();

    DateTime now = rtc.now();
    if (now.second() != lastSecond) {
        lastSecond = now.second();
        readSensors();
        updateDisplayValues();
        checkNutrientDispense();
    }

    if (now.unixtime() - lastDataSentTime.unixtime() >= 30) {
        sendData();
        lastDataSentTime = now;
    }

    waterPumpTimerControl();
    // digitalWrite(led, LOW);
}

void waterPumpTimerControl()
{
    static bool lastButtonState = HIGH; // Previous state of the button
    bool currentButtonState = digitalRead(WATER_PUMP_BTN);

    if (waterLevelLow) {
        pumpState = false;
        digitalWrite(WATER_PUMP_PIN, LOW);
        return;
    }

    // Detect button press (falling edge)
    if (lastButtonState == HIGH && currentButtonState == LOW) {
        pumpState = !pumpState; // Toggle pump state
        digitalWrite(WATER_PUMP_PIN, pumpState ? HIGH : LOW); // Control the pump
        lastToggleTime = rtc.now(); // Sync the timer with the button press
    }

    lastButtonState = currentButtonState;

    // Water Pump timer logic
    DateTime currentTime = rtc.now();
    long elapsedSeconds = currentTime.unixtime() - lastToggleTime.unixtime(); // Seconds elapsed

    // Update pump state based on elapsed time
    if (pumpState && elapsedSeconds >= ON_DURATION * 60) {
        // Turn OFF the pump after ON_DURATION
        digitalWrite(WATER_PUMP_PIN, LOW); // Relay LOW = Pump OFF
        pumpState = false;
        lastToggleTime = currentTime;
        sendPumpData("Water", false);
        Serial.println("Pump OFF");
    } else if (!pumpState && elapsedSeconds >= OFF_DURATION * 60) {
        // Turn ON the pump after OFF_DURATION
        sendPumpData("Water", true);
        digitalWrite(WATER_PUMP_PIN, HIGH); // Relay HIGH = Pump ON
        pumpState = true;
        lastToggleTime = currentTime;
        Serial.println("Pump ON");
    }
}

void readSensors()
{
    DateTime currentTime = rtc.now();
    long elapsedSeconds = currentTime.unixtime() - dataLastCollectedTime.unixtime(); // Seconds elapsed

    if (digitalRead(LOW_LEVEL_SENSOR) == HIGH) {
        waterLevelLow = true;
        Serial.println("Water Level LOW!");
    }

    if (digitalRead(HIGH_LEVEL_SENSOR) == HIGH) {
        waterLevelLow = false;
        Serial.println("Water Tank Full!");
    }

    // if (elapsedSeconds >= DATA_COLLECT_INTERVAL) {
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);

    for (int i = 0; i < 10; i++) { // Get 10 sample values from the sensor for smoothing
        buf[i] = analogRead(PHPIN);
        delay(10);
    }
    for (int i = 0; i < 9; i++) { // Sort the analog values from small to large
        for (int j = i + 1; j < 10; j++) {
            if (buf[i] > buf[j]) {
                tempBuf = buf[i];
                buf[i] = buf[j];
                buf[j] = tempBuf;
            }
        }
    }
    avgValue = 0;
    for (int i = 2; i < 8; i++) { // Take the average value of 6 center samples
        avgValue += buf[i];
    }
    pHValue = (((float)avgValue * 3.3) / 4095) - 1.51;

    ltr.setMode(LTR390_MODE_ALS);
    alsRaw = ltr.readALS();
    delay(100);
    ltr.setMode(LTR390_MODE_UVS);
    uvsRaw = ltr.readUVS();

    alsLux = alsToLux(alsRaw);
    uvIndex = uvsToUvIndex(uvsRaw);

    temperature = temp.temperature;
    hum = humidity.relative_humidity;

    // ds18b20_sensor.requestTemperatures();
    // water_temp = ds18b20_sensor.getTempCByIndex(0);
    // if (water_temp == DEVICE_DISCONNECTED_C) {
    // Serial.println("Error: DS18B20 Disconnected!");
    // water_temp = 0;
    // }

    auto [ec, tds] = calculateECTDS(TDSPIN, 5.0, temperature, ec_tds_buffer, 10);
    tdsValue = tds;
    ecValue = ec;

    Serial.printf("Temperature: %.2f, Humidity: %.2f, ALS: %.1f, UVS: %.2f, ALSRaw: %.1f, UVSRaw: %.2f, PH: %.2f, TDS: "
                  "%.2f, EC: %.2f, Water Temp: %.2f\n",
        temperature, hum, alsLux, uvIndex, alsRaw, uvsRaw, pHValue, tdsValue, ecValue, water_temp);
    // }
}

void initializeLTRSensor()
{
    if (!ltr.begin()) {
        Serial.println("Couldn't find LTR sensor!");
        while (1)
            delay(10);
    }
    Serial.println("Found LTR sensor!");
    ltr.setGain(LTR390_GAIN_9);
    ltr.setResolution(LTR390_RESOLUTION_18BIT);
}

void reconnectMqtt()
{
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP32Client", mqtt_username, mqtt_password)) {
            Serial.println("connected");
            client.subscribe(command_topic);
        } else {
            Serial.printf("failed, rc=%d\n", client.state());
            delay(5000);
        }
    }
}

void sendData()
{
    StaticJsonDocument<256> doc;
    doc["userEmail"] = "vsk102002@gmail.com";
    doc["deviceId"] = "test";
    doc["type"] = "data-log";
    doc["temperature"] = temperature;
    doc["humidity"] = hum;
    doc["pH"] = pHValue;
    doc["TDS"] = tdsValue;
    doc["EC"] = ecValue;
    doc["ALS"] = alsLux;
    doc["ALSraw"] = alsRaw;
    doc["UVS"] = uvIndex;
    doc["UVSraw"] = uvsRaw;
    doc["waterTemp"] = water_temp;

    char jsonBuffer[256];
    serializeJson(doc, jsonBuffer);
    client.publish(topic, jsonBuffer);
}

void sendPumpData(const char* pumpType, bool status)
{
    StaticJsonDocument<256> doc;
    doc["userEmail"] = "vsk102002@gmail.com";
    doc["deviceId"] = "test";
    doc["type"] = "pump-log";
    doc["status"] = status;
    doc["pumpType"] = pumpType;

    char jsonBuffer[256];
    serializeJson(doc, jsonBuffer);
    client.publish(topic, jsonBuffer);
}

void setupWifi()
{
    WiFi.begin("Viren_Macbook", "12345678");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\nWiFi connected. IP: %s\n", WiFi.localIP().toString().c_str());
}

void handleMqttCallback(char* topic, byte* message, unsigned int length)
{
    // Convert the incoming message to a String for easy parsing.
    String msg;
    for (unsigned int i = 0; i < length; i++) {
        msg += (char)message[i];
    }
    Serial.printf("Message arrived on topic %s: %s\n", topic, msg.c_str());

    // Parse the JSON message.
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, msg);
    if (error) {
        Serial.println("Failed to parse JSON command");
        return;
    }

    // Read the command from the JSON. (Assuming the key is "command")
    const char* command = doc["command"];
    if (command == nullptr) {
        Serial.println("No command received");
        return;
    }

    // Process the command.
    if (strcmp(command, "nutrient_on") == 0) {
        // Turn ON nutrient pumps.
        digitalWrite(NUTRIENT_PUMP1_PIN, LOW); // LOW turns pump ON
        digitalWrite(NUTRIENT_PUMP2_PIN, LOW);
        nutrientDispenseInProgress = false; // Reset any dosing state.
        nutrientDosingDone = false;
        sendPumpData("Nutrient", true);
        Serial.println("Command: Nutrient Pumps ON");
    }
    if (strcmp(command, "nutrient_off") == 0) {
        // Turn OFF nutrient pumps.
        digitalWrite(NUTRIENT_PUMP1_PIN, HIGH); // HIGH turns pump OFF
        digitalWrite(NUTRIENT_PUMP2_PIN, HIGH);
        nutrientDispenseInProgress = false;
        nutrientDosingDone = true;
        sendPumpData("Nutrient", false);
        Serial.println("Command: Nutrient Pumps OFF");
    }
    if (strcmp(command, "water_on") == 0) {
        // Turn ON water pump.
        digitalWrite(WATER_PUMP_PIN, HIGH); // HIGH turns water pump ON
        pumpState = true;
        lastToggleTime = rtc.now();
        sendPumpData("Water", true);
        Serial.println("Command: Water Pump ON");
    }
    if (strcmp(command, "water_off") == 0) {
        // Turn OFF water pump.
        digitalWrite(WATER_PUMP_PIN, LOW); // LOW turns water pump OFF
        pumpState = false;
        lastToggleTime = rtc.now();
        sendPumpData("Water", false);
        Serial.println("Command: Water Pump OFF");
    }
    if (strcmp(command, "emergency_off") == 0) {
        // Master failsafe: Turn OFF both nutrient and water pumps.
        digitalWrite(WATER_PUMP_PIN, LOW); // Shut off water pump.
        pumpState = false;
        digitalWrite(NUTRIENT_PUMP1_PIN, HIGH); // Shut off nutrient pumps.
        digitalWrite(NUTRIENT_PUMP2_PIN, HIGH);
        nutrientDispenseInProgress = false;
        nutrientDosingDone = true;
        lastToggleTime = rtc.now();
        lastDosingEndTime = rtc.now();
        sendPumpData("Water", false);
        sendPumpData("Nutrient", false);
        Serial.println("Command: Emergency OFF - All pumps shut down");
    }
    // if (strcmp(command, "request_status") == 0) {
    //   // Build a status response containing both pump statuses.
    //   StaticJsonDocument<256> statusDoc;
    //   statusDoc["userEmail"] = "vsk102002@gmail.com";
    //   statusDoc["deviceId"] = "test";
    //   statusDoc["type"] = "status-response";
    //   // Water pump status: use the pumpState variable.
    //   statusDoc["waterPumpStatus"] = pumpState;
    //   // Nutrient pump status: use the global nutrientPumpState variable.
    //   statusDoc["nutrientPumpStatus"] = nutrientDispenseInProgress;

    //   char statusBuffer[256];
    //   serializeJson(statusDoc, statusBuffer);
    //   client.publish(topic, statusBuffer);
    //   Serial.println("Command: Status requested. Status response sent.");
    // }
}

void setupDisplay()
{
    screen.begin();
    screen.fillScreen(COLOR_RGB565_BLACK);
    screen.setTextSize(1);
    screen.setTextColor(COLOR_RGB565_WHITE);

    // Draw static labels once:
    screen.setCursor(10, 10);
    screen.print("Sensor Readings:");

    screen.setCursor(10, 20);
    screen.print("Temperature:");

    screen.setCursor(10, 30);
    screen.print("Humidity:");

    screen.setCursor(10, 40);
    screen.print("pH:");

    screen.setCursor(10, 50);
    screen.print("TDS:");

    screen.setCursor(10, 60);
    screen.print("EC:");

    screen.setCursor(10, 70);
    screen.print("Water Temp:");

    screen.setCursor(10, 80);
    screen.print("ALS:"); // Ambient Light Sensor (lux)

    screen.setCursor(10, 90);
    screen.print("UVS:"); // UV Sensor (UV index)

    // Moved pump status and elapsed time down:
    screen.setCursor(10, 100);
    screen.print("Pump:");

    screen.setCursor(10, 110);
    screen.print("Elapsed:");
}

void updateDisplayValues()
{
    char buffer[30];

    // For each dynamic value, clear the old number and print the new one.
    // Adjust the coordinates and size of the cleared rectangle as needed.

    // Temperature value (clearing area where value appears)
    screen.fillRect(90, 20, 40, 8, COLOR_RGB565_BLACK); // clear area
    screen.setCursor(90, 20);
    snprintf(buffer, sizeof(buffer), "%.2f", temperature);
    screen.print(buffer);

    // Humidity value
    screen.fillRect(90, 30, 40, 8, COLOR_RGB565_BLACK);
    screen.setCursor(90, 30);
    snprintf(buffer, sizeof(buffer), "%.2f", hum);
    screen.print(buffer);

    // pH value
    screen.fillRect(90, 40, 40, 8, COLOR_RGB565_BLACK);
    screen.setCursor(90, 40);
    snprintf(buffer, sizeof(buffer), "%.2f", pHValue);
    screen.print(buffer);

    // TDS value
    screen.fillRect(90, 50, 40, 8, COLOR_RGB565_BLACK);
    screen.setCursor(90, 50);
    snprintf(buffer, sizeof(buffer), "%.2f", tdsValue);
    screen.print(buffer);

    // EC value
    screen.fillRect(90, 60, 40, 8, COLOR_RGB565_BLACK);
    screen.setCursor(90, 60);
    snprintf(buffer, sizeof(buffer), "%.2f", ecValue);
    screen.print(buffer);

    // Water temperature value
    screen.fillRect(90, 70, 40, 8, COLOR_RGB565_BLACK);
    screen.setCursor(90, 70);
    snprintf(buffer, sizeof(buffer), "%.2f", water_temp);
    screen.print(buffer);

    // ALS (lux) value on line 80
    screen.fillRect(50, 80, 50, 8, COLOR_RGB565_BLACK); // Adjust x position and width as needed
    screen.setCursor(50, 80);
    snprintf(buffer, sizeof(buffer), "%.1f", alsLux);
    screen.print(buffer);

    // UVS (UV index) value on line 90
    screen.fillRect(50, 90, 50, 8, COLOR_RGB565_BLACK); // Adjust x position and width as needed
    screen.setCursor(50, 90);
    snprintf(buffer, sizeof(buffer), "%.2f", uvIndex);
    screen.print(buffer);

    // Update Pump Status on line 100
    screen.fillRect(50, 100, 50, 8, COLOR_RGB565_BLACK); // Clear previous pump status area
    screen.setCursor(50, 100);
    screen.print(pumpState ? "ON" : "OFF");

    // Update Elapsed Time on line 110
    DateTime now = rtc.now();
    long elapsedSeconds = now.unixtime() - lastToggleTime.unixtime();
    int minutes = elapsedSeconds / 60;
    int seconds = elapsedSeconds % 60;
    screen.fillRect(60, 110, 60, 8, COLOR_RGB565_BLACK); // Clear the previous elapsed time area
    screen.setCursor(60, 110);
    snprintf(buffer, sizeof(buffer), "%02d:%02d", minutes, seconds);
    screen.print(buffer);
}

void checkNutrientDispense()
{
    DateTime now = rtc.now();
    if (tdsValue < TDS_LOW_LIMIT - 200) {
        return;
    }

    // 1. If TDS is at or above the high limit:
    if (tdsValue >= TDS_HIGH_LIMIT) {
        // If dosing is currently in progress, stop it.
        if (nutrientDispenseInProgress) {
            digitalWrite(NUTRIENT_PUMP1_PIN, HIGH);
            digitalWrite(NUTRIENT_PUMP2_PIN, HIGH);
            nutrientDispenseInProgress = false;
            lastDosingEndTime = now; // record when dosing stopped
            sendPumpData("Nutrient", false);
            Serial.println("TDS reached high limit. Stopping dosing.");
        }
        // Set the dosing lock so that dosing will not start.
        nutrientDosingDone = true;
        return;
    }

    // 2. Enforce the cooldown period (if a dosing event has just finished).
    if (nutrientDosingDone) {
        // If last dosing end time is valid and the cooldown has not yet expired:
        if (lastDosingEndTime.unixtime() > 0 && (now.unixtime() - lastDosingEndTime.unixtime()) < COOLDOWN_PERIOD_SEC) {
            // Still in cooldown—do nothing.
            return;
        } else {
            // Cooldown period has passed; clear the dosing lock.
            nutrientDosingDone = false;
        }
    }

    // 3. If TDS is below the low limit and dosing is not in progress, start dosing.
    if (tdsValue < TDS_LOW_LIMIT && !nutrientDispenseInProgress) {
        // Start both nutrient pumps.
        digitalWrite(NUTRIENT_PUMP1_PIN, LOW);
        digitalWrite(NUTRIENT_PUMP2_PIN, LOW);
        nutrientDispenseStartTimeRtc = now; // record the start time via the RTC
        nutrientDispenseInProgress = true;
        sendPumpData("Nutrient", true);
        Serial.println("Starting nutrient dosing...");
    }

    // 4. If dosing is in progress, check if the dosing duration (14 seconds) has elapsed.
    if (nutrientDispenseInProgress) {
        long elapsedSec = now.unixtime() - nutrientDispenseStartTimeRtc.unixtime();
        if (elapsedSec >= NUTRIENT_DISPENSE_DURATION_SEC) {
            // Stop both pumps.
            digitalWrite(NUTRIENT_PUMP1_PIN, HIGH);
            digitalWrite(NUTRIENT_PUMP2_PIN, HIGH);
            nutrientDispenseInProgress = false;
            nutrientDosingDone = true; // Set the dosing lock.
            lastDosingEndTime = now; // Record the end time.
            sendPumpData("Nutrient", false);
            Serial.println("Nutrient dosing complete.");
        }
    }
}

float alsToLux(uint16_t alsRawValue)
{
    // Example: When using a gain of 9 and 18-bit resolution, one possible conversion:
    float conversionFactor = 0.6; // (lux per raw count) -- adjust as needed.
    return alsRawValue * conversionFactor;
}

float uvsToUvIndex(uint16_t uvsRawValue)
{
    float conversionFactor = 0.001461; // (UV index per raw count) -- adjust as needed.
    return uvsRawValue * conversionFactor;
}
