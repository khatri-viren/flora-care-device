#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_LTR390.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <RTClib.h>
#include "DFRobot_GDL.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ——— Pin / peripheral defines ———
#define SDA_PIN 19  // <— change to your SDA wiring
#define SCL_PIN 20  // <— change to your SCL wiring
#define PHPIN 6
#define TDSPIN 2
#define LED_PIN 15
#define WATER_PUMP_PIN 9
#define WATER_PUMP_BTN 7
#define NUTRIENT_PUMP1_PIN 4
#define NUTRIENT_PUMP2_PIN 5
#define TFT_DC 8
#define TFT_CS 1
#define TFT_RST 14

// ——— Globals & objects ———
Adafruit_AHTX0 aht;
Adafruit_LTR390 ltr;
DFRobot_ST7735_128x160_HW_SPI screen(TFT_DC, TFT_CS, TFT_RST);
RTC_DS3231 rtc;

WiFiClientSecure espClient;
PubSubClient mqtt(espClient);

// Shared sensor & pump state
float temperature = 0, hum = 0, pHValue = 0, tdsValue = 0, ecValue = 0, water_temp = 0, alsLux = 0, uvIndex = 0;
bool pumpState = false;  // false = OFF, true = ON
DateTime lastToggleTime;

// For pH measurement
unsigned long avgValue;
int buf[10], tempBuf;

// For EC/TDS
int ec_tds_buffer[10] = { 0 };

// Timer durations (in minutes / seconds)
const int ON_DURATION = 10;   // water pump ON minutes
const int OFF_DURATION = 20;  // water pump OFF minutes
const unsigned long NUTRIENT_DISPENSE_DURATION_SEC = 14;
const unsigned long COOLDOWN_PERIOD_SEC = 300;

// TDS thresholds
const float TDS_LOW_LIMIT = 700.0;
const float TDS_HIGH_LIMIT = 1000.0;

// Nutrient dosing state
bool nutrientDispenseInProgress = false;
bool nutrientDosingDone = false;
DateTime nutrientDispenseStartTimeRtc;
DateTime lastDosingEndTime;

// MQTT / WiFi
DateTime lastDataSentTime;
const char* mqtt_broker = "rff61f13.ala.asia-southeast1.emqxsl.com";
const int mqtt_port = 8883;
const char* mqtt_user = "test";
const char* mqtt_pass = "hydrobud12345";
const char* mqtt_topic_data = "test/test";
const char* mqtt_topic_cmd = "devices/test/command";
const char* ca_cert = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD
QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB
CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97
nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt
43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P
T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4
gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO
BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR
TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw
DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr
hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg
06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF
PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls
YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk
CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=
-----END CERTIFICATE-----
)EOF";

// ——— Forward declarations ———
void setupHardware();
void setupWiFi();
void reconnectMqtt();
void handleMqttCallback(char* topic, uint8_t* payload, unsigned int length);

void sensorTask(void*);
void displayTask(void*);
void pumpTask(void*);
void mqttTask(void*);

std::pair<float, float> calculateECTDS(int pin, float aref, float dynamicTemp, int buffer[], int bufferSize);
int getMedianNum(int bArray[], int iFilterLen);
float alsToLux(uint16_t alsRawValue);
float uvsToUvIndex(uint16_t uvsRawValue);
void sendData();
void sendPumpData(const char* pumpType, bool status);
void checkNutrientDispense();
void setupDisplay();

void setup() {
  Serial.begin(115200);
  Serial.println(">> setup() start");
  setupHardware();
  setupWiFi();
  espClient.setCACert(ca_cert);
  mqtt.setServer(mqtt_broker, mqtt_port);
  mqtt.setCallback(handleMqttCallback);

  lastToggleTime = rtc.now();

 xTaskCreatePinnedToCore(sensorTask,  "Sensor",  4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(displayTask, "Display", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(pumpTask,    "Pump",    4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(mqttTask,    "MQTT",    8192, NULL, 1, NULL, 0);

  // Terminate Arduino loop task
  vTaskDelete(NULL);
}

void loop() {
  // never runs; all logic is in tasks
}

// ——— Hardware initialization ———
void setupHardware() {
  // Setup I2C with pull‑ups
  Wire.begin(SDA_PIN, SCL_PIN);
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);

  // RTC
  if (!rtc.begin())
    while (1)
      ;
  if (rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // Sensors
  if (!aht.begin())
    while (1) {
      Serial.println("AHT not found");
      delay(500);
    }
  if (!ltr.begin())
    while (1) {
      Serial.println("LTR not found");
      delay(500);
    }
  ltr.setGain(LTR390_GAIN_9);
  ltr.setResolution(LTR390_RESOLUTION_18BIT);

  // Display
  screen.begin();
  screen.fillScreen(COLOR_RGB565_BLACK);
  screen.setTextSize(1);
  screen.setTextColor(COLOR_RGB565_WHITE);
  setupDisplay();

  // Pump pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(WATER_PUMP_PIN, OUTPUT);
  pinMode(WATER_PUMP_BTN, INPUT_PULLUP);
  pinMode(NUTRIENT_PUMP1_PIN, OUTPUT);
  pinMode(NUTRIENT_PUMP2_PIN, OUTPUT);
  digitalWrite(NUTRIENT_PUMP1_PIN, HIGH);
  digitalWrite(NUTRIENT_PUMP2_PIN, HIGH);
  digitalWrite(WATER_PUMP_PIN, LOW);
}

void setupWiFi() {
  WiFi.begin("Airtel_vire_4844", "air53020");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.printf("\nWiFi connected: %s\n", WiFi.localIP().toString().c_str());
}

void reconnectMqtt() {
  while (!mqtt.connected()) {
    Serial.print("MQTT connecting…");
    if (mqtt.connect("ESP32Client", mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      mqtt.subscribe(mqtt_topic_cmd);
    } else {
      Serial.printf("failed, rc=%d\n", mqtt.state());
      delay(2000);
    }
  }
}

// ——— MQTT callback ———
void handleMqttCallback(char* topic, uint8_t* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, msg) != DeserializationError::Ok) return;
  const char* command = doc["command"];
  if (!command) return;

  if (strcmp(command, "nutrient_on") == 0) {
    digitalWrite(NUTRIENT_PUMP1_PIN, LOW);
    digitalWrite(NUTRIENT_PUMP2_PIN, LOW);
    nutrientDispenseInProgress = false;
    nutrientDosingDone = false;
    sendPumpData("Nutrient", true);
  } else if (strcmp(command, "nutrient_off") == 0) {
    digitalWrite(NUTRIENT_PUMP1_PIN, HIGH);
    digitalWrite(NUTRIENT_PUMP2_PIN, HIGH);
    nutrientDispenseInProgress = false;
    nutrientDosingDone = true;
    sendPumpData("Nutrient", false);
  } else if (strcmp(command, "water_on") == 0) {
    digitalWrite(WATER_PUMP_PIN, HIGH);
    pumpState = true;
    lastToggleTime = rtc.now();
    sendPumpData("Water", true);
  } else if (strcmp(command, "water_off") == 0) {
    digitalWrite(WATER_PUMP_PIN, LOW);
    pumpState = false;
    lastToggleTime = rtc.now();
    sendPumpData("Water", false);
  } else if (strcmp(command, "emergency_off") == 0) {
    digitalWrite(WATER_PUMP_PIN, LOW);
    pumpState = false;
    digitalWrite(NUTRIENT_PUMP1_PIN, HIGH);
    digitalWrite(NUTRIENT_PUMP2_PIN, HIGH);
    nutrientDispenseInProgress = false;
    nutrientDosingDone = true;
    lastToggleTime = rtc.now();
    lastDosingEndTime = rtc.now();
    sendPumpData("Water", false);
    sendPumpData("Nutrient", false);
  }
}

// ——— Tasks ———

void sensorTask(void* pv) {
  for (;;) {
    sensors_event_t ev_h, ev_t;
    aht.getEvent(&ev_h, &ev_t);
    hum = ev_h.relative_humidity;
    temperature = ev_t.temperature;

    // pH smoothing
    for (int i = 0; i < 10; i++) {
      buf[i] = analogRead(PHPIN);
      delay(10);
    }
    // sort
    for (int i = 0; i < 9; i++) {
      for (int j = i + 1; j < 10; j++) {
        if (buf[i] > buf[j]) {
          tempBuf = buf[i];
          buf[i] = buf[j];
          buf[j] = tempBuf;
        }
      }
    }
    avgValue = 0;
    for (int i = 2; i < 8; i++) avgValue += buf[i];
    pHValue = ((float)avgValue * 3.3 / 4095) - 1.51;

    // LTR ALS + UVS
    ltr.setMode(LTR390_MODE_ALS);
    uint16_t rawAls = ltr.readALS();
    delay(100);
    ltr.setMode(LTR390_MODE_UVS);
    uint16_t rawUvs = ltr.readUVS();
    alsLux = alsToLux(rawAls);
    uvIndex = uvsToUvIndex(rawUvs);

    // EC/TDS
    auto [ec, tds] = calculateECTDS(TDSPIN, 5.0, temperature, ec_tds_buffer, 10);
    ecValue = ec;
    tdsValue = tds;

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void displayTask(void* pv) {
  char bufStr[32];
  for (;;) {
    // digitalWrite(LED_PIN, HIGH);

    // Temp
    screen.fillRect(90, 20, 40, 8, COLOR_RGB565_BLACK);
    screen.setCursor(90, 20);
    snprintf(bufStr, sizeof(bufStr), "%.2f", temperature);
    screen.print(bufStr);

    // Humidity
    screen.fillRect(90, 30, 40, 8, COLOR_RGB565_BLACK);
    screen.setCursor(90, 30);
    snprintf(bufStr, sizeof(bufStr), "%.2f", hum);
    screen.print(bufStr);

    // pH
    screen.fillRect(90, 40, 40, 8, COLOR_RGB565_BLACK);
    screen.setCursor(90, 40);
    snprintf(bufStr, sizeof(bufStr), "%.2f", pHValue);
    screen.print(bufStr);

    // TDS
    screen.fillRect(90, 50, 40, 8, COLOR_RGB565_BLACK);
    screen.setCursor(90, 50);
    snprintf(bufStr, sizeof(bufStr), "%.2f", tdsValue);
    screen.print(bufStr);

    // EC
    screen.fillRect(90, 60, 40, 8, COLOR_RGB565_BLACK);
    screen.setCursor(90, 60);
    snprintf(bufStr, sizeof(bufStr), "%.2f", ecValue);
    screen.print(bufStr);

    // Water temp
    screen.fillRect(90, 70, 40, 8, COLOR_RGB565_BLACK);
    screen.setCursor(90, 70);
    snprintf(bufStr, sizeof(bufStr), "%.2f", water_temp);
    screen.print(bufStr);

    // ALS
    screen.fillRect(50, 80, 50, 8, COLOR_RGB565_BLACK);
    screen.setCursor(50, 80);
    snprintf(bufStr, sizeof(bufStr), "%.1f", alsLux);
    screen.print(bufStr);

    // UVS
    screen.fillRect(50, 90, 50, 8, COLOR_RGB565_BLACK);
    screen.setCursor(50, 90);
    snprintf(bufStr, sizeof(bufStr), "%.2f", uvIndex);
    screen.print(bufStr);

    // Pump state
    screen.fillRect(50, 100, 50, 8, COLOR_RGB565_BLACK);
    screen.setCursor(50, 100);
    screen.print(pumpState ? "ON" : "OFF");

    // Elapsed
    auto now = rtc.now();
    int secs = now.unixtime() - lastToggleTime.unixtime();
    screen.fillRect(60, 110, 60, 8, COLOR_RGB565_BLACK);
    screen.setCursor(60, 110);
    snprintf(bufStr, sizeof(bufStr), "%02d:%02d", secs / 60, secs % 60);
    screen.print(bufStr);

    // digitalWrite(LED_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void pumpTask(void* pv) {
  const int ON_SEC = ON_DURATION * 60;
  const int OFF_SEC = OFF_DURATION * 60;
  for (;;) {
    // Button toggle
    static bool lastBtnState = HIGH;
    bool btn = digitalRead(WATER_PUMP_BTN);
    if (lastBtnState == HIGH && btn == LOW) {
      pumpState = !pumpState;
      digitalWrite(WATER_PUMP_PIN, pumpState ? HIGH : LOW);
      lastToggleTime = rtc.now();
      sendPumpData("Water", pumpState);
    }
    lastBtnState = btn;

    // Scheduled water control
    int elapsed = rtc.now().unixtime() - lastToggleTime.unixtime();
    if (pumpState && elapsed >= ON_SEC) {
      digitalWrite(WATER_PUMP_PIN, LOW);
      pumpState = false;
      lastToggleTime = rtc.now();
      sendPumpData("Water", false);
    } else if (!pumpState && elapsed >= OFF_SEC) {
      digitalWrite(WATER_PUMP_PIN, HIGH);
      pumpState = true;
      lastToggleTime = rtc.now();
      sendPumpData("Water", true);
    }

    // Nutrient dosing
    checkNutrientDispense();

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void mqttTask(void* pv) {
  const TickType_t interval = pdMS_TO_TICKS(30000);
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    if (!mqtt.connected()) reconnectMqtt();
    mqtt.loop();

    if (xTaskGetTickCount() - lastWake >= interval) {
      sendData();
      lastWake = xTaskGetTickCount();
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ——— Helper functions ———

std::pair<float, float> calculateECTDS(int pin, float aref, float dynamicTemp, int buffer[], int bufferSize) {
  static unsigned long sampleTime = millis();
  static unsigned long printTime = millis();
  static int bufIndex = 0;

  if (millis() - sampleTime > 40U) {
    sampleTime = millis();
    buffer[bufIndex++] = analogRead(pin);
    if (bufIndex == bufferSize) bufIndex = 0;
  }
  if (millis() - printTime > 800U) {
    printTime = millis();
    int tempBuf2[bufferSize];
    memcpy(tempBuf2, buffer, sizeof(tempBuf2));
    int median = getMedianNum(tempBuf2, bufferSize);
    float voltage = (median * aref) / 4096.0;
    float ec = voltage * 1000.0;
    float tds = (133.42 * pow(voltage, 3) - 255.86 * pow(voltage, 2) + 857.39 * voltage) * 0.5;
    return { ec, tds };
  }
  return { 0.0, 0.0 };
}

int getMedianNum(int bArray[], int len) {
  int bTab[len];
  memcpy(bTab, bArray, sizeof(bTab));
  for (int i = 0; i < len - 1; i++)
    for (int j = 0; j < len - i - 1; j++)
      if (bTab[j] > bTab[j + 1]) {
        int t = bTab[j];
        bTab[j] = bTab[j + 1];
        bTab[j + 1] = t;
      }
  if (len % 2) return bTab[len / 2];
  return (bTab[len / 2] + bTab[len / 2 - 1]) / 2;
}

float alsToLux(uint16_t v) {
  return v * 0.6;
}
float uvsToUvIndex(uint16_t v) {
  return v * 0.001461;
}

void sendData() {
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
  doc["ALSraw"] = (int)ltr.readALS();
  doc["UVS"] = uvIndex;
  doc["UVSraw"] = (int)ltr.readUVS();
  doc["waterTemp"] = water_temp;
  char bufJson[256];
  serializeJson(doc, bufJson);
  mqtt.publish(mqtt_topic_data, bufJson);
}

void sendPumpData(const char* pumpType, bool status) {
  StaticJsonDocument<256> doc;
  doc["userEmail"] = "vsk102002@gmail.com";
  doc["deviceId"] = "test";
  doc["type"] = "pump-log";
  doc["pumpType"] = pumpType;
  doc["status"] = status;
  char bufJson[256];
  serializeJson(doc, bufJson);
  mqtt.publish(mqtt_topic_data, bufJson);
}

void checkNutrientDispense() {
  DateTime now = rtc.now();
  if (tdsValue >= TDS_HIGH_LIMIT) {
    if (nutrientDispenseInProgress) {
      digitalWrite(NUTRIENT_PUMP1_PIN, HIGH);
      digitalWrite(NUTRIENT_PUMP2_PIN, HIGH);
      nutrientDispenseInProgress = false;
      lastDosingEndTime = now;
      sendPumpData("Nutrient", false);
    }
    nutrientDosingDone = true;
    return;
  }
  if (nutrientDosingDone && (now.unixtime() - lastDosingEndTime.unixtime()) < COOLDOWN_PERIOD_SEC) {
    return;
  }
  nutrientDosingDone = false;
  if (tdsValue < TDS_LOW_LIMIT && !nutrientDispenseInProgress) {
    digitalWrite(NUTRIENT_PUMP1_PIN, LOW);
    digitalWrite(NUTRIENT_PUMP2_PIN, LOW);
    nutrientDispenseStartTimeRtc = now;
    nutrientDispenseInProgress = true;
    sendPumpData("Nutrient", true);
  }
  if (nutrientDispenseInProgress && (now.unixtime() - nutrientDispenseStartTimeRtc.unixtime()) >= NUTRIENT_DISPENSE_DURATION_SEC) {
    digitalWrite(NUTRIENT_PUMP1_PIN, HIGH);
    digitalWrite(NUTRIENT_PUMP2_PIN, HIGH);
    nutrientDispenseInProgress = false;
    nutrientDosingDone = true;
    lastDosingEndTime = now;
    sendPumpData("Nutrient", false);
  }
}

void setupDisplay() {
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
  screen.print("ALS:");
  screen.setCursor(10, 90);
  screen.print("UVS:");
  screen.setCursor(10, 100);
  screen.print("Pump:");
  screen.setCursor(10, 110);
  screen.print("Elapsed:");
}
