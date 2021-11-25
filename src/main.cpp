#include <Arduino.h>
#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define DHTPIN 0
#define DHTTYPE DHT11
#define PHOTORESISTORPIN 36
#define SOUNDSENSORPIN 39

// MARK: Constants
const int serialDelay = 200;
const int startupDelay = 2000;
const int retryWifiConnectionDelay = 1000;
const int retryMqttConnectionDelay = 1000;

const int maxAnalogValue = 4095;
const int minAnalogValue = 0;
const int maxValue = 100;
const int minValue = 0;

const int temperatureIterations = 100;
const int temperatureIterationDelay = 5;
const int humidityIterations = 100;
const int humidityIterationDelay = 5;
const int photoResistorIterations = 100;
const int photoResistorIterationDelay = 5;
const int soundSensorIterations = 100;
const int soundSensorIterationDelay = 5;

const char* networkName = "+slatt*!";
const char* networkPassword = "2WBSqd6Q4@mb6wb8Mqs^";
const char* mqttServer = "mqtt.flespi.io";
const int mqttPort = 1883;
const char* mqttUser = "GUHtG1woJmnhfbzRcCOTiNe47alwqJfUtMdXBxb9ImgNnEuq667eJc7EagYoP486";
const char* mqttPassword = "";

// MARK: Sensor values
float lastRecordedTemperature, lastRecordedHumidity, lastRecordedLight, lastRecorderSound;
float temperature, humidity, light, sound;

// MARK: Variables
DHT dht(DHTPIN, DHTTYPE);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// MARK: Event functions
void disconnectedWiFi(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.print("⚠ WiFi lost connection. Reason: ");
  Serial.println(info.disconnected.reason);
  if ((WiFi.status() != WL_CONNECTED)) {
    Serial.print(millis());
    Serial.println("• Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print('.');
      delay(retryWifiConnectionDelay);
    }
    Serial.println("✔️ Reconnected WiFi");
  }
}

void receivedMessage(char* topic, byte* message, unsigned int length) {
  Serial.print("📣 Received message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
  }
}

// MARK: Setup functions
void setupSerial() {
  delay(serialDelay);
  Serial.begin(9600);
  delay(startupDelay);
  Serial.println("✔️ Setup Serial");
}

void setupDHT() {
  dht.begin();
  Serial.println("✔️ Setup DHT");
}

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.begin(networkName, networkPassword);
  Serial.print("• Connecting to WiFi ...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(retryWifiConnectionDelay);
  }
  Serial.println();
  Serial.println("✔️ Setup WiFi Connection");
  WiFi.onEvent(disconnectedWiFi, SYSTEM_EVENT_STA_DISCONNECTED);
}

void setupMQTT() {
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(receivedMessage);
  while (!mqttClient.connected()) {
    Serial.println("• Connecting to MQTT Broker ...");
    String clientId = WiFi.macAddress();
    if (mqttClient.connect(clientId.c_str(), mqttUser, mqttPassword)) {
      Serial.println("✔️ Setup MQTT Connection");
    } else {
      Serial.print("⚠ Connecting to MQTT failed. rc=");
      Serial.println(mqttClient.state());
      delay(retryMqttConnectionDelay);
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  setupSerial();
  setupDHT();
  setupWiFi();
  setupMQTT();
}

// MARK: Helper functions
float invertAnalogValue(float analogValue) {
  return maxAnalogValue - analogValue;
}

float normalizeAnalogValue(float analogValue) {
  int value = map(analogValue, minAnalogValue, maxAnalogValue, minValue, maxValue);
  if (value < minValue) {
    return minValue;
  } else if (value > maxValue) {
    return maxValue;
  } else {
    return value;
  }
}

// MARK: DHT
void readTemperature() {
  float sumTemperature = 0;
  for (int i = 0; i < temperatureIterations; i++) {
    sumTemperature += dht.readTemperature();
    delay(temperatureIterationDelay);
  }
  temperature = sumTemperature / temperatureIterations;
}

void readHumidity() {
  float sumHumidity = 0;
  for (int i = 0; i < humidityIterations; i++) {
    sumHumidity += dht.readHumidity();
    delay(humidityIterationDelay);
  }
  humidity = sumHumidity / humidityIterations;
}

// MARK: PhotoResistor
void readPhotoResistor() {
  float sumLight = 0;
  for (int i = 0; i < photoResistorIterations; i++) {
    sumLight += normalizeAnalogValue(invertAnalogValue(analogRead(PHOTORESISTORPIN)));
    delay(photoResistorIterationDelay);
  }
  light = sumLight / photoResistorIterations;
}

// MARK: Sound sensor
void readSoundSensor() {
  float sumSound = 0;
  for (int i = 0; i < soundSensorIterations; i++) {
    sumSound += normalizeAnalogValue(analogRead(SOUNDSENSORPIN));
    delay(soundSensorIterationDelay);
  }
  sound = sumSound / soundSensorIterations;
}

// MARK: WiFi & MQTT
void reconnectWiFiIfNeeded() {
  if ((WiFi.status() != WL_CONNECTED)) {
    Serial.print(millis());
    Serial.println("• Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print('.');
      delay(retryWifiConnectionDelay);
    }
    Serial.println("✔️ Reconnected WiFi");
  }
}

void reconnectMQTTIfNeeded() {
  while (!mqttClient.connected()) {
    Serial.println("• Reconnecting to MQTT Broker ..");
    String clientId = WiFi.macAddress();
    if (mqttClient.connect(clientId.c_str(), mqttUser, mqttPassword)) {
      Serial.println("✔️ Reconnected MQTT Connection.");
    } else {
      Serial.print("⚠ Reconnecting to MQTT failed. rc=");
      Serial.println(mqttClient.state());
      delay(retryMqttConnectionDelay);
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  reconnectWiFiIfNeeded();
  reconnectMQTTIfNeeded();
  readTemperature();
  readHumidity();
  readPhotoResistor();
  readSoundSensor();
}
