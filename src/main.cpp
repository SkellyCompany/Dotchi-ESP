#include <Arduino.h>
#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define DHTPIN 0
#define DHTTYPE DHT11
#define PHOTORESISTORPIN 36
#define SOUNDSENSORPIN 39

// MARK: Constants
// const char* networkName = "+slatt*!";
// const char* networkPassword = "2WBSqd6Q4@mb6wb8Mqs^";
const char* networkName = "EASV-IoT";
const char* networkPassword = "Stewardesse";
const char* mqttServer = "mqtt.flespi.io";
const int mqttPort = 1883;
const char* mqttUser = "E5Re8Mn5WxlTbv6FLUIY2Vnk8DOnrYjTRZXQB9JlwU5vgZ0PJyDBjQKfimsxaAgS";
const char* mqttPassword = "";

const char* temperatureMetricTopic = "temperature";
const char* humidityMetricTopic = "humidity";
const char* lightMetricTopic = "lightIntensity";
const char* soundMetricTopic = "soundIntensity";

const int serialDelay = 200;
const int startupDelay = 2000;
const int retryWifiConnectionDelay = 1000;
const int retryMqttConnectionDelay = 1000;

const int maxAnalogValue = 4095;
const int minAnalogValue = 0;
const int maxValue = 100;
const int minValue = 0;

const int temperatureIterations = 100, temperatureIterationDelay = 5;
const int humidityIterations = 100, humidityIterationDelay = 5;
const int lightIntensityIterations = 100, lightIntensityIterationDelay = 5;
const int soundIntensityIterations = 100, soundIntensityIterationDelay = 5;

const int temperatureDeltaTreshold = 1;
const int humidityDeltaTreshold = 1;
const int lightIntensityDeltaTreshold = 1;
const int soundIntensityDeltaTreshold = 1;

// MARK: Sensor values
float temperatureLastUpdateValue = -1000, humidityLastUpdateValue = -1000, lightIntensityLastUpdateValue = -1000, soundIntensityLastUpdateValue = -1000;
float temperature, humidity, lightIntensity, soundIntensity;

// MARK: Variables
DHT dht(DHTPIN, DHTTYPE);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

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

// MARK: Event functions
void receivedMessage(char* topic, byte* message, unsigned int length) {
  Serial.print("📣 Received message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
  }
}

// MARK: Setup
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
      Serial.print("ⓧ Connecting to MQTT failed. rc=");
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

// MARK: Reconnecting WiFi & MQTT
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
      Serial.print("ⓧ Reconnecting to MQTT failed. rc=");
      Serial.println(mqttClient.state());
      delay(retryMqttConnectionDelay);
    }
  }
}

void reconnectIfNeeded() {
  reconnectWiFiIfNeeded();
  reconnectMQTTIfNeeded();
}

// MARK: Reading metrics
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

void readLightIntensity() {
  float sumLightIntensity = 0;
  for (int i = 0; i < lightIntensityIterations; i++) {
    sumLightIntensity += normalizeAnalogValue(invertAnalogValue(analogRead(PHOTORESISTORPIN)));
    delay(lightIntensityIterationDelay);
  }
  lightIntensity = sumLightIntensity / lightIntensityIterations;
}

void readSoundIntensity() {
  float sumSoundIntensity = 0;
  for (int i = 0; i < soundIntensityIterations; i++) {
    sumSoundIntensity += normalizeAnalogValue(analogRead(SOUNDSENSORPIN));
    delay(soundIntensityIterationDelay);
  }
  soundIntensity = sumSoundIntensity / soundIntensityIterations;
}

void readMetrics() {
  readTemperature();
  readHumidity();
  readLightIntensity();
  readSoundIntensity();
}

// MARK: Sending metrics
bool sendMetric(String topic, String message) {
  if (mqttClient.publish( topic.c_str(), message.c_str())) {
    Serial.println("✔️ Metric published. Topic: " + topic + ". Message: " + message);
    return true;
  } else {
    Serial.println("ⓧ Metric publishing failed. Topic: " + topic + ". Message: " + message);
    return false;
  }
}

void sendTemperatureMetricsIfNeeded() {
  float temperatureDelta = temperatureLastUpdateValue > temperature ? temperatureLastUpdateValue - temperature : temperature - temperatureLastUpdateValue;
  if(temperatureDelta >= temperatureDeltaTreshold) {
    String topic = String(temperatureMetricTopic);
    String message = String(temperature);
    if(sendMetric(topic, message)) {
      temperatureLastUpdateValue = temperature;
    }
  }
}

void sendHumidityMetricsIfNeeded() {
  float humidityDelta = humidityLastUpdateValue > humidity ? humidityLastUpdateValue - humidity : humidity - humidityLastUpdateValue;
  if(humidityDelta >= humidityDeltaTreshold) {
    String topic = String(humidityMetricTopic);
    String message = String(humidity);
    if(sendMetric(topic, message)) {
      humidityLastUpdateValue = humidity;
    }
  }
}

void sendLightMetricsIfNeeded() {
  float lightDelta = lightIntensityLastUpdateValue > lightIntensity ? lightIntensityLastUpdateValue - lightIntensity : lightIntensity - lightIntensityLastUpdateValue;
  if(lightDelta >= lightIntensityDeltaTreshold) {
    String topic = String(lightMetricTopic);
    String message = String(lightIntensity);
    if(sendMetric(topic, message)) {
      lightIntensityLastUpdateValue = lightIntensity;
    }
  }
}

void sendSoundMetricsIfNeeded() {
  float soundDelta = soundIntensityLastUpdateValue > soundIntensity ? soundIntensityLastUpdateValue - soundIntensity : soundIntensity - soundIntensityLastUpdateValue;
  if(soundDelta >= soundIntensityDeltaTreshold) {
    String topic = String(soundMetricTopic);
    String message = String(soundIntensity);
    if(sendMetric(topic, message)) {
      soundIntensityLastUpdateValue = soundIntensity;
    }
  }
}

void sendMetricsIfNeeded() {
  sendTemperatureMetricsIfNeeded();
  sendHumidityMetricsIfNeeded();
  sendLightMetricsIfNeeded();
  sendSoundMetricsIfNeeded();
}

// MARK: Loop
void loop() {
  // put your main code here, to run repeatedly:
  reconnectIfNeeded();
  readMetrics();
  sendMetricsIfNeeded();
}
