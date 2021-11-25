#include <Arduino.h>
#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define DHTPIN 0
#define DHTTYPE DHT11
#define PHOTORESISTORPIN 36
#define SOUNDSENSORPIN 39

// MARK: Constants
const char* networkName = "+slatt*!";
const char* networkPassword = "2WBSqd6Q4@mb6wb8Mqs^";
const char* mqttServer = "mqtt.flespi.io";
const int mqttPort = 1883;
const char* mqttUser = "GUHtG1woJmnhfbzRcCOTiNe47alwqJfUtMdXBxb9ImgNnEuq667eJc7EagYoP486";
const char* mqttPassword = "";

const char* temperatureMetricTopic = "temperature";
const char* humidityMetricTopic = "humidity";
const char* lightMetricTopic = "light";
const char* soundMetricTopic = "sound";

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
const int photoResistorIterations = 100, photoResistorIterationDelay = 5;
const int soundSensorIterations = 100, soundSensorIterationDelay = 5;

const int temperatureDeltaTreshold = 1;
const int humidityDeltaTreshold = 1;
const int lightDeltaTreshold = 1;
const int soundDeltaTreshold = 1;

// MARK: Sensor values
float temperatureLastUpdateValue = -1000, humidityLastUpdateValue = -1000, lightLastUpdateValue = -1000, soundLastUpdateValue = -1000;
float temperature, humidity, light, sound;

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

void readPhotoResistor() {
  float sumLight = 0;
  for (int i = 0; i < photoResistorIterations; i++) {
    sumLight += normalizeAnalogValue(invertAnalogValue(analogRead(PHOTORESISTORPIN)));
    delay(photoResistorIterationDelay);
  }
  light = sumLight / photoResistorIterations;
}

void readSoundSensor() {
  float sumSound = 0;
  for (int i = 0; i < soundSensorIterations; i++) {
    sumSound += normalizeAnalogValue(analogRead(SOUNDSENSORPIN));
    delay(soundSensorIterationDelay);
  }
  sound = sumSound / soundSensorIterations;
}

void readMetrics() {
  readTemperature();
  readHumidity();
  readPhotoResistor();
  readSoundSensor();
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
    String topic = String(temperatureMetricTopic) + "/" + WiFi.macAddress();
    String message = String(temperature);
    if(sendMetric(topic, message)) {
      temperatureLastUpdateValue = temperature;
    }
  }
}

void sendHumidityMetricsIfNeeded() {
  float humidityDelta = humidityLastUpdateValue > humidity ? humidityLastUpdateValue - humidity : humidity - humidityLastUpdateValue;
  if(humidityDelta >= humidityDeltaTreshold) {
    String topic = String(humidityMetricTopic) + "/" + WiFi.macAddress();
    String message = String(humidity);
    if(sendMetric(topic, message)) {
      humidityLastUpdateValue = humidity;
    }
  }
}

void sendLightMetricsIfNeeded() {
  float lightDelta = lightLastUpdateValue > light ? lightLastUpdateValue - light : light - lightLastUpdateValue;
  if(lightDelta >= lightDeltaTreshold) {
    String topic = String(lightMetricTopic) + "/" + WiFi.macAddress();
    String message = String(light);
    if(sendMetric(topic, message)) {
      lightLastUpdateValue = light;
    }
  }
}

void sendSoundMetricsIfNeeded() {
  float soundDelta = soundLastUpdateValue > sound ? soundLastUpdateValue - sound : sound - soundLastUpdateValue;
  if(soundDelta >= soundDeltaTreshold) {
    String topic = String(soundMetricTopic) + "/" + WiFi.macAddress();
    String message = String(sound);
    if(sendMetric(topic, message)) {
      soundLastUpdateValue = sound;
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
