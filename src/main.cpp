#include <Arduino.h>
#include <DHT.h>
#include <WiFi.h>

#define DHTPIN 0
#define DHTTYPE DHT11
#define PHOTORESISTORPIN 36
#define SOUNDSENSORPIN 39

// MARK: Constants
const int serialDelay = 200;
const int startupDelay = 2000;

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


// MARK: Variables
DHT dht(DHTPIN, DHTTYPE);
float lastRecordedTemperature, lastRecordedHumidity, lastRecordedLight, lastRecorderSound;
float temperature, humidity, light, sound;

// MARK: Event functions
void reconnectWiFi(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.print("⚠ WiFi lost connection. Reason: ");
  Serial.println(info.disconnected.reason);
  if ((WiFi.status() != WL_CONNECTED)) {
    Serial.print(millis());
    Serial.println("• Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print('.');
      delay(1000);
    }
    Serial.println("✔️ Reconnected WiFi");
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
  Serial.print("• Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println();
  Serial.println("✔️ Setup WiFi");
  WiFi.onEvent(reconnectWiFi, SYSTEM_EVENT_STA_DISCONNECTED);
}

void setup() {
  // put your setup code here, to run once:
  setupSerial();
  setupDHT();
  setupWiFi();
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

void loop() {
  // put your main code here, to run repeatedly:
  readTemperature();
  readHumidity();
  readPhotoResistor();
  readSoundSensor();
}
