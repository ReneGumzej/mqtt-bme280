#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>
#include <MQTT.h>

// WIFI Config
#define WIFI_SSID "FRITZ!Box 7530 PG"
#define WIFI_PASSWD "1Kaffee@W@rk!"

// MQTT Config
#define MQTT_HOST IPAddress(192,168,178,30)
#define MQTT_PORT 1883

// Topic Config
#define MQTT_PUB_TEMP "esp/bme280/temperature"
#define MQTT_PUB_HUM "esp/bme280/humidity"
#define MQTT_PUB_PRESS "esp/bme280/pressure"

// Sensor Config
#define DEEP_SLEEP_DURATION 900

// Sensor init
Adafruit_BME280 bme;
float temp = 0.00;
float hum = 0.00;
float press = 0.00;

WiFiClient espClient;
MQTTClient mqttClient;

unsigned long lastMillis = 0;

void connectToWifi()
{
  WiFi.begin(WIFI_SSID, WIFI_PASSWD);

  while (WiFi.status() != WL_CONNECTED) {
        Serial.println("Not Connected to WiFi... Trying to reconnect");
        delay(1000);
    }

  Serial.println("Connected to WiFi!");  
  Serial.println(WiFi.localIP());
}

void connectToMqttBroker()
{
  mqttClient.begin(MQTT_HOST, espClient);

  Serial.print("\nconnecting to MQTT-Broker ...");
  while (!mqttClient.connect("esp8266", "rene", "root")) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("Connection to MQTT-Broker established!");
  mqttClient.subscribe(MQTT_PUB_TEMP);
  mqttClient.subscribe(MQTT_PUB_HUM);
  mqttClient.subscribe(MQTT_PUB_PRESS);
}

void readSensorData()
{
  mqttClient.loop();
  delay(10);
  temp = bme.readTemperature();
  hum = bme.readHumidity();
  press = bme.readPressure()/100;

  Serial.print("| Temperature: ");
  Serial.print(String(temp));
  
  Serial.print(" | Humidity: ");
  Serial.print(String(hum));
  
  Serial.print(" | Pressure: ");
  Serial.print(String(press));
  Serial.println();
  
  if(!mqttClient.connected())
  {
    Serial.println("No Connection to MQTT-Broker..trying to connect.");
    connectToMqttBroker();
  }

  mqttClient.publish(MQTT_PUB_TEMP, String(temp));
  mqttClient.publish(MQTT_PUB_HUM, String(hum));
  mqttClient.publish(MQTT_PUB_PRESS, String(press));

}
// main
void setup() {
  Serial.begin(9600);
  if(!bme.begin(0x76, &Wire))
  {
    Serial.println("Sensor not found. Please check wiring.");
    while(true);
  }

  connectToWifi();
  connectToMqttBroker();
  readSensorData();
  delay(2000);
  Serial.println("Start Deep Sleep");
  ESP.deepSleep(DEEP_SLEEP_DURATION * 1e6);
} 

void loop() {

}


