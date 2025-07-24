#include <Arduino.h>
#include <DHT.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ESP32引脚定义
#define DHT_PIN 4          // DHT22传感器连接引脚
#define RELAY_PIN 5        // 继电器模块控制引脚
#define LED_PIN 13         // WS2812B LED控制引脚

// 传感器配置
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);

// PID控制配置
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, 2.0, 5.0, 1.0, DIRECT);

// WiFi和MQTT配置
const char* ssid = "your_wifi_ssid";
const char* password = "your_wifi_password";
const char* mqtt_server = "192.168.1.100";
WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  dht.begin();

  // PID初始化
  Setpoint = 60;  // 设置目标湿度60%
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 100);

  // 连接WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // 连接MQTT服务器
  client.setServer(mqtt_server, 1883);
  while (!client.connected()) {
    if (client.connect("ESP32_Humidity_Controller")) {
      Serial.println("MQTT connected");
    } else {
      delay(500);
    }
  }
}

void loop() {
  // 读取温湿度数据
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (!isnan(humidity)) {
    Input = humidity;
    myPID.Compute();

    // 根据PID输出控制继电器
    if (Output > 50) {
      digitalWrite(RELAY_PIN, HIGH);  // 打开加湿器
      digitalWrite(LED_PIN, HIGH);    // 打开指示灯
    } else {
      digitalWrite(RELAY_PIN, LOW);   // 关闭加湿器
      digitalWrite(LED_PIN, LOW);     // 关闭指示灯
    }

    // 发布数据到MQTT
    char msg[50];
    snprintf(msg, sizeof(msg), "{\"humidity\":%.2f,\"temperature\":%.2f}", humidity, temperature);
    client.publish("humidity_control/data", msg);
  }

  delay(2000);
}