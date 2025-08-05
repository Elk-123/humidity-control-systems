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
  Serial.println("正在连接WiFi...");
  int wifiTimeout = 0;
  while (WiFi.status() != WL_CONNECTED && wifiTimeout < 20) {
    delay(500);
    Serial.print(".");
    wifiTimeout++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi连接成功!");
    Serial.print("IP地址: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi连接失败，继续运行本地模式");
  }

  // 连接MQTT服务器
  if (WiFi.status() == WL_CONNECTED) {
    client.setServer(mqtt_server, 1883);
    Serial.println("正在连接MQTT服务器...");
    int mqttTimeout = 0;
    while (!client.connected() && mqttTimeout < 10) {
      if (client.connect("ESP32_Humidity_Controller")) {
        Serial.println("MQTT连接成功!");
      } else {
        Serial.print("MQTT连接失败，错误码: ");
        Serial.println(client.state());
        delay(500);
        mqttTimeout++;
      }
    }
    
    if (!client.connected()) {
      Serial.println("MQTT连接超时，将无法发送数据");
    }
  }
}

void loop() {
  // 检查WiFi连接状态
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi连接断开，尝试重连...");
    WiFi.reconnect();
    delay(1000);
  }
  
  // 维护MQTT连接
  if (WiFi.status() == WL_CONNECTED && !client.connected()) {
    Serial.println("重新连接MQTT...");
    if (client.connect("ESP32_Humidity_Controller")) {
      Serial.println("MQTT重新连接成功");
    }
  }
  
  // 读取温湿度数据
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  // 检查传感器数据是否有效
  if (!isnan(humidity) && !isnan(temperature)) {
    Serial.printf("湿度: %.2f%%, 温度: %.2f°C\n", humidity, temperature);
    
    Input = humidity;
    myPID.Compute();

    // 根据PID输出控制继电器
    if (Output > 50) {
      digitalWrite(RELAY_PIN, HIGH);  // 打开加湿器
      digitalWrite(LED_PIN, HIGH);    // 打开指示灯
      Serial.println("加湿器已开启");
    } else {
      digitalWrite(RELAY_PIN, LOW);   // 关闭加湿器
      digitalWrite(LED_PIN, LOW);     // 关闭指示灯
      Serial.println("加湿器已关闭");
    }

    // 发布数据到MQTT
    if (client.connected()) {
      char msg[50];
      snprintf(msg, sizeof(msg), "{\"humidity\":%.2f,\"temperature\":%.2f}", humidity, temperature);
      if (client.publish("humidity_control/data", msg)) {
        Serial.println("数据发送成功");
      } else {
        Serial.println("数据发送失败");
      }
    }
  } else {
    Serial.println("传感器读取失败，请检查连接");
  }

  delay(2000);
}