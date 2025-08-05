#include <Arduino.h>
#include <DHT.h>

// ESP32引脚定义
#define DHT_PIN 4          // DHT22传感器连接引脚
#define RELAY_PIN 5        // 继电器控制引脚
#define LED_PIN 13         // LED指示灯引脚

// 传感器配置
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);

// 控制参数
const float TARGET_HUMIDITY = 60.0;  // 目标湿度60%
const float HUMIDITY_TOLERANCE = 2.0; // 湿度容差±2%
bool relayState = false;             // 继电器状态

// 数据采集间隔（毫秒）
const unsigned long SENSOR_INTERVAL = 2000;  // 2秒读取一次
unsigned long lastSensorRead = 0;

void setup() {
  // 初始化串口通信
  Serial.begin(115200);
  Serial.println("=== 湿度控制系统测试固件 (含继电器) ===");
  Serial.println("正在初始化传感器和继电器...");
  
  // 初始化引脚
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // 继电器初始状态：关闭
  digitalWrite(LED_PIN, LOW);
  
  // 初始化DHT传感器
  dht.begin();
  
  // 等待传感器稳定
  delay(2000);
  Serial.println("初始化完成！");
  Serial.println("开始读取温湿度数据并控制继电器...");
  Serial.println("----------------------------------------");
  Serial.println("时间\t温度(°C)\t湿度(%)\t继电器\t状态");
  Serial.println("----------------------------------------");
}

void loop() {
  unsigned long currentTime = millis();
  
  // 每2秒读取一次传感器数据
  if (currentTime - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = currentTime;
    
    // 读取温湿度数据
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
    
    // 获取当前时间戳
    unsigned long timestamp = currentTime / 1000;  // 转换为秒
    
    // 检查传感器数据是否有效
    if (!isnan(humidity) && !isnan(temperature)) {
      // 湿度控制逻辑
      bool shouldActivate = false;
      String controlReason = "";
      
      if (humidity < TARGET_HUMIDITY - HUMIDITY_TOLERANCE) {
        // 湿度太低，需要加湿
        shouldActivate = true;
        controlReason = "加湿";
      } else if (humidity > TARGET_HUMIDITY + HUMIDITY_TOLERANCE) {
        // 湿度太高，需要除湿
        shouldActivate = true;
        controlReason = "除湿";
      } else {
        // 湿度在目标范围内
        shouldActivate = false;
        controlReason = "正常";
      }
      
      // 控制继电器
      if (shouldActivate && !relayState) {
        digitalWrite(RELAY_PIN, HIGH);
        relayState = true;
        Serial.printf("继电器已开启 - %s\n", controlReason.c_str());
      } else if (!shouldActivate && relayState) {
        digitalWrite(RELAY_PIN, LOW);
        relayState = false;
        Serial.printf("继电器已关闭 - %s\n", controlReason.c_str());
      }
      
      // 打印格式化的数据
      Serial.printf("%lu\t%.1f\t\t%.1f\t\t%s\t%s\n", 
                   timestamp, temperature, humidity, 
                   relayState ? "开启" : "关闭", controlReason.c_str());
      
      // LED指示灯闪烁表示数据正常
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      
    } else {
      // 传感器读取失败
      Serial.printf("%lu\t--\t\t--\t\t%s\t错误\n", 
                   timestamp, relayState ? "开启" : "关闭");
      Serial.println("传感器读取失败，请检查连接！");
      
      // LED快速闪烁表示错误
      for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(200);
        digitalWrite(LED_PIN, LOW);
        delay(200);
      }
    }
  }
  
  // 处理串口命令
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "status") {
      Serial.println("=== 系统状态 ===");
      Serial.printf("传感器类型: DHT22\n");
      Serial.printf("传感器引脚: GPIO%d\n", DHT_PIN);
      Serial.printf("继电器引脚: GPIO%d\n", RELAY_PIN);
      Serial.printf("LED引脚: GPIO%d\n", LED_PIN);
      Serial.printf("目标湿度: %.1f%%\n", TARGET_HUMIDITY);
      Serial.printf("湿度容差: ±%.1f%%\n", HUMIDITY_TOLERANCE);
      Serial.printf("继电器状态: %s\n", relayState ? "开启" : "关闭");
      Serial.println("================");
    }
    else if (command == "read") {
      // 立即读取一次数据
      float h = dht.readHumidity();
      float t = dht.readTemperature();
      
      if (!isnan(h) && !isnan(t)) {
        Serial.printf("立即读取 - 温度: %.1f°C, 湿度: %.1f%%\n", t, h);
      } else {
        Serial.println("立即读取失败！");
      }
    }
    else if (command == "relay_on") {
      // 手动开启继电器
      digitalWrite(RELAY_PIN, HIGH);
      relayState = true;
      Serial.println("继电器已手动开启");
    }
    else if (command == "relay_off") {
      // 手动关闭继电器
      digitalWrite(RELAY_PIN, LOW);
      relayState = false;
      Serial.println("继电器已手动关闭");
    }
    else if (command == "help") {
      Serial.println("=== 可用命令 ===");
      Serial.println("status - 显示系统状态");
      Serial.println("read - 立即读取一次数据");
      Serial.println("relay_on - 手动开启继电器");
      Serial.println("relay_off - 手动关闭继电器");
      Serial.println("help - 显示此帮助信息");
      Serial.println("================");
    }
  }
} 