# 湿度控制系统

一个基于ESP32的智能湿度控制系统，包含硬件控制、Web前端界面和自动化控制逻辑。

## 项目特性

- 🌡️ **实时监控**: DHT22传感器实时采集温湿度数据
- 🎛️ **智能控制**: 自动控制加湿器/除湿器维持目标湿度
- 📊 **Web界面**: React前端提供直观的数据展示和控制界面
- 📱 **响应式设计**: 支持桌面和移动设备访问
- 🔧 **可扩展**: 模块化设计，易于扩展新功能

## 项目结构

```
humidity-control-systems/
├── firmware/                # ESP32固件代码
│   ├── main.cpp                # 主固件（含WiFi和MQTT）
│   ├── test_firmware.cpp       # 测试固件（仅串口）
│   └── test_firmware_with_relay.cpp  # 测试固件（含继电器）
├── web/                     # React Web前端
│   ├── src/                     # 前端源代码
│   │   ├── components/          # React组件
│   │   ├── services/            # API服务
│   │   └── ...
│   └── package.json
├── docs/                   # 项目文档
├── scripts/                # 启动脚本
└── platformio.ini          # PlatformIO配置
```

## 快速开始

### 硬件准备

1. **ESP32-WROOM-32开发板**
2. **DHT22温湿度传感器**
3. **继电器模块**
4. **面包板和连接线**
5. **10KΩ电阻**

### 硬件连接

参考 `docs/Assembly_Guide.md` 进行硬件连接。

### 固件烧录

```bash
# 测试固件（仅串口输出）
pio run -e test_firmware --target upload

# 测试固件（含继电器控制）
pio run -e test_firmware_with_relay --target upload

# 完整固件（含WiFi和MQTT）
pio run -e esp32dev --target upload
```

### Web前端启动

```bash
# Windows
scripts/start_web.bat

# Linux/Mac
chmod +x scripts/start_web.sh
./scripts/start_web.sh
```

访问 http://localhost:3000 查看Web界面。

## 开发进度

- ✅ **硬件层**: ESP32固件开发完成
- ✅ **Web前端**: React界面开发完成
- 🚧 **后端服务**: 待开发
- 🚧 **数据库**: 待开发
- 🚧 **系统集成**: 待完成

## 技术栈

### 硬件
- **ESP32-WROOM-32**: 主控制器
- **DHT22**: 温湿度传感器
- **继电器模块**: 设备控制
- **PlatformIO**: 开发框架

### 前端
- **React 18**: 前端框架
- **Ant Design**: UI组件库
- **Chart.js**: 数据可视化
- **Axios**: HTTP客户端

## 贡献

欢迎提交Issue和Pull Request来改进项目。

## 许可证

MIT License