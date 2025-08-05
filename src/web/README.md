# 湿度控制系统 - Web前端

这是湿度控制系统的Web前端界面，基于React开发，提供实时数据展示和设备控制功能。

## 功能特性

- 🌡️ **实时数据展示**: 显示当前温度、湿度和目标湿度
- 📊 **历史数据图表**: 温湿度趋势图表
- 🎛️ **设备控制**: 手动控制加湿器和除湿器
- ⚙️ **参数设置**: 调整目标湿度等参数
- 📱 **响应式设计**: 支持桌面和移动设备

## 技术栈

- **React 18**: 前端框架
- **Ant Design**: UI组件库
- **Chart.js**: 数据可视化
- **Axios**: HTTP客户端

## 快速开始

### 环境要求

- Node.js 16+ 
- npm 8+

### 安装依赖

```bash
cd src/web
npm install
```

### 启动开发服务器

```bash
npm start
```

应用将在 http://localhost:3000 启动

### 构建生产版本

```bash
npm run build
```

## 项目结构

```
src/web/
├── public/                 # 静态资源
├── src/
│   ├── components/         # React组件
│   │   ├── SensorDisplay.js    # 传感器数据显示
│   │   ├── ControlPanel.js     # 控制面板
│   │   ├── DeviceStatus.js     # 设备状态
│   │   └── DataChart.js        # 数据图表
│   ├── services/           # API服务
│   │   └── api.js
│   ├── App.js             # 主应用组件
│   ├── index.js           # 应用入口
│   └── index.css          # 全局样式
├── package.json           # 项目配置
└── README.md             # 项目说明
```

## 组件说明

### SensorDisplay
显示当前温度、湿度和目标湿度的实时数据。

### ControlPanel
提供目标湿度设置功能，包含滑块控制和保存按钮。

### DeviceStatus
显示设备状态并提供手动控制功能，包括加湿器和除湿器的开关控制。

### DataChart
使用Chart.js绘制温湿度历史趋势图表。

## API接口

Web前端通过以下API与后端通信：

- `GET /api/sensor/data` - 获取传感器数据
- `POST /api/control/target-humidity` - 设置目标湿度
- `POST /api/control/device` - 控制设备
- `GET /api/device/status` - 获取设备状态
- `GET /api/data/history` - 获取历史数据

## 开发说明

### 模拟数据
当前版本使用模拟数据进行演示，数据每2秒更新一次。

### 样式定制
主要样式在 `src/index.css` 中定义，使用CSS Grid和Flexbox布局。

### 响应式设计
支持桌面和移动设备，使用媒体查询适配不同屏幕尺寸。

## 部署

### 开发环境
```bash
npm start
```

### 生产环境
```bash
npm run build
```

构建后的文件在 `build/` 目录中，可以部署到任何静态文件服务器。

## 故障排除

### 常见问题

1. **依赖安装失败**
   - 检查Node.js版本
   - 清除npm缓存: `npm cache clean --force`
   - 删除node_modules重新安装

2. **端口被占用**
   - 修改端口: `PORT=3001 npm start`
   - 或停止占用端口的进程

3. **API连接失败**
   - 检查后端服务是否启动
   - 确认API地址配置正确

## 贡献

欢迎提交Issue和Pull Request来改进项目。

## 许可证

MIT License 