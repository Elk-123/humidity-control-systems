#!/bin/bash

echo "========================================"
echo "湿度控制系统 - 测试固件烧录脚本"
echo "========================================"
echo

echo "正在编译测试固件..."
pio run -e test_firmware

if [ $? -ne 0 ]; then
    echo "编译失败！请检查代码和依赖。"
    exit 1
fi

echo
echo "正在烧录固件到ESP32..."
pio run -e test_firmware --target upload

if [ $? -ne 0 ]; then
    echo "烧录失败！请检查USB连接和ESP32状态。"
    exit 1
fi

echo
echo "固件烧录成功！"
echo "正在启动串口监控..."
echo
echo "按 Ctrl+C 退出监控"
echo

pio device monitor -e test_firmware 