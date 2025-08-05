#!/bin/bash

echo "========================================"
echo "湿度控制系统 - Web前端启动脚本"
echo "========================================"
echo

echo "正在进入Web目录..."
cd src/web

echo "正在安装依赖..."
npm install

if [ $? -ne 0 ]; then
    echo "依赖安装失败！"
    exit 1
fi

echo
echo "依赖安装完成！"
echo "正在启动Web应用..."
echo
echo "Web应用将在浏览器中自动打开"
echo "地址: http://localhost:3000"
echo
echo "按 Ctrl+C 停止应用"
echo

npm start 