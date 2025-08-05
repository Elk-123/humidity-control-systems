@echo off
echo ========================================
echo 湿度控制系统 - 测试固件烧录脚本
echo ========================================
echo.

echo 正在编译测试固件...
pio run -e test_firmware

if %errorlevel% neq 0 (
    echo 编译失败！请检查代码和依赖。
    pause
    exit /b 1
)

echo.
echo 正在烧录固件到ESP32...
pio run -e test_firmware --target upload

if %errorlevel% neq 0 (
    echo 烧录失败！请检查USB连接和ESP32状态。
    pause
    exit /b 1
)

echo.
echo 固件烧录成功！
echo 正在启动串口监控...
echo.
echo 按 Ctrl+C 退出监控
echo.

pio device monitor -e test_firmware 