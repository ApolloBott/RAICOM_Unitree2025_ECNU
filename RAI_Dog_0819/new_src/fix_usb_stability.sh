#!/bin/bash
# USB串口设备稳定性优化脚本

echo "=== USB串口设备稳定性优化 ==="

# 1. 禁用USB自动挂起
echo "1. 禁用USB自动挂起..."
echo -1 | sudo tee /sys/module/usbcore/parameters/autosuspend > /dev/null
echo "   ✓ USB自动挂起已禁用"

# 2. 增加USB缓冲区大小
echo "2. 优化USB缓冲区..."
echo 16384 | sudo tee /sys/module/usbserial/parameters/buffer_size > /dev/null 2>&1
echo "   ✓ USB串口缓冲区已优化"

# 3. 设置串口设备权限
echo "3. 设置串口设备权限..."
for device in /dev/arm /dev/detector0 /dev/detector1 /dev/detector2 /dev/right; do
    if [ -e "$device" ]; then
        sudo chmod 666 "$device"
        echo "   ✓ $device 权限已设置"
    else
        echo "   - $device 不存在"
    fi
done

# 4. 重新加载CH341驱动
echo "4. 重新加载CH341驱动..."
sudo modprobe -r ch341 2>/dev/null
sleep 2
sudo modprobe ch341
echo "   ✓ CH341驱动已重新加载"

# 5. 检查USB设备状态
echo "5. 检查USB设备状态..."
lsusb | grep -i ch34 && echo "   ✓ 检测到CH341设备" || echo "   - 未检测到CH341设备"

# 6. 创建udev规则以保证设备稳定性
echo "6. 创建udev规则..."
sudo tee /etc/udev/rules.d/99-usb-serial-stable.rules > /dev/null << 'EOF'
# USB串口设备稳定性规则
ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", RUN+="/bin/sh -c 'echo 0 > /sys/bus/usb/devices/%k/power/autosuspend_delay_ms'"
ACTION=="add", SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", GROUP="dialout"
EOF
echo "   ✓ udev规则已创建"

# 7. 重新加载udev规则
echo "7. 重新加载udev规则..."
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "   ✓ udev规则已重新加载"

echo ""
echo "=== 优化完成 ==="
echo "建议:"
echo "1. 重新插拔USB设备"
echo "2. 运行 python3 usb_monitor.py 检查设备状态"
echo "3. 如果问题持续，考虑更换USB线缆或端口"
