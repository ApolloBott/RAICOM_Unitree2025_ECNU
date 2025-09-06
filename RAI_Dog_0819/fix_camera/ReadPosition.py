import time
from Board import setBusServoPulse
import readchar
import serial

DURATION_MS = 500
servo_ids = list(range(1, 7))  # 控制 ID 1~6 的舵机
current_index = 0

ser2 = serial.Serial('/dev/arm', 115200, timeout=1)
# 初始位置预设（根据ID顺序 1~6）
initial_positions = [500, 500, 300, 500, 500, 0]

def control_servo(servo_id):
    while True:
        val = input(f"[舵机 ID={servo_id}] 请输入目标脉宽 (0-1000)，或按回车跳过：")
        if val == '':
            print("跳过当前舵机控制。")
            return
        try:
            pulse = int(val)
            if 0 <= pulse <= 1000:
                setBusServoPulse(servo_id, pulse, DURATION_MS, ser2)
                print(f"✅ 舵机 {servo_id} 移动至脉宽 {pulse}")
                time.sleep(DURATION_MS / 1000 + 0.2)
                return
            else:
                print("❌ 请输入 0–1000 范围内的数值。")
        except ValueError:
            print("❌ 输入无效，请输入整数或回车。")

def move_all_to_initial():
    print("\n🔄 所有舵机回到初始位置...")
    for sid, pos in zip(servo_ids, initial_positions):
        print(f"→ 舵机 {sid} 移动至 {pos}")
        setBusServoPulse(sid, pos, DURATION_MS, ser2)
        time.sleep(DURATION_MS / 1000 + 0.1)
    print("✅ 所有舵机已复位完成。\n")

def main():
    global current_index
    print("🧠 舵机交互控制程序（↑ / ↓ 切换舵机，输入脉宽控制位置）")
    print("🔁 按 b 回初始位置，按 q 退出程序。\n")

    while True:
        sid = servo_ids[current_index]
        print(f"\n➡ 当前控制舵机：ID = {sid}")
        control_servo(sid)

        print("按 ↑ / ↓ 切换舵机，b 回初始位置，q 退出...")
        key = readchar.readkey()

        if key == readchar.key.UP:
            current_index = (current_index - 1) % len(servo_ids)
        elif key == readchar.key.DOWN:
            current_index = (current_index + 1) % len(servo_ids)
        elif key == 'b':
            move_all_to_initial()
        elif key == 'q':
            print("👋 已退出程序。")
            break
        else:
            print("⚠ 未识别按键，请使用 ↑ / ↓ 或 b / q。")

if __name__ == '__main__':
    main()
