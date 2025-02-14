#!/usr/bin/env python3
import sys
import subprocess
import time
import math

def get_value_from_model(command, target_index):
    """
    Chạy lệnh gz model và trích xuất giá trị cần (target_index: 0->roll, 1->pitch, 2->yaw)
    từ dòng chứa RPY của block "Pose [ XYZ (m) ] [ RPY (rad) ]:" (loại trừ phần "Inertial").
    """
    try:
        result = subprocess.run(command, stdout=subprocess.PIPE, text=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Lỗi khi chạy lệnh: {' '.join(command)}")
        return None

    output = result.stdout
    lines = output.splitlines()
    for i, line in enumerate(lines):
        if "Pose [ XYZ (m) ] [ RPY (rad) ]:" in line and "Inertial" not in line:
            # Dòng sau chứa tọa độ, dòng kế tiếp chứa các giá trị RPY
            if i + 2 < len(lines):
                rpy_line = lines[i + 2].strip().strip("[]")
                parts = rpy_line.split()
                if len(parts) >= 3:
                    try:
                        value = float(parts[target_index])
                        return value
                    except ValueError:
                        print("Không chuyển đổi được giá trị RPY sang số thực.")
                        return None
    return None

def get_pitch():
    # Lấy pitch từ link gun: pitch ở vị trí index 1 của RPY
    cmd = ["gz", "model", "-m", "warship", "--link", "gun"]
    return get_value_from_model(cmd, target_index=1)

def get_yaw():
    # Lấy yaw từ link gr: yaw ở vị trí index 2 của RPY
    cmd = ["gz", "model", "-m", "warship", "--link", "trusted"]
    return get_value_from_model(cmd, target_index=2)

def publish_command(topic, data_value):
    """
    Gửi lệnh publish đến topic đã cho với giá trị data_value.
    """
    cmd = ["gz", "topic", "-t", topic, "-m", "gz.msgs.Double", "-p", f"data: {data_value}"]
    print(f"Thực thi lệnh: {' '.join(cmd)}")
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError:
        print(f"Lỗi khi gửi lệnh cho topic {topic}")

def adjust_axis(get_current, topic, target_value, axis_name, tolerance=0.5):
    """
    Điều chỉnh giá trị của một trục cho đến khi đạt được giá trị mục tiêu.
    - get_current: hàm lấy giá trị hiện tại (ví dụ get_yaw, get_pitch)
    - topic: topic cần publish lệnh ("/gr" cho yaw, "/gun" cho pitch)
    - target_value: giá trị mục tiêu cho trục đó
    - axis_name: tên trục (cho mục đích hiển thị)
    """
    print(f"--- Điều chỉnh {axis_name} đến mục tiêu: {target_value} ---")
    while True:
        current_value = get_current()
        if current_value is None:
            print(f"Không lấy được giá trị hiện tại của {axis_name}. Thử lại")

            continue

        diff = target_value - current_value
        print(f"{axis_name} hiện tại = {current_value:.6f} | Hiệu lệch = {diff:.6f}")

        if abs(diff) < tolerance:
            # Nếu đã đạt mục tiêu, gửi lệnh dừng (data = 0) và thoát vòng lặp
            publish_command(topic, 0.0)
            print(f"{axis_name} đã đạt mục tiêu.")
            break
        else:
            data = 1.0 if diff > 0 else -1.0
            publish_command(topic, data)



def main():
    if len(sys.argv) != 3:
        print("Usage: python3 rot.py [target_yaw] [target_pitch]")
        sys.exit(1)
    try:
        target_yaw = float(sys.argv[1])
        target_pitch = float(sys.argv[2])
    except ValueError:
        print("Vui lòng nhập 2 giá trị số cho target_yaw và target_pitch.")
        sys.exit(1)

	
    print(f"Target: yaw = {target_yaw}, pitch = {target_pitch}")
    publish_command("/gr",0)
    publish_command("/gun",0)
    # Điều chỉnh yaw (trục gr) trước
        # Sau khi yaw đạt mục tiêu, điều chỉnh pitch (trục gun)
    adjust_axis(get_pitch, "/gun", target_pitch, "Pitch")
    adjust_axis(get_yaw, "/gr", target_yaw, "Yaw")



if __name__ == '__main__':
    main()

