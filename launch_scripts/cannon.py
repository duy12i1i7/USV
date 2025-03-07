#!/usr/bin/env python3
import sys
import subprocess
import time
import math
import re

def transform_warship(target_warship):
    """
    Chuyển đổi từ chuỗi dạng "warship_1" sang "warship1_rocket1".
    """
    # Mẫu: bắt đầu bằng "warship", sau đó dấu gạch dưới và số
    pattern = r"^(warship)_(\d+)$"
    match = re.match(pattern, target_warship)
    if match:
        prefix = match.group(1)      # "warship"
        number = match.group(2)      # "1" (hoặc số khác)
        # Nối lại thành "warship" + number + "_rocket" + number
        return f"{prefix}{number}_rocket{number}"
    else:
        # Nếu không khớp, trả về chuỗi ban đầu
        return target_warship


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

def get_pitch(warship_id):
    # Lấy pitch từ link gun: pitch ở vị trí index 1 của RPY
    cmd = ["gz", "model", "-m", warship_id, "--link", "gun"]
    return get_value_from_model(cmd, target_index=1)

def get_yaw(warship_id):
    # Lấy yaw từ link gr: yaw ở vị trí index 2 của RPY
    cmd = ["gz", "model", "-m", warship_id, "--link", "trusted"]
    return get_value_from_model(cmd, target_index=2)

def publish_command(model, j, data_value):
    """
    Gửi lệnh publish đến topic đã cho với giá trị data_value.
    """
    topic = f"/model/{model}/joint/{j}/cmd_vel"
    # Chuỗi payload dùng để thực thi (không có dấu nháy đơn)
    payload_for_execution = f"data: {data_value}"
    # Chuỗi payload dùng để in ra (có dấu nháy đơn như mong muốn)
    payload_for_print = f"'data: {data_value}'"
    
    # Xây dựng lệnh với payload thực thi không có dấu nháy đơn
    cmd = ["gz", "topic", "-t", topic, "-m", "gz.msgs.Double", "-p", payload_for_execution]
    print(f"Thực thi lệnh: {' '.join(['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Double', '-p', payload_for_print])}")
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError:
        print(f"Lỗi khi gửi lệnh cho topic {topic}")


def adjust_axis(get_current, model, j, target_value, axis_name, tolerance=0.5):
    """
    Điều chỉnh giá trị của một trục cho đến khi đạt được giá trị mục tiêu.
    - get_current: hàm lấy giá trị hiện tại (ví dụ get_yaw, get_pitch)
    - topic: topic cần publish lệnh ("/gr" cho yaw, "/gun" cho pitch)
    - target_value: giá trị mục tiêu cho trục đó
    - axis_name: tên trục (cho mục đích hiển thị)
    """
    print(f"--- Điều chỉnh {axis_name} đến mục tiêu: {target_value} ---")
    while True:
        current_value = get_current(model)
        if current_value is None:
            print(f"Không lấy được giá trị hiện tại của {axis_name}. Thử lại")

            continue

        diff = target_value - current_value
        print(f"{axis_name} hiện tại = {current_value:.6f} | Hiệu lệch = {diff:.6f}")

        if abs(diff) < tolerance:
            # Nếu đã đạt mục tiêu, gửi lệnh dừng (data = 0) và thoát vòng lặp
            publish_command(model, j, 0.0)
            print(f"{axis_name} đã đạt mục tiêu.")
            break
        else:
            data = 1.0 if diff > 0 else -1.0
            publish_command(model, j, data)


def yaw_pitch_to_vector(yaw, pitch):
    # Công thức chuyển đổi cơ bản từ yaw, pitch sang vector đơn vị
    vx = math.cos(pitch) * math.cos(yaw)
    vy = math.cos(pitch) * math.sin(yaw)
    vz = math.sin(pitch)
    return [vx, vy, vz]



def main():
    if len(sys.argv) != 5:
        print("Usage: python3 cannon.py [warship_name] [target_yaw] [target_pitch] [max_speed_rocket]")
        sys.exit(1)
    try:
        target_warship = sys.argv[1]
        target_yaw = float(sys.argv[2])
        target_pitch = float(sys.argv[3])
        maxSpd = float(sys.argv[4])
    except ValueError:
        print("Vui lòng nhập 4 giá trị số cho"+ValueError)
        sys.exit(1)

	
    print(f"Target: yaw = {target_yaw}, pitch = {target_pitch}")
    publish_command(target_warship,"j2",0)
    publish_command(target_warship,"j1",0)
    # Điều chỉnh yaw (trục gr) trước
        # Sau khi yaw đạt mục tiêu, điều chỉnh pitch (trục gun)
    adjust_axis(get_pitch, target_warship, "j1", target_pitch, "Pitch")
    adjust_axis(get_yaw, target_warship, "j2", target_yaw, "Yaw")




if __name__ == '__main__':
    main()
