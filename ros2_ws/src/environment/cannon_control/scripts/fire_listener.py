#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cannon_control.msg import FireCommand  # Import message tùy chỉnh
import subprocess
import os

class FireCommandListener(Node):
    def __init__(self):
        super().__init__('fire_command_listener')
        # Tạo subscriber lắng nghe topic '/fire'
        self.subscription = self.create_subscription(
            FireCommand,
            '/fire',
            self.listener_callback,
            10)
        self.get_logger().info("FireCommandListener node đã sẵn sàng.")

    def listener_callback(self, msg):
        # Hàm được gọi mỗi khi có message trên topic /fire
        self.get_logger().info(f"Nhận lệnh bắn: source={msg.source}, yaw={msg.target_yaw}, pitch={msg.target_pitch}, speed={msg.max_speed_rocket}")
        # Xác định đường dẫn tới script cannon.py (giả sử nằm cùng thư mục với file này)
        script_path = os.path.join(os.path.dirname(__file__), 'cannon.py')
        # Gọi chương trình cannon.py với tham số từ message
        try:
            subprocess.run(["python3", script_path, str(msg.source),
                            str(msg.target_yaw), str(msg.target_pitch),
                            str(msg.max_speed_rocket)], check=True)
            self.get_logger().info("Đã gọi cannon.py thành công.")
        except Exception as e:
            self.get_logger().error(f"Lỗi khi gọi cannon.py: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FireCommandListener()
    rclpy.spin(node)  # Giữ cho node chạy và lắng nghe
    node.destroy_node()
    rclpy.shutdown()

