import rclpy
import os
import subprocess
from rclpy.node import Node
from warship_control.msg import FireCommand  # Import message tùy chỉnh

class FireListener(Node):
    def __init__(self):
        super().__init__('fire_listener')
        topic = f'/fire'
        # Tạo subscriber lắng nghe topic với QoS độ sâu 10
        self.subscription = self.create_subscription(
            FireCommand,
            topic,
            self.fire_callback,
            10)
        self.subscription  # giữ tham chiếu tránh bị GC

    def fire_callback(self, msg):
        # Lấy dữ liệu từ message
        target_ship = msg.target_ship
        target_yaw = msg.target_yaw
        target_pitch = msg.target_pitch
        max_speed = msg.max_speed_rocket

        # Tự tính đường dẫn file (nếu đặt cùng thư mục với file này)
        script_path = os.path.join(os.path.dirname(__file__), 'cannon.py')

        # Log thông tin nhận được (phục vụ kiểm tra)
        self.get_logger().info(
            f'Nhận lệnh bắn: target={target_ship}, yaw={target_yaw}, pitch={target_pitch}, speed={max_speed}')

        # Chuẩn bị tham số gọi script
        args = [target_ship, str(target_yaw), str(target_pitch), str(max_speed)]

        # Gọi script cannon.py
        try:
            subprocess.run(["python3", script_path] + args, check=True)
            self.get_logger().info("Đã gọi cannon.py thành công")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Chạy cannon.py thất bại: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FireListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

