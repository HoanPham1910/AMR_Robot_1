import time
import threading
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Twist
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

from Library.ezi_servo_controller import EziServoController
from Topic_python.joint_states import EncoderJointStatePublisher


class CmdVelSubscriber(Node):
    def __init__(self, motor_left, motor_right):
        super().__init__('cmd_vel_subscriber')
        self.motor_left = motor_left
        self.motor_right = motor_right

        # --- Thông số robot ---
        self.radius = 0.065          # m
        self.wheel_distance = 0.65   # m

        # --- Biến nội bộ ---
        self.prev_rpm_left = 0.0
        self.prev_rpm_right = 0.0
        self.prev_dir_left = None
        self.prev_dir_right = None

        # --- Odometry ---
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # --- ROS publishers ---
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)

        # --- JointState subscription (encoder feedback) ---
        self.create_subscription(JointState, 'joint_states', self.encoder_callback, 10)
        self.last_encoder = None

        self.get_logger().info("✅ CmdVelSubscriber started (with Odometry).")

    # ============================================================
    # 1️⃣ Nhận lệnh vận tốc từ ROS (cmd_vel)
    # ============================================================
    def cmd_callback(self, msg):
        v = msg.linear.x        # tốc độ tiến/lùi (m/s)
        w = msg.angular.z       # tốc độ quay (rad/s)

        # Tính vận tốc bánh trái/phải (m/s)
        v_left = v + (w * self.wheel_distance / 2.0)
        v_right = v - (w * self.wheel_distance / 2.0)

        rpm_left = (v_left / (2 * math.pi * self.radius)) * 60.0
        rpm_right = (v_right / (2 * math.pi * self.radius)) * 60.0

        dir_left = 1 if rpm_left >= 0 else 0
        dir_right = 0 if rpm_right >= 0 else 1

        # --- Motor trái ---
        if self.prev_dir_left != dir_left or abs(self.prev_rpm_left - rpm_left) > 1e-2:
            self.motor_left.move_velocity_rpm(abs(rpm_left), direction=dir_left)
        else:
            self.motor_left.velocity_override_rpm(abs(rpm_left))

        # --- Motor phải ---
        if self.prev_dir_right != dir_right or abs(self.prev_rpm_right - rpm_right) > 1e-2:
            self.motor_right.move_velocity_rpm(abs(rpm_right), direction=dir_right)
        else:
            self.motor_right.velocity_override_rpm(abs(rpm_right))

        # Lưu trạng thái
        self.prev_dir_left = dir_left
        self.prev_dir_right = dir_right
        self.prev_rpm_left = rpm_left
        self.prev_rpm_right = rpm_right

    # ============================================================
    # 2️⃣ Callback encoder → tính ODOM & TF
    # ============================================================
    def encoder_callback(self, msg):
        # Giả sử joint[0]=left, joint[1]=right (đơn vị: rad hoặc tick quy đổi)
        if self.last_encoder is None:
            self.last_encoder = msg.position
            self.last_time = self.get_clock().now()
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt == 0:
            return

        d_left = (msg.position[0] - self.last_encoder[0]) * self.radius
        d_right = (msg.position[1] - self.last_encoder[1]) * self.radius
        self.last_encoder = msg.position
        self.last_time = current_time

        d = (d_left + d_right) / 2.0
        dth = (d_right - d_left) / self.wheel_distance

        self.x += d * math.cos(self.th + dth / 2.0)
        self.y += d * math.sin(self.th + dth / 2.0)
        self.th += dth

        # --- Publish Odometry ---
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        q = quaternion_from_euler(0, 0, self.th)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        self.odom_pub.publish(odom_msg)

        # --- TF broadcast ---
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)


# ==================== MAIN ====================
def main():
    rclpy.init()

    # --- Kết nối EziServo ---
    motor_left = EziServoController("192.168.0.2", 2002)
    motor_right = EziServoController("192.168.0.7", 2002)
    motor_left.set_servo_enable(True)
    motor_right.set_servo_enable(True)
    motor_left.stop()
    motor_right.stop()
    motor_left.clear_position()
    motor_right.clear_position()

    # --- ROS node ---
    cmd_node = CmdVelSubscriber(motor_left, motor_right)
    encoder_node = EncoderJointStatePublisher(motor_left, motor_right)

    executor = MultiThreadedExecutor()
    executor.add_node(cmd_node)
    executor.add_node(encoder_node)

    try:
        print("⚙️ Running Velocity + Odometry + TF mode")
        executor.spin()
    except KeyboardInterrupt:
        print("\n🛑 Dừng chương trình")
    finally:
        motor_left.stop()
        motor_right.stop()
        motor_left.close()
        motor_right.close()
        cmd_node.destroy_node()
        encoder_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
