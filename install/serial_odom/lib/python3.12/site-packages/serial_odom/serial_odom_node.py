import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import re
import math
from rclpy.qos import QoSProfile

def euler_to_quaternion(yaw):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)

def parse_line(line):
    match = re.match(r"x:\s*(-?\d+\.\d+)\s*m,\s*y:\s*(-?\d+\.\d+)\s*m,\s*θ:\s*(-?\d+\.\d+)\s*rad", line)
    if match:
        x = float(match.group(1))
        y = float(match.group(2))
        theta = float(match.group(3))
        return x, y, theta
    return None

class SerialOdomNode(Node):
    def __init__(self):
        super().__init__('serial_odom_node')
        
        # Set your actual serial port here
        port = '/dev/ttyACM0'
        baud = 115200

        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Opened serial port: {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open {port}: {e}")
            raise SystemExit

        qos = QoSProfile(depth=10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        self.last_time = self.get_clock().now()
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_theta = 0.0

    def timer_callback(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")
            return

        result = parse_line(line)
        if not result:
            return

        x, y, theta = result
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        dx = x - self.prev_x
        dy = y - self.prev_y
        dtheta = theta - self.prev_theta

        vx = dx / dt if dt > 0 else 0.0
        vy = dy / dt if dt > 0 else 0.0
        vth = dtheta / dt if dt > 0 else 0.0

        self.prev_x = x
        self.prev_y = y
        self.prev_theta = theta

        # Create quaternion from yaw
        _, _, qz, qw = euler_to_quaternion(theta)

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)

        # Publish TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"

        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SerialOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

