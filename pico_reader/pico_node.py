import rclpy
from rclpy.node import Node
import serial
import serial.tools.list_ports
import math
import time

from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def find_pico_port():
    PICO_VID = 0x2E8A
    candidates = []
    for port in serial.tools.list_ports.comports():
        if port.vid == PICO_VID:
            candidates.insert(0, port.device)
        elif port.device.startswith('/dev/ttyACM'):
            candidates.append(port.device)
    return candidates[0] if candidates else None


class PicoNode(Node):

    def __init__(self):
        super().__init__('pico_node')

        self.ser = None
        self.port_used = None
        self.connect_serial()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.filtered_odom_pub = self.create_publisher(String, '/filtered_odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)

        self.x = 0.0
        self.y = 0.0
        self.theta_enc = 0.0
        self.theta_imu = 0.0
        self.prev = [0, 0, 0, 0]

        self.last_time = self.get_clock().now().nanoseconds * 1e-9
        self.cmd_angular_z = 0.0

        self.CPR = 1920
        self.WHEEL_RADIUS = 0.044
        self.WHEEL_BASE = 0.30
        self.GYRO_SCALE = 1.07

        self.gz_bias_samples = []
        self.GZ_BIAS = 0.0

        self.timer = self.create_timer(0.02, self.loop)

    def connect_serial(self):
        while True:
            port = find_pico_port()
            if port is None:
                print("[SERIAL] No Pico/ACM port found, retrying in 2s...")
                time.sleep(2)
                continue
            try:
                print(f"[SERIAL] Trying port {port}")
                self.ser = serial.Serial()
                self.ser.port = port
                self.ser.baudrate = 115200
                self.ser.timeout = 1
                self.ser.dtr = False
                self.ser.rts = False
                self.ser.open()
                time.sleep(0.5)
                self.ser.reset_input_buffer()
                self.port_used = port
                print(f"[SERIAL] Connected to {port}")
                return
            except Exception as e:
                print(f"[SERIAL] Failed to connect to {port}: {e}, retrying in 2s...")
                time.sleep(2)

    def cmd_cb(self, msg):
        self.cmd_angular_z = msg.angular.z
        v = msg.linear.x
        w = msg.angular.z
        vl = v - (w * self.WHEEL_BASE / 2)
        vr = v + (w * self.WHEEL_BASE / 2)
        left  = max(min(int(vl * 100), 100), -100)
        right = max(min(int(vr * 100), 100), -100)
        MIN_PWM = 35
        if 0 < left < MIN_PWM:
            left = MIN_PWM
        elif -MIN_PWM < left < 0:
            left = -MIN_PWM
        if 0 < right < MIN_PWM:
            right = MIN_PWM
        elif -MIN_PWM < right < 0:
            right = -MIN_PWM
        try:
            self.ser.write(f"{left},{right}\n".encode())
        except Exception:
            pass

    def loop(self):
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
        except Exception as e:
            print(f"[SERIAL] Read error on {self.port_used}: {e}, reconnecting...")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.port_used = None
            self.gz_bias_samples = []
            self.GZ_BIAS = 0.0
            self.connect_serial()
            return

        if not line:
            return

        parts = line.split(',')
        if len(parts) != 10:
            return

        try:
            acc_x, acc_y, acc_z, gx, gy, gz, e1, e2, e3, e4 = map(int, parts)
        except Exception:
            return

        enc = [e1, e2, e3, e4]

        if len(self.gz_bias_samples) < 100:
            self.gz_bias_samples.append(gz)
            print(f"[CALIBRATING] {len(self.gz_bias_samples)}/100 samples collected...")
            return

        if self.GZ_BIAS == 0.0:
            self.GZ_BIAS = sum(self.gz_bias_samples) / len(self.gz_bias_samples)
            self.get_logger().info(f"GZ bias calibrated: {self.GZ_BIAS}")
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0:
            return

        gz_dps  = (gz - self.GZ_BIAS) / 131.0
        gz_rads = -gz_dps * math.pi / 180.0

        d  = [enc[i] - self.prev[i] for i in range(4)]
        self.prev = enc.copy()

        dm = [(d[i] / self.CPR) * 2 * math.pi * self.WHEEL_RADIUS for i in range(4)]

        dl = (dm[1] + dm[2]) / 2.0
        dr = (dm[0] + dm[3]) / 2.0

        dc = (dl + dr) / 2.0
        dtheta_enc = (dr - dl) / self.WHEEL_BASE
        self.theta_enc += dtheta_enc

        if abs(self.cmd_angular_z) > 0.01:
            self.theta_imu += gz_rads * dt * self.GYRO_SCALE
        self.theta_imu = math.atan2(math.sin(self.theta_imu), math.cos(self.theta_imu))

        # project displacement onto world X/Y using current heading
        self.x += dc * math.cos(self.theta_imu)
        self.y += dc * math.sin(self.theta_imu)

        imu = Imu()
        imu.header.stamp    = self.get_clock().now().to_msg()
        imu.header.frame_id = "imu_link"
        imu.linear_acceleration.x = acc_x / 16384.0 * 9.81
        imu.linear_acceleration.y = acc_y / 16384.0 * 9.81
        imu.linear_acceleration.z = acc_z / 16384.0 * 9.81
        imu.angular_velocity.x    = gx / 131.0 * math.pi / 180.0
        imu.angular_velocity.y    = gy / 131.0 * math.pi / 180.0
        imu.angular_velocity.z    = gz_rads
        self.imu_pub.publish(imu)

        print(
            "\n-----------------------------"
            f"\nX:           {self.x:.4f} m"
            f"\nY:           {self.y:.4f} m"
            f"\nENCODER YAW: {math.degrees(self.theta_enc):.2f} deg"
            f"\nIMU YAW:     {math.degrees(self.theta_imu):.2f} deg"
            f"\nGZ:          {gz_dps:.3f} deg/s"
            f"\nENCODERS:    {e1}, {e2}, {e3}, {e4}"
            f"\nPORT:        {self.port_used}"
            "\n-----------------------------"
        )

        odom = Odometry()
        odom.header.stamp    = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_link"
        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta_imu / 2)
        odom.pose.pose.orientation.w = math.cos(self.theta_imu / 2)
        odom.pose.covariance[0]  = 0.05
        odom.pose.covariance[7]  = 0.05
        odom.pose.covariance[35] = 0.1
        odom.twist.covariance[0]  = 0.05
        odom.twist.covariance[7]  = 0.05
        odom.twist.covariance[35] = 0.1
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp       = odom.header.stamp
        t.header.frame_id    = "odom"
        t.child_frame_id     = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z    = math.sin(self.theta_imu / 2)
        t.transform.rotation.w    = math.cos(self.theta_imu / 2)
        self.tf_broadcaster.sendTransform(t)

        filtered = String()
        filtered.data = f"x: {self.x:.4f} yaw: {math.degrees(self.theta_imu):.2f}"
        self.filtered_odom_pub.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = PicoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
