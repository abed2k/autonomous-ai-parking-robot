import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv2 import aruco
import cv2
import numpy as np
import math
import time
from scipy.spatial.transform import Rotation

# ── Camera ──────────────────────────────────────────────────────────
CAMERA_MATRIX = np.array([
    [834.24401578,   0.0,          314.36318849],
    [0.0,            828.02604102, 195.32008769],
    [0.0,            0.0,          1.0         ]
], dtype=np.float64)
DIST_COEFFS = np.array([[-0.11480671, 3.03709683, -0.01482503, 0.00027578, -12.45467093]])
MARKER_LENGTH  = 0.17
ARUCO_DICT     = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
OBJ_PTS = np.array([
    [-MARKER_LENGTH/2,  MARKER_LENGTH/2, 0],
    [ MARKER_LENGTH/2,  MARKER_LENGTH/2, 0],
    [ MARKER_LENGTH/2, -MARKER_LENGTH/2, 0],
    [-MARKER_LENGTH/2, -MARKER_LENGTH/2, 0]
], dtype=np.float32)

FOCAL_LENGTH_PX  = 834.24
ASSUMED_Z_METERS = 1.18

# ── Motion params ────────────────────────────────────────────────────
LINEAR_FAST     = 0.5
LINEAR_SLOW     = 0.2
ANGULAR_FAST    = 2.0
ANGULAR_SLOW    = 1.0
SLOWDOWN_DIST   = 0.10
SLOWDOWN_DEG    = 10.0
TOLERANCE_DIST  = 0.02
TOLERANCE_ANGLE = 2.0

STEP1_DIST = 1.20
STEP2_DIST = 2.00

CONFIRM_FRAMES = 5

def encoder_to_real(encoder_x)
    return (encoder_x * 2) / 1.363

def calc_move(dis, y, pitch):
    return 1.666 - 1.258*dis + 0.354*(dis**2) + 0.370*y - 0.003*abs(pitch)

def detect_red_lines(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red1, upper_red1 = np.array([0,   60,  60]),  np.array([15, 255, 255])
    lower_red2, upper_red2 = np.array([160, 60,  60]),  np.array([180, 255, 255])
    mask = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1),
                          cv2.inRange(hsv, lower_red2, upper_red2))
    kernel_close = np.ones((9, 9), np.uint8)
    kernel_open  = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel_open,  iterations=1)
    mask = cv2.medianBlur(mask, 5)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid = [c for c in contours if cv2.contourArea(c) > 80]
    valid = sorted(valid, key=cv2.contourArea, reverse=True)[:2]
    centers = []
    for cnt in valid:
        M = cv2.moments(cnt)
        if M['m00'] != 0:
            centers.append((int(M['m10']/M['m00']), int(M['m01']/M['m00'])))
    dist_cm = None
    if len(centers) == 2:
        p1, p2 = centers[0], centers[1]
        pixel_dist = math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
        dist_cm = (pixel_dist / FOCAL_LENGTH_PX) * ASSUMED_Z_METERS * 100
    return frame, dist_cm

class ArucoParking(Node):
    def __init__(self):
        super().__init__('aruco_parking')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(String, '/filtered_odom', self.odom_cb, 10)
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.enc_x = None
        self.yaw   = None
        self.flush_camera()
        self.get_logger().info('ArUco Parking Node started.')

    def flush_camera(self):
        for _ in range(30):
            self.cap.grab()

    def odom_cb(self, msg):
        try:
            parts = msg.data.split()
            self.enc_x = float(parts[1])
            self.yaw   = float(parts[3])
        except:
            pass

    def wait_for_odom(self):
        while self.enc_x is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    def stop(self, duration=0.5):
        msg = Twist()
        end = time.time() + duration
        while time.time() < end:
            self.pub.publish(msg)
            time.sleep(0.05)

    def angle_diff(self, target, current):
        diff = target - current
        while diff > 180.0:  diff -= 360.0
        while diff < -180.0: diff += 360.0
        return diff

    def _raw_detect(self, frame):
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, ARUCO_DICT)
        if ids is None or len(ids) == 0:
            return None
        pts = corners[0][0]
        perimeter = cv2.arcLength(pts, True)
        if perimeter < 80:
            return None
        _, rvec, tvec = cv2.solvePnP(OBJ_PTS, pts, CAMERA_MATRIX, DIST_COEFFS)
        x_cam, y_cam, z_cam = tvec.flatten()
        distance = math.sqrt(x_cam**2 + y_cam**2 + z_cam**2)
        if not (0.3 < distance < 5.0):
            return None
        R, _ = cv2.Rodrigues(rvec)
        rot = Rotation.from_matrix(R)
        roll, pitch, yaw_m = rot.as_euler('xyz', degrees=True)
        is_good = abs(x_cam) < 0.04 and abs(pitch) < 5.0 and 1.7 < distance < 1.8
        return {'dis': distance, 'x': x_cam, 'y': y_cam,
                'pitch': pitch, 'yaw': yaw_m, 'is_good': is_good}

    def single_detect(self):
        ret, frame = self.cap.read()
        if not ret:
            return None
        return self._raw_detect(frame)

    def confirm_stationary(self):
        """Confirm marker in CONFIRM_FRAMES consecutive frames while stopped."""
        self.flush_camera()
        count = 0
        samples = []
        attempts = 0
        max_attempts = CONFIRM_FRAMES * 6

        while attempts < max_attempts:
            rclpy.spin_once(self, timeout_sec=0.02)
            ret, frame = self.cap.read()
            if not ret:
                attempts += 1
                continue
            data = self._raw_detect(frame)
            attempts += 1
            if data is not None:
                count += 1
                samples.append(data)
                self.get_logger().info(
                    f'Confirm {count}/{CONFIRM_FRAMES}: '
                    f'dis={data["dis"]:.2f} x={data["x"]:.3f} pitch={data["pitch"]:.1f}')
                if count >= CONFIRM_FRAMES:
                    avg = {k: sum(s[k] for s in samples) / len(samples)
                           for k in ['dis', 'x', 'y', 'pitch', 'yaw']}
                    avg['is_good'] = samples[-1]['is_good']
                    self.get_logger().info('Marker CONFIRMED.')
                    return avg
            else:
                if count > 0:
                    self.get_logger().info('Lost during confirmation, resetting.')
                count = 0
                samples = []

        self.get_logger().warn('Confirmation failed — resuming motion.')
        return None

    def move_distance(self, distance):
        """
        Move exactly `distance` real meters using encoder.
        On single detection: stop and confirm. If confirmed return data.
        If not confirmed: resume, tracking total traveled via encoder accumulation.
        """
        self.wait_for_odom()
        direction = 1.0 if distance > 0 else -1.0
        target_real = abs(distance)
        total_traveled = 0.0          # accumulates real meters across resume cycles
        prev_enc = self.enc_x

        self.get_logger().info(f'Moving {distance:.2f}m (encoder)...')

        while True:
            rclpy.spin_once(self, timeout_sec=0.02)

            # Accumulate encoder travel continuously
            cur_enc = self.enc_x
            delta_real = encoder_to_real(abs(cur_enc - prev_enc))
            total_traveled += delta_real
            prev_enc = cur_enc

            remaining = target_real - total_traveled
            self.get_logger().debug(f'traveled={total_traveled:.3f} remaining={remaining:.3f}')

            if remaining <= TOLERANCE_DIST:
                break

            data = self.single_detect()
            if data is not None:
                self.stop()
                self.get_logger().info(
                    f'Marker glimpsed (dis={data["dis"]:.2f}) at traveled={total_traveled:.3f}m, confirming...')
                confirmed = self.confirm_stationary()
                if confirmed is not None:
                    return confirmed
                # Resume — prev_enc already current, total_traveled already correct
                self.get_logger().info(
                    f'Resuming move, {remaining:.3f}m still needed...')
                prev_enc = self.enc_x
                continue

            speed = direction * (LINEAR_SLOW if remaining <= SLOWDOWN_DIST else LINEAR_FAST)
            msg = Twist()
            msg.linear.x = speed
            self.pub.publish(msg)

        self.stop()
        self.get_logger().info(f'Move done. total_traveled={total_traveled:.3f}m')
        return None

    def rotate_angle(self, angle_deg):
        """Rotate exactly angle_deg using IMU yaw."""
        self.wait_for_odom()
        target_yaw = self.yaw + angle_deg
        while target_yaw > 180.0:  target_yaw -= 360.0
        while target_yaw < -180.0: target_yaw += 360.0
        self.get_logger().info(f'Rotating {angle_deg:.1f}deg (IMU yaw)...')
        while True:
            rclpy.spin_once(self, timeout_sec=0.02)
            remaining = self.angle_diff(target_yaw, self.yaw)
            if abs(remaining) <= TOLERANCE_ANGLE:
                break
            speed = math.copysign(
                ANGULAR_SLOW if abs(remaining) <= SLOWDOWN_DEG else ANGULAR_FAST,
                remaining)
            msg = Twist()
            msg.angular.z = speed
            self.pub.publish(msg)
        self.stop()
        self.get_logger().info(f'Rotation done. yaw={self.yaw:.2f}deg')

    def rotate_360_search(self):
        """
        Rotate a full 360 using IMU yaw accumulation.
        On single detection: stop and confirm. If confirmed return data.
        If not confirmed: resume, continuing to accumulate toward 360.
        """
        self.wait_for_odom()
        total_rotated = 0.0
        prev_yaw = self.yaw
        self.get_logger().info('Rotating 360 searching for marker (IMU yaw)...')

        while True:
            rclpy.spin_once(self, timeout_sec=0.02)

            # Accumulate absolute rotation
            cur_yaw = self.yaw
            delta = abs(self.angle_diff(cur_yaw, prev_yaw))
            total_rotated += delta
            prev_yaw = cur_yaw

            self.get_logger().debug(f'total_rotated={total_rotated:.1f}deg')

            if total_rotated >= 358.0:
                self.stop()
                self.get_logger().info('360 done, no marker confirmed.')
                return None

            data = self.single_detect()
            if data is not None:
                self.stop()
                self.get_logger().info(
                    f'Marker glimpsed at {total_rotated:.1f}deg, confirming...')
                confirmed = self.confirm_stationary()
                if confirmed is not None:
                    return confirmed
                # Resume rotation — prev_yaw already updated
                self.get_logger().info(
                    f'Resuming rotation, {360.0 - total_rotated:.1f}deg remaining...')
                prev_yaw = self.yaw
                continue

            msg = Twist()
            msg.angular.z = ANGULAR_FAST
            self.pub.publish(msg)

    def make_is_good(self):
        self.get_logger().info('Adjusting until is_good...')
        while True:
            rclpy.spin_once(self, timeout_sec=0.02)
            ret, frame = self.cap.read()
            if not ret:
                continue
            data = self._raw_detect(frame)
            if data is None:
                msg = Twist()
                msg.angular.z = 0.5
                self.pub.publish(msg)
                continue
            if data['is_good']:
                self.stop()
                self.get_logger().info('is_good = True!')
                return data
            dis   = data['dis']
            x     = data['x']
            pitch = data['pitch']
            msg   = Twist()
            if dis < 1.7:
                msg.linear.x = -LINEAR_SLOW
            elif dis > 1.8:
                msg.linear.x = LINEAR_SLOW
            elif abs(x) >= 0.04:
                msg.angular.z = -x * 5.0
            elif abs(pitch) >= 5.0:
                msg.angular.z = math.copysign(0.5, -pitch)
            self.pub.publish(msg)

    def run(self):
        self.wait_for_odom()

        self.get_logger().info('STEP 1: Moving 120cm...')
        data = self.move_distance(STEP1_DIST)

        if data is None:
            while True:
                self.get_logger().info('STEP 2: Rotating 360 to search...')
                data = self.rotate_360_search()
                if data is None:
                    self.get_logger().info('No marker. Moving 200cm...')
                    data = self.move_distance(STEP2_DIST)
                    if data is not None:
                        break
                    continue
                break

        dis   = data['dis']
        x     = data['x']
        y     = data['y']
        pitch = data['pitch']
        self.get_logger().info(
            f'STEP 3 CONFIRMED: dis={dis:.3f} x={x:.3f} y={y:.3f} pitch={pitch:.1f}')

        move = calc_move(dis, y, pitch)
        self.get_logger().info(f'STEP 4: calculated move={move:.3f}m')

        rotation_angle = 90.0 - abs(pitch)
        if pitch > 0:
            step5_dir = 1
            self.get_logger().info(f'STEP 5: Pitch positive → CCW {rotation_angle:.1f}deg')
            self.rotate_angle(rotation_angle)
        else:
            step5_dir = -1
            self.get_logger().info(f'STEP 5: Pitch negative → CW {rotation_angle:.1f}deg')
            self.rotate_angle(-rotation_angle)

        self.get_logger().info(f'STEP 6: Moving {move:.3f}m...')
        self.move_distance(move)

        if step5_dir == 1:
            self.get_logger().info('STEP 7: Rotating CW 90deg...')
            self.rotate_angle(-90.0)
        else:
            self.get_logger().info('STEP 7: Rotating CCW 90deg...')
            self.rotate_angle(90.0)

        self.get_logger().info('STEP 8: Adjusting until is_good...')
        self.make_is_good()

        self.get_logger().info('STEP 9: Stopping 1s...')
        self.stop(duration=1.0)

        self.get_logger().info('STEP 10: Detecting red lines...')
        for _ in range(200):
            ret, frame = self.cap.read()
            if not ret:
                continue
            frame, dist_cm = detect_red_lines(frame)
            if dist_cm is not None:
                print(f'[RED LINES] Distance between lines: {dist_cm:.2f} cm')
            else:
                print('[RED LINES] Need 2 lines, waiting...')
            rclpy.spin_once(self, timeout_sec=0.02)

        self.get_logger().info('PARKING COMPLETE!')

def main():
    rclpy.init()
    node = ArucoParking()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.cap.release()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
