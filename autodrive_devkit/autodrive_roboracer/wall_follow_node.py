#!/usr/bin/env python3
"""
Wall-following node for the AutoDRIVE RoboRacer (Penn RoboRacer technical challenge).

Pipeline (runs once per LiDAR scan, ~40 Hz):

  1. Clean the scan (NaN/inf -> max range).
  2. Steering: PD controller that keeps the car on the centerline between the
     left and right walls. Each wall is estimated from two beams (one
     perpendicular, one angled forward), which gives both the distance to the
     wall and the car's heading relative to it. The distance is projected a
     short distance ahead (lookahead) so the controller reacts early in corners.
  3. Safety: if something is directly ahead or a wall is very close on one
     side, the steering is biased toward the open side.
  4. Speed: target speed is the minimum of
       - a braking-distance limit from the free space straight ahead, and
       - a lateral-acceleration limit from the commanded steering curvature.
     A PI loop on the measured speed (odometry) turns that into a throttle.

Every gain is a ROS parameter, so you can tune without rebuilding, e.g.:
  ros2 run autodrive_roboracer wall_follow_node --ros-args -p v_max:=7.0 -p kp:=1.2
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int32

# Topics (from the challenge sheet / AutoDRIVE bridge config)
LIDAR_TOPIC = '/autodrive/roboracer_1/lidar'
ODOM_TOPIC = '/autodrive/roboracer_1/odom'
THROTTLE_TOPIC = '/autodrive/roboracer_1/throttle_command'
STEERING_TOPIC = '/autodrive/roboracer_1/steering_command'
LAP_COUNT_TOPIC = '/autodrive/roboracer_1/lap_count'
LAST_LAP_TOPIC = '/autodrive/roboracer_1/last_lap_time'
BEST_LAP_TOPIC = '/autodrive/roboracer_1/best_lap_time'
COLLISION_TOPIC = '/autodrive/roboracer_1/collision_count'

# Default tuning. Each key becomes a ROS parameter of the same name.
DEFAULTS = {
    # Vehicle
    'max_steer_rad': 0.5236,     # steering command 1.0 == ~0.52 rad (full left)
    'wheelbase': 0.324,          # m
    'car_half_width': 0.17,      # m (half body width + small margin)

    # Wall following (centerline between both walls)
    'theta_deg': 45.0,           # angle between the perpendicular and the forward beam
    'lookahead': 0.4,            # m, how far ahead the wall distance is projected
    'wall_clip': 1.5,            # m, cap on per-side distance (open areas / gaps)
    'kp': 0.25,                  # rad of steer per m of centering error
    'kd': 0.12,                  # rad of steer per (m/s) of error rate

    # Corner handling: steer toward the deepest free direction
    'gap_weight': 0.7,           # 0 = pure wall following, 1 = pure gap steering
    'gap_fov_deg': 90.0,         # only look this far left/right for the gap
    'disparity_thresh': 0.25,    # m jump between beams that counts as a wall edge
    'gap_margin': 0.15,          # m extra clearance added when inflating wall edges
    'gap_gain': 0.9,             # rad of steer per rad of gap angle (30 deg gap -> full lock)
    'gap_ref_dist': 3.0,         # m, targets further than this get proportionally gentler steering
    'gap_min_scale': 0.25,       # floor on that gentler-steering factor
    'gap_full_angle_deg': 35.0,  # opening angle at which full steering strength returns
    'outside_steer': 0.0,        # rad of steer toward the outside before a corner (0 = off)
    'outside_dist': 3.5,         # m, start drifting outside when the wall ahead is closer than this
    'outside_min_diff': 0.8,     # m, how lopsided the view ahead must be to count as a corner
    'corner_front_dist': 2.0,    # m, below this the gap steering takes over fully

    # Side safety
    'side_min_dist': 0.30,       # m, start pushing away from a wall at the side
    'side_push_gain': 1.5,       # rad per m of side intrusion
    'steer_smoothing': 0.15,     # 0 = no smoothing, ->1 = heavy smoothing

    # Speed planner
    'v_max': 8.0,                # m/s on straights (raise this to get faster laps)
    'v_min': 1.8,                # m/s floor so the car never stalls
    'a_brake': 3.5,              # m/s^2 assumed braking capability
    'a_lat': 4.0,                # m/s^2 allowed lateral acceleration
    'stop_margin': 0.9,          # m kept free ahead when planning braking distance
    'v_corner': 2.25,            # m/s target in a sharp corner
    'corner_angle_deg': 60.0,    # gap angle treated as a sharp corner
    'speed_cone_deg': 5.0,       # cone around the heading used to measure free distance

    # Throttle (PI on measured speed, plus feed-forward)
    'throttle_ff': 0.045,        # throttle per m/s (feed-forward)
    'throttle_kp': 0.5,          # throttle per m/s of speed error
    'throttle_ki': 0.05,         # throttle per (m/s * s) of speed error
    'throttle_max': 1.0,
    'throttle_min': -1.0,        # negative throttle is used as a brake
    'open_loop_v_full': 20.0,    # m/s at full throttle, only used if no odometry
}


class WallFollowController:
    """ROS-free controller so the logic can be unit tested offline."""

    def __init__(self, params=None):
        self.p = dict(DEFAULTS)
        if params:
            self.p.update(params)
        self.prev_error = None
        self.prev_steer = 0.0
        self.speed_integral = 0.0
        self.debug = {}

    # ------------------------------------------------------------------ helpers
    @staticmethod
    def _index(angle, angle_min, angle_inc, n):
        i = int(round((angle - angle_min) / angle_inc))
        return min(max(i, 0), n - 1)

    def _range_at(self, ranges, angle, angle_min, angle_inc):
        """Median of 5 beams around an angle (robust to single bad returns)."""
        n = len(ranges)
        i = self._index(angle, angle_min, angle_inc, n)
        lo, hi = max(i - 2, 0), min(i + 3, n)
        return float(np.median(ranges[lo:hi]))

    def _wall(self, ranges, side, angle_min, angle_inc):
        """
        Estimate one wall from two beams.
        side = +1 for the left wall, -1 for the right wall.
        Returns the wall distance projected `lookahead` metres ahead.
        """
        theta = math.radians(self.p['theta_deg'])
        b = self._range_at(ranges, side * math.pi / 2, angle_min, angle_inc)
        a = self._range_at(ranges, side * (math.pi / 2 - theta), angle_min, angle_inc)
        # Angle of the wall relative to the car's heading (F1TENTH wall-follow lab).
        alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
        dist = b * math.cos(alpha)
        # Heading toward the wall makes alpha < 0, so the projected distance shrinks.
        projected = dist + self.p['lookahead'] * math.sin(alpha)
        clip = self.p['wall_clip']
        return min(max(projected, 0.0), clip)

    # --------------------------------------------------------------------- main
    def _gap_angle(self, r, angles):
        """Disparity extender: inflate wall edges by the car's half width, then
        return the angle of the deepest reachable direction."""
        p = self.p
        fov = math.radians(p['gap_fov_deg'])
        idx = np.where(np.abs(angles) <= fov)[0]
        rr = r[idx].copy()
        ang_inc = float(angles[1] - angles[0])
        width = p['car_half_width'] + p['gap_margin']
        ext = rr.copy()
        jumps = np.where(np.abs(np.diff(rr)) > p['disparity_thresh'])[0]
        for j in jumps:
            near = min(rr[j], rr[j + 1])
            n = int(math.ceil(width / max(near * ang_inc, 1e-3)))
            if rr[j] < rr[j + 1]:          # closer on the right of the edge -> extend leftwards
                lo, hi = j + 1, min(j + 1 + n, len(ext))
            else:                          # closer on the left -> extend rightwards
                lo, hi = max(j - n + 1, 0), j + 1
            ext[lo:hi] = np.minimum(ext[lo:hi], near)
        best = float(np.max(ext))
        # Middle of the contiguous block of near-best beams around the deepest beam.
        k = int(np.argmax(ext - 1e-3 * np.abs(angles[idx])))  # tie-break: straighter
        thresh = best * 0.92
        lo = k
        while lo > 0 and ext[lo - 1] >= thresh:
            lo -= 1
        hi = k
        while hi < len(ext) - 1 and ext[hi + 1] >= thresh:
            hi += 1
        mid = (lo + hi) // 2
        return float(angles[idx][mid]), float(ext[mid])

    def compute(self, ranges, angle_min, angle_inc, range_max, speed, dt):
        """
        ranges: 1-D array of LiDAR ranges. speed: measured m/s (None if unknown).
        Returns (throttle_cmd, steering_cmd), both in [-1, 1].
        """
        p = self.p
        r = np.asarray(ranges, dtype=float)
        r = np.where(np.isfinite(r), r, range_max)
        r = np.clip(r, 0.0, range_max)
        n = len(r)
        angles = angle_min + angle_inc * np.arange(n)
        dt = dt if dt > 0 else 0.025

        # ---- 1. Centerline wall following ---------------------------------
        d_left = self._wall(r, +1, angle_min, angle_inc)
        d_right = self._wall(r, -1, angle_min, angle_inc)
        error = 0.5 * (d_left - d_right)        # > 0: more room on the left -> steer left
        d_error = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        self.prev_error = error
        wall_steer = p['kp'] * error + p['kd'] * d_error

        # ---- 2. Corner handling: deepest free direction ----------------------
        gap_ang, gap_depth = self._gap_angle(r, angles)
        # Pure pursuit to a point at that angle.
        # Steer hard toward close openings (corners) but gently toward far ones
        # (straights): the gain falls off with the depth of the target, like pure pursuit.
        # This stops the car weaving on the straight. Set gap_ref_dist to 99 to disable.
        depth_scale = min(max(p['gap_ref_dist'] / max(gap_depth, 0.1), p['gap_min_scale']), 1.0)
        # ...but only when the opening is roughly ahead. Once it swings to the side
        # (hairpin entry/exit), go back to full steering strength.
        side = min(abs(gap_ang) / math.radians(p['gap_full_angle_deg']), 1.0) ** 2
        depth_scale = depth_scale + (1.0 - depth_scale) * side
        gap_steer = p['gap_gain'] * gap_ang * depth_scale
        front_mask = np.abs(angles) <= math.radians(p['speed_cone_deg'])
        front = float(np.min(r[front_mask]))
        # Near a wall ahead (a corner), trust the free-space direction completely.
        closeness = min(max((p['corner_front_dist'] - front) / p['corner_front_dist'], 0.0), 1.0)
        w = p['gap_weight'] + (1.0 - p['gap_weight']) * closeness
        steer = (1.0 - w) * wall_steer + w * gap_steer

        # Out-in-out line (off when outside_steer = 0): on the approach to a corner,
        # drift toward the outside wall so the turn itself can be wider and faster.
        if p['outside_steer'] > 0 and p['corner_front_dist'] < front < p['outside_dist']:
            left_ahead = float(np.mean(r[(angles > math.radians(20)) & (angles < math.radians(60))]))
            right_ahead = float(np.mean(r[(angles < -math.radians(20)) & (angles > -math.radians(60))]))
            if abs(left_ahead - right_ahead) > p['outside_min_diff']:
                turn_dir = 1.0 if left_ahead > right_ahead else -1.0
                steer -= turn_dir * p['outside_steer']

        # ---- 3. Side safety ------------------------------------------------
        side_left = float(np.min(r[(angles > math.radians(35)) & (angles < math.radians(110))]))
        side_right = float(np.min(r[(angles < -math.radians(35)) & (angles > -math.radians(110))]))
        if side_left < p['side_min_dist']:
            steer -= p['side_push_gain'] * (p['side_min_dist'] - side_left)
        if side_right < p['side_min_dist']:
            steer += p['side_push_gain'] * (p['side_min_dist'] - side_right)

        steer = max(-p['max_steer_rad'], min(p['max_steer_rad'], steer))
        a = p['steer_smoothing']
        steer = a * self.prev_steer + (1.0 - a) * steer
        self.prev_steer = steer

        # ---- 4. Speed planning ---------------------------------------------
        free = max(front, min(gap_depth, front + 1.0)) if abs(gap_ang) < 0.35 else front
        v_brake = math.sqrt(2.0 * p['a_brake'] * max(free - p['stop_margin'], 0.0))
        curvature = abs(math.tan(steer)) / p['wheelbase']
        v_curve = math.sqrt(p['a_lat'] / curvature) if curvature > 1e-3 else p['v_max']
        # Slow down for sharp corners as soon as the free direction swings sideways.
        turn = min(abs(gap_ang) / math.radians(p['corner_angle_deg']), 1.0)
        v_turn = p['v_max'] - (p['v_max'] - p['v_corner']) * turn
        v_target = max(p['v_min'], min(p['v_max'], v_brake, v_curve, v_turn))

        # ---- 5. Throttle ---------------------------------------------------
        if speed is None:
            throttle = v_target / p['open_loop_v_full']
        else:
            err_v = v_target - speed
            self.speed_integral = max(-3.0, min(3.0, self.speed_integral + err_v * dt))
            throttle = (p['throttle_ff'] * v_target + p['throttle_kp'] * err_v
                        + p['throttle_ki'] * self.speed_integral)
        throttle = max(p['throttle_min'], min(p['throttle_max'], throttle))

        self.debug = dict(d_left=d_left, d_right=d_right, error=error, front=front,
                          gap_deg=math.degrees(gap_ang), v_target=v_target, steer_rad=steer)
        steering_cmd = max(-1.0, min(1.0, steer / p['max_steer_rad']))
        return float(throttle), float(steering_cmd)


class WallFollowNode(Node):
    def __init__(self):
        super().__init__('wall_follow_node')
        for key, value in DEFAULTS.items():
            self.declare_parameter(key, value)
        self.controller = WallFollowController(self._read_params())

        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.pub_throttle = self.create_publisher(Float32, THROTTLE_TOPIC, qos)
        self.pub_steering = self.create_publisher(Float32, STEERING_TOPIC, qos)
        self.create_subscription(LaserScan, LIDAR_TOPIC, self.scan_callback, qos)
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_callback, qos)
        self.create_subscription(Int32, LAP_COUNT_TOPIC, self.lap_count_callback, qos)
        self.create_subscription(Float32, LAST_LAP_TOPIC, self.last_lap_callback, qos)
        self.create_subscription(Float32, BEST_LAP_TOPIC, self.best_lap_callback, qos)
        self.create_subscription(Int32, COLLISION_TOPIC, self.collision_callback, qos)
        self.add_on_set_parameters_callback(self._on_params)

        # Optional recorder: ros2 run ... --ros-args -p record:=true
        # Saves to autodrive_devkit/logs/ (the folder shared with your PC).
        self.declare_parameter('record', False)
        self.declare_parameter('log_dir', '/home/autodrive_devkit/src/autodrive_devkit/logs')
        self.record = bool(self.get_parameter('record').value)
        self.rec = []
        self.pose = (0.0, 0.0, 0.0)
        if self.record:
            import os
            import time
            os.makedirs(self.get_parameter('log_dir').value, exist_ok=True)
            self.rec_path = os.path.join(self.get_parameter('log_dir').value,
                                         time.strftime('run_%Y%m%d_%H%M%S.npz'))
            self.get_logger().info('Recording. Press Ctrl+C to stop and save to ' + self.rec_path)

        self.speed = None
        self.last_scan_time = None
        self.lap_count = None
        self.last_lap = 0.0
        self.best_lap = 0.0
        self.collisions = None
        self.scan_counter = 0
        self.create_timer(0.25, self.watchdog)
        self.get_logger().info('Wall follower started. Waiting for LiDAR on ' + LIDAR_TOPIC)

    def _read_params(self):
        return {k: self.get_parameter(k).value for k in DEFAULTS}

    def _on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for prm in params:
            if prm.name in DEFAULTS and prm.name not in ('record', 'log_dir'):
                self.controller.p[prm.name] = float(prm.value)
        return SetParametersResult(successful=True)

    def odom_callback(self, msg):
        v = msg.twist.twist.linear
        self.speed = math.sqrt(v.x * v.x + v.y * v.y)
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def scan_callback(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.025 if self.last_scan_time is None else min(max(now - self.last_scan_time, 1e-3), 0.2)
        self.last_scan_time = now
        throttle, steering = self.controller.compute(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_max, self.speed, dt)
        self.publish(throttle, steering)
        if self.record and len(self.rec) < 40 * 180:   # up to 3 minutes
            self.rec.append((now, self.pose[0], self.pose[1], self.pose[2],
                             -1.0 if self.speed is None else self.speed, throttle, steering,
                             -1 if self.collisions is None else self.collisions,
                             -1 if self.lap_count is None else self.lap_count,
                             np.asarray(msg.ranges, dtype=np.float32)))

        self.scan_counter += 1
        if self.scan_counter % 40 == 0:
            d = self.controller.debug
            self.get_logger().info(
                f"v={0.0 if self.speed is None else self.speed:4.1f}/{d['v_target']:4.1f} m/s  "
                f"L={d['d_left']:4.2f} R={d['d_right']:4.2f} front={d['front']:4.1f} gap={d['gap_deg']:+4.0f}deg  "
                f"thr={throttle:+.2f} steer={steering:+.2f}")

    def save_recording(self):
        if not self.rec:
            return
        meta = np.array([row[:9] for row in self.rec], dtype=np.float64)
        scans = np.stack([row[9] for row in self.rec])
        np.savez_compressed(self.rec_path, meta=meta, scans=scans,
                            columns=np.array(['t', 'x', 'y', 'yaw', 'v', 'throttle', 'steering',
                                              'collisions', 'lap']))

    def watchdog(self):
        # Stop the car if the LiDAR stream stops (e.g. bridge disconnected).
        if self.last_scan_time is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_scan_time > 0.5:
            self.publish(0.0, 0.0)

    def publish(self, throttle, steering):
        self.pub_throttle.publish(Float32(data=float(throttle)))
        self.pub_steering.publish(Float32(data=float(steering)))

    # Lap / collision logging, so you can read lap times from the terminal too.
    def lap_count_callback(self, msg):
        if self.lap_count is not None and msg.data > self.lap_count:
            self.get_logger().info(
                f'LAP {msg.data} complete. last={self.last_lap:.3f}s best={self.best_lap:.3f}s')
        self.lap_count = msg.data

    def last_lap_callback(self, msg):
        self.last_lap = msg.data

    def best_lap_callback(self, msg):
        self.best_lap = msg.data

    def collision_callback(self, msg):
        if self.collisions is not None and msg.data > self.collisions:
            self.get_logger().warn(f'COLLISION! total collisions = {msg.data}')
        self.collisions = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        if node.record:
            try:
                node.save_recording()
                print('Saved recording to ' + node.rec_path + f' ({len(node.rec)} scans)')
            except Exception as e:
                print('Could not save recording:', e)
        try:
            node.publish(0.0, 0.0)   # leave the car stopped and straight
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
