#!/usr/bin/env python3
"""
Auto-tuner for wall_follow_node, run against the real AutoDRIVE simulator.

It hill-climbs the controller's parameters one at a time:
  1. Measure the current best settings (baseline).
  2. Nudge one parameter up or down, reset the car, drive WARMUP + LAPS laps.
  3. Keep the change only if every lap was clean (no collisions) AND the mean
     lap time beat the best by MIN_GAIN seconds, confirmed by a second run.
  4. Repeat over all parameters, in passes, until stopped (Ctrl+C) or MAX_TRIALS.

Every trial is appended to logs/autotune_results.jsonl, and the best settings
so far are written to logs/autotune_best.json.

Usage (with the bridge, the simulator in Autonomous mode, and wall_follow_node
all running), in another container terminal:
  python3 /home/autodrive_devkit/src/autodrive_devkit/tools/autotune.py
"""

import json
import os
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Bool, Float32, Int32
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters

LOG_DIR = '/home/autodrive_devkit/src/autodrive_devkit/logs'
TARGET = '/wall_follow_node'

WARMUP = 1          # laps ignored after each reset (standing start)
LAPS = 4            # measured laps per trial
MIN_GAIN = 0.06     # s, a change must beat the best mean lap by at least this
LAP_TIMEOUT = 40.0  # s, give up on a trial if a lap takes longer than this
MAX_TRIALS = 400

# Starting point = the 11.98 s settings. (name: (start, step, min, max))
SPACE = {
    'v_corner':           (2.0,  0.25, 1.2, 4.0),
    'a_lat':              (4.0,  0.75, 2.0, 12.0),
    'a_brake':            (3.0,  0.5,  1.5, 8.0),
    'stop_margin':        (1.0,  0.15, 0.4, 1.6),
    'corner_angle_deg':   (60.0, 10.0, 25.0, 90.0),
    'v_min':              (1.2,  0.2,  0.8, 2.5),
    'gap_margin':         (0.25, 0.05, 0.10, 0.40),
    'gap_full_angle_deg': (35.0, 7.0,  15.0, 60.0),
    'corner_front_dist':  (2.0,  0.3,  1.0, 3.5),
    'outside_steer':      (0.0,  0.05, 0.0, 0.30),
    'outside_dist':       (3.5,  0.5,  2.0, 6.0),
    'kp':                 (0.35, 0.1,  0.1, 0.9),
    'v_max':              (8.0,  1.0,  5.0, 12.0),
}

QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)


class AutoTuner(Node):
    def __init__(self):
        super().__init__('autotune')
        self.pub_reset = self.create_publisher(Bool, '/autodrive/reset_command', QOS)
        self.create_subscription(Int32, '/autodrive/roboracer_1/lap_count', self.on_lap, QOS)
        self.create_subscription(Float32, '/autodrive/roboracer_1/last_lap_time', self.on_last, QOS)
        self.create_subscription(Int32, '/autodrive/roboracer_1/collision_count', self.on_col, QOS)
        self.cli = self.create_client(SetParameters, TARGET + '/set_parameters')
        self.lap_count = None
        self.last_lap = None
        self.collisions = None
        self.lap_events = []        # (time, lap_time) appended on each new lap
        os.makedirs(LOG_DIR, exist_ok=True)
        self.results_path = os.path.join(LOG_DIR, 'autotune_results.jsonl')
        self.best_path = os.path.join(LOG_DIR, 'autotune_best.json')

    # --- topic callbacks ---
    def on_last(self, msg):
        self.last_lap = float(msg.data)

    def on_lap(self, msg):
        if self.lap_count is not None and msg.data > self.lap_count:
            # last_lap_time may arrive a moment later; read it after a short wait
            self.lap_events.append(time.time())
        self.lap_count = msg.data

    def on_col(self, msg):
        self.collisions = msg.data

    # --- helpers ---
    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def set_params(self, params):
        req = SetParameters.Request()
        for k, v in params.items():
            req.parameters.append(Parameter(name=k, value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(v))))
        fut = self.cli.call_async(req)
        end = time.time() + 5.0
        while not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not fut.done():
            raise RuntimeError('wall_follow_node did not answer set_parameters. Is it running?')

    def reset_car(self):
        for _ in range(10):
            self.pub_reset.publish(Bool(data=True))
            self.spin_for(0.05)
        for _ in range(10):
            self.pub_reset.publish(Bool(data=False))
            self.spin_for(0.05)
        self.spin_for(1.0)

    def run_trial(self, params):
        """Returns (lap_times, collided)."""
        self.set_params(params)
        self.reset_car()
        self.lap_events.clear()
        col0 = self.collisions
        laps = []
        last_mark = time.time()
        while len(laps) < WARMUP + LAPS and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if col0 is not None and self.collisions is not None and self.collisions > col0:
                return laps[WARMUP:], True
            if self.lap_events:
                self.lap_events.pop(0)
                self.spin_for(0.3)              # let last_lap_time update
                laps.append(self.last_lap)
                last_mark = time.time()
            if time.time() - last_mark > LAP_TIMEOUT:
                return laps[WARMUP:], True      # stuck or lost: treat as a failure
        return laps[WARMUP:], False

    def log(self, rec):
        with open(self.results_path, 'a') as f:
            f.write(json.dumps(rec) + '\n')

    # --- search ---
    def main(self):
        self.get_logger().info('Waiting for wall_follow_node and lap topics...')
        if not self.cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('wall_follow_node is not running (no set_parameters service).')
        while self.lap_count is None and rclpy.ok():
            self.spin_for(0.5)
            self.get_logger().info('...waiting for /autodrive/roboracer_1/lap_count (is the simulator connected and in Autonomous?)')

        best = {k: v[0] for k, v in SPACE.items()}
        best_mean = None
        for attempt in range(3):
            laps, col = self.run_trial(best)
            self.log(dict(t=time.time(), kind='baseline', params=best, laps=laps, collided=col))
            if not col and len(laps) == LAPS:
                best_mean = sum(laps) / len(laps)
                break
        if best_mean is None:
            raise RuntimeError('Baseline settings could not finish clean laps. Check the setup.')
        self.get_logger().info(f'BASELINE mean lap {best_mean:.3f} s  laps={laps}')
        json.dump(dict(mean_lap=best_mean, params=best), open(self.best_path, 'w'), indent=2)

        trials = 0
        names = list(SPACE)
        while trials < MAX_TRIALS and rclpy.ok():
            random.shuffle(names)
            improved_any = False
            for name in names:
                start, step, lo, hi = SPACE[name]
                for direction in (+1, -1):
                    val = round(min(max(best[name] + direction * step, lo), hi), 4)
                    if val == best[name]:
                        continue
                    cand = dict(best, **{name: val})
                    laps, col = self.run_trial(cand)
                    trials += 1
                    mean = sum(laps) / len(laps) if laps else None
                    self.log(dict(t=time.time(), kind='trial', change={name: val}, params=cand,
                                  laps=laps, collided=col, mean=mean))
                    tag = 'COLLISION' if col else f'mean {mean:.3f}'
                    self.get_logger().info(f'trial {trials}: {name}={val}  -> {tag}  (best {best_mean:.3f})')
                    if col or mean is None or mean > best_mean - MIN_GAIN:
                        continue
                    # Confirm with a second run before accepting.
                    laps2, col2 = self.run_trial(cand)
                    trials += 1
                    allc = laps + laps2
                    mean2 = sum(allc) / len(allc) if allc else None
                    self.log(dict(t=time.time(), kind='confirm', change={name: val}, params=cand,
                                  laps=laps2, collided=col2, mean=mean2))
                    if col2 or mean2 is None or mean2 > best_mean - MIN_GAIN / 2:
                        self.get_logger().info(f'   not confirmed ({"COLLISION" if col2 else f"{mean2:.3f}"})')
                        continue
                    best, best_mean, improved_any = cand, mean2, True
                    json.dump(dict(mean_lap=best_mean, params=best), open(self.best_path, 'w'), indent=2)
                    self.get_logger().info(f'*** NEW BEST {best_mean:.3f} s with {name}={val}')
                    break   # move on to the next parameter
            if not improved_any:
                for k in SPACE:   # nothing helped: try smaller steps next pass
                    s, st, lo, hi = SPACE[k]
                    SPACE[k] = (s, st / 2.0, lo, hi)
                self.get_logger().info('No improvement this pass; halving step sizes.')
        self.get_logger().info(f'Done. Best mean lap {best_mean:.3f} s: {best}')


def main():
    rclpy.init()
    node = AutoTuner()
    try:
        node.main()
    except KeyboardInterrupt:
        pass
    finally:
        print('Results in', node.results_path, 'and', node.best_path)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
