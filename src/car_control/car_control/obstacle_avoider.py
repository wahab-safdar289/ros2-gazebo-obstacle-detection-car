#!/usr/bin/env python3
"""Lidar-based obstacle avoider.

Subscribes : /scan            (sensor_msgs/LaserScan)
Publishes  : /cmd_vel_avoid   (geometry_msgs/Twist)

Convention: angle 0 = straight ahead, positive angle = left (counter-clockwise),
positive angular.z = turn left. All distances are measured from the lidar.

The decision logic (analyze_scan + AvoiderLogic) has no ROS dependency apart
from the imports at the top, so it can be unit-tested with fake scans.
"""
import math
from dataclasses import dataclass, fields
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import LaserScan


# ============================================================ configuration
@dataclass
class AvoiderConfig:
    # Speeds (keep at or below the diff_drive_controller limits)
    max_speed: float = 0.35          # m/s, free road
    min_speed: float = 0.10          # m/s, creeping speed just before the stop distance
    reverse_speed: float = 0.10      # m/s, used only for recovery
    turn_speed: float = 0.8          # rad/s

    # Distances ahead of the lidar [m]
    stop_distance: float = 0.45      # stop and turn (the bumper is ~0.145 m ahead of the lidar)
    slow_distance: float = 1.20      # start slowing and steering away
    clear_margin: float = 0.15       # must see stop_distance + margin free to leave TURN
    side_clearance: float = 0.35     # steer away from walls closer than this

    # Corridor the car sweeps: car_width / 2 + corridor_margin on each side
    car_width: float = 0.27
    corridor_margin: float = 0.10

    # Angular sectors [deg]
    side_sector_start_deg: float = 15.0
    side_sector_end_deg: float = 90.0
    rear_half_angle_deg: float = 30.0

    # Steering
    side_gain: float = 1.0
    turn_deadband: float = 0.15      # [m] left/right free-space difference needed to change turn side

    # Recovery
    turn_timeout: float = 8.0        # [s] turning in place without finding a way out
    reverse_duration: float = 1.5    # [s]
    reverse_clearance: float = 0.60  # [m] rear must be at least this free to start reversing
    reverse_stop_distance: float = 0.35  # [m] stop reversing when the rear gets this close

    # Lidar handling: a return below range_min is treated as an obstacle at range_min
    below_min_is_obstacle: bool = True


def validate_config(c: AvoiderConfig) -> None:
    """Fail fast on parameter values that would make the car unsafe or the logic meaningless."""
    if not 0.0 < c.stop_distance < c.slow_distance:
        raise ValueError('need 0 < stop_distance < slow_distance')
    if not 0.0 < c.min_speed <= c.max_speed:
        raise ValueError('need 0 < min_speed <= max_speed')
    if c.reverse_speed <= 0.0 or c.turn_speed <= 0.0:
        raise ValueError('reverse_speed and turn_speed must be positive')
    if c.car_width <= 0.0 or c.corridor_margin < 0.0:
        raise ValueError('car_width must be > 0 and corridor_margin >= 0')
    if not 0.0 <= c.side_sector_start_deg < c.side_sector_end_deg <= 180.0:
        raise ValueError('need 0 <= side_sector_start_deg < side_sector_end_deg <= 180')
    if c.turn_timeout <= 0.0 or c.reverse_duration <= 0.0:
        raise ValueError('turn_timeout and reverse_duration must be positive')


# ============================================================ scan analysis
@dataclass
class Sectors:
    front: float       # nearest obstacle inside the driving corridor, measured along x [m]
    left_min: float    # nearest return in the left side sector [m]
    right_min: float
    left_free: float   # mean range in the left side sector (how open it is) [m]
    right_free: float
    rear: float        # nearest return behind the car [m]
    valid: bool        # False if the scan has no usable samples at all


def analyze_scan(ranges: List[float], angle_min: float, angle_increment: float,
                 range_min: float, range_max: float, cfg: AvoiderConfig) -> Sectors:
    half_w = cfg.car_width / 2.0 + cfg.corridor_margin
    side_lo = math.radians(cfg.side_sector_start_deg)
    side_hi = math.radians(cfg.side_sector_end_deg)
    rear_half = math.radians(cfg.rear_half_angle_deg)

    front = range_max
    left_min = right_min = rear = range_max
    left_vals: List[float] = []
    right_vals: List[float] = []
    front_n = rear_n = total_valid = 0

    for i, r in enumerate(ranges):
        if math.isnan(r):
            continue                       # invalid sample, ignore
        if r > range_max:                  # +inf: nothing detected within range
            r = range_max
        elif r < range_min:                # too close to measure (or -inf)
            if cfg.below_min_is_obstacle:
                r = range_min
            else:
                continue
        total_valid += 1

        a = angle_min + i * angle_increment
        a = math.atan2(math.sin(a), math.cos(a))      # wrap to [-pi, pi]
        x = r * math.cos(a)
        y = r * math.sin(a)

        # Driving corridor: points ahead of the lidar inside the car's swept width
        if x > 0.0 and abs(y) <= half_w:
            front_n += 1
            front = min(front, x)

        if side_lo <= a <= side_hi:
            left_vals.append(r)
            left_min = min(left_min, r)
        elif -side_hi <= a <= -side_lo:
            right_vals.append(r)
            right_min = min(right_min, r)

        if abs(a) >= math.pi - rear_half:
            rear_n += 1
            rear = min(rear, r)

    def mean(v: List[float]) -> float:
        return sum(v) / len(v) if v else 0.0       # no data = treated as blocked

    return Sectors(
        front=front if front_n > 0 else 0.0,       # no data ahead = treated as blocked
        left_min=left_min if left_vals else 0.0,
        right_min=right_min if right_vals else 0.0,
        left_free=mean(left_vals),
        right_free=mean(right_vals),
        rear=rear if rear_n > 0 else 0.0,
        valid=total_valid > 0,
    )


# ============================================================ decision logic
CRUISE = 'CRUISE'
TURN = 'TURN'
REVERSE = 'REVERSE'
HALT = 'HALT'


class AvoiderLogic:
    """Small state machine: CRUISE -> TURN -> (REVERSE) -> CRUISE, with HALT as the safe state."""

    def __init__(self, cfg: AvoiderConfig):
        self.cfg = cfg
        self.state = HALT              # start stopped until the first valid scan
        self.reason = 'waiting for first scan'
        self.turn_dir = 1.0            # +1 = left, -1 = right
        self._t0 = 0.0                 # start time of the current TURN / REVERSE

    # ---- helpers
    def _set(self, state: str, reason: str = '') -> None:
        self.state = state
        self.reason = reason

    def halt(self, reason: str) -> None:
        self._set(HALT, reason)

    def _pick_dir(self, s: Sectors) -> float:
        diff = s.left_free - s.right_free
        if abs(diff) > self.cfg.turn_deadband:
            self.turn_dir = 1.0 if diff > 0.0 else -1.0
        return self.turn_dir

    def _start_turn(self, s: Sectors, now: float) -> None:
        self._pick_dir(s)
        self._t0 = now
        self._set(TURN, 'obstacle ahead')

    def _start_reverse(self, now: float) -> None:
        self._t0 = now
        self._set(REVERSE, 'stuck, backing up')

    # ---- main entry point
    def update(self, s: Sectors, now: float) -> Tuple[float, float, str]:
        """Return (linear_x, angular_z, state)."""
        c = self.cfg
        clear_at = c.stop_distance + c.clear_margin

        if not s.valid:
            self._set(HALT, 'no valid lidar samples')
            return 0.0, 0.0, self.state

        if self.state == HALT:
            if s.front >= clear_at:
                self._set(CRUISE, 'path clear')
            elif s.rear >= c.reverse_clearance:
                self._start_reverse(now)
            else:
                return 0.0, 0.0, HALT      # boxed in: stay stopped until something changes

        if self.state == REVERSE:
            if s.rear <= c.reverse_stop_distance or now - self._t0 >= c.reverse_duration:
                self._start_turn(s, now)   # fall through into TURN below
            else:
                return -c.reverse_speed, 0.0, REVERSE

        if self.state == TURN:
            if s.front >= clear_at:
                self._set(CRUISE, 'path clear')
            elif now - self._t0 > c.turn_timeout:
                if s.rear >= c.reverse_clearance:
                    self._start_reverse(now)
                    return -c.reverse_speed, 0.0, REVERSE
                self._set(HALT, 'turn timed out and rear is blocked')
                return 0.0, 0.0, HALT
            else:
                return 0.0, self.turn_dir * c.turn_speed, TURN

        # ---- CRUISE
        if s.front <= c.stop_distance:
            self._start_turn(s, now)
            return 0.0, self.turn_dir * c.turn_speed, TURN

        if s.front < c.slow_distance:
            factor = (s.front - c.stop_distance) / (c.slow_distance - c.stop_distance)
            linear = c.min_speed + factor * (c.max_speed - c.min_speed)
            angular = self._pick_dir(s) * c.turn_speed * (1.0 - factor)
            return linear, angular, CRUISE

        # Free road: full speed, nudge away from nearby side walls
        push = 0.0
        if s.left_min < c.side_clearance:
            push -= (c.side_clearance - s.left_min) / c.side_clearance
        if s.right_min < c.side_clearance:
            push += (c.side_clearance - s.right_min) / c.side_clearance
        angular = max(-c.turn_speed, min(c.turn_speed, c.side_gain * push))
        return c.max_speed, angular, CRUISE


# ============================================================ ROS node
class ObstacleAvoider(Node):

    def __init__(self):
        super().__init__('obstacle_avoider')

        # Node-level parameters
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('cmd_topic', 'cmd_vel_avoid')
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('scan_timeout', 0.5)

        # Logic parameters: one ROS parameter per AvoiderConfig field.
        # dynamic_typing lets YAML write `8` instead of `8.0` without an error.
        descriptor = ParameterDescriptor(dynamic_typing=True)
        values = {}
        for f in fields(AvoiderConfig):
            self.declare_parameter(f.name, f.default, descriptor)
            values[f.name] = type(f.default)(self.get_parameter(f.name).value)
        self.cfg = AvoiderConfig(**values)
        validate_config(self.cfg)          # raises ValueError -> node refuses to start

        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        rate = float(self.get_parameter('control_rate').value)

        self.logic = AvoiderLogic(self.cfg)
        self._scan = None
        self._scan_time = 0.0
        self._last_state = None

        self.pub = self.create_publisher(
            Twist, self.get_parameter('cmd_topic').value, 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self._scan_cb, qos_profile_sensor_data)
        self.create_timer(1.0 / rate, self._loop)

        self.get_logger().info(
            f'obstacle_avoider ready: stop={self.cfg.stop_distance} m, '
            f'slow={self.cfg.slow_distance} m, max_speed={self.cfg.max_speed} m/s. '
            f'Holding still until the first lidar scan arrives.')

    # ---- helpers
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub.publish(msg)

    def publish_stop(self) -> None:
        self._publish(0.0, 0.0)

    # ---- callbacks
    def _scan_cb(self, msg: LaserScan) -> None:
        self._scan = msg
        self._scan_time = self._now()

    def _loop(self) -> None:
        now = self._now()

        # Safety: no scan, or an old scan -> never drive on stale data
        if self._scan is None or (now - self._scan_time) > self.scan_timeout:
            self.logic.halt('lidar scan missing or stale')
            self.publish_stop()
            self.get_logger().warning('No fresh /scan data: holding the car stopped.',
                                   throttle_duration_sec=2.0)
            self._report(HALT)
            return

        scan = self._scan
        sectors = analyze_scan(
            list(scan.ranges), scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max, self.cfg)

        linear, angular, state = self.logic.update(sectors, now)
        self._publish(linear, angular)
        self._report(state)

        self.get_logger().info(
            f'[{state}] front={sectors.front:.2f} m  left_free={sectors.left_free:.2f}  '
            f'right_free={sectors.right_free:.2f}  rear={sectors.rear:.2f}  '
            f'-> v={linear:.2f} m/s  w={angular:.2f} rad/s',
            throttle_duration_sec=2.0)

    def _report(self, state: str) -> None:
        if state != self._last_state:
            self.get_logger().info(f'State -> {state} ({self.logic.reason})')
            self._last_state = state


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.publish_stop()            # last command is always "stop"
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
