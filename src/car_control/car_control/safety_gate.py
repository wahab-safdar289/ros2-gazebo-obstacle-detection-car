#!/usr/bin/env python3
"""Safety gate: the last software layer between the obstacle avoider and the wheels.

Subscribes : /cmd_vel_avoid                 (geometry_msgs/Twist)   - what the avoider wants
             /scan                          (sensor_msgs/LaserScan) - independent obstacle check
Publishes  : /diff_drive_controller/cmd_vel (geometry_msgs/TwistStamped) - what the car may do

What it enforces, in priority order:
  1. No fresh, valid lidar data            -> stop immediately
  2. Obstacle inside emergency distance    -> no forward motion (turning in place is still allowed)
  3. Rear blocked                          -> no reverse motion
  4. Stale / missing / non-finite command  -> ramp to a stop
  5. Speed limits, a forward-speed limit that shrinks as an obstacle gets closer,
     and acceleration limits

This node does not import anything from obstacle_avoider on purpose: it has its own
scan processing, so a bug in the avoider cannot also disable the safety check.
"""
import math
from dataclasses import dataclass, fields
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


# ============================================================ configuration
@dataclass
class SafetyConfig:
    # Hard limits (keep at or below the diff_drive_controller limits in controllers.yaml)
    max_linear: float = 0.40             # m/s forward
    max_reverse: float = 0.15            # m/s backward (positive number)
    max_angular: float = 1.2             # rad/s
    max_linear_accel: float = 0.5        # m/s^2 speeding up
    max_linear_decel: float = 1.0        # m/s^2 slowing down
    max_angular_accel: float = 1.5       # rad/s^2

    # Distances measured from the lidar [m]. The bumper is ~0.145 m ahead of the lidar.
    emergency_distance: float = 0.30     # forward motion is blocked at or below this
    caution_distance: float = 0.60       # forward speed limit starts shrinking below this
    rear_emergency_distance: float = 0.30  # reverse motion is blocked at or below this

    # Swept corridor: car_width / 2 + corridor_margin on each side
    car_width: float = 0.27
    corridor_margin: float = 0.10

    # Lidar handling: a return below range_min is treated as an obstacle at range_min
    below_min_is_obstacle: bool = True

    # Watchdogs [s]
    cmd_timeout: float = 0.3             # no command for this long -> ramp to stop
    scan_timeout: float = 0.5            # no scan for this long -> stop immediately


def validate_config(c: SafetyConfig) -> None:
    """Refuse to start with values that would make the gate meaningless or unsafe."""
    if not 0.0 < c.emergency_distance < c.caution_distance:
        raise ValueError('need 0 < emergency_distance < caution_distance')
    if c.rear_emergency_distance <= 0.0:
        raise ValueError('rear_emergency_distance must be > 0')
    if min(c.max_linear, c.max_reverse, c.max_angular) <= 0.0:
        raise ValueError('max_linear, max_reverse and max_angular must be > 0')
    if min(c.max_linear_accel, c.max_linear_decel, c.max_angular_accel) <= 0.0:
        raise ValueError('acceleration limits must be > 0')
    if c.car_width <= 0.0 or c.corridor_margin < 0.0:
        raise ValueError('car_width must be > 0 and corridor_margin >= 0')
    if c.cmd_timeout <= 0.0 or c.scan_timeout <= 0.0:
        raise ValueError('timeouts must be > 0')


# ============================================================ scan processing
def scan_clearances(ranges: List[float], angle_min: float, angle_increment: float,
                    range_min: float, range_max: float,
                    cfg: SafetyConfig) -> Tuple[float, float, bool]:
    """Return (front, rear, valid).

    front / rear = distance along the car's x axis to the nearest return inside the
    swept corridor ahead of / behind the lidar. A direction with no usable samples is
    reported as 0.0 (blocked). valid = False if the scan has no usable samples at all.
    """
    half_w = cfg.car_width / 2.0 + cfg.corridor_margin
    front = rear = range_max
    front_n = rear_n = total_valid = 0

    for i, r in enumerate(ranges):
        if math.isnan(r):
            continue                         # invalid sample, ignore
        if r > range_max:                    # +inf: nothing detected within range
            r = range_max
        elif r < range_min:                  # too close to measure (or -inf)
            if cfg.below_min_is_obstacle:
                r = range_min
            else:
                continue
        total_valid += 1

        a = angle_min + i * angle_increment
        x = r * math.cos(a)
        y = r * math.sin(a)
        if abs(y) > half_w:
            continue
        if x > 0.0:
            front_n += 1
            front = min(front, x)
        elif x < 0.0:
            rear_n += 1
            rear = min(rear, -x)

    return (front if front_n > 0 else 0.0,
            rear if rear_n > 0 else 0.0,
            total_valid > 0)


# ============================================================ safety logic
OK = 'OK'
LIMITING = 'LIMITING'
EMERGENCY_STOP = 'EMERGENCY_STOP'
REAR_BLOCKED = 'REAR_BLOCKED'
NO_SCAN = 'NO_SCAN'
NO_CMD = 'NO_CMD'
CMD_INVALID = 'CMD_INVALID'


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _ramp(current: float, target: float, accel: float, decel: float, dt: float) -> float:
    """Move current toward target without exceeding the acceleration / deceleration limit."""
    speeding_up = abs(target) > abs(current) and current * target >= 0.0
    step = (accel if speeding_up else decel) * dt
    return current + _clamp(target - current, -step, step)


class SafetyLogic:
    """Holds the last output (needed for acceleration limiting) and filters each command."""

    def __init__(self, cfg: SafetyConfig):
        self.cfg = cfg
        self.lin = 0.0
        self.ang = 0.0

    def step(self, cmd_lin: float, cmd_ang: float, front: float, rear: float,
             scan_ok: bool, cmd_ok: bool, dt: float) -> Tuple[float, float, str]:
        """Return (linear_x, angular_z, status) that is safe to send to the controller."""
        c = self.cfg

        # 1. Blind: no fresh, valid scan -> stop immediately, no ramp
        if not scan_ok:
            self.lin = self.ang = 0.0
            return 0.0, 0.0, NO_SCAN

        # 2. Command checks: missing/stale or non-finite -> target is zero
        status = OK
        if not cmd_ok:
            tgt_lin = tgt_ang = 0.0
            status = NO_CMD
        elif not (math.isfinite(cmd_lin) and math.isfinite(cmd_ang)):
            tgt_lin = tgt_ang = 0.0
            status = CMD_INVALID
        else:
            tgt_lin, tgt_ang = cmd_lin, cmd_ang

        # 3. Hard speed limits
        tgt_lin = _clamp(tgt_lin, -c.max_reverse, c.max_linear)
        tgt_ang = _clamp(tgt_ang, -c.max_angular, c.max_angular)

        # 4. Forward obstacle logic
        if tgt_lin > 0.0 or self.lin > 0.0:
            if front <= c.emergency_distance:
                tgt_lin = min(tgt_lin, 0.0)
                self.lin = min(self.lin, 0.0)     # drop forward speed at once
                status = EMERGENCY_STOP
            elif tgt_lin > 0.0 and front < c.caution_distance:
                scale = (front - c.emergency_distance) / (c.caution_distance - c.emergency_distance)
                allowed = c.max_linear * _clamp(scale, 0.0, 1.0)
                if tgt_lin > allowed:
                    tgt_lin = allowed
                    if status == OK:
                        status = LIMITING

        # 5. Reverse obstacle logic
        if (tgt_lin < 0.0 or self.lin < 0.0) and rear <= c.rear_emergency_distance:
            tgt_lin = max(tgt_lin, 0.0)
            self.lin = max(self.lin, 0.0)
            status = REAR_BLOCKED

        # 6. Acceleration limits
        self.lin = _ramp(self.lin, tgt_lin, c.max_linear_accel, c.max_linear_decel, dt)
        self.ang = _ramp(self.ang, tgt_ang, c.max_angular_accel, c.max_angular_accel, dt)
        return self.lin, self.ang, status


# ============================================================ ROS node
class SafetyGate(Node):

    def __init__(self):
        super().__init__('safety_gate')

        # Node-level parameters
        self.declare_parameter('input_topic', 'cmd_vel_avoid')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('output_topic', 'diff_drive_controller/cmd_vel')
        self.declare_parameter('output_frame', 'base_footprint')
        self.declare_parameter('output_stamped', True)
        self.declare_parameter('publish_rate', 50.0)

        # Safety parameters: one ROS parameter per SafetyConfig field.
        # dynamic_typing lets YAML write `1` instead of `1.0` without an error.
        descriptor = ParameterDescriptor(dynamic_typing=True)
        values = {}
        for f in fields(SafetyConfig):
            self.declare_parameter(f.name, f.default, descriptor)
            values[f.name] = type(f.default)(self.get_parameter(f.name).value)
        self.cfg = SafetyConfig(**values)
        validate_config(self.cfg)            # raises ValueError -> node refuses to start

        self.frame = str(self.get_parameter('output_frame').value)
        self.stamped = bool(self.get_parameter('output_stamped').value)
        rate = float(self.get_parameter('publish_rate').value)
        if rate <= 0.0:
            raise ValueError('publish_rate must be > 0')
        self.period = 1.0 / rate

        self.logic = SafetyLogic(self.cfg)

        # State
        self._cmd_lin = 0.0
        self._cmd_ang = 0.0
        self._cmd_time = None                # node-clock time of the last command
        self._front = 0.0
        self._rear = 0.0
        self._scan_valid = False
        self._scan_time = None               # node-clock time of the last scan
        self._last_loop = None
        self._last_status = None

        out_type = TwistStamped if self.stamped else Twist
        self.pub = self.create_publisher(
            out_type, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            Twist, self.get_parameter('input_topic').value, self._cmd_cb, 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self._scan_cb, qos_profile_sensor_data)
        self.create_timer(self.period, self._loop)

        self.get_logger().info(
            f'safety_gate ready: emergency={self.cfg.emergency_distance} m, '
            f'caution={self.cfg.caution_distance} m, max_linear={self.cfg.max_linear} m/s, '
            f'output={"TwistStamped" if self.stamped else "Twist"}. '
            f'Publishing zero until a scan and a command arrive.')

    # ---- helpers
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish(self, linear: float, angular: float) -> None:
        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame
            msg.twist.linear.x = float(linear)
            msg.twist.angular.z = float(angular)
        else:
            msg = Twist()
            msg.linear.x = float(linear)
            msg.angular.z = float(angular)
        self.pub.publish(msg)

    # ---- callbacks
    def _cmd_cb(self, msg: Twist) -> None:
        self._cmd_lin = msg.linear.x
        self._cmd_ang = msg.angular.z
        self._cmd_time = self._now()

    def _scan_cb(self, msg: LaserScan) -> None:
        self._front, self._rear, self._scan_valid = scan_clearances(
            list(msg.ranges), msg.angle_min, msg.angle_increment,
            msg.range_min, msg.range_max, self.cfg)
        self._scan_time = self._now()

    def _loop(self) -> None:
        now = self._now()
        dt = self.period if self._last_loop is None else now - self._last_loop
        self._last_loop = now
        dt = _clamp(dt, 0.0, 0.2)            # guard against sim-time pauses and jumps

        scan_ok = (self._scan_time is not None
                   and self._scan_valid
                   and (now - self._scan_time) <= self.cfg.scan_timeout)
        cmd_ok = (self._cmd_time is not None
                  and (now - self._cmd_time) <= self.cfg.cmd_timeout)

        lin, ang, status = self.logic.step(
            self._cmd_lin, self._cmd_ang, self._front, self._rear,
            scan_ok, cmd_ok, dt)
        self._publish(lin, ang)

        if status != self._last_status:
            self.get_logger().info(
                f'Status -> {status} (front={self._front:.2f} m, rear={self._rear:.2f} m)')
            self._last_status = status
        if status in (EMERGENCY_STOP, NO_SCAN, REAR_BLOCKED):
            self.get_logger().warning(
                f'{status}: front={self._front:.2f} m, rear={self._rear:.2f} m',
                throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            for _ in range(3):               # last commands are always "stop"
                node._publish(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
