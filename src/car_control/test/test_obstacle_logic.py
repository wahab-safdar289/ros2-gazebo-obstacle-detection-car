"""Unit tests for the car_control decision logic. No Gazebo and no running ROS nodes needed.

Covers:
  * obstacle_avoider : scan analysis, the state machine and its config validation
  * safety_gate      : scan clearances, the safety filter and its config validation
  * the YAML files   : misspelled keys, valid values, limits consistent across layers

Scan convention used by the helpers (same as the lidar in sensors.xacro):
  360 samples from -180 to +179 degrees, 0 deg = straight ahead, positive = left.
  Sample index = degrees + 180.

Run (ROS workspace must be sourced because the modules import rclpy):
  python3 -m pytest src/car_control/test/test_obstacle_logic.py -v
"""
import dataclasses
import math
import os

import pytest

from car_control import obstacle_avoider as oa
from car_control import safety_gate as sg

N = 360
ANGLE_MIN = -math.pi
INC = 2.0 * math.pi / N
RANGE_MIN = 0.12
RANGE_MAX = 8.0
INF = math.inf
NAN = math.nan

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS_FILE = os.path.join(HERE, '..', 'config', 'car_control_params.yaml')
CONTROLLERS_FILE = os.path.join(
    HERE, '..', '..', 'car_description', 'config', 'controllers.yaml')


# ============================================================ scan helpers
def empty_scan():
    """Nothing detected anywhere (the lidar reports +inf)."""
    return [INF] * N


def idx(deg):
    return (deg + 180) % N


def set_range(ranges, deg_lo, deg_hi, value):
    for d in range(deg_lo, deg_hi + 1):
        ranges[idx(d)] = value


def add_flat_wall(ranges, dist, center_deg, half_width_deg):
    """Flat wall `dist` metres away, perpendicular to the direction `center_deg`."""
    for k in range(-half_width_deg, half_width_deg + 1):
        ranges[idx(center_deg + k)] = dist / math.cos(math.radians(k))


def add_side_wall(ranges, y_dist, side, deg_lo, deg_hi):
    """Wall parallel to the driving direction, y_dist to the left (+1) or right (-1)."""
    for d in range(deg_lo, deg_hi + 1):
        ranges[idx(side * d)] = y_dist / math.sin(math.radians(d))


def analyze(ranges, cfg=None):
    return oa.analyze_scan(ranges, ANGLE_MIN, INC, RANGE_MIN, RANGE_MAX,
                           cfg or oa.AvoiderConfig())


def clearances(ranges, cfg=None):
    return sg.scan_clearances(ranges, ANGLE_MIN, INC, RANGE_MIN, RANGE_MAX,
                              cfg or sg.SafetyConfig())


def sectors(front=8.0, left_min=8.0, right_min=8.0, left_free=8.0,
            right_free=8.0, rear=8.0, valid=True):
    return oa.Sectors(front=front, left_min=left_min, right_min=right_min,
                      left_free=left_free, right_free=right_free,
                      rear=rear, valid=valid)


def cruising_logic(cfg=None):
    """An AvoiderLogic that has already left HALT and is cruising on a free road."""
    logic = oa.AvoiderLogic(cfg or oa.AvoiderConfig())
    _, _, state = logic.update(sectors(), 0.0)
    assert state == oa.CRUISE
    return logic


def settle(logic, lin, ang, front=8.0, rear=8.0, scan_ok=True, cmd_ok=True,
           steps=300, dt=0.02):
    """Feed the same inputs to a SafetyLogic for a while and return the last output."""
    out = None
    for _ in range(steps):
        out = logic.step(lin, ang, front, rear, scan_ok, cmd_ok, dt)
    return out


# ============================================================ avoider: analyze_scan
def test_empty_scan_is_all_clear():
    s = analyze(empty_scan())
    assert s.valid
    assert s.front > 7.9 and s.rear > 7.9
    assert s.left_free == pytest.approx(RANGE_MAX)
    assert s.right_free == pytest.approx(RANGE_MAX)


def test_wall_ahead_is_measured_along_the_driving_direction():
    ranges = empty_scan()
    add_flat_wall(ranges, 1.0, 0, 40)
    assert analyze(ranges).front == pytest.approx(1.0, abs=1e-6)


def test_obstacle_on_the_side_is_not_counted_as_ahead():
    ranges = empty_scan()
    set_range(ranges, 85, 95, 0.3)            # 0.3 m to the left: outside the corridor
    s = analyze(ranges)
    assert s.front > 7.0
    assert s.left_min == pytest.approx(0.3)


def test_gap_wider_than_the_car_is_open():
    ranges = empty_scan()                     # 0.8 m gate: walls 0.4 m either side
    add_side_wall(ranges, 0.4, +1, 10, 85)
    add_side_wall(ranges, 0.4, -1, 10, 85)
    s = analyze(ranges)
    assert s.front > 7.0
    assert s.left_min < 0.5 and s.right_min < 0.5


def test_gap_narrower_than_the_car_is_blocked():
    ranges = empty_scan()                     # walls 0.2 m either side: car cannot fit
    add_side_wall(ranges, 0.2, +1, 10, 85)
    add_side_wall(ranges, 0.2, -1, 10, 85)
    assert analyze(ranges).front < 0.1


def test_rear_obstacle_is_detected():
    ranges = empty_scan()
    add_flat_wall(ranges, 0.5, 180, 20)
    s = analyze(ranges)
    assert s.rear == pytest.approx(0.5, abs=1e-6)
    assert s.front > 7.0


def test_more_open_side_has_larger_free_space():
    ranges = empty_scan()
    set_range(ranges, 20, 80, 0.5)            # clutter on the left only
    s = analyze(ranges)
    assert s.left_free < s.right_free
    assert s.right_free == pytest.approx(RANGE_MAX)


def test_all_nan_scan_is_invalid_and_blocked():
    s = analyze([NAN] * N)
    assert not s.valid
    assert s.front == 0.0 and s.rear == 0.0


def test_no_data_ahead_is_treated_as_blocked():
    ranges = empty_scan()
    set_range(ranges, -3, 3, NAN)             # corridor ahead has only invalid samples
    s = analyze(ranges)
    assert s.valid
    assert s.front == 0.0


@pytest.mark.parametrize('bad_value', [0.05, -INF])
def test_return_below_range_min_counts_as_an_obstacle(bad_value):
    ranges = empty_scan()
    set_range(ranges, -3, 3, bad_value)
    assert analyze(ranges).front < 0.13


# ============================================================ avoider: state machine
def test_starts_halted_and_stays_stopped_without_a_valid_scan():
    logic = oa.AvoiderLogic(oa.AvoiderConfig())
    assert logic.state == oa.HALT
    assert logic.update(sectors(valid=False), 0.0) == (0.0, 0.0, oa.HALT)


def test_halt_becomes_cruise_when_the_path_is_clear():
    cfg = oa.AvoiderConfig()
    logic = oa.AvoiderLogic(cfg)
    lin, ang, state = logic.update(sectors(), 0.0)
    assert state == oa.CRUISE
    assert lin == pytest.approx(cfg.max_speed)
    assert ang == pytest.approx(0.0)


def test_slows_down_and_steers_between_slow_and_stop_distance():
    cfg = oa.AvoiderConfig()
    logic = cruising_logic(cfg)
    mid = (cfg.stop_distance + cfg.slow_distance) / 2.0        # factor = 0.5
    lin, ang, state = logic.update(
        sectors(front=mid, left_free=2.0, right_free=1.0), 0.1)
    assert state == oa.CRUISE
    assert lin == pytest.approx(cfg.min_speed + 0.5 * (cfg.max_speed - cfg.min_speed))
    assert ang == pytest.approx(0.5 * cfg.turn_speed)          # steering left


def test_stops_and_turns_toward_the_more_open_side():
    cfg = oa.AvoiderConfig()
    logic = cruising_logic(cfg)
    lin, ang, state = logic.update(
        sectors(front=0.40, left_free=3.0, right_free=1.0), 1.0)
    assert (state, lin) == (oa.TURN, 0.0)
    assert ang == pytest.approx(cfg.turn_speed)                # left

    logic = cruising_logic(cfg)
    lin, ang, state = logic.update(
        sectors(front=0.40, left_free=1.0, right_free=3.0), 1.0)
    assert (state, lin) == (oa.TURN, 0.0)
    assert ang == pytest.approx(-cfg.turn_speed)               # right


def test_turn_direction_does_not_flip_inside_the_deadband():
    cfg = oa.AvoiderConfig()
    logic = cruising_logic(cfg)
    logic.turn_dir = -1.0
    # left is only 0.1 m more open: below turn_deadband (0.15), so keep turning right
    _, ang, state = logic.update(
        sectors(front=0.40, left_free=2.0, right_free=1.9), 1.0)
    assert state == oa.TURN
    assert ang == pytest.approx(-cfg.turn_speed)


def test_keeps_turning_until_stop_distance_plus_margin_is_free():
    cfg = oa.AvoiderConfig()
    logic = cruising_logic(cfg)
    logic.update(sectors(front=0.40, left_free=3.0, right_free=1.0), 1.0)

    # 0.55 m is past stop_distance but below stop + clear_margin (0.60): still turning
    lin, ang, state = logic.update(
        sectors(front=0.55, left_free=3.0, right_free=1.0), 1.1)
    assert state == oa.TURN
    assert lin == 0.0 and ang == pytest.approx(cfg.turn_speed)

    lin, _, state = logic.update(sectors(front=0.65), 1.2)
    assert state == oa.CRUISE
    assert lin > 0.0


def test_pushes_away_from_a_nearby_side_wall_on_a_free_road():
    cfg = oa.AvoiderConfig()
    expected = (cfg.side_clearance - 0.2) / cfg.side_clearance

    lin, ang, _ = cruising_logic(cfg).update(sectors(left_min=0.2), 0.1)
    assert lin == pytest.approx(cfg.max_speed)
    assert ang == pytest.approx(-expected)                     # wall on the left: steer right

    _, ang, _ = cruising_logic(cfg).update(sectors(right_min=0.2), 0.1)
    assert ang == pytest.approx(expected)                      # wall on the right: steer left


def test_dead_end_turn_timeout_leads_to_reverse_then_a_new_turn():
    cfg = oa.AvoiderConfig()
    logic = cruising_logic(cfg)
    logic.update(sectors(front=0.30, rear=2.0), 0.0)           # TURN starts at t = 0

    lin, ang, state = logic.update(sectors(front=0.30, rear=2.0), cfg.turn_timeout + 1.0)
    t0 = cfg.turn_timeout + 1.0
    assert state == oa.REVERSE
    assert lin == pytest.approx(-cfg.reverse_speed) and ang == 0.0

    _, _, state = logic.update(sectors(front=0.30, rear=2.0), t0 + 0.5)
    assert state == oa.REVERSE                                 # still backing up

    lin, _, state = logic.update(
        sectors(front=0.30, rear=2.0), t0 + cfg.reverse_duration + 0.1)
    assert (state, lin) == (oa.TURN, 0.0)                      # back to turning


def test_turn_timeout_with_blocked_rear_halts():
    cfg = oa.AvoiderConfig()
    logic = cruising_logic(cfg)
    logic.update(sectors(front=0.30, rear=0.2), 0.0)
    out = logic.update(sectors(front=0.30, rear=0.2), cfg.turn_timeout + 1.0)
    assert out == (0.0, 0.0, oa.HALT)


def test_boxed_in_stays_halted_until_something_opens_up():
    logic = oa.AvoiderLogic(oa.AvoiderConfig())
    assert logic.update(sectors(front=0.3, rear=0.2), 0.0) == (0.0, 0.0, oa.HALT)
    assert logic.update(sectors(front=0.3, rear=0.2), 0.1) == (0.0, 0.0, oa.HALT)
    _, _, state = logic.update(sectors(front=3.0, rear=0.2), 0.2)
    assert state == oa.CRUISE


def test_halt_with_blocked_front_but_clear_rear_backs_up():
    cfg = oa.AvoiderConfig()
    logic = oa.AvoiderLogic(cfg)
    lin, _, state = logic.update(sectors(front=0.3, rear=2.0), 0.0)
    assert state == oa.REVERSE
    assert lin == pytest.approx(-cfg.reverse_speed)


def test_reversing_stops_when_the_rear_gets_close():
    cfg = oa.AvoiderConfig()
    logic = oa.AvoiderLogic(cfg)
    logic.update(sectors(front=0.3, rear=2.0), 0.0)            # HALT -> REVERSE
    lin, _, state = logic.update(
        sectors(front=0.3, rear=cfg.reverse_stop_distance - 0.05), 0.1)
    assert (state, lin) == (oa.TURN, 0.0)


def test_invalid_scan_halts_a_cruising_car_and_recovers_afterwards():
    logic = cruising_logic()
    assert logic.update(sectors(valid=False), 1.0) == (0.0, 0.0, oa.HALT)
    _, _, state = logic.update(sectors(), 1.1)
    assert state == oa.CRUISE


def test_output_never_exceeds_the_configured_speeds():
    cfg = oa.AvoiderConfig()
    logic = oa.AvoiderLogic(cfg)
    t = 0.0
    for front in (8.0, 5.0, 2.0, 1.2, 1.0, 0.8, 0.6, 0.5, 0.45, 0.3, 0.1, 0.0, 3.0):
        for rear in (8.0, 0.5, 0.1):
            for left, right in ((8.0, 1.0), (1.0, 8.0), (0.0, 0.0)):
                t += 0.1
                lin, ang, _ = logic.update(
                    sectors(front=front, rear=rear, left_min=left, right_min=right,
                            left_free=left, right_free=right), t)
                assert -cfg.reverse_speed <= lin <= cfg.max_speed
                assert abs(ang) <= cfg.turn_speed + 1e-9


def test_end_to_end_scan_to_command_turns_away_from_the_cluttered_side():
    ranges = empty_scan()
    add_flat_wall(ranges, 0.4, 0, 40)         # wall ahead
    set_range(ranges, 20, 80, 0.5)            # extra clutter on the left
    lin, ang, state = cruising_logic().update(analyze(ranges), 1.0)
    assert state == oa.TURN
    assert lin == 0.0
    assert ang < 0.0                          # turns right, toward the open side


# ============================================================ gate: scan_clearances
def test_gate_clear_scan():
    front, rear, valid = clearances(empty_scan())
    assert valid
    assert front > 7.9 and rear > 7.9


def test_gate_front_obstacle():
    ranges = empty_scan()
    add_flat_wall(ranges, 0.5, 0, 40)
    front, _, _ = clearances(ranges)
    assert front == pytest.approx(0.5, abs=1e-6)


def test_gate_rear_obstacle():
    ranges = empty_scan()
    add_flat_wall(ranges, 0.4, 180, 40)
    front, rear, _ = clearances(ranges)
    assert rear == pytest.approx(0.4, abs=1e-6)
    assert front > 7.9


def test_gate_all_nan_scan_is_invalid():
    front, rear, valid = clearances([NAN] * N)
    assert not valid
    assert front == 0.0 and rear == 0.0


def test_gate_no_data_ahead_is_treated_as_blocked():
    ranges = empty_scan()
    set_range(ranges, -3, 3, NAN)
    front, rear, valid = clearances(ranges)
    assert valid
    assert front == 0.0
    assert rear > 7.9


# ============================================================ gate: SafetyLogic
def test_gate_no_scan_stops_immediately():
    logic = sg.SafetyLogic(sg.SafetyConfig())
    settle(logic, 0.3, 0.0)
    assert logic.lin == pytest.approx(0.3)
    lin, ang, status = logic.step(0.3, 0.0, 8.0, 8.0, False, True, 0.02)
    assert (lin, ang, status) == (0.0, 0.0, sg.NO_SCAN)
    assert logic.lin == 0.0


def test_gate_limits_acceleration():
    cfg = sg.SafetyConfig()
    lin, _, status = sg.SafetyLogic(cfg).step(0.4, 0.0, 8.0, 8.0, True, True, 0.02)
    assert lin == pytest.approx(cfg.max_linear_accel * 0.02)
    assert status == sg.OK


def test_gate_clamps_speeds_to_the_hard_limits():
    cfg = sg.SafetyConfig()
    lin, ang, status = settle(sg.SafetyLogic(cfg), 5.0, 10.0)
    assert lin == pytest.approx(cfg.max_linear)
    assert ang == pytest.approx(cfg.max_angular)
    assert status == sg.OK

    lin, _, _ = settle(sg.SafetyLogic(cfg), -5.0, 0.0)
    assert lin == pytest.approx(-cfg.max_reverse)


def test_gate_emergency_blocks_forward_motion():
    cfg = sg.SafetyConfig()
    logic = sg.SafetyLogic(cfg)
    settle(logic, 0.3, 0.0)
    lin, _, status = logic.step(0.3, 0.0, cfg.emergency_distance - 0.05, 8.0,
                                True, True, 0.02)
    assert lin == 0.0
    assert status == sg.EMERGENCY_STOP


def test_gate_emergency_still_allows_turning_in_place():
    cfg = sg.SafetyConfig()
    lin, ang, status = settle(sg.SafetyLogic(cfg), 0.3, 0.8,
                              front=cfg.emergency_distance - 0.05, steps=100)
    assert lin == 0.0
    assert ang == pytest.approx(0.8)
    assert status == sg.EMERGENCY_STOP


def test_gate_caution_zone_limits_forward_speed():
    cfg = sg.SafetyConfig()
    logic = sg.SafetyLogic(cfg)
    settle(logic, 0.4, 0.0)                                     # cruise at full speed
    mid = (cfg.emergency_distance + cfg.caution_distance) / 2.0
    lin, _, status = settle(logic, 0.4, 0.0, front=mid, steps=100)
    assert lin == pytest.approx(cfg.max_linear * 0.5, abs=1e-6)
    assert status == sg.LIMITING


def test_gate_blocks_reverse_when_the_rear_is_close():
    cfg = sg.SafetyConfig()
    lin, _, status = sg.SafetyLogic(cfg).step(
        -0.1, 0.0, 8.0, cfg.rear_emergency_distance - 0.05, True, True, 0.02)
    assert lin == 0.0
    assert status == sg.REAR_BLOCKED


def test_gate_allows_reverse_when_the_rear_is_clear():
    lin, _, status = settle(sg.SafetyLogic(sg.SafetyConfig()), -0.1, 0.0, rear=2.0)
    assert lin == pytest.approx(-0.1)
    assert status == sg.OK


def test_gate_stale_command_ramps_down_instead_of_dropping_instantly():
    logic = sg.SafetyLogic(sg.SafetyConfig())
    settle(logic, 0.3, 0.0)
    lin, _, status = logic.step(0.3, 0.0, 8.0, 8.0, True, False, 0.02)
    assert 0.0 < lin < 0.3
    assert status == sg.NO_CMD

    lin, _, status = settle(logic, 0.3, 0.0, cmd_ok=False, steps=100)
    assert lin == pytest.approx(0.0, abs=1e-9)
    assert status == sg.NO_CMD


@pytest.mark.parametrize('lin_cmd, ang_cmd', [(NAN, 0.0), (INF, 0.0), (0.0, NAN), (0.0, -INF)])
def test_gate_rejects_non_finite_commands(lin_cmd, ang_cmd):
    lin, ang, status = sg.SafetyLogic(sg.SafetyConfig()).step(
        lin_cmd, ang_cmd, 8.0, 8.0, True, True, 0.02)
    assert (lin, ang, status) == (0.0, 0.0, sg.CMD_INVALID)


def test_gate_stops_the_car_even_if_the_avoider_keeps_driving_into_a_wall():
    cfg = sg.SafetyConfig()
    logic = sg.SafetyLogic(cfg)
    dt = 0.02
    wall_dist = 2.0                  # lidar-to-wall distance at the start
    travelled = 0.0
    min_front = wall_dist
    lin = 0.0
    for _ in range(3000):            # 60 s of simulated driving
        front = wall_dist - travelled
        min_front = min(min_front, front)
        lin, _, _ = logic.step(1.0, 0.0, front, 8.0, True, True, dt)   # avoider demands 1 m/s
        travelled += lin * dt
    assert min_front > cfg.emergency_distance - 0.01
    assert lin < 0.01


# ============================================================ config validation
@pytest.mark.parametrize('bad', [
    dict(stop_distance=1.5),
    dict(slow_distance=0.4),
    dict(min_speed=0.5),
    dict(min_speed=0.0),
    dict(turn_speed=0.0),
    dict(reverse_speed=-0.1),
    dict(car_width=0.0),
    dict(corridor_margin=-0.1),
    dict(side_sector_start_deg=95.0),
    dict(side_sector_end_deg=200.0),
    dict(turn_timeout=0.0),
    dict(reverse_duration=0.0),
])
def test_avoider_rejects_unsafe_config(bad):
    with pytest.raises(ValueError):
        oa.validate_config(oa.AvoiderConfig(**bad))


@pytest.mark.parametrize('bad', [
    dict(emergency_distance=0.7),
    dict(caution_distance=0.2),
    dict(rear_emergency_distance=0.0),
    dict(max_linear=0.0),
    dict(max_reverse=0.0),
    dict(max_angular=-1.0),
    dict(max_linear_accel=0.0),
    dict(max_linear_decel=0.0),
    dict(max_angular_accel=0.0),
    dict(car_width=0.0),
    dict(corridor_margin=-0.01),
    dict(cmd_timeout=0.0),
    dict(scan_timeout=0.0),
])
def test_gate_rejects_unsafe_config(bad):
    with pytest.raises(ValueError):
        sg.validate_config(sg.SafetyConfig(**bad))


# ============================================================ cross-layer consistency
def check_layers_agree(av, sf):
    """The avoider must act first; the gate is the backup with the wider limits."""
    assert av.stop_distance > sf.emergency_distance
    assert av.reverse_stop_distance > sf.rear_emergency_distance
    assert av.max_speed <= sf.max_linear
    assert av.reverse_speed <= sf.max_reverse
    assert av.turn_speed <= sf.max_angular


def test_default_configs_are_valid_and_consistent():
    av, sf = oa.AvoiderConfig(), sg.SafetyConfig()
    oa.validate_config(av)
    sg.validate_config(sf)
    check_layers_agree(av, sf)


# ============================================================ YAML files
def build(cls, values):
    return cls(**{f.name: type(f.default)(values[f.name]) for f in dataclasses.fields(cls)})


@pytest.fixture(scope='module')
def params():
    yaml = pytest.importorskip('yaml')
    with open(PARAMS_FILE) as f:
        data = yaml.safe_load(f)
    return (data['obstacle_avoider']['ros__parameters'],
            data['safety_gate']['ros__parameters'])


AVOIDER_NODE_KEYS = {'use_sim_time', 'scan_topic', 'cmd_topic', 'control_rate', 'scan_timeout'}
SAFETY_NODE_KEYS = {'use_sim_time', 'input_topic', 'scan_topic', 'output_topic',
                    'output_frame', 'output_stamped', 'publish_rate'}


def test_yaml_has_no_misspelled_or_missing_keys(params):
    av, sf = params
    av_fields = {f.name for f in dataclasses.fields(oa.AvoiderConfig)}
    sf_fields = {f.name for f in dataclasses.fields(sg.SafetyConfig)}
    # a misspelled key is silently ignored by ROS, so catch it here
    assert set(av) - av_fields - AVOIDER_NODE_KEYS == set()
    assert set(sf) - sf_fields - SAFETY_NODE_KEYS == set()
    # every tunable should be visible in the YAML
    assert av_fields - set(av) == set()
    assert sf_fields - set(sf) == set()


def test_yaml_values_are_valid_and_consistent(params):
    av = build(oa.AvoiderConfig, params[0])
    sf = build(sg.SafetyConfig, params[1])
    oa.validate_config(av)
    sg.validate_config(sf)
    check_layers_agree(av, sf)


def test_gate_limits_stay_inside_the_controller_limits(params):
    yaml = pytest.importorskip('yaml')
    if not os.path.exists(CONTROLLERS_FILE):
        pytest.skip('car_description/config/controllers.yaml not found next to this package')
    with open(CONTROLLERS_FILE) as f:
        ctrl = yaml.safe_load(f)['diff_drive_controller']['ros__parameters']
    sf = build(sg.SafetyConfig, params[1])
    assert sf.max_linear <= ctrl['linear.x.max_velocity']
    assert sf.max_reverse <= abs(ctrl['linear.x.min_velocity'])
    assert sf.max_angular <= ctrl['angular.z.max_velocity']
    assert sf.max_linear_accel <= ctrl['linear.x.max_acceleration']
    assert sf.max_linear_decel <= abs(ctrl['linear.x.max_deceleration'])
    assert sf.max_angular_accel <= ctrl['angular.z.max_acceleration']
