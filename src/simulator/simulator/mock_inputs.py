from __future__ import annotations

import argparse
import math
import time
from itertools import zip_longest
from pathlib import Path

from .control_bus import command_name, read_commands
from .field import FieldGeometry
from .interactive_inputs import SimulatorInputState, load_unit_scene_file, parse_position
from .scene import OWNERSHIP_MODES, RosProjection


def parse_bool(value: str) -> bool:
    lowered = value.strip().lower()
    if lowered in {"1", "true", "yes", "y", "on"}:
        return True
    if lowered in {"0", "false", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"invalid bool value: {value}")


def clamp_u8(value: int) -> int:
    return max(0, min(255, int(value)))


def clamp_u16(value: int) -> int:
    return max(0, min(65535, int(value)))


def clamp_u32(value: int) -> int:
    return max(0, min(0xFFFFFFFF, int(value)))


def default_self_position(team: str, field: FieldGeometry, x: int, y: int) -> tuple[int, int]:
    if x >= 0 and y >= 0:
        return official_bt_point(field, x, y)
    return (245, 750) if str(team).strip().lower() == "red" else (2555, 750)


def default_uwb_position(
    field: FieldGeometry,
    self_position: tuple[int, int],
    x: int,
    y: int,
) -> tuple[int, int]:
    source_x = self_position[0] if x < 0 else x
    source_y = self_position[1] if y < 0 else y
    return official_bt_point(field, source_x, source_y)


def official_bt_point(field: FieldGeometry, x: int, y: int) -> tuple[int, int]:
    return (
        max(1, field.clamp_x(x)),
        max(1, field.clamp_y(y)),
    )


def uwb_raw_point(field: FieldGeometry, x: int, y: int) -> tuple[int, int]:
    official_x, official_y = official_bt_point(field, x, y)
    return (official_x, field.height - official_y)


def payload_position_cm(payload: dict, field: FieldGeometry) -> tuple[int, int] | None:
    position = (
        parse_position(payload.get("position_cm"))
        or parse_position(payload.get("position"))
        or parse_position(payload.get("pos"))
        or parse_position(payload)
    )
    if position is None:
        return None
    if not math.isfinite(position[0]) or not math.isfinite(position[1]):
        return None
    return official_bt_point(field, int(round(position[0])), int(round(position[1])))


def position_publish_frames(
    projection: RosProjection,
) -> list[
    tuple[
        tuple[int, tuple[int, int]] | None,
        tuple[int, tuple[int, int]] | None,
    ]
]:
    """Pair both sides into one PositionData frame per roster slot.

    PositionData carries one friend and one enemy row. Publishing the two sides
    together keeps a full roster below the subscriber queue depth instead of
    dropping the first rows of every timer burst.
    """
    friend_rows = sorted(projection.positions["friend"].items())
    enemy_rows = sorted(projection.positions["enemy"].items())
    return list(zip_longest(friend_rows, enemy_rows, fillvalue=None))


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish minimal offline mock topics for behavior_tree decision tests.")
    parser.add_argument(
        "--input-owner",
        choices=tuple(sorted(OWNERSHIP_MODES)),
        default="mock",
        help="ROS input owner. Only mock may run this publisher (default: mock).",
    )
    parser.add_argument("--team", choices=("red", "blue"), default="red")
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--posture", type=int, default=1)
    parser.add_argument("--gimbal-fire-status", type=int, default=0)
    parser.add_argument("--gimbal-cap-state", type=int, default=0)
    parser.add_argument("--gimbal-follow-mode", type=parse_bool, default=False)
    parser.add_argument("--gimbal-aim-mode", type=parse_bool, default=False)
    parser.add_argument("--gimbal-rotate", type=int, default=0)
    parser.add_argument("--gimbal-yaw-velocity", type=float, default=0.0)
    parser.add_argument("--gimbal-yaw-angle", type=float, default=0.0)
    parser.add_argument("--mock-cap-v", type=int, default=0)
    parser.add_argument("--mock-navi-lower-head", type=int, default=0)
    parser.add_argument("--mock-navi-vel-x", type=float, default=0.0)
    parser.add_argument("--mock-navi-vel-y", type=float, default=0.0)
    parser.add_argument("--mock-bullet-initial-speed", type=float, default=0.0)
    parser.add_argument("--mock-bullet-has-shoot-data", type=parse_bool, default=False)
    parser.add_argument("--mock-bullet-type", type=int, default=0)
    parser.add_argument("--mock-bullet-shooter-number", type=int, default=0)
    parser.add_argument("--mock-bullet-launching-frequency", type=int, default=0)
    parser.add_argument("--mock-bullet-projectile-allowance-17mm", type=int, default=0)
    parser.add_argument("--mock-bullet-projectile-allowance-42mm", type=int, default=0)
    parser.add_argument("--mock-bullet-remaining-gold-coin", type=int, default=0)
    parser.add_argument("--mock-bullet-projectile-allowance-fortress-17mm", type=int, default=0)
    parser.add_argument("--mock-external-aim", type=parse_bool, default=False)
    parser.add_argument("--mock-external-aim-follow", type=parse_bool, default=True)
    parser.add_argument("--mock-external-aim-fire", type=parse_bool, default=True)
    parser.add_argument("--mock-external-aim-yaw", type=float, default=0.0)
    parser.add_argument("--mock-external-aim-pitch", type=float, default=0.0)
    parser.add_argument("--mock-external-aim-target-id", type=int, default=1)
    parser.add_argument("--mock-external-aim-target-x", type=float, default=6.0)
    parser.add_argument("--mock-external-aim-target-y", type=float, default=0.0)
    parser.add_argument("--mock-external-aim-target-z", type=float, default=0.0)
    parser.add_argument("--mock-external-aim-frame", default="gimbal_world")
    parser.add_argument("--time-left", type=int, default=420)
    parser.add_argument("--ammo-left", type=int, default=200)
    parser.add_argument("--self-health", type=int, default=400)
    parser.add_argument("--enemy-health", type=int, default=400)
    parser.add_argument("--self-outpost-health", type=int, default=1500)
    parser.add_argument("--enemy-outpost-health", type=int, default=1500)
    parser.add_argument("--self-base-health", type=int, default=5000)
    parser.add_argument("--enemy-base-health", type=int, default=5000)
    parser.add_argument("--team-buff-recovery", type=int, default=0)
    parser.add_argument("--team-buff-cooling", type=int, default=0)
    parser.add_argument("--team-buff-defence", type=int, default=0)
    parser.add_argument("--team-buff-vulnerability", type=int, default=0)
    parser.add_argument("--team-buff-attack", type=int, default=0)
    parser.add_argument("--team-buff-remaining-energy", type=int, default=0)
    parser.add_argument("--event-center-gain-point-status", type=int, default=0)
    parser.add_argument("--event-self-small-energy-status", type=int, default=0)
    parser.add_argument("--event-self-large-energy-status", type=int, default=0)
    parser.add_argument("--event-self-fortress-gain-point-status", type=int, default=0)
    parser.add_argument("--event-self-outpost-gain-point-status", type=int, default=0)
    parser.add_argument("--event-self-base-gain-point-status", type=parse_bool, default=False)
    parser.add_argument("--event-raw", type=int, default=0)
    parser.add_argument("--sentry-can-activate-energy", type=parse_bool, default=False)
    parser.add_argument("--rfid-raw", type=int, default=0)
    parser.add_argument("--rfid-has-status-2", type=parse_bool, default=False)
    parser.add_argument("--rfid-status-2-raw", type=int, default=0)
    parser.add_argument("--rfid-center-gain-point", type=parse_bool, default=False)
    parser.add_argument("--rfid-self-base", type=parse_bool, default=False)
    parser.add_argument("--rfid-self-fortress", type=parse_bool, default=False)
    parser.add_argument("--rfid-self-outpost", type=parse_bool, default=False)
    parser.add_argument("--rfid-self-supply", type=parse_bool, default=False)
    parser.add_argument("--rfid-self-highland", type=parse_bool, default=False)
    parser.add_argument("--rfid-self-road-crossing", type=parse_bool, default=False)
    parser.add_argument("--rfid-self-central-highland-crossing", type=parse_bool, default=False)
    parser.add_argument("--rfid-self-tunnel", "--rfid-tunnel", dest="rfid_self_tunnel", type=parse_bool, default=False)
    parser.add_argument("--rfid-self-assembly", type=parse_bool, default=False)
    parser.add_argument("--rfid-self-fly-ramp", type=parse_bool, default=False)
    parser.add_argument("--rfid-enemy-fortress", type=parse_bool, default=False)
    parser.add_argument("--rfid-enemy-outpost", type=parse_bool, default=False)
    parser.add_argument("--rfid-enemy-highland", type=parse_bool, default=False)
    parser.add_argument("--rfid-enemy-road-crossing", type=parse_bool, default=False)
    parser.add_argument("--rfid-enemy-central-highland-crossing", type=parse_bool, default=False)
    parser.add_argument("--rfid-enemy-tunnel", type=parse_bool, default=False)
    parser.add_argument("--rfid-enemy-assembly", type=parse_bool, default=False)
    parser.add_argument("--rfid-enemy-fly-ramp", type=parse_bool, default=False)
    parser.add_argument("--navi-reached", type=parse_bool, default=False)
    parser.add_argument("--navi-reachable", type=parse_bool, default=True)
    parser.add_argument("--navi-should-rotate", type=parse_bool, default=True)
    parser.add_argument("--publish-self-position", type=parse_bool, default=True)
    parser.add_argument("--self-position-x", type=int, default=-1)
    parser.add_argument("--self-position-y", type=int, default=-1)
    parser.add_argument("--publish-uwb-position", type=parse_bool, default=False)
    parser.add_argument("--uwb-position-x", type=int, default=-1)
    parser.add_argument("--uwb-position-y", type=int, default=-1)
    parser.add_argument("--official-target-valid", type=parse_bool, default=False)
    parser.add_argument("--official-target-x", type=int, default=0)
    parser.add_argument("--official-target-y", type=int, default=0)
    parser.add_argument("--official-target-armor-type", type=int, default=1)
    parser.add_argument("--simulate-match", type=parse_bool, default=False)
    parser.add_argument("--start-running", type=parse_bool, default=False)
    parser.add_argument("--match-duration-sec", type=int, default=420)
    parser.add_argument("--control-file", default="")
    parser.add_argument("--unit-scene", default="", help="JSON/YAML unit scene loaded before publishing mock topics.")
    args = parser.parse_args(argv)
    if args.hz <= 0:
        parser.error("--hz must be > 0")
    if args.match_duration_sec <= 0:
        parser.error("--match-duration-sec must be > 0")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.input_owner != "mock":
        print(
            "manual_ros owns formal ROS inputs externally; simulator.mock_inputs does not publish in this mode. "
            "Use simulator.start --input-owner manual_ros with Foxglove or a ROS CLI publisher.",
            flush=True,
        )
        return 2
    unit_scene_items = None
    if str(args.unit_scene).strip():
        unit_scene_path = Path(args.unit_scene).expanduser().resolve()
        try:
            unit_scene_items = load_unit_scene_file(unit_scene_path)
        except (OSError, RuntimeError, ValueError) as exc:
            print(f"failed to load unit scene {unit_scene_path}: {exc}", flush=True)
            return 2
    try:
        import rclpy
        from gimbal_driver.msg import (
            BuffData,
            BulletInfo,
            Chassis,
            EventData,
            FireCode,
            GameData,
            GimbalAngles,
            Health,
            PositionData,
            RfidStatus,
            SentryInfo,
            StampedUInt16,
            StampedUInt16MultiArray,
            Vel,
        )
        from rclpy.node import Node
        from std_msgs.msg import Bool, UInt8, UInt16
    except ImportError as exc:
        print(
            "ROS Python deps are missing. Source ROS/workspace first, e.g. "
            "`source /opt/ros/humble/setup.bash && source install/setup.bash`",
            flush=True,
        )
        print(str(exc), flush=True)
        return 2
    AimResult = None
    AimTarget = None
    AimTargetArray = None
    if bool(args.mock_external_aim):
        try:
            from sentry_msgs.msg import AimResult, AimTarget, AimTargetArray
        except ImportError as exc:
            print(
                "sentry_msgs Python deps are missing but --mock-external-aim is enabled. "
                "Build/source sentry_msgs or disable --mock-external-aim.",
                flush=True,
            )
            print(str(exc), flush=True)
            return 2

    class MockDecisionInputs(Node):
        def __init__(self) -> None:
            super().__init__("simulator_mock_inputs")
            self.team = "blue" if args.team == "blue" else "red"
            self.team_red = self.team == "red"
            self.match_duration_sec = max(1, min(65535, int(args.match_duration_sec)))
            self.time_left_float = float(max(0, min(self.match_duration_sec, int(args.time_left))))
            self.time_left = int(math.ceil(self.time_left_float))
            self.ammo_left = max(0, min(65535, int(args.ammo_left)))
            self.self_health = max(0, min(65535, int(args.self_health)))
            self.enemy_health = max(0, min(65535, int(args.enemy_health)))
            structure_health = {
                ("friend", "outpost"): max(0, min(65535, int(args.self_outpost_health))),
                ("enemy", "outpost"): max(0, min(65535, int(args.enemy_outpost_health))),
                ("friend", "base"): max(0, min(65535, int(args.self_base_health))),
                ("enemy", "base"): max(0, min(65535, int(args.enemy_base_health))),
            }
            self.sim_input_state = SimulatorInputState(
                field=FieldGeometry(),
                structure_health_overrides=structure_health,
                team=args.team,
                ownership_mode=args.input_owner,
            )
            if unit_scene_items is not None:
                self.sim_input_state.apply_unit_scene(unit_scene_items, clear=True)
            self.self_position_x, self.self_position_y = default_self_position(
                args.team,
                self.sim_input_state.field,
                int(args.self_position_x),
                int(args.self_position_y),
            )
            self.uwb_position_x, self.uwb_position_y = default_uwb_position(
                self.sim_input_state.field,
                (self.self_position_x, self.self_position_y),
                int(args.uwb_position_x),
                int(args.uwb_position_y),
            )
            self.posture = max(0, min(255, int(args.posture)))
            self.event_center_gain_point_status = min(3, clamp_u8(args.event_center_gain_point_status))
            self.sim_input_state.apply_command("set_self_health", {"hp": self.self_health})
            self.sim_input_state.apply_command("set_ammo", {"ammo": self.ammo_left})
            self.sim_input_state.apply_command("set_posture", {"posture": self.posture})
            self.sim_input_state.apply_command(
                "set_self_position",
                {"x": self.self_position_x, "y": self.self_position_y},
            )
            self.simulate_match = bool(args.simulate_match)
            self.match_started = not self.simulate_match
            self.match_running = not self.simulate_match
            if self.simulate_match and bool(args.start_running):
                self.match_started = True
                self.match_running = True
            self.last_wall_time = time.monotonic()
            self._last_projection_conflicts: tuple[object, ...] = ()
            self.control_path: Path | None = None
            self.control_offset = 0
            if str(args.control_file).strip():
                self.control_path = Path(args.control_file).expanduser().resolve()
                self.control_path.parent.mkdir(parents=True, exist_ok=True)
                if self.control_path.exists():
                    try:
                        self.control_offset = int(self.control_path.stat().st_size)
                    except OSError:
                        self.control_offset = 0

            self.pub_gimbal_angles = self.create_publisher(GimbalAngles, "/ly/gimbal/angles", 10)
            self.pub_gimbal_firecode = self.create_publisher(FireCode, "/ly/gimbal/firecode", 10)
            self.pub_gimbal_chassis = self.create_publisher(Chassis, "/ly/gimbal/chassis", 10)
            self.pub_gimbal_posture = self.create_publisher(UInt8, "/ly/gimbal/posture", 10)
            self.pub_gimbal_cap_v = self.create_publisher(UInt8, "/ly/gimbal/capV", 10)
            self.pub_team = self.create_publisher(Bool, "/ly/friend/is_team_red", 10)
            self.pub_game_start = self.create_publisher(Bool, "/ly/game/is_start", 10)
            self.pub_time_left = self.create_publisher(StampedUInt16, "/ly/game/time_left", 10)
            self.pub_ammo_left = self.create_publisher(UInt16, "/ly/friend/ammo_left", 10)
            self.pub_game_all = self.create_publisher(GameData, "/ly/game/all", 10)
            self.pub_me_hp = self.create_publisher(Health, "/ly/friend/hp", 10)
            self.pub_enemy_hp = self.create_publisher(Health, "/ly/enemy/hp", 10)
            self.pub_friend_op_hp = self.create_publisher(StampedUInt16, "/ly/friend/op_hp", 10)
            self.pub_enemy_op_hp = self.create_publisher(StampedUInt16, "/ly/enemy/op_hp", 10)
            self.pub_friend_base_hp = self.create_publisher(StampedUInt16, "/ly/friend/base_hp", 10)
            self.pub_enemy_base_hp = self.create_publisher(StampedUInt16, "/ly/enemy/base_hp", 10)
            self.pub_team_buff = self.create_publisher(BuffData, "/ly/team/buff", 10)
            self.pub_rfid = self.create_publisher(RfidStatus, "/ly/game/rfid", 10)
            self.pub_event_data = self.create_publisher(EventData, "/ly/game/event_data", 10)
            self.pub_sentry_info = self.create_publisher(SentryInfo, "/ly/game/sentry/info", 10)
            self.pub_navi_reached = self.create_publisher(Bool, "/ly/navi/reached", 10)
            self.pub_navi_reachable = self.create_publisher(Bool, "/ly/navi/reachable", 10)
            self.pub_navi_should_rotate = self.create_publisher(Bool, "/ly/navi/should_rotate", 10)
            self.pub_navi_vel = self.create_publisher(Vel, "/ly/navi/vel", 10)
            self.pub_navi_lower_head = self.create_publisher(UInt8, "/ly/navi/lower_head", 10)
            self.pub_navi_position = self.create_publisher(StampedUInt16MultiArray, "/ly/navi/position", 10)
            self.pub_uwb_position = self.create_publisher(StampedUInt16MultiArray, "/ly/friend/uwb_pos", 10)
            self.pub_navi_target_official = self.create_publisher(
                StampedUInt16MultiArray,
                "/ly/navi/target_official",
                10,
            )
            self.pub_position_data = self.create_publisher(PositionData, "/ly/position/data", 10)
            self.pub_bullet_info = self.create_publisher(BulletInfo, "/ly/game/bullet", 10)
            self.pub_external_aim_targets = None
            self.pub_external_aim_result = None
            if bool(args.mock_external_aim):
                self.pub_external_aim_targets = self.create_publisher(
                    AimTargetArray,
                    "/ly/aim/armor_targets",
                    10,
                )
                self.pub_external_aim_result = self.create_publisher(AimResult, "/ly/aim/result", 10)

            period = 1.0 / float(args.hz)
            self.timer = self.create_timer(period, self._publish_all)
            self.get_logger().info(
                "mock inputs started: "
                f"input_owner={args.input_owner} team={args.team} hz={args.hz:.1f} "
                f"time_left={self.time_left} ammo_left={self.ammo_left} "
                f"enemy_outpost_hp={self.sim_input_state.structure_hp('enemy', 'outpost')} "
                f"enemy_base_hp={self.sim_input_state.structure_hp('enemy', 'base')} "
                f"team_buff_attack={clamp_u16(args.team_buff_attack)} "
                f"team_buff_defence={clamp_u8(args.team_buff_defence)} "
                f"sentry_can_activate_energy={str(bool(args.sentry_can_activate_energy)).lower()} "
                f"navi_reachable={str(bool(args.navi_reachable)).lower()} "
                f"navi_vel=({float(args.mock_navi_vel_x):.2f},{float(args.mock_navi_vel_y):.2f}) "
                f"uwb_position={str(bool(args.publish_uwb_position)).lower()}"
                f"({self.uwb_position_x},{self.uwb_position_y}) "
                f"firecode=({clamp_u8(args.gimbal_fire_status) & 0b11},"
                f"{clamp_u8(args.gimbal_cap_state) & 0b11},"
                f"{int(bool(args.gimbal_follow_mode))},"
                f"{int(bool(args.gimbal_aim_mode))},"
                f"{clamp_u8(args.gimbal_rotate) & 0b11}) "
                f"external_aim={str(bool(args.mock_external_aim)).lower()} "
                f"simulate_match={str(self.simulate_match).lower()} "
                f"running={str(self.match_running).lower()} "
                f"units={len(self.sim_input_state.units)} "
                f"unit_scene={(Path(args.unit_scene).expanduser().resolve().as_posix() if str(args.unit_scene).strip() else '-')} "
                f"control_file={(self.control_path.as_posix() if self.control_path else '-')}"
            )

        def _clamp_time_left(self) -> None:
            self.time_left_float = max(0.0, min(float(self.match_duration_sec), float(self.time_left_float)))
            self.time_left = max(0, min(self.match_duration_sec, int(math.ceil(self.time_left_float))))

        def _apply_command(self, payload: dict) -> None:
            command = command_name(payload)
            if not command:
                return
            if command == "start":
                if self.time_left_float <= 0.0:
                    self.time_left_float = float(self.match_duration_sec)
                    self._clamp_time_left()
                self.match_started = True
                self.match_running = True
                return
            if command == "pause":
                self.match_running = False
                return
            if command == "reset":
                self.time_left_float = float(self.match_duration_sec)
                self._clamp_time_left()
                self.match_started = False
                self.match_running = False
                self.sim_input_state.reset()
                sentry = next(
                    (
                        unit
                        for unit in self.sim_input_state.scene.units.values()
                        if unit.side == "friend" and unit.unit_key == "sentry"
                    ),
                    None,
                )
                if sentry is not None:
                    self.self_health = int(sentry.hp)
                    self.self_position_x, self.self_position_y = sentry.x, sentry.y
                return
            if command == "rewind":
                try:
                    delta = float(payload.get("seconds", 0.0))
                except (TypeError, ValueError):
                    return
                if not math.isfinite(delta):
                    return
                self.time_left_float += max(0.0, delta)
                self._clamp_time_left()
                return
            if command == "forward":
                try:
                    delta = float(payload.get("seconds", 0.0))
                except (TypeError, ValueError):
                    return
                if not math.isfinite(delta):
                    return
                self.time_left_float -= max(0.0, delta)
                self._clamp_time_left()
                return
            if command == "set_time_left":
                try:
                    target = float(payload.get("seconds", self.time_left_float))
                except (TypeError, ValueError):
                    return
                if not math.isfinite(target):
                    return
                self.time_left_float = target
                self._clamp_time_left()
                return
            if command == "set_team":
                team = str(payload.get("team", "")).strip().lower()
                if team not in {"red", "blue"}:
                    return
                self.team = team
                self.team_red = team == "red"
                self.sim_input_state.apply_command("set_team", {"team": team})
                return
            if command == "set_event_center_gain_point_status":
                self.event_center_gain_point_status = min(
                    3,
                    clamp_u8(payload.get("status", payload.get("value", self.event_center_gain_point_status))),
                )
                return
            if command == "set_self_health":
                hp = clamp_u16(payload.get("hp", payload.get("health", payload.get("self_health", self.self_health))))
                self.self_health = hp
                self.sim_input_state.apply_control_payload(payload)
                return
            if command == "set_ammo":
                ammo = clamp_u16(payload.get("ammo", payload.get("ammo_left", payload.get("count", self.ammo_left))))
                self.ammo_left = ammo
                self.sim_input_state.apply_control_payload(payload)
                return
            if command == "set_posture":
                posture = clamp_u8(payload.get("posture", payload.get("id", payload.get("value", self.posture))))
                self.posture = posture
                self.sim_input_state.apply_control_payload(payload)
                return
            if command == "set_self_position":
                position = payload_position_cm(payload, self.sim_input_state.field)
                if position is None:
                    return
                self.self_position_x, self.self_position_y = position
                self.uwb_position_x, self.uwb_position_y = default_uwb_position(
                    self.sim_input_state.field,
                    (self.self_position_x, self.self_position_y),
                    int(args.uwb_position_x),
                    int(args.uwb_position_y),
                )
                self.sim_input_state.apply_command(
                    "set_self_position",
                    {"x": self.self_position_x, "y": self.self_position_y},
                )
                return
            self.sim_input_state.apply_control_payload(payload)
            self._sync_self_health_from_payload(payload)
            self._sync_self_health_from_scene()

        def _sync_self_health_from_payload(self, payload: dict) -> None:
            entity_id = str(payload.get("entity_id", "")).strip().lower()
            side = str(payload.get("side", "")).strip().lower()
            unit_key = str(payload.get("unit_key", "")).strip().lower()
            type_name = str(payload.get("type", "")).strip().lower()
            target_is_friend = side in {"friend", "self", "ally"} or entity_id.startswith(("friend:", "self:", "ally:"))
            target_is_sentry = (
                "sentry" in entity_id
                or unit_key == "sentry"
                or type_name == "sentry"
                or int(payload.get("type_id", 0) or 0) == 7
            )
            if not (target_is_friend and target_is_sentry):
                return
            hp = payload.get("hp", payload.get("health"))
            if hp is not None:
                self.self_health = clamp_u16(hp)

        def _sync_self_health_from_scene(self) -> None:
            sentry = next(
                (
                    unit
                    for unit in self.sim_input_state.scene.units.values()
                    if unit.side == "friend" and unit.unit_key == "sentry"
                ),
                None,
            )
            if sentry is not None:
                self.self_health = clamp_u16(sentry.hp)

        def _poll_commands(self) -> None:
            if self.control_path is None:
                return
            commands, new_offset = read_commands(self.control_path, self.control_offset)
            self.control_offset = new_offset
            for payload in commands:
                self._apply_command(payload)

        @staticmethod
        def _publish_stamped_u16(pub: object, stamp: object, value: int) -> None:
            msg = StampedUInt16()
            msg.header.stamp = stamp
            msg.data = max(0, min(65535, int(value)))
            pub.publish(msg)

        @staticmethod
        def _clamp_u8(value: int) -> int:
            return clamp_u8(value)

        @staticmethod
        def _clamp_u16(value: int) -> int:
            return clamp_u16(value)

        def _publish_bool(self, pub: object, value: bool) -> None:
            msg = Bool()
            msg.data = bool(value)
            pub.publish(msg)

        def _stamped_u16_array(self, stamp: object, values: list[int]) -> StampedUInt16MultiArray:
            msg = StampedUInt16MultiArray()
            msg.header.stamp = stamp
            msg.data = [self._clamp_u16(value) for value in values]
            return msg

        def _report_projection_conflicts(self, projection: RosProjection) -> None:
            if projection.conflicts == self._last_projection_conflicts:
                return
            previous = self._last_projection_conflicts
            self._last_projection_conflicts = projection.conflicts
            if projection.conflicts:
                details = "; ".join(
                    f"{item.kind}:{item.side}:{item.formal_key}={','.join(item.entity_ids)}"
                    for item in projection.conflicts
                )
                self.get_logger().warn(
                    "scene ROS projection conflict; conflicting scene values are withheld and baseline "
                    "Health values remain: "
                    + details
                )
            elif previous:
                self.get_logger().info("scene ROS projection conflict resolved")

        def _health_msg(self, side: str, stamp: object, projection: RosProjection) -> Health:
            msg = Health()
            msg.header.stamp = stamp
            base_hp = self.self_health if side == "friend" else self.enemy_health
            fields = {
                unit.health_field: base_hp
                for unit in self.sim_input_state.catalog.units
                if unit.health_published and unit.health_field is not None
            }
            fields.update(projection.health[side])
            for field, value in fields.items():
                setattr(msg, field, max(0, min(65535, int(value))))
            return msg

        def _publish_unit_positions(self, stamp: object, projection: RosProjection) -> None:
            for friend_row, enemy_row in position_publish_frames(projection):
                if friend_row is not None:
                    try:
                        self.sim_input_state.catalog.unit_by_position_car_id(friend_row[0], "friend")
                    except KeyError:
                        friend_row = None
                if enemy_row is not None:
                    try:
                        self.sim_input_state.catalog.unit_by_position_car_id(enemy_row[0], "enemy")
                    except KeyError:
                        enemy_row = None
                if friend_row is None and enemy_row is None:
                    continue
                msg = PositionData()
                msg.header.stamp = stamp
                if friend_row is not None:
                    msg.friendcarid = friend_row[0]
                    msg.friendx = int(friend_row[1][0])
                    msg.friendy = int(friend_row[1][1])
                if enemy_row is not None:
                    msg.enemycarid = enemy_row[0]
                    msg.enemyx = int(enemy_row[1][0])
                    msg.enemyy = int(enemy_row[1][1])
                self.pub_position_data.publish(msg)

        def _publish_navi_position(self, stamp: object) -> None:
            if not bool(args.publish_self_position):
                return
            msg = self._stamped_u16_array(stamp, [self.self_position_x, self.self_position_y])
            msg.map_frame = "official_map_cm"
            msg.source_frame = "simulator_mock_inputs"
            msg.map_point.x = float(self.self_position_x) / 100.0
            msg.map_point.y = float(self.self_position_y) / 100.0
            msg.map_point.z = 0.0
            self.pub_navi_position.publish(msg)

        def _publish_uwb_position(self, stamp: object) -> None:
            if not bool(args.publish_uwb_position):
                return
            raw_x, raw_y = uwb_raw_point(
                self.sim_input_state.field,
                self.uwb_position_x,
                self.uwb_position_y,
            )
            msg = self._stamped_u16_array(stamp, [raw_x, raw_y])
            msg.map_frame = "official_map_cm"
            msg.source_frame = "simulator_mock_inputs_uwb"
            msg.map_point.x = float(self.uwb_position_x) / 100.0
            msg.map_point.y = float(self.uwb_position_y) / 100.0
            msg.map_point.z = 0.0
            self.pub_uwb_position.publish(msg)

        def _publish_firecode(self, stamp: object) -> None:
            msg = FireCode()
            msg.header.stamp = stamp
            msg.field_mask = FireCode.FIELD_ALL
            msg.fire_status = clamp_u8(args.gimbal_fire_status) & 0b11
            msg.cap_state = clamp_u8(args.gimbal_cap_state) & 0b11
            msg.follow_mode = bool(args.gimbal_follow_mode)
            msg.aim_mode = bool(args.gimbal_aim_mode)
            msg.rotate = clamp_u8(args.gimbal_rotate) & 0b11
            msg.raw = (
                (msg.fire_status & 0b11)
                | ((msg.cap_state & 0b11) << 2)
                | ((1 if msg.follow_mode else 0) << 4)
                | ((1 if msg.aim_mode else 0) << 5)
                | ((msg.rotate & 0b11) << 6)
            )
            self.pub_gimbal_firecode.publish(msg)

        def _publish_chassis(self, stamp: object) -> None:
            msg = Chassis()
            msg.header.stamp = stamp
            msg.angular_velocity = float(args.gimbal_yaw_velocity)
            msg.steer_angle = float(args.gimbal_yaw_angle)
            msg.velocity_x = float(args.mock_navi_vel_x)
            msg.velocity_y = float(args.mock_navi_vel_y)
            self.pub_gimbal_chassis.publish(msg)

        def _publish_navi_vel(self, stamp: object) -> None:
            msg = Vel()
            msg.header.stamp = stamp
            msg.x = float(args.mock_navi_vel_x)
            msg.y = float(args.mock_navi_vel_y)
            self.pub_navi_vel.publish(msg)

        def _publish_bullet_info(self, stamp: object) -> None:
            msg = BulletInfo()
            msg.header.stamp = stamp
            msg.has_initial_speed = float(args.mock_bullet_initial_speed) > 0.0
            msg.initial_speed = max(0.0, float(args.mock_bullet_initial_speed))
            msg.has_shoot_data = bool(args.mock_bullet_has_shoot_data)
            msg.bullet_type = clamp_u8(args.mock_bullet_type)
            msg.shooter_number = clamp_u8(args.mock_bullet_shooter_number)
            msg.launching_frequency = clamp_u8(args.mock_bullet_launching_frequency)
            msg.has_projectile_allowance = any(
                int(value) > 0
                for value in (
                    args.mock_bullet_projectile_allowance_17mm,
                    args.mock_bullet_projectile_allowance_42mm,
                    args.mock_bullet_remaining_gold_coin,
                    args.mock_bullet_projectile_allowance_fortress_17mm,
                )
            )
            msg.projectile_allowance_17mm = clamp_u16(args.mock_bullet_projectile_allowance_17mm)
            msg.projectile_allowance_42mm = clamp_u16(args.mock_bullet_projectile_allowance_42mm)
            msg.remaining_gold_coin = clamp_u16(args.mock_bullet_remaining_gold_coin)
            msg.projectile_allowance_fortress_17mm = clamp_u16(
                args.mock_bullet_projectile_allowance_fortress_17mm
            )
            self.pub_bullet_info.publish(msg)

        def _publish_external_aim(self, stamp: object) -> None:
            if (
                not bool(args.mock_external_aim)
                or self.pub_external_aim_targets is None
                or self.pub_external_aim_result is None
            ):
                return
            frame_id = str(args.mock_external_aim_frame).strip() or "gimbal_world"
            target = AimTarget()
            target.header.stamp = stamp
            target.header.frame_id = frame_id
            target.id = clamp_u8(args.mock_external_aim_target_id)
            target.position.x = float(args.mock_external_aim_target_x)
            target.position.y = float(args.mock_external_aim_target_y)
            target.position.z = float(args.mock_external_aim_target_z)

            targets = AimTargetArray()
            targets.header.stamp = stamp
            targets.header.frame_id = frame_id
            targets.aim_targets = [target]
            self.pub_external_aim_targets.publish(targets)

            result = AimResult()
            result.header.stamp = stamp
            result.follow = bool(args.mock_external_aim_follow)
            result.fire = bool(args.mock_external_aim_fire)
            result.yaw = float(args.mock_external_aim_yaw)
            result.pitch = float(args.mock_external_aim_pitch)
            self.pub_external_aim_result.publish(result)

        def _publish_official_target(self, stamp: object) -> None:
            if not bool(args.official_target_valid):
                return
            target_x, target_y = official_bt_point(
                self.sim_input_state.field,
                int(args.official_target_x),
                int(args.official_target_y),
            )
            armor_type = self._clamp_u16(args.official_target_armor_type)
            msg = self._stamped_u16_array(stamp, [target_x, target_y, armor_type])
            msg.map_frame = "official_map_cm"
            msg.source_frame = "simulator_mock_inputs"
            msg.map_point.x = float(target_x) / 100.0
            msg.map_point.y = float(target_y) / 100.0
            msg.map_point.z = 0.0
            self.pub_navi_target_official.publish(msg)

        def _event_data_msg(self, stamp: object) -> EventData:
            msg = EventData()
            msg.header.stamp = stamp
            msg.raw = clamp_u32(args.event_raw)
            msg.center_gain_point_status = self.event_center_gain_point_status
            msg.self_small_energy_status = self._clamp_u8(args.event_self_small_energy_status)
            msg.self_large_energy_status = self._clamp_u8(args.event_self_large_energy_status)
            msg.self_fortress_gain_point_status = self._clamp_u8(args.event_self_fortress_gain_point_status)
            msg.self_outpost_gain_point_status = self._clamp_u8(args.event_self_outpost_gain_point_status)
            msg.self_base_gain_point_status = bool(args.event_self_base_gain_point_status)
            return msg

        def _sentry_info_msg(self, stamp: object) -> SentryInfo:
            msg = SentryInfo()
            msg.header.stamp = stamp
            msg.can_activate_energy_mechanism = bool(args.sentry_can_activate_energy)
            return msg

        def _team_buff_msg(self) -> BuffData:
            msg = BuffData()
            msg.recoverybuff = self._clamp_u8(args.team_buff_recovery)
            msg.coolingbuff = self._clamp_u8(args.team_buff_cooling)
            msg.defencebuff = self._clamp_u8(args.team_buff_defence)
            msg.vulnerabilitybuff = self._clamp_u8(args.team_buff_vulnerability)
            msg.attackbuff = self._clamp_u16(args.team_buff_attack)
            msg.remainingenergy = self._clamp_u8(args.team_buff_remaining_energy)
            return msg

        def _rfid_msg(self, stamp: object) -> RfidStatus:
            msg = RfidStatus()
            msg.header.stamp = stamp
            msg.raw = clamp_u32(args.rfid_raw)
            msg.has_rfid_status_2 = bool(args.rfid_has_status_2 or args.rfid_status_2_raw or args.rfid_enemy_tunnel)
            msg.rfid_status_2_raw = self._clamp_u8(args.rfid_status_2_raw)
            msg.central_rmul = bool(args.rfid_center_gain_point)
            msg.friend_base = bool(args.rfid_self_base)
            msg.friend_bastion = bool(args.rfid_self_fortress)
            msg.friend_outpost = bool(args.rfid_self_outpost)
            msg.friend_supply_remix = bool(args.rfid_self_supply)
            msg.friend_highland = bool(args.rfid_self_highland)
            msg.friend_roadland_high = bool(args.rfid_self_road_crossing)
            msg.friend_central_high = bool(args.rfid_self_central_highland_crossing)
            msg.friend_tunnel_roadland_mid = bool(args.rfid_self_tunnel)
            msg.friend_armor = bool(args.rfid_self_assembly)
            msg.friend_flyroad_front = bool(args.rfid_self_fly_ramp)
            msg.enemy_bastion = bool(args.rfid_enemy_fortress)
            msg.enemy_outpost = bool(args.rfid_enemy_outpost)
            msg.enemy_highland = bool(args.rfid_enemy_highland)
            msg.enemy_roadland_high = bool(args.rfid_enemy_road_crossing)
            msg.enemy_central_high = bool(args.rfid_enemy_central_highland_crossing)
            msg.enemy_tunnel_roadland_mid = bool(args.rfid_enemy_tunnel)
            msg.enemy_armor = bool(args.rfid_enemy_assembly)
            msg.enemy_flyroad_front = bool(args.rfid_enemy_fly_ramp)
            return msg

        def _publish_all(self) -> None:
            self._poll_commands()
            now_wall = time.monotonic()
            dt = max(0.0, now_wall - self.last_wall_time)
            self.last_wall_time = now_wall
            if self.simulate_match and self.match_started and self.match_running and self.time_left_float > 0.0:
                self.time_left_float -= dt
                self._clamp_time_left()
                if self.time_left_float <= 0.0:
                    self.match_running = False
            now = self.get_clock().now().to_msg()
            projection = self.sim_input_state.scene.project_ros_inputs()
            self._report_projection_conflicts(projection)

            gimbal = GimbalAngles()
            gimbal.yaw = float(args.yaw)
            gimbal.pitch = float(args.pitch)
            gimbal.header.stamp = now
            self.pub_gimbal_angles.publish(gimbal)
            self._publish_firecode(now)
            self._publish_chassis(now)

            posture = UInt8()
            posture.data = self.posture
            self.pub_gimbal_posture.publish(posture)

            cap_v = UInt8()
            cap_v.data = clamp_u8(args.mock_cap_v)
            self.pub_gimbal_cap_v.publish(cap_v)

            team = Bool()
            team.data = self.team_red
            self.pub_team.publish(team)

            game_start = Bool()
            game_start.data = bool(self.match_started)
            self.pub_game_start.publish(game_start)

            self._publish_stamped_u16(self.pub_time_left, now, self.time_left)

            ammo = UInt16()
            ammo.data = self.ammo_left
            self.pub_ammo_left.publish(ammo)

            game_all = GameData()
            game_all.header.stamp = now
            game_all.gamecode = 0
            game_all.ammoleft = self.ammo_left
            game_all.timeleft = self.time_left
            game_all.selfhealth = self.self_health
            game_all.exteventdata = clamp_u32(args.event_raw)
            self.pub_game_all.publish(game_all)

            self._publish_stamped_u16(
                self.pub_friend_op_hp, now, projection.structures["friend"]["outpost"])
            self._publish_stamped_u16(
                self.pub_enemy_op_hp, now, projection.structures["enemy"]["outpost"])
            self._publish_stamped_u16(
                self.pub_friend_base_hp, now, projection.structures["friend"]["base"])
            self._publish_stamped_u16(
                self.pub_enemy_base_hp, now, projection.structures["enemy"]["base"])

            self.pub_me_hp.publish(self._health_msg("friend", now, projection))
            self.pub_enemy_hp.publish(self._health_msg("enemy", now, projection))
            self._publish_unit_positions(now, projection)

            self.pub_team_buff.publish(self._team_buff_msg())
            self.pub_rfid.publish(self._rfid_msg(now))
            self.pub_event_data.publish(self._event_data_msg(now))
            self.pub_sentry_info.publish(self._sentry_info_msg(now))
            self._publish_bool(self.pub_navi_reached, bool(args.navi_reached))
            self._publish_bool(self.pub_navi_reachable, bool(args.navi_reachable))
            self._publish_bool(self.pub_navi_should_rotate, bool(args.navi_should_rotate))
            self._publish_navi_vel(now)
            lower_head = UInt8()
            lower_head.data = clamp_u8(args.mock_navi_lower_head)
            self.pub_navi_lower_head.publish(lower_head)
            self._publish_navi_position(now)
            self._publish_uwb_position(now)
            self._publish_official_target(now)
            self._publish_bullet_info(now)
            self._publish_external_aim(now)

    rclpy.init(args=None)
    node = MockDecisionInputs()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if exc.__class__.__name__ != "ExternalShutdownException":
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
