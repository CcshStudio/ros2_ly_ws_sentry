from __future__ import annotations

import argparse
import json
import math
import os
import shlex
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

from .config import repo_root


DEFAULT_BT_CONFIG_BY_MODE = {
    "regional": "regional_competition.json",
    "league": "league_competition.json",
    "showcase": "regional/debug/showcase_competition.json",
}

MOCK_PRESET_DESCRIPTIONS = {
    "none": "No preset overlay; use explicit --mock-* values and parser defaults.",
    "buff-ready": "Regional energy-window check with external aim, sentry activation, buff energy, and center RFID.",
    "outpost-dead": "Regional resource state where the enemy outpost HP is already zero.",
    "nav-unreachable": "Regional navigation failure context with an unreachable current goal.",
    "official-target-sentry": "Official target fallback for a sentry armor position on /ly/navi/target_official.",
    "uwb-fusion": "Regional self-position fusion rehearsal with opt-in /ly/friend/uwb_pos.",
    "bullet-resource": "BulletInfo resource snapshot with speed, shoot data, projectile allowance, and gold coin fields.",
    "multi-unit-regional": "Regional multi-unit scene using sample/unit_scene.json plus HP, RFID, buff, and target context.",
    "full-roster-regional": (
        "Full red/blue unit roster for checking packaged unit art, formal health-unit HP mapping, "
        "and placed-unit PositionData."
    ),
    "low-resource": "Low HP and low ammo recovery context for resource-fallback decisions.",
}

MOCK_PRESETS: dict[str, dict[str, object]] = {
    "none": {},
    "buff-ready": {
        "mode": "regional",
        "mock_external_aim": True,
        "mock_external_aim_yaw": 6.0,
        "mock_external_aim_pitch": -1.0,
        "mock_time_left": 411,
        "mock_ammo": 43,
        "mock_self_health": 382,
        "mock_sentry_can_activate_energy": True,
        "mock_event_self_small_energy_status": 2,
        "mock_team_buff_attack": 1,
        "mock_team_buff_remaining_energy": 35,
        "mock_rfid_center_gain_point": True,
        "mock_rfid_self_outpost": True,
        "mock_self_position_x": 924,
        "mock_self_position_y": 1388,
    },
    "outpost-dead": {
        "mode": "regional",
        "mock_time_left": 360,
        "mock_ammo": 120,
        "mock_self_health": 360,
        "mock_enemy_outpost_health": 0,
        "mock_enemy_base_health": 4200,
        "mock_navi_reachable": True,
        "mock_navi_should_rotate": True,
        "mock_self_position_x": 1100,
        "mock_self_position_y": 1130,
    },
    "nav-unreachable": {
        "mode": "regional",
        "mock_time_left": 390,
        "mock_ammo": 80,
        "mock_navi_reached": False,
        "mock_navi_reachable": False,
        "mock_navi_should_rotate": False,
        "mock_self_position_x": 1220,
        "mock_self_position_y": 760,
    },
    "official-target-sentry": {
        "mode": "regional",
        "mock_time_left": 402,
        "mock_ammo": 70,
        "mock_self_position_x": 1220,
        "mock_self_position_y": 760,
        "mock_official_target_valid": True,
        "mock_official_target_x": 1505,
        "mock_official_target_y": 905,
        "mock_official_target_armor_type": 6,
    },
    "uwb-fusion": {
        "mode": "regional",
        "mock_time_left": 386,
        "mock_ammo": 90,
        "mock_self_health": 360,
        "mock_navi_reachable": True,
        "mock_navi_should_rotate": True,
        "mock_self_position_x": 1110,
        "mock_self_position_y": 720,
        "mock_publish_uwb_position": True,
        "mock_uwb_position_x": 1220,
        "mock_uwb_position_y": 760,
    },
    "bullet-resource": {
        "mode": "regional",
        "mock_time_left": 392,
        "mock_ammo": 88,
        "mock_self_health": 365,
        "mock_bullet_initial_speed": 23.4,
        "mock_bullet_has_shoot_data": True,
        "mock_bullet_type": 1,
        "mock_bullet_shooter_number": 7,
        "mock_bullet_launching_frequency": 9,
        "mock_bullet_projectile_allowance_17mm": 118,
        "mock_bullet_projectile_allowance_42mm": 6,
        "mock_bullet_remaining_gold_coin": 14,
        "mock_bullet_projectile_allowance_fortress_17mm": 32,
    },
    "multi-unit-regional": {
        "mode": "regional",
        "unit_scene": "src/simulator/sample/unit_scene.json",
        "mock_external_aim": True,
        "mock_external_aim_yaw": 8.0,
        "mock_external_aim_pitch": -1.5,
        "mock_time_left": 398,
        "mock_ammo": 80,
        "mock_self_health": 360,
        "mock_enemy_health": 180,
        "mock_enemy_outpost_health": 1100,
        "mock_team_buff_attack": 1,
        "mock_team_buff_remaining_energy": 27,
        "mock_rfid_center_gain_point": True,
        "mock_rfid_self_highland": True,
        "mock_navi_reachable": True,
        "mock_self_position_x": 820,
        "mock_self_position_y": 830,
    },
    "full-roster-regional": {
        "mode": "regional",
        "unit_scene": "src/simulator/sample/unit_scenes/full_roster.json",
        "mock_external_aim": True,
        "mock_external_aim_yaw": 5.0,
        "mock_external_aim_pitch": -1.0,
        "mock_time_left": 390,
        "mock_ammo": 120,
        "mock_self_health": 390,
        "mock_enemy_health": 220,
        "mock_enemy_outpost_health": 1500,
        "mock_enemy_base_health": 5000,
        "mock_team_buff_attack": 1,
        "mock_team_buff_remaining_energy": 18,
        "mock_rfid_center_gain_point": True,
        "mock_navi_reachable": True,
        "mock_self_position_x": 700,
        "mock_self_position_y": 820,
    },
    "low-resource": {
        "mode": "regional",
        "mock_time_left": 398,
        "mock_ammo": 5,
        "mock_self_health": 118,
        "mock_enemy_outpost_health": 1100,
        "mock_self_position_x": 183,
        "mock_self_position_y": 245,
        "mock_navi_reachable": True,
    },
}


def parse_bool(value: str) -> bool:
    lowered = value.strip().lower()
    if lowered in {"1", "true", "yes", "y", "on"}:
        return True
    if lowered in {"0", "false", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"invalid bool value: {value}")


def bool_text(value: bool) -> str:
    return "true" if bool(value) else "false"


def collect_explicit_cli_dests(parser: argparse.ArgumentParser, argv: list[str]) -> set[str]:
    option_actions = parser._option_string_actions  # noqa: SLF001 - argparse has no public equivalent.
    explicit: set[str] = set()
    for token in argv:
        if token == "--":
            break
        if not token.startswith("-"):
            continue
        option_name = token.split("=", 1)[0]
        action = option_actions.get(option_name)
        if action is not None and action.dest != "help":
            explicit.add(action.dest)
    return explicit


def apply_mock_preset(args: argparse.Namespace, explicit_dests: set[str]) -> None:
    preset = MOCK_PRESETS.get(str(args.mock_preset), {})
    for dest, value in preset.items():
        if dest in explicit_dests:
            continue
        setattr(args, dest, value)


def _dest_to_cli_name(dest: str) -> str:
    if dest.startswith("mock_"):
        return "--mock-" + dest[len("mock_") :].replace("_", "-")
    return "--" + dest.replace("_", "-")


def _preset_value_text(value: object) -> str:
    if isinstance(value, bool):
        return bool_text(value)
    if isinstance(value, list):
        return " ".join(str(item) for item in value)
    return str(value)


def _preset_cli_parts(dest: str, value: object) -> list[str]:
    cli_name = _dest_to_cli_name(dest)
    if isinstance(value, list):
        return [f"{cli_name} {_preset_value_text(item)}" for item in value]
    return [f"{cli_name} {_preset_value_text(value)}"]


def print_mock_presets() -> int:
    print("Mock presets:")
    for name, overlay in MOCK_PRESETS.items():
        print(f"  {name}: {MOCK_PRESET_DESCRIPTIONS[name]}")
        if overlay:
            parts = [part for dest, value in overlay.items() for part in _preset_cli_parts(dest, value)]
            print(f"    overlay: {' '.join(parts)}")
        else:
            print("    overlay: current parser defaults")
    return 0


def print_mock_sequences() -> int:
    from .mock_sequence import print_sample_sequences

    return print_sample_sequences()


def print_unit_scenes() -> int:
    from .unit_scene import print_unit_scene_samples

    return print_unit_scene_samples()


def _collect_pids_by_pattern(pattern: str) -> list[int]:
    try:
        proc = subprocess.run(
            ["pgrep", "-f", pattern],
            check=False,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError:
        return []
    if proc.returncode not in (0, 1):
        return []
    pids: list[int] = []
    for line in proc.stdout.splitlines():
        text = line.strip()
        if not text:
            continue
        try:
            pids.append(int(text))
        except ValueError:
            continue
    return pids


def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def _terminate_pids(pids: list[int], timeout_sec: float = 1.2) -> list[int]:
    if not pids:
        return []
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    deadline = time.monotonic() + max(0.1, timeout_sec)
    while time.monotonic() < deadline:
        alive = [pid for pid in pids if _pid_alive(pid)]
        if not alive:
            return []
        time.sleep(0.05)
    alive = [pid for pid in pids if _pid_alive(pid)]
    for pid in alive:
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass
    time.sleep(0.05)
    return [pid for pid in alive if _pid_alive(pid)]


def cleanup_stale_live_viewers() -> None:
    this_pid = os.getpid()
    targets = sorted(
        {
            pid
            for pattern in ("simulator.main", "simulator ")
            for pid in _collect_pids_by_pattern(pattern)
            if pid != this_pid
        }
    )
    if not targets:
        return
    remain = _terminate_pids(targets)
    if remain:
        print(f"warning: stale simulator.main processes still alive: {remain}", file=sys.stderr)
    else:
        print(f"cleaned stale simulator.main processes: {targets}")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Record behavior_tree decision traces with start.sh and optional config indexing "
            "from src/behavior_tree/Scripts/ConfigJson."
        )
    )
    parser.add_argument(
        "--entry",
        choices=("nogate", "gated"),
        default="nogate",
        help="Entry forwarded to scripts/start.sh (default: nogate).",
    )
    parser.add_argument(
        "--mode",
        choices=("regional", "league", "showcase"),
        default="league",
        help="Competition mode forwarded to scripts/start.sh (default: league).",
    )
    parser.add_argument(
        "--bt-config",
        default="",
        help=(
            "Behavior-tree config preset name or path. Examples: league_competition.json, "
            "league/chase_only_competition.json, /abs/path/custom.json"
        ),
    )
    parser.add_argument(
        "--trace",
        default="",
        help="Trace output JSONL path. Default: log/decision_trace_<timestamp>.jsonl",
    )
    parser.add_argument(
        "--every",
        type=int,
        default=5,
        help="decision_trace_every_n_ticks value (default: 5).",
    )
    parser.add_argument(
        "--offline-decision",
        action="store_true",
        help="Run decision offline: behavior_tree only, with built-in mock input topics.",
    )
    parser.add_argument(
        "--input-owner",
        choices=("mock", "manual_ros"),
        default="mock",
        help=(
            "Formal ROS input owner. mock starts simulator inputs; manual_ros only observes while "
            "Foxglove or a ROS CLI publisher supplies inputs externally (default: mock)."
        ),
    )
    parser.add_argument(
        "--external-input-publisher",
        default="",
        help=(
            "Required nonempty acknowledgement label with --offline-decision --input-owner manual_ros, "
            "for example foxglove or ros2-cli. The simulator never executes or starts this publisher."
        ),
    )
    parser.add_argument(
        "--mock-preset",
        choices=tuple(MOCK_PRESETS.keys()),
        default="none",
        help="Named simulator-only mock input preset overlay for --offline-decision (default: none).",
    )
    parser.add_argument(
        "--list-mock-presets",
        action="store_true",
        help="List available --mock-preset overlays and exit.",
    )
    parser.add_argument(
        "--list-mock-sequences",
        action="store_true",
        help="List bundled --mock-sequence examples and exit.",
    )
    parser.add_argument(
        "--list-unit-scenes",
        action="store_true",
        help="List bundled --unit-scene examples and exit.",
    )
    parser.add_argument(
        "--trace-on",
        action="store_true",
        help="Force trace recording in --offline-decision mode.",
    )
    parser.add_argument(
        "--mock-team",
        choices=("red", "blue"),
        default="red",
        help="Mock team color for --offline-decision (default: red).",
    )
    parser.add_argument(
        "--mock-hz",
        type=float,
        default=20.0,
        help="Mock input publish rate for --offline-decision (default: 20).",
    )
    parser.add_argument(
        "--mock-time-left",
        type=int,
        default=420,
        help="Mock /ly/game/time_left value for --offline-decision (default: 420).",
    )
    parser.add_argument(
        "--mock-ammo",
        type=int,
        default=200,
        help="Mock /ly/friend/ammo_left value for --offline-decision (default: 200).",
    )
    parser.add_argument(
        "--mock-posture",
        type=int,
        default=1,
        help="Mock /ly/gimbal/posture value for --offline-decision (default: 1).",
    )
    parser.add_argument(
        "--mock-yaw",
        type=float,
        default=0.0,
        help="Mock /ly/gimbal/angles yaw (default: 0).",
    )
    parser.add_argument(
        "--mock-pitch",
        type=float,
        default=0.0,
        help="Mock /ly/gimbal/angles pitch (default: 0).",
    )
    parser.add_argument("--mock-gimbal-fire-status", type=int, default=0, help="Mock /ly/gimbal/firecode fire_status.")
    parser.add_argument("--mock-gimbal-cap-state", type=int, default=0, help="Mock /ly/gimbal/firecode cap_state.")
    parser.add_argument(
        "--mock-gimbal-follow-mode",
        type=parse_bool,
        default=False,
        help="Mock /ly/gimbal/firecode follow_mode.",
    )
    parser.add_argument(
        "--mock-gimbal-aim-mode",
        type=parse_bool,
        default=False,
        help="Mock /ly/gimbal/firecode aim_mode.",
    )
    parser.add_argument("--mock-gimbal-rotate", type=int, default=0, help="Mock /ly/gimbal/firecode rotate.")
    parser.add_argument(
        "--mock-gimbal-yaw-velocity",
        type=float,
        default=0.0,
        help="Mock /ly/gimbal/chassis angular_velocity in deg/s.",
    )
    parser.add_argument(
        "--mock-gimbal-yaw-angle",
        type=float,
        default=0.0,
        help="Mock /ly/gimbal/chassis steer_angle in deg.",
    )
    parser.add_argument("--mock-cap-v", type=int, default=0, help="Mock /ly/gimbal/capV.")
    parser.add_argument("--mock-self-health", type=int, default=400, help="Mock sentry HP (default: 400).")
    parser.add_argument("--mock-enemy-health", type=int, default=400, help="Default enemy unit HP (default: 400).")
    parser.add_argument("--mock-self-outpost-health", type=int, default=1500, help="Mock self outpost HP.")
    parser.add_argument("--mock-enemy-outpost-health", type=int, default=1500, help="Mock enemy outpost HP.")
    parser.add_argument("--mock-self-base-health", type=int, default=5000, help="Mock self base HP.")
    parser.add_argument("--mock-enemy-base-health", type=int, default=5000, help="Mock enemy base HP.")
    parser.add_argument("--mock-team-buff-recovery", type=int, default=0, help="Mock /ly/team/buff recoverybuff.")
    parser.add_argument("--mock-team-buff-cooling", type=int, default=0, help="Mock /ly/team/buff coolingbuff.")
    parser.add_argument("--mock-team-buff-defence", type=int, default=0, help="Mock /ly/team/buff defencebuff.")
    parser.add_argument(
        "--mock-team-buff-vulnerability",
        type=int,
        default=0,
        help="Mock /ly/team/buff vulnerabilitybuff.",
    )
    parser.add_argument("--mock-team-buff-attack", type=int, default=0, help="Mock /ly/team/buff attackbuff.")
    parser.add_argument(
        "--mock-team-buff-remaining-energy",
        type=int,
        default=0,
        help="Mock /ly/team/buff remainingenergy.",
    )
    parser.add_argument("--mock-event-raw", type=int, default=0, help="Mock /ly/game/event_data raw.")
    parser.add_argument(
        "--mock-event-center-gain-point-status",
        type=int,
        default=0,
        help="Mock event_data center_gain_point_status (0=unoccupied, 1=ours, 2=enemy, 3=contested).",
    )
    parser.add_argument(
        "--mock-event-self-small-energy-status",
        type=int,
        default=0,
        help="Mock event_data self_small_energy_status.",
    )
    parser.add_argument(
        "--mock-event-self-large-energy-status",
        type=int,
        default=0,
        help="Mock event_data self_large_energy_status.",
    )
    parser.add_argument(
        "--mock-event-self-fortress-gain-point-status",
        type=int,
        default=0,
        help="Mock event_data self_fortress_gain_point_status.",
    )
    parser.add_argument(
        "--mock-event-self-outpost-gain-point-status",
        type=int,
        default=0,
        help="Mock event_data self_outpost_gain_point_status.",
    )
    parser.add_argument(
        "--mock-event-self-base-gain-point-status",
        type=parse_bool,
        default=False,
        help="Mock event_data self_base_gain_point_status.",
    )
    parser.add_argument(
        "--mock-sentry-can-activate-energy",
        type=parse_bool,
        default=False,
        help="Mock /ly/game/sentry/info can_activate_energy_mechanism.",
    )
    parser.add_argument("--mock-rfid-raw", type=int, default=0, help="Mock /ly/game/rfid raw.")
    parser.add_argument(
        "--mock-rfid-has-status-2",
        type=parse_bool,
        default=False,
        help="Mock /ly/game/rfid has_rfid_status_2.",
    )
    parser.add_argument("--mock-rfid-status-2-raw", type=int, default=0, help="Mock /ly/game/rfid rfid_status_2_raw.")
    parser.add_argument(
        "--mock-rfid-center-gain-point",
        type=parse_bool,
        default=False,
        help="Mock RFID center gain point.",
    )
    parser.add_argument("--mock-rfid-self-base", type=parse_bool, default=False, help="Mock self base RFID.")
    parser.add_argument("--mock-rfid-self-fortress", type=parse_bool, default=False, help="Mock self fortress RFID.")
    parser.add_argument("--mock-rfid-self-outpost", type=parse_bool, default=False, help="Mock self outpost RFID.")
    parser.add_argument("--mock-rfid-self-supply", type=parse_bool, default=False, help="Mock self supply RFID.")
    parser.add_argument("--mock-rfid-self-highland", type=parse_bool, default=False, help="Mock self highland RFID.")
    parser.add_argument(
        "--mock-rfid-self-road-crossing",
        type=parse_bool,
        default=False,
        help="Mock self road crossing RFID.",
    )
    parser.add_argument(
        "--mock-rfid-self-central-highland-crossing",
        type=parse_bool,
        default=False,
        help="Mock self central highland crossing RFID.",
    )
    parser.add_argument("--mock-rfid-self-tunnel", type=parse_bool, default=False, help="Mock self tunnel RFID.")
    parser.add_argument("--mock-rfid-self-assembly", type=parse_bool, default=False, help="Mock self assembly RFID.")
    parser.add_argument("--mock-rfid-self-fly-ramp", type=parse_bool, default=False, help="Mock self fly-ramp RFID.")
    parser.add_argument("--mock-rfid-enemy-fortress", type=parse_bool, default=False, help="Mock enemy fortress RFID.")
    parser.add_argument("--mock-rfid-enemy-outpost", type=parse_bool, default=False, help="Mock enemy outpost RFID.")
    parser.add_argument("--mock-rfid-enemy-highland", type=parse_bool, default=False, help="Mock enemy highland RFID.")
    parser.add_argument(
        "--mock-rfid-enemy-road-crossing",
        type=parse_bool,
        default=False,
        help="Mock enemy road crossing RFID.",
    )
    parser.add_argument(
        "--mock-rfid-enemy-central-highland-crossing",
        type=parse_bool,
        default=False,
        help="Mock enemy central highland crossing RFID.",
    )
    parser.add_argument("--mock-rfid-enemy-tunnel", type=parse_bool, default=False, help="Mock enemy tunnel RFID.")
    parser.add_argument("--mock-rfid-enemy-assembly", type=parse_bool, default=False, help="Mock enemy assembly RFID.")
    parser.add_argument("--mock-rfid-enemy-fly-ramp", type=parse_bool, default=False, help="Mock enemy fly-ramp RFID.")
    parser.add_argument("--mock-navi-reached", type=parse_bool, default=False, help="Mock /ly/navi/reached.")
    parser.add_argument("--mock-navi-reachable", type=parse_bool, default=True, help="Mock /ly/navi/reachable.")
    parser.add_argument(
        "--mock-navi-should-rotate",
        type=parse_bool,
        default=True,
        help="Mock /ly/navi/should_rotate.",
    )
    parser.add_argument("--mock-navi-lower-head", type=int, default=0, help="Mock /ly/navi/lower_head.")
    parser.add_argument("--mock-navi-vel-x", type=float, default=0.0, help="Mock /ly/navi/vel x.")
    parser.add_argument("--mock-navi-vel-y", type=float, default=0.0, help="Mock /ly/navi/vel y.")
    parser.add_argument(
        "--mock-publish-self-position",
        type=parse_bool,
        default=True,
        help="Publish mock /ly/navi/position (default: true).",
    )
    parser.add_argument(
        "--mock-self-position-x",
        type=int,
        default=-1,
        help="Mock /ly/navi/position official-map x cm; -1 uses team base default.",
    )
    parser.add_argument(
        "--mock-self-position-y",
        type=int,
        default=-1,
        help="Mock /ly/navi/position official-map y cm; -1 uses team base default.",
    )
    parser.add_argument(
        "--mock-publish-uwb-position",
        type=parse_bool,
        default=False,
        help="Publish mock /ly/friend/uwb_pos using raw y expected by behavior_tree (default: false).",
    )
    parser.add_argument(
        "--mock-uwb-position-x",
        type=int,
        default=-1,
        help="Mock /ly/friend/uwb_pos official-map x cm; -1 follows mock self position.",
    )
    parser.add_argument(
        "--mock-uwb-position-y",
        type=int,
        default=-1,
        help="Mock /ly/friend/uwb_pos official-map y cm; -1 follows mock self position.",
    )
    parser.add_argument(
        "--mock-official-target-valid",
        type=parse_bool,
        default=False,
        help="Publish mock /ly/navi/target_official.",
    )
    parser.add_argument("--mock-official-target-x", type=int, default=0, help="Mock official target x cm.")
    parser.add_argument("--mock-official-target-y", type=int, default=0, help="Mock official target y cm.")
    parser.add_argument(
        "--mock-official-target-armor-type",
        type=int,
        default=1,
        help="Mock official target armor type id.",
    )
    parser.add_argument(
        "--mock-bullet-initial-speed",
        type=float,
        default=0.0,
        help="Mock /ly/game/bullet initial_speed; <=0 leaves has_initial_speed false.",
    )
    parser.add_argument(
        "--mock-bullet-has-shoot-data",
        type=parse_bool,
        default=False,
        help="Mock /ly/game/bullet has_shoot_data.",
    )
    parser.add_argument("--mock-bullet-type", type=int, default=0, help="Mock /ly/game/bullet bullet_type.")
    parser.add_argument(
        "--mock-bullet-shooter-number",
        type=int,
        default=0,
        help="Mock /ly/game/bullet shooter_number.",
    )
    parser.add_argument(
        "--mock-bullet-launching-frequency",
        type=int,
        default=0,
        help="Mock /ly/game/bullet launching_frequency.",
    )
    parser.add_argument(
        "--mock-bullet-projectile-allowance-17mm",
        type=int,
        default=0,
        help="Mock /ly/game/bullet projectile_allowance_17mm.",
    )
    parser.add_argument(
        "--mock-bullet-projectile-allowance-42mm",
        type=int,
        default=0,
        help="Mock /ly/game/bullet projectile_allowance_42mm.",
    )
    parser.add_argument(
        "--mock-bullet-remaining-gold-coin",
        type=int,
        default=0,
        help="Mock /ly/game/bullet remaining_gold_coin.",
    )
    parser.add_argument(
        "--mock-bullet-projectile-allowance-fortress-17mm",
        type=int,
        default=0,
        help="Mock /ly/game/bullet projectile_allowance_fortress_17mm.",
    )
    parser.add_argument(
        "--mock-external-aim",
        type=parse_bool,
        default=False,
        help="Publish optional sentry_msgs external aim topics /ly/aim/armor_targets and /ly/aim/result.",
    )
    parser.add_argument("--mock-external-aim-follow", type=parse_bool, default=True, help="Mock AimResult.follow.")
    parser.add_argument("--mock-external-aim-fire", type=parse_bool, default=True, help="Mock AimResult.fire.")
    parser.add_argument("--mock-external-aim-yaw", type=float, default=0.0, help="Mock AimResult.yaw.")
    parser.add_argument("--mock-external-aim-pitch", type=float, default=0.0, help="Mock AimResult.pitch.")
    parser.add_argument("--mock-external-aim-target-id", type=int, default=1, help="Mock AimTarget.id.")
    parser.add_argument("--mock-external-aim-target-x", type=float, default=6.0, help="Mock AimTarget.position.x.")
    parser.add_argument("--mock-external-aim-target-y", type=float, default=0.0, help="Mock AimTarget.position.y.")
    parser.add_argument("--mock-external-aim-target-z", type=float, default=0.0, help="Mock AimTarget.position.z.")
    parser.add_argument(
        "--mock-external-aim-frame",
        default="gimbal_world",
        help="Mock external aim frame_id (default: gimbal_world).",
    )
    parser.add_argument(
        "--bypass-is-start",
        action="store_true",
        help="Debug only: bypass /ly/game/is_start gate in offline mode.",
    )
    parser.add_argument(
        "--keep-to-navi",
        "--keep-tf-goal-bridge",
        dest="keep_to_navi",
        action="store_true",
        help=(
            "Offline mode only: keep NaviSetting.ToNavi from source config. "
            "Default is to force official map coordinates (ToNavi=false)."
        ),
    )
    parser.add_argument(
        "--match-duration-sec",
        type=int,
        default=420,
        help="Offline super confrontation match duration in seconds (default: 420).",
    )
    parser.add_argument(
        "--control-file",
        default="/tmp/simulator_match_control.jsonl",
        help="JSONL command channel between live viewer and mock inputs (default: /tmp/simulator_match_control.jsonl).",
    )
    parser.add_argument(
        "--unit-scene",
        default="",
        help="JSON/YAML unit scene passed to both live viewer and offline mock inputs.",
    )
    parser.add_argument(
        "--mock-sequence",
        default="",
        help=(
            "JSON/YAML timed control-bus sequence for --offline-decision. "
            "Actions append existing simulator control commands to --control-file."
        ),
    )
    parser.add_argument(
        "--mock-sequence-poll-sec",
        type=float,
        default=0.05,
        help="Mock sequence scheduler poll interval in seconds (default: 0.05).",
    )
    parser.add_argument(
        "--list-configs",
        action="store_true",
        help="List available presets under src/behavior_tree/Scripts/ConfigJson and exit.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the final start.sh command without executing.",
    )
    parser.add_argument(
        "--play",
        action="store_true",
        help="Open pygame viewer on the trace file after recording exits.",
    )
    parser.add_argument(
        "--live-view",
        action="store_true",
        help="Start the browser Tactical Board while decision is running (follow growing trace file).",
    )
    parser.add_argument(
        "--viewer-config",
        default="",
        help="Optional simulator YAML config for the live viewer (for example src/simulator/config/league_3v3.yaml).",
    )
    parser.add_argument(
        "--live-debug-pygame",
        action="store_true",
        help="Use the native pygame debug renderer for --live-view instead of the browser-only runtime.",
    )
    parser.add_argument(
        "--live-follow-poll",
        type=float,
        default=0.25,
        help="Follow poll interval for --live-view (default: 0.25s).",
    )
    parser.add_argument(
        "--live-web-host",
        default="",
        help="Override viewer web host in live view (default: YAML web_stream.host).",
    )
    parser.add_argument(
        "--live-web-port",
        type=int,
        default=0,
        help="Override viewer web port in live view (default: YAML web_stream.port).",
    )
    parser.add_argument(
        "--live-web-fps",
        type=float,
        default=0.0,
        help="Override viewer web stream fps in live view (default: YAML web_stream.fps).",
    )
    parser.add_argument(
        "--live-web-jpeg-quality",
        type=int,
        default=0,
        help="Override viewer web JPEG quality in live view (default: YAML web_stream.jpeg_quality).",
    )
    parser.add_argument(
        "--ros-state-file",
        default="/tmp/simulator_ros_topics.json",
        help="Live ROS topic state JSON path for --live-view (default: /tmp/simulator_ros_topics.json).",
    )
    parser.add_argument(
        "--no-live-ros-monitor",
        action="store_true",
        help="Disable the live ROS topic monitor used by the viewer right panel.",
    )
    parser.add_argument(
        "extra_launch_args",
        nargs=argparse.REMAINDER,
        help="Extra launch args passed through to scripts/start.sh. Put them after '--'.",
    )
    raw_argv = list(sys.argv[1:] if argv is None else argv)
    explicit_dests = collect_explicit_cli_dests(parser, raw_argv)
    args = parser.parse_args(raw_argv)
    if (
        args.mock_preset != "none"
        and not args.offline_decision
        and not args.list_mock_presets
        and not args.list_mock_sequences
        and not args.list_unit_scenes
    ):
        parser.error("--mock-preset requires --offline-decision")
    if str(args.mock_sequence).strip() and not args.offline_decision:
        parser.error("--mock-sequence requires --offline-decision")
    external_input_publisher = str(args.external_input_publisher).strip()
    if args.offline_decision and args.input_owner == "manual_ros" and not external_input_publisher:
        parser.error(
            "--offline-decision --input-owner manual_ros requires --external-input-publisher "
            "(for example foxglove or ros2-cli)"
        )
    if args.input_owner == "mock" and external_input_publisher:
        parser.error("--external-input-publisher is only valid with --input-owner manual_ros")
    if args.input_owner == "manual_ros" and args.mock_preset != "none":
        parser.error("--mock-preset is only available with --input-owner mock")
    if args.input_owner == "manual_ros" and str(args.mock_sequence).strip():
        parser.error("--mock-sequence is only available with --input-owner mock")
    if args.offline_decision:
        apply_mock_preset(args, explicit_dests)
    if args.every < 1:
        parser.error("--every must be >= 1")
    if args.mock_hz <= 0:
        parser.error("--mock-hz must be > 0")
    if args.live_follow_poll <= 0:
        parser.error("--live-follow-poll must be > 0")
    if args.live_web_port < 0 or args.live_web_port > 65535:
        parser.error("--live-web-port must be in [0, 65535]")
    if args.live_web_fps < 0:
        parser.error("--live-web-fps must be >= 0")
    if args.live_web_jpeg_quality < 0 or args.live_web_jpeg_quality > 100:
        parser.error("--live-web-jpeg-quality must be in [0, 100]")
    if not args.ros_state_file.strip():
        parser.error("--ros-state-file must not be empty")
    if args.match_duration_sec <= 0:
        parser.error("--match-duration-sec must be > 0")
    if args.mock_sequence_poll_sec <= 0:
        parser.error("--mock-sequence-poll-sec must be > 0")
    if str(args.mock_sequence).strip() and not str(args.control_file).strip():
        parser.error("--mock-sequence requires a non-empty --control-file")
    if args.play and args.live_view:
        parser.error("--play and --live-view are mutually exclusive")
    return args


def behavior_tree_root(root: Path) -> Path:
    return (root / "src" / "behavior_tree").resolve()


def config_dir(root: Path) -> Path:
    return (behavior_tree_root(root) / "Scripts" / "ConfigJson").resolve()


def normalize_extra_launch_args(raw: list[str]) -> list[str]:
    if raw and raw[0] == "--":
        return raw[1:]
    return raw


def iter_config_paths(config_root: Path, include_legacy: bool = False) -> list[Path]:
    paths = sorted(config_root.glob("*.json"))
    paths.extend(sorted((config_root / "league").glob("*.json")))
    paths.extend(sorted((config_root / "regional" / "debug").glob("*.json")))
    paths.extend(sorted((config_root / "regional" / "test").glob("*.json")))
    return paths


def choose_trace_path(root: Path, configured: str) -> Path:
    if configured.strip():
        raw = Path(configured).expanduser()
        if raw.is_absolute():
            return raw.resolve()
        return (root / raw).resolve()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return (root / "log" / f"decision_trace_{stamp}.jsonl").resolve()


def resolve_runtime_path(root: Path, configured: str) -> Path:
    raw = Path(configured).expanduser()
    if raw.is_absolute():
        return raw.resolve()
    return (root / raw).resolve()


def resolve_bt_config_path(root: Path, mode: str, configured: str) -> tuple[Path, str]:
    bt_root = behavior_tree_root(root)
    cfg_root = config_dir(root)

    candidates: list[Path] = []
    if not configured.strip():
        default_name = DEFAULT_BT_CONFIG_BY_MODE[mode]
        candidates.append(cfg_root / default_name)
    else:
        raw = Path(configured).expanduser()
        if raw.is_absolute():
            candidates.append(raw)
        else:
            candidates.append((root / raw).resolve())
            candidates.append((bt_root / raw).resolve())
            candidates.append((cfg_root / raw).resolve())
            candidates.append((cfg_root / raw.name).resolve())
            candidates.append((cfg_root / "league" / raw.name).resolve())
            candidates.append((cfg_root / "regional" / "debug" / raw.name).resolve())
            candidates.append((cfg_root / "regional" / "test" / raw.name).resolve())
            if raw.suffix != ".json":
                candidates.append((cfg_root / f"{raw.name}.json").resolve())
                candidates.append((cfg_root / "league" / f"{raw.name}.json").resolve())
                candidates.append((cfg_root / "regional" / "debug" / f"{raw.name}.json").resolve())
                candidates.append((cfg_root / "regional" / "test" / f"{raw.name}.json").resolve())

    seen: set[Path] = set()
    existing: Path | None = None
    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        if resolved.exists() and resolved.is_file():
            existing = resolved
            break

    if existing is None:
        attempted = "\n".join(str(c) for c in seen)
        raise FileNotFoundError(f"Cannot find bt config. Tried:\n{attempted}")

    try:
        launch_rel = existing.relative_to(bt_root).as_posix()
        launch_value = launch_rel
    except ValueError:
        launch_value = existing.as_posix()

    return existing, launch_value


def print_configs(root: Path) -> int:
    cfg_root = config_dir(root)
    if not cfg_root.exists():
        print(f"config dir not found: {cfg_root}", file=sys.stderr)
        return 2
    print(f"Config root: {cfg_root}")
    for item in iter_config_paths(cfg_root):
        rel = item.relative_to(cfg_root).as_posix()
        print(rel)
    return 0


def has_launch_arg(args: list[str], key: str) -> bool:
    prefix = f"{key}:="
    return any(item.startswith(prefix) for item in args)


def should_enable_trace(args: argparse.Namespace) -> bool:
    if args.offline_decision:
        return args.trace_on or args.play or args.live_view or bool(args.trace.strip())
    return True


def build_start_command(
    root: Path,
    entry: str,
    mode: str,
    bt_config_launch: str,
    debug_bypass_is_start: bool,
    trace_enabled: bool,
    trace_path: Path | None,
    every: int,
    offline_decision: bool,
    extra_launch_args: list[str],
) -> list[str]:
    if offline_decision:
        command: list[str] = [
            str((root / "scripts" / "launch" / "start_sentry_all.sh").resolve()),
            "--mode",
            mode,
            f"bt_config_file:={bt_config_launch}",
        ]
    else:
        command = [
            str((root / "scripts" / "start.sh").resolve()),
            entry,
            "--mode",
            mode,
            f"bt_config_file:={bt_config_launch}",
        ]

    if offline_decision:
        offline_defaults = (
            ("offline", "true"),
            ("use_gimbal", "false"),
            ("use_behavior_tree", "true"),
            ("debug_bypass_is_start", "true" if debug_bypass_is_start else "false"),
            ("runtime_rearm_start_gate", "true"),
        )
        for key, value in offline_defaults:
            if not has_launch_arg(extra_launch_args, key):
                command.append(f"{key}:={value}")

    if trace_enabled and trace_path is not None:
        if not has_launch_arg(extra_launch_args, "decision_trace_enabled"):
            command.append("decision_trace_enabled:=true")
        if not has_launch_arg(extra_launch_args, "decision_trace_file"):
            command.append(f"decision_trace_file:={trace_path.as_posix()}")
        if not has_launch_arg(extra_launch_args, "decision_trace_every_n_ticks"):
            command.append(f"decision_trace_every_n_ticks:={every}")

    command.extend(extra_launch_args)
    return command


def build_mock_command(root: Path, args: argparse.Namespace) -> tuple[list[str], str]:
    if args.input_owner != "mock":
        raise ValueError("simulator.mock_inputs is only valid with --input-owner mock")
    system_python = Path("/usr/bin/python3")
    python_exec = str(system_python if system_python.exists() else Path(sys.executable))
    python_args = [
        python_exec,
        "-m",
        "simulator.mock_inputs",
        "--input-owner",
        "mock",
        "--team",
        args.mock_team,
        "--hz",
        str(args.mock_hz),
        "--time-left",
        str(args.mock_time_left),
        "--ammo-left",
        str(args.mock_ammo),
        "--posture",
        str(args.mock_posture),
        "--yaw",
        str(args.mock_yaw),
        "--pitch",
        str(args.mock_pitch),
        "--simulate-match",
        "true",
        "--start-running",
        "false",
        "--match-duration-sec",
        str(args.match_duration_sec),
    ]
    mock_pass_through: list[tuple[str, object]] = [
        ("--gimbal-fire-status", args.mock_gimbal_fire_status),
        ("--gimbal-cap-state", args.mock_gimbal_cap_state),
        ("--gimbal-follow-mode", bool_text(args.mock_gimbal_follow_mode)),
        ("--gimbal-aim-mode", bool_text(args.mock_gimbal_aim_mode)),
        ("--gimbal-rotate", args.mock_gimbal_rotate),
        ("--gimbal-yaw-velocity", args.mock_gimbal_yaw_velocity),
        ("--gimbal-yaw-angle", args.mock_gimbal_yaw_angle),
        ("--mock-cap-v", args.mock_cap_v),
        ("--self-health", args.mock_self_health),
        ("--enemy-health", args.mock_enemy_health),
        ("--self-outpost-health", args.mock_self_outpost_health),
        ("--enemy-outpost-health", args.mock_enemy_outpost_health),
        ("--self-base-health", args.mock_self_base_health),
        ("--enemy-base-health", args.mock_enemy_base_health),
        ("--team-buff-recovery", args.mock_team_buff_recovery),
        ("--team-buff-cooling", args.mock_team_buff_cooling),
        ("--team-buff-defence", args.mock_team_buff_defence),
        ("--team-buff-vulnerability", args.mock_team_buff_vulnerability),
        ("--team-buff-attack", args.mock_team_buff_attack),
        ("--team-buff-remaining-energy", args.mock_team_buff_remaining_energy),
        ("--event-raw", args.mock_event_raw),
        ("--event-center-gain-point-status", args.mock_event_center_gain_point_status),
        ("--event-self-small-energy-status", args.mock_event_self_small_energy_status),
        ("--event-self-large-energy-status", args.mock_event_self_large_energy_status),
        (
            "--event-self-fortress-gain-point-status",
            args.mock_event_self_fortress_gain_point_status,
        ),
        (
            "--event-self-outpost-gain-point-status",
            args.mock_event_self_outpost_gain_point_status,
        ),
        ("--event-self-base-gain-point-status", bool_text(args.mock_event_self_base_gain_point_status)),
        ("--sentry-can-activate-energy", bool_text(args.mock_sentry_can_activate_energy)),
        ("--rfid-raw", args.mock_rfid_raw),
        ("--rfid-has-status-2", bool_text(args.mock_rfid_has_status_2)),
        ("--rfid-status-2-raw", args.mock_rfid_status_2_raw),
        ("--rfid-center-gain-point", bool_text(args.mock_rfid_center_gain_point)),
        ("--rfid-self-base", bool_text(args.mock_rfid_self_base)),
        ("--rfid-self-fortress", bool_text(args.mock_rfid_self_fortress)),
        ("--rfid-self-outpost", bool_text(args.mock_rfid_self_outpost)),
        ("--rfid-self-supply", bool_text(args.mock_rfid_self_supply)),
        ("--rfid-self-highland", bool_text(args.mock_rfid_self_highland)),
        ("--rfid-self-road-crossing", bool_text(args.mock_rfid_self_road_crossing)),
        (
            "--rfid-self-central-highland-crossing",
            bool_text(args.mock_rfid_self_central_highland_crossing),
        ),
        ("--rfid-self-tunnel", bool_text(args.mock_rfid_self_tunnel)),
        ("--rfid-self-assembly", bool_text(args.mock_rfid_self_assembly)),
        ("--rfid-self-fly-ramp", bool_text(args.mock_rfid_self_fly_ramp)),
        ("--rfid-enemy-fortress", bool_text(args.mock_rfid_enemy_fortress)),
        ("--rfid-enemy-outpost", bool_text(args.mock_rfid_enemy_outpost)),
        ("--rfid-enemy-highland", bool_text(args.mock_rfid_enemy_highland)),
        ("--rfid-enemy-road-crossing", bool_text(args.mock_rfid_enemy_road_crossing)),
        (
            "--rfid-enemy-central-highland-crossing",
            bool_text(args.mock_rfid_enemy_central_highland_crossing),
        ),
        ("--rfid-enemy-tunnel", bool_text(args.mock_rfid_enemy_tunnel)),
        ("--rfid-enemy-assembly", bool_text(args.mock_rfid_enemy_assembly)),
        ("--rfid-enemy-fly-ramp", bool_text(args.mock_rfid_enemy_fly_ramp)),
        ("--navi-reached", bool_text(args.mock_navi_reached)),
        ("--navi-reachable", bool_text(args.mock_navi_reachable)),
        ("--navi-should-rotate", bool_text(args.mock_navi_should_rotate)),
        ("--mock-navi-lower-head", args.mock_navi_lower_head),
        ("--mock-navi-vel-x", args.mock_navi_vel_x),
        ("--mock-navi-vel-y", args.mock_navi_vel_y),
        ("--publish-self-position", bool_text(args.mock_publish_self_position)),
        ("--self-position-x", args.mock_self_position_x),
        ("--self-position-y", args.mock_self_position_y),
        ("--publish-uwb-position", bool_text(args.mock_publish_uwb_position)),
        ("--uwb-position-x", args.mock_uwb_position_x),
        ("--uwb-position-y", args.mock_uwb_position_y),
        ("--official-target-valid", bool_text(args.mock_official_target_valid)),
        ("--official-target-x", args.mock_official_target_x),
        ("--official-target-y", args.mock_official_target_y),
        ("--official-target-armor-type", args.mock_official_target_armor_type),
        ("--mock-bullet-initial-speed", args.mock_bullet_initial_speed),
        ("--mock-bullet-has-shoot-data", bool_text(args.mock_bullet_has_shoot_data)),
        ("--mock-bullet-type", args.mock_bullet_type),
        ("--mock-bullet-shooter-number", args.mock_bullet_shooter_number),
        ("--mock-bullet-launching-frequency", args.mock_bullet_launching_frequency),
        ("--mock-bullet-projectile-allowance-17mm", args.mock_bullet_projectile_allowance_17mm),
        ("--mock-bullet-projectile-allowance-42mm", args.mock_bullet_projectile_allowance_42mm),
        ("--mock-bullet-remaining-gold-coin", args.mock_bullet_remaining_gold_coin),
        (
            "--mock-bullet-projectile-allowance-fortress-17mm",
            args.mock_bullet_projectile_allowance_fortress_17mm,
        ),
        ("--mock-external-aim", bool_text(args.mock_external_aim)),
        ("--mock-external-aim-follow", bool_text(args.mock_external_aim_follow)),
        ("--mock-external-aim-fire", bool_text(args.mock_external_aim_fire)),
        ("--mock-external-aim-yaw", args.mock_external_aim_yaw),
        ("--mock-external-aim-pitch", args.mock_external_aim_pitch),
        ("--mock-external-aim-target-id", args.mock_external_aim_target_id),
        ("--mock-external-aim-target-x", args.mock_external_aim_target_x),
        ("--mock-external-aim-target-y", args.mock_external_aim_target_y),
        ("--mock-external-aim-target-z", args.mock_external_aim_target_z),
        ("--mock-external-aim-frame", args.mock_external_aim_frame),
    ]
    for cli_name, value in mock_pass_through:
        python_args.extend([cli_name, str(value)])
    if str(args.control_file).strip():
        python_args.extend(["--control-file", str(Path(args.control_file).expanduser().resolve())])
    if str(args.unit_scene).strip():
        python_args.extend(["--unit-scene", resolve_runtime_path(root, args.unit_scene).as_posix()])

    setup_cmds: list[str] = []
    ros_setup = Path("/opt/ros/humble/setup.bash")
    ws_setup = root / "install" / "setup.bash"
    if ros_setup.exists():
        setup_cmds.append(f"source {shlex.quote(str(ros_setup))}")
    if ws_setup.exists():
        setup_cmds.append(f"source {shlex.quote(str(ws_setup))}")
    setup_cmds.append("mkdir -p /tmp/ros2_logs")
    setup_cmds.append("export ROS_LOG_DIR=/tmp/ros2_logs")
    setup_cmds.append(
        f"export PYTHONPATH={shlex.quote(str((root / 'src' / 'simulator').resolve()))}:$PYTHONPATH"
    )

    quoted_python = " ".join(shlex.quote(item) for item in python_args)
    setup_cmds.append(quoted_python)
    shell_cmd = " && ".join(setup_cmds)
    return (["bash", "-lc", shell_cmd], shell_cmd)


def build_mock_sequence_command(root: Path, sequence_path: Path, control_file: str, poll_sec: float) -> tuple[list[str], str]:
    python_args = [
        sys.executable,
        "-m",
        "simulator.mock_sequence",
        sequence_path.as_posix(),
        "--control-file",
        str(Path(control_file).expanduser().resolve()),
        "--poll-sec",
        str(poll_sec),
    ]
    setup_cmds = [
        f"export PYTHONPATH={shlex.quote(str((root / 'src' / 'simulator').resolve()))}:$PYTHONPATH",
        " ".join(shlex.quote(item) for item in python_args),
    ]
    shell_cmd = " && ".join(setup_cmds)
    return (["bash", "-lc", shell_cmd], shell_cmd)


def load_mock_sequence_for_start(sequence_path: Path) -> int:
    from .mock_sequence import load_mock_sequence_file

    sequence = load_mock_sequence_file(sequence_path)
    return sequence.action_count


def load_unit_scene_for_start(unit_scene_path: Path) -> int:
    from .interactive_inputs import load_unit_scene_file

    units = load_unit_scene_file(unit_scene_path)
    return len(units)


def build_ros_topic_monitor_command(root: Path, state_file: str) -> tuple[list[str], str]:
    system_python = Path("/usr/bin/python3")
    python_exec = str(system_python if system_python.exists() else Path(sys.executable))
    python_args = [
        python_exec,
        "-m",
        "simulator.ros_topic_monitor",
        "--state-file",
        str(Path(state_file).expanduser().resolve()),
    ]

    setup_cmds: list[str] = []
    ros_setup = Path("/opt/ros/humble/setup.bash")
    ws_setup = root / "install" / "setup.bash"
    if ros_setup.exists():
        setup_cmds.append(f"source {shlex.quote(str(ros_setup))}")
    if ws_setup.exists():
        setup_cmds.append(f"source {shlex.quote(str(ws_setup))}")
    setup_cmds.append("mkdir -p /tmp/ros2_logs")
    setup_cmds.append("export ROS_LOG_DIR=/tmp/ros2_logs")
    setup_cmds.append(
        f"export PYTHONPATH={shlex.quote(str((root / 'src' / 'simulator').resolve()))}:$PYTHONPATH"
    )
    quoted_python = " ".join(shlex.quote(item) for item in python_args)
    setup_cmds.append(quoted_python)
    shell_cmd = " && ".join(setup_cmds)
    return (["bash", "-lc", shell_cmd], shell_cmd)


def build_offline_bt_config(root: Path, source_config: Path) -> Path:
    with source_config.open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"invalid config root (expect object): {source_config}")

    navi = data.get("NaviSetting")
    if not isinstance(navi, dict):
        navi = {}
        data["NaviSetting"] = navi
    navi["ToNavi"] = False
    navi.pop("UseTfGoalBridge", None)

    out_dir = (root / "log" / "simulator").resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{source_config.stem}.offline_official_goal_pos.json"
    with out_path.open("w", encoding="utf-8") as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return out_path


def run_viewer(trace_path: Path, unit_scene: str = "") -> int:
    cmd = [sys.executable, "-m", "simulator.main", trace_path.as_posix(), "--debug-pygame", "--no-web-stream"]
    if unit_scene.strip():
        cmd.extend(["--unit-scene", unit_scene])
    return subprocess.run(cmd, check=False).returncode


def build_live_viewer_command(
    trace_path: Path,
    follow_poll: float,
    web_host: str,
    web_port: int,
    web_fps: float,
    web_jpeg_quality: int,
    control_file: str,
    match_duration_sec: int,
    ros_state_file: str,
    unit_scene: str,
    *,
    input_owner: str = "mock",
    debug_pygame: bool = False,
    viewer_config: str = "",
) -> list[str]:
    cmd = [
        sys.executable,
        "-m",
        "simulator.main",
        trace_path.as_posix(),
        "--follow",
        "--follow-poll",
        f"{follow_poll}",
        "--follow-wait",
        "600",
        "--input-owner",
        input_owner,
    ]
    if web_host.strip():
        cmd.extend(["--web-host", web_host.strip()])
    if web_port > 0:
        cmd.extend(["--web-port", str(web_port)])
    if web_fps > 0:
        cmd.extend(["--web-fps", f"{web_fps}"])
    if web_jpeg_quality > 0:
        cmd.extend(["--web-jpeg-quality", str(web_jpeg_quality)])
    if str(control_file).strip():
        cmd.extend(["--control-file", str(Path(control_file).expanduser().resolve())])
    if match_duration_sec > 0:
        cmd.extend(["--match-duration-sec", str(match_duration_sec)])
    if str(ros_state_file).strip():
        cmd.extend(["--ros-state-file", str(Path(ros_state_file).expanduser().resolve())])
    if str(unit_scene).strip():
        cmd.extend(["--unit-scene", unit_scene])
    if str(viewer_config).strip():
        cmd.extend(["--config", str(Path(viewer_config).expanduser().resolve())])
    if debug_pygame:
        cmd.append("--debug-pygame")
    return cmd


def start_live_viewer(
    trace_path: Path,
    follow_poll: float,
    web_host: str,
    web_port: int,
    web_fps: float,
    web_jpeg_quality: int,
    control_file: str,
    match_duration_sec: int,
    ros_state_file: str,
    unit_scene: str,
    *,
    input_owner: str = "mock",
    debug_pygame: bool = False,
    viewer_config: str = "",
) -> subprocess.Popen[bytes]:
    cmd = build_live_viewer_command(
        trace_path,
        follow_poll,
        web_host,
        web_port,
        web_fps,
        web_jpeg_quality,
        control_file,
        match_duration_sec,
        ros_state_file,
        unit_scene,
        input_owner=input_owner,
        debug_pygame=debug_pygame,
        viewer_config=viewer_config,
    )
    return subprocess.Popen(cmd)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    root = repo_root().resolve()

    if args.list_mock_presets:
        return print_mock_presets()

    if args.list_mock_sequences:
        return print_mock_sequences()

    if args.list_unit_scenes:
        return print_unit_scenes()

    if args.list_configs:
        return print_configs(root)

    extra_launch_args = normalize_extra_launch_args(args.extra_launch_args)
    try:
        config_file, bt_config_launch = resolve_bt_config_path(root, args.mode, args.bt_config)
    except FileNotFoundError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    if args.offline_decision and not args.keep_to_navi:
        try:
            offline_config = build_offline_bt_config(root, config_file)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            print(f"failed to prepare offline BT config: {exc}", file=sys.stderr)
            return 2
        config_file = offline_config
        bt_config_launch = offline_config.as_posix()
        print(f"offline BT config (official map goal_pos): {offline_config}")

    trace_enabled = should_enable_trace(args)
    trace_path: Path | None = None
    if trace_enabled:
        trace_path = choose_trace_path(root, args.trace)
        trace_path.parent.mkdir(parents=True, exist_ok=True)

    start_cmd = build_start_command(
        root=root,
        entry=args.entry,
        mode=args.mode,
        bt_config_launch=bt_config_launch,
        debug_bypass_is_start=args.bypass_is_start,
        trace_enabled=trace_enabled,
        trace_path=trace_path,
        every=args.every,
        offline_decision=args.offline_decision,
        extra_launch_args=extra_launch_args,
    )

    print(f"decision mode: {args.mode}")
    print(f"bt config: {config_file}")
    print(f"input owner: {args.input_owner}")
    if args.input_owner == "manual_ros":
        print(f"external input publisher: {str(args.external_input_publisher).strip()}")
    if args.offline_decision:
        if args.input_owner == "mock":
            print("run profile: offline decision test (behavior_tree + mock inputs)")
        else:
            print("run profile: offline decision test (behavior_tree + external manual ROS inputs)")
        if args.mock_preset != "none":
            print(f"mock preset: {args.mock_preset} - {MOCK_PRESET_DESCRIPTIONS[args.mock_preset]}")
    else:
        print("run profile: stack run")
    print(f"trace enabled: {str(trace_enabled).lower()}")
    if trace_path is not None:
        print(f"trace output: {trace_path}")
    print(f"start command: {' '.join(start_cmd)}")
    mock_cmd: list[str] | None = None
    mock_cmd_desc = ""
    sequence_cmd: list[str] | None = None
    sequence_cmd_desc = ""
    ros_monitor_cmd: list[str] | None = None
    ros_monitor_cmd_desc = ""
    control_path: Path | None = None
    unit_scene_path: Path | None = None
    mock_sequence_path: Path | None = None
    ros_state_path = Path(args.ros_state_file).expanduser().resolve()
    if str(args.unit_scene).strip():
        unit_scene_path = resolve_runtime_path(root, args.unit_scene)
        print(f"unit scene: {unit_scene_path}")
        try:
            unit_scene_count = load_unit_scene_for_start(unit_scene_path)
        except (OSError, RuntimeError, ValueError) as exc:
            print(f"failed to load unit scene {unit_scene_path}: {exc}", file=sys.stderr)
            return 2
        print(f"unit scene units: {unit_scene_count}")
    if str(args.mock_sequence).strip():
        mock_sequence_path = resolve_runtime_path(root, args.mock_sequence)
        print(f"mock sequence: {mock_sequence_path}")
        try:
            mock_sequence_action_count = load_mock_sequence_for_start(mock_sequence_path)
        except (OSError, RuntimeError, ValueError) as exc:
            print(f"failed to load mock sequence {mock_sequence_path}: {exc}", file=sys.stderr)
            return 2
        print(f"mock sequence actions: {mock_sequence_action_count}")
    if args.offline_decision and args.input_owner == "mock":
        if str(args.control_file).strip():
            control_path = Path(args.control_file).expanduser().resolve()
            print(f"control file: {control_path}")
        mock_cmd, mock_cmd_desc = build_mock_command(root, args)
        print(f"mock command: {mock_cmd_desc}")
        if mock_sequence_path is not None:
            sequence_cmd, sequence_cmd_desc = build_mock_sequence_command(
                root,
                mock_sequence_path,
                args.control_file,
                args.mock_sequence_poll_sec,
            )
            print(f"mock sequence command: {sequence_cmd_desc}")
    if (args.live_view and not args.no_live_ros_monitor) or args.input_owner == "manual_ros":
        ros_monitor_cmd, ros_monitor_cmd_desc = build_ros_topic_monitor_command(root, args.ros_state_file)
        print(f"live ROS topic state: {ros_state_path}")
        print(f"ROS topic monitor command: {ros_monitor_cmd_desc}")
    if args.live_view and trace_path is not None:
        live_viewer_cmd = build_live_viewer_command(
            trace_path,
            args.live_follow_poll,
            args.live_web_host,
            args.live_web_port,
            args.live_web_fps,
            args.live_web_jpeg_quality,
            args.control_file,
            args.match_duration_sec,
            args.ros_state_file,
            unit_scene_path.as_posix() if unit_scene_path is not None else "",
            input_owner=args.input_owner,
            debug_pygame=args.live_debug_pygame,
            viewer_config=args.viewer_config,
        )
        print(f"live viewer command: {' '.join(shlex.quote(item) for item in live_viewer_cmd)}")

    if args.dry_run:
        return 0

    mock_proc: subprocess.Popen[bytes] | None = None
    sequence_proc: subprocess.Popen[bytes] | None = None
    viewer_proc: subprocess.Popen[bytes] | None = None
    ros_monitor_proc: subprocess.Popen[bytes] | None = None
    try:
        if control_path is not None:
            control_path.parent.mkdir(parents=True, exist_ok=True)
            if control_path.exists():
                try:
                    control_path.unlink()
                except OSError:
                    pass
        if ros_state_path.exists():
            try:
                ros_state_path.unlink()
            except OSError:
                pass
        if mock_cmd is not None:
            mock_proc = subprocess.Popen(mock_cmd)
            # If mock exits immediately, offline inputs are not being published.
            time.sleep(0.5)
            mock_rc = mock_proc.poll()
            if mock_rc is not None:
                print(
                    (
                        "mock inputs exited early "
                        f"(rc={mock_rc}); offline /ly/gimbal/angles may be missing. "
                        "Likely Python/ROS env mismatch (e.g. conda)."
                    ),
                    file=sys.stderr,
                )
                return 2
        if sequence_cmd is not None:
            sequence_proc = subprocess.Popen(sequence_cmd)
            time.sleep(0.2)
            sequence_rc = sequence_proc.poll()
            if sequence_rc is not None and sequence_rc != 0:
                print(
                    f"mock sequence exited early (rc={sequence_rc}); scripted control commands were not published.",
                    file=sys.stderr,
                )
                return 2
        if ros_monitor_cmd is not None:
            ros_monitor_proc = subprocess.Popen(ros_monitor_cmd)
            time.sleep(0.3)
            monitor_rc = ros_monitor_proc.poll()
            if monitor_rc is not None:
                print(
                    f"live ROS topic monitor exited early (rc={monitor_rc}); right panel will use trace only.",
                    file=sys.stderr,
                )
                ros_monitor_proc = None
        if args.live_view:
            if trace_path is None:
                print("trace is disabled but --live-view requested", file=sys.stderr)
                return 2
            cleanup_stale_live_viewers()
            if args.live_web_port > 0:
                print(
                    "live web stream: "
                    f"http://127.0.0.1:{args.live_web_port}/ "
                    f"(LAN: http://<your-ip>:{args.live_web_port}/)"
                )
            else:
                print("live web stream: use YAML web_stream.port (default 9000)")
            viewer_proc = start_live_viewer(
                trace_path,
                args.live_follow_poll,
                args.live_web_host,
                args.live_web_port,
                args.live_web_fps,
                args.live_web_jpeg_quality,
                args.control_file,
                args.match_duration_sec,
                args.ros_state_file,
                unit_scene_path.as_posix() if unit_scene_path is not None else "",
                input_owner=args.input_owner,
                debug_pygame=args.live_debug_pygame,
                viewer_config=args.viewer_config,
            )
            # Give viewer a moment to start and enter follow wait state.
            time.sleep(0.5)
            viewer_rc = viewer_proc.poll()
            if viewer_rc is not None:
                print(
                    f"live viewer exited early (rc={viewer_rc}); check pygame/display environment.",
                    file=sys.stderr,
                )
                return 2
        rc = subprocess.run(start_cmd, check=False).returncode
        if rc != 0:
            return rc

        if args.play:
            if trace_path is None:
                print("trace is disabled but --play requested", file=sys.stderr)
                return 2
            if not trace_path.exists():
                print(f"trace file not found after run: {trace_path}", file=sys.stderr)
                return 2
            return run_viewer(trace_path, unit_scene_path.as_posix() if unit_scene_path is not None else "")
        return 0
    finally:
        if viewer_proc is not None and viewer_proc.poll() is None:
            viewer_proc.terminate()
            try:
                viewer_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                viewer_proc.kill()
                viewer_proc.wait(timeout=3)
        if mock_proc is not None and mock_proc.poll() is None:
            mock_proc.terminate()
            try:
                mock_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                mock_proc.kill()
                mock_proc.wait(timeout=3)
        if sequence_proc is not None and sequence_proc.poll() is None:
            sequence_proc.terminate()
            try:
                sequence_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                sequence_proc.kill()
                sequence_proc.wait(timeout=3)
        if ros_monitor_proc is not None and ros_monitor_proc.poll() is None:
            ros_monitor_proc.terminate()
            try:
                ros_monitor_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                ros_monitor_proc.kill()
                ros_monitor_proc.wait(timeout=3)


if __name__ == "__main__":
    raise SystemExit(main())
