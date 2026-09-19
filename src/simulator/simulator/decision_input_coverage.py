from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .config import resolve_path
from .offline_workflow import workflow_by_key


SCHEMA = "ly_simulator_decision_input_coverage_v1"


@dataclass(frozen=True)
class DecisionInputCoverage:
    key: str
    label: str
    status: str
    purpose: str
    formal_topics: tuple[str, ...]
    mock_inputs: tuple[str, ...]
    command_bus: tuple[str, ...] = ()
    trace_fields: tuple[str, ...] = ()
    fixtures: tuple[str, ...] = ()
    workflows: tuple[str, ...] = ()
    viewer_surfaces: tuple[str, ...] = ()
    notes: tuple[str, ...] = ()
    gaps: tuple[str, ...] = ()


COVERAGE: tuple[DecisionInputCoverage, ...] = (
    DecisionInputCoverage(
        key="match_state",
        label="Match Gate, Clock, Ammo, And Self HP",
        status="covered",
        purpose="Exercise game-start gating, match time, ammunition, self HP, and low-resource decisions.",
        formal_topics=(
            "/ly/game/is_start",
            "/ly/game/time_left",
            "/ly/friend/ammo_left",
            "/ly/game/all",
            "/ly/friend/hp",
        ),
        mock_inputs=(
            "--mock-time-left",
            "--mock-ammo",
            "--mock-self-health",
            "--match-duration-sec",
        ),
        command_bus=("start", "pause", "reset", "set_time_left", "set_self_health", "set_ammo"),
        trace_fields=(
            "referee.is_game_begin",
            "referee.time_left",
            "referee.ammo",
            "referee.self_hp",
            "events.self_low_hp",
            "events.self_low_ammo",
        ),
        fixtures=(
            "start_gate_lifecycle",
            "low_resource_recovery",
            "multi_unit_decision_context",
            "self_zero_hp_runtime_mismatch",
        ),
        workflows=("low-resource-recovery-exit", "regional-outpost-collapse"),
        viewer_surfaces=("Decision tab", "Events tab", "Runtime tab", "Inputs tab", "/status.json.simulator_inputs"),
        notes=(
            "Use positive low-health values for self-health recovery/posture rehearsal. Current /ly/game/all.selfhealth handling ignores zero.",
        ),
    ),
    DecisionInputCoverage(
        key="structure_hp",
        label="Outpost And Base HP",
        status="covered",
        purpose="Exercise enemy/friendly outpost and base resources used by regional, outpost, and recovery logic.",
        formal_topics=(
            "/ly/friend/op_hp",
            "/ly/enemy/op_hp",
            "/ly/friend/base_hp",
            "/ly/enemy/base_hp",
        ),
        mock_inputs=(
            "--mock-self-outpost-health",
            "--mock-enemy-outpost-health",
            "--mock-self-base-health",
            "--mock-enemy-base-health",
        ),
        command_bus=("set_structure_health", "set_structure_hp"),
        trace_fields=(
            "referee.self_outpost_hp",
            "referee.enemy_outpost_hp",
            "referee.self_base_hp",
            "referee.enemy_base_hp",
            "events.enemy_outpost_alive",
            "events.outpost_attack_window_open",
        ),
        fixtures=("outpost_attack", "multi_unit_decision_context", "route_churn_warning"),
        workflows=("regional-outpost-collapse", "regional-buff-timeout"),
        viewer_surfaces=("Events tab", "Inputs tab structure controls", "/status.json.simulator_inputs.structures"),
    ),
    DecisionInputCoverage(
        key="unit_hp_position",
        label="Friend/Enemy Unit HP And Position",
        status="covered_with_formal_exclusions",
        purpose=(
            "Exercise unit HP and official-field position context used for target selection, reliable enemy "
            "position evidence, and unit-info summaries."
        ),
        formal_topics=("/ly/friend/hp", "/ly/enemy/hp", "/ly/position/data"),
        mock_inputs=("--unit-scene", "--mock-enemy-health"),
        command_bus=("set_unit", "set_units", "set_unit_hp", "remove_unit", "clear_units"),
        trace_fields=(
            "units.friend",
            "units.enemy",
            "unit_info.friend",
            "unit_info.enemy",
            "target_state.hitable_targets",
            "target_state.reliable_enemy_positions",
        ),
        fixtures=("multi_unit_decision_context", "unit_zero_hp_runtime_mismatch"),
        workflows=("multi-unit-target-priority", "regional-outpost-collapse", "full-roster-visual-inputs"),
        viewer_surfaces=("Map unit sprites", "Events tab UnitInfo summaries", "Inputs tab unit controls", "simulator.unit_trace"),
        notes=(
            "Hero, Engineer, Infantry1, Infantry2, and Sentry are formal UnitInfo traceable units.",
            "Infantry3 and Drone can be placed and rendered, but current RobotLists/UnitInfo excludes them.",
            "Outpost/base zero HP is accepted by structure subscribers; current unit Health.msg subscribers ignore zero HP.",
        ),
        gaps=(
            "Drone and Infantry3 are visual/offline context only for formal UnitInfo until BT RobotLists changes.",
            "Simulator unit HP controls can express zero, but current formal unit-health subscribers ignore zero; validate unit_info before treating a unit as confirmed dead.",
        ),
    ),
    DecisionInputCoverage(
        key="self_position",
        label="Sentry Self Position",
        status="covered",
        purpose="Exercise self-position dependent route/reach decisions in official field centimeters.",
        formal_topics=("/ly/navi/position", "/ly/friend/uwb_pos", "/ly/position/data"),
        mock_inputs=(
            "--mock-self-position-x",
            "--mock-self-position-y",
            "--mock-publish-self-position",
            "--mock-publish-uwb-position",
            "--mock-uwb-position-x",
            "--mock-uwb-position-y",
        ),
        command_bus=("set_self_position",),
        trace_fields=("unit_info.friend", "goal_reach_state.position_fresh", "goal_reach_state.distance_cm"),
        fixtures=("start_gate_lifecycle", "multi_unit_decision_context"),
        workflows=(
            "regional-outpost-collapse",
            "multi-unit-target-priority",
            "low-resource-recovery-exit",
            "uwb-position-fusion",
        ),
        viewer_surfaces=("Runtime tab", "Events tab", "Map current goal/self context", "/status.json.current_record"),
        notes=(
            "/ly/navi/position uses official cm directly.",
            "/ly/friend/uwb_pos accepts official cm in simulator CLI, then publishes raw y so behavior_tree reconstructs official y.",
            "/ly/position/data raw y is inverted before publish.",
        ),
    ),
    DecisionInputCoverage(
        key="navigation_status_velocity",
        label="Navigation Reachability, Rotation, Velocity, And Lower-Head",
        status="covered",
        purpose="Exercise navigation reach/reachable/should-rotate/velocity fields that affect output gates and control velocity.",
        formal_topics=(
            "/ly/navi/reached",
            "/ly/navi/reachable",
            "/ly/navi/should_rotate",
            "/ly/navi/vel",
            "/ly/navi/lower_head",
        ),
        mock_inputs=(
            "--mock-navi-reached",
            "--mock-navi-reachable",
            "--mock-navi-should-rotate",
            "--mock-navi-vel-x",
            "--mock-navi-vel-y",
            "--mock-navi-lower-head",
        ),
        trace_fields=(
            "goal_reach_state",
            "navi_status",
            "navi_velocity",
            "gimbal.navi_lower_head",
        ),
        fixtures=("goal_pos_raw_bridge", "chase_goal_pos", "chase_goal_pos_raw_bridge", "route_churn_warning"),
        workflows=("regional-outpost-collapse", "multi-unit-target-priority"),
        viewer_surfaces=("Events tab goal reach", "Runtime tab navigation status", "Foxglove metrics", "/status.json.current_record"),
    ),
    DecisionInputCoverage(
        key="referee_event_energy",
        label="Referee EventData And Sentry Energy Info",
        status="covered",
        purpose="Exercise buff/energy activation, gain-point status, and regional event gates.",
        formal_topics=("/ly/game/event_data", "/ly/game/sentry/info"),
        mock_inputs=(
            "--mock-event-raw",
            "--mock-event-center-gain-point-status",
            "--mock-event-self-small-energy-status",
            "--mock-event-self-large-energy-status",
            "--mock-event-self-fortress-gain-point-status",
            "--mock-event-self-outpost-gain-point-status",
            "--mock-event-self-base-gain-point-status",
            "--mock-sentry-can-activate-energy",
        ),
        trace_fields=(
            "events.event_data_fresh",
            "events.sentry_info_fresh",
            "events.buff_can_activate",
            "events.buff_activating",
            "events.buff_activated",
            "referee.event_center_gain_point_status",
            "referee.event_self_*",
            "referee.sentry_can_activate_energy",
            "referee.energy_activate_confirm_pulse",
        ),
        fixtures=("buff_activation", "outpost_attack", "multi_unit_decision_context"),
        workflows=("regional-buff-timeout", "regional-outpost-collapse"),
        viewer_surfaces=("Events tab", "Runtime tab referee/resource rows", "/status.json.current_record.referee"),
    ),
    DecisionInputCoverage(
        key="team_buff",
        label="Team Buff State",
        status="covered",
        purpose="Exercise attack/defence/recovery/cooling/vulnerability/remaining-energy resource context.",
        formal_topics=("/ly/team/buff",),
        mock_inputs=(
            "--mock-team-buff-recovery",
            "--mock-team-buff-cooling",
            "--mock-team-buff-defence",
            "--mock-team-buff-vulnerability",
            "--mock-team-buff-attack",
            "--mock-team-buff-remaining-energy",
        ),
        trace_fields=("referee.team_buff",),
        fixtures=("buff_activation", "multi_unit_decision_context"),
        workflows=("regional-buff-timeout", "regional-outpost-collapse", "full-roster-visual-inputs"),
        viewer_surfaces=("Events tab", "/status.json.current_record.referee"),
    ),
    DecisionInputCoverage(
        key="rfid",
        label="RFID Match Zones",
        status="covered",
        purpose="Exercise center gain point, tunnel, side-zone, outpost, fortress, and highland RFID-derived gates.",
        formal_topics=("/ly/game/rfid",),
        mock_inputs=(
            "--mock-rfid-raw",
            "--mock-rfid-has-status-2",
            "--mock-rfid-status-2-raw",
            "--mock-rfid-center-gain-point",
            "--mock-rfid-self-base",
            "--mock-rfid-self-fortress",
            "--mock-rfid-self-outpost",
            "--mock-rfid-self-supply",
            "--mock-rfid-self-highland",
            "--mock-rfid-self-road-crossing",
            "--mock-rfid-self-central-highland-crossing",
            "--mock-rfid-self-tunnel",
            "--mock-rfid-self-assembly",
            "--mock-rfid-self-fly-ramp",
            "--mock-rfid-enemy-fortress",
            "--mock-rfid-enemy-outpost",
            "--mock-rfid-enemy-highland",
            "--mock-rfid-enemy-road-crossing",
            "--mock-rfid-enemy-central-highland-crossing",
            "--mock-rfid-enemy-tunnel",
            "--mock-rfid-enemy-assembly",
            "--mock-rfid-enemy-fly-ramp",
        ),
        trace_fields=("referee.rfid_match", "referee.rfid_status", "referee.has_rfid_status_2"),
        fixtures=("multi_unit_decision_context", "rfid_full_zone_context"),
        workflows=("regional-buff-timeout", "regional-outpost-collapse", "multi-unit-target-priority"),
        viewer_surfaces=("Events tab RFID rows", "/status.json.current_record.referee"),
    ),
    DecisionInputCoverage(
        key="official_target_fallback",
        label="Official Target Fallback",
        status="covered",
        purpose="Exercise /ly/navi/target_official fallback used when chase target position is bridged from navigation.",
        formal_topics=("/ly/navi/target_official",),
        mock_inputs=(
            "--mock-official-target-valid",
            "--mock-official-target-x",
            "--mock-official-target-y",
            "--mock-official-target-armor-type",
        ),
        trace_fields=(
            "decision_output.chase_official_target_valid",
            "decision_output.chase_official_armor_type",
            "navi_relative_target.official_target_valid",
            "navi_relative_target.official_armor_type",
            "unit_info.enemy.position_source",
        ),
        fixtures=("chase_goal_pos", "chase_goal_pos_raw_bridge"),
        workflows=("official-target-fallback", "multi-unit-target-priority"),
        viewer_surfaces=("Decision tab output metadata", "Events tab UnitInfo source", "/status.json.current_record.output"),
        notes=("ArmorType Sentry is ID 6; simulator UnitType Drone is also ID 6 but belongs to a different enum.",),
    ),
    DecisionInputCoverage(
        key="gimbal_fire_posture",
        label="Gimbal Angles, FireCode, CapV, And Posture",
        status="covered",
        purpose="Exercise posture feedback, fire-code state, cap voltage, yaw/pitch, chassis yaw velocity, and runtime posture manager evidence.",
        formal_topics=(
            "/ly/gimbal/angles",
            "/ly/gimbal/firecode",
            "/ly/gimbal/chassis",
            "/ly/gimbal/posture",
            "/ly/gimbal/capV",
        ),
        mock_inputs=(
            "--mock-yaw",
            "--mock-pitch",
            "--mock-posture",
            "--mock-gimbal-fire-status",
            "--mock-gimbal-cap-state",
            "--mock-gimbal-follow-mode",
            "--mock-gimbal-aim-mode",
            "--mock-gimbal-rotate",
            "--mock-gimbal-yaw-velocity",
            "--mock-gimbal-yaw-angle",
            "--mock-cap-v",
        ),
        command_bus=("set_posture",),
        trace_fields=(
            "posture",
            "gimbal.yaw_deg",
            "gimbal.pitch_deg",
            "gimbal.yaw_vel_deg_per_sec",
            "gimbal.yaw_angle_deg",
            "gimbal.cap_v",
            "gimbal.fire_code",
        ),
        fixtures=("low_resource_recovery", "route_churn_warning", "multi_unit_decision_context"),
        workflows=("regional-buff-timeout", "low-resource-recovery-exit"),
        viewer_surfaces=("Runtime tab gimbal/posture rows", "Decision tab aim mode", "/status.json.current_record.gimbal"),
    ),
    DecisionInputCoverage(
        key="tactical_protection",
        label="ProtectCastle, ProtectHero, And Final FollowMode",
        status="covered",
        purpose=(
            "Exercise BT-owned Tactical protection evidence from fresh enemy position and referee event "
            "sources, while keeping final FollowMode/Rotate output separate from lower-machine feedback."
        ),
        formal_topics=(
            "/ly/position/data",
            "/ly/game/event_data",
            "/ly/gimbal/firecode",
            "/ly/control/firecode",
        ),
        mock_inputs=(
            "--unit-scene",
            "--mock-event-self-fortress-gain-point-status",
            "--mock-navi-should-rotate",
        ),
        command_bus=("set_unit", "set_unit_hp", "set_self_position"),
        trace_fields=(
            "tactical.protect_castle",
            "tactical.protect_hero",
            "tactical.regional_defense",
            "control_output.fire_code",
            "gimbal_feedback.fire_code",
        ),
        fixtures=("tactical_protect_castle", "tactical_follow_rotate"),
        workflows=("tactical-protection",),
        viewer_surfaces=(
            "Tactical browser panel",
            "Pygame Control tab",
            "Foxglove decision/tactical and decision/control_output channels",
        ),
        notes=(
            "ProtectCastle source activation is written by behavior_tree trace v4; simulator clients do not infer it from scene geometry.",
            "FollowMode/Rotate acceptance must use control_output, not the asynchronous gimbal feedback snapshot.",
        ),
    ),
    DecisionInputCoverage(
        key="external_aim",
        label="Optional External Aim",
        status="optional_covered",
        purpose="Exercise sentry_msgs external aim candidates and result when the selected BT config enables ExternalAimSettings.",
        formal_topics=("/ly/aim/armor_targets", "/ly/aim/result"),
        mock_inputs=(
            "--mock-external-aim",
            "--mock-external-aim-follow",
            "--mock-external-aim-fire",
            "--mock-external-aim-yaw",
            "--mock-external-aim-pitch",
            "--mock-external-aim-target-id",
            "--mock-external-aim-target-x",
            "--mock-external-aim-target-y",
            "--mock-external-aim-target-z",
            "--mock-external-aim-frame",
        ),
        trace_fields=("target_state.external_aim_active", "target_state.hitable_targets", "navi_relative_target"),
        fixtures=(),
        workflows=("official-target-fallback",),
        viewer_surfaces=("Events tab target freshness", "Runtime tab relative target rows"),
        notes=("Requires sentry_msgs Python imports from a built/sourced workspace and a BT config that reads external aim.",),
        gaps=("No bundled launched external-aim trace fixture is included yet.",),
    ),
    DecisionInputCoverage(
        key="bullet_state",
        label="Bullet Speed And Projectile Allowance",
        status="covered",
        purpose="Publish and trace cached BulletInfo so behavior-tree subscriptions can be rehearsed without hardware.",
        formal_topics=("/ly/game/bullet",),
        mock_inputs=(
            "--mock-bullet-initial-speed",
            "--mock-bullet-has-shoot-data",
            "--mock-bullet-type",
            "--mock-bullet-shooter-number",
            "--mock-bullet-launching-frequency",
            "--mock-bullet-projectile-allowance-17mm",
            "--mock-bullet-projectile-allowance-42mm",
            "--mock-bullet-remaining-gold-coin",
            "--mock-bullet-projectile-allowance-fortress-17mm",
        ),
        trace_fields=(
            "bullet_info.has_received",
            "bullet_info.age_ms",
            "bullet_info.initial_speed",
            "bullet_info.has_shoot_data",
            "bullet_info.projectile_allowance_17mm",
            "bullet_info.remaining_gold_coin",
        ),
        fixtures=("bullet_info_resource",),
        workflows=("bullet-info-resource-snapshot",),
        viewer_surfaces=(
            "Runtime tab Bullet Info rows",
            "/status.json.current_record.bullet_info",
            "Foxglove decision frame bullet_info",
        ),
        notes=("Current BT comment says decisions still use legacy ammo/speed inputs while BulletInfo is cached.",),
    ),
)


STATUS_RANK = {
    "covered": 0,
    "covered_with_formal_exclusions": 0,
    "optional_covered": 1,
    "partial_mock_only": 2,
}


def coverage_by_key() -> dict[str, DecisionInputCoverage]:
    return {item.key: item for item in COVERAGE}


def coverage_payload() -> dict[str, Any]:
    rows = [coverage_item_payload(item) for item in COVERAGE]
    counts: dict[str, int] = {}
    for item in COVERAGE:
        counts[item.status] = counts.get(item.status, 0) + 1
    return {
        "schema": SCHEMA,
        "summary": {
            "inputs": len(COVERAGE),
            "status_counts": counts,
            "known_gaps": sum(1 for item in COVERAGE if item.gaps),
        },
        "inputs": rows,
        "issues": validate_coverage(),
    }


def coverage_item_payload(item: DecisionInputCoverage) -> dict[str, Any]:
    return {
        "key": item.key,
        "label": item.label,
        "status": item.status,
        "purpose": item.purpose,
        "formal_topics": list(item.formal_topics),
        "mock_inputs": list(item.mock_inputs),
        "command_bus": list(item.command_bus),
        "trace_fields": list(item.trace_fields),
        "fixtures": list(item.fixtures),
        "workflows": list(item.workflows),
        "viewer_surfaces": list(item.viewer_surfaces),
        "notes": list(item.notes),
        "gaps": list(item.gaps),
    }


def validate_coverage() -> list[str]:
    issues: list[str] = []
    keys = [item.key for item in COVERAGE]
    if len(keys) != len(set(keys)):
        issues.append("duplicate coverage keys")
    for item in COVERAGE:
        if item.status not in STATUS_RANK:
            issues.append(f"{item.key}: unknown status {item.status}")
        if not item.formal_topics:
            issues.append(f"{item.key}: missing formal topics")
        if not item.mock_inputs:
            issues.append(f"{item.key}: missing mock inputs")
        for fixture in item.fixtures:
            if not fixture_path(fixture).is_file():
                issues.append(f"{item.key}: missing fixture {fixture}")
        for workflow in item.workflows:
            if workflow not in workflow_by_key():
                issues.append(f"{item.key}: unknown workflow {workflow}")
    return issues


def fixture_path(name: str) -> Path:
    return resolve_path(f"src/simulator/sample/scenarios/{name}.jsonl")


def print_summary(payload: dict[str, Any]) -> int:
    print("Decision input coverage:")
    for item in payload.get("inputs", []):
        row = item if isinstance(item, dict) else {}
        print(f"  {row.get('key')}: {row.get('label')} [{row.get('status')}]")
        print(f"    topics: {', '.join(row.get('formal_topics', []))}")
        print(f"    mocks: {', '.join(row.get('mock_inputs', [])[:5])}{' ...' if len(row.get('mock_inputs', [])) > 5 else ''}")
        if row.get("gaps"):
            print(f"    gaps: {'; '.join(row.get('gaps', []))}")
    issues = payload.get("issues", [])
    if issues:
        print("Catalog issues:")
        for issue in issues:
            print(f"  - {issue}")
        return 1
    return 0


def print_detail(payload: dict[str, Any], key: str) -> int:
    rows = [row for row in payload.get("inputs", []) if isinstance(row, dict) and row.get("key") == key]
    if not rows:
        print(f"unknown decision input coverage key: {key}")
        print("available keys: " + ", ".join(sorted(coverage_by_key())))
        return 2
    row = rows[0]
    print(f"{row['key']} - {row['label']} [{row['status']}]")
    print(f"Purpose: {row['purpose']}")
    for label, field in (
        ("Formal topics", "formal_topics"),
        ("Mock inputs", "mock_inputs"),
        ("Command bus", "command_bus"),
        ("Trace fields", "trace_fields"),
        ("Fixtures", "fixtures"),
        ("Workflows", "workflows"),
        ("Viewer surfaces", "viewer_surfaces"),
        ("Notes", "notes"),
        ("Known gaps", "gaps"),
    ):
        values = row.get(field, [])
        if values:
            print(f"{label}:")
            for value in values:
                print(f"  - {value}")
    return 0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="List simulator coverage for formal behavior-tree decision inputs.")
    parser.add_argument("input", nargs="?", help="Coverage key. Omit to list all inputs.")
    parser.add_argument("--json", action="store_true", help="Print machine-readable coverage JSON.")
    parser.add_argument(
        "--fail-on-partial",
        action="store_true",
        help="Return non-zero if any input is intentionally partial or optional.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    payload = coverage_payload()
    if args.json:
        selected = str(args.input or "").strip()
        if selected:
            rows = [row for row in payload["inputs"] if row["key"] == selected]
            if not rows:
                print(json.dumps({"schema": SCHEMA, "inputs": [], "issues": [f"unknown input: {selected}"]}))
                return 2
            payload = {**payload, "inputs": rows}
        print(json.dumps(payload, ensure_ascii=True, indent=2, sort_keys=True))
    elif str(args.input or "").strip():
        code = print_detail(payload, str(args.input).strip())
        if code:
            return code
    else:
        code = print_summary(payload)
        if code:
            return code

    if payload.get("issues"):
        return 1
    if args.fail_on_partial:
        partial = [item.key for item in COVERAGE if STATUS_RANK[item.status] > 0]
        if partial:
            print("partial decision input coverage: " + ", ".join(partial))
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
