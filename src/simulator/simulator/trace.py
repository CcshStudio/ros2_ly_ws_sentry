from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

from .model import (
    BulletInfoState,
    ControlAnglesState,
    ControlFireCodeState,
    ControlOutputState,
    ControlTrajectoryState,
    DecisionIntent,
    DecisionOutput,
    EventSnapshot,
    FieldState,
    FireCodeSnapshot,
    GimbalFeedbackState,
    GimbalState,
    GoalReachState,
    NaviStatus,
    FaceModeState,
    NaviVelocity,
    PostureRuntime,
    OutpostEngagementLock,
    RefereeState,
    RelativeTarget,
    RfidMatchState,
    RuntimeGuard,
    TacticalDecisionState,
    TargetState,
    TraceRecord,
    UnitInfoRecord,
    UnitRecord,
)


LOCATION_COUNT = 50
UNIT_TYPE_NAMES = {
    1: "Hero",
    2: "Engineer",
    3: "Infantry1",
    4: "Infantry2",
    5: "Infantry3",
    6: "Drone",
    7: "Sentry",
    8: "Dart",
    9: "Radar",
}


def as_dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def as_list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def number(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return default
        out = float(value)
        return out if math.isfinite(out) else default
    except (TypeError, ValueError):
        return default


def integer(value: Any, default: int = 0) -> int:
    try:
        if value is None:
            return default
        return int(value)
    except (TypeError, ValueError):
        return default


def optional_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if value is None:
        return None
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        text = value.strip().lower()
        if text in {"true", "1", "yes", "on"}:
            return True
        if text in {"false", "0", "no", "off"}:
            return False
    return None


def boolean(value: Any, default: bool = False) -> bool:
    parsed = optional_bool(value)
    return default if parsed is None else parsed


def optional_number(value: Any) -> float | None:
    out = number(value, float("nan"))
    return out if math.isfinite(out) else None


def optional_integer(value: Any) -> int | None:
    try:
        if value is None:
            return None
        return int(value)
    except (TypeError, ValueError):
        return None


def compact_label(value: Any, default: str = "-") -> str:
    if isinstance(value, dict):
        name = value.get("name")
        ident = value.get("id")
        if name is not None and ident is not None:
            return f"{name} ({ident})"
        if name is not None:
            return str(name)
        if ident is not None:
            return str(ident)
    if value is None:
        return default
    return str(value)


def compact_label_list(value: Any) -> tuple[str, ...]:
    out: list[str] = []
    for item in as_list(value):
        out.append(compact_label(item))
    return tuple(out)


def parse_position(value: Any) -> tuple[float, float] | None:
    if isinstance(value, dict):
        x = value.get("x")
        y = value.get("y")
    elif isinstance(value, (list, tuple)) and len(value) >= 2:
        x, y = value[0], value[1]
    else:
        return None
    px = number(x, float("nan"))
    py = number(y, float("nan"))
    if not math.isfinite(px) or not math.isfinite(py):
        return None
    return (px, py)


def goal_base_id(goal_id: int) -> int:
    return goal_id - LOCATION_COUNT if goal_id >= LOCATION_COUNT else goal_id


def goal_side(goal_id: int, fallback_team: str) -> str:
    if goal_id >= LOCATION_COUNT:
        return "blue"
    if 0 <= goal_id < LOCATION_COUNT:
        return "red"
    return "blue" if fallback_team == "blue" else "red"


def normalize_unit(raw: dict[str, Any], fallback_side: str) -> UnitRecord:
    pos = parse_position(raw.get("position_cm"))
    if pos == (0, 0):
        pos = None
    return UnitRecord(
        side=str(raw.get("side", fallback_side)),
        type_name=str(raw.get("type", raw.get("name", "Unknown"))),
        type_id=integer(raw.get("type_id", raw.get("id")), 0),
        hp=integer(raw.get("hp"), 0),
        max_hp=integer(raw.get("max_hp"), 0),
        distance_m=number(raw.get("distance_m"), 0.0),
        position_cm=pos,
    )


def normalize_units(raw: dict[str, Any]) -> tuple[UnitRecord, ...]:
    units_root = as_dict(raw.get("units"))
    units: list[UnitRecord] = []
    for side in ("friend", "enemy"):
        for item in as_list(units_root.get(side)):
            if isinstance(item, dict):
                units.append(normalize_unit(item, side))
    return tuple(units)


def unit_type_name(type_id: int) -> str:
    return UNIT_TYPE_NAMES.get(type_id, f"Unit{type_id}" if type_id else "Unknown")


def normalize_unit_info_item(raw: dict[str, Any], fallback_side: str) -> UnitInfoRecord:
    car_id = integer(raw.get("car_id", raw.get("id", raw.get("type_id"))), 0)
    type_id = car_id % 100 if car_id >= 100 else car_id
    pos = parse_position(raw.get("position_cm"))
    if pos is None:
        pos = parse_position({"x": raw.get("position_x"), "y": raw.get("position_y")})
    has_position = boolean(raw.get("has_position"), pos is not None)
    if pos == (0, 0) and not has_position:
        pos = None
    return UnitInfoRecord(
        side=str(raw.get("side", fallback_side)),
        car_id=car_id,
        type_id=type_id,
        type_name=str(raw.get("type", raw.get("name", unit_type_name(type_id)))),
        hp=integer(raw.get("hp"), 0),
        has_hp=boolean(raw.get("has_hp"), raw.get("hp") is not None),
        hp_fresh=boolean(raw.get("hp_fresh")),
        position_cm=pos,
        has_position=has_position,
        position_fresh=boolean(raw.get("position_fresh")),
        position_source=str(raw.get("position_source", "")),
        area_id=integer(raw.get("area_id"), 0),
        area_name=str(raw.get("area_name", "")),
        area_used_nearest_fallback=boolean(raw.get("area_used_nearest_fallback")),
    )


def normalize_unit_info(raw: dict[str, Any]) -> tuple[UnitInfoRecord, ...]:
    info_root = as_dict(raw.get("unit_info"))
    units: list[UnitInfoRecord] = []
    for side in ("friend", "enemy"):
        for item in as_list(info_root.get(side)):
            if isinstance(item, dict):
                units.append(normalize_unit_info_item(item, side))
    return tuple(units)


def normalize_output(raw: dict[str, Any], navi: dict[str, Any], team: str, goal_names: dict[int, str]) -> DecisionOutput:
    output = as_dict(raw.get("decision_output"))
    flat_goal = None if isinstance(raw.get("navi_goal"), dict) else raw.get("navi_goal")
    goal_id = integer(output.get("goal_id", navi.get("id", flat_goal)), 0)
    base_id = integer(output.get("goal_base_id", navi.get("base_id")), goal_base_id(goal_id))
    side = str(output.get("goal_side", navi.get("side", goal_side(goal_id, team)))).lower()
    if side not in ("red", "blue"):
        side = goal_side(goal_id, team)

    pos = parse_position(output.get("goal_pos_cm"))
    if pos is None:
        pos = parse_position(navi.get("position_cm"))
    if pos == (0, 0):
        pos = None

    kind = str(output.get("kind", "goal_pos" if pos is not None else "goal_id"))
    uses_goal_pos = optional_bool(output.get("uses_goal_pos"))
    if uses_goal_pos is None:
        uses_goal_pos = pos is not None and kind != "goal_id"

    topic = str(output.get("output_topic", ""))
    if not topic and uses_goal_pos:
        topic = "/ly/navi/goal_pos"
    if not topic and kind == "goal_id":
        topic = "/ly/navi/goal"

    final_topic = str(output.get("final_goal_pos_topic", ""))
    if not final_topic and uses_goal_pos:
        if kind.endswith("bridge") or topic in {"/ly/navi/target_rel", "/ly/navi/goal_pos_raw"}:
            final_topic = "/goal_pose"
        else:
            final_topic = "/ly/navi/goal_pos"

    return DecisionOutput(
        kind=kind,
        goal_id=goal_id,
        goal_base_id=base_id,
        goal_name=str(output.get("goal_name", navi.get("name", goal_names.get(base_id, f"Goal{base_id}")))),
        goal_side=side,
        goal_pos_cm=pos,
        speed_level=integer(output.get("speed_level", navi.get("speed_level", raw.get("speed_level"))), 0),
        publish_enabled=optional_bool(output.get("publish_enabled", navi.get("publish_enabled"))),
        publish_allowed=optional_bool(output.get("publish_allowed", navi.get("publish_allowed"))),
        output_topic=topic,
        output_frame=str(output.get("output_frame", output.get("goal_pos_frame", "map"))),
        final_goal_pos_topic=final_topic,
        uses_goal_pos=uses_goal_pos,
        uses_to_navi=bool(output.get("uses_to_navi", output.get("uses_tf_goal_bridge", False))),
        relative_target_valid=bool(output.get("relative_target_valid", False)),
        chase_official_target_valid=boolean(output.get("chase_official_target_valid")),
        chase_official_armor_type=integer(output.get("chase_official_armor_type"), 0),
        source=str(output.get("source", "navi_goal")),
    )


def normalize_decision_intent(raw: dict[str, Any], output: DecisionOutput) -> DecisionIntent:
    intent = as_dict(raw.get("decision_intent"))
    return DecisionIntent(
        layer=str(intent.get("layer", "unknown")),
        reason=str(intent.get("reason", "unknown")),
        base_goal_id=integer(intent.get("base_goal_id"), output.goal_base_id),
        resolved_goal_id=integer(intent.get("resolved_goal_id"), output.goal_id),
        goal_team=str(intent.get("goal_team", output.goal_side)),
        apply_team_offset=optional_bool(intent.get("apply_team_offset")),
        priority=integer(intent.get("priority"), 0),
        detail=str(intent.get("detail", intent.get("reason", "-"))),
    )


def normalize_target_state(raw: dict[str, Any]) -> TargetState:
    state = as_dict(raw.get("target_state"))
    return TargetState(
        has_recent_target=boolean(state.get("has_recent_target")),
        external_aim_active=optional_bool(state.get("external_aim_active")),
        fresh_current_aim=optional_bool(state.get("fresh_current_aim")),
        fresh_auto_aim=boolean(state.get("fresh_auto_aim")),
        fresh_buff=boolean(state.get("fresh_buff")),
        fresh_outpost=boolean(state.get("fresh_outpost")),
        hitable_targets=compact_label_list(state.get("hitable_targets")),
        reliable_enemy_positions=compact_label_list(state.get("reliable_enemy_positions")),
    )


def normalize_events(raw: dict[str, Any]) -> EventSnapshot:
    events = as_dict(raw.get("events"))
    return EventSnapshot(
        event_data_fresh=boolean(events.get("event_data_fresh")),
        sentry_info_fresh=boolean(events.get("sentry_info_fresh")),
        buff_task_enabled=boolean(events.get("buff_task_enabled")),
        buff_can_activate=boolean(events.get("buff_can_activate")),
        buff_activating=boolean(events.get("buff_activating")),
        buff_activated=boolean(events.get("buff_activated")),
        outpost_task_enabled=boolean(events.get("outpost_task_enabled")),
        enemy_outpost_alive=boolean(events.get("enemy_outpost_alive")),
        outpost_attack_window_open=boolean(events.get("outpost_attack_window_open")),
        self_low_hp=boolean(events.get("self_low_hp")),
        self_low_ammo=boolean(events.get("self_low_ammo")),
        recent_damage_over_30=boolean(events.get("recent_damage_over_30")),
        armor_target_visible=boolean(events.get("armor_target_visible")),
        buff_target_locked=boolean(events.get("buff_target_locked")),
        outpost_target_locked=boolean(events.get("outpost_target_locked")),
        goal_reached=boolean(events.get("goal_reached")),
        goal_unreachable=boolean(events.get("goal_unreachable")),
        regional_defense_active=boolean(events.get("regional_defense_active")),
        self_fortress_gain_point_status=integer(events.get("self_fortress_gain_point_status"), 0),
        self_outpost_gain_point_status=integer(events.get("self_outpost_gain_point_status"), 0),
        self_base_gain_point_status=boolean(events.get("self_base_gain_point_status")),
    )


def normalize_relative_target(raw: dict[str, Any]) -> RelativeTarget:
    target = as_dict(raw.get("navi_relative_target"))
    return RelativeTarget(
        valid=boolean(target.get("valid")),
        frame_id=str(target.get("frame_id", target.get("frame", ""))),
        x=optional_number(target.get("x")),
        y=optional_number(target.get("y")),
        z=optional_number(target.get("z")),
        distance=optional_number(target.get("distance")),
        yaw_error_deg=optional_number(target.get("yaw_error_deg")),
        pitch_error_deg=optional_number(target.get("pitch_error_deg")),
        armor_type=integer(target.get("armor_type"), 0),
        aim_mode=integer(target.get("aim_mode"), 0),
        official_target_valid=boolean(target.get("official_target_valid")),
        official_armor_type=integer(target.get("official_armor_type"), 0),
    )


def normalize_goal_reach(raw: dict[str, Any], output: DecisionOutput) -> GoalReachState:
    state = as_dict(raw.get("goal_reach_state"))
    return GoalReachState(
        status=str(state.get("status", "unknown")),
        status_id=integer(state.get("status_id"), 0),
        reason=str(state.get("reason", "none")),
        reason_id=integer(state.get("reason_id"), 0),
        goal_id=integer(state.get("goal_id"), output.goal_id),
        base_goal_id=integer(state.get("base_goal_id"), output.goal_base_id),
        goal_age_ms=optional_integer(state.get("goal_age_ms")),
        external_reach_fresh=boolean(state.get("external_reach_fresh")),
        external_reach=boolean(state.get("external_reach")),
        external_reachable_fresh=boolean(state.get("external_reachable_fresh")),
        external_reachable=boolean(state.get("external_reachable"), True),
        position_fresh=boolean(state.get("position_fresh")),
        has_position=boolean(state.get("has_position")),
        distance_cm=optional_number(state.get("distance_cm")),
        arrive_distance_cm=integer(state.get("arrive_distance_cm"), 0),
        face_distance_cm=integer(state.get("face_distance_cm"), 0),
        distance_fallback_allowed=boolean(state.get("distance_fallback_allowed")),
        within_arrive_distance=boolean(state.get("within_arrive_distance")),
        within_face_distance=boolean(state.get("within_face_distance")),
        near_goal_confirm_pending=boolean(state.get("near_goal_confirm_pending")),
        near_goal_confirm_elapsed_ms=integer(state.get("near_goal_confirm_elapsed_ms"), 0),
        timeout=boolean(state.get("timeout")),
    )


def normalize_navi_velocity(raw: dict[str, Any]) -> NaviVelocity:
    velocity = as_dict(raw.get("navi_velocity"))
    return NaviVelocity(
        input_x=integer(velocity.get("input_x"), 0),
        input_y=integer(velocity.get("input_y"), 0),
        output_x=integer(velocity.get("output_x"), 0),
        output_y=integer(velocity.get("output_y"), 0),
        raw_to_mps=optional_number(velocity.get("raw_to_mps")),
    )


def normalize_navi_status(raw: dict[str, Any]) -> NaviStatus:
    status = as_dict(raw.get("navi_status"))
    return NaviStatus(
        should_rotate=optional_bool(status.get("should_rotate")),
        should_rotate_fresh=optional_bool(status.get("should_rotate_fresh")),
        reached=optional_bool(status.get("reached")),
        reached_fresh=optional_bool(status.get("reached_fresh")),
        reachable=optional_bool(status.get("reachable")),
        reachable_fresh=optional_bool(status.get("reachable_fresh")),
    )


def normalize_face_mode(raw: dict[str, Any]) -> FaceModeState:
    state = as_dict(raw.get("face_mode"))
    return FaceModeState(
        requested=optional_bool(state.get("requested")),
        active=optional_bool(state.get("active")),
        patrol_fallback=optional_bool(state.get("patrol_fallback")),
        suppress_fire=optional_bool(state.get("suppress_fire")),
        source=str(state.get("source", "none")),
        phase=str(state.get("phase", "idle")),
        has_angles=optional_bool(state.get("has_angles")),
        yaw=optional_number(state.get("yaw")),
        pitch=optional_number(state.get("pitch")),
    )


def normalize_gimbal(raw: dict[str, Any]) -> GimbalState:
    gimbal = as_dict(raw.get("gimbal"))
    fire_code = as_dict(gimbal.get("fire_code"))
    return GimbalState(
        yaw_deg=optional_number(gimbal.get("yaw_deg")),
        pitch_deg=optional_number(gimbal.get("pitch_deg")),
        yaw_vel_deg_per_sec=optional_number(gimbal.get("yaw_vel_deg_per_sec")),
        yaw_angle_deg=optional_number(gimbal.get("yaw_angle_deg")),
        cap_v=integer(gimbal.get("cap_v"), 0),
        navi_lower_head=integer(gimbal.get("navi_lower_head"), 0),
        fire_status=integer(fire_code.get("fire_status"), 0),
        cap_state=integer(fire_code.get("cap_state"), 0),
        follow_mode=integer(fire_code.get("follow_mode"), 0),
        aim_mode=integer(fire_code.get("aim_mode"), 0),
        rotate=integer(fire_code.get("rotate"), 0),
    )


def empty_fire_code_snapshot() -> FireCodeSnapshot:
    return FireCodeSnapshot(
        field_mask=None,
        raw=None,
        fire_status=None,
        cap_state=None,
        follow_mode=None,
        aim_mode=None,
        rotate=None,
    )


def normalize_fire_code_snapshot(raw: dict[str, Any], available: bool) -> FireCodeSnapshot:
    if not available:
        return empty_fire_code_snapshot()
    return FireCodeSnapshot(
        field_mask=optional_integer(raw.get("field_mask")),
        raw=optional_integer(raw.get("raw")),
        fire_status=optional_integer(raw.get("fire_status")),
        cap_state=optional_integer(raw.get("cap_state")),
        follow_mode=optional_bool(raw.get("follow_mode")),
        aim_mode=optional_bool(raw.get("aim_mode")),
        rotate=optional_integer(raw.get("rotate")),
    )


def normalize_gimbal_feedback(raw: dict[str, Any]) -> GimbalFeedbackState:
    feedback = as_dict(raw.get("gimbal_feedback"))
    available = boolean(feedback.get("available"))
    return GimbalFeedbackState(
        available=available,
        age_ms=optional_integer(feedback.get("age_ms")) if available else None,
        fire_code=normalize_fire_code_snapshot(as_dict(feedback.get("fire_code")), available),
    )


def empty_control_output() -> ControlOutputState:
    return ControlOutputState(
        available=False,
        sequence=None,
        age_ms=None,
        source="not_recorded",
        angles=ControlAnglesState(published=False, yaw=None, pitch=None),
        fire_code=ControlFireCodeState(
            published=False,
            field_mask=None,
            raw=None,
            fire_status=None,
            cap_state=None,
            follow_mode=None,
            aim_mode=None,
            rotate=None,
        ),
        trajectory=ControlTrajectoryState(
            published=False,
            available=False,
            unavailable_reason="not_recorded",
            yaw=None,
            pitch=None,
            yaw_omega=None,
            pitch_omega=None,
            yaw_alpha=None,
            pitch_alpha=None,
        ),
    )


def normalize_control_output(raw: dict[str, Any]) -> ControlOutputState:
    output = as_dict(raw.get("control_output"))
    if not boolean(output.get("available")):
        return empty_control_output()

    angles_raw = as_dict(output.get("angles"))
    angles_published = boolean(angles_raw.get("published"))
    fire_raw = as_dict(output.get("fire_code"))
    fire_published = boolean(fire_raw.get("published"))
    trajectory_raw = as_dict(output.get("trajectory"))
    trajectory_published = boolean(trajectory_raw.get("published"))
    trajectory_available = boolean(trajectory_raw.get("available"))

    return ControlOutputState(
        available=True,
        sequence=optional_integer(output.get("sequence")),
        age_ms=optional_integer(output.get("age_ms")),
        source=str(output.get("source", "unknown")),
        angles=ControlAnglesState(
            published=angles_published,
            yaw=optional_number(angles_raw.get("yaw")) if angles_published else None,
            pitch=optional_number(angles_raw.get("pitch")) if angles_published else None,
        ),
        fire_code=ControlFireCodeState(
            published=fire_published,
            field_mask=optional_integer(fire_raw.get("field_mask")) if fire_published else None,
            raw=optional_integer(fire_raw.get("raw")) if fire_published else None,
            fire_status=optional_integer(fire_raw.get("fire_status")) if fire_published else None,
            cap_state=optional_integer(fire_raw.get("cap_state")) if fire_published else None,
            follow_mode=optional_bool(fire_raw.get("follow_mode")) if fire_published else None,
            aim_mode=optional_bool(fire_raw.get("aim_mode")) if fire_published else None,
            rotate=optional_integer(fire_raw.get("rotate")) if fire_published else None,
        ),
        trajectory=ControlTrajectoryState(
            published=trajectory_published,
            available=trajectory_available,
            unavailable_reason=str(trajectory_raw.get("unavailable_reason", "")),
            yaw=optional_number(trajectory_raw.get("yaw")) if trajectory_available else None,
            pitch=optional_number(trajectory_raw.get("pitch")) if trajectory_available else None,
            yaw_omega=optional_number(trajectory_raw.get("yaw_omega")) if trajectory_available else None,
            pitch_omega=optional_number(trajectory_raw.get("pitch_omega")) if trajectory_available else None,
            yaw_alpha=optional_number(trajectory_raw.get("yaw_alpha")) if trajectory_available else None,
            pitch_alpha=optional_number(trajectory_raw.get("pitch_alpha")) if trajectory_available else None,
        ),
    )


def empty_tactical_state() -> TacticalDecisionState:
    return TacticalDecisionState(
        available=False,
        protect_castle_enabled=None,
        protect_castle_rfid_enabled=None,
        protect_castle_stay_when_rfid_enabled=None,
        protect_castle_stay_when_rfid_active=None,
        protect_castle_enemy_pos_enabled=None,
        protect_castle_rfid_event_raw_active=None,
        protect_castle_rfid_event_active=None,
        protect_castle_enemy_pos_active=None,
        protect_outpost_enabled=None,
        protect_outpost_hp=None,
        protect_outpost_hp_fresh=None,
        protect_outpost_destroyed=None,
        protect_outpost_phase="not_recorded",
        protect_outpost_event_generation=None,
        protect_outpost_damage_window_ms=None,
        protect_outpost_damage_threshold_hp=None,
        protect_outpost_search_hold_sec=None,
        protect_outpost_priority=None,
        protect_hero_enabled=None,
        protect_hero_proactive_hold_when_hero_in_highland=None,
        protect_hero_active=None,
        protect_hero_priority=None,
        protect_hero_profile_ready=None,
        protect_hero_elapsed_ready=None,
        protect_hero_position_fresh=None,
        protect_hero_position_valid=None,
        protect_hero_x_cm=None,
        protect_hero_y_cm=None,
        protect_hero_in_highland=None,
        protect_hero_in_protect_area=None,
        protect_hero_health_fresh=None,
        protect_hero_known_dead=None,
        protect_hero_own_base_enemy_count=None,
        protect_hero_own_highland_enemy_count=None,
        protect_hero_threat_ready=None,
        protect_hero_mode_gate_ready=None,
        protect_hero_start_elapsed_sec=None,
        protect_hero_hold_sec=None,
        protect_hero_no_enemy_release_sec=None,
        protect_hero_position_fresh_ms=None,
        protect_hero_health_fresh_ms=None,
        protect_hero_goal_base_id=None,
        regional_defense_threat_active=None,
        regional_defense_search_kind="not_recorded",
        regional_defense_fortress_enemy_count=None,
        regional_defense_own_base_enemy_count=None,
    )


def normalize_tactical(raw: dict[str, Any]) -> TacticalDecisionState:
    tactical = as_dict(raw.get("tactical"))
    if not boolean(tactical.get("available")):
        return empty_tactical_state()

    protect_castle = as_dict(tactical.get("protect_castle"))
    protect_outpost = as_dict(tactical.get("protect_outpost"))
    protect_hero = as_dict(tactical.get("protect_hero"))
    regional_defense = as_dict(tactical.get("regional_defense"))
    return TacticalDecisionState(
        available=True,
        protect_castle_enabled=optional_bool(protect_castle.get("enabled")),
        protect_castle_rfid_enabled=optional_bool(protect_castle.get("rfid_enabled")),
        protect_castle_stay_when_rfid_enabled=optional_bool(protect_castle.get("stay_when_rfid_enabled")),
        protect_castle_stay_when_rfid_active=optional_bool(protect_castle.get("stay_when_rfid_active")),
        protect_castle_enemy_pos_enabled=optional_bool(protect_castle.get("enemy_pos_enabled")),
        protect_castle_rfid_event_raw_active=optional_bool(protect_castle.get("rfid_event_raw_active")),
        protect_castle_rfid_event_active=optional_bool(protect_castle.get("rfid_event_active")),
        protect_castle_enemy_pos_active=optional_bool(protect_castle.get("enemy_pos_active")),
        protect_outpost_enabled=optional_bool(protect_outpost.get("enabled")),
        protect_outpost_hp=optional_integer(protect_outpost.get("hp")),
        protect_outpost_hp_fresh=optional_bool(protect_outpost.get("hp_fresh")),
        protect_outpost_destroyed=optional_bool(protect_outpost.get("destroyed")),
        protect_outpost_phase=str(protect_outpost.get("phase", "unknown")),
        protect_outpost_event_generation=optional_integer(protect_outpost.get("event_generation")),
        protect_outpost_damage_window_ms=optional_integer(protect_outpost.get("damage_window_ms")),
        protect_outpost_damage_threshold_hp=optional_integer(protect_outpost.get("damage_threshold_hp")),
        protect_outpost_search_hold_sec=optional_integer(protect_outpost.get("search_hold_sec")),
        protect_outpost_priority=optional_integer(protect_outpost.get("priority")),
        protect_hero_enabled=optional_bool(protect_hero.get("enabled")),
        protect_hero_proactive_hold_when_hero_in_highland=optional_bool(
            protect_hero.get("proactive_hold_when_hero_in_highland")),
        protect_hero_active=optional_bool(protect_hero.get("active")),
        protect_hero_priority=optional_integer(protect_hero.get("priority")),
        protect_hero_profile_ready=optional_bool(protect_hero.get("profile_ready")),
        protect_hero_elapsed_ready=optional_bool(protect_hero.get("elapsed_ready")),
        protect_hero_position_fresh=optional_bool(protect_hero.get("position_fresh")),
        protect_hero_position_valid=optional_bool(protect_hero.get("position_valid")),
        protect_hero_x_cm=optional_integer(protect_hero.get("hero_x_cm")),
        protect_hero_y_cm=optional_integer(protect_hero.get("hero_y_cm")),
        protect_hero_in_highland=optional_bool(protect_hero.get("in_highland")),
        protect_hero_in_protect_area=optional_bool(protect_hero.get("in_protect_area")),
        protect_hero_health_fresh=optional_bool(protect_hero.get("health_fresh")),
        protect_hero_known_dead=optional_bool(protect_hero.get("known_dead")),
        protect_hero_own_base_enemy_count=optional_integer(protect_hero.get("own_base_enemy_count")),
        protect_hero_own_highland_enemy_count=optional_integer(protect_hero.get("own_highland_enemy_count")),
        protect_hero_threat_ready=optional_bool(protect_hero.get("threat_ready")),
        protect_hero_mode_gate_ready=optional_bool(protect_hero.get("mode_gate_ready")),
        protect_hero_start_elapsed_sec=optional_integer(protect_hero.get("start_elapsed_sec")),
        protect_hero_hold_sec=optional_integer(protect_hero.get("hold_sec")),
        protect_hero_no_enemy_release_sec=optional_integer(protect_hero.get("no_enemy_release_sec")),
        protect_hero_position_fresh_ms=optional_integer(protect_hero.get("position_fresh_ms")),
        protect_hero_health_fresh_ms=optional_integer(protect_hero.get("health_fresh_ms")),
        protect_hero_goal_base_id=optional_integer(protect_hero.get("goal_base_id")),
        regional_defense_threat_active=optional_bool(regional_defense.get("threat_active")),
        regional_defense_search_kind=str(regional_defense.get("search_kind", "unknown")),
        regional_defense_fortress_enemy_count=optional_integer(regional_defense.get("fortress_enemy_count")),
        regional_defense_own_base_enemy_count=optional_integer(regional_defense.get("own_base_enemy_count")),
    )


def normalize_bullet_info(raw: dict[str, Any]) -> BulletInfoState:
    info = as_dict(raw.get("bullet_info"))
    has_initial_speed = boolean(info.get("has_initial_speed"))
    return BulletInfoState(
        has_received=boolean(info.get("has_received")),
        age_ms=optional_integer(info.get("age_ms")),
        has_initial_speed=has_initial_speed,
        initial_speed=optional_number(info.get("initial_speed")) if has_initial_speed else None,
        has_shoot_data=boolean(info.get("has_shoot_data")),
        bullet_type=integer(info.get("bullet_type"), 0),
        shooter_number=integer(info.get("shooter_number"), 0),
        launching_frequency=integer(info.get("launching_frequency"), 0),
        has_projectile_allowance=boolean(info.get("has_projectile_allowance")),
        projectile_allowance_17mm=integer(info.get("projectile_allowance_17mm"), 0),
        projectile_allowance_42mm=integer(info.get("projectile_allowance_42mm"), 0),
        remaining_gold_coin=integer(info.get("remaining_gold_coin"), 0),
        projectile_allowance_fortress_17mm=integer(info.get("projectile_allowance_fortress_17mm"), 0),
    )


def normalize_runtime_guard(raw: dict[str, Any]) -> RuntimeGuard:
    guard = as_dict(raw.get("runtime_guard"))
    return RuntimeGuard(
        fault=str(guard.get("fault", "-")),
        recovery_requested=boolean(guard.get("recovery_requested")),
        recovering=boolean(guard.get("recovering")),
    )


def normalize_field(raw: dict[str, Any]) -> FieldState:
    field = as_dict(raw.get("field_cm"))
    return FieldState(
        width=integer(field.get("width"), 0),
        height=integer(field.get("height"), 0),
        frame=str(field.get("frame", "left_bottom_origin_cm")),
    )


def normalize_referee(raw: dict[str, Any]) -> RefereeState:
    referee = as_dict(raw.get("referee"))
    team_buff = as_dict(referee.get("team_buff"))
    rfid_match = normalize_rfid_match(referee)
    self_hp = integer(referee.get("self_hp", raw.get("self_hp")), 0)
    return RefereeState(
        self_hp=self_hp,
        self_max_hp=max(self_hp, integer(referee.get("self_max_hp"), 400)),
        self_outpost_hp=optional_integer(referee.get("self_outpost_hp")),
        enemy_outpost_hp=optional_integer(referee.get("enemy_outpost_hp")),
        self_base_hp=optional_integer(referee.get("self_base_hp")),
        enemy_base_hp=optional_integer(referee.get("enemy_base_hp")),
        ammo=integer(referee.get("ammo", raw.get("ammo")), 0),
        time_left=integer(referee.get("time_left", raw.get("time_left")), 0),
        sentry_can_activate_energy=boolean(referee.get("sentry_can_activate_energy")),
        energy_activate_confirm_pulse=boolean(referee.get("energy_activate_confirm_pulse")),
        event_center_gain_point_status=optional_integer(referee.get("event_center_gain_point_status")),
        event_self_fortress_gain_point_status=optional_integer(referee.get("event_self_fortress_gain_point_status")),
        event_self_outpost_gain_point_status=optional_integer(referee.get("event_self_outpost_gain_point_status")),
        event_self_base_gain_point_status=optional_bool(referee.get("event_self_base_gain_point_status")),
        team_buff_attack=optional_integer(team_buff.get("attack")),
        team_buff_defence=optional_integer(team_buff.get("defence")),
        team_buff_remaining_energy=optional_integer(team_buff.get("remaining_energy")),
        rfid_status=optional_integer(referee.get("rfid_status", rfid_match.raw)),
        has_rfid_status_2=optional_bool(referee.get("has_rfid_status_2", rfid_match.has_rfid_status_2)),
        rfid_status_2=optional_integer(referee.get("rfid_status_2", rfid_match.rfid_status_2_raw)),
        rfid_match=rfid_match,
    )


def normalize_rfid_match(referee: dict[str, Any]) -> RfidMatchState:
    rfid = as_dict(referee.get("rfid_match"))
    raw = optional_integer(rfid.get("raw", referee.get("rfid_status")))
    has_status_2 = optional_bool(rfid.get("has_rfid_status_2", referee.get("has_rfid_status_2")))
    status_2 = optional_integer(rfid.get("rfid_status_2_raw", referee.get("rfid_status_2")))
    return RfidMatchState(
        fresh=optional_bool(rfid.get("fresh")),
        any=optional_bool(rfid.get("any")),
        raw=raw,
        has_rfid_status_2=has_status_2,
        rfid_status_2_raw=status_2,
        self_base_gain_point=optional_bool(rfid.get("self_base_gain_point")),
        self_supply=optional_bool(rfid.get("self_supply")),
        self_non_resource_supply=optional_bool(rfid.get("self_non_resource_supply")),
        self_resource_supply=optional_bool(rfid.get("self_resource_supply")),
        self_highland_gain_point=optional_bool(rfid.get("self_highland_gain_point")),
        enemy_highland_gain_point=optional_bool(rfid.get("enemy_highland_gain_point")),
        self_road_crossing=optional_bool(rfid.get("self_road_crossing")),
        enemy_road_crossing=optional_bool(rfid.get("enemy_road_crossing")),
        self_central_highland_crossing=optional_bool(rfid.get("self_central_highland_crossing")),
        enemy_central_highland_crossing=optional_bool(rfid.get("enemy_central_highland_crossing")),
        self_tunnel=optional_bool(rfid.get("self_tunnel")),
        enemy_tunnel=optional_bool(rfid.get("enemy_tunnel")),
        tunnel=optional_bool(rfid.get("tunnel")),
        center_gain_point=optional_bool(rfid.get("center_gain_point")),
        self_fortress_gain_point=optional_bool(rfid.get("self_fortress_gain_point")),
        enemy_fortress_gain_point=optional_bool(rfid.get("enemy_fortress_gain_point")),
        self_outpost_gain_point=optional_bool(rfid.get("self_outpost_gain_point")),
        enemy_outpost_gain_point=optional_bool(rfid.get("enemy_outpost_gain_point")),
        self_assembly_gain_point=optional_bool(rfid.get("self_assembly_gain_point")),
        enemy_assembly_gain_point=optional_bool(rfid.get("enemy_assembly_gain_point")),
        self_fly_ramp=optional_bool(rfid.get("self_fly_ramp")),
        enemy_fly_ramp=optional_bool(rfid.get("enemy_fly_ramp")),
        on_self_side=optional_bool(rfid.get("on_self_side")),
        on_enemy_side=optional_bool(rfid.get("on_enemy_side")),
    )


def normalize_posture_runtime(posture: dict[str, Any]) -> PostureRuntime:
    runtime = as_dict(posture.get("runtime"))
    degraded = as_dict(runtime.get("degraded"))
    return PostureRuntime(
        has_pending=boolean(runtime.get("has_pending")),
        feedback_stale=boolean(runtime.get("feedback_stale")),
        retry_count=optional_integer(runtime.get("retry_count")),
        referee_timer_fresh=boolean(runtime.get("referee_timer_fresh")),
        referee_enhanced_posture=boolean(runtime.get("referee_enhanced_posture")),
        using_referee_timer=boolean(runtime.get("using_referee_timer")),
        degraded_attack=boolean(degraded.get("attack")),
        degraded_defense=boolean(degraded.get("defense")),
        degraded_move=boolean(degraded.get("move")),
    )


def normalize_outpost_engagement_lock(raw: dict[str, Any]) -> OutpostEngagementLock:
    lock = as_dict(raw.get("outpost_engagement_lock"))
    return OutpostEngagementLock(
        active=boolean(lock.get("active")),
        hold_target=boolean(lock.get("hold_target")),
        enhanced_armed=boolean(lock.get("enhanced_armed")),
        enhanced_pending=boolean(lock.get("enhanced_pending")),
        enhanced_active=boolean(lock.get("enhanced_active")),
        enhanced_unavailable=boolean(lock.get("enhanced_unavailable")),
        exit_reason=str(lock.get("exit_reason", "none")),
        normal_exit_hp=integer(lock.get("normal_exit_hp"), 0),
        enhanced_exit_hp=integer(lock.get("enhanced_exit_hp"), 0),
    )


def normalize_record(raw: dict[str, Any], index: int, goal_names: dict[int, str]) -> TraceRecord:
    navi = as_dict(raw.get("navi_goal"))
    posture = as_dict(raw.get("posture"))
    posture_runtime = as_dict(posture.get("runtime"))
    target = as_dict(raw.get("target_armor"))

    team = str(raw.get("team", "red")).lower()
    if team not in ("red", "blue"):
        team = "red"

    output = normalize_output(raw, navi, team, goal_names)
    decision_intent = normalize_decision_intent(raw, output)
    target_state = normalize_target_state(raw)
    events = normalize_events(raw)
    navi_relative_target = normalize_relative_target(raw)
    goal_reach = normalize_goal_reach(raw, output)
    navi_velocity = normalize_navi_velocity(raw)
    navi_status = normalize_navi_status(raw)
    face_mode = normalize_face_mode(raw)
    gimbal = normalize_gimbal(raw)
    gimbal_feedback = normalize_gimbal_feedback(raw)
    control_output = normalize_control_output(raw)
    tactical = normalize_tactical(raw)
    bullet_info = normalize_bullet_info(raw)
    runtime_guard = normalize_runtime_guard(raw)
    field = normalize_field(raw)
    referee = normalize_referee(raw)
    posture_runtime_state = normalize_posture_runtime(posture)
    outpost_engagement_lock = normalize_outpost_engagement_lock(raw)

    return TraceRecord(
        raw=raw,
        index=index,
        schema=str(raw.get("schema", "ly_decision_trace_v1")),
        schema_version=integer(raw.get("schema_version"), 1),
        has_decision_output=bool(as_dict(raw.get("decision_output"))),
        has_decision_intent=bool(as_dict(raw.get("decision_intent"))),
        t=number(raw.get("t", raw.get("elapsed_sec")), float(index)),
        event=str(raw.get("event", "tick")),
        tick=integer(raw.get("tick"), index),
        field=field,
        competition_profile=str(raw.get("competition_profile", "-")),
        team=team,
        strategy=str(raw.get("strategy_mode", "-")),
        aim=str(raw.get("aim_mode", "-")),
        target=compact_label(target, str(raw.get("target_armor", "-"))),
        target_state=target_state,
        output=output,
        decision_intent=decision_intent,
        events=events,
        navi_relative_target=navi_relative_target,
        goal_reach=goal_reach,
        navi_velocity=navi_velocity,
        navi_status=navi_status,
        face_mode=face_mode,
        goal_id=output.goal_id,
        goal_base_id=output.goal_base_id,
        goal_name=output.goal_name,
        goal_side=output.goal_side,
        goal_position=output.goal_pos_cm,
        speed_level=output.speed_level,
        posture_command=compact_label(posture.get("command", raw.get("posture_cmd"))),
        posture_state=compact_label(posture.get("state", raw.get("posture_state"))),
        posture_current=compact_label(posture_runtime.get("current")),
        posture_desired=compact_label(posture_runtime.get("desired")),
        posture_pending=compact_label(posture_runtime.get("pending")),
        posture_reason=str(posture.get("last_reason", "-")),
        posture_runtime=posture_runtime_state,
        outpost_engagement_lock=outpost_engagement_lock,
        hp=referee.self_hp,
        ammo=referee.ammo,
        time_left=referee.time_left,
        referee=referee,
        units=normalize_units(raw),
        unit_info=normalize_unit_info(raw),
        gimbal=gimbal,
        gimbal_feedback=gimbal_feedback,
        control_output=control_output,
        tactical=tactical,
        bullet_info=bullet_info,
        runtime_guard=runtime_guard,
    )


def load_trace(path: Path, goal_names: dict[int, str]) -> tuple[list[TraceRecord], int]:
    records: list[TraceRecord] = []
    bad_lines = 0
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            text = line.strip()
            if not text or text.startswith("#"):
                continue
            try:
                item = json.loads(text)
            except json.JSONDecodeError:
                bad_lines += 1
                continue
            if not isinstance(item, dict):
                bad_lines += 1
                continue
            records.append(normalize_record(item, len(records), goal_names))
    if not records:
        raise ValueError(f"no trace records loaded from {path}")
    return records, bad_lines


def load_trace_incremental(
    path: Path,
    goal_names: dict[int, str],
    start_offset: int,
    start_index: int,
) -> tuple[list[TraceRecord], int, int]:
    records: list[TraceRecord] = []
    bad_lines = 0
    with path.open("r", encoding="utf-8") as stream:
        stream.seek(max(0, start_offset))
        while True:
            line_start = stream.tell()
            line = stream.readline()
            if not line:
                break
            # Writer may still be appending the current line; keep it for next poll.
            if not line.endswith("\n"):
                stream.seek(line_start)
                break
            text = line.strip()
            if not text or text.startswith("#"):
                continue
            try:
                item = json.loads(text)
            except json.JSONDecodeError:
                bad_lines += 1
                continue
            if not isinstance(item, dict):
                bad_lines += 1
                continue
            records.append(normalize_record(item, start_index + len(records), goal_names))
        new_offset = stream.tell()
    return records, bad_lines, new_offset


def build_changes(records: list[TraceRecord]) -> list[dict[str, Any]]:
    changes: list[dict[str, Any]] = []
    last: dict[str, Any] | None = None
    for record in records:
        state = {
            "strategy": record.strategy,
            "aim": record.aim,
            "route": record.output.route_key,
            "kind": record.output.kind,
            "target": record.target,
            "posture": (record.posture_command, record.posture_state),
            "intent": (
                record.decision_intent.layer,
                record.decision_intent.reason,
                record.decision_intent.base_goal_id,
                record.decision_intent.resolved_goal_id,
            ),
            "events": record.events.change_key(),
            "reach": (
                record.goal_reach.status,
                record.goal_reach.reason,
                record.goal_reach.timeout,
            ),
            "navi_status": (
                record.navi_status.should_rotate,
                record.navi_status.reached,
                record.navi_status.reachable,
            ),
            "guard": record.runtime_guard.fault,
        }
        if last is None or state != last or record.event != "tick":
            parts = []
            if record.event != "tick":
                parts.append(record.event)
            if last is None or state["strategy"] != last["strategy"]:
                parts.append(f"strategy={record.strategy}")
            if last is None or state["aim"] != last["aim"]:
                parts.append(f"aim={record.aim}")
            if last is None or state["route"] != last["route"] or state["kind"] != last["kind"]:
                parts.append(f"output={record.output.kind}:{record.output.goal_name}:{record.output.goal_id}")
            if last is None or state["target"] != last["target"]:
                parts.append(f"target={record.target}")
            if last is None or state["intent"] != last["intent"]:
                parts.append(f"intent={record.decision_intent.layer}/{record.decision_intent.reason}")
            if last is None or state["events"] != last["events"]:
                parts.append(f"events={record.events.compact_text()}")
            if last is None or state["reach"] != last["reach"]:
                parts.append(f"reach={record.goal_reach.compact_text()}")
            if last is None or state["navi_status"] != last["navi_status"]:
                parts.append(f"navi={record.navi_status.compact_text()}")
            if last is None or state["posture"] != last["posture"]:
                parts.append(f"posture={record.posture_command}/{record.posture_state}")
            if last is None or state["guard"] != last["guard"]:
                parts.append(f"guard={record.runtime_guard.fault}")
            changes.append({"index": record.index, "t": record.t, "text": ", ".join(parts)})
            last = state
    return changes
