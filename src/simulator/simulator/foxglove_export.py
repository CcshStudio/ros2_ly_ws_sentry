from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

from .config import goals_by_id, load_config, load_plugin_points, resolve_path
from .model import TraceRecord
from .trace import load_trace


DECISION_SCHEMA_NAME = "ly_simulator.DecisionFrame"
METRICS_SCHEMA_NAME = "ly_simulator.DecisionMetrics"
POINT2_SCHEMA_NAME = "foxglove.Point2"
CONTROL_OUTPUT_SCHEMA_NAME = "ly_simulator.ControlOutput"
TACTICAL_SCHEMA_NAME = "ly_simulator.TacticalEvidence"
SCENE_SCHEMA_NAME = "ly_simulator.SceneObservation"


def decision_frame_schema() -> dict[str, Any]:
    return {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "type": "object",
        "additionalProperties": False,
        "properties": {
            "schema": {"type": "string"},
            "schema_version": {"type": "integer"},
            "source_schema": {"type": "string"},
            "source_schema_version": {"type": "integer"},
            "tick": {"type": "integer"},
            "t": {"type": "number"},
            "event": {"type": "string"},
            "competition_profile": {"type": "string"},
            "team": {"type": "string"},
            "strategy": {"type": "string"},
            "aim": {"type": "string"},
            "target": {"type": "string"},
            "output": {"type": "object"},
            "intent": {"type": "object"},
            "events": {"type": "object"},
            "goal_reach": {"type": "object"},
            "navi_status": {"type": "object"},
            "navi_velocity": {"type": "object"},
            "navi_relative_target": {"type": "object"},
            "target_state": {"type": "object"},
            "posture": {"type": "object"},
            "referee": {"type": "object"},
            "gimbal": {"type": "object"},
            "bullet_info": {"type": "object"},
            "runtime_guard": {"type": "object"},
            "units": {"type": "array", "items": {"type": "object"}},
            "unit_info": {"type": "array", "items": {"type": "object"}},
        },
        "required": [
            "schema",
            "schema_version",
            "source_schema",
            "source_schema_version",
            "tick",
            "t",
            "event",
            "team",
            "strategy",
            "aim",
            "output",
            "intent",
            "events",
            "goal_reach",
            "navi_status",
            "navi_velocity",
            "navi_relative_target",
            "referee",
            "gimbal",
            "runtime_guard",
            "units",
        ],
    }


def metrics_schema() -> dict[str, Any]:
    return {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "type": "object",
        "additionalProperties": False,
        "properties": {
            "tick": {"type": "integer"},
            "t": {"type": "number"},
            "goal_id": {"type": "integer"},
            "goal_base_id": {"type": "integer"},
            "speed_level": {"type": "integer"},
            "hp": {"type": "integer"},
            "ammo": {"type": "integer"},
            "time_left": {"type": "integer"},
            "target_visible": {"type": "integer"},
            "buff_locked": {"type": "integer"},
            "outpost_locked": {"type": "integer"},
            "goal_reached": {"type": "integer"},
            "goal_unreachable": {"type": "integer"},
            "goal_reach_status_id": {"type": "integer"},
            "goal_reach_reason_id": {"type": "integer"},
            "goal_distance_cm": {"type": ["number", "null"]},
            "should_rotate": {"type": ["integer", "null"]},
            "navi_reached": {"type": ["integer", "null"]},
            "navi_reachable": {"type": ["integer", "null"]},
            "navi_velocity_output_x_raw": {"type": "integer"},
            "navi_velocity_output_y_raw": {"type": "integer"},
            "navi_velocity_output_x_mps": {"type": ["number", "null"]},
            "navi_velocity_output_y_mps": {"type": ["number", "null"]},
            "yaw_deg": {"type": ["number", "null"]},
            "pitch_deg": {"type": ["number", "null"]},
            "cap_v": {"type": "integer"},
            "fire_status": {"type": "integer"},
            "follow_mode": {"type": "integer"},
        },
        "required": ["tick", "t", "goal_id", "speed_level", "hp", "ammo", "time_left"],
    }


def point2_schema() -> dict[str, Any]:
    return {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "type": "object",
        "additionalProperties": False,
        "properties": {"x": {"type": "number"}, "y": {"type": "number"}},
        "required": ["x", "y"],
    }


def control_output_schema() -> dict[str, Any]:
    return {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "type": "object",
        "additionalProperties": False,
        "properties": {
            "schema": {"type": "string"},
            "schema_version": {"type": "integer"},
            "source_schema": {"type": "string"},
            "source_schema_version": {"type": "integer"},
            "tick": {"type": "integer"},
            "t": {"type": "number"},
            "available": {"type": "boolean"},
            "sequence": {"type": ["integer", "null"]},
            "age_ms": {"type": ["integer", "null"]},
            "source": {"type": "string"},
            "angles": {"type": "object"},
            "fire_code": {"type": "object"},
            "trajectory": {"type": "object"},
        },
        "required": [
            "schema",
            "schema_version",
            "source_schema",
            "source_schema_version",
            "tick",
            "t",
            "available",
            "sequence",
            "age_ms",
            "source",
            "angles",
            "fire_code",
            "trajectory",
        ],
    }


def tactical_schema() -> dict[str, Any]:
    return {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "type": "object",
        "additionalProperties": False,
        "properties": {
            "schema": {"type": "string"},
            "schema_version": {"type": "integer"},
            "source_schema": {"type": "string"},
            "source_schema_version": {"type": "integer"},
            "tick": {"type": "integer"},
            "t": {"type": "number"},
            "available": {"type": "boolean"},
            "protect_castle": {"type": "object"},
            "protect_outpost": {"type": "object"},
            "protect_hero": {"type": "object"},
            "regional_defense": {"type": "object"},
        },
        "required": [
            "schema",
            "schema_version",
            "source_schema",
            "source_schema_version",
            "tick",
            "t",
            "available",
            "protect_castle",
            "protect_outpost",
            "protect_hero",
            "regional_defense",
        ],
    }


def scene_observation_schema() -> dict[str, Any]:
    return {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "type": "object",
        "additionalProperties": False,
        "properties": {
            "schema": {"type": "string"},
            "schema_version": {"type": "integer"},
            "source_schema": {"type": "string"},
            "source_schema_version": {"type": "integer"},
            "tick": {"type": "integer"},
            "t": {"type": "number"},
            "field": {"type": "object"},
            "team": {"type": "string"},
            "units": {"type": "array", "items": {"type": "object"}},
            "structures": {"type": "array", "items": {"type": "object"}},
        },
        "required": [
            "schema",
            "schema_version",
            "source_schema",
            "source_schema_version",
            "tick",
            "t",
            "field",
            "team",
            "units",
            "structures",
        ],
    }


def record_to_decision_frame(record: TraceRecord) -> dict[str, Any]:
    output = record.output
    intent = record.decision_intent
    events = record.events
    goal_reach = record.goal_reach
    navi_status = record.navi_status
    navi_velocity = record.navi_velocity
    relative_target = record.navi_relative_target
    target = record.target_state
    posture_runtime = record.posture_runtime
    referee = record.referee
    gimbal = record.gimbal
    bullet = record.bullet_info
    guard = record.runtime_guard
    return {
        "schema": "ly_simulator_decision_frame_v1",
        "schema_version": 1,
        "source_schema": record.schema,
        "source_schema_version": record.schema_version,
        "tick": record.tick,
        "t": record.t,
        "event": record.event,
        "competition_profile": record.competition_profile,
        "team": record.team,
        "strategy": record.strategy,
        "aim": record.aim,
        "target": record.target,
        "output": {
            "kind": output.kind,
            "goal_id": output.goal_id,
            "goal_base_id": output.goal_base_id,
            "goal_name": output.goal_name,
            "goal_side": output.goal_side,
            "goal_pos_cm": point_to_json(output.goal_pos_cm),
            "speed_level": output.speed_level,
            "publish_enabled": output.publish_enabled,
            "publish_allowed": output.publish_allowed,
            "output_topic": output.output_topic,
            "output_frame": output.output_frame,
            "final_goal_pos_topic": output.final_goal_pos_topic,
            "uses_goal_pos": output.uses_goal_pos,
            "uses_to_navi": output.uses_to_navi,
            "relative_target_valid": output.relative_target_valid,
            "chase_official_target_valid": output.chase_official_target_valid,
            "chase_official_armor_type": output.chase_official_armor_type,
            "source": output.source,
        },
        "intent": {
            "layer": intent.layer,
            "reason": intent.reason,
            "base_goal_id": intent.base_goal_id,
            "resolved_goal_id": intent.resolved_goal_id,
            "goal_team": intent.goal_team,
            "apply_team_offset": intent.apply_team_offset,
            "priority": intent.priority,
            "detail": intent.detail,
        },
        "events": {
            "summary": events.compact_text(),
            "event_data_fresh": events.event_data_fresh,
            "sentry_info_fresh": events.sentry_info_fresh,
            "buff_task_enabled": events.buff_task_enabled,
            "buff_can_activate": events.buff_can_activate,
            "buff_activating": events.buff_activating,
            "buff_activated": events.buff_activated,
            "outpost_task_enabled": events.outpost_task_enabled,
            "enemy_outpost_alive": events.enemy_outpost_alive,
            "outpost_attack_window_open": events.outpost_attack_window_open,
            "self_low_hp": events.self_low_hp,
            "self_low_ammo": events.self_low_ammo,
            "recent_damage_over_30": events.recent_damage_over_30,
            "armor_target_visible": events.armor_target_visible,
            "buff_target_locked": events.buff_target_locked,
            "outpost_target_locked": events.outpost_target_locked,
            "goal_reached": events.goal_reached,
            "goal_unreachable": events.goal_unreachable,
            "regional_defense_active": events.regional_defense_active,
        },
        "outpost_engagement_lock": {
            "active": record.outpost_engagement_lock.active,
            "hold_target": record.outpost_engagement_lock.hold_target,
            "enhanced_armed": record.outpost_engagement_lock.enhanced_armed,
            "enhanced_pending": record.outpost_engagement_lock.enhanced_pending,
            "enhanced_active": record.outpost_engagement_lock.enhanced_active,
            "enhanced_unavailable": record.outpost_engagement_lock.enhanced_unavailable,
            "exit_reason": record.outpost_engagement_lock.exit_reason,
            "normal_exit_hp": record.outpost_engagement_lock.normal_exit_hp,
            "enhanced_exit_hp": record.outpost_engagement_lock.enhanced_exit_hp,
        },
        "goal_reach": {
            "summary": goal_reach.compact_text(),
            "status": goal_reach.status,
            "status_id": goal_reach.status_id,
            "reason": goal_reach.reason,
            "reason_id": goal_reach.reason_id,
            "goal_id": goal_reach.goal_id,
            "base_goal_id": goal_reach.base_goal_id,
            "goal_age_ms": goal_reach.goal_age_ms,
            "external_reach_fresh": goal_reach.external_reach_fresh,
            "external_reach": goal_reach.external_reach,
            "external_reachable_fresh": goal_reach.external_reachable_fresh,
            "external_reachable": goal_reach.external_reachable,
            "position_fresh": goal_reach.position_fresh,
            "has_position": goal_reach.has_position,
            "distance_cm": goal_reach.distance_cm,
            "arrive_distance_cm": goal_reach.arrive_distance_cm,
            "face_distance_cm": goal_reach.face_distance_cm,
            "distance_fallback_allowed": goal_reach.distance_fallback_allowed,
            "within_arrive_distance": goal_reach.within_arrive_distance,
            "within_face_distance": goal_reach.within_face_distance,
            "near_goal_confirm_pending": goal_reach.near_goal_confirm_pending,
            "near_goal_confirm_elapsed_ms": goal_reach.near_goal_confirm_elapsed_ms,
            "timeout": goal_reach.timeout,
        },
        "navi_status": {
            "summary": navi_status.compact_text(),
            "should_rotate": navi_status.should_rotate,
            "should_rotate_fresh": navi_status.should_rotate_fresh,
            "reached": navi_status.reached,
            "reached_fresh": navi_status.reached_fresh,
            "reachable": navi_status.reachable,
            "reachable_fresh": navi_status.reachable_fresh,
        },
        "navi_velocity": {
            "summary": navi_velocity.compact_text(),
            "input_x": navi_velocity.input_x,
            "input_y": navi_velocity.input_y,
            "output_x": navi_velocity.output_x,
            "output_y": navi_velocity.output_y,
            "raw_to_mps": navi_velocity.raw_to_mps,
            "output_x_mps": scaled_velocity(navi_velocity.output_x, navi_velocity.raw_to_mps),
            "output_y_mps": scaled_velocity(navi_velocity.output_y, navi_velocity.raw_to_mps),
        },
        "navi_relative_target": {
            "valid": relative_target.valid,
            "frame_id": relative_target.frame_id,
            "x": relative_target.x,
            "y": relative_target.y,
            "z": relative_target.z,
            "distance": relative_target.distance,
            "yaw_error_deg": relative_target.yaw_error_deg,
            "pitch_error_deg": relative_target.pitch_error_deg,
            "armor_type": relative_target.armor_type,
            "aim_mode": relative_target.aim_mode,
            "official_target_valid": relative_target.official_target_valid,
            "official_armor_type": relative_target.official_armor_type,
        },
        "target_state": {
            "has_recent_target": target.has_recent_target,
            "external_aim_active": target.external_aim_active,
            "fresh_current_aim": target.fresh_current_aim,
            "fresh_auto_aim": target.fresh_auto_aim,
            "fresh_buff": target.fresh_buff,
            "fresh_outpost": target.fresh_outpost,
            "hitable_targets": list(target.hitable_targets),
            "reliable_enemy_positions": list(target.reliable_enemy_positions),
        },
        "posture": {
            "command": record.posture_command,
            "state": record.posture_state,
            "current": record.posture_current,
            "desired": record.posture_desired,
            "pending": record.posture_pending,
            "reason": record.posture_reason,
            "has_pending": posture_runtime.has_pending,
            "feedback_stale": posture_runtime.feedback_stale,
            "retry_count": posture_runtime.retry_count,
            "referee_timer_fresh": posture_runtime.referee_timer_fresh,
            "referee_enhanced_posture": posture_runtime.referee_enhanced_posture,
            "using_referee_timer": posture_runtime.using_referee_timer,
            "degraded_attack": posture_runtime.degraded_attack,
            "degraded_defense": posture_runtime.degraded_defense,
            "degraded_move": posture_runtime.degraded_move,
        },
        "referee": {
            "self_hp": referee.self_hp,
            "self_max_hp": referee.self_max_hp,
            "self_outpost_hp": referee.self_outpost_hp,
            "enemy_outpost_hp": referee.enemy_outpost_hp,
            "self_base_hp": referee.self_base_hp,
            "enemy_base_hp": referee.enemy_base_hp,
            "ammo": referee.ammo,
            "time_left": referee.time_left,
            "sentry_can_activate_energy": referee.sentry_can_activate_energy,
            "energy_activate_confirm_pulse": referee.energy_activate_confirm_pulse,
            "event_center_gain_point_status": referee.event_center_gain_point_status,
            "team_buff_attack": referee.team_buff_attack,
            "team_buff_defence": referee.team_buff_defence,
            "team_buff_remaining_energy": referee.team_buff_remaining_energy,
            "rfid_fresh": referee.rfid_fresh,
            "rfid_any": referee.rfid_any,
            "rfid_tunnel": referee.rfid_tunnel,
            "rfid_center_gain_point": referee.rfid_center_gain_point,
            "rfid_status": referee.rfid_status,
            "has_rfid_status_2": referee.has_rfid_status_2,
            "rfid_status_2": referee.rfid_status_2,
            "rfid_match": referee.rfid_match.as_payload(),
        },
        "gimbal": {
            "yaw_deg": gimbal.yaw_deg,
            "pitch_deg": gimbal.pitch_deg,
            "yaw_vel_deg_per_sec": gimbal.yaw_vel_deg_per_sec,
            "yaw_angle_deg": gimbal.yaw_angle_deg,
            "cap_v": gimbal.cap_v,
            "navi_lower_head": gimbal.navi_lower_head,
            "fire_status": gimbal.fire_status,
            "cap_state": gimbal.cap_state,
            "follow_mode": gimbal.follow_mode,
            "aim_mode": gimbal.aim_mode,
            "rotate": gimbal.rotate,
        },
        "bullet_info": {
            "summary": bullet.compact_text(),
            "has_received": bullet.has_received,
            "age_ms": bullet.age_ms,
            "has_initial_speed": bullet.has_initial_speed,
            "initial_speed": bullet.initial_speed,
            "has_shoot_data": bullet.has_shoot_data,
            "bullet_type": bullet.bullet_type,
            "shooter_number": bullet.shooter_number,
            "launching_frequency": bullet.launching_frequency,
            "has_projectile_allowance": bullet.has_projectile_allowance,
            "projectile_allowance_17mm": bullet.projectile_allowance_17mm,
            "projectile_allowance_42mm": bullet.projectile_allowance_42mm,
            "remaining_gold_coin": bullet.remaining_gold_coin,
            "projectile_allowance_fortress_17mm": bullet.projectile_allowance_fortress_17mm,
        },
        "runtime_guard": {
            "fault": guard.fault,
            "recovery_requested": guard.recovery_requested,
            "recovering": guard.recovering,
        },
        "units": [
            {
                "side": unit.side,
                "type": unit.type_name,
                "type_id": unit.type_id,
                "hp": unit.hp,
                "max_hp": unit.max_hp,
                "distance_m": unit.distance_m,
                "position_cm": point_to_json(unit.position_cm),
            }
            for unit in record.units
        ],
        "unit_info": [
            {
                "side": unit.side,
                "car_id": unit.car_id,
                "type": unit.type_name,
                "type_id": unit.type_id,
                "hp": unit.hp,
                "has_hp": unit.has_hp,
                "hp_fresh": unit.hp_fresh,
                "position_cm": point_to_json(unit.position_cm),
                "has_position": unit.has_position,
                "position_fresh": unit.position_fresh,
                "position_source": unit.position_source,
                "area_id": unit.area_id,
                "area_name": unit.area_name,
                "area_used_nearest_fallback": unit.area_used_nearest_fallback,
            }
            for unit in record.unit_info
        ],
    }


def record_to_control_output(record: TraceRecord) -> dict[str, Any]:
    output = record.control_output
    fire_code = output.fire_code
    trajectory = output.trajectory
    return {
        "schema": "ly_simulator_control_output_v1",
        "schema_version": 1,
        "source_schema": record.schema,
        "source_schema_version": record.schema_version,
        "tick": record.tick,
        "t": record.t,
        "available": output.available,
        "sequence": output.sequence,
        "age_ms": output.age_ms,
        "source": output.source,
        "angles": {
            "published": output.angles.published,
            "yaw": output.angles.yaw,
            "pitch": output.angles.pitch,
        },
        "fire_code": {
            "published": fire_code.published,
            "field_mask": fire_code.field_mask,
            "raw": fire_code.raw,
            "fire_status": fire_code.fire_status,
            "cap_state": fire_code.cap_state,
            "follow_mode": fire_code.follow_mode,
            "aim_mode": fire_code.aim_mode,
            "rotate": fire_code.rotate,
        },
        "trajectory": {
            "published": trajectory.published,
            "available": trajectory.available,
            "unavailable_reason": trajectory.unavailable_reason,
            "yaw": trajectory.yaw,
            "pitch": trajectory.pitch,
            "yaw_omega": trajectory.yaw_omega,
            "pitch_omega": trajectory.pitch_omega,
            "yaw_alpha": trajectory.yaw_alpha,
            "pitch_alpha": trajectory.pitch_alpha,
        },
    }


def record_to_tactical_evidence(record: TraceRecord) -> dict[str, Any]:
    tactical = record.tactical.as_payload()
    return {
        "schema": "ly_simulator_tactical_evidence_v1",
        "schema_version": 1,
        "source_schema": record.schema,
        "source_schema_version": record.schema_version,
        "tick": record.tick,
        "t": record.t,
        **tactical,
    }


def record_to_scene_observation(record: TraceRecord) -> dict[str, Any]:
    structures = (
        ("friend", "outpost", record.referee.self_outpost_hp),
        ("enemy", "outpost", record.referee.enemy_outpost_hp),
        ("friend", "base", record.referee.self_base_hp),
        ("enemy", "base", record.referee.enemy_base_hp),
    )
    return {
        "schema": "ly_simulator_scene_observation_v1",
        "schema_version": 1,
        "source_schema": record.schema,
        "source_schema_version": record.schema_version,
        "tick": record.tick,
        "t": record.t,
        "field": {
            "width_cm": record.field.width,
            "height_cm": record.field.height,
            "frame": record.field.frame,
        },
        "team": record.team,
        "units": [
            {
                "side": unit.side,
                "type": unit.type_name,
                "type_id": unit.type_id,
                "hp": unit.hp,
                "max_hp": unit.max_hp,
                "position_cm": point_to_json(unit.position_cm),
            }
            for unit in record.units
        ],
        "structures": [
            {"side": side, "kind": kind, "hp": hp}
            for side, kind, hp in structures
        ],
    }


def record_to_metrics(record: TraceRecord) -> dict[str, Any]:
    scale = record.navi_velocity.raw_to_mps
    return {
        "tick": record.tick,
        "t": record.t,
        "goal_id": record.output.goal_id,
        "goal_base_id": record.output.goal_base_id,
        "speed_level": record.output.speed_level,
        "hp": record.hp,
        "ammo": record.ammo,
        "time_left": record.time_left,
        "target_visible": int(record.events.armor_target_visible),
        "buff_locked": int(record.events.buff_target_locked),
        "outpost_locked": int(record.events.outpost_target_locked),
        "goal_reached": int(record.events.goal_reached),
        "goal_unreachable": int(record.events.goal_unreachable),
        "goal_reach_status_id": record.goal_reach.status_id,
        "goal_reach_reason_id": record.goal_reach.reason_id,
        "goal_distance_cm": record.goal_reach.distance_cm,
        "should_rotate": optional_int_bool(record.navi_status.should_rotate),
        "navi_reached": optional_int_bool(record.navi_status.reached),
        "navi_reachable": optional_int_bool(record.navi_status.reachable),
        "navi_velocity_output_x_raw": record.navi_velocity.output_x,
        "navi_velocity_output_y_raw": record.navi_velocity.output_y,
        "navi_velocity_output_x_mps": scaled_velocity(record.navi_velocity.output_x, scale),
        "navi_velocity_output_y_mps": scaled_velocity(record.navi_velocity.output_y, scale),
        "yaw_deg": record.gimbal.yaw_deg,
        "pitch_deg": record.gimbal.pitch_deg,
        "cap_v": record.gimbal.cap_v,
        "fire_status": record.gimbal.fire_status,
        "follow_mode": record.gimbal.follow_mode,
    }


def optional_int_bool(value: bool | None) -> int | None:
    return None if value is None else int(value)


def scaled_velocity(raw: int, scale: float | None) -> float | None:
    return None if scale is None else float(raw) * scale


def record_to_goal_point(record: TraceRecord) -> dict[str, float] | None:
    point = record.output.goal_pos_cm
    if point is None:
        return None
    return {"x": float(point[0]), "y": float(point[1])}


def point_to_json(point: tuple[float, float] | None) -> dict[str, float] | None:
    if point is None:
        return None
    return {"x": float(point[0]), "y": float(point[1])}


def json_bytes(payload: dict[str, Any]) -> bytes:
    return json.dumps(payload, ensure_ascii=True, separators=(",", ":"), sort_keys=True).encode("utf-8")


def time_ns(record: TraceRecord) -> int:
    return max(0, int(record.t * 1_000_000_000))


def export_records_with_writer(
    writer: Any,
    records: list[TraceRecord],
    *,
    topic_prefix: str = "/sentry/simulator",
    include_metrics: bool = True,
    include_goal_points: bool = True,
    message_encoding: Any = "json",
    schema_encoding: Any = "jsonschema",
) -> None:
    """Register the stable MCAP channels and write normalized trace evidence.

    Keeping the writer protocol separate from file ownership makes the channel
    contract testable without importing the optional ``mcap`` package.
    """

    prefix = "/" + topic_prefix.strip("/")

    def register_json_channel(name: str, schema: dict[str, Any], topic: str) -> int:
        schema_id = writer.register_schema(
            name=name,
            encoding=schema_encoding,
            data=json_bytes(schema),
        )
        return writer.register_channel(
            topic=topic,
            message_encoding=message_encoding,
            schema_id=schema_id,
        )

    channels = {
        "decision": register_json_channel(
            DECISION_SCHEMA_NAME,
            decision_frame_schema(),
            f"{prefix}/decision",
        ),
        "control_output": register_json_channel(
            CONTROL_OUTPUT_SCHEMA_NAME,
            control_output_schema(),
            f"{prefix}/decision/control_output",
        ),
        "tactical": register_json_channel(
            TACTICAL_SCHEMA_NAME,
            tactical_schema(),
            f"{prefix}/decision/tactical",
        ),
        "scene": register_json_channel(
            SCENE_SCHEMA_NAME,
            scene_observation_schema(),
            f"{prefix}/scene",
        ),
    }
    if include_metrics:
        channels["metrics"] = register_json_channel(
            METRICS_SCHEMA_NAME,
            metrics_schema(),
            f"{prefix}/metrics",
        )
    if include_goal_points:
        channels["goal_point"] = register_json_channel(
            POINT2_SCHEMA_NAME,
            point2_schema(),
            f"{prefix}/goal_point_cm",
        )

    def write(channel: str, sequence: int, stamp: int, payload: dict[str, Any]) -> None:
        writer.add_message(
            channel_id=channels[channel],
            log_time=stamp,
            publish_time=stamp,
            sequence=sequence,
            data=json_bytes(payload),
        )

    for sequence, record in enumerate(records):
        stamp = time_ns(record)
        write("decision", sequence, stamp, record_to_decision_frame(record))
        write("control_output", sequence, stamp, record_to_control_output(record))
        write("tactical", sequence, stamp, record_to_tactical_evidence(record))
        write("scene", sequence, stamp, record_to_scene_observation(record))
        if "metrics" in channels:
            write("metrics", sequence, stamp, record_to_metrics(record))
        if "goal_point" in channels:
            point = record_to_goal_point(record)
            if point is not None:
                write("goal_point", sequence, stamp, point)


def export_records_to_mcap(
    records: list[TraceRecord],
    output_path: Path,
    topic_prefix: str = "/sentry/simulator",
    include_metrics: bool = True,
    include_goal_points: bool = True,
) -> None:
    try:
        from mcap.writer import Writer
        from mcap.well_known import MessageEncoding, SchemaEncoding
    except ImportError as exc:
        raise RuntimeError(
            "Foxglove MCAP export requires the optional Python package 'mcap'. "
            "Install it with: python3 -m pip install -r src/simulator/requirements-foxglove.txt"
        ) from exc

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("wb") as stream:
        writer = Writer(stream)
        writer.start(library="ly simulator foxglove_export")
        export_records_with_writer(
            writer,
            records,
            topic_prefix=topic_prefix,
            include_metrics=include_metrics,
            include_goal_points=include_goal_points,
            message_encoding=MessageEncoding.JSON,
            schema_encoding=SchemaEncoding.JSONSchema,
        )
        writer.finish()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export simulator decision JSONL traces to Foxglove-readable MCAP.")
    parser.add_argument("trace", nargs="?", default="", help="Decision trace JSONL. Defaults to config paths.sample_trace.")
    parser.add_argument("-o", "--output", default="", help="Output .mcap path. Defaults next to the trace file.")
    parser.add_argument("--config", default="", help="Optional simulator YAML config for goal names.")
    parser.add_argument("--points-json", default="", help="Optional tools/maps map_plugin JSON/YAML with point coordinates.")
    parser.add_argument("--topic-prefix", default="/sentry/simulator", help="MCAP topic prefix.")
    parser.add_argument("--no-metrics", action="store_true", help="Do not export numeric metrics topic.")
    parser.add_argument("--no-goal-points", action="store_true", help="Do not export foxglove.Point2 goal-point topic.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    config = load_config(resolve_path(args.config) if args.config else None)
    paths = config.get("paths", {}) if isinstance(config.get("paths"), dict) else {}
    trace_path = resolve_path(args.trace or paths.get("sample_trace", "src/simulator/sample/sample_trace.jsonl"))
    if not trace_path.exists():
        print(f"trace file not found: {trace_path}", file=sys.stderr)
        return 2

    goals = goals_by_id(config)
    if args.points_json:
        goals.update(load_plugin_points(resolve_path(args.points_json)))
    goal_names = {goal_id: str(goal.get("name", f"Goal{goal_id}")) for goal_id, goal in goals.items()}

    output_path = Path(args.output).expanduser().resolve() if args.output else trace_path.with_suffix(".mcap")
    records, bad_lines = load_trace(trace_path, goal_names)
    if bad_lines:
        print(f"warning: skipped {bad_lines} bad trace line(s)", file=sys.stderr)

    try:
        export_records_to_mcap(
            records,
            output_path,
            topic_prefix=args.topic_prefix,
            include_metrics=not args.no_metrics,
            include_goal_points=not args.no_goal_points,
        )
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    print(f"wrote {len(records)} records to {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
