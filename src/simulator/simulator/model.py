from __future__ import annotations

from dataclasses import dataclass
from typing import Any


PointCm = tuple[float, float]


@dataclass(frozen=True)
class FieldState:
    width: int
    height: int
    frame: str

    def as_config(self) -> dict[str, Any]:
        if self.width <= 0 or self.height <= 0:
            return {}
        return {"width": self.width, "height": self.height, "frame": self.frame}


@dataclass(frozen=True)
class DecisionOutput:
    kind: str
    goal_id: int
    goal_base_id: int
    goal_name: str
    goal_side: str
    goal_pos_cm: PointCm | None
    speed_level: int
    publish_enabled: bool | None
    publish_allowed: bool | None
    output_topic: str
    output_frame: str
    final_goal_pos_topic: str
    uses_goal_pos: bool
    uses_to_navi: bool
    relative_target_valid: bool
    chase_official_target_valid: bool
    chase_official_armor_type: int
    source: str

    @property
    def route_key(self) -> tuple[int, str, PointCm | None]:
        return (self.goal_id, self.goal_side, self.goal_pos_cm)

    def publish_text(self) -> str:
        if self.publish_allowed is None and self.publish_enabled is None:
            return "-"
        return f"allowed={self.publish_allowed} enabled={self.publish_enabled}"

    def topic_text(self) -> str:
        if not self.output_topic and not self.final_goal_pos_topic:
            return "-"
        if self.output_topic == self.final_goal_pos_topic or not self.final_goal_pos_topic:
            return self.output_topic
        return f"{self.output_topic} -> {self.final_goal_pos_topic}"


@dataclass(frozen=True)
class UnitRecord:
    side: str
    type_name: str
    type_id: int
    hp: int
    max_hp: int
    distance_m: float
    position_cm: PointCm | None

    @property
    def health_ratio(self) -> float | None:
        if self.max_hp <= 0:
            return None
        return max(0.0, min(1.0, self.hp / self.max_hp))


@dataclass(frozen=True)
class UnitInfoRecord:
    side: str
    car_id: int
    type_id: int
    type_name: str
    hp: int
    has_hp: bool
    hp_fresh: bool
    position_cm: PointCm | None
    has_position: bool
    position_fresh: bool
    position_source: str
    area_id: int
    area_name: str
    area_used_nearest_fallback: bool

    def compact_text(self) -> str:
        hp_text = f"hp={self.hp}" if self.has_hp else "hp=-"
        pos_text = "pos=fresh" if self.position_fresh else "pos=stale" if self.has_position else "pos=-"
        source = self.position_source if self.position_source else "-"
        area = self.area_name if self.area_name else "-"
        return f"{self.type_name}:{hp_text} {pos_text} src={source} area={area}"


@dataclass(frozen=True)
class DecisionIntent:
    layer: str
    reason: str
    base_goal_id: int
    resolved_goal_id: int
    goal_team: str
    apply_team_offset: bool | None
    priority: int
    detail: str


@dataclass(frozen=True)
class TargetState:
    has_recent_target: bool
    external_aim_active: bool | None
    fresh_current_aim: bool | None
    fresh_auto_aim: bool
    fresh_buff: bool
    fresh_outpost: bool
    hitable_targets: tuple[str, ...]
    reliable_enemy_positions: tuple[str, ...]

    def fresh_text(self) -> str:
        def flag(value: bool | None) -> str:
            return "-" if value is None else str(int(value))

        parts = [
            f"recent={int(self.has_recent_target)}",
            f"external={flag(self.external_aim_active)}",
            f"current={flag(self.fresh_current_aim)}",
            f"auto={int(self.fresh_auto_aim)}",
            f"buff={int(self.fresh_buff)}",
            f"outpost={int(self.fresh_outpost)}",
        ]
        return " ".join(parts)


@dataclass(frozen=True)
class EventSnapshot:
    event_data_fresh: bool
    sentry_info_fresh: bool
    buff_task_enabled: bool
    buff_can_activate: bool
    buff_activating: bool
    buff_activated: bool
    outpost_task_enabled: bool
    enemy_outpost_alive: bool
    outpost_attack_window_open: bool
    self_low_hp: bool
    self_low_ammo: bool
    recent_damage_over_30: bool
    armor_target_visible: bool
    buff_target_locked: bool
    outpost_target_locked: bool
    goal_reached: bool
    goal_unreachable: bool
    regional_defense_active: bool
    self_fortress_gain_point_status: int
    self_outpost_gain_point_status: int
    self_base_gain_point_status: bool

    def compact_text(self) -> str:
        parts: list[str] = []
        if self.regional_defense_active:
            parts.append("RD")
        if self.buff_activated:
            parts.append("Buff=done")
        elif self.buff_activating:
            parts.append("Buff=active")
        elif self.buff_can_activate:
            parts.append("Buff=can")
        elif self.buff_task_enabled:
            parts.append("Buff=task")
        if self.enemy_outpost_alive:
            parts.append("Outpost=alive")
        if self.outpost_attack_window_open:
            parts.append("Window=open")
        risks = []
        if self.self_low_hp:
            risks.append("hp")
        if self.self_low_ammo:
            risks.append("ammo")
        if self.recent_damage_over_30:
            risks.append("damage")
        if risks:
            parts.append("Low=" + ",".join(risks))
        if self.goal_reached:
            parts.append("Goal=reached")
        if self.goal_unreachable:
            parts.append("Goal=blocked")
        return " ".join(parts) if parts else "-"

    def change_key(self) -> tuple[object, ...]:
        return (
            self.regional_defense_active,
            self.buff_can_activate,
            self.buff_activating,
            self.buff_activated,
            self.outpost_attack_window_open,
            self.self_low_hp,
            self.self_low_ammo,
            self.recent_damage_over_30,
            self.goal_reached,
            self.goal_unreachable,
        )


@dataclass(frozen=True)
class RelativeTarget:
    valid: bool
    frame_id: str
    x: float | None
    y: float | None
    z: float | None
    distance: float | None
    yaw_error_deg: float | None
    pitch_error_deg: float | None
    armor_type: int
    aim_mode: int
    official_target_valid: bool
    official_armor_type: int


@dataclass(frozen=True)
class GoalReachState:
    status: str
    status_id: int
    reason: str
    reason_id: int
    goal_id: int
    base_goal_id: int
    goal_age_ms: int | None
    external_reach_fresh: bool
    external_reach: bool
    external_reachable_fresh: bool
    external_reachable: bool
    position_fresh: bool
    has_position: bool
    distance_cm: float | None
    arrive_distance_cm: int
    face_distance_cm: int
    distance_fallback_allowed: bool
    within_arrive_distance: bool
    within_face_distance: bool
    near_goal_confirm_pending: bool
    near_goal_confirm_elapsed_ms: int
    timeout: bool

    @property
    def has_detail(self) -> bool:
        return self.status != "unknown" or self.reason != "none" or self.distance_cm is not None

    def compact_text(self) -> str:
        if not self.has_detail:
            return "-"
        distance = "-" if self.distance_cm is None else f"{self.distance_cm:.0f}cm"
        return f"{self.status}/{self.reason} dist={distance} age={self.goal_age_ms if self.goal_age_ms is not None else '-'}ms"


@dataclass(frozen=True)
class NaviVelocity:
    input_x: int
    input_y: int
    output_x: int
    output_y: int
    raw_to_mps: float | None

    def compact_text(self) -> str:
        scale = "" if self.raw_to_mps is None else f" scale={self.raw_to_mps:g}"
        return f"in=({self.input_x},{self.input_y}) out=({self.output_x},{self.output_y}){scale}"


@dataclass(frozen=True)
class NaviStatus:
    should_rotate: bool | None
    should_rotate_fresh: bool | None
    reached: bool | None
    reached_fresh: bool | None
    reachable: bool | None
    reachable_fresh: bool | None

    def compact_text(self) -> str:
        def flag(value: bool | None) -> str:
            return "-" if value is None else str(int(value))

        return (
            f"rotate={flag(self.should_rotate)} fresh={flag(self.should_rotate_fresh)} "
            f"reached={flag(self.reached)} reachable={flag(self.reachable)}"
        )


@dataclass(frozen=True)
class FaceModeState:
    requested: bool | None
    active: bool | None
    patrol_fallback: bool | None
    suppress_fire: bool | None
    source: str
    phase: str
    has_angles: bool | None
    yaw: float | None
    pitch: float | None

    def compact_text(self) -> str:
        if not self.requested:
            return "off"
        state = "active" if self.active else ("patrol" if self.patrol_fallback else "suppressed")
        return f"{state} source={self.source} phase={self.phase}"


@dataclass(frozen=True)
class GimbalState:
    yaw_deg: float | None
    pitch_deg: float | None
    yaw_vel_deg_per_sec: float | None
    yaw_angle_deg: float | None
    cap_v: int
    navi_lower_head: int
    fire_status: int
    cap_state: int
    follow_mode: int
    aim_mode: int
    rotate: int


@dataclass(frozen=True)
class FireCodeSnapshot:
    field_mask: int | None
    raw: int | None
    fire_status: int | None
    cap_state: int | None
    follow_mode: bool | None
    aim_mode: bool | None
    rotate: int | None


@dataclass(frozen=True)
class GimbalFeedbackState:
    available: bool
    age_ms: int | None
    fire_code: FireCodeSnapshot

    @property
    def fire_status(self) -> int | None:
        return self.fire_code.fire_status

    @property
    def cap_state(self) -> int | None:
        return self.fire_code.cap_state

    @property
    def follow_mode(self) -> bool | None:
        return self.fire_code.follow_mode

    @property
    def aim_mode(self) -> bool | None:
        return self.fire_code.aim_mode

    @property
    def rotate(self) -> int | None:
        return self.fire_code.rotate


@dataclass(frozen=True)
class ControlAnglesState:
    published: bool
    yaw: float | None
    pitch: float | None


@dataclass(frozen=True)
class ControlFireCodeState:
    published: bool
    field_mask: int | None
    raw: int | None
    fire_status: int | None
    cap_state: int | None
    follow_mode: bool | None
    aim_mode: bool | None
    rotate: int | None


@dataclass(frozen=True)
class ControlTrajectoryState:
    published: bool
    available: bool
    unavailable_reason: str
    yaw: float | None
    pitch: float | None
    yaw_omega: float | None
    pitch_omega: float | None
    yaw_alpha: float | None
    pitch_alpha: float | None


@dataclass(frozen=True)
class ControlOutputState:
    available: bool
    sequence: int | None
    age_ms: int | None
    source: str
    angles: ControlAnglesState
    fire_code: ControlFireCodeState
    trajectory: ControlTrajectoryState

    @property
    def is_available(self) -> bool:
        return self.available


@dataclass(frozen=True)
class TacticalDecisionState:
    """BT-owned tactical evidence serialized alongside a decision trace row."""

    available: bool
    protect_castle_enabled: bool | None
    protect_castle_rfid_enabled: bool | None
    protect_castle_stay_when_rfid_enabled: bool | None
    protect_castle_stay_when_rfid_active: bool | None
    protect_castle_enemy_pos_enabled: bool | None
    protect_castle_rfid_event_raw_active: bool | None
    protect_castle_rfid_event_active: bool | None
    protect_castle_enemy_pos_active: bool | None
    protect_outpost_enabled: bool | None
    protect_outpost_hp: int | None
    protect_outpost_hp_fresh: bool | None
    protect_outpost_destroyed: bool | None
    protect_outpost_phase: str
    protect_outpost_event_generation: int | None
    protect_outpost_damage_window_ms: int | None
    protect_outpost_damage_threshold_hp: int | None
    protect_outpost_search_hold_sec: int | None
    protect_outpost_priority: int | None
    protect_hero_enabled: bool | None
    protect_hero_proactive_hold_when_hero_in_highland: bool | None
    protect_hero_active: bool | None
    protect_hero_priority: int | None
    protect_hero_profile_ready: bool | None
    protect_hero_elapsed_ready: bool | None
    protect_hero_position_fresh: bool | None
    protect_hero_position_valid: bool | None
    protect_hero_x_cm: int | None
    protect_hero_y_cm: int | None
    protect_hero_in_highland: bool | None
    protect_hero_in_protect_area: bool | None
    protect_hero_health_fresh: bool | None
    protect_hero_known_dead: bool | None
    protect_hero_own_base_enemy_count: int | None
    protect_hero_own_highland_enemy_count: int | None
    protect_hero_threat_ready: bool | None
    protect_hero_mode_gate_ready: bool | None
    protect_hero_start_elapsed_sec: int | None
    protect_hero_hold_sec: int | None
    protect_hero_no_enemy_release_sec: int | None
    protect_hero_position_fresh_ms: int | None
    protect_hero_health_fresh_ms: int | None
    protect_hero_goal_base_id: int | None
    regional_defense_threat_active: bool | None
    regional_defense_search_kind: str
    regional_defense_fortress_enemy_count: int | None
    regional_defense_own_base_enemy_count: int | None

    def as_payload(self) -> dict[str, Any]:
        return {
            "available": self.available,
            "protect_castle": {
                "enabled": self.protect_castle_enabled,
                "rfid_enabled": self.protect_castle_rfid_enabled,
                "stay_when_rfid_enabled": self.protect_castle_stay_when_rfid_enabled,
                "stay_when_rfid_active": self.protect_castle_stay_when_rfid_active,
                "enemy_pos_enabled": self.protect_castle_enemy_pos_enabled,
                "rfid_event_raw_active": self.protect_castle_rfid_event_raw_active,
                "rfid_event_active": self.protect_castle_rfid_event_active,
                "enemy_pos_active": self.protect_castle_enemy_pos_active,
            },
            "protect_outpost": {
                "enabled": self.protect_outpost_enabled,
                "hp": self.protect_outpost_hp,
                "hp_fresh": self.protect_outpost_hp_fresh,
                "destroyed": self.protect_outpost_destroyed,
                "phase": self.protect_outpost_phase,
                "event_generation": self.protect_outpost_event_generation,
                "damage_window_ms": self.protect_outpost_damage_window_ms,
                "damage_threshold_hp": self.protect_outpost_damage_threshold_hp,
                "search_hold_sec": self.protect_outpost_search_hold_sec,
                "priority": self.protect_outpost_priority,
            },
            "protect_hero": {
                "enabled": self.protect_hero_enabled,
                "proactive_hold_when_hero_in_highland": self.protect_hero_proactive_hold_when_hero_in_highland,
                "active": self.protect_hero_active,
                "priority": self.protect_hero_priority,
                "profile_ready": self.protect_hero_profile_ready,
                "elapsed_ready": self.protect_hero_elapsed_ready,
                "position_fresh": self.protect_hero_position_fresh,
                "position_valid": self.protect_hero_position_valid,
                "hero_x_cm": self.protect_hero_x_cm,
                "hero_y_cm": self.protect_hero_y_cm,
                "in_highland": self.protect_hero_in_highland,
                "in_protect_area": self.protect_hero_in_protect_area,
                "health_fresh": self.protect_hero_health_fresh,
                "known_dead": self.protect_hero_known_dead,
                "own_base_enemy_count": self.protect_hero_own_base_enemy_count,
                "own_highland_enemy_count": self.protect_hero_own_highland_enemy_count,
                "threat_ready": self.protect_hero_threat_ready,
                "mode_gate_ready": self.protect_hero_mode_gate_ready,
                "start_elapsed_sec": self.protect_hero_start_elapsed_sec,
                "hold_sec": self.protect_hero_hold_sec,
                "no_enemy_release_sec": self.protect_hero_no_enemy_release_sec,
                "position_fresh_ms": self.protect_hero_position_fresh_ms,
                "health_fresh_ms": self.protect_hero_health_fresh_ms,
                "goal_base_id": self.protect_hero_goal_base_id,
            },
            "regional_defense": {
                "threat_active": self.regional_defense_threat_active,
                "search_kind": self.regional_defense_search_kind,
                "fortress_enemy_count": self.regional_defense_fortress_enemy_count,
                "own_base_enemy_count": self.regional_defense_own_base_enemy_count,
            },
        }


@dataclass(frozen=True)
class BulletInfoState:
    has_received: bool
    age_ms: int | None
    has_initial_speed: bool
    initial_speed: float | None
    has_shoot_data: bool
    bullet_type: int
    shooter_number: int
    launching_frequency: int
    has_projectile_allowance: bool
    projectile_allowance_17mm: int
    projectile_allowance_42mm: int
    remaining_gold_coin: int
    projectile_allowance_fortress_17mm: int

    def compact_text(self) -> str:
        if not self.has_received:
            return "not received"
        speed = "-" if self.initial_speed is None else f"{self.initial_speed:.1f}m/s"
        age = "-" if self.age_ms is None else f"{self.age_ms}ms"
        return (
            f"speed={speed} shoot={int(self.has_shoot_data)} "
            f"allow17={self.projectile_allowance_17mm} age={age}"
        )


@dataclass(frozen=True)
class RuntimeGuard:
    fault: str
    recovery_requested: bool
    recovering: bool


@dataclass(frozen=True)
class RfidMatchState:
    fresh: bool | None
    any: bool | None
    raw: int | None
    has_rfid_status_2: bool | None
    rfid_status_2_raw: int | None
    self_base_gain_point: bool | None
    self_supply: bool | None
    self_non_resource_supply: bool | None
    self_resource_supply: bool | None
    self_highland_gain_point: bool | None
    enemy_highland_gain_point: bool | None
    self_road_crossing: bool | None
    enemy_road_crossing: bool | None
    self_central_highland_crossing: bool | None
    enemy_central_highland_crossing: bool | None
    self_tunnel: bool | None
    enemy_tunnel: bool | None
    tunnel: bool | None
    center_gain_point: bool | None
    self_fortress_gain_point: bool | None
    enemy_fortress_gain_point: bool | None
    self_outpost_gain_point: bool | None
    enemy_outpost_gain_point: bool | None
    self_assembly_gain_point: bool | None
    enemy_assembly_gain_point: bool | None
    self_fly_ramp: bool | None
    enemy_fly_ramp: bool | None
    on_self_side: bool | None
    on_enemy_side: bool | None

    def as_payload(self) -> dict[str, Any]:
        return {
            "fresh": self.fresh,
            "any": self.any,
            "raw": self.raw,
            "has_rfid_status_2": self.has_rfid_status_2,
            "rfid_status_2_raw": self.rfid_status_2_raw,
            "self_base_gain_point": self.self_base_gain_point,
            "self_supply": self.self_supply,
            "self_non_resource_supply": self.self_non_resource_supply,
            "self_resource_supply": self.self_resource_supply,
            "self_highland_gain_point": self.self_highland_gain_point,
            "enemy_highland_gain_point": self.enemy_highland_gain_point,
            "self_road_crossing": self.self_road_crossing,
            "enemy_road_crossing": self.enemy_road_crossing,
            "self_central_highland_crossing": self.self_central_highland_crossing,
            "enemy_central_highland_crossing": self.enemy_central_highland_crossing,
            "self_tunnel": self.self_tunnel,
            "enemy_tunnel": self.enemy_tunnel,
            "tunnel": self.tunnel,
            "center_gain_point": self.center_gain_point,
            "self_fortress_gain_point": self.self_fortress_gain_point,
            "enemy_fortress_gain_point": self.enemy_fortress_gain_point,
            "self_outpost_gain_point": self.self_outpost_gain_point,
            "enemy_outpost_gain_point": self.enemy_outpost_gain_point,
            "self_assembly_gain_point": self.self_assembly_gain_point,
            "enemy_assembly_gain_point": self.enemy_assembly_gain_point,
            "self_fly_ramp": self.self_fly_ramp,
            "enemy_fly_ramp": self.enemy_fly_ramp,
            "on_self_side": self.on_self_side,
            "on_enemy_side": self.on_enemy_side,
        }


@dataclass(frozen=True)
class RefereeState:
    self_hp: int
    self_max_hp: int
    self_outpost_hp: int | None
    enemy_outpost_hp: int | None
    self_base_hp: int | None
    enemy_base_hp: int | None
    ammo: int
    time_left: int
    sentry_can_activate_energy: bool
    energy_activate_confirm_pulse: bool
    event_center_gain_point_status: int | None
    event_self_fortress_gain_point_status: int | None
    event_self_outpost_gain_point_status: int | None
    event_self_base_gain_point_status: bool | None
    team_buff_attack: int | None
    team_buff_defence: int | None
    team_buff_remaining_energy: int | None
    rfid_status: int | None
    has_rfid_status_2: bool | None
    rfid_status_2: int | None
    rfid_match: RfidMatchState

    @property
    def rfid_fresh(self) -> bool | None:
        return self.rfid_match.fresh

    @property
    def rfid_any(self) -> bool | None:
        return self.rfid_match.any

    @property
    def rfid_tunnel(self) -> bool | None:
        return self.rfid_match.tunnel

    @property
    def rfid_center_gain_point(self) -> bool | None:
        return self.rfid_match.center_gain_point


@dataclass(frozen=True)
class PostureRuntime:
    has_pending: bool
    feedback_stale: bool
    retry_count: int | None
    referee_timer_fresh: bool
    referee_enhanced_posture: bool
    using_referee_timer: bool
    degraded_attack: bool
    degraded_defense: bool
    degraded_move: bool


@dataclass(frozen=True)
class OutpostEngagementLock:
    active: bool
    hold_target: bool
    enhanced_armed: bool
    enhanced_pending: bool
    enhanced_active: bool
    enhanced_unavailable: bool
    exit_reason: str
    normal_exit_hp: int
    enhanced_exit_hp: int


@dataclass(frozen=True)
class TraceRecord:
    raw: dict[str, Any]
    index: int
    schema: str
    schema_version: int
    has_decision_output: bool
    has_decision_intent: bool
    t: float
    event: str
    tick: int
    field: FieldState
    competition_profile: str
    team: str
    strategy: str
    aim: str
    target: str
    target_state: TargetState
    output: DecisionOutput
    decision_intent: DecisionIntent
    events: EventSnapshot
    navi_relative_target: RelativeTarget
    goal_reach: GoalReachState
    navi_velocity: NaviVelocity
    navi_status: NaviStatus
    face_mode: FaceModeState
    goal_id: int
    goal_base_id: int
    goal_name: str
    goal_side: str
    goal_position: PointCm | None
    speed_level: int
    posture_command: str
    posture_state: str
    posture_current: str
    posture_desired: str
    posture_pending: str
    posture_reason: str
    posture_runtime: PostureRuntime
    outpost_engagement_lock: OutpostEngagementLock
    hp: int
    ammo: int
    time_left: int
    referee: RefereeState
    units: tuple[UnitRecord, ...]
    unit_info: tuple[UnitInfoRecord, ...]
    gimbal: GimbalState
    gimbal_feedback: GimbalFeedbackState
    control_output: ControlOutputState
    tactical: TacticalDecisionState
    bullet_info: BulletInfoState
    runtime_guard: RuntimeGuard
