from __future__ import annotations

import bisect
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

from .assets import load_unit_asset_catalog
from .control_bus import append_command, command_name, read_commands
from .field import FieldGeometry
from .field import field_to_screen as field_to_screen_point
from .field import screen_to_field as screen_to_field_point
from .inputs_panel import InputsPanel
from .interactive_inputs import SimulatorInputState, unit_decision_summary
from .model import TraceRecord, UnitInfoRecord, UnitRecord
from .panel_scroll import PanelScrollState
from .trace import as_dict, as_list, build_changes, load_trace_incremental, parse_position
from .workspace import Rect as WorkspaceRect
from .workspace import Selection, ViewportState, WorkspaceLayout


def fit_rect(pg: Any, src_size: tuple[int, int], dst_rect: Any) -> Any:
    src_w, src_h = src_size
    scale = min(dst_rect.width / src_w, dst_rect.height / src_h)
    width = int(src_w * scale)
    height = int(src_h * scale)
    return pg.Rect(
        dst_rect.x + (dst_rect.width - width) // 2,
        dst_rect.y + (dst_rect.height - height) // 2,
        width,
        height,
    )


def color(pg: Any, colors: dict[str, str], name: str, fallback: str) -> Any:
    return pg.Color(colors.get(name, fallback))


def record_position(record: TraceRecord, goals: dict[int, dict[str, Any]]) -> tuple[float, float] | None:
    if record.output.kind == "relative_target_bridge":
        return None
    goal = goals.get(record.goal_base_id)
    if goal:
        pos = goal.get(record.goal_side)
        if pos is not None and pos != (0, 0):
            return pos
    if record.output.goal_pos_cm is not None and record.output.uses_goal_pos:
        return record.output.goal_pos_cm
    if record.output.goal_pos_cm is not None:
        return record.output.goal_pos_cm
    return None


def point_payload(pos: tuple[float, float] | None) -> list[float] | None:
    return [float(pos[0]), float(pos[1])] if pos is not None else None


def record_status_payload(record: TraceRecord) -> dict[str, Any]:
    output = record.output
    intent = record.decision_intent
    relative_target = record.navi_relative_target
    goal_reach = record.goal_reach
    navi_status = record.navi_status
    navi_velocity = record.navi_velocity
    face_mode = record.face_mode
    payload = {
        "index": record.index,
        "tick": record.tick,
        "event": record.event,
        "t": record.t,
        "team": record.team,
        "strategy": record.strategy,
        "aim": record.aim,
        "target": record.target,
        "goal": {
            "id": output.goal_id,
            "base_id": output.goal_base_id,
            "name": output.goal_name,
            "side": output.goal_side,
            "pos_cm": point_payload(output.goal_pos_cm),
        },
        "output": {
            "kind": output.kind,
            "topic": output.output_topic,
            "final_topic": output.final_goal_pos_topic,
            "frame": output.output_frame,
            "publish_enabled": output.publish_enabled,
            "publish_allowed": output.publish_allowed,
            "speed_level": output.speed_level,
            "uses_goal_pos": output.uses_goal_pos,
            "uses_to_navi": output.uses_to_navi,
            "relative_target_valid": output.relative_target_valid,
            "chase_official_target_valid": output.chase_official_target_valid,
            "chase_official_armor_type": output.chase_official_armor_type,
        },
        "goal_reach": {
            "status": goal_reach.status,
            "status_id": goal_reach.status_id,
            "reason": goal_reach.reason,
            "reason_id": goal_reach.reason_id,
            "goal_id": goal_reach.goal_id,
            "base_goal_id": goal_reach.base_goal_id,
            "goal_age_ms": goal_reach.goal_age_ms,
            "distance_cm": goal_reach.distance_cm,
            "external_reach_fresh": goal_reach.external_reach_fresh,
            "external_reach": goal_reach.external_reach,
            "external_reachable_fresh": goal_reach.external_reachable_fresh,
            "external_reachable": goal_reach.external_reachable,
            "position_fresh": goal_reach.position_fresh,
            "has_position": goal_reach.has_position,
            "near_goal_confirm_pending": goal_reach.near_goal_confirm_pending,
            "near_goal_confirm_elapsed_ms": goal_reach.near_goal_confirm_elapsed_ms,
            "timeout": goal_reach.timeout,
        },
        "navi_status": {
            "should_rotate": navi_status.should_rotate,
            "should_rotate_fresh": navi_status.should_rotate_fresh,
            "reached": navi_status.reached,
            "reached_fresh": navi_status.reached_fresh,
            "reachable": navi_status.reachable,
            "reachable_fresh": navi_status.reachable_fresh,
        },
        "navi_velocity": {
            "input_x": navi_velocity.input_x,
            "input_y": navi_velocity.input_y,
            "output_x": navi_velocity.output_x,
            "output_y": navi_velocity.output_y,
            "raw_to_mps": navi_velocity.raw_to_mps,
        },
        "face_mode": {
            "requested": face_mode.requested,
            "active": face_mode.active,
            "patrol_fallback": face_mode.patrol_fallback,
            "suppress_fire": face_mode.suppress_fire,
            "source": face_mode.source,
            "phase": face_mode.phase,
            "has_angles": face_mode.has_angles,
            "yaw": face_mode.yaw,
            "pitch": face_mode.pitch,
        },
        "relative_target": {
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
        "intent": {"layer": intent.layer, "reason": intent.reason, "priority": intent.priority},
        "posture": {
            "command": record.posture_command,
            "state": record.posture_state,
            "current": record.posture_current,
            "desired": record.posture_desired,
            "using_referee_timer": record.posture_runtime.using_referee_timer,
            "referee_enhanced_posture": record.posture_runtime.referee_enhanced_posture,
        },
        "referee": {
            "hp": record.hp,
            "ammo": record.ammo,
            "time_left": record.time_left,
            "rfid_status": record.referee.rfid_status,
            "has_rfid_status_2": record.referee.has_rfid_status_2,
            "rfid_status_2": record.referee.rfid_status_2,
            "rfid_match": record.referee.rfid_match.as_payload(),
        },
        "bullet_info": bullet_info_status_payload(record),
        "gimbal_feedback": gimbal_feedback_status_payload(record),
        "control_output": control_output_status_payload(record),
        "tactical": tactical_status_payload(record),
        "runtime_guard": {
            "fault": record.runtime_guard.fault,
            "recovering": record.runtime_guard.recovering,
        },
    }
    if record.unit_info:
        payload["unit_info"] = unit_info_status_payload(record)
    return payload


def unit_info_status_payload(record: TraceRecord) -> dict[str, Any]:
    friend = [unit for unit in record.unit_info if unit.side == "friend"]
    enemy = [unit for unit in record.unit_info if unit.side == "enemy"]
    fresh_enemy = [unit.type_name for unit in enemy if unit.position_fresh and unit.has_position]
    fresh_friend = [unit.type_name for unit in friend if unit.position_fresh and unit.has_position]
    return {
        "friend": len(friend),
        "enemy": len(enemy),
        "fresh_friend_positions": fresh_friend,
        "fresh_enemy_positions": fresh_enemy,
    }


def bullet_info_status_payload(record: TraceRecord) -> dict[str, Any]:
    bullet = record.bullet_info
    return {
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
    }


def gimbal_feedback_status_payload(record: TraceRecord) -> dict[str, Any]:
    feedback = record.gimbal_feedback
    fire_code = feedback.fire_code
    return {
        "available": feedback.available,
        "age_ms": feedback.age_ms,
        "fire_code": {
            "field_mask": fire_code.field_mask,
            "raw": fire_code.raw,
            "fire_status": fire_code.fire_status,
            "cap_state": fire_code.cap_state,
            "follow_mode": fire_code.follow_mode,
            "aim_mode": fire_code.aim_mode,
            "rotate": fire_code.rotate,
        },
    }


def control_output_status_payload(record: TraceRecord) -> dict[str, Any]:
    output = record.control_output
    fire_code = output.fire_code
    trajectory = output.trajectory
    return {
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


def tactical_status_payload(record: TraceRecord) -> dict[str, Any]:
    return record.tactical.as_payload()


class Viewer:
    def __init__(
        self,
        pygame: Any,
        records: list[TraceRecord],
        changes: list[dict[str, Any]],
        config: dict[str, Any],
        goals: dict[int, dict[str, Any]],
        map_path: Path,
        bad_lines: int,
        start_paused: bool | None,
        speed: float | None,
        trace_path: Path | None = None,
        goal_names: dict[int, str] | None = None,
        follow: bool = False,
        follow_poll_sec: float = 0.25,
        follow_offset: int = 0,
        streamer: Any | None = None,
    ) -> None:
        self.pg = pygame
        self.records = records
        self.changes = changes
        self.config = config
        self.goals = goals
        self.map_path = map_path
        self.bad_lines = bad_lines
        self.trace_path = trace_path
        self.goal_names = goal_names or {}
        self.follow = bool(follow and trace_path is not None)
        self.follow_poll_sec = max(0.05, float(follow_poll_sec))
        self.follow_offset = max(0, int(follow_offset))
        self.last_follow_poll = time.perf_counter()
        self.streamer = streamer
        self.live_ros_topics: dict[str, Any] = {}
        self.live_ros_wall_time = 0.0
        self.last_ros_monitor_poll = time.perf_counter()
        self.live_goal_history: list[tuple[float, float]] = []

        window = as_dict(config.get("window"))
        self.width = int(window.get("width", 1500))
        self.height = int(window.get("height", 900))
        self.windowed_size = (self.width, self.height)
        self.fullscreen = False
        self.min_width = int(window.get("min_width", 960))
        self.min_height = int(window.get("min_height", 620))
        self.panel_w = int(window.get("panel_width", 390))
        self.timeline_h = int(window.get("timeline_height", 92))
        workspace = as_dict(config.get("workspace"))
        self.inspector_width = max(280, int(workspace.get("inspector_width", self.panel_w)))
        self.inspector_zone = "left" if workspace.get("inspector_zone") == "left" else "right"
        self.inspector_collapsed = bool(workspace.get("inspector_collapsed", False))
        self.shelf_height = max(140, int(workspace.get("shelf_height", self.timeline_h)))
        self.shelf_collapsed = bool(workspace.get("shelf_collapsed", True))
        self.playing = not (window.get("start_paused", False) if start_paused is None else start_paused)
        self.playback_speed = float(window.get("playback_speed", 1.0) if speed is None else speed)
        self.show_labels = bool(window.get("show_point_labels", False))

        self.layers = as_dict(config.get("layers"))
        self.layer_buttons: dict[str, Any] = {}
        self.timeline_config = as_dict(config.get("timeline"))
        self.panel_tab = "decision"
        self.panel_tab_buttons: dict[str, Any] = {}
        self.panel_scroll = PanelScrollState()
        self.panel_body_rect: Any | None = None
        self.ros_monitor = as_dict(config.get("ros_monitor"))
        ros_state_file = str(self.ros_monitor.get("state_file", "")).strip()
        self.ros_state_path: Path | None = Path(ros_state_file).expanduser().resolve() if ros_state_file else None
        self.ros_monitor_poll_sec = max(0.02, self.to_float(self.ros_monitor.get("poll_sec"), 0.05))
        self.ros_monitor_stale_sec = max(0.1, self.to_float(self.ros_monitor.get("stale_sec"), 1.0))
        self.map_tags = as_dict(config.get("map_tags"))
        self.goal_tags_expanded = bool(self.map_tags.get("goal_tags_expanded", False))
        self.hover_goal_tags = bool(self.map_tags.get("hover_goal_tags", True))
        self.goal_tag_hover_radius_px = max(4, int(self.map_tags.get("hover_radius_px", 18)))
        self.goal_tag_button_rect = None
        self.colors_raw = as_dict(config.get("colors"))
        self.unit_styles = as_dict(config.get("unit_styles"))
        self.unit_assets = load_unit_asset_catalog(as_dict(config.get("assets")))
        self.unit_sprite_cache: dict[tuple[str, str, int, str], Any | None] = {}
        self.armor_sprite_cache: dict[tuple[str, int, str], Any | None] = {}
        self.simulator_inputs = as_dict(config.get("simulator_inputs"))
        self.simulator_inputs_enabled = bool(self.simulator_inputs.get("enabled", True))
        self.show_trace_units_with_scene = bool(self.simulator_inputs.get("show_trace_units_with_scene", False))
        self.default_field = FieldGeometry.from_config(config.get("field_cm"))
        self.sim_input_state = SimulatorInputState.from_config(
            self.simulator_inputs,
            field=self.default_field,
        )
        self.inputs_panel = InputsPanel(self)
        self.dragging_unit: Any | None = None
        self.drag_position: tuple[int, int] | None = None
        self.selected_structure_key: str | None = None
        self.workspace_selection = Selection()
        self.map_pan_anchor: tuple[int, int] | None = None
        self.map_tool_buttons: dict[str, Any] = {}
        self.inspector_buttons: dict[str, tuple[Any, dict[str, Any]]] = {}
        self.workspace_buttons: dict[str, Any] = {}
        self.activity_buttons: dict[str, Any] = {}
        self.workspace_splitter: str | None = None
        self.next_scene_entity_id = 0
        self.times = [record.t for record in records]
        self.current_index = 0
        self.current_time = self.times[0]
        self.match_control = as_dict(config.get("match_control"))
        self.match_control_enabled = bool(self.match_control.get("enabled", False))
        self.match_duration_sec = max(1, int(self.match_control.get("duration_sec", 420)))
        self.rewind_step_sec = max(1, int(self.match_control.get("rewind_step_sec", 10)))
        self.forward_step_sec = max(1, int(self.match_control.get("forward_step_sec", 10)))
        control_file = str(self.match_control.get("control_file", "")).strip()
        self.control_path: Path | None = None
        if control_file:
            self.control_path = Path(control_file).expanduser().resolve()
        self.control_buttons: dict[str, Any] = {}
        self.last_control_status = "idle"
        initial_time_left = self.records[self.current_index].time_left
        if initial_time_left <= 0:
            initial_time_left = self.match_duration_sec
        self.match_time_left_sec = float(max(0, min(self.match_duration_sec, int(initial_time_left))))
        self.match_started = False
        self.match_running = False
        self.last_trace_time_left = int(round(self.match_time_left_sec))
        self.last_control_poll = time.perf_counter()
        self.control_poll_sec = 0.08
        self.control_read_offset = 0
        if self.control_path is not None and self.control_path.exists():
            try:
                self.control_read_offset = int(self.control_path.stat().st_size)
            except OSError:
                self.control_read_offset = 0

        self.scripted_path = as_dict(config.get("scripted_path"))
        default_side = self.records[self.current_index].team
        self.scripted_side = self.normalize_side(str(self.scripted_path.get("side", "auto")), default_side)
        self.scripted_speed_cmps = max(0.0, self.to_float(self.scripted_path.get("speed_cmps"), 0.0))
        self.scripted_loop = bool(self.scripted_path.get("loop", False))
        self.scripted_show_labels = bool(self.scripted_path.get("show_labels", False))
        self.scripted_show_future = bool(self.scripted_path.get("show_future", False))
        self.scripted_label = str(self.scripted_path.get("label", "ScriptedPath"))
        self.scripted_line_width = max(1, int(self.scripted_path.get("line_width", 3)))
        self.scripted_marker_radius = max(4, int(self.scripted_path.get("marker_radius", 8)))
        self.scripted_color_raw = str(self.scripted_path.get("color", "#f5c542"))
        self.scripted_waypoints = self.build_scripted_waypoints()
        self.scripted_motion_points = self.scripted_waypoints[:]
        if self.scripted_loop and len(self.scripted_waypoints) > 2:
            self.scripted_motion_points.append(self.scripted_waypoints[0])
        self.scripted_segment_lengths, self.scripted_total_length = self.compute_path_lengths(self.scripted_motion_points)
        self.scripted_enabled = bool(self.scripted_path.get("enabled", False)) and len(self.scripted_waypoints) >= 2

        self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
        pygame.display.set_caption("LY Simulator")
        try:
            driver = pygame.display.get_driver()
            if str(driver).strip().lower() == "offscreen":
                print(
                    "warning: SDL video driver is offscreen; pygame window may be invisible (WSL GUI/X11/Wayland issue).",
                    file=sys.stderr,
                )
        except Exception:
            pass
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("DejaVu Sans", 16)
        self.small_font = pygame.font.SysFont("DejaVu Sans", 13)
        self.title_font = pygame.font.SysFont("DejaVu Sans", 22, bold=True)
        self.mono_font = pygame.font.SysFont("DejaVu Sans Mono", 14)

        self.palette = {
            "bg": color(pygame, self.colors_raw, "bg", "#101216"),
            "panel": color(pygame, self.colors_raw, "panel", "#1b2026"),
            "panel2": color(pygame, self.colors_raw, "panel2", "#252b33"),
            "text": color(pygame, self.colors_raw, "text", "#edf2f7"),
            "muted": color(pygame, self.colors_raw, "muted", "#a8b0ba"),
            "line": color(pygame, self.colors_raw, "line", "#3a424d"),
            "accent": color(pygame, self.colors_raw, "accent", "#f5c542"),
            "friend": color(pygame, self.colors_raw, "friend", "#4cc38a"),
            "enemy": color(pygame, self.colors_raw, "enemy", "#e85d5d"),
            "red": color(pygame, self.colors_raw, "red", "#e85d5d"),
            "blue": color(pygame, self.colors_raw, "blue", "#5d8ee8"),
            "black": color(pygame, self.colors_raw, "black", "#000000"),
            "white": color(pygame, self.colors_raw, "white", "#ffffff"),
            "neutral": color(pygame, self.colors_raw, "neutral", "#4fb3d9"),
        }
        try:
            self.scripted_color = pygame.Color(self.scripted_color_raw)
        except ValueError:
            self.scripted_color = self.palette["accent"]

        self.map_image = pygame.image.load(str(map_path)).convert_alpha()
        self.map_size = self.map_image.get_size()
        self.scaled_map = None
        self.cached_map_key: tuple[int, int] | None = None
        self.map_viewport = self._new_map_viewport()

    def run(self) -> None:
        last = time.perf_counter()
        running = True
        while running:
            now = time.perf_counter()
            dt = now - last
            last = now
            for event in self.pg.event.get():
                if event.type == self.pg.QUIT:
                    running = False
                elif event.type == self.pg.VIDEORESIZE:
                    if self.fullscreen:
                        continue
                    self.resize_workspace(event.w, event.h)
                elif event.type == self.pg.KEYDOWN:
                    self.handle_key(event.key)
                elif event.type == self.pg.MOUSEBUTTONDOWN:
                    self.handle_mouse(event)
                elif event.type == self.pg.MOUSEWHEEL:
                    self.handle_mouse_wheel(event)
                elif event.type == self.pg.MOUSEMOTION:
                    self.handle_mouse_motion(event)
                elif event.type == self.pg.MOUSEBUTTONUP:
                    self.handle_mouse_up(event)

            if self.follow and (now - self.last_control_poll) >= self.control_poll_sec:
                self.poll_control_commands()
                self.last_control_poll = now

            if self.follow and (now - self.last_follow_poll) >= self.follow_poll_sec:
                self.poll_trace_updates()
                self.last_follow_poll = now

            if self.ros_state_path is not None and (now - self.last_ros_monitor_poll) >= self.ros_monitor_poll_sec:
                self.poll_ros_topic_state()
                self.last_ros_monitor_poll = now

            self.tick_match_clock(dt)

            if self.playing and len(self.records) > 1:
                self.current_time += dt * self.playback_speed
                if self.current_time >= self.times[-1]:
                    # In follow mode keep tailing latest records instead of auto-pausing.
                    if self.follow:
                        self.current_time = self.times[-1]
                    else:
                        self.current_time = self.times[-1]
                        self.playing = False
                self.current_index = min(
                    len(self.records) - 1,
                    max(0, bisect.bisect_right(self.times, self.current_time) - 1),
                )

            self.draw()
            if self.streamer is not None:
                self.streamer.update_metadata(self.web_status_metadata())
                self.streamer.publish_surface(self.screen, self.pg)
            self.pg.display.flip()
            self.clock.tick(int(as_dict(self.config.get("window")).get("fps", 60)))

    def handle_key(self, key: int) -> None:
        pg = self.pg
        jump = int(self.timeline_config.get("jump_step", 25))
        if key == pg.K_ESCAPE and self.fullscreen:
            self.toggle_fullscreen()
        elif key in (pg.K_ESCAPE, pg.K_q):
            pg.event.post(pg.event.Event(pg.QUIT))
        elif key == pg.K_F11:
            self.toggle_fullscreen()
        elif self.panel_rect().collidepoint(pg.mouse.get_pos()) and self.handle_panel_scroll_key(key):
            return
        elif key == pg.K_SPACE:
            self.playing = not self.playing
            self.current_time = self.records[self.current_index].t
        elif key in (pg.K_RIGHT, pg.K_PERIOD):
            self.seek_index(self.current_index + 1)
        elif key in (pg.K_LEFT, pg.K_COMMA):
            self.seek_index(self.current_index - 1)
        elif key == pg.K_PAGEUP:
            self.seek_index(self.current_index + jump)
        elif key == pg.K_PAGEDOWN:
            self.seek_index(self.current_index - jump)
        elif key == pg.K_HOME:
            self.seek_index(0)
        elif key == pg.K_END:
            self.seek_index(len(self.records) - 1)
        elif key in (pg.K_EQUALS, pg.K_PLUS, pg.K_KP_PLUS):
            self.playback_speed = min(16.0, self.playback_speed * 1.5)
        elif key in (pg.K_MINUS, pg.K_KP_MINUS):
            self.playback_speed = max(0.1, self.playback_speed / 1.5)
        elif key == pg.K_l:
            self.show_labels = not self.show_labels
        elif key == pg.K_t:
            self.goal_tags_expanded = not self.goal_tags_expanded
        elif key == pg.K_f:
            self.reset_map_view()
        elif key == pg.K_0:
            self.set_map_actual_size()
        elif key in (pg.K_1, pg.K_KP1):
            self.set_panel_tab("decision")
        elif key in (pg.K_2, pg.K_KP2):
            self.set_panel_tab("events")
        elif key in (pg.K_3, pg.K_KP3):
            self.set_panel_tab("runtime")
        elif key in (pg.K_4, pg.K_KP4):
            self.set_panel_tab("control")
        elif key in (pg.K_5, pg.K_KP5):
            self.set_panel_tab("inputs")
        elif key in (pg.K_6, pg.K_KP6):
            self.set_panel_tab("layers")
        elif key == pg.K_s:
            self.send_match_command("start")
        elif key == pg.K_p:
            self.send_match_command("pause")
        elif key == pg.K_r:
            self.send_match_command("reset")
        elif key == pg.K_LEFTBRACKET:
            self.send_match_command("rewind", {"seconds": self.rewind_step_sec})
        elif key == pg.K_RIGHTBRACKET:
            self.send_match_command("forward", {"seconds": self.forward_step_sec})

    def handle_mouse(self, event: Any) -> None:
        if event.button in (4, 5):
            if self.panel_rect().collidepoint(event.pos):
                self.scroll_panel(-48 if event.button == 4 else 48)
            elif self.map_area_rect().collidepoint(event.pos):
                self.adjust_map_zoom(1.15 if event.button == 4 else 1 / 1.15, event.pos)
            return
        if event.button == 2 and self.map_area_rect().collidepoint(event.pos):
            self.map_pan_anchor = event.pos
            return
        if event.button != 1:
            return
        if self.handle_workspace_mouse_down(event.pos):
            return
        if self.handle_inspector_mouse_down(event.pos):
            return
        for tab, rect in self.panel_tab_buttons.items():
            if rect.collidepoint(event.pos):
                self.set_panel_tab(tab)
                return
        for command, rect in self.control_buttons.items():
            if rect.collidepoint(event.pos):
                if command == "rewind":
                    self.send_match_command(command, {"seconds": self.rewind_step_sec})
                elif command == "forward":
                    self.send_match_command(command, {"seconds": self.forward_step_sec})
                else:
                    self.send_match_command(command)
                return
        if self.panel_rect().collidepoint(event.pos) and (
            self.panel_body_rect is None or not self.panel_body_rect.collidepoint(event.pos)
        ):
            return
        for layer_key, rect in self.layer_buttons.items():
            if rect.collidepoint(event.pos):
                self.layers[layer_key] = not bool(self.layers.get(layer_key, True))
                return
        if self.handle_sim_panel_mouse_down(event.pos):
            return
        if self.goal_tag_button_rect is not None and self.goal_tag_button_rect.collidepoint(event.pos):
            self.goal_tags_expanded = not self.goal_tags_expanded
            return
        for command, rect in self.map_tool_buttons.items():
            if rect.collidepoint(event.pos):
                if command == "fit":
                    self.reset_map_view()
                elif command == "actual":
                    self.set_map_actual_size()
                elif command == "focus":
                    self.zoom_to_selection()
                elif command == "fullscreen":
                    self.toggle_fullscreen()
                else:
                    self.adjust_map_zoom(
                        1.15 if command == "zoom_in" else 1 / 1.15,
                        rect.center,
                    )
                return
        if self.handle_sim_map_mouse_down(event.pos):
            return
        track = self.timeline_rect()
        if track.collidepoint(event.pos):
            ratio = (event.pos[0] - track.x) / max(1, track.width)
            self.seek_index(round(ratio * (len(self.records) - 1)))

    def handle_mouse_wheel(self, event: Any) -> None:
        pointer = self.pg.mouse.get_pos()
        if self.panel_rect().collidepoint(pointer):
            delta = -int(getattr(event, "y", 0)) * 48
            if getattr(event, "flipped", False):
                delta = -delta
            self.scroll_panel(delta)
            return
        if self.map_area_rect().collidepoint(pointer):
            direction = int(getattr(event, "y", 0))
            if getattr(event, "flipped", False):
                direction = -direction
            if direction:
                self.adjust_map_zoom(1.15**direction, pointer)

    def handle_panel_scroll_key(self, key: int) -> bool:
        pg = self.pg
        body_height = max(80, self.panel_scroll.body_height(self.panel_tab, 220))
        if key == pg.K_UP:
            self.scroll_panel(-36)
            return True
        if key == pg.K_DOWN:
            self.scroll_panel(36)
            return True
        if key == pg.K_PAGEUP:
            self.scroll_panel(-body_height)
            return True
        if key == pg.K_PAGEDOWN:
            self.scroll_panel(body_height)
            return True
        if key == pg.K_HOME:
            self.panel_scroll.set_offset(self.panel_tab, 0)
            return True
        if key == pg.K_END:
            self.panel_scroll.set_offset(self.panel_tab, self.panel_max_scroll(self.panel_tab))
            return True
        return False

    def set_panel_tab(self, tab: str) -> None:
        self.panel_tab = tab
        if tab != "inputs":
            self.inputs_panel.clear_buttons()
        self.clamp_panel_scroll(tab)

    def panel_max_scroll(self, tab: str | None = None) -> int:
        key = tab or self.panel_tab
        return self.panel_scroll.max_scroll(key)

    def clamp_panel_scroll(self, tab: str | None = None) -> None:
        key = tab or self.panel_tab
        self.panel_scroll.clamp(key)

    def scroll_panel(self, delta: int) -> None:
        self.panel_scroll.scroll(self.panel_tab, delta)

    def handle_mouse_motion(self, event: Any) -> None:
        if self.workspace_splitter == "inspector":
            if self.inspector_zone == "right":
                self.inspector_width = max(280, min(460, self.width - event.pos[0] - 40))
            else:
                self.inspector_width = max(280, min(460, event.pos[0] - 96))
            self._refresh_map_viewport()
            return
        if self.workspace_splitter == "shelf":
            self.shelf_collapsed = False
            self.shelf_height = max(140, min(300, self.height - event.pos[1] - 48))
            self._refresh_map_viewport()
            return
        if self.map_pan_anchor is not None:
            dx, dy = getattr(event, "rel", (0, 0))
            self.map_viewport.pan_by(dx, dy)
            self.map_pan_anchor = event.pos
            return
        if self.dragging_unit is not None:
            self.drag_position = event.pos

    def handle_mouse_up(self, event: Any) -> None:
        if event.button == 1 and self.workspace_splitter is not None:
            self.workspace_splitter = None
            return
        if event.button == 2:
            self.map_pan_anchor = None
            return
        if event.button != 1 or self.dragging_unit is None:
            return
        image_rect = self.map_image_rect()
        field_pos = self.screen_to_field(event.pos, image_rect)
        unit = self.dragging_unit
        self.dragging_unit = None
        self.drag_position = None
        if field_pos is None:
            return
        x, y = field_pos
        payload = self.scene_drag_payload(unit, int(round(x)), int(round(y)))
        if payload is not None:
            self.send_sim_command("place_unit", payload)

    def send_match_command(self, command: str, payload: dict[str, Any] | None = None) -> None:
        if not self.match_control_enabled or self.control_path is None or not self.follow:
            return
        try:
            append_command(self.control_path, command, payload)
            self.apply_local_match_command(command, payload)
            self.last_control_status = f"cmd={command}"
        except OSError:
            self.last_control_status = f"cmd={command} failed"

    def controls_available(self) -> bool:
        return bool(self.match_control_enabled and self.control_path is not None and self.follow)

    def send_sim_command(self, command: str, payload: dict[str, Any]) -> None:
        if not self.controls_available():
            self.last_control_status = "sim input disabled"
            return
        if self.sim_input_state.scene.ownership_mode != "mock":
            self.last_control_status = "manual_ros observer mode"
            return
        try:
            append_command(self.control_path, command, payload)
            self.apply_local_sim_command(command, payload)
            self.last_control_status = f"cmd={command}"
        except OSError:
            self.last_control_status = f"cmd={command} failed"

    def apply_local_match_command(self, command: str, payload: dict[str, Any] | None = None) -> None:
        cmd = str(command).strip().lower()
        body = payload or {}
        if cmd == "start":
            if self.match_time_left_sec <= 0.0:
                self.match_time_left_sec = float(self.match_duration_sec)
            self.match_started = True
            self.match_running = True
            self.playing = True
            return
        if cmd == "pause":
            self.match_running = False
            self.playing = False
            return
        if cmd == "reset":
            self.match_time_left_sec = float(self.match_duration_sec)
            self.match_started = False
            self.match_running = False
            self.playing = False
            self.live_goal_history.clear()
            return
        if cmd in {"rewind", "forward", "set_time_left"}:
            try:
                sec = float(body.get("seconds", 0.0))
            except (TypeError, ValueError):
                return
            if not math.isfinite(sec):
                return
            if cmd == "rewind":
                self.match_time_left_sec += max(0.0, sec)
            elif cmd == "forward":
                self.match_time_left_sec -= max(0.0, sec)
            else:
                self.match_time_left_sec = sec
            self.match_time_left_sec = max(0.0, min(float(self.match_duration_sec), self.match_time_left_sec))
            if self.match_time_left_sec <= 0.0:
                self.match_running = False
            return

    def apply_local_sim_command(self, command: str, payload: dict[str, Any] | None = None) -> None:
        self.sim_input_state.apply_command(command, payload or {})

    def handle_sim_panel_mouse_down(self, pos: tuple[int, int]) -> bool:
        return self.inputs_panel.handle_mouse_down(pos)

    def handle_workspace_mouse_down(self, pos: tuple[int, int]) -> bool:
        for command, rect in self.workspace_buttons.items():
            if not rect.collidepoint(pos):
                continue
            if command == "fit":
                self.reset_map_view()
            elif command == "actual":
                self.set_map_actual_size()
            elif command == "focus":
                self.zoom_to_selection()
            elif command == "fullscreen":
                self.toggle_fullscreen()
            elif command == "reset_layout":
                self.reset_workspace_layout()
            elif command == "toggle_dock":
                self.inspector_zone = "left" if self.inspector_zone == "right" else "right"
                self._refresh_map_viewport()
            elif command == "toggle_inspector":
                self.inspector_collapsed = not self.inspector_collapsed
                self._refresh_map_viewport()
            elif command == "toggle_shelf":
                self.shelf_collapsed = not self.shelf_collapsed
                self._refresh_map_viewport()
            elif command == "inspector_splitter":
                self.workspace_splitter = "inspector"
            elif command == "shelf_splitter":
                self.workspace_splitter = "shelf"
            return True
        for tab, rect in self.activity_buttons.items():
            if rect.collidepoint(pos):
                self.set_panel_tab(tab)
                return True
        return False

    def reset_workspace_layout(self) -> None:
        self.inspector_width = 320
        self.inspector_zone = "right"
        self.inspector_collapsed = False
        self.shelf_height = 180
        self.shelf_collapsed = True
        self._refresh_map_viewport(reset=True)

    def handle_inspector_mouse_down(self, pos: tuple[int, int]) -> bool:
        for _, (rect, payload) in self.inspector_buttons.items():
            if rect.collidepoint(pos):
                command_payload = dict(payload)
                command = str(command_payload.pop("_command", "set_structure_health"))
                self.send_sim_command(command, command_payload)
                return True
        return False

    def select_overview(self) -> None:
        """Select the map-level inspector without changing simulator state."""

        self.workspace_selection = Selection("overview", "")
        self.selected_structure_key = None
        self.sim_input_state.scene.selected_entity_id = None

    def select_unit(self, entity_id: str) -> None:
        """Select a catalog-backed scene robot if it exists."""

        if entity_id not in self.sim_input_state.scene.units:
            return
        self.workspace_selection = Selection("unit", entity_id)
        self.selected_structure_key = None
        self.sim_input_state.scene.selected_entity_id = entity_id

    def select_structure(self, structure_key: str) -> None:
        """Select a configured Base or Outpost if it exists."""

        if not any(item.key == structure_key for item in self.sim_input_state.structures):
            return
        self.workspace_selection = Selection("structure", structure_key)
        self.selected_structure_key = structure_key
        self.sim_input_state.scene.selected_entity_id = None

    def select_area(self, area_key: str) -> None:
        if self.map_area_by_key(area_key) is None:
            return
        self.workspace_selection = Selection("area", area_key)
        self.selected_structure_key = None
        self.sim_input_state.scene.selected_entity_id = None

    def handle_sim_map_mouse_down(self, pos: tuple[int, int]) -> bool:
        image_rect = self.map_image_rect()
        if not image_rect.collidepoint(pos):
            return False
        hit = self.hit_sim_unit(pos, image_rect)
        if hit is not None:
            unit = self.sim_input_state.scene.units.get(hit)
            if unit is None:
                return False
            self.select_unit(unit.entity_id)
            if self.simulator_inputs_enabled and self.sim_input_state.scene.ownership_mode == "mock":
                self.dragging_unit = unit
                self.drag_position = pos
            return True
        structure_key = self.hit_sim_structure(pos, image_rect)
        if structure_key is not None:
            self.select_structure(structure_key)
            return True
        area_key = self.hit_map_area(pos, image_rect)
        if area_key is not None:
            self.select_area(area_key)
            return True
        self.select_overview()
        return True

    def map_area_items(self) -> list[dict[str, Any]]:
        items: list[dict[str, Any]] = []
        for zone in as_list(as_dict(self.config.get("terrain")).get("zones")):
            if isinstance(zone, dict) and str(zone.get("name", "")).strip():
                items.append(zone)
        for item in as_list(as_dict(self.config.get("structures")).get("items")):
            if isinstance(item, dict) and str(item.get("name", "")).strip():
                items.append(item)
        return items

    def map_area_by_key(self, area_key: str) -> dict[str, Any] | None:
        return next((item for item in self.map_area_items() if str(item.get("name", "")) == area_key), None)

    def hit_map_area(self, pos: tuple[int, int], image_rect: Any) -> str | None:
        field_pos = self.screen_to_field(pos, image_rect)
        if field_pos is None:
            return None
        x, y = field_pos
        for item in reversed(self.map_area_items()):
            shape = str(item.get("shape", "polygon"))
            if shape == "circle":
                center = parse_position(item.get("center"))
                radius = self.to_float(item.get("radius"), 0.0)
                if center is not None and radius > 0 and math.hypot(x - center[0], y - center[1]) <= radius:
                    return str(item["name"])
                continue
            points = self.map_area_field_points(item)
            if len(points) >= 3 and self.point_in_polygon((x, y), points):
                return str(item["name"])
        return None

    @staticmethod
    def point_in_polygon(point: tuple[float, float], polygon: list[tuple[float, float]]) -> bool:
        x, y = point
        inside = False
        previous = polygon[-1]
        for current in polygon:
            x1, y1 = current
            x2, y2 = previous
            if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / max(1e-9, y2 - y1) + x1:
                inside = not inside
            previous = current
        return inside

    def map_area_field_points(self, item: dict[str, Any]) -> list[tuple[float, float]]:
        raw_points = as_list(item.get("polygon"))
        if not raw_points and isinstance(item.get("rect"), (list, tuple)) and len(item["rect"]) >= 4:
            x, y, width, height = item["rect"][:4]
            raw_points = [(x, y), (x + width, y), (x + width, y + height), (x, y + height)]
        points: list[tuple[float, float]] = []
        for raw in raw_points:
            point = parse_position(raw)
            if point is not None:
                points.append(point)
        return points

    def scene_drag_payload(self, unit: Any, x: int, y: int) -> dict[str, Any] | None:
        entity_id = getattr(unit, "entity_id", None)
        side = getattr(unit, "side", None)
        unit_key = getattr(unit, "unit_key", None)
        hp = getattr(unit, "hp", None)
        if not isinstance(entity_id, str) or not isinstance(side, str) or not isinstance(unit_key, str):
            type_name = str(getattr(unit, "type_name", "")).strip()
            side = str(getattr(unit, "side", "")).strip().lower()
            try:
                archetype = self.sim_input_state.catalog.unit_by_key(type_name.lower())
            except KeyError:
                return None
            self.next_scene_entity_id += 1
            entity_id = f"{side}:{archetype.key}:pygame{self.next_scene_entity_id}"
            unit_key = archetype.key
            hp = getattr(unit, "hp", archetype.default_hp)
        return {
            "entity_id": entity_id,
            "side": side,
            "unit_key": unit_key,
            "hp": int(hp),
            "x": x,
            "y": y,
        }

    def poll_control_commands(self) -> None:
        if not self.match_control_enabled or self.control_path is None or not self.follow:
            return
        commands, new_offset = read_commands(self.control_path, self.control_read_offset)
        self.control_read_offset = new_offset
        for payload in commands:
            command = command_name(payload)
            if not command:
                continue
            self.apply_local_match_command(command, payload)
            self.apply_local_sim_command(command, payload)
            self.last_control_status = f"cmd={command}"

    def tick_match_clock(self, dt: float) -> None:
        if not self.match_control_enabled or not self.follow:
            return
        if not self.match_started or not self.match_running:
            return
        if self.match_time_left_sec <= 0.0:
            self.match_running = False
            return
        self.match_time_left_sec = max(0.0, self.match_time_left_sec - max(0.0, float(dt)))
        if self.match_time_left_sec <= 0.0:
            self.match_running = False

    def seek_index(self, index: int) -> None:
        self.current_index = max(0, min(len(self.records) - 1, index))
        self.current_time = self.records[self.current_index].t

    def web_status_metadata(self) -> dict[str, Any]:
        record = self.records[self.current_index]
        input_state = self.sim_input_state.snapshot(team=record.team, goals=self.goals)
        current_record = record_status_payload(record)
        goal_position = point_payload(record_position(record, self.goals))
        route_cm = [point_payload(point) for point in self.path_points()]
        return {
            "trace": {
                "name": self.trace_path.name if self.trace_path is not None else "",
                "follow": self.follow,
                "records": len(self.records),
                "bad_lines": self.bad_lines,
                "duration_sec": self.records[-1].t - self.records[0].t if self.records else 0.0,
                "tick_range": {
                    "first": self.records[0].tick if self.records else 0,
                    "last": self.records[-1].tick if self.records else 0,
                },
            },
            "replay": {
                "playing": self.playing,
                "speed": self.playback_speed,
                "current_index": self.current_index,
                "current_record": self.current_index + 1,
                "total_records": len(self.records),
                "panel_tab": self.panel_tab,
                "match_time_left": round(self.match_time_left_sec, 2),
                "match_running": self.match_running,
                "controls_available": self.controls_available(),
            },
            "current_record": current_record,
            "scene": {
                "ownership_mode": input_state.get("ownership_mode", "mock"),
                "selected_entity_id": input_state.get("selected_entity_id"),
                "show_trace_units": not bool(input_state.get("units")),
                "units": input_state.get("units", []),
                "structures": input_state.get("structures", []),
            },
            "decision": {
                "goal": {
                    "id": record.output.goal_id,
                    "name": record.output.goal_name,
                    "position_cm": goal_position,
                },
                "intent": current_record["intent"],
                "route_cm": route_cm,
            },
            "control_output": current_record["control_output"],
            "simulator_inputs": {
                "enabled": self.simulator_inputs_enabled,
                "status": self.last_control_status,
                "state": input_state,
            },
        }

    def poll_trace_updates(self) -> None:
        if self.trace_path is None:
            return
        if not self.trace_path.exists():
            return
        try:
            new_records, bad_lines, new_offset = load_trace_incremental(
                self.trace_path,
                self.goal_names,
                self.follow_offset,
                len(self.records),
            )
        except OSError:
            return
        self.follow_offset = new_offset
        self.bad_lines += bad_lines
        if not new_records:
            return

        at_tail = self.current_index >= (len(self.records) - 1)
        self.records.extend(new_records)
        self.times.extend(record.t for record in new_records)
        self.changes = build_changes(self.records)
        latest_time_left = int(new_records[-1].time_left)
        latest_time_left = max(0, min(self.match_duration_sec, latest_time_left))
        if latest_time_left != self.last_trace_time_left:
            self.match_time_left_sec = float(latest_time_left)
            self.last_trace_time_left = latest_time_left
            if latest_time_left < self.match_duration_sec:
                self.match_started = True
            if latest_time_left <= 0:
                self.match_running = False

        if at_tail:
            self.current_index = len(self.records) - 1
            self.current_time = self.records[self.current_index].t

    def poll_ros_topic_state(self) -> None:
        if self.ros_state_path is None or not self.ros_state_path.exists():
            return
        try:
            with self.ros_state_path.open("r", encoding="utf-8") as stream:
                payload = json.load(stream)
        except (OSError, json.JSONDecodeError):
            return
        topics = as_dict(payload.get("topics"))
        self.live_ros_topics = topics
        self.live_ros_wall_time = self.to_float(payload.get("wall_time"), 0.0)
        self.update_live_goal_history()

    def workspace_layout(self) -> WorkspaceLayout:
        return WorkspaceLayout.desktop(
            self.width,
            self.height,
            inspector_width=getattr(self, "inspector_width", self.panel_w),
            inspector_zone=getattr(self, "inspector_zone", "right"),
            inspector_collapsed=getattr(self, "inspector_collapsed", False),
            shelf_height=getattr(self, "shelf_height", self.timeline_h),
            shelf_collapsed=getattr(self, "shelf_collapsed", True),
        )

    def resize_workspace(self, width: int, height: int) -> None:
        self.width = max(self.min_width, int(width))
        self.height = max(self.min_height, int(height))
        self.windowed_size = (self.width, self.height)
        self.screen = self.pg.display.set_mode((self.width, self.height), self.pg.RESIZABLE)
        self.cached_map_key = None
        self._refresh_map_viewport()

    def map_area_rect(self) -> Any:
        rect = self.workspace_layout().viewport
        return self.pg.Rect(round(rect.x), round(rect.y), round(rect.width), round(rect.height))

    def map_content_rect(self) -> Any:
        return self.map_area_rect().inflate(-18, -18)

    def _new_map_viewport(self) -> ViewportState:
        field = self.field_geometry()
        content = self.map_content_rect()
        return ViewportState.fit(
            field_width=field.width,
            field_height=field.height,
            viewport=WorkspaceRect(content.x, content.y, content.width, content.height),
        )

    def _refresh_map_viewport(self, *, reset: bool = False) -> None:
        field = self.field_geometry()
        content = self.map_content_rect()
        viewport = WorkspaceRect(content.x, content.y, content.width, content.height)
        if (
            self.map_viewport.field_width != field.width
            or self.map_viewport.field_height != field.height
        ):
            self.map_viewport = ViewportState.fit(
                field_width=field.width,
                field_height=field.height,
                viewport=viewport,
            )
            return
        self.map_viewport.set_viewport(viewport)
        if reset:
            self.map_viewport.reset_to_fit()

    def map_image_rect(self) -> Any:
        self._refresh_map_viewport()
        top_left = self.map_viewport.field_to_screen(0.0, self.map_viewport.field_height)
        bottom_right = self.map_viewport.field_to_screen(
            self.map_viewport.field_width,
            0.0,
        )
        return self.pg.Rect(
            round(top_left[0]),
            round(top_left[1]),
            max(1, round(bottom_right[0] - top_left[0])),
            max(1, round(bottom_right[1] - top_left[1])),
        )

    def reset_map_view(self) -> None:
        self._refresh_map_viewport(reset=True)

    def set_map_actual_size(self) -> None:
        self._refresh_map_viewport()
        self.map_viewport.actual_size(self.map_size[0])

    def adjust_map_zoom(self, factor: float, anchor: tuple[int, int] | None = None) -> None:
        self._refresh_map_viewport()
        point = anchor or tuple(round(value) for value in self.map_viewport.viewport.center)
        self.map_viewport.zoom_at(point[0], point[1], factor=factor)

    def zoom_to_selection(self) -> None:
        selection = getattr(self, "workspace_selection", Selection())
        bounds: WorkspaceRect | None = None
        if selection.kind == "unit":
            unit = self.sim_input_state.scene.units.get(selection.key)
            if unit is not None:
                bounds = WorkspaceRect(unit.x - 140, unit.y - 140, 280, 280)
        elif selection.kind == "structure":
            structure = self.selected_structure()
            if structure is not None:
                position = self.sim_structure_position(structure)
                if position is not None:
                    bounds = WorkspaceRect(position[0] - 180, position[1] - 180, 360, 360)
        elif selection.kind == "area":
            item = self.map_area_by_key(selection.key)
            if item is not None and str(item.get("shape", "")) == "circle":
                center = parse_position(item.get("center"))
                radius = max(1.0, self.to_float(item.get("radius"), 120.0))
                if center is not None:
                    bounds = WorkspaceRect(center[0] - radius, center[1] - radius, radius * 2, radius * 2)
            elif item is not None:
                points = self.map_area_field_points(item)
                if points:
                    xs, ys = zip(*points)
                    bounds = WorkspaceRect(min(xs), min(ys), max(1.0, max(xs) - min(xs)), max(1.0, max(ys) - min(ys)))
        if bounds is None:
            self.reset_map_view()
            return
        self._refresh_map_viewport()
        self.map_viewport.zoom_to_field_bounds(bounds)

    @property
    def map_zoom(self) -> float:
        content = self.map_content_rect()
        fit_scale = min(
            content.width / self.map_viewport.field_width,
            content.height / self.map_viewport.field_height,
        )
        return self.map_viewport.scale / max(0.0001, fit_scale)

    def toggle_fullscreen(self) -> None:
        if self.fullscreen:
            self.screen = self.pg.display.set_mode(self.windowed_size, self.pg.RESIZABLE)
            self.fullscreen = False
        else:
            self.windowed_size = (self.width, self.height)
            self.screen = self.pg.display.set_mode((0, 0), self.pg.FULLSCREEN)
            self.fullscreen = True
        self.width, self.height = self.screen.get_size()
        self.scaled_map = None
        self.cached_map_key = None
        self._refresh_map_viewport(reset=True)

    def panel_rect(self) -> Any:
        rect = self.workspace_layout().inspector.rect
        return self.pg.Rect(round(rect.x), round(rect.y), round(rect.width), round(rect.height))

    def operations_shelf_rect(self) -> Any:
        rect = self.workspace_layout().operations_shelf.rect
        return self.pg.Rect(round(rect.x), round(rect.y), round(rect.width), round(rect.height))

    def timeline_rect(self) -> Any:
        shelf = self.operations_shelf_rect()
        return self.pg.Rect(shelf.x + 20, shelf.bottom - 28, max(1, shelf.width - 40), 12)

    def draw(self) -> None:
        self.screen.fill(self.palette["bg"])
        layout = self.workspace_layout()
        self.draw_command_bar(layout.command_bar)
        self.draw_activity_rail(layout.activity_rail)
        self.draw_battlefield_viewport(layout.viewport)
        self.draw_contextual_inspector(layout.inspector.rect)
        self.draw_operations_shelf(layout.operations_shelf.rect)
        self.draw_drag_preview()

    def draw_command_bar(self, rect: WorkspaceRect) -> None:
        pg = self.pg
        bar = pg.Rect(round(rect.x), round(rect.y), round(rect.width), round(rect.height))
        self.draw_flight_deck_card(bar)
        self.workspace_buttons = {}
        self.control_buttons = {}
        title = self.title_font.render("SENTINEL FLIGHT DECK", True, self.palette["text"])
        self.screen.blit(title, (bar.x + 18, bar.y + 15))
        record = self.records[self.current_index]
        state = "LIVE" if self.follow else "TRACE"
        summary = self.small_font.render(
            f"{state}  ·  {record.strategy}  ·  {record.output.goal_name or 'No active goal'}",
            True,
            self.palette["muted"],
        )
        self.screen.blit(summary, (bar.x + 332, bar.y + 20))

        controls = [("fit", "Fit"), ("actual", "1:1"), ("focus", "Focus"), ("fullscreen", "Full"), ("reset_layout", "Reset")]
        if self.controls_available():
            controls = [("start", "Start"), ("pause", "Pause")] + controls
        button_w = 56
        button_h = 40
        gap = 8
        x = bar.right - 14 - len(controls) * button_w - (len(controls) - 1) * gap
        for command, label in controls:
            button = pg.Rect(x, bar.y + 8, button_w, button_h)
            self.draw_control_button(button, label)
            if command in {"start", "pause"}:
                self.control_buttons[command] = button
            else:
                self.workspace_buttons[command] = button
            x += button_w + gap

    def draw_activity_rail(self, rect: WorkspaceRect) -> None:
        pg = self.pg
        rail = pg.Rect(round(rect.x), round(rect.y), round(rect.width), round(rect.height))
        self.draw_flight_deck_card(rail)
        self.activity_buttons = {}
        y = rail.y + 14
        for tab, icon, _label in self.activity_rail_items():
            button = pg.Rect(rail.x + 8, y, rail.width - 16, 44)
            active = tab == self.panel_tab
            fill = self.palette["accent"] if active else self.palette["panel2"]
            text_color = self.palette["black"] if active else self.palette["text"]
            pg.draw.rect(self.screen, fill, button, border_radius=7)
            pg.draw.rect(self.screen, self.palette["line"], button, 1, border_radius=7)
            glyph = self.font.render(icon, True, text_color)
            self.screen.blit(glyph, glyph.get_rect(center=button.center))
            self.activity_buttons[tab] = button
            y += 54

    @staticmethod
    def activity_rail_items() -> tuple[tuple[str, str, str], ...]:
        """Keep the native rail compact while retaining the browser surface names."""

        return (
            ("decision", "◎", "Decision"),
            ("events", "◌", "Events"),
            ("runtime", "⌁", "Runtime"),
            ("control", "⌘", "Control"),
            ("inputs", "◈", "Inputs"),
            ("layers", "▤", "Layers"),
        )

    def draw_battlefield_viewport(self, rect: WorkspaceRect) -> None:
        del rect
        self.draw_map()

    def draw_contextual_inspector(self, rect: WorkspaceRect) -> None:
        del rect
        self.draw_panel()

    def draw_operations_shelf(self, rect: WorkspaceRect) -> None:
        pg = self.pg
        shelf = pg.Rect(round(rect.x), round(rect.y), round(rect.width), round(rect.height))
        self.draw_flight_deck_card(shelf)
        title = self.font.render("Operations shelf", True, self.palette["text"])
        self.screen.blit(title, (shelf.x + 16, shelf.y + 11))
        state = "Expand" if self.shelf_collapsed else "Collapse"
        toggle = pg.Rect(shelf.right - 96, shelf.y + 7, 80, 28)
        self.draw_control_button(toggle, state)
        self.workspace_buttons["toggle_shelf"] = toggle
        if self.shelf_collapsed:
            record = self.records[self.current_index]
            text = self.small_font.render(
                f"{record.t:.1f}s  ·  {record.event}  ·  {record.output.goal_name or 'No goal'}",
                True,
                self.palette["muted"],
            )
            self.screen.blit(text, (shelf.x + 16 + title.get_width() + 18, shelf.y + 13))
            return
        splitter = pg.Rect(shelf.x + 120, shelf.y, max(1, shelf.width - 240), 5)
        pg.draw.rect(self.screen, self.palette["line"], splitter, border_radius=2)
        self.workspace_buttons["shelf_splitter"] = splitter.inflate(0, 8)
        self.draw_timeline()

    def draw_map(self) -> None:
        pg = self.pg
        area = self.map_area_rect()
        self.draw_flight_deck_card(area)
        image_rect = self.map_image_rect()
        rect_key = (image_rect.width, image_rect.height)
        if self.scaled_map is None or self.cached_map_key != rect_key:
            self.scaled_map = pg.transform.smoothscale(self.map_image, (image_rect.width, image_rect.height))
            self.cached_map_key = rect_key
        previous_clip = self.screen.get_clip()
        self.screen.set_clip(area)
        self.screen.blit(self.scaled_map, image_rect)
        pg.draw.rect(self.screen, self.palette["line"], image_rect, 1, border_radius=10)

        if self.layers.get("terrain", True):
            self.draw_terrain(image_rect)
        if self.layers.get("structures", True):
            self.draw_structures(image_rect)
        if self.layers.get("simulator_inputs", True):
            self.draw_sim_structure_badges(image_rect)
        if self.layers.get("grid", True):
            self.draw_grid(image_rect)
        if self.layers.get("all_goals", True):
            self.draw_all_goals(image_rect)
        if self.layers.get("goal_path", True):
            self.draw_path(image_rect)
        if self.layers.get("scripted_path", True):
            self.draw_scripted_path(image_rect)
        if self.layers.get("units", True):
            if self.show_trace_units():
                self.draw_units(image_rect)
            self.draw_sim_units(image_rect)
        if self.layers.get("current_goal", True):
            self.draw_current_goal(image_rect)
        self.screen.set_clip(previous_clip)
        self.draw_map_tools(area)

    def draw_map_tools(self, area: Any) -> None:
        self.map_tool_buttons = {}
        labels = (("fit", "Fit"), ("actual", "1:1"), ("zoom_out", "-"), ("zoom_in", "+"), ("focus", "Focus"), ("fullscreen", "Full"))
        x = area.x + 12
        y = area.y + 12
        for command, label in labels:
            width = 44 if len(label) > 1 else 30
            rect = self.pg.Rect(x, y, width, 26)
            self.draw_control_button(rect, label)
            self.map_tool_buttons[command] = rect
            x += width + 6
        zoom_label = self.small_font.render(f"{self.map_zoom:.2f}x", True, self.palette["text"])
        self.screen.blit(zoom_label, (x + 4, y + 5))

    def field_geometry(self) -> FieldGeometry:
        record = self.records[self.current_index]
        field = record.field.as_config() or as_dict(self.config.get("field_cm"))
        return FieldGeometry.from_config(field)

    def field_size(self) -> tuple[int, int]:
        field = self.field_geometry()
        return (field.width, field.height)

    def field_to_screen(self, pos: tuple[float, float], image_rect: Any) -> tuple[int, int]:
        del image_rect
        self._refresh_map_viewport()
        x, y = self.field_geometry().clamp_point(pos)
        sx, sy = self.map_viewport.field_to_screen(x, y)
        return (round(sx), round(sy))

    def screen_to_field(self, pos: tuple[int, int], image_rect: Any) -> tuple[float, float] | None:
        if not image_rect.collidepoint(pos):
            return None
        self._refresh_map_viewport()
        return self.field_geometry().clamp_point(self.map_viewport.screen_to_field(*pos))

    @staticmethod
    def to_float(value: Any, default: float = 0.0) -> float:
        try:
            out = float(value)
        except (TypeError, ValueError):
            return default
        return out if math.isfinite(out) else default

    @staticmethod
    def normalize_side(side: str, fallback: str) -> str:
        text = str(side).strip().lower()
        if text == "auto":
            text = fallback
        return text if text in ("red", "blue") else fallback

    @staticmethod
    def compute_path_lengths(points: list[tuple[float, float]]) -> tuple[list[float], float]:
        if len(points) < 2:
            return [], 0.0
        lengths: list[float] = []
        total = 0.0
        for idx in range(len(points) - 1):
            a = points[idx]
            b = points[idx + 1]
            seg = math.hypot(b[0] - a[0], b[1] - a[1])
            lengths.append(seg)
            total += seg
        return lengths, total

    def build_scripted_waypoints(self) -> list[tuple[float, float]]:
        points: list[tuple[float, float]] = []
        raw_points = as_list(self.scripted_path.get("points_cm"))
        for raw in raw_points:
            pos = parse_position(raw)
            if pos is not None:
                points.append(pos)
        if points:
            return points

        for raw_id in as_list(self.scripted_path.get("goal_ids")):
            try:
                goal_id = int(raw_id)
            except (TypeError, ValueError):
                continue
            goal = self.goals.get(goal_id)
            if not isinstance(goal, dict):
                continue
            pos = parse_position(goal.get(self.scripted_side))
            if pos is None or pos == (0, 0):
                other_side = "blue" if self.scripted_side == "red" else "red"
                pos = parse_position(goal.get(other_side))
            if pos is None or pos == (0, 0):
                continue
            points.append(pos)
        return points

    def scripted_elapsed_sec(self) -> float:
        controls_available = bool(self.match_control_enabled and self.follow)
        if controls_available:
            if not self.match_started:
                return 0.0
            return max(0.0, float(self.match_duration_sec) - float(self.match_time_left_sec))
        return max(0.0, float(self.records[self.current_index].t - self.records[0].t))

    def scripted_motion_state(self) -> tuple[tuple[float, float], int] | None:
        if not self.scripted_enabled or not self.scripted_waypoints:
            return None
        if len(self.scripted_waypoints) == 1:
            return (self.scripted_waypoints[0], 0)
        if self.scripted_total_length <= 0.0:
            return (self.scripted_waypoints[0], 0)

        distance = self.scripted_elapsed_sec() * self.scripted_speed_cmps
        if not math.isfinite(distance) or distance < 0.0:
            distance = 0.0
        if self.scripted_loop:
            distance = distance % self.scripted_total_length
        else:
            distance = min(distance, self.scripted_total_length)

        for idx, seg in enumerate(self.scripted_segment_lengths):
            start = self.scripted_motion_points[idx]
            end = self.scripted_motion_points[idx + 1]
            if seg <= 1e-6:
                if distance <= seg:
                    return (start, idx)
                continue
            if distance <= seg or idx == len(self.scripted_segment_lengths) - 1:
                ratio = max(0.0, min(1.0, distance / seg))
                return (
                    (start[0] + (end[0] - start[0]) * ratio, start[1] + (end[1] - start[1]) * ratio),
                    idx,
                )
            distance -= seg
        if self.scripted_motion_points:
            return (self.scripted_motion_points[-1], max(0, len(self.scripted_segment_lengths) - 1))
        return (self.scripted_waypoints[-1], 0)

    def scripted_marker_position(self) -> tuple[float, float] | None:
        state = self.scripted_motion_state()
        return None if state is None else state[0]

    def draw_scripted_path(self, image_rect: Any) -> None:
        if not self.scripted_enabled:
            return
        pg = self.pg
        motion_state = self.scripted_motion_state()
        if motion_state is None:
            return
        marker, active_segment_idx = motion_state
        if self.scripted_show_future:
            visible_points = self.scripted_waypoints
        else:
            visible_points = self.scripted_motion_points[: active_segment_idx + 2]
            if not visible_points:
                visible_points = self.scripted_waypoints[:1]
        path_points = [self.field_to_screen(pos, image_rect) for pos in visible_points]
        if len(path_points) >= 2:
            pg.draw.lines(
                self.screen,
                self.scripted_color,
                bool(self.scripted_show_future and self.scripted_loop and len(path_points) > 2),
                path_points,
                self.scripted_line_width,
            )
        for idx, point in enumerate(path_points):
            pg.draw.circle(self.screen, self.scripted_color, point, 4)
            if self.scripted_show_labels or self.show_labels:
                self.draw_label(f"P{idx}", point[0] + 6, point[1] - 10, image_rect)

        sx, sy = self.field_to_screen(marker, image_rect)
        pg.draw.circle(self.screen, self.palette["black"], (sx, sy), self.scripted_marker_radius + 3)
        pg.draw.circle(self.screen, self.scripted_color, (sx, sy), self.scripted_marker_radius)
        elapsed = int(round(self.scripted_elapsed_sec()))
        start_idx = active_segment_idx % max(1, len(self.scripted_waypoints))
        target_idx = (active_segment_idx + 1) % max(1, len(self.scripted_waypoints))
        self.draw_label(
            f"{self.scripted_label} P{start_idx}->P{target_idx} v={self.scripted_speed_cmps:.0f}cm/s t={elapsed}s",
            sx + self.scripted_marker_radius + 6,
            sy - 24,
            image_rect,
        )

    def draw_grid(self, image_rect: Any) -> None:
        pg = self.pg
        field_w, field_h = self.field_size()
        overlay = pg.Surface((image_rect.width, image_rect.height), pg.SRCALPHA)
        grid_color = pg.Color(255, 255, 255, 34)
        for x_cm in range(0, field_w + 1, 400):
            x = round(x_cm / field_w * image_rect.width)
            pg.draw.line(overlay, grid_color, (x, 0), (x, image_rect.height), 1)
        for y_cm in range(0, field_h + 1, 300):
            y = round((1.0 - y_cm / field_h) * image_rect.height)
            pg.draw.line(overlay, grid_color, (0, y), (image_rect.width, y), 1)
        self.screen.blit(overlay, image_rect.topleft)

    def draw_terrain(self, image_rect: Any) -> None:
        terrain = as_dict(self.config.get("terrain"))
        if not terrain.get("enabled", True):
            return
        levels = as_dict(terrain.get("levels"))
        overlay = self.pg.Surface((image_rect.width, image_rect.height), self.pg.SRCALPHA)
        labels: list[tuple[str, int, int]] = []
        for zone in as_list(terrain.get("zones")):
            if not isinstance(zone, dict):
                continue
            points = self.terrain_zone_points(zone, image_rect)
            if len(points) < 3:
                continue
            level = as_dict(levels.get(str(zone.get("level", "ground"))))
            fill = self.pg.Color(level.get("color", "#4fb3d9"))
            fill.a = int(level.get("alpha", 46))
            outline = self.pg.Color(level.get("outline", level.get("color", "#4fb3d9")))
            outline.a = int(level.get("outline_alpha", 110))
            self.pg.draw.polygon(overlay, fill, points)
            self.pg.draw.lines(overlay, outline, True, points, 2)
            if terrain.get("show_labels", False) or self.show_labels:
                cx = round(sum(point[0] for point in points) / len(points)) + image_rect.x
                cy = round(sum(point[1] for point in points) / len(points)) + image_rect.y
                label = f"{zone.get('name', 'terrain')} {level.get('label', zone.get('level', ''))}"
                labels.append((str(label), cx + 4, cy - 10))
        self.screen.blit(overlay, image_rect.topleft)
        for label, x, y in labels:
            self.draw_label(label, x, y, image_rect)

    def draw_structures(self, image_rect: Any) -> None:
        structures = as_dict(self.config.get("structures"))
        if not structures.get("enabled", True):
            return

        styles = as_dict(structures.get("styles"))
        show_labels = bool(structures.get("show_labels", False) or self.goal_tags_expanded)
        mouse_pos = self.pg.mouse.get_pos()
        overlay = self.pg.Surface((image_rect.width, image_rect.height), self.pg.SRCALPHA)
        labels: list[tuple[str, int, int]] = []

        for item in as_list(structures.get("items")):
            if not isinstance(item, dict):
                continue
            kind = str(item.get("kind", "wall"))
            style = as_dict(styles.get(kind))
            fill = self.pg.Color(style.get("fill", "#708090"))
            fill.a = int(style.get("alpha", 44))
            outline = self.pg.Color(style.get("outline", style.get("fill", "#708090")))
            outline.a = int(style.get("outline_alpha", 130))
            label_name = str(item.get("name", kind))
            shape = str(item.get("shape", "polygon"))

            if shape == "circle":
                center = parse_position(item.get("center"))
                if center is None:
                    continue
                radius_cm = float(item.get("radius", 0.0))
                if not math.isfinite(radius_cm) or radius_cm <= 0.0:
                    continue
                sx, sy = self.field_to_screen(center, image_rect)
                field_w, field_h = self.field_size()
                px_per_cm_x = image_rect.width / max(1.0, float(field_w))
                px_per_cm_y = image_rect.height / max(1.0, float(field_h))
                radius_px = max(2, int(round(radius_cm * min(px_per_cm_x, px_per_cm_y))))
                local_center = (sx - image_rect.x, sy - image_rect.y)
                self.pg.draw.circle(overlay, fill, local_center, radius_px)
                self.pg.draw.circle(overlay, outline, local_center, radius_px, 2)
                hovered = (
                    self.hover_goal_tags
                    and math.hypot(mouse_pos[0] - sx, mouse_pos[1] - sy)
                    <= max(radius_px, self.goal_tag_hover_radius_px)
                )
                if show_labels or hovered:
                    labels.append((label_name, sx + radius_px + 6, sy - 8))
                continue

            points = self.structure_points(item, image_rect)
            if shape == "polyline":
                if len(points) < 2:
                    continue
                field_w, field_h = self.field_size()
                px_per_cm_x = image_rect.width / max(1.0, float(field_w))
                px_per_cm_y = image_rect.height / max(1.0, float(field_h))
                width_cm = float(item.get("width_cm", 5.0))
                if not math.isfinite(width_cm) or width_cm <= 0.0:
                    width_cm = 5.0
                width_px = max(1, int(round(width_cm * min(px_per_cm_x, px_per_cm_y))))
                self.pg.draw.lines(overlay, outline, False, points, width_px)
                for point in points:
                    self.pg.draw.circle(overlay, fill, point, max(2, width_px + 1))
                screen_points = [(point[0] + image_rect.x, point[1] + image_rect.y) for point in points]
                xs = [point[0] for point in screen_points]
                ys = [point[1] for point in screen_points]
                bounds = self.pg.Rect(min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys))
                hovered = self.hover_goal_tags and bounds.inflate(
                    self.goal_tag_hover_radius_px,
                    self.goal_tag_hover_radius_px).collidepoint(mouse_pos)
                if show_labels or hovered:
                    cx = round(sum(point[0] for point in points) / len(points)) + image_rect.x
                    cy = round(sum(point[1] for point in points) / len(points)) + image_rect.y
                    labels.append((label_name, cx + 4, cy - 10))
                continue
            if len(points) < 3:
                continue
            self.pg.draw.polygon(overlay, fill, points)
            self.pg.draw.lines(overlay, outline, True, points, 2)
            screen_points = [(point[0] + image_rect.x, point[1] + image_rect.y) for point in points]
            xs = [point[0] for point in screen_points]
            ys = [point[1] for point in screen_points]
            bounds = self.pg.Rect(min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys))
            hovered = self.hover_goal_tags and bounds.inflate(self.goal_tag_hover_radius_px, self.goal_tag_hover_radius_px).collidepoint(mouse_pos)
            if show_labels or hovered:
                cx = round(sum(point[0] for point in points) / len(points)) + image_rect.x
                cy = round(sum(point[1] for point in points) / len(points)) + image_rect.y
                labels.append((label_name, cx + 4, cy - 10))

        self.screen.blit(overlay, image_rect.topleft)
        for label, x, y in labels:
            self.draw_label(label, x, y, image_rect)

    def structure_points(self, item: dict[str, Any], image_rect: Any) -> list[tuple[int, int]]:
        raw_points = as_list(item.get("polygon"))
        if not raw_points:
            raw_points = as_list(item.get("polyline"))
        if not raw_points and isinstance(item.get("rect"), (list, tuple)) and len(item["rect"]) >= 4:
            x, y, w, h = item["rect"][:4]
            raw_points = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]

        points: list[tuple[int, int]] = []
        for raw in raw_points:
            pos = parse_position(raw)
            if pos is None:
                continue
            sx, sy = self.field_to_screen(pos, image_rect)
            points.append((sx - image_rect.x, sy - image_rect.y))
        return points

    def terrain_zone_points(self, zone: dict[str, Any], image_rect: Any) -> list[tuple[int, int]]:
        raw_points = as_list(zone.get("polygon"))
        if not raw_points and isinstance(zone.get("rect"), (list, tuple)) and len(zone["rect"]) >= 4:
            x, y, w, h = zone["rect"][:4]
            raw_points = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
        points: list[tuple[int, int]] = []
        for raw in raw_points:
            pos = parse_position(raw)
            if pos is None:
                continue
            sx, sy = self.field_to_screen(pos, image_rect)
            points.append((sx - image_rect.x, sy - image_rect.y))
        return points

    def draw_all_goals(self, image_rect: Any) -> None:
        pg = self.pg
        mouse_pos = pg.mouse.get_pos()
        for goal_id, goal in self.goals.items():
            for side in ("red", "blue"):
                pos = goal.get(side)
                if pos is None or pos == (0, 0):
                    continue
                sx, sy = self.field_to_screen(pos, image_rect)
                pg.draw.circle(self.screen, self.palette["black"], (sx, sy), 5)
                pg.draw.circle(self.screen, self.palette[side], (sx, sy), 4)
                hovered = self.hover_goal_tags and math.hypot(mouse_pos[0] - sx, mouse_pos[1] - sy) <= self.goal_tag_hover_radius_px
                if self.goal_tags_expanded or hovered:
                    suffix = f" {side}" if self.goal_tags_expanded else f" {side} {pos[0]:.0f},{pos[1]:.0f}"
                    self.draw_label(f"{goal_id}:{goal.get('name', '')}{suffix}", sx + 6, sy - 10, image_rect)

    def path_points(self) -> list[tuple[float, float]]:
        live_goal_history = getattr(self, "live_goal_history", [])
        if live_goal_history:
            return live_goal_history
        points: list[tuple[float, float]] = []
        last_goal: tuple[int, str, tuple[float, float] | None] | None = None
        timeline_config = getattr(self, "timeline_config", {})
        limit = int(timeline_config.get("path_history_limit", 500))
        for record in self.records[max(0, self.current_index - limit) : self.current_index + 1]:
            key = record.output.route_key
            if key == last_goal:
                continue
            pos = record_position(record, self.goals)
            if pos is None:
                continue
            points.append(pos)
            last_goal = key
        return points

    def draw_path(self, image_rect: Any) -> None:
        pg = self.pg
        points = [self.field_to_screen(pos, image_rect) for pos in self.path_points()]
        if len(points) < 2:
            return
        pg.draw.lines(self.screen, self.palette["accent"], False, points, 3)
        for point in points[:-1]:
            pg.draw.circle(self.screen, self.palette["accent"], point, 4)

    def draw_current_goal(self, image_rect: Any) -> None:
        pg = self.pg
        record = self.records[self.current_index]
        live_topic = "/goal_pose"
        live_pos = self.live_goal_position(live_topic)
        if live_pos is None:
            live_topic = "/ly/navi/goal_pos"
            live_pos = self.live_goal_position(live_topic)
        pos = live_pos if live_pos is not None else record_position(record, self.goals)
        if pos is None:
            return
        sx, sy = self.field_to_screen(pos, image_rect)
        side_color = self.palette[record.output.goal_side] if record.output.goal_side in ("red", "blue") else self.palette["accent"]
        pulse = 2 + int((time.perf_counter() * 4) % 4)
        pg.draw.circle(self.screen, self.palette["black"], (sx, sy), 15 + pulse)
        pg.draw.circle(self.screen, side_color, (sx, sy), 12 + pulse)
        pg.draw.circle(self.screen, self.palette["white"], (sx, sy), 5)
        if live_pos is not None:
            label = f"LIVE {live_topic} {pos[0]:.0f},{pos[1]:.0f}"
        else:
            label = f"TRACE {record.output.goal_name} id={record.output.goal_id}"
        self.draw_label(label, sx + 14, sy - 30, image_rect)

    def draw_units(self, image_rect: Any) -> None:
        for unit in self.records[self.current_index].units:
            self.draw_unit(unit, image_rect)

    def show_trace_units(self) -> bool:
        if self.show_trace_units_with_scene:
            return True
        return not bool(self.sim_input_state.scene.units)

    def absolute_field_side(self, relative_side: str) -> str:
        team = self.records[self.current_index].team
        if team not in ("red", "blue"):
            team = "red"
        if relative_side == "friend":
            return team
        if relative_side == "enemy":
            return "blue" if team == "red" else "red"
        return team

    def unit_field_side(self, side: str) -> str:
        text = str(side).strip().lower()
        if text in ("red", "blue"):
            return text
        return self.absolute_field_side(text)

    def sim_structure_position(self, item: Any) -> tuple[float, float] | None:
        team = self.records[self.current_index].team
        return self.sim_input_state.structure_position(item, team, self.goals)

    def draw_sim_structure_badges(self, image_rect: Any) -> None:
        if not self.simulator_inputs_enabled:
            return
        pg = self.pg
        for item in self.sim_input_state.structures:
            pos = self.sim_structure_position(item)
            if pos is None:
                continue
            sx, sy = self.field_to_screen(pos, image_rect)
            hp = int(self.sim_input_state.structure_health.get(item.key, item.hp))
            max_hp = max(1, int(item.max_hp))
            ratio = max(0.0, min(1.0, hp / max_hp))
            side_color = self.palette[self.absolute_field_side(item.side)]
            radius = 10 if item.structure == "outpost" else 12
            selected = item.key == self.selected_structure_key
            if selected:
                pg.draw.circle(self.screen, self.palette["accent"], (sx, sy), radius + 9, 2)
            pg.draw.circle(self.screen, self.palette["black"], (sx, sy), radius + 5)
            pg.draw.circle(self.screen, side_color, (sx, sy), radius + 2)
            pg.draw.circle(self.screen, self.palette["panel"], (sx, sy), max(2, radius - 4))
            self.draw_health_bar(sx - 22, sy + radius + 5, 44, 5, ratio)
            if self.show_labels or self.panel_tab == "inputs" or selected:
                label = f"{item.label} {hp}/{max_hp}"
                self.draw_label(label, sx + radius + 8, sy - 12, image_rect)

    def draw_sim_units(self, image_rect: Any) -> None:
        if not self.simulator_inputs_enabled:
            return
        for unit, archetype in self.sim_scene_units():
            self.draw_sim_unit(unit, archetype, image_rect)

    def sim_scene_units(self) -> list[tuple[Any, Any]]:
        items: list[tuple[Any, Any]] = []
        for unit in self.sim_input_state.scene.units.values():
            try:
                archetype = self.sim_input_state.catalog.unit_by_key(unit.unit_key)
            except KeyError:
                continue
            items.append((unit, archetype))
        return sorted(items, key=lambda item: item[0].entity_id)

    def draw_sim_unit(self, unit: Any, archetype: Any, image_rect: Any) -> None:
        pg = self.pg
        x = unit.x
        y = unit.y
        sx, sy = self.field_to_screen((x, y), image_rect)
        type_name = archetype.label
        type_styles = as_dict(self.unit_styles.get("types"))
        style = as_dict(type_styles.get(type_name, type_styles.get("default", {})))
        side = unit.side
        side_style = as_dict(self.unit_styles.get(side, {}))
        radius = int(style.get("radius", 7)) + 2
        label = str(style.get("label", type_name[:1] or "?"))
        field_side = self.unit_field_side(side)
        marker_extent = max(radius, self.unit_assets.unit_size_px // 2)
        selected = self.sim_input_state.scene.selected_entity_id == unit.entity_id
        if not self.draw_unit_art(sx, sy, field_side, type_name, self.unit_assets.unit_size_px, selected=selected):
            fill = pg.Color(side_style.get("color", self.colors_raw.get(side, "#4fb3d9")))
            outline = pg.Color(side_style.get("outline", "#000000"))
            pg.draw.circle(self.screen, self.palette["white"], (sx, sy), radius + 5)
            pg.draw.circle(self.screen, outline, (sx, sy), radius + 3)
            pg.draw.circle(self.screen, fill, (sx, sy), radius + 1)
            text = self.small_font.render(label, True, self.palette["black"])
            self.screen.blit(text, text.get_rect(center=(sx, sy)))
            marker_extent = radius
        max_hp = max(1, int(archetype.max_hp))
        hp = max(0, min(max_hp, int(unit.hp)))
        self.draw_health_bar(sx - 22, sy + marker_extent + 6, 44, 5, hp / max_hp)
        if self.show_labels or self.panel_tab == "inputs":
            channels = unit_decision_summary(side, int(archetype.position_car_id or 0))
            self.draw_label(
                f"SIM {side}:{type_name} {hp}/{max_hp} {channels}",
                sx + marker_extent + 7,
                sy + 10,
                image_rect,
            )

    def hit_sim_unit(self, pos: tuple[int, int], image_rect: Any) -> str | None:
        for unit, archetype in reversed(self.sim_scene_units()):
            x = unit.x
            y = unit.y
            sx, sy = self.field_to_screen((x, y), image_rect)
            type_name = archetype.label
            type_styles = as_dict(self.unit_styles.get("types"))
            style = as_dict(type_styles.get(type_name, type_styles.get("default", {})))
            radius = max(int(style.get("radius", 7)) + 8, self.unit_assets.unit_size_px // 2 + 8)
            if math.hypot(pos[0] - sx, pos[1] - sy) <= radius:
                return unit.entity_id
        return None

    def hit_sim_structure(self, pos: tuple[int, int], image_rect: Any) -> str | None:
        for item in reversed(self.sim_input_state.structures):
            position = self.sim_structure_position(item)
            if position is None:
                continue
            sx, sy = self.field_to_screen(position, image_rect)
            radius = 10 if item.structure == "outpost" else 12
            if math.hypot(pos[0] - sx, pos[1] - sy) <= radius + 10:
                return item.key
        return None

    def selected_structure(self) -> Any | None:
        key = getattr(self, "selected_structure_key", None)
        if not isinstance(key, str):
            return None
        return next((item for item in self.sim_input_state.structures if item.key == key), None)

    def draw_drag_preview(self) -> None:
        if self.dragging_unit is None or self.drag_position is None:
            return
        unit = self.dragging_unit
        pg = self.pg
        side = str(getattr(unit, "side", "friend"))
        type_name = str(getattr(unit, "type_name", ""))
        if not type_name:
            try:
                type_name = self.sim_input_state.catalog.unit_by_key(str(unit.unit_key)).label
            except (AttributeError, KeyError):
                type_name = "?"
        type_styles = as_dict(self.unit_styles.get("types"))
        style = as_dict(type_styles.get(type_name, type_styles.get("default", {})))
        side_style = as_dict(self.unit_styles.get(side, {}))
        radius = int(style.get("radius", 8)) + 3
        label = str(style.get("label", type_name[:1] or "?"))
        field_side = self.unit_field_side(side)
        if self.draw_unit_art(
            self.drag_position[0],
            self.drag_position[1],
            field_side,
            type_name,
            self.unit_assets.drag_unit_size_px,
            selected=True,
            alpha=190,
        ):
            return
        fill = pg.Color(side_style.get("color", self.colors_raw.get(side, "#4fb3d9")))
        outline = pg.Color(side_style.get("outline", "#000000"))
        pg.draw.circle(self.screen, self.palette["white"], self.drag_position, radius + 5)
        pg.draw.circle(self.screen, outline, self.drag_position, radius + 3)
        pg.draw.circle(self.screen, fill, self.drag_position, radius + 1)
        text = self.small_font.render(label, True, self.palette["black"])
        self.screen.blit(text, text.get_rect(center=self.drag_position))

    def draw_unit(self, unit: UnitRecord, image_rect: Any) -> None:
        if unit.position_cm is None:
            return
        if not (math.isfinite(unit.position_cm[0]) and math.isfinite(unit.position_cm[1])):
            return
        pg = self.pg
        sx, sy = self.field_to_screen(unit.position_cm, image_rect)
        type_styles = as_dict(self.unit_styles.get("types"))
        style = as_dict(type_styles.get(unit.type_name, type_styles.get("default", {})))
        side_style = as_dict(self.unit_styles.get(unit.side, {}))
        radius = int(style.get("radius", 7))
        label = str(style.get("label", unit.type_name[:1] or "?"))
        field_side = self.unit_field_side(unit.side)
        marker_extent = max(radius, self.unit_assets.trace_unit_size_px // 2)
        if not self.draw_unit_art(sx, sy, field_side, unit.type_name, self.unit_assets.trace_unit_size_px):
            fill = pg.Color(side_style.get("color", self.colors_raw.get(unit.side, "#4fb3d9")))
            outline = pg.Color(side_style.get("outline", "#000000"))
            pg.draw.circle(self.screen, outline, (sx, sy), radius + 3)
            pg.draw.circle(self.screen, fill, (sx, sy), radius + 1)
            text = self.small_font.render(label, True, self.palette["black"])
            self.screen.blit(text, text.get_rect(center=(sx, sy)))
            marker_extent = radius
        if self.layers.get("unit_health_bars", True) and unit.health_ratio is not None:
            self.draw_health_bar(sx - 19, sy + marker_extent + 6, 38, 5, unit.health_ratio)
        if self.show_labels:
            suffix = f" {unit.hp}/{unit.max_hp}" if unit.max_hp else ""
            self.draw_label(f"{unit.side}:{unit.type_name}{suffix}", sx + marker_extent + 6, sy + 8, image_rect)

    def unit_sprite(self, field_side: str, type_name: str, size_px: int) -> Any | None:
        path = self.unit_assets.path_for(field_side, type_name)
        if path is None:
            return None
        key = (
            str(field_side).strip().lower(),
            self.unit_assets.canonical_type_key(type_name),
            max(1, int(size_px)),
            path.as_posix(),
        )
        if key in self.unit_sprite_cache:
            return self.unit_sprite_cache[key]
        try:
            raw = self.pg.image.load(str(path)).convert_alpha()
            raw_rect = raw.get_rect()
            scale = min(size_px / max(1, raw_rect.width), size_px / max(1, raw_rect.height))
            target_size = (
                max(1, int(round(raw_rect.width * scale))),
                max(1, int(round(raw_rect.height * scale))),
            )
            sprite = self.pg.transform.smoothscale(raw, target_size)
        except Exception:
            sprite = None
        self.unit_sprite_cache[key] = sprite
        return sprite

    def draw_unit_art(
        self,
        sx: int,
        sy: int,
        field_side: str,
        type_name: str,
        size_px: int,
        *,
        selected: bool = False,
        alpha: int | None = None,
    ) -> bool:
        sprite = self.unit_sprite(field_side, type_name, size_px)
        if sprite is None:
            return False
        pg = self.pg
        radius = max(10, int(size_px) // 2)
        side_color = self.palette[field_side] if field_side in ("red", "blue") else self.palette["neutral"]
        pg.draw.circle(self.screen, self.palette["black"], (sx, sy), radius + (6 if selected else 4))
        if selected:
            pg.draw.circle(self.screen, self.palette["white"], (sx, sy), radius + 4)
        pg.draw.circle(self.screen, side_color, (sx, sy), radius + 2, 2)
        if alpha is not None:
            sprite = sprite.copy()
            sprite.set_alpha(max(0, min(255, int(alpha))))
        self.screen.blit(sprite, sprite.get_rect(center=(sx, sy)))
        return True

    def armor_sprite(self, name: str, size_px: int) -> Any | None:
        path = self.unit_assets.armor_path_for(name)
        if path is None:
            return None
        key = (str(name).strip().lower(), max(1, int(size_px)), path.as_posix())
        if key in self.armor_sprite_cache:
            return self.armor_sprite_cache[key]
        try:
            raw = self.pg.image.load(str(path)).convert_alpha()
            raw_rect = raw.get_rect()
            scale = min(size_px / max(1, raw_rect.width), size_px / max(1, raw_rect.height))
            target_size = (
                max(1, int(round(raw_rect.width * scale))),
                max(1, int(round(raw_rect.height * scale))),
            )
            sprite = self.pg.transform.smoothscale(raw, target_size)
        except Exception:
            sprite = None
        self.armor_sprite_cache[key] = sprite
        return sprite

    def draw_health_bar(self, x: int, y: int, width: int, height: int, ratio: float) -> None:
        pg = self.pg
        ratio = max(0.0, min(1.0, ratio))
        fill = self.palette["friend"] if ratio >= 0.45 else self.palette["accent"] if ratio >= 0.2 else self.palette["enemy"]
        rect = pg.Rect(x, y, width, height)
        pg.draw.rect(self.screen, self.palette["black"], rect.inflate(2, 2), border_radius=3)
        pg.draw.rect(self.screen, self.palette["panel2"], rect, border_radius=3)
        if ratio > 0:
            pg.draw.rect(self.screen, fill, pg.Rect(x, y, max(1, round(width * ratio)), height), border_radius=3)

    def draw_label(self, label: str, x: int, y: int, bounds: Any) -> None:
        pg = self.pg
        surface = self.small_font.render(label, True, self.palette["text"])
        rect = surface.get_rect()
        rect.topleft = (
            min(max(bounds.x + 8, x), bounds.right - rect.width - 14),
            min(max(bounds.y + 8, y), bounds.bottom - rect.height - 10),
        )
        bg = rect.inflate(12, 8)
        pg.draw.rect(self.screen, self.palette["panel2"], bg, border_radius=5)
        self.screen.blit(surface, rect)

    def draw_panel(self) -> None:
        pg = self.pg
        rect = self.panel_rect()
        if self.panel_tab != "inputs":
            self.inputs_panel.clear_buttons()
        self.layer_buttons = {}
        self.draw_flight_deck_card(rect)
        self.panel_tab_buttons = {}
        if self.inspector_collapsed:
            toggle = pg.Rect(rect.x + 8, rect.y + 10, rect.width - 16, 30)
            self.draw_control_button(toggle, "Inspect")
            self.workspace_buttons["toggle_inspector"] = toggle
            self.panel_body_rect = None
            return
        record = self.records[self.current_index]
        x = rect.x + 18
        title = self.font.render("Inspector", True, self.palette["text"])
        self.screen.blit(title, (x, rect.y + 16))
        dock = pg.Rect(rect.right - 142, rect.y + 10, 58, 28)
        collapse = pg.Rect(rect.right - 76, rect.y + 10, 58, 28)
        self.draw_control_button(dock, "Dock")
        self.draw_control_button(collapse, "Hide")
        self.workspace_buttons["toggle_dock"] = dock
        self.workspace_buttons["toggle_inspector"] = collapse
        splitter_x = rect.x - 3 if self.inspector_zone == "right" else rect.right - 3
        splitter = pg.Rect(splitter_x, rect.y + 48, 6, max(20, rect.height - 58))
        pg.draw.rect(self.screen, self.palette["line"], splitter, border_radius=3)
        self.workspace_buttons["inspector_splitter"] = splitter.inflate(8, 0)
        y = self.draw_selection_inspector(x, rect.y + 54, rect.width - 36)
        surface = self.small_font.render(self.panel_tab.upper(), True, self.palette["accent"])
        self.screen.blit(surface, (x, y + 8))
        y += 30
        body_rect = pg.Rect(rect.x, y, rect.width, max(1, rect.bottom - y - 10))
        self.panel_body_rect = body_rect
        self.panel_scroll.set_body_height(self.panel_tab, body_rect.height)
        scroll_offset = self.panel_scroll.offset(self.panel_tab)
        previous_clip = self.screen.get_clip()
        self.screen.set_clip(body_rect)
        content_y = body_rect.y - scroll_offset
        virtual_panel = pg.Rect(rect.x, rect.y, rect.width, 100000)
        final_y = self.draw_panel_body(x, content_y, rect.width - 36, virtual_panel, record)
        self.screen.set_clip(previous_clip)
        content_height = max(0, final_y - content_y)
        self.panel_scroll.set_content_height(self.panel_tab, content_height)
        self.clamp_panel_scroll()
        self.draw_panel_scrollbar(body_rect)

    def draw_selection_inspector(self, x: int, y: int, max_width: int) -> int:
        self.inspector_buttons = {}
        structure = self.selected_structure()
        if structure is None:
            unit = self.selected_scene_unit()
            if unit is not None:
                return self.draw_unit_selection_inspector(x, y, max_width, unit)
            if self.workspace_selection.kind == "area":
                return self.draw_area_selection_inspector(x, y, max_width)
            return self.draw_overview_selection_inspector(x, y, max_width)

        pg = self.pg
        hp = int(self.sim_input_state.structure_health.get(structure.key, structure.hp))
        max_hp = max(1, int(structure.max_hp))
        position = self.sim_structure_position(structure)
        title = f"{structure.structure.upper()} INSPECTOR"
        status = "Alive" if hp > 0 else "Destroyed"
        coordinate = "-" if position is None else f"{position[0]:.0f}, {position[1]:.0f} cm"
        card_h = 142
        card = pg.Rect(x, y, max_width, card_h)
        self.draw_flight_deck_inner_card(card, active=True)
        self.draw_text(title, x + 12, y + 9, self.small_font, self.palette["accent"], max_width - 24)
        self.draw_text(f"{structure.label}  {status}", x + 12, y + 30, self.font, self.palette["text"], max_width - 24)
        self.draw_text(f"HP {hp}/{max_hp}  ·  {coordinate}", x + 12, y + 52, self.small_font, self.palette["muted"], max_width - 24)
        self.draw_health_bar(x + 12, y + 72, max_width - 24, 5, hp / max_hp)

        actions = (
            (f"-{structure.step}", max(0, hp - int(structure.step))),
            (f"+{structure.step}", min(max_hp, hp + int(structure.step))),
            ("Restore", max_hp),
            ("Destroy", 0),
        )
        gap = 6
        button_w = (max_width - gap * (len(actions) - 1)) // len(actions)
        for index, (label, next_hp) in enumerate(actions):
            button = pg.Rect(x + index * (button_w + gap), y + 100, button_w, 32)
            self.draw_control_button(button, label)
            self.inspector_buttons[f"{structure.key}:{label}"] = (
                button,
                {"side": structure.side, "structure": structure.structure, "hp": int(next_hp)},
            )
        return card.bottom + 8

    def selected_scene_unit(self) -> Any | None:
        selection = getattr(self, "workspace_selection", Selection())
        if selection.kind != "unit":
            return None
        return self.sim_input_state.scene.units.get(selection.key)

    def draw_overview_selection_inspector(self, x: int, y: int, max_width: int) -> int:
        pg = self.pg
        record = self.records[self.current_index]
        card = pg.Rect(x, y, max_width, 104)
        self.draw_flight_deck_inner_card(card, active=True)
        self.draw_text("BATTLEFIELD OVERVIEW", x + 12, y + 10, self.small_font, self.palette["accent"], max_width - 24)
        self.draw_text(record.output.goal_name or "No active goal", x + 12, y + 32, self.font, self.palette["text"], max_width - 24)
        rows = [
            f"Layer  {record.decision_intent.layer or '-'}",
            f"Reason  {record.decision_intent.reason or '-'}",
            f"Output  {record.output.topic_text()}",
        ]
        text_y = y + 56
        for row in rows:
            text_y = self.draw_text(row, x + 12, text_y, self.small_font, self.palette["muted"], max_width - 24)
        return card.bottom + 8

    def draw_unit_selection_inspector(self, x: int, y: int, max_width: int, unit: Any) -> int:
        pg = self.pg
        try:
            archetype = self.sim_input_state.catalog.unit_by_key(unit.unit_key)
            label = archetype.label
            max_hp = max(1, int(archetype.max_hp))
        except KeyError:
            label = str(getattr(unit, "unit_key", "Robot"))
            max_hp = max(1, int(getattr(unit, "hp", 1)))
        hp = max(0, min(max_hp, int(unit.hp)))
        card = pg.Rect(x, y, max_width, 152)
        self.draw_flight_deck_inner_card(card, active=True)
        self.draw_text("ROBOT INSPECTOR", x + 12, y + 10, self.small_font, self.palette["accent"], max_width - 24)
        self.draw_text(f"{unit.side.title()} {label}", x + 12, y + 32, self.font, self.palette["text"], max_width - 24)
        self.draw_text(f"Position  {unit.x:.0f}, {unit.y:.0f} cm", x + 12, y + 54, self.small_font, self.palette["muted"], max_width - 24)
        self.draw_text(f"HP  {hp}/{max_hp}", x + 12, y + 72, self.small_font, self.palette["muted"], max_width - 24)
        self.draw_health_bar(x + 12, y + 92, max_width - 24, 5, hp / max_hp)
        step = 50 if max_hp > 100 else 10
        actions = ((f"-{step}", max(0, hp - step)), (f"+{step}", min(max_hp, hp + step)), ("Restore", max_hp), ("Destroy", 0))
        gap = 6
        button_w = (max_width - gap * (len(actions) - 1)) // len(actions)
        for index, (label_text, next_hp) in enumerate(actions):
            button = pg.Rect(x + index * (button_w + gap), y + 112, button_w, 32)
            self.draw_control_button(button, label_text)
            self.inspector_buttons[f"{unit.entity_id}:{label_text}"] = (
                button,
                {"_command": "set_unit_hp", "entity_id": unit.entity_id, "hp": int(next_hp)},
            )
        return card.bottom + 8

    def draw_area_selection_inspector(self, x: int, y: int, max_width: int) -> int:
        pg = self.pg
        item = self.map_area_by_key(self.workspace_selection.key)
        card = pg.Rect(x, y, max_width, 98)
        self.draw_flight_deck_inner_card(card, active=True)
        if item is None:
            title, detail = "Field area", "Map reference"
        else:
            title = str(item.get("name", "Field area"))
            detail = str(item.get("kind", item.get("level", "Reference")))
        self.draw_text("FIELD AREA", x + 12, y + 10, self.small_font, self.palette["accent"], max_width - 24)
        self.draw_text(title, x + 12, y + 32, self.font, self.palette["text"], max_width - 24)
        self.draw_text(f"{detail}  ·  reference only", x + 12, y + 56, self.small_font, self.palette["muted"], max_width - 24)
        return card.bottom + 8

    def draw_panel_header(self, x: int, y: int, max_width: int, record: TraceRecord) -> int:
        title = self.title_font.render("LY Simulator", True, self.palette["text"])
        self.screen.blit(title, (x, y))
        mode = "LIVE" if self.follow else "TRACE"
        state = "PLAY" if self.playing else "PAUSE"
        summary = f"{mode} {state} {self.current_index + 1}/{len(self.records)} x{self.playback_speed:.2g}"
        pill_w = min(max_width - title.get_width() - 10, max(120, self.small_font.size(summary)[0] + 18))
        if pill_w > 80:
            rect = self.pg.Rect(x + max_width - pill_w, y + 2, pill_w, 24)
            fill = self.palette["panel2"]
            border = self.palette["accent"] if self.follow else self.palette["line"]
            self.pg.draw.rect(self.screen, fill, rect, border_radius=6)
            self.pg.draw.rect(self.screen, border, rect, 1, border_radius=6)
            label = self.fit_word(summary, self.small_font, max(8, pill_w - 12))
            text = self.small_font.render(label, True, self.palette["text"])
            self.screen.blit(text, text.get_rect(center=rect.center))
        y += max(title.get_height(), 24) + 4
        timeline = f"tick={record.tick} t={record.t:.2f}s event={record.event}"
        return self.draw_text(timeline, x, y, self.small_font, self.palette["muted"], max_width)

    def draw_panel_body(self, x: int, y: int, max_width: int, panel: Any, record: TraceRecord) -> int:
        if self.panel_tab == "events":
            y = self.draw_map_tag_controls(x, y, max_width)
            y = self.draw_section(x, y, "Decision Conditions", self.condition_rows(record), max_width)
            y = self.draw_section(x, y, "Goal Reach", self.goal_reach_rows(record), max_width)
            y = self.draw_section(x, y, "Navi State", self.navi_state_rows(record), max_width)
            y = self.draw_section(x, y, "Target State", self.target_rows(record), max_width)
            y = self.draw_section(x, y, "Relative Target", self.relative_target_rows(record), max_width)
            y = self.draw_section(x, y, "Referee / Energy", self.referee_rows(record), max_width)
            return self.draw_section(x, y, "Units", self.unit_rows(record), max_width)

        if self.panel_tab == "runtime":
            y = self.draw_section(x, y, "ROS Output", self.ros_output_rows(record), max_width)
            y = self.draw_section(x, y, "Posture", [
                ("Command", record.posture_command),
                ("State", record.posture_state),
                ("Current", record.posture_current),
                ("Desired", record.posture_desired),
                ("Pending", record.posture_pending),
                ("Reason", record.posture_reason),
            ], max_width)
            y = self.draw_section(x, y, "Gimbal / FireCode", self.gimbal_rows(record), max_width)
            y = self.draw_section(x, y, "Bullet Info", self.bullet_info_rows(record), max_width)
            y = self.draw_section(x, y, "Runtime Guard", self.runtime_guard_rows(record), max_width)
            return self.draw_resource_bars(x, y, max_width, record)

        if self.panel_tab == "control":
            y = self.draw_section(x, y, "Final Control Output", self.control_output_rows(record), max_width)
            y = self.draw_section(x, y, "Lower-Machine Feedback", self.gimbal_feedback_rows(record), max_width)
            return self.draw_section(x, y, "Legacy Gimbal State", self.gimbal_rows(record), max_width)

        if self.panel_tab == "inputs":
            return self.inputs_panel.draw(x, y, max_width, panel)

        if self.panel_tab == "layers":
            y = self.draw_map_tag_controls(x, y, max_width)
            y = self.draw_layer_controls(x, y, max_width)
            y = self.draw_section(x, y, "Asset Catalog", self.asset_status_rows(), max_width)
            return self.draw_section(x, y, "Map Overlays", self.layer_summary_rows(), max_width)

        y = self.draw_map_tag_controls(x, y, max_width)
        y = self.draw_section(x, y, "Decision", [
            ("Profile", record.competition_profile),
            ("Team", record.team),
            ("Strategy", record.strategy),
            ("Aim", record.aim),
            ("Target", record.target),
            ("Events", record.events.compact_text()),
        ], max_width)
        y = self.draw_section(x, y, "Decision Output", [
            ("Kind", record.output.kind),
            ("UseXY", self.use_xy_text(record.output)),
            ("Goal", f"{record.output.goal_name} ({record.output.goal_id})"),
            ("Side", record.output.goal_side),
            ("Speed", str(record.output.speed_level)),
            ("Position", self.format_position(record_position(record, self.goals))),
            ("Topic", record.output.topic_text()),
            ("Source", record.output.source),
            (
                "Official",
                "valid="
                f"{self.flag(record.output.chase_official_target_valid)} "
                f"armor={record.output.chase_official_armor_type}",
            ),
            ("Publish", record.output.publish_text()),
        ], max_width)
        y = self.draw_section(x, y, "Decision Intent", [
            ("Layer", record.decision_intent.layer),
            ("Reason", record.decision_intent.reason),
            ("BaseGoal", str(record.decision_intent.base_goal_id)),
            ("Resolved", str(record.decision_intent.resolved_goal_id)),
            ("Team", record.decision_intent.goal_team),
            ("Priority", str(record.decision_intent.priority)),
            ("Detail", record.decision_intent.detail),
        ], max_width)
        if self.layers.get("recent_changes", True):
            y = self.draw_recent_changes(x, y, panel)
        return y

    def draw_panel_scrollbar(self, body_rect: Any) -> None:
        max_scroll = self.panel_max_scroll()
        if max_scroll <= 0 or body_rect.height <= 0:
            return
        pg = self.pg
        content_height = max(body_rect.height, self.panel_scroll.content_height(self.panel_tab, body_rect.height))
        track = pg.Rect(body_rect.right - 7, body_rect.y + 4, 3, max(12, body_rect.height - 8))
        thumb_h = max(24, round(track.height * body_rect.height / max(1, content_height)))
        scroll = self.panel_scroll.offset(self.panel_tab)
        thumb_y = track.y + round((track.height - thumb_h) * scroll / max(1, max_scroll))
        pg.draw.rect(self.screen, self.palette["panel2"], track, border_radius=2)
        pg.draw.rect(self.screen, self.palette["accent"], pg.Rect(track.x, thumb_y, track.width, thumb_h), border_radius=2)

    def draw_panel_tabs(self, x: int, y: int, max_width: int) -> int:
        pg = self.pg
        y += 4
        labels = [
            ("decision", "Decision"),
            ("events", "Events"),
            ("runtime", "Runtime"),
            ("control", "Control"),
            ("inputs", "Inputs"),
            ("layers", "Layers"),
        ]
        gap = 6
        columns = 3
        button_w = max(58, (max_width - gap * (columns - 1)) // columns)
        button_h = 26
        self.panel_tab_buttons = {}
        for idx, (tab, label) in enumerate(labels):
            row = idx // columns
            col = idx % columns
            rect = pg.Rect(x + col * (button_w + gap), y + row * (button_h + gap), button_w, button_h)
            self.panel_tab_buttons[tab] = rect
            fill = self.palette["accent"] if self.panel_tab == tab else self.palette["panel2"]
            text_color = self.palette["black"] if self.panel_tab == tab else self.palette["text"]
            pg.draw.rect(self.screen, fill, rect, border_radius=5)
            pg.draw.rect(self.screen, self.palette["line"], rect, 1, border_radius=5)
            text = self.small_font.render(label, True, text_color)
            self.screen.blit(text, text.get_rect(center=rect.center))
        rows = math.ceil(len(labels) / columns)
        y += rows * button_h + max(0, rows - 1) * gap + 8
        pg.draw.line(self.screen, self.palette["line"], (x, y), (x + max_width, y), 1)
        return y + 8

    def draw_target_preview(self, x: int, y: int, max_width: int, record: TraceRecord) -> int:
        sprite = self.armor_sprite("armor_no_background", 46)
        if sprite is None:
            sprite = self.armor_sprite("armor_fine_edited", 46)
        if sprite is None:
            return y

        pg = self.pg
        rect = pg.Rect(x, y, max_width, 48)
        pg.draw.rect(self.screen, self.palette["panel2"], rect, border_radius=5)
        pg.draw.rect(self.screen, self.palette["line"], rect, 1, border_radius=5)

        icon_center = (rect.x + 25, rect.centery)
        pg.draw.circle(self.screen, self.palette["black"], icon_center, 20)
        pg.draw.circle(self.screen, self.palette["accent"], icon_center, 18, 1)
        self.screen.blit(sprite, sprite.get_rect(center=icon_center))

        text_x = x + 54
        top = self.draw_text(record.target, text_x, y + 6, self.small_font, self.palette["text"], max_width - 62)
        self.draw_text(record.target_state.fresh_text(), text_x, top, self.small_font, self.palette["muted"], max_width - 62)
        return rect.bottom + 6

    def condition_rows(self, record: TraceRecord) -> list[tuple[str, str]]:
        events = record.events
        return [
            ("Summary", events.compact_text()),
            ("Fresh", f"event={self.flag(events.event_data_fresh)} sentry={self.flag(events.sentry_info_fresh)}"),
            (
                "Buff",
                " ".join(
                    [
                        f"task={self.flag(events.buff_task_enabled)}",
                        f"can={self.flag(events.buff_can_activate)}",
                        f"active={self.flag(events.buff_activating)}",
                        f"done={self.flag(events.buff_activated)}",
                    ]
                ),
            ),
            (
                "Outpost",
                " ".join(
                    [
                        f"task={self.flag(events.outpost_task_enabled)}",
                        f"alive={self.flag(events.enemy_outpost_alive)}",
                        f"window={self.flag(events.outpost_attack_window_open)}",
                    ]
                ),
            ),
            (
                "Risk",
                " ".join(
                    [
                        f"low_hp={self.flag(events.self_low_hp)}",
                        f"low_ammo={self.flag(events.self_low_ammo)}",
                        f"damage30={self.flag(events.recent_damage_over_30)}",
                    ]
                ),
            ),
            (
                "Goal",
                f"reached={self.flag(events.goal_reached)} unreachable={self.flag(events.goal_unreachable)}",
            ),
        ]

    def goal_reach_rows(self, record: TraceRecord) -> list[tuple[str, str]]:
        reach = record.goal_reach
        return [
            ("Summary", reach.compact_text()),
            ("Goal", f"id={reach.goal_id} base={reach.base_goal_id} age={self.value_text(reach.goal_age_ms)}ms"),
            ("Status", f"{reach.status} ({reach.status_id}) reason={reach.reason} ({reach.reason_id})"),
            ("External", f"reach={self.flag(reach.external_reach)} fresh={self.flag(reach.external_reach_fresh)} reachable={self.flag(reach.external_reachable)} fresh={self.flag(reach.external_reachable_fresh)}"),
            ("Position", f"has={self.flag(reach.has_position)} fresh={self.flag(reach.position_fresh)} dist={self.format_optional_float(reach.distance_cm, '{:.0f}cm')}"),
            ("Thresholds", f"arrive={reach.arrive_distance_cm}cm face={reach.face_distance_cm}cm fallback={self.flag(reach.distance_fallback_allowed)}"),
            ("Within", f"arrive={self.flag(reach.within_arrive_distance)} face={self.flag(reach.within_face_distance)} timeout={self.flag(reach.timeout)}"),
            ("Near goal", f"pending={self.flag(reach.near_goal_confirm_pending)} elapsed={reach.near_goal_confirm_elapsed_ms}ms"),
        ]

    def navi_state_rows(self, record: TraceRecord) -> list[tuple[str, str]]:
        status = record.navi_status
        velocity = record.navi_velocity
        vx = self.scaled_velocity_text(velocity.output_x, velocity.raw_to_mps)
        vy = self.scaled_velocity_text(velocity.output_y, velocity.raw_to_mps)
        return [
            ("Status", status.compact_text()),
            ("Rotate", f"should={self.flag(status.should_rotate)} fresh={self.flag(status.should_rotate_fresh)}"),
            ("Reachable", f"reached={self.flag(status.reached)} fresh={self.flag(status.reached_fresh)} reachable={self.flag(status.reachable)} fresh={self.flag(status.reachable_fresh)}"),
            ("Vel raw", f"in=({velocity.input_x},{velocity.input_y}) out=({velocity.output_x},{velocity.output_y})"),
            ("Vel m/s", f"x={vx} y={vy} scale={self.value_text(velocity.raw_to_mps)}"),
        ]

    def target_rows(self, record: TraceRecord) -> list[tuple[str, str]]:
        target_state = record.target_state
        return [
            ("Target", record.target),
            ("Fresh", target_state.fresh_text()),
            ("Locks", f"armor={self.flag(record.events.armor_target_visible)} buff={self.flag(record.events.buff_target_locked)} outpost={self.flag(record.events.outpost_target_locked)}"),
            ("Hitable", self.join_items(target_state.hitable_targets)),
            ("EnemyPos", self.join_items(target_state.reliable_enemy_positions)),
        ]

    def relative_target_rows(self, record: TraceRecord) -> list[tuple[str, str]]:
        target = record.navi_relative_target
        rel_xyz = ", ".join(
            [
                self.format_optional_float(target.x, "x={:.2f}"),
                self.format_optional_float(target.y, "y={:.2f}"),
                self.format_optional_float(target.z, "z={:.2f}"),
            ]
        )
        errors = ", ".join(
            [
                self.format_optional_float(target.distance, "dist={:.2f}"),
                self.format_optional_float(target.yaw_error_deg, "yaw={:.1f}deg"),
                self.format_optional_float(target.pitch_error_deg, "pitch={:.1f}deg"),
            ]
        )
        return [
            ("Mode", record.output.kind),
            ("Trace valid", f"output={self.flag(record.output.relative_target_valid)} target={self.flag(target.valid)}"),
            ("Frame", target.frame_id or "-"),
            ("Relative", rel_xyz),
            ("Errors", errors),
            ("Armor", f"type={target.armor_type} aim={target.aim_mode}"),
            ("Official", f"valid={self.flag(target.official_target_valid)} armor={target.official_armor_type}"),
            (
                "Output official",
                f"valid={self.flag(record.output.chase_official_target_valid)} armor={record.output.chase_official_armor_type}",
            ),
        ]

    def referee_rows(self, record: TraceRecord) -> list[tuple[str, str]]:
        referee = record.referee
        rfid = referee.rfid_match
        return [
            ("HP", f"self={record.hp} outpost={self.value_text(referee.self_outpost_hp)} base={self.value_text(referee.self_base_hp)}"),
            ("EnemyHP", f"outpost={self.value_text(referee.enemy_outpost_hp)} base={self.value_text(referee.enemy_base_hp)}"),
            ("Ammo/Time", f"ammo={record.ammo} time={record.time_left}"),
            ("Energy", f"can={self.flag(referee.sentry_can_activate_energy)} pulse={self.flag(referee.energy_activate_confirm_pulse)}"),
            ("GainPoint", f"center={self.value_text(referee.event_center_gain_point_status)} fortress={self.value_text(referee.event_self_fortress_gain_point_status)} outpost={self.value_text(referee.event_self_outpost_gain_point_status)} base={self.flag(referee.event_self_base_gain_point_status)}"),
            ("TeamBuff", f"atk={self.value_text(referee.team_buff_attack)} def={self.value_text(referee.team_buff_defence)} energy={self.value_text(referee.team_buff_remaining_energy)}"),
            ("RFID raw", f"fresh={self.flag(rfid.fresh)} any={self.flag(rfid.any)} raw={self.value_text(referee.rfid_status)} r2={self.value_text(referee.rfid_status_2)}"),
            ("RFID self", f"base={self.flag(rfid.self_base_gain_point)} supply={self.flag(rfid.self_supply)} high={self.flag(rfid.self_highland_gain_point)} road={self.flag(rfid.self_road_crossing)} tunnel={self.flag(rfid.self_tunnel)}"),
            ("RFID enemy", f"high={self.flag(rfid.enemy_highland_gain_point)} road={self.flag(rfid.enemy_road_crossing)} tunnel={self.flag(rfid.enemy_tunnel)} fortress={self.flag(rfid.enemy_fortress_gain_point)} outpost={self.flag(rfid.enemy_outpost_gain_point)}"),
            ("RFID zone", f"center={self.flag(rfid.center_gain_point)} self_side={self.flag(rfid.on_self_side)} enemy_side={self.flag(rfid.on_enemy_side)} fly={self.flag(rfid.self_fly_ramp)}/{self.flag(rfid.enemy_fly_ramp)}"),
        ]

    def unit_rows(self, record: TraceRecord) -> list[tuple[str, str]]:
        friend = [unit for unit in record.units if unit.side == "friend"]
        enemy = [unit for unit in record.units if unit.side == "enemy"]
        friend_text = self.units_text(friend)
        enemy_text = self.units_text(enemy)
        rows = [
            ("Friend", friend_text),
            ("Enemy", enemy_text),
        ]
        if record.unit_info:
            friend_info = [unit for unit in record.unit_info if unit.side == "friend"]
            enemy_info = [unit for unit in record.unit_info if unit.side == "enemy"]
            rows.extend(
                [
                    ("FriendInfo", self.unit_info_text(friend_info)),
                    ("EnemyInfo", self.unit_info_text(enemy_info)),
                ]
            )
        return rows

    def gimbal_rows(self, record: TraceRecord) -> list[tuple[str, str]]:
        gimbal = record.gimbal
        return [
            ("Angles", f"yaw={self.format_optional_float(gimbal.yaw_deg)} pitch={self.format_optional_float(gimbal.pitch_deg)}"),
            ("YawRaw", f"vel={self.format_optional_float(gimbal.yaw_vel_deg_per_sec)} angle={self.format_optional_float(gimbal.yaw_angle_deg)}"),
            ("Cap", f"cap_v={gimbal.cap_v} lower_head={gimbal.navi_lower_head}"),
            ("FireCode", f"fire={gimbal.fire_status} cap={gimbal.cap_state} follow={gimbal.follow_mode}"),
            ("Aim/Rotate", f"aim={gimbal.aim_mode} rotate={gimbal.rotate}"),
        ]

    def gimbal_feedback_rows(self, record: TraceRecord) -> list[tuple[str, str]]:
        feedback = record.gimbal_feedback
        if not feedback.available:
            return [
                ("Available", "N"),
                ("FireCode", "not received"),
            ]
        fire_code = feedback.fire_code
        age = "-" if feedback.age_ms is None else f"{feedback.age_ms}ms"
        return [
            ("Available", f"Y age={age}"),
            (
                "FireCode",
                " ".join(
                    [
                        f"fire={self.value_text(fire_code.fire_status)}",
                        f"cap={self.value_text(fire_code.cap_state)}",
                        f"follow={self.flag(fire_code.follow_mode)}",
                        f"aim={self.flag(fire_code.aim_mode)}",
                        f"rotate={self.value_text(fire_code.rotate)}",
                    ]
                ),
            ),
        ]

    def control_output_rows(self, record: TraceRecord) -> list[tuple[str, str]]:
        output = record.control_output
        if not output.available:
            return [
                ("Available", "N"),
                ("Source", output.source),
                ("Trajectory", output.trajectory.unavailable_reason),
            ]
        angles = output.angles
        fire_code = output.fire_code
        trajectory = output.trajectory
        angle_text = "not published"
        if angles.published:
            angle_text = (
                f"yaw={self.format_optional_float(angles.yaw)} "
                f"pitch={self.format_optional_float(angles.pitch)}"
            )
        fire_text = "not published"
        if fire_code.published:
            fire_text = " ".join(
                [
                    f"fire={self.value_text(fire_code.fire_status)}",
                    f"cap={self.value_text(fire_code.cap_state)}",
                    f"follow={self.flag(fire_code.follow_mode)}",
                    f"aim={self.flag(fire_code.aim_mode)}",
                    f"rotate={self.value_text(fire_code.rotate)}",
                ]
            )
        trajectory_text = (
            f"published={self.flag(trajectory.published)} valid={self.flag(trajectory.available)}"
        )
        if trajectory.available:
            trajectory_text += (
                f" yaw={self.format_optional_float(trajectory.yaw)}"
                f" pitch={self.format_optional_float(trajectory.pitch)}"
            )
        elif trajectory.unavailable_reason:
            trajectory_text += f" reason={trajectory.unavailable_reason}"
        age = "-" if output.age_ms is None else f"{output.age_ms}ms"
        return [
            ("Available", f"Y seq={self.value_text(output.sequence)} age={age}"),
            ("Source", output.source),
            ("Angles", angle_text),
            ("FireCode", fire_text),
            ("Trajectory", trajectory_text),
        ]

    def bullet_info_rows(self, record: TraceRecord) -> list[tuple[str, str]]:
        bullet = record.bullet_info
        return [
            ("Summary", bullet.compact_text()),
            ("Fresh", f"received={self.flag(bullet.has_received)} age={self.value_text(bullet.age_ms)}ms"),
            ("Speed", f"has={self.flag(bullet.has_initial_speed)} value={self.format_optional_float(bullet.initial_speed)}m/s"),
            (
                "Shoot",
                f"has={self.flag(bullet.has_shoot_data)} type={bullet.bullet_type} "
                f"shooter={bullet.shooter_number} hz={bullet.launching_frequency}",
            ),
            (
                "Allowance",
                " ".join(
                    [
                        f"has={self.flag(bullet.has_projectile_allowance)}",
                        f"17={bullet.projectile_allowance_17mm}",
                        f"42={bullet.projectile_allowance_42mm}",
                        f"coin={bullet.remaining_gold_coin}",
                        f"fort17={bullet.projectile_allowance_fortress_17mm}",
                    ]
                ),
            ),
        ]

    def runtime_guard_rows(self, record: TraceRecord) -> list[tuple[str, str]]:
        guard = record.runtime_guard
        posture_runtime = record.posture_runtime
        lock = record.outpost_engagement_lock
        return [
            ("Fault", guard.fault),
            ("Recovery", f"requested={self.flag(guard.recovery_requested)} recovering={self.flag(guard.recovering)}"),
            ("PostureRT", f"pending={self.flag(posture_runtime.has_pending)} stale={self.flag(posture_runtime.feedback_stale)} retry={self.value_text(posture_runtime.retry_count)}"),
            ("Degraded", f"atk={self.flag(posture_runtime.degraded_attack)} def={self.flag(posture_runtime.degraded_defense)} move={self.flag(posture_runtime.degraded_move)}"),
            ("OutpostLock", f"active={self.flag(lock.active)} hold7={self.flag(lock.hold_target)} arm={self.flag(lock.enhanced_armed)} pending={self.flag(lock.enhanced_pending)} active4={self.flag(lock.enhanced_active)} exit={lock.exit_reason}"),
        ]

    def draw_match_controls(self, x: int, y: int, max_width: int, record: TraceRecord) -> int:
        pg = self.pg
        controls_available = bool(self.match_control_enabled and self.control_path is not None and self.follow)
        if controls_available:
            remaining = max(0, min(self.match_duration_sec, int(math.ceil(self.match_time_left_sec))))
            state = "RUN" if self.match_running else ("READY" if self.match_started else "WAIT_START")
        else:
            remaining = record.time_left if record.time_left > 0 else self.match_duration_sec
            remaining = max(0, min(self.match_duration_sec, int(remaining)))
            state = "TRACE"
        summary = f"Match {state} {self.format_mmss(remaining)} / {self.format_mmss(self.match_duration_sec)}"
        y = self.draw_text(summary, x, y, self.mono_font, self.palette["text"], max_width)
        if self.scripted_enabled:
            mode = "loop" if self.scripted_loop else "once"
            path_status = f"path {len(self.scripted_waypoints)}pts {self.scripted_speed_cmps:.0f}cm/s {mode}"
            y = self.draw_text(path_status, x, y, self.small_font, self.palette["muted"], max_width)

        self.control_buttons = {}
        if controls_available:
            y += 5
            gap = 6
            labels = [
                ("start", "Start"),
                ("pause", "Pause"),
                ("rewind", f"+{self.rewind_step_sec}s"),
                ("forward", f"-{self.forward_step_sec}s"),
                ("reset", "Reset"),
            ]
            count = len(labels)
            button_w = max(54, (max_width - gap * (count - 1)) // count)
            button_h = 26
            for idx, (command, label) in enumerate(labels):
                bx = x + idx * (button_w + gap)
                rect = pg.Rect(bx, y, button_w, button_h)
                self.control_buttons[command] = rect
                self.draw_control_button(rect, label)
            y += button_h + 4
            if self.last_control_status != "idle":
                y = self.draw_text(self.last_control_status, x, y, self.small_font, self.palette["muted"], max_width)
        else:
            y += 2
            y = self.draw_text("controls: disabled", x, y, self.small_font, self.palette["muted"], max_width)

        y += 6
        pg.draw.line(self.screen, self.palette["line"], (x, y), (x + max_width, y), 1)
        return y + 7

    def draw_map_tag_controls(self, x: int, y: int, max_width: int) -> int:
        pg = self.pg
        y += 2
        state = "expanded" if self.goal_tags_expanded else "hover"
        label = f"Map Tags: {state}"
        y = self.draw_text(label, x, y, self.small_font, self.palette["muted"], max_width - 118)
        button_w = 108
        button_h = 24
        rect = pg.Rect(x + max_width - button_w, y - button_h, button_w, button_h)
        self.goal_tag_button_rect = rect
        button_label = "Collapse" if self.goal_tags_expanded else "Expand"
        self.draw_control_button(rect, button_label)
        y += 4
        pg.draw.line(self.screen, self.palette["line"], (x, y), (x + max_width, y), 1)
        return y + 8

    def draw_layer_controls(self, x: int, y: int, max_width: int) -> int:
        pg = self.pg
        y += 3
        self.draw_text("Map Layers", x, y, self.font, self.palette["accent"], max_width)
        y += 25
        labels = [
            ("terrain", "Terrain"),
            ("structures", "Structures"),
            ("simulator_inputs", "Sim Inputs"),
            ("grid", "Grid"),
            ("all_goals", "All Goals"),
            ("current_goal", "Current"),
            ("goal_path", "Goal Path"),
            ("scripted_path", "Scripted"),
            ("units", "Units"),
            ("unit_health_bars", "HP Bars"),
            ("recent_changes", "Changes"),
        ]
        gap = 6
        columns = 2
        button_w = max(80, (max_width - gap * (columns - 1)) // columns)
        button_h = 24
        for idx, (key, label) in enumerate(labels):
            col = idx % columns
            row = idx // columns
            rect = pg.Rect(x + col * (button_w + gap), y + row * (button_h + gap), button_w, button_h)
            self.layer_buttons[key] = rect
            active = bool(self.layers.get(key, True))
            fill = self.palette["accent"] if active else self.palette["panel2"]
            text_color = self.palette["black"] if active else self.palette["muted"]
            pg.draw.rect(self.screen, fill, rect, border_radius=5)
            pg.draw.rect(self.screen, self.palette["line"], rect, 1, border_radius=5)
            prefix = "ON" if active else "OFF"
            text = self.small_font.render(f"{prefix} {label}", True, text_color)
            self.screen.blit(text, text.get_rect(center=rect.center))
        rows = (len(labels) + columns - 1) // columns
        y += rows * button_h + max(0, rows - 1) * gap + 10
        pg.draw.line(self.screen, self.palette["line"], (x, y), (x + max_width, y), 1)
        return y + 8

    def asset_status_rows(self) -> list[tuple[str, str]]:
        catalog = self.unit_assets
        provenance = catalog.provenance
        manifest = catalog.manifest_path.name if catalog.manifest_path is not None else "-"
        source = str(provenance.get("archive_name", "-"))
        imported = str(provenance.get("imported_at", "-"))
        license_status = str(provenance.get("license_status", "unknown"))
        redistribution = str(provenance.get("redistribution", "-"))
        aliases = ", ".join(f"{alias}->{target}" for alias, target in sorted(catalog.aliases.items()))
        return [
            ("Enabled", self.flag(catalog.enabled)),
            ("Manifest", manifest),
            ("Units", f"{len(catalog.unit_paths)} sprites"),
            ("Armor", f"{len(catalog.armor_paths)} sprites"),
            ("Aliases", aliases if aliases else "-"),
            ("Source", f"{source} imported={imported}"),
            ("License", license_status),
            ("Redistrib", redistribution),
        ]

    def layer_summary_rows(self) -> list[tuple[str, str]]:
        enabled = sorted(key for key, value in self.layers.items() if bool(value))
        disabled = sorted(key for key, value in self.layers.items() if not bool(value))
        return [
            ("Enabled", ", ".join(enabled) if enabled else "-"),
            ("Disabled", ", ".join(disabled) if disabled else "-"),
            ("Tags", "expanded" if self.goal_tags_expanded else "hover"),
            ("Scripted", "on" if self.scripted_enabled else "off"),
        ]

    def draw_control_button(self, rect: Any, label: str) -> None:
        pg = self.pg
        hovered = rect.collidepoint(pg.mouse.get_pos())
        pg.draw.rect(self.screen, self.palette["panel2"], rect, border_radius=5)
        border = self.palette["accent"] if hovered else self.palette["line"]
        pg.draw.rect(self.screen, border, rect, 1, border_radius=5)
        txt = self.small_font.render(label, True, self.palette["white"] if hovered else self.palette["text"])
        self.screen.blit(txt, txt.get_rect(center=rect.center))

    def draw_flight_deck_card(self, rect: Any, *, active: bool = False) -> None:
        """Draw the shared matte Flight Deck card material without changing hit areas."""

        pg = self.pg
        shadow = rect.move(0, 3)
        pg.draw.rect(self.screen, self.palette["black"], shadow, border_radius=12)
        pg.draw.rect(self.screen, self.palette["panel"], rect, border_radius=12)
        border = self.palette["accent"] if active else self.palette["line"]
        pg.draw.rect(self.screen, border, rect, 1, border_radius=12)
        highlight = pg.Rect(rect.x + 12, rect.y + 1, max(1, rect.width - 24), 1)
        pg.draw.rect(self.screen, self.palette["panel2"], highlight, border_radius=1)

    def draw_flight_deck_inner_card(self, rect: Any, *, active: bool = False) -> None:
        """Draw an Inspector card with the same material grammar at a denser scale."""

        pg = self.pg
        shadow = rect.move(0, 2)
        pg.draw.rect(self.screen, self.palette["black"], shadow, border_radius=12)
        pg.draw.rect(self.screen, self.palette["panel2"], rect, border_radius=12)
        border = self.palette["accent"] if active else self.palette["line"]
        pg.draw.rect(self.screen, border, rect, 1, border_radius=12)
        highlight = pg.Rect(rect.x + 12, rect.y + 1, max(1, rect.width - 24), 1)
        pg.draw.rect(self.screen, self.palette["line"], highlight, border_radius=1)

    @staticmethod
    def wrapped_line_count(text: str, font: Any, max_width: int) -> int:
        if max_width <= 0:
            return 0
        words = str(text).split() or [""]
        lines = 1
        line = ""
        for word in words:
            candidate = word if not line else f"{line} {word}"
            if font.size(candidate)[0] <= max_width:
                line = candidate
                continue
            lines += 1
            line = word
        return lines

    def draw_section(self, x: int, y: int, title: str, rows: list[tuple[str, str]], max_width: int) -> int:
        pg = self.pg
        key_w = min(90, max(72, max_width // 4))
        value_width = max(1, max_width - key_w - 24)
        line_height = self.small_font.get_linesize()
        row_heights = [max(line_height, self.wrapped_line_count(value, self.small_font, value_width) * line_height) for _, value in rows]
        card_h = 40 + sum(height + 5 for height in row_heights) + 14
        card = pg.Rect(x, y + 3, max_width, card_h)
        self.draw_flight_deck_inner_card(card)
        content_x = card.x + 12
        content_y = card.y + 12
        self.draw_text(title, content_x, content_y, self.font, self.palette["accent"], max_width - 24)
        content_y += 26
        for key, value in rows:
            key_surface = self.small_font.render(key, True, self.palette["muted"])
            self.screen.blit(key_surface, (content_x, content_y + 2))
            content_y = self.draw_text(value, content_x + key_w, content_y, self.small_font, self.palette["text"], value_width)
            content_y += 5
        return card.bottom + 12

    def ros_output_rows(self, record: TraceRecord) -> list[tuple[str, str]]:
        output = record.output
        live_start = self.live_ros_value_text("/ly/game/is_start")
        live_time = self.live_ros_value_text("/ly/game/time_left")
        rows: list[tuple[str, str]] = [
            ("Live source", self.live_ros_status_text()),
            ("Live goal_pose", self.live_ros_value_text("/goal_pose", "cm")),
            ("Live legacy goal_pos", self.live_ros_value_text("/ly/navi/goal_pos", "cm")),
            ("Live raw", self.live_ros_value_text("/ly/navi/goal_pos_raw", "cm")),
            ("Live goal_id", self.live_ros_value_text("/ly/navi/goal")),
            ("Live speed", self.live_ros_value_text("/ly/navi/speed_level")),
            ("Live rotate", self.live_ros_value_text("/ly/navi/should_rotate")),
            ("Live vel", self.live_ros_value_text("/ly/control/vel")),
            ("Live game", f"start={live_start} time={live_time}"),
        ]
        rows.extend(
            [
                ("Trace topic", output.output_topic or "-"),
                ("Trace payload", self.ros_payload_text(record)),
            ]
        )
        if output.final_goal_pos_topic and output.final_goal_pos_topic != output.output_topic:
            rows.append(("Trace final", output.final_goal_pos_topic))
        rows.extend(
            [
                ("Trace speed", f"/ly/navi/speed_level={output.speed_level}"),
                ("Trace rotate", f"/ly/navi/should_rotate={self.flag(record.navi_status.should_rotate)} fresh={self.flag(record.navi_status.should_rotate_fresh)}"),
                ("Trace vel", record.navi_velocity.compact_text()),
                ("SimPath", "visual only" if self.scripted_enabled else "off"),
                ("Publish", output.publish_text()),
            ]
        )
        return rows

    def live_ros_value(self, topic: str) -> Any | None:
        item = as_dict(self.live_ros_topics.get(topic))
        if not item:
            return None
        return item.get("value")

    def live_goal_position(self, topic: str) -> tuple[float, float] | None:
        pos = parse_position(self.live_ros_value(topic))
        if pos is None:
            return None
        if not (math.isfinite(pos[0]) and math.isfinite(pos[1])):
            return None
        return pos

    def update_live_goal_history(self) -> None:
        pos = self.live_goal_position("/goal_pose")
        if pos is None:
            pos = self.live_goal_position("/ly/navi/goal_pos")
        if pos is None:
            return
        if self.live_goal_history:
            prev = self.live_goal_history[-1]
            if math.hypot(pos[0] - prev[0], pos[1] - prev[1]) < 0.5:
                return
        self.live_goal_history.append(pos)
        limit = max(1, int(self.timeline_config.get("path_history_limit", 500)))
        if len(self.live_goal_history) > limit:
            del self.live_goal_history[: len(self.live_goal_history) - limit]

    def live_ros_value_text(self, topic: str, suffix: str = "") -> str:
        item = as_dict(self.live_ros_topics.get(topic))
        if not item:
            return "-"
        value = item.get("value")
        text = self.format_live_value(value, suffix)
        age_text = self.live_ros_item_age_text(item)
        return f"{text} age={age_text}" if age_text else text

    def live_ros_status_text(self) -> str:
        if self.ros_state_path is None:
            return "disabled"
        if not self.ros_state_path.exists():
            return f"waiting {self.ros_state_path.name}"
        if self.live_ros_wall_time <= 0.0:
            return "loading"
        age = max(0.0, time.time() - self.live_ros_wall_time)
        stale = " stale" if age > self.ros_monitor_stale_sec else ""
        return f"monitor age={age:.2f}s{stale}"

    def live_ros_item_age_text(self, item: dict[str, Any]) -> str:
        item_time = self.to_float(item.get("wall_time"), 0.0)
        if item_time <= 0.0:
            return ""
        age = max(0.0, time.time() - item_time)
        stale = " stale" if age > self.ros_monitor_stale_sec else ""
        return f"{age:.2f}s{stale}"

    @staticmethod
    def format_live_value(value: Any, suffix: str = "") -> str:
        if isinstance(value, list):
            text = "[" + ", ".join(str(item) for item in value) + "]"
        else:
            text = str(value)
        return f"{text} {suffix}".strip()

    def live_ros_age_text(self) -> str:
        if self.live_ros_wall_time <= 0.0:
            return "-"
        age = max(0.0, time.time() - self.live_ros_wall_time)
        stale = " stale" if age > self.ros_monitor_stale_sec else ""
        return f"{age:.2f}s{stale}"

    def ros_payload_text(self, record: TraceRecord) -> str:
        output = record.output
        if output.kind == "relative_target_bridge":
            frame = record.navi_relative_target.frame_id or "-"
            return f"target_rel valid={str(output.relative_target_valid).lower()} frame={frame}"
        pos = record_position(record, self.goals)
        if output.uses_goal_pos or pos is not None:
            if pos is None:
                return "[?, ?]"
            return f"[{pos[0]:.0f}, {pos[1]:.0f}] cm"
        return f"id={output.goal_id}"

    def draw_resource_bars(self, x: int, y: int, max_width: int, record: TraceRecord) -> int:
        max_hp = record.referee.self_max_hp
        ammo_max = int(as_dict(self.config.get("resources")).get("ammo_max", 50))
        self.draw_metric_bar(x + 96, y, max_width - 96, record.hp, max_hp)
        self.draw_metric_bar(x + 96, y + 8, max_width - 96, record.ammo, ammo_max)
        return y + 18

    def draw_metric_bar(self, x: int, y: int, width: int, value: int, maximum: int) -> None:
        if width <= 0 or maximum <= 0:
            return
        ratio = max(0.0, min(1.0, value / maximum))
        self.draw_health_bar(x, y, width, 5, ratio)

    def draw_recent_changes(self, x: int, y: int, rect: Any) -> int:
        self.draw_text("Recent Changes", x, y, self.font, self.palette["accent"], rect.width - 36)
        y += 24
        indexes = [item["index"] for item in self.changes]
        current = bisect.bisect_right(indexes, self.current_index) - 1
        visible = self.changes[max(0, current - 7) : current + 1]
        for item in visible:
            color_value = self.palette["text"] if item["index"] == self.current_index else self.palette["muted"]
            y = self.draw_text(f"{item['t']:.1f}s {item['text']}", x, y, self.small_font, color_value, rect.width - 36)
            y += 3
            if y > rect.bottom - 30:
                break
        return y

    def draw_timeline(self) -> None:
        pg = self.pg
        panel = self.operations_shelf_rect()
        track = self.timeline_rect()
        pg.draw.rect(self.screen, self.palette["panel2"], track, border_radius=8)
        ratio = self.current_index / max(1, len(self.records) - 1)
        pg.draw.rect(self.screen, self.palette["accent"], pg.Rect(track.x, track.y, round(track.width * ratio), track.height), border_radius=8)
        for item in self.changes:
            x = track.x + round(track.width * item["index"] / max(1, len(self.records) - 1))
            pg.draw.line(self.screen, self.palette["neutral"], (x, track.y - 6), (x, track.bottom + 6), 1)
        record = self.records[self.current_index]
        self.draw_text(self.map_path.name, track.x, track.y - 24, self.small_font, self.palette["muted"], track.width // 2)
        right = f"tick={record.tick} event={record.event} output={record.output.goal_name}:{record.output.goal_id}"
        text_w = self.small_font.size(right)[0]
        self.draw_text(right, track.right - text_w, track.y - 24, self.small_font, self.palette["muted"], text_w + 4)

    def draw_text(self, text: str, x: int, y: int, font: Any, draw_color: Any, max_width: int) -> int:
        if max_width <= 0:
            return y
        words = str(text).split() or [""]
        line = ""
        line_height = font.get_linesize()
        for word in words:
            candidate = word if not line else f"{line} {word}"
            if font.size(candidate)[0] <= max_width:
                line = candidate
                continue
            if line:
                self.screen.blit(font.render(line, True, draw_color), (x, y))
                y += line_height
            line = self.fit_word(word, font, max_width)
        if line:
            self.screen.blit(font.render(line, True, draw_color), (x, y))
            y += line_height
        return y

    @staticmethod
    def fit_word(word: str, font: Any, max_width: int) -> str:
        if font.size(word)[0] <= max_width:
            return word
        out = word
        while out and font.size(out + "...")[0] > max_width:
            out = out[:-1]
        return out + "..." if out else "..."

    @staticmethod
    def flag(value: Any) -> str:
        if value is None:
            return "-"
        if isinstance(value, bool):
            return "1" if value else "0"
        if isinstance(value, (int, float)):
            return "1" if bool(value) else "0"
        text = str(value).strip().lower()
        if text in {"true", "1", "yes", "on"}:
            return "1"
        if text in {"false", "0", "no", "off"}:
            return "0"
        return str(value)

    @staticmethod
    def value_text(value: Any) -> str:
        return "-" if value is None else str(value)

    @staticmethod
    def join_items(items: tuple[str, ...]) -> str:
        return ", ".join(items) if items else "-"

    @staticmethod
    def format_optional_float(value: float | None, template: str = "{:.1f}") -> str:
        if value is None:
            return "-"
        return template.format(value)

    @staticmethod
    def scaled_velocity_text(raw: int, scale: float | None) -> str:
        if scale is None:
            return "-"
        value = raw * scale
        return f"{value:.2f}"

    @staticmethod
    def units_text(units: list[UnitRecord]) -> str:
        if not units:
            return "-"
        parts = []
        for unit in units:
            hp = f"{unit.hp}/{unit.max_hp}" if unit.max_hp else str(unit.hp)
            parts.append(f"{unit.type_name}:{hp}")
        return " ".join(parts)

    @staticmethod
    def unit_info_text(units: list[UnitInfoRecord]) -> str:
        if not units:
            return "-"
        parts = []
        for unit in sorted(units, key=lambda item: item.type_id):
            hp = f"hp={unit.hp}" if unit.has_hp else "hp=-"
            pos = "pos=fresh" if unit.position_fresh else "pos=stale" if unit.has_position else "pos=-"
            source = unit.position_source if unit.position_source else "-"
            area = unit.area_name if unit.area_name else "-"
            parts.append(f"{unit.type_name}:{hp},{pos},{source},{area}")
        return " ".join(parts)

    @staticmethod
    def format_position(pos: tuple[float, float] | None) -> str:
        if pos is None:
            return "-"
        return f"{pos[0]:.0f}, {pos[1]:.0f} cm"

    @staticmethod
    def format_mmss(total_seconds: int) -> str:
        total = max(0, int(total_seconds))
        return f"{total // 60:02d}:{total % 60:02d}"

    @staticmethod
    def publish_text(raw: dict[str, Any]) -> str:
        navi = as_dict(raw.get("navi_goal"))
        allowed = navi.get("publish_allowed")
        enabled = navi.get("publish_enabled")
        if allowed is None and enabled is None:
            return "-"
        return f"allowed={allowed} enabled={enabled}"

    @staticmethod
    def use_xy_text(output: Any) -> str:
        if output.kind == "relative_target_bridge":
            return "true relative bridge"
        if output.uses_goal_pos:
            return "true bridge" if output.uses_to_navi else "true direct"
        return "false goal_id"

    @staticmethod
    def outpost_text(raw: dict[str, Any]) -> str:
        referee = as_dict(raw.get("referee"))
        self_hp = referee.get("self_outpost_hp")
        enemy_hp = referee.get("enemy_outpost_hp")
        if self_hp is None and enemy_hp is None:
            return "-"
        return f"self={self_hp} enemy={enemy_hp}"
