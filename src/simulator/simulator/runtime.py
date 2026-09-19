"""Headless simulator runtime shared by the browser tactical board.

This module deliberately owns simulation/replay state only.  It has no
pygame dependency and produces the metadata consumed by ``tactical_web``.
The native pygame ``Viewer`` remains an explicit debug renderer.
"""

from __future__ import annotations

import bisect
import json
import math
import time
from pathlib import Path
from typing import Any

from .control_bus import append_command, command_name, read_commands
from .field import FieldGeometry
from .interactive_inputs import SimulatorInputState
from .trace import as_dict, build_changes, load_trace_incremental, parse_position
from .viewer import point_payload, record_position, record_status_payload


class SimulationRuntime:
    """Advance trace, match and editable scene state without drawing a UI."""

    def __init__(
        self,
        *,
        records: list[Any],
        config: dict[str, Any],
        goals: dict[int, dict[str, Any]],
        bad_lines: int,
        trace_path: Path | None,
        goal_names: dict[int, str] | None,
        follow: bool,
        follow_poll_sec: float,
        follow_offset: int,
        start_paused: bool | None,
        speed: float | None,
    ) -> None:
        self.records = records
        self.changes = build_changes(records)
        self.config = config
        self.goals = goals
        self.bad_lines = int(bad_lines)
        self.trace_path = trace_path
        self.goal_names = goal_names or {}
        self.follow = bool(follow and trace_path is not None)
        self.follow_poll_sec = max(0.05, float(follow_poll_sec))
        self.follow_offset = max(0, int(follow_offset))
        self.last_follow_poll = time.perf_counter()
        self.last_control_poll = time.perf_counter()
        self.last_ros_monitor_poll = time.perf_counter()
        self.control_poll_sec = 0.08
        self.live_ros_topics: dict[str, Any] = {}
        self.live_ros_wall_time = 0.0
        self.live_goal_history: list[tuple[float, float]] = []

        window = as_dict(config.get("window"))
        self.playing = not (window.get("start_paused", False) if start_paused is None else start_paused)
        self.playback_speed = float(window.get("playback_speed", 1.0) if speed is None else speed)
        self.timeline_config = as_dict(config.get("timeline"))
        self.times = [record.t for record in records]
        self.current_index = 0
        self.current_time = self.times[0]

        simulator_inputs = as_dict(config.get("simulator_inputs"))
        self.simulator_inputs_enabled = bool(simulator_inputs.get("enabled", True))
        self.sim_input_state = SimulatorInputState.from_config(
            simulator_inputs,
            field=FieldGeometry.from_config(config.get("field_cm")),
        )
        self.active_team = self.sim_input_state.team
        self.visual_goal_tracking = bool(simulator_inputs.get("visual_goal_tracking", False))
        self.visual_unit_speed_cm_s = max(
            0.0,
            self._float(simulator_inputs.get("visual_unit_speed_cm_s"), 250.0),
        )
        self.supply_heal_enabled = bool(simulator_inputs.get("supply_heal_enabled", False))
        self.supply_heal_rate_ratio = max(
            0.0,
            self._float(simulator_inputs.get("supply_heal_rate_ratio"), 0.25),
        )
        self.supply_radius_cm = max(
            0.0,
            self._float(simulator_inputs.get("supply_radius_cm"), 180.0),
        )
        self._last_self_health_command_time = 0.0
        self._last_self_health_command_hp = -1

        self.ros_monitor = as_dict(config.get("ros_monitor"))
        ros_state_file = str(self.ros_monitor.get("state_file", "")).strip()
        self.ros_state_path = Path(ros_state_file).expanduser().resolve() if ros_state_file else None
        self.ros_monitor_poll_sec = max(0.02, self._float(self.ros_monitor.get("poll_sec"), 0.05))

        match = as_dict(config.get("match_control"))
        self.match_control_enabled = bool(match.get("enabled", False))
        self.match_duration_sec = max(1, int(match.get("duration_sec", 420)))
        control_file = str(match.get("control_file", "")).strip()
        self.control_path = Path(control_file).expanduser().resolve() if control_file else None
        self.control_read_offset = 0
        if self.control_path is not None and self.control_path.exists():
            try:
                self.control_read_offset = int(self.control_path.stat().st_size)
            except OSError:
                pass
        self.last_control_status = "idle"
        initial_time_left = records[0].time_left or self.match_duration_sec
        self.match_time_left_sec = float(max(0, min(self.match_duration_sec, int(initial_time_left))))
        self.last_trace_time_left = int(round(self.match_time_left_sec))
        self.match_started = False
        self.match_started_by_user = False
        self.match_running = False

    @staticmethod
    def _float(value: Any, default: float = 0.0) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return default
        return number if math.isfinite(number) else default

    def controls_available(self) -> bool:
        return bool(self.match_control_enabled and self.control_path is not None and self.follow)

    def update(self, dt: float) -> None:
        now = time.perf_counter()
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
            self.current_time += max(0.0, dt) * self.playback_speed
            if self.current_time >= self.times[-1]:
                self.current_time = self.times[-1]
                if not self.follow:
                    self.playing = False
            self.current_index = min(len(self.records) - 1, max(0, bisect.bisect_right(self.times, self.current_time) - 1))
        self.advance_visual_units(dt)

    def _supply_point(self) -> tuple[int, int] | None:
        for goal in self.config.get("goals", []):
            if not isinstance(goal, dict) or str(goal.get("name", "")) != "SelfBase":
                continue
            point = goal.get(self.active_team)
            if isinstance(point, (list, tuple)) and len(point) >= 2:
                try:
                    return int(point[0]), int(point[1])
                except (TypeError, ValueError):
                    return None
        return None

    def _publish_simulated_self_health(self, hp: int) -> None:
        if self.control_path is None:
            return
        now = time.perf_counter()
        if hp == self._last_self_health_command_hp and now - self._last_self_health_command_time < 0.1:
            return
        append_command(self.control_path, "set_self_health", {"hp": hp})
        self._last_self_health_command_time = now
        self._last_self_health_command_hp = hp

    def advance_visual_units(self, dt: float) -> None:
        if not self.visual_goal_tracking or not self.match_running or dt <= 0.0 or not self.records:
            return
        goal = record_position(self.records[self.current_index], self.goals)
        if goal is None:
            return
        step = self.visual_unit_speed_cm_s * min(float(dt), 0.25)
        if step <= 0.0:
            return
        targets = [
            unit
            for unit in self.sim_input_state.scene.units.values()
            if unit.side == "friend" and unit.unit_key == "sentry"
        ]
        for unit in targets:
            dx = float(goal[0]) - unit.x
            dy = float(goal[1]) - unit.y
            distance = math.hypot(dx, dy)
            if distance <= step or distance < 1e-6:
                next_x = int(round(goal[0]))
                next_y = int(round(goal[1]))
            else:
                next_x = int(round(unit.x + dx / distance * step))
                next_y = int(round(unit.y + dy / distance * step))
            self.sim_input_state.apply_command(
                "place_unit",
                {
                    "entity_id": unit.entity_id,
                    "side": unit.side,
                    "unit_key": unit.unit_key,
                    "hp": unit.hp,
                    "x": next_x,
                    "y": next_y,
                },
            )
            if self.supply_heal_enabled:
                supply_point = self._supply_point()
                if supply_point is not None and math.hypot(next_x - supply_point[0], next_y - supply_point[1]) <= self.supply_radius_cm:
                    archetype = self.sim_input_state.catalog.unit_by_key(unit.unit_key)
                    max_hp = max(1, int(archetype.max_hp))
                    healed_hp = min(max_hp, int(round(unit.hp + max_hp * self.supply_heal_rate_ratio * dt)))
                    if healed_hp > unit.hp:
                        self.sim_input_state.apply_command(
                            "place_unit",
                            {
                                "entity_id": unit.entity_id,
                                "side": unit.side,
                                "unit_key": unit.unit_key,
                                "hp": healed_hp,
                                "x": next_x,
                                "y": next_y,
                            },
                        )
                        self._publish_simulated_self_health(healed_hp)

    def run(self, streamer: Any) -> None:
        """Publish state until interrupted; no display server is required."""
        last = time.perf_counter()
        try:
            while True:
                now = time.perf_counter()
                self.update(now - last)
                last = now
                streamer.update_metadata(self.web_status_metadata())
                time.sleep(1.0 / 60.0)
        except KeyboardInterrupt:
            return

    def apply_local_match_command(self, command: str, payload: dict[str, Any]) -> None:
        cmd = str(command).strip().lower()
        if cmd == "start":
            if self.match_time_left_sec <= 0.0:
                self.match_time_left_sec = float(self.match_duration_sec)
            self.match_started = True
            self.match_started_by_user = True
            self.match_running = True
            self.playing = True
        elif cmd == "pause":
            self.match_running = False
            self.playing = False
        elif cmd == "reset":
            self.match_time_left_sec = float(self.match_duration_sec)
            self.match_started = False
            self.match_started_by_user = False
            self.match_running = False
            self.playing = False
            self.live_goal_history.clear()
            self.sim_input_state.reset()
            self.active_team = self.sim_input_state.team
        elif cmd in {"rewind", "forward", "set_time_left"}:
            seconds = self._float(payload.get("seconds"), 0.0)
            if cmd == "rewind":
                self.match_time_left_sec += max(0.0, seconds)
            elif cmd == "forward":
                self.match_time_left_sec -= max(0.0, seconds)
            else:
                self.match_time_left_sec = seconds
            self.match_time_left_sec = max(0.0, min(float(self.match_duration_sec), self.match_time_left_sec))
            if self.match_time_left_sec <= 0.0:
                self.match_running = False

    def poll_control_commands(self) -> None:
        if not self.controls_available():
            return
        commands, self.control_read_offset = read_commands(self.control_path, self.control_read_offset)
        for payload in commands:
            command = command_name(payload)
            if not command:
                continue
            self.apply_local_match_command(command, payload)
            self.sim_input_state.apply_command(command, payload)
            if command == "set_team":
                self.active_team = self.sim_input_state.team
            self.last_control_status = f"cmd={command}"

    def tick_match_clock(self, dt: float) -> None:
        if not self.controls_available() or not self.match_started or not self.match_running:
            return
        self.match_time_left_sec = max(0.0, self.match_time_left_sec - max(0.0, dt))
        if self.match_time_left_sec <= 0.0:
            self.match_running = False

    def poll_trace_updates(self) -> None:
        if self.trace_path is None or not self.trace_path.exists():
            return
        try:
            new_records, bad_lines, new_offset = load_trace_incremental(
                self.trace_path, self.goal_names, self.follow_offset, len(self.records)
            )
        except OSError:
            return
        self.follow_offset = new_offset
        self.bad_lines += bad_lines
        if not new_records:
            return
        at_tail = self.current_index >= len(self.records) - 1
        self.records.extend(new_records)
        self.times.extend(record.t for record in new_records)
        self.changes = build_changes(self.records)
        latest_time_left = max(0, min(self.match_duration_sec, int(new_records[-1].time_left)))
        if latest_time_left != self.last_trace_time_left:
            self.match_time_left_sec = float(latest_time_left)
            self.last_trace_time_left = latest_time_left
            self.match_started = self.match_started or latest_time_left < self.match_duration_sec
            if latest_time_left <= 0:
                self.match_running = False
        if at_tail:
            self.current_index = len(self.records) - 1
            self.current_time = self.records[self.current_index].t

    def poll_ros_topic_state(self) -> None:
        if self.ros_state_path is None or not self.ros_state_path.exists():
            return
        try:
            payload = json.loads(self.ros_state_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return
        self.live_ros_topics = as_dict(as_dict(payload).get("topics"))
        self.live_ros_wall_time = self._float(as_dict(payload).get("wall_time"), 0.0)
        pos = parse_position(as_dict(self.live_ros_topics.get("/goal_pose")).get("value"))
        if pos is None:
            pos = parse_position(as_dict(self.live_ros_topics.get("/ly/navi/goal_pos")).get("value"))
        if pos is not None and (not self.live_goal_history or math.hypot(pos[0] - self.live_goal_history[-1][0], pos[1] - self.live_goal_history[-1][1]) >= 0.5):
            self.live_goal_history.append(pos)
            limit = max(1, int(self.timeline_config.get("path_history_limit", 500)))
            del self.live_goal_history[:-limit]

    def path_points(self) -> list[tuple[float, float]]:
        if self.live_goal_history:
            return list(self.live_goal_history)
        points: list[tuple[float, float]] = []
        last_goal: tuple[int, str, tuple[float, float] | None] | None = None
        limit = int(self.timeline_config.get("path_history_limit", 500))
        for record in self.records[max(0, self.current_index - limit) : self.current_index + 1]:
            if record.output.route_key == last_goal:
                continue
            pos = record_position(record, self.goals)
            if pos is not None:
                points.append(pos)
                last_goal = record.output.route_key
        return points

    def web_status_metadata(self) -> dict[str, Any]:
        record = self.records[self.current_index]
        input_state = self.sim_input_state.snapshot(team=self.active_team, goals=self.goals)
        current_record = record_status_payload(record)
        return {
            "trace": {
                "name": self.trace_path.name if self.trace_path is not None else "",
                "follow": self.follow, "records": len(self.records), "bad_lines": self.bad_lines,
                "duration_sec": self.records[-1].t - self.records[0].t,
                "tick_range": {"first": self.records[0].tick, "last": self.records[-1].tick},
            },
            "replay": {
                "playing": self.playing, "speed": self.playback_speed, "current_index": self.current_index,
                "current_record": self.current_index + 1, "total_records": len(self.records),
                "match_time_left": round(self.match_time_left_sec, 2),
                "match_duration_sec": self.match_duration_sec,
                "match_running": self.match_running,
                "match_started": self.match_started,
                "match_started_user": self.match_started_by_user,
                "controls_available": self.controls_available(),
            },
            "current_record": current_record,
            "scene": {"team": input_state.get("team", self.active_team), "ownership_mode": input_state.get("ownership_mode", "mock"), "selected_entity_id": input_state.get("selected_entity_id"), "show_trace_units": not bool(input_state.get("units")), "units": input_state.get("units", []), "structures": input_state.get("structures", [])},
            "decision": {"goal": {"id": record.output.goal_id, "name": record.output.goal_name, "position_cm": point_payload(record_position(record, self.goals))}, "intent": current_record["intent"], "route_cm": [point_payload(point) for point in self.path_points()]},
            "control_output": current_record["control_output"],
            "simulator_inputs": {"enabled": self.simulator_inputs_enabled, "status": self.last_control_status, "state": input_state},
        }
