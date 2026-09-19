from __future__ import annotations

import copy
import json
from inspect import getfile
from pathlib import Path

from simulator.config import goals_by_id, load_config, resolve_path
from simulator.control_bus import append_command
from simulator.interactive_inputs import load_unit_scene_file
from simulator.main import make_bootstrap_record
from simulator.runtime import SimulationRuntime


def test_headless_runtime_supply_heals_3v3_sentry_and_syncs_decision_health(tmp_path: Path) -> None:
    config = copy.deepcopy(load_config(resolve_path("src/simulator/config/league_3v3.yaml")))
    control_file = tmp_path / "control.jsonl"
    config["match_control"] = {"enabled": True, "duration_sec": 420, "control_file": control_file.as_posix()}
    goals = goals_by_id(config)
    names = {goal_id: str(goal.get("name", "")) for goal_id, goal in goals.items()}
    record = make_bootstrap_record(config, goals, names)
    runtime = SimulationRuntime(
        records=[record],
        config=config,
        goals=goals,
        bad_lines=0,
        trace_path=Path("/tmp/decision_trace.jsonl"),
        goal_names=names,
        follow=True,
        follow_poll_sec=0.25,
        follow_offset=0,
        start_paused=True,
        speed=None,
    )
    runtime.match_running = True
    runtime.sim_input_state.apply_command(
        "place_unit",
        {
            "entity_id": "friend:sentry:offline",
            "side": "friend",
            "unit_key": "sentry",
            "hp": 100,
            "x": 75,
            "y": 700,
        },
    )

    runtime.advance_visual_units(1.0)
    payload = runtime.web_status_metadata()
    sentry = next(unit for unit in payload["scene"]["units"] if unit["type"] == "Sentry")
    commands = [json.loads(line) for line in control_file.read_text(encoding="utf-8").splitlines()]

    assert sentry["hp"] == 200
    assert commands[-1] == {"ts": commands[-1]["ts"], "command": "set_self_health", "hp": 200}


def test_headless_runtime_reset_restores_3v3_initial_sentry(tmp_path: Path) -> None:
    config = copy.deepcopy(load_config(resolve_path("src/simulator/config/league_3v3.yaml")))
    goals = goals_by_id(config)
    names = {goal_id: str(goal.get("name", "")) for goal_id, goal in goals.items()}
    record = make_bootstrap_record(config, goals, names)
    runtime = SimulationRuntime(
        records=[record],
        config=config,
        goals=goals,
        bad_lines=0,
        trace_path=Path("/tmp/decision_trace.jsonl"),
        goal_names=names,
        follow=True,
        follow_poll_sec=0.25,
        follow_offset=0,
        start_paused=True,
        speed=None,
    )
    runtime.sim_input_state.apply_command(
        "place_unit",
        {
            "entity_id": "friend:sentry:offline",
            "side": "friend",
            "unit_key": "sentry",
            "hp": 111,
            "x": 600,
            "y": 400,
        },
    )

    runtime.apply_local_match_command("reset", {})
    payload = runtime.web_status_metadata()
    sentry = next(unit for unit in payload["scene"]["units"] if unit["type"] == "Sentry")

    assert sentry["position_cm"] == {"x": 75, "y": 700}
    assert sentry["hp"] == 400


def test_headless_runtime_advances_shared_scene_state_without_pygame(tmp_path: Path) -> None:
    config = copy.deepcopy(load_config(resolve_path("src/simulator/config/default.yaml")))
    config["simulator_inputs"]["initial_units"] = load_unit_scene_file(
        resolve_path("src/simulator/sample/unit_scenes/tactical_board.yaml")
    )
    control_file = tmp_path / "control.jsonl"
    config["match_control"] = {"enabled": True, "duration_sec": 420, "control_file": control_file.as_posix()}
    goals = goals_by_id(config)
    names = {goal_id: str(goal.get("name", "")) for goal_id, goal in goals.items()}
    record = make_bootstrap_record(config, goals, names)
    runtime = SimulationRuntime(
        records=[record],
        config=config,
        goals=goals,
        bad_lines=0,
        trace_path=Path("/tmp/decision_trace.jsonl"),
        goal_names=names,
        follow=True,
        follow_poll_sec=0.25,
        follow_offset=0,
        start_paused=True,
        speed=None,
    )

    append_command(control_file, "start")
    append_command(control_file, "set_team", {"team": "blue"})
    append_command(control_file, "set_structure_health", {"side": "friend", "structure": "base", "hp": 3300})
    append_command(control_file, "set_structure_health", {"side": "friend", "structure": "outpost", "hp": 1200})
    append_command(
        control_file,
        "place_unit",
        {"entity_id": "enemy:hero", "side": "enemy", "unit_key": "hero", "hp": 200, "x": 1777, "y": 888},
    )
    runtime.poll_control_commands()
    runtime.tick_match_clock(1.0)
    payload = runtime.web_status_metadata()

    assert runtime.match_running is True
    assert payload["replay"]["controls_available"] is True
    assert payload["replay"]["match_time_left"] < 420
    assert payload["scene"]["team"] == "blue"
    friend_base = next(item for item in payload["scene"]["structures"] if item["key"] == "friend_base")
    friend_outpost = next(item for item in payload["scene"]["structures"] if item["key"] == "friend_outpost")
    moved_enemy_hero = next(item for item in payload["scene"]["units"] if item["entity_id"] == "enemy:hero")
    assert friend_base["hp"] == 3300
    assert friend_outpost["hp"] == 1200
    assert moved_enemy_hero["position_cm"] == {"x": 1777, "y": 888}
    assert friend_base["field_side"] == "blue"
    assert moved_enemy_hero["field_side"] == "red"
    assert "import pygame" not in Path(getfile(SimulationRuntime)).read_text(encoding="utf-8").lower()


def test_match_clock_rewind_and_forward_only_change_remaining_time(tmp_path: Path) -> None:
    config = copy.deepcopy(load_config(resolve_path("src/simulator/config/default.yaml")))
    config["match_control"] = {
        "enabled": True,
        "duration_sec": 420,
        "control_file": (tmp_path / "control.jsonl").as_posix(),
    }
    goals = goals_by_id(config)
    names = {goal_id: str(goal.get("name", "")) for goal_id, goal in goals.items()}
    record = make_bootstrap_record(config, goals, names)
    runtime = SimulationRuntime(
        records=[record],
        config=config,
        goals=goals,
        bad_lines=0,
        trace_path=tmp_path / "decision_trace.jsonl",
        goal_names=names,
        follow=True,
        follow_poll_sec=0.25,
        follow_offset=0,
        start_paused=True,
        speed=None,
    )
    original_scene = runtime.sim_input_state.snapshot()

    runtime.apply_local_match_command("start", {})
    runtime.apply_local_match_command("forward", {"seconds": 60})
    runtime.apply_local_match_command("rewind", {"seconds": 25})
    runtime.apply_local_match_command("set_time_left", {"seconds": 30})

    payload = runtime.web_status_metadata()

    assert payload["replay"]["match_time_left"] == 30
    assert payload["replay"]["match_duration_sec"] == 420
    assert payload["replay"]["match_running"] is True
    assert payload["replay"]["match_started"] is True
    assert payload["replay"]["match_started_user"] is True
    assert runtime.sim_input_state.snapshot() == original_scene

    runtime.apply_local_match_command("pause", {})
    paused = runtime.web_status_metadata()["replay"]
    assert paused["match_running"] is False
    assert paused["match_started"] is True
    assert paused["match_started_user"] is True

    runtime.apply_local_match_command("reset", {})
    reset = runtime.web_status_metadata()["replay"]
    assert reset["match_running"] is False
    assert reset["match_started"] is False
    assert reset["match_started_user"] is False
