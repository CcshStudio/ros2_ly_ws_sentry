from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

from .config import goals_by_id, load_config, load_plugin_points, resolve_path
from .foxglove_export import export_records_to_mcap
from .interactive_inputs import load_unit_scene_file
from .trace import build_changes, load_trace, load_trace_incremental, normalize_record
from .validation import format_validation, validation_report, validate_records
from .viewer import Viewer


def league_3v3_map_objects(goals: dict[int, dict[str, Any]]) -> list[dict[str, Any]]:
    """Return presentation-only red/blue target markers for the 3v3 field."""
    labels = {
        "SelfBase": "己方基地",
        "CenterAnchor": "中央控制区锚点",
        "EnemyStartApproach": "敌方基地外围",
        "CenterOwnSide": "己方控制区前沿",
        "CenterTop": "控制区上侧巡逻点",
        "CenterEnemySide": "敌方控制区前沿",
    }
    objects: list[dict[str, Any]] = []
    for goal_id, goal in sorted(goals.items()):
        name = str(goal.get("name", f"Goal{goal_id}"))
        label = labels.get(name, name)
        for field_side in ("red", "blue"):
            point = goal.get(field_side)
            if not isinstance(point, (list, tuple)) or len(point) < 2:
                continue
            if name == "SelfBase":
                label = "红方基地" if field_side == "red" else "蓝方基地"
            elif name == "EnemyStartApproach":
                label = "逼近蓝方基地" if field_side == "red" else "逼近红方基地"
            elif name == "CenterOwnSide":
                label = "红方控制区前沿" if field_side == "red" else "蓝方控制区前沿"
            elif name == "CenterEnemySide":
                label = "蓝方控制区前沿" if field_side == "red" else "红方控制区前沿"
            objects.append({
                "key": f"{name}:{field_side}",
                "label": label,
                "detail": f"{name} · {field_side}",
                "field_side": field_side,
                "x": point[0],
                "y": point[1],
            })
    return objects


def trace_status_payload(trace_path: Path, records: list, bad_lines: int, follow: bool) -> dict:
    first_tick = records[0].tick if records else 0
    last_tick = records[-1].tick if records else 0
    duration = records[-1].t - records[0].t if records else 0.0
    return {
        "name": trace_path.name,
        "follow": bool(follow),
        "records": len(records),
        "bad_lines": int(bad_lines),
        "duration_sec": duration,
        "tick_range": {"first": first_tick, "last": last_tick},
    }


def import_pygame():
    try:
        import pygame  # type: ignore
    except ImportError:
        print(
            "pygame is required. Install it with: python3 -m pip install -r src/simulator/requirements.txt",
            file=sys.stderr,
        )
        raise
    return pygame


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Browser-first simulator for behavior_tree decision JSONL traces.")
    parser.add_argument("trace", nargs="?", default="", help="Decision trace JSONL. Defaults to config paths.sample_trace.")
    parser.add_argument("--config", default="", help="Optional YAML override for viewer layout, colors, field, and goals.")
    parser.add_argument(
        "--input-owner",
        choices=("mock", "manual_ros"),
        default="mock",
        help="Scene input owner; manual_ros renders an observer-only tactical board.",
    )
    parser.add_argument("--map", dest="map_path", default="", help="Basemap image path.")
    parser.add_argument("--points-json", default="", help="Optional tools/maps map_plugin JSON/YAML with point coordinates.")
    parser.add_argument("--start-paused", action="store_true", help="Start playback paused.")
    parser.add_argument("--speed", type=float, default=0.0, help="Initial playback speed. 0 keeps YAML default.")
    parser.add_argument("--follow", action="store_true", help="Follow a growing trace file and update the view in real time.")
    parser.add_argument("--follow-poll", type=float, default=0.25, help="Follow mode poll interval in seconds (default: 0.25).")
    parser.add_argument(
        "--follow-wait",
        type=float,
        default=20.0,
        help="Compatibility option; follow mode now opens the viewer immediately before first trace records arrive.",
    )
    parser.add_argument(
        "--web-stream",
        dest="web_stream",
        action="store_true",
        help="Enable the HTTP Tactical Board (overrides YAML).",
    )
    parser.add_argument(
        "--no-web-stream",
        dest="web_stream",
        action="store_false",
        help="Disable the HTTP Tactical Board and use the native debug renderer.",
    )
    parser.set_defaults(web_stream=None)
    parser.add_argument(
        "--debug-pygame",
        action="store_true",
        help="Open the native pygame debug renderer. The browser Tactical Board remains the production UI.",
    )
    parser.add_argument("--web-host", default="", help="HTTP stream bind host (overrides YAML web_stream.host).")
    parser.add_argument("--web-port", type=int, default=0, help="HTTP stream bind port (overrides YAML web_stream.port).")
    parser.add_argument("--web-fps", type=float, default=0.0, help="Max stream FPS (overrides YAML web_stream.fps).")
    parser.add_argument(
        "--web-jpeg-quality",
        type=int,
        default=0,
        help="JPEG quality in [1,100] (overrides YAML web_stream.jpeg_quality).",
    )
    parser.add_argument(
        "--control-file",
        default="",
        help="Path to match-control command JSONL (overrides YAML match_control.control_file).",
    )
    parser.add_argument(
        "--unit-scene",
        default="",
        help="JSON/YAML unit scene loaded into the Inputs tab at startup.",
    )
    parser.add_argument(
        "--ros-state-file",
        default="",
        help="Path to live ROS topic monitor JSON state file (overrides YAML ros_monitor.state_file).",
    )
    parser.add_argument(
        "--match-duration-sec",
        type=int,
        default=0,
        help="Match duration in seconds (overrides YAML match_control.duration_sec).",
    )
    parser.add_argument("--validate-only", action="store_true", help="Load trace/config and run offline consistency checks, then exit.")
    parser.add_argument(
        "--validate-format",
        choices=("text", "json"),
        default="text",
        help="Output format for --validate-only (default: text).",
    )
    parser.add_argument(
        "--export-foxglove",
        default="",
        help="Write loaded trace records to a Foxglove-readable MCAP file, then exit. Requires optional mcap package.",
    )
    parser.add_argument("--smoke-test", action="store_true", help="Load config, trace, map, and pygame, draw one frame, then exit.")
    parser.add_argument(
        "--smoke-screenshot",
        default="",
        help="With --smoke-test, write the rendered pygame frame to this PNG path before exiting.",
    )
    return parser.parse_args(argv)


def make_bootstrap_record(config: dict, goals: dict[int, dict], goal_names: dict[int, str]):
    field = config.get("field_cm", {}) if isinstance(config.get("field_cm"), dict) else {}
    match_cfg = config.get("match_control", {}) if isinstance(config.get("match_control"), dict) else {}
    ros_monitor_cfg = config.get("ros_monitor", {}) if isinstance(config.get("ros_monitor"), dict) else {}
    duration_sec = int(match_cfg.get("duration_sec", 420) or 420)
    home = goals.get(0, {})
    home_pos = home.get("red") or (0.0, 0.0)
    home_name = str(home.get("name", goal_names.get(0, "Home")))
    raw = {
        "schema": "ly_decision_trace_v1",
        "schema_version": 1,
        "event": "offline_wait_start",
        "tick": 0,
        "t": 0.0,
        "elapsed_sec": 0,
        "field_cm": {
            "width": int(field.get("width", 2800) or 2800),
            "height": int(field.get("height", 1500) or 1500),
            "frame": str(field.get("frame", "left_bottom_origin_cm")),
        },
        "competition_profile": "regional",
        "strategy_mode": "waiting_start",
        "team": "red",
        "aim_mode": "-",
        "target_armor": {"id": 0, "name": "NoTarget", "distance_m": 0.0},
        "units": {
            "friend": [
                {
                    "type_id": 7,
                    "type": "Sentry",
                    "side": "friend",
                    "hp": 400,
                    "max_hp": 400,
                    "distance_m": 0.0,
                    "position_cm": {"x": home_pos[0], "y": home_pos[1]},
                }
            ],
            "enemy": [],
        },
        "navi_goal": {
            "id": 0,
            "base_id": 0,
            "name": home_name,
            "side": "red",
            "publish_allowed": False,
            "publish_enabled": False,
            "speed_level": 0,
            "position_cm": {"x": home_pos[0], "y": home_pos[1]},
        },
        "posture": {
            "command": {"id": 0, "name": "WaitStart"},
            "state": {"id": 0, "name": "WaitStart"},
            "last_reason": "offline viewer opened before first trace",
            "runtime": {
                "current": {"id": 0, "name": "WaitStart"},
                "desired": {"id": 0, "name": "WaitStart"},
                "pending": {"id": 0, "name": "Unknown"},
            },
        },
        "referee": {
            "self_hp": 400,
            "ammo": 0,
            "time_left": duration_sec,
        },
    }
    return normalize_record(raw, 0, goal_names)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.follow_poll <= 0:
        print("--follow-poll must be > 0", file=sys.stderr)
        return 2
    if args.export_foxglove.strip() and args.follow:
        print("--export-foxglove reads a complete trace file and cannot be combined with --follow", file=sys.stderr)
        return 2
    if args.validate_format != "text" and not args.validate_only:
        print("--validate-format requires --validate-only", file=sys.stderr)
        return 2
    if args.smoke_screenshot.strip() and not args.smoke_test:
        print("--smoke-screenshot requires --smoke-test", file=sys.stderr)
        return 2

    config = load_config(resolve_path(args.config) if args.config else None)
    paths = config.get("paths", {}) if isinstance(config.get("paths"), dict) else {}
    web_cfg = config.get("web_stream", {}) if isinstance(config.get("web_stream"), dict) else {}
    match_cfg = config.get("match_control", {}) if isinstance(config.get("match_control"), dict) else {}
    ros_monitor_cfg = config.get("ros_monitor", {}) if isinstance(config.get("ros_monitor"), dict) else {}
    simulator_inputs_cfg = (
        dict(config.get("simulator_inputs", {})) if isinstance(config.get("simulator_inputs"), dict) else {}
    )
    simulator_inputs_cfg["input_owner"] = args.input_owner
    config["simulator_inputs"] = simulator_inputs_cfg

    web_stream_enabled = (
        bool(args.web_stream)
        if args.web_stream is not None
        else bool(web_cfg.get("enabled", True))
    )
    web_host = args.web_host.strip() if args.web_host.strip() else str(web_cfg.get("host", "0.0.0.0"))
    web_port = args.web_port if args.web_port > 0 else int(web_cfg.get("port", 9000))
    if not math.isfinite(args.web_fps):
        print("--web-fps must be > 0", file=sys.stderr)
        return 2
    web_fps = args.web_fps if args.web_fps > 0 else float(web_cfg.get("fps", 12.0))
    web_jpeg_quality = (
        args.web_jpeg_quality if args.web_jpeg_quality > 0 else int(web_cfg.get("jpeg_quality", 80))
    )

    if web_port <= 0 or web_port > 65535:
        print("--web-port must be in [1, 65535]", file=sys.stderr)
        return 2
    if not math.isfinite(web_fps) or web_fps <= 0:
        print("--web-fps must be > 0", file=sys.stderr)
        return 2
    if web_jpeg_quality < 1 or web_jpeg_quality > 100:
        print("--web-jpeg-quality must be in [1, 100]", file=sys.stderr)
        return 2
    if args.match_duration_sec < 0:
        print("--match-duration-sec must be >= 0", file=sys.stderr)
        return 2

    if args.control_file.strip():
        match_cfg["control_file"] = args.control_file.strip()
    if args.match_duration_sec > 0:
        match_cfg["duration_sec"] = int(args.match_duration_sec)
    if match_cfg:
        config["match_control"] = match_cfg
    unit_scene_config = str(simulator_inputs_cfg.get("unit_scene_file", "")).strip()
    unit_scene_arg = args.unit_scene.strip()
    unit_scene_file = unit_scene_arg if unit_scene_arg else unit_scene_config
    if unit_scene_file:
        unit_scene_path = resolve_path(unit_scene_file)
        try:
            simulator_inputs_cfg["initial_units"] = load_unit_scene_file(unit_scene_path)
        except (OSError, RuntimeError, ValueError) as exc:
            print(f"failed to load unit scene {unit_scene_path}: {exc}", file=sys.stderr)
            return 2
        simulator_inputs_cfg["unit_scene_file"] = unit_scene_path.as_posix()
        config["simulator_inputs"] = simulator_inputs_cfg
    if args.ros_state_file.strip():
        ros_monitor_cfg["state_file"] = args.ros_state_file.strip()
    if ros_monitor_cfg:
        config["ros_monitor"] = ros_monitor_cfg
    control_file = str(match_cfg.get("control_file", "")).strip()
    if control_file:
        control_file = str(Path(control_file).expanduser().resolve())
    control_file_path = Path(control_file) if control_file else None
    control_step_sec = int(match_cfg.get("rewind_step_sec", 10) or 10)

    trace_path = resolve_path(args.trace or paths.get("sample_trace", "src/simulator/sample/sample_trace.jsonl"))
    map_path = resolve_path(args.map_path or paths.get("default_map", "tools/maps/basemaps/buff_map_field.png"))
    field_cfg = config.get("field_cm", {}) if isinstance(config.get("field_cm"), dict) else {}
    field_width_cm = int(field_cfg.get("width_cm", field_cfg.get("width", 0)) or 0)
    field_height_cm = int(field_cfg.get("height_cm", field_cfg.get("height", 0)) or 0)
    is_3v3_field = field_width_cm == 1200 and field_height_cm == 800
    if not args.follow and not trace_path.exists():
        print(f"trace file not found: {trace_path}", file=sys.stderr)
        return 2
    needs_viewer = not args.validate_only and not args.export_foxglove.strip()
    if needs_viewer and not map_path.exists():
        print(f"map image not found: {map_path}", file=sys.stderr)
        return 2

    goals = goals_by_id(config)
    if args.points_json:
        goals.update(load_plugin_points(resolve_path(args.points_json)))
    goal_names = {goal_id: str(goal.get("name", f"Goal{goal_id}")) for goal_id, goal in goals.items()}
    stream_map_objects = league_3v3_map_objects(goals) if is_3v3_field else None

    records: list = []
    bad_lines = 0
    follow_offset = 0
    streamer = None

    if web_stream_enabled and not args.validate_only and not args.export_foxglove.strip():
        try:
            from .web_stream import SimulatorWebStream

            streamer = SimulatorWebStream(
                host=web_host,
                port=web_port,
                fps=web_fps,
                jpeg_quality=web_jpeg_quality,
                control_file=control_file,
                default_step_sec=control_step_sec,
                map_path=map_path.as_posix(),
                map_objects=stream_map_objects,
            )
            streamer.start()
            if web_host == "0.0.0.0":
                print(
                    f"tactical board: http://127.0.0.1:{web_port}/ "
                    f"(LAN: http://<your-ip>:{web_port}/)"
                )
            else:
                print(f"tactical board: http://{web_host}:{web_port}/")
        except Exception as exc:
            print(f"web stream disabled: {exc}", file=sys.stderr)
            streamer = None

    if args.follow:
        if trace_path.exists():
            try:
                records, bad_lines, follow_offset = load_trace_incremental(
                    trace_path,
                    goal_names,
                    start_offset=0,
                    start_index=0,
                )
            except OSError:
                records = []
                bad_lines = 0
                follow_offset = 0
        if not records:
            records = [make_bootstrap_record(config, goals, goal_names)]
            bad_lines = 0
            follow_offset = 0
    else:
        try:
            records, bad_lines = load_trace(trace_path, goal_names)
        except (OSError, ValueError) as exc:
            print(str(exc), file=sys.stderr)
            return 2

    validation_issues = validate_records(records, config, bad_lines)
    validation_payload = validation_report(records, validation_issues)
    if streamer is not None:
        streamer.update_metadata(
            {
                "trace": trace_status_payload(trace_path, records, bad_lines, args.follow),
                "validation": validation_payload,
            }
        )
    if args.export_foxglove.strip():
        output_path = Path(args.export_foxglove).expanduser().resolve()
        try:
            export_records_to_mcap(records, output_path)
        except RuntimeError as exc:
            print(str(exc), file=sys.stderr)
            return 2
        print(f"wrote {len(records)} records to {output_path}")
        return 0

    if args.validate_only:
        if args.validate_format == "json":
            print(json.dumps(validation_payload, ensure_ascii=True, indent=2))
        else:
            print(format_validation(records, validation_issues))
        return 2 if any(issue.severity == "error" for issue in validation_issues) else 0

    debug_pygame = bool(args.debug_pygame or args.smoke_test or not web_stream_enabled)
    if not debug_pygame:
        if streamer is None:
            print("browser runtime requires the Tactical Board HTTP server", file=sys.stderr)
            return 2
        from .runtime import SimulationRuntime

        runtime = SimulationRuntime(
            records=records,
            config=config,
            goals=goals,
            bad_lines=bad_lines,
            trace_path=trace_path,
            goal_names=goal_names if args.follow else None,
            follow=args.follow,
            follow_poll_sec=args.follow_poll,
            follow_offset=follow_offset,
            start_paused=True if args.start_paused else None,
            speed=args.speed if args.speed > 0 else None,
        )
        try:
            streamer.update_metadata(runtime.web_status_metadata())
            runtime.run(streamer)
        finally:
            streamer.stop()
        return 0

    pygame = import_pygame()
    pygame.init()
    pygame_initialized = True

    try:
        viewer = Viewer(
            pygame=pygame,
            records=records,
            changes=build_changes(records),
            config=config,
            goals=goals,
            map_path=map_path,
            bad_lines=bad_lines,
            start_paused=True if args.start_paused else None,
            speed=args.speed if args.speed > 0 else None,
            trace_path=trace_path,
            goal_names=goal_names if args.follow else None,
            follow=args.follow,
            follow_poll_sec=args.follow_poll,
            follow_offset=follow_offset,
            streamer=streamer,
        )
        if args.smoke_test:
            viewer.draw()
            if streamer is not None:
                streamer.publish_surface(viewer.screen, pygame)
            if args.smoke_screenshot.strip():
                screenshot_path = Path(args.smoke_screenshot).expanduser().resolve()
                try:
                    screenshot_path.parent.mkdir(parents=True, exist_ok=True)
                    pygame.image.save(viewer.screen, str(screenshot_path))
                except Exception as exc:
                    print(f"failed to write smoke screenshot {screenshot_path}: {exc}", file=sys.stderr)
                    return 2
                print(f"smoke screenshot: {screenshot_path}")
            pygame.display.flip()
            return 0
        viewer.run()
    finally:
        if streamer is not None:
            streamer.stop()
        if pygame_initialized:
            pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
