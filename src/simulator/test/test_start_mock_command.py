from __future__ import annotations

from pathlib import Path

import pytest

from simulator.start import (
    MOCK_PRESET_DESCRIPTIONS,
    MOCK_PRESETS,
    build_mock_command,
    build_mock_sequence_command,
    build_live_viewer_command,
    parse_args,
    print_mock_presets,
    print_mock_sequences,
    print_unit_scenes,
)


REPO_ROOT = Path(__file__).resolve().parents[3]


def test_live_viewer_command_forwards_manual_ros_input_ownership(tmp_path: Path) -> None:
    command = build_live_viewer_command(
        tmp_path / "trace.jsonl",
        0.25,
        "127.0.0.1",
        9001,
        12.0,
        80,
        "",
        420,
        tmp_path.joinpath("topics.json").as_posix(),
        "",
        input_owner="manual_ros",
    )

    assert "--input-owner" in command
    assert command[command.index("--input-owner") + 1] == "manual_ros"


def test_live_viewer_command_forwards_viewer_config(tmp_path: Path) -> None:
    config_path = tmp_path / "league_3v3.yaml"
    config_path.write_text("schema: ly_simulator_config_v1\n", encoding="utf-8")

    command = build_live_viewer_command(
        tmp_path / "trace.jsonl",
        0.25,
        "",
        0,
        0.0,
        0,
        "",
        0,
        "",
        "",
        viewer_config=str(config_path),
    )

    assert "--config" in command
    assert command[command.index("--config") + 1] == str(config_path.resolve())


def test_build_mock_command_forwards_decision_context_inputs(tmp_path: Path) -> None:
    scene_path = tmp_path / "scene.json"
    scene_path.write_text('{"units":[]}\n', encoding="utf-8")
    control_path = tmp_path / "control.jsonl"
    args = parse_args(
        [
            "--offline-decision",
            "--mock-team",
            "blue",
            "--mock-self-health",
            "315",
            "--mock-enemy-health",
            "180",
            "--mock-enemy-outpost-health",
            "0",
            "--mock-enemy-base-health",
            "4200",
            "--mock-team-buff-attack",
            "2",
            "--mock-team-buff-defence",
            "1",
            "--mock-team-buff-remaining-energy",
            "41",
            "--mock-event-center-gain-point-status",
            "1",
            "--mock-event-self-small-energy-status",
            "2",
            "--mock-event-self-fortress-gain-point-status",
            "1",
            "--mock-event-self-base-gain-point-status",
            "true",
            "--mock-sentry-can-activate-energy",
            "true",
            "--mock-rfid-center-gain-point",
            "true",
            "--mock-rfid-self-outpost",
            "true",
            "--mock-rfid-enemy-tunnel",
            "true",
            "--mock-navi-reached",
            "true",
            "--mock-navi-reachable",
            "false",
            "--mock-navi-should-rotate",
            "false",
            "--mock-self-position-x",
            "1220",
            "--mock-self-position-y",
            "760",
            "--mock-publish-uwb-position",
            "true",
            "--mock-uwb-position-x",
            "1212",
            "--mock-uwb-position-y",
            "748",
            "--mock-official-target-valid",
            "true",
            "--mock-official-target-x",
            "1505",
            "--mock-official-target-y",
            "905",
            "--mock-official-target-armor-type",
            "6",
            "--mock-gimbal-fire-status",
            "1",
            "--mock-gimbal-follow-mode",
            "true",
            "--mock-cap-v",
            "22",
            "--mock-navi-lower-head",
            "1",
            "--mock-navi-vel-x",
            "0.35",
            "--mock-navi-vel-y",
            "-0.2",
            "--mock-bullet-initial-speed",
            "23.4",
            "--mock-bullet-projectile-allowance-17mm",
            "120",
            "--mock-external-aim",
            "true",
            "--mock-external-aim-fire",
            "false",
            "--mock-external-aim-yaw",
            "7.5",
            "--mock-external-aim-pitch",
            "-1.2",
            "--mock-external-aim-target-id",
            "6",
            "--control-file",
            str(control_path),
            "--unit-scene",
            str(scene_path),
        ]
    )

    _command, shell_cmd = build_mock_command(REPO_ROOT, args)

    assert "simulator.mock_inputs" in shell_cmd
    assert "--team blue" in shell_cmd
    assert "--self-health 315" in shell_cmd
    assert "--enemy-health 180" in shell_cmd
    assert "--enemy-outpost-health 0" in shell_cmd
    assert "--enemy-base-health 4200" in shell_cmd
    assert "--team-buff-attack 2" in shell_cmd
    assert "--team-buff-defence 1" in shell_cmd
    assert "--team-buff-remaining-energy 41" in shell_cmd
    assert "--event-center-gain-point-status 1" in shell_cmd
    assert "--event-self-small-energy-status 2" in shell_cmd
    assert "--event-self-fortress-gain-point-status 1" in shell_cmd
    assert "--event-self-base-gain-point-status true" in shell_cmd
    assert "--sentry-can-activate-energy true" in shell_cmd
    assert "--rfid-center-gain-point true" in shell_cmd
    assert "--rfid-self-outpost true" in shell_cmd
    assert "--rfid-enemy-tunnel true" in shell_cmd
    assert "--navi-reached true" in shell_cmd
    assert "--navi-reachable false" in shell_cmd
    assert "--navi-should-rotate false" in shell_cmd
    assert "--self-position-x 1220" in shell_cmd
    assert "--self-position-y 760" in shell_cmd
    assert "--publish-uwb-position true" in shell_cmd
    assert "--uwb-position-x 1212" in shell_cmd
    assert "--uwb-position-y 748" in shell_cmd
    assert "--official-target-valid true" in shell_cmd
    assert "--official-target-x 1505" in shell_cmd
    assert "--official-target-y 905" in shell_cmd
    assert "--official-target-armor-type 6" in shell_cmd
    assert "--gimbal-fire-status 1" in shell_cmd
    assert "--gimbal-follow-mode true" in shell_cmd
    assert "--mock-cap-v 22" in shell_cmd
    assert "--mock-navi-lower-head 1" in shell_cmd
    assert "--mock-navi-vel-x 0.35" in shell_cmd
    assert "--mock-navi-vel-y -0.2" in shell_cmd
    assert "--mock-bullet-initial-speed 23.4" in shell_cmd
    assert "--mock-bullet-projectile-allowance-17mm 120" in shell_cmd
    assert "--mock-external-aim true" in shell_cmd
    assert "--mock-external-aim-fire false" in shell_cmd
    assert "--mock-external-aim-yaw 7.5" in shell_cmd
    assert "--mock-external-aim-pitch -1.2" in shell_cmd
    assert "--mock-external-aim-target-id 6" in shell_cmd
    assert str(control_path.resolve()) in shell_cmd
    assert str(scene_path.resolve()) in shell_cmd


def test_mock_preset_expands_common_decision_context() -> None:
    args = parse_args(["--offline-decision", "--mock-preset", "buff-ready"])

    _command, shell_cmd = build_mock_command(REPO_ROOT, args)

    assert args.mode == "regional"
    assert "--mock-external-aim true" in shell_cmd
    assert "--time-left 411" in shell_cmd
    assert "--ammo-left 43" in shell_cmd
    assert "--sentry-can-activate-energy true" in shell_cmd
    assert "--event-self-small-energy-status 2" in shell_cmd
    assert "--team-buff-attack 1" in shell_cmd
    assert "--team-buff-remaining-energy 35" in shell_cmd
    assert "--rfid-center-gain-point true" in shell_cmd
    assert "--self-position-x 924" in shell_cmd
    assert "--self-position-y 1388" in shell_cmd


def test_uwb_fusion_mock_preset_enables_dedicated_uwb_position() -> None:
    args = parse_args(["--offline-decision", "--mock-preset", "uwb-fusion"])

    _command, shell_cmd = build_mock_command(REPO_ROOT, args)

    assert args.mode == "regional"
    assert "--time-left 386" in shell_cmd
    assert "--self-position-x 1110" in shell_cmd
    assert "--self-position-y 720" in shell_cmd
    assert "--publish-uwb-position true" in shell_cmd
    assert "--uwb-position-x 1220" in shell_cmd
    assert "--uwb-position-y 760" in shell_cmd


def test_bullet_resource_mock_preset_enables_bullet_info_fields() -> None:
    args = parse_args(["--offline-decision", "--mock-preset", "bullet-resource"])

    _command, shell_cmd = build_mock_command(REPO_ROOT, args)

    assert args.mode == "regional"
    assert "--time-left 392" in shell_cmd
    assert "--ammo-left 88" in shell_cmd
    assert "--mock-bullet-initial-speed 23.4" in shell_cmd
    assert "--mock-bullet-has-shoot-data true" in shell_cmd
    assert "--mock-bullet-type 1" in shell_cmd
    assert "--mock-bullet-shooter-number 7" in shell_cmd
    assert "--mock-bullet-launching-frequency 9" in shell_cmd
    assert "--mock-bullet-projectile-allowance-17mm 118" in shell_cmd
    assert "--mock-bullet-projectile-allowance-42mm 6" in shell_cmd
    assert "--mock-bullet-remaining-gold-coin 14" in shell_cmd
    assert "--mock-bullet-projectile-allowance-fortress-17mm 32" in shell_cmd


def test_explicit_mock_flags_override_mock_preset_values() -> None:
    args = parse_args(
        [
            "--offline-decision",
            "--mode",
            "league",
            "--mock-preset",
            "official-target-sentry",
            "--mock-official-target-x",
            "1800",
            "--mock-official-target-armor-type",
            "1",
        ]
    )

    _command, shell_cmd = build_mock_command(REPO_ROOT, args)

    assert args.mode == "league"
    assert "--official-target-valid true" in shell_cmd
    assert "--official-target-x 1800" in shell_cmd
    assert "--official-target-y 905" in shell_cmd
    assert "--official-target-armor-type 1" in shell_cmd


def test_multi_unit_mock_preset_uses_sample_unit_scene() -> None:
    args = parse_args(["--offline-decision", "--mock-preset", "multi-unit-regional"])

    _command, shell_cmd = build_mock_command(REPO_ROOT, args)

    assert args.mode == "regional"
    assert "--mock-external-aim true" in shell_cmd
    assert "--enemy-outpost-health 1100" in shell_cmd
    assert str((REPO_ROOT / "src" / "simulator" / "sample" / "unit_scene.json").resolve()) in shell_cmd


def test_full_roster_mock_preset_uses_full_roster_scene() -> None:
    args = parse_args(["--offline-decision", "--mock-preset", "full-roster-regional"])

    _command, shell_cmd = build_mock_command(REPO_ROOT, args)

    assert args.mode == "regional"
    assert "--mock-external-aim true" in shell_cmd
    assert "--ammo-left 120" in shell_cmd
    assert "--enemy-outpost-health 1500" in shell_cmd
    assert str((REPO_ROOT / "src" / "simulator" / "sample" / "unit_scenes" / "full_roster.json").resolve()) in shell_cmd
    assert "start_sentry_all.sh" not in shell_cmd


def test_invalid_mock_preset_is_rejected() -> None:
    with pytest.raises(SystemExit):
        parse_args(["--mock-preset", "missing"])


def test_mock_preset_requires_offline_decision() -> None:
    with pytest.raises(SystemExit):
        parse_args(["--mock-preset", "buff-ready"])


def test_mock_sequence_requires_offline_decision(tmp_path: Path) -> None:
    sequence_path = tmp_path / "sequence.json"
    sequence_path.write_text('{"actions":[]}\n', encoding="utf-8")

    with pytest.raises(SystemExit):
        parse_args(["--mock-sequence", str(sequence_path)])


def test_build_mock_sequence_command_uses_control_bus_file(tmp_path: Path) -> None:
    sequence_path = tmp_path / "sequence.json"
    control_path = tmp_path / "control.jsonl"
    sequence_path.write_text('{"actions":[{"at_sec":0,"command":"start"}]}\n', encoding="utf-8")
    args = parse_args(
        [
            "--offline-decision",
            "--mock-sequence",
            str(sequence_path),
            "--control-file",
            str(control_path),
            "--mock-sequence-poll-sec",
            "0.02",
        ]
    )

    _command, shell_cmd = build_mock_sequence_command(
        REPO_ROOT,
        sequence_path.resolve(),
        args.control_file,
        args.mock_sequence_poll_sec,
    )

    assert "simulator.mock_sequence" in shell_cmd
    assert str(sequence_path.resolve()) in shell_cmd
    assert str(control_path.resolve()) in shell_cmd
    assert "--poll-sec 0.02" in shell_cmd
    assert "start_sentry_all.sh" not in shell_cmd


def test_dry_run_validates_mock_sequence_before_launch(tmp_path: Path, capsys) -> None:
    sequence_path = tmp_path / "sequence.json"
    trace_path = tmp_path / "trace.jsonl"
    sequence_path.write_text('{"actions":[{"at_sec":0,"command":"start"}]}\n', encoding="utf-8")

    from simulator.start import main

    assert main(["--offline-decision", "--mock-sequence", str(sequence_path), "--trace", str(trace_path), "--dry-run"]) == 0
    output = capsys.readouterr().out

    assert f"mock sequence: {sequence_path.resolve()}" in output
    assert "mock sequence actions: 1" in output
    assert "mock sequence command:" in output


def test_dry_run_rejects_invalid_mock_sequence(tmp_path: Path, capsys) -> None:
    sequence_path = tmp_path / "sequence.json"
    trace_path = tmp_path / "trace.jsonl"
    sequence_path.write_text('{"actions":[{"at_sec":0,"command":"missing"}]}\n', encoding="utf-8")

    from simulator.start import main

    assert main(["--offline-decision", "--mock-sequence", str(sequence_path), "--trace", str(trace_path), "--dry-run"]) == 2
    error = capsys.readouterr().err

    assert "failed to load mock sequence" in error
    assert "unsupported command" in error


def test_dry_run_validates_unit_scene_before_launch(tmp_path: Path, capsys) -> None:
    scene_path = tmp_path / "scene.json"
    trace_path = tmp_path / "trace.jsonl"
    scene_path.write_text(
        '{"units":[{"side":"enemy","type":"Hero","hp":120,"x":2555,"y":900}]}\n',
        encoding="utf-8",
    )

    from simulator.start import main

    assert main(["--offline-decision", "--unit-scene", str(scene_path), "--trace", str(trace_path), "--dry-run"]) == 0
    output = capsys.readouterr().out

    assert f"unit scene: {scene_path.resolve()}" in output
    assert "unit scene units: 1" in output
    assert str(scene_path.resolve()) in output


def test_dry_run_rejects_invalid_unit_scene(tmp_path: Path, capsys) -> None:
    scene_path = tmp_path / "scene.json"
    trace_path = tmp_path / "trace.jsonl"
    scene_path.write_text('{"units": ', encoding="utf-8")

    from simulator.start import main

    assert main(["--offline-decision", "--unit-scene", str(scene_path), "--trace", str(trace_path), "--dry-run"]) == 2
    error = capsys.readouterr().err

    assert "failed to load unit scene" in error
    assert "invalid JSON unit scene" in error


def test_mock_preset_catalog_has_descriptions() -> None:
    assert {
        "none",
        "buff-ready",
        "outpost-dead",
        "nav-unreachable",
        "official-target-sentry",
        "uwb-fusion",
        "bullet-resource",
    } <= set(MOCK_PRESETS)

    full_roster_description = MOCK_PRESET_DESCRIPTIONS["full-roster-regional"]
    assert "formal health-unit HP mapping" in full_roster_description
    assert "placed-unit PositionData" in full_roster_description


def test_list_mock_presets_prints_overlay_flags(capsys) -> None:
    assert print_mock_presets() == 0
    output = capsys.readouterr().out

    assert "buff-ready:" in output
    assert "overlay:" in output
    assert "--mock-external-aim true" in output
    assert "--mock-sentry-can-activate-energy true" in output
    assert "--unit-scene src/simulator/sample/unit_scene.json" in output


def test_list_mock_sequences_does_not_require_offline_decision(capsys) -> None:
    args = parse_args(["--list-mock-sequences"])

    assert args.list_mock_sequences is True
    assert print_mock_sequences() == 0
    output = capsys.readouterr().out

    assert "Mock sequence samples:" in output
    assert "regional_timed_context.json: 14 actions" in output
    assert "buff_timeout_context.json: 10 actions" in output
    assert "low_resource_recovery_exit.json: 12 actions" in output


def test_start_main_lists_mock_sequences(capsys) -> None:
    from simulator.start import main

    assert main(["--list-mock-sequences"]) == 0
    output = capsys.readouterr().out

    assert "Mock sequence samples:" in output
    assert "regional_timed_context.json" in output


def test_list_unit_scenes_does_not_require_offline_decision(capsys) -> None:
    args = parse_args(["--list-unit-scenes"])

    assert args.list_unit_scenes is True
    assert print_unit_scenes() == 0
    output = capsys.readouterr().out

    assert "Unit scene samples:" in output
    assert "unit_scene.json: 7 units" in output
    assert "full_roster.json: 14 units" in output


def test_start_main_lists_unit_scenes(capsys) -> None:
    from simulator.start import main

    assert main(["--list-unit-scenes"]) == 0
    output = capsys.readouterr().out

    assert "Unit scene samples:" in output
    assert "full_roster.json" in output
