from __future__ import annotations

import json
from pathlib import Path

import pytest

from simulator.control_bus import read_commands
from simulator.mock_sequence import emit_due_actions, iter_sample_sequences, load_mock_sequence_file, main, parse_mock_sequence


SAMPLE_SEQUENCE_DIR = Path(__file__).resolve().parents[1] / "sample" / "mock_sequences"


def test_bundled_mock_sequences_are_valid_and_non_empty() -> None:
    sequence_paths = sorted(SAMPLE_SEQUENCE_DIR.glob("*.json"))

    assert {path.name for path in sequence_paths} >= {
        "regional_timed_context.json",
        "buff_timeout_context.json",
        "low_resource_recovery_exit.json",
        "official_target_fallback_companion.json",
        "multi_unit_target_priority_rehearsal.json",
        "league_3v3_center_status.json",
    }
    for sequence_path in sequence_paths:
        sequence = load_mock_sequence_file(sequence_path)
        assert sequence.action_count > 0, sequence_path.name
        assert sequence.actions[0].at_sec >= 0.0, sequence_path.name


def test_bundled_mock_sequence_catalog_includes_metadata() -> None:
    samples = iter_sample_sequences(SAMPLE_SEQUENCE_DIR)
    by_name = {sample.name: sample for sample in samples}

    assert set(by_name) >= {
        "regional_timed_context.json",
        "buff_timeout_context.json",
        "low_resource_recovery_exit.json",
        "official_target_fallback_companion.json",
        "multi_unit_target_priority_rehearsal.json",
        "league_3v3_center_status.json",
    }
    assert by_name["regional_timed_context.json"].action_count == 14
    assert "time-varying unit" in by_name["regional_timed_context.json"].description
    assert by_name["buff_timeout_context.json"].action_count == 10
    assert "buff window" in by_name["buff_timeout_context.json"].description
    assert by_name["low_resource_recovery_exit.json"].action_count == 12
    assert "Low-resource recovery" in by_name["low_resource_recovery_exit.json"].description
    assert by_name["official_target_fallback_companion.json"].action_count == 8
    assert "official-target-sentry" in by_name["official_target_fallback_companion.json"].description
    assert by_name["multi_unit_target_priority_rehearsal.json"].action_count == 13
    assert "target-priority" in by_name["multi_unit_target_priority_rehearsal.json"].description
    assert by_name["league_3v3_center_status.json"].action_count == 6
    assert "3v3 center status" in by_name["league_3v3_center_status.json"].description


def test_list_samples_cli_prints_catalog(capsys) -> None:
    assert main(["--list-samples"]) == 0
    output = capsys.readouterr().out

    assert "Mock sequence samples:" in output
    assert "regional_timed_context.json: 14 actions" in output
    assert "buff_timeout_context.json: 10 actions" in output
    assert "low_resource_recovery_exit.json: 12 actions" in output
    assert "official_target_fallback_companion.json: 8 actions" in output
    assert "multi_unit_target_priority_rehearsal.json: 13 actions" in output
    assert "league_3v3_center_status.json: 6 actions" in output
    assert "description:" in output


def test_mock_sequence_loads_orders_and_dispatches_once(tmp_path: Path) -> None:
    sequence_path = tmp_path / "sequence.json"
    sequence_path.write_text(
        json.dumps(
            {
                "schema": "ly_simulator_mock_sequence_v1",
                "actions": [
                    {"at_sec": 5, "command": "set_unit_hp", "side": "enemy", "type_id": 1, "hp": 0},
                    {"at_sec": 1, "command": "set_structure_health", "side": "enemy", "structure": "outpost", "hp": 20},
                ],
            }
        ),
        encoding="utf-8",
    )

    sequence = load_mock_sequence_file(sequence_path)

    assert sequence.action_count == 2
    assert [action.at_sec for action in sequence.actions] == [1.0, 5.0]
    assert sequence.due_actions(0.5) == []
    assert [action.payload for action in sequence.due_actions(1.0)] == [
        {"command": "set_structure_health", "side": "enemy", "structure": "outpost", "hp": 20}
    ]
    assert sequence.due_actions(4.0) == []
    assert [action.payload for action in sequence.due_actions(5.0)] == [
        {"command": "set_unit_hp", "side": "enemy", "type_id": 1, "hp": 0}
    ]
    assert sequence.due_actions(30.0) == []

    sequence.reset()
    assert [action.command for action in sequence.due_actions(10.0)] == ["set_structure_health", "set_unit_hp"]


def test_mock_sequence_accepts_top_level_action_list() -> None:
    sequence = parse_mock_sequence(
        [
            {"at": 0, "command": "start"},
            {
                "elapsed_sec": 3.5,
                "command": "set_unit",
                "payload": {"side": "enemy", "type": "Hero", "x": 1200, "y": 900},
            },
        ]
    )

    assert [action.at_sec for action in sequence.actions] == [0.0, 3.5]
    assert sequence.actions[0].payload == {"command": "start"}
    assert sequence.actions[1].payload == {
        "command": "set_unit",
        "side": "enemy",
        "type": "Hero",
        "x": 1200,
        "y": 900,
    }


def test_emit_due_actions_appends_existing_control_bus_payloads(tmp_path: Path) -> None:
    control_path = tmp_path / "control.jsonl"
    sequence = parse_mock_sequence(
        {
            "actions": [
                {"at_sec": 0, "command": "start"},
                {"at_sec": 2, "command": "set_time_left", "seconds": 300},
                {"at_sec": 2, "command": "set_self_health", "hp": 210},
                {"at_sec": 2, "command": "set_ammo", "ammo": 18},
                {"at_sec": 2, "command": "set_posture", "posture": 2},
                {"at_sec": 2, "command": "set_self_position", "x": 820, "y": 830},
                {
                    "at_sec": 2,
                    "command": "set_structure_health",
                    "payload": {"side": "enemy", "structure": "outpost", "hp": 0},
                },
            ]
        }
    )

    assert emit_due_actions(sequence, control_path, 0.0) == 1
    assert emit_due_actions(sequence, control_path, 1.0) == 0
    assert emit_due_actions(sequence, control_path, 2.0) == 6

    commands, offset = read_commands(control_path, 0)
    assert offset > 0
    assert [command["command"] for command in commands] == [
        "start",
        "set_time_left",
        "set_self_health",
        "set_ammo",
        "set_posture",
        "set_self_position",
        "set_structure_health",
    ]
    assert commands[1]["seconds"] == 300
    assert commands[2]["hp"] == 210
    assert commands[3]["ammo"] == 18
    assert commands[4]["posture"] == 2
    assert commands[5]["x"] == 820
    assert commands[6]["side"] == "enemy"
    assert commands[6]["hp"] == 0


@pytest.mark.parametrize(
    "raw",
    [
        {"actions": [{"at_sec": -1, "command": "set_unit_hp"}]},
        {"actions": [{"at_sec": "soon", "command": "set_unit_hp"}]},
        {"actions": [{"at_sec": 1, "command": "unsupported"}]},
        {"actions": [{"at_sec": 1}]},
    ],
)
def test_mock_sequence_rejects_invalid_actions(raw: object) -> None:
    with pytest.raises(ValueError):
        parse_mock_sequence(raw)
