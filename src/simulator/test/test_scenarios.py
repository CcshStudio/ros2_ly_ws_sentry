from __future__ import annotations

import json
from collections import Counter
from pathlib import Path
from typing import Any

from simulator.config import goals_by_id, load_config
from simulator.trace import load_trace
from simulator.validation import validate_records


SCENARIO_DIR = Path(__file__).resolve().parents[1] / "sample" / "scenarios"
MANIFEST_PATH = SCENARIO_DIR / "manifest.json"


def load_manifest() -> dict[str, Any]:
    with MANIFEST_PATH.open("r", encoding="utf-8") as stream:
        manifest = json.load(stream)
    assert manifest["schema"] == "ly_simulator_scenario_manifest_v1"
    return manifest


def goal_names() -> dict[int, str]:
    config = load_config(None)
    return {goal_id: str(goal.get("name", f"Goal{goal_id}")) for goal_id, goal in goals_by_id(config).items()}


def status_for_issues(errors: int, warnings: int) -> str:
    if errors:
        return "FAIL"
    if warnings:
        return "WARN"
    return "PASS"


def assert_counter(actual: Counter[str], expected: dict[str, int], label: str) -> None:
    assert dict(actual) == expected, f"{label}: expected {expected}, got {dict(actual)}"


def assert_true_flags(record: Any, root_name: str, flags: list[str]) -> None:
    root = getattr(record, root_name)
    for flag in flags:
        assert getattr(root, flag) is True, f"{root_name}.{flag} should be true in {record.index}"


def assert_optional_sequence(actual: list[Any], expected: list[Any] | None, label: str) -> None:
    if expected is not None:
        assert actual == expected, f"{label}: expected {expected}, got {actual}"


def assert_unit_expectations(records: list[Any], expected: dict[str, Any], label: str) -> None:
    unit_info_counts = expected.get("unit_info_counts")
    if unit_info_counts is not None:
        actual_counts = Counter(unit.side for record in records for unit in record.unit_info)
        assert dict(actual_counts) == unit_info_counts, f"{label}: unit_info counts mismatch"

    for side in ("friend", "enemy"):
        key = f"fresh_{side}_unit_info"
        if key in expected:
            actual = sorted(
                {
                    unit.type_name
                    for record in records
                    for unit in record.unit_info
                    if unit.side == side and unit.has_position and unit.position_fresh
                }
            )
            assert actual == sorted(expected[key]), f"{label}: {key} expected {expected[key]}, got {actual}"

        type_key = f"{side}_unit_types"
        if type_key in expected:
            actual_types = sorted(
                {
                    unit.type_name
                    for record in records
                    for unit in record.units
                    if unit.side == side
                }
            )
            assert actual_types == sorted(expected[type_key]), f"{label}: {type_key} mismatch"


def assert_bullet_info_expectations(records: list[Any], expected: dict[str, Any], label: str) -> None:
    expected_bullet = expected.get("bullet_info")
    if expected_bullet is None:
        return
    assert records, f"{label}: missing records for bullet_info expectation"
    bullet = records[0].bullet_info
    for field, expected_value in expected_bullet.items():
        actual = getattr(bullet, field)
        assert actual == expected_value, f"{label}: bullet_info.{field} expected {expected_value}, got {actual}"


def assert_target_expectations(records: list[Any], expected: dict[str, Any], label: str) -> None:
    expected_target = expected.get("target_armor")
    if expected_target is not None:
        assert records, f"{label}: missing records for target_armor expectation"
        target = records[0].raw.get("target_armor", {})
        assert isinstance(target, dict), f"{label}: target_armor should be an object"
        for field, expected_value in expected_target.items():
            actual = target.get(field)
            assert actual == expected_value, f"{label}: target_armor.{field} expected {expected_value}, got {actual}"

    expected_hitable = expected.get("hitable_targets")
    if expected_hitable is not None:
        actual_hitable = sorted({target for record in records for target in record.target_state.hitable_targets})
        assert actual_hitable == sorted(expected_hitable), (
            f"{label}: hitable_targets expected {expected_hitable}, got {actual_hitable}"
        )


def assert_rfid_expectations(records: list[Any], expected: dict[str, Any], label: str) -> None:
    expected_rfid = expected.get("rfid_match")
    if expected_rfid is None:
        return
    assert records, f"{label}: missing records for rfid_match expectation"
    rfid = records[0].referee.rfid_match.as_payload()
    for field, expected_value in expected_rfid.items():
        actual = rfid.get(field)
        assert actual == expected_value, f"{label}: rfid_match.{field} expected {expected_value}, got {actual}"


def assert_nested_evidence(actual: Any, expected: Any, label: str) -> None:
    if isinstance(expected, dict):
        assert isinstance(actual, dict), f"{label}: expected object, got {actual!r}"
        for key, expected_value in expected.items():
            assert key in actual, f"{label}: missing {key}"
            assert_nested_evidence(actual[key], expected_value, f"{label}.{key}")
        return
    assert actual == expected, f"{label}: expected {expected!r}, got {actual!r}"


def assert_tactical_expectations(records: list[Any], expected: dict[str, Any], label: str) -> None:
    tactical_expected = expected.get("tactical")
    if tactical_expected is not None:
        for record in records:
            assert_nested_evidence(record.tactical.as_payload(), tactical_expected, f"{label}: tactical")

    final_control_expected = expected.get("final_control")
    if final_control_expected is not None:
        for record in records:
            actual = {
                "available": record.control_output.available,
                "source": record.control_output.source,
                "fire_code": {
                    "published": record.control_output.fire_code.published,
                    "follow_mode": record.control_output.fire_code.follow_mode,
                    "rotate": record.control_output.fire_code.rotate,
                },
            }
            assert_nested_evidence(actual, final_control_expected, f"{label}: final_control")


def test_manifest_paths_are_unique_and_present() -> None:
    manifest = load_manifest()
    scenarios = manifest["scenarios"]
    names = [item["name"] for item in scenarios]
    paths = [item["path"] for item in scenarios]

    assert len(names) == len(set(names))
    assert len(paths) == len(set(paths))
    for scenario_path in paths:
        assert (SCENARIO_DIR / scenario_path).exists(), scenario_path

    expected_files = {item["path"] for item in scenarios} | {"manifest.json"}
    actual_files = {path.name for path in SCENARIO_DIR.iterdir() if path.is_file()}
    assert actual_files == expected_files


def test_all_scenario_fixtures_validate_against_manifest() -> None:
    config = load_config(None)
    names = goal_names()
    manifest = load_manifest()
    scenario_records: list[Any] = []

    for scenario in manifest["scenarios"]:
        expected = scenario["expected"]
        records, bad_lines = load_trace(SCENARIO_DIR / scenario["path"], names)
        issues = validate_records(records, config, bad_lines)
        errors = sum(1 for issue in issues if issue.severity == "error")
        warnings = sum(1 for issue in issues if issue.severity == "warning")
        warning_codes = sorted({issue.code for issue in issues if issue.severity == "warning"})

        assert status_for_issues(errors, warnings) == expected["status"], scenario["name"]
        assert warning_codes == sorted(expected.get("warning_codes", [])), scenario["name"]
        assert len(records) == expected["records"], scenario["name"]
        assert_counter(Counter(f"v{record.schema_version}" for record in records), expected["schema_versions"], scenario["name"])
        assert_counter(Counter(record.output.kind for record in records), expected["output_kinds"], scenario["name"])
        assert sorted({record.decision_intent.layer for record in records}) == sorted(expected["intent_layers"])
        assert sorted({record.decision_intent.reason for record in records}) == sorted(expected["intent_reasons"])
        assert sorted({record.goal_name for record in records}) == sorted(expected["goal_names"])
        assert sorted({record.aim for record in records}) == sorted(expected["aim_modes"])
        assert_optional_sequence([record.event for record in records], expected.get("event_sequence"), scenario["name"])
        assert_optional_sequence([record.goal_name for record in records], expected.get("goal_sequence"), scenario["name"])
        assert_optional_sequence(
            [record.decision_intent.reason for record in records],
            expected.get("intent_reason_sequence"),
            scenario["name"],
        )
        assert_optional_sequence(
            [record.output.publish_allowed for record in records],
            expected.get("publish_allowed_sequence"),
            scenario["name"],
        )
        assert_optional_sequence([record.time_left for record in records], expected.get("time_left_sequence"), scenario["name"])
        assert_unit_expectations(records, expected, scenario["name"])
        assert_bullet_info_expectations(records, expected, scenario["name"])
        assert_target_expectations(records, expected, scenario["name"])
        assert_rfid_expectations(records, expected, scenario["name"])
        assert_tactical_expectations(records, expected, scenario["name"])

        for flag in expected.get("target_state_true", []):
            assert any(getattr(record.target_state, flag) is True for record in records), (
                f"target_state.{flag} should be true in at least one record for {scenario['name']}"
            )
        for record in records:
            assert_true_flags(record, "events", expected.get("events_true", []))
            if "runtime_guard_fault" in expected:
                assert record.runtime_guard.fault == expected["runtime_guard_fault"]
        scenario_records.extend(records)

    assert Counter(record.output.kind for record in scenario_records) == {
        "chase_goal_pos": 1,
        "chase_goal_pos_raw_bridge": 1,
        "goal_id": 1,
        "goal_pos": 24,
        "goal_pos_raw_bridge": 1,
        "relative_target_bridge": 2,
    }
    assert {record.decision_intent.layer for record in scenario_records} == {
        "Startup",
        "RegionalDefense",
        "RegionalPatrol",
        "BuffTask",
        "Chase",
        "OutpostTask",
        "Recovery",
        "Tactical",
    }
