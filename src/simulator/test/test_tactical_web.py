from __future__ import annotations

import json
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

import pytest

from simulator.main import league_3v3_map_objects
from simulator.tactical_web import build_tactical_html, tactical_state_from_status
from simulator.web_visual_check import find_sync_playwright
from simulator.web_stream import SimulatorWebStream


def tactical_metadata(*, ownership_mode: str = "mock", team: str = "red") -> dict[str, Any]:
    return {
        "control_enabled": True,
        "replay": {"controls_available": True},
        "current_record": {
            "tick": 42,
            "strategy": "Regional",
            "goal": {"id": 25, "name": "PreRoadland", "side": "red", "pos_cm": [457, 72]},
            "intent": {"layer": "default", "reason": "area_patrol", "priority": 10},
            "gimbal_feedback": {"available": True, "age_ms": 12, "fire_code": {"rotate": 1}},
            "control_output": {
                "available": True,
                "sequence": 9,
                "source": "normal",
                "angles": {"published": True, "yaw": 2.5, "pitch": -1.0},
                "fire_code": {"published": True, "follow_mode": True, "rotate": 0},
                "trajectory": {"published": True, "available": True, "yaw": 2.5, "pitch": -1.0},
            },
            "tactical": {
                "available": True,
                "protect_castle": {
                    "enabled": True,
                    "rfid_enabled": True,
                    "stay_when_rfid_enabled": True,
                    "stay_when_rfid_active": True,
                    "enemy_pos_enabled": True,
                    "rfid_event_raw_active": False,
                    "rfid_event_active": False,
                    "enemy_pos_active": True,
                },
                "protect_hero": {"enabled": True, "active": False},
                "regional_defense": {
                    "threat_active": True,
                    "search_kind": "own_base",
                    "fortress_enemy_count": 0,
                    "own_base_enemy_count": 1,
                },
            },
        },
        "simulator_inputs": {
            "enabled": True,
            "state": {
                "team": team,
                "ownership_mode": ownership_mode,
                "field": {"width_cm": 2800, "height_cm": 1500, "frame": "left_bottom_origin_cm"},
                "selected_entity_id": "enemy:hero:a",
                "units": [
                    {
                        "entity_id": "enemy:hero:a",
                        "side": "enemy",
                        "field_side": "blue",
                        "unit_key": "hero",
                        "asset_key": "Hero",
                        "type": "Hero",
                        "hp": 180,
                        "max_hp": 200,
                        "position_cm": {"x": 1200, "y": 700},
                    }
                ],
                "structures": [
                    {
                        "key": "friend_base",
                        "label": "Friend Base",
                        "side": "friend",
                        "kind": "base",
                        "hp": 4800,
                        "max_hp": 5000,
                        "position_cm": {"x": 245, "y": 755},
                    },
                    {
                        "key": "friend_outpost",
                        "label": "Friend Outpost",
                        "side": "friend",
                        "kind": "outpost",
                        "hp": 1500,
                        "max_hp": 1500,
                        "position_cm": {"x": 1011, "y": 429},
                    },
                ],
                "palette": [
                    {"side": "enemy", "unit_key": "hero", "asset_key": "Hero", "type": "Hero", "hp": 200, "max_hp": 200}
                ],
            },
        },
    }


def ensure_open(card: Any) -> None:
    if not card.evaluate("element => element.open"):
        card.locator("summary").click()


def get_json(stream: SimulatorWebStream, path: str) -> tuple[int, dict[str, Any]]:
    url = f"http://127.0.0.1:{stream.port}{path}"
    try:
        with urllib.request.urlopen(url, timeout=2.0) as response:
            return response.status, json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode("utf-8"))


def get_text(stream: SimulatorWebStream, path: str) -> tuple[int, str]:
    url = f"http://127.0.0.1:{stream.port}{path}"
    with urllib.request.urlopen(url, timeout=2.0) as response:
        return response.status, response.read().decode("utf-8")


def post_json(stream: SimulatorWebStream, path: str, payload: dict[str, Any]) -> tuple[int, dict[str, Any]]:
    url = f"http://127.0.0.1:{stream.port}{path}"
    request = urllib.request.Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        method="POST",
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(request, timeout=2.0) as response:
            return response.status, json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode("utf-8"))


def started_stream(
    tmp_path: Path, map_objects: list[dict[str, Any]] | None = None
) -> tuple[SimulatorWebStream, Path]:
    control_file = tmp_path / "tactical-control.jsonl"
    stream = SimulatorWebStream(
        host="127.0.0.1",
        port=0,
        control_file=control_file.as_posix(),
        map_objects=map_objects,
    )
    stream.start()
    return stream, control_file


def test_tactical_state_keeps_control_output_separate_from_feedback() -> None:
    state = tactical_state_from_status(tactical_metadata())

    assert state["field"] == {"width_cm": 2800, "height_cm": 1500, "frame": "left_bottom_origin_cm"}
    assert state["scene"]["ownership_mode"] == "mock"
    assert state["scene"]["team"] == "red"
    assert state["scene"]["show_trace_units"] is False
    assert state["decision"]["goal"]["id"] == 25
    assert state["gimbal_feedback"]["fire_code"]["rotate"] == 1
    assert state["control_output"]["fire_code"]["rotate"] == 0
    assert state["control_output"]["trajectory"]["available"] is True
    assert state["tactical"]["protect_castle"]["stay_when_rfid_enabled"] is True
    assert state["tactical"]["protect_castle"]["stay_when_rfid_active"] is True
    assert state["tactical"]["protect_castle"]["enemy_pos_active"] is True


def test_tactical_state_keeps_the_configured_team_color() -> None:
    state = tactical_state_from_status(tactical_metadata(team="blue"))

    assert state["scene"]["team"] == "blue"


def test_tactical_state_exposes_match_started() -> None:
    metadata = tactical_metadata()
    metadata["replay"].update({"match_started": True, "match_started_user": True})

    assert tactical_state_from_status(metadata)["match"]["started"] is True

    metadata["replay"].update({"match_started": True, "match_started_user": False})
    assert tactical_state_from_status(metadata)["match"]["started"] is False


def test_league_3v3_map_objects_include_red_and_blue_targets() -> None:
    goals = {
        0: {"name": "SelfBase", "red": [75, 700], "blue": [1125, 100]},
        1: {"name": "CenterAnchor", "red": [600, 400], "blue": [600, 400]},
    }

    objects = league_3v3_map_objects(goals)

    assert len(objects) == 4
    assert {item["field_side"] for item in objects} == {"red", "blue"}
    assert {item["key"] for item in objects} == {
        "SelfBase:red",
        "SelfBase:blue",
        "CenterAnchor:red",
        "CenterAnchor:blue",
    }


def test_tactical_html_exposes_local_health_draft_controls() -> None:
    body = build_tactical_html(0).decode("utf-8")

    assert "card('当前血量'" in body
    assert "card('血量设置'" in body
    assert "data-hp-step" in body
    assert "data-payload" not in body


def test_tactical_html_exposes_a_persistent_team_badge() -> None:
    body = build_tactical_html(0).decode("utf-8")

    assert 'id="teamPill"' in body


def test_tactical_html_accepts_explicit_empty_map_objects() -> None:
    body = build_tactical_html(0, map_objects=[]).decode("utf-8")

    assert "const MAP_OBJECTS=[]" in body
    assert "state.match.started" in body
    assert "const show=!started&&item.field_side===activeTeam" in body


def test_team_switch_buttons_emit_a_validated_team_command(tmp_path: Path) -> None:
    factory, reason = find_sync_playwright()
    if factory is None:
        pytest.skip(f"Playwright unavailable: {reason}")

    stream, control_file = started_stream(tmp_path)
    try:
        stream.update_metadata(tactical_metadata(team="red"))
        with factory() as playwright:
            browser = playwright.chromium.launch(headless=True)
            try:
                page = browser.new_page(viewport={"width": 1440, "height": 900})
                page.goto(f"http://127.0.0.1:{stream.port}/tactical", wait_until="domcontentloaded")
                team_badge = page.locator("#teamPill")
                team_badge.wait_for(state="visible")
                assert team_badge.text_content() == "我方 · 红方"

                page.locator("#viewSideBlue").click()
                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline and not control_file.exists():
                    page.wait_for_timeout(25)
                command = json.loads(control_file.read_text(encoding="utf-8").splitlines()[-1])
                assert {key: value for key, value in command.items() if key != "ts"} == {
                    "command": "set_team",
                    "team": "blue",
                }
            finally:
                browser.close()
    finally:
        stream.stop()


def test_piece_border_uses_physical_field_side_after_team_switch(tmp_path: Path) -> None:
    factory, reason = find_sync_playwright()
    if factory is None:
        pytest.skip(f"Playwright unavailable: {reason}")

    stream, _control_file = started_stream(tmp_path)
    try:
        metadata = tactical_metadata(team="blue")
        units = metadata["simulator_inputs"]["state"]["units"]
        units[0].update({"side": "friend", "field_side": "blue"})
        units.append(
            {
                "entity_id": "enemy:hero:b",
                "side": "enemy",
                "field_side": "red",
                "unit_key": "hero",
                "asset_key": "Hero",
                "type": "Hero",
                "hp": 180,
                "max_hp": 200,
                "position_cm": {"x": 1800, "y": 700},
            }
        )
        units.extend(
            [
                {
                    "entity_id": "friend:hero:c",
                    "side": "friend",
                    "field_side": "red",
                    "unit_key": "hero",
                    "asset_key": "Hero",
                    "type": "Hero",
                    "hp": 180,
                    "max_hp": 200,
                    "position_cm": {"x": 1500, "y": 550},
                },
                {
                    "entity_id": "enemy:hero:d",
                    "side": "enemy",
                    "field_side": "blue",
                    "unit_key": "hero",
                    "asset_key": "Hero",
                    "type": "Hero",
                    "hp": 180,
                    "max_hp": 200,
                    "position_cm": {"x": 2100, "y": 550},
                },
            ]
        )
        stream.update_metadata(metadata)
        with factory() as playwright:
            browser = playwright.chromium.launch(headless=True)
            try:
                page = browser.new_page(viewport={"width": 1440, "height": 900})
                page.goto(f"http://127.0.0.1:{stream.port}/tactical", wait_until="domcontentloaded")
                page.locator(".piece").first.wait_for(state="visible")
                page.wait_for_timeout(250)
                assert page.locator(".piece.friend.field-blue").count() == 1
                assert page.locator(".piece.enemy.field-red").count() == 1
                assert page.locator(".piece.friend.field-red .ring").evaluate(
                    "element => getComputedStyle(element).borderColor"
                ) == "rgb(189, 112, 116)"
                assert page.locator(".piece.enemy.field-blue .ring").evaluate(
                    "element => getComputedStyle(element).borderColor"
                ) == "rgb(109, 156, 203)"
            finally:
                browser.close()
    finally:
        stream.stop()


def test_piece_border_css_never_uses_relative_friend_or_enemy_roles() -> None:
    body = build_tactical_html(0).decode("utf-8")

    assert ".piece.enemy .ring" not in body
    assert ".piece.friend .ring" not in body
    assert ".piece.field-red .ring" in body
    assert ".piece.field-blue .ring" in body


def test_tactical_state_exposes_match_clock_bounds() -> None:
    metadata = tactical_metadata()
    metadata["replay"].update({"match_time_left": 365, "match_duration_sec": 420, "match_running": True})

    state = tactical_state_from_status(metadata)

    assert state["match"] == {"time_left": 365, "duration_sec": 420, "running": True, "started": False}


def test_tactical_html_exposes_match_clock_controls() -> None:
    body = build_tactical_html(0).decode("utf-8")

    for identifier in (
        "matchClock",
        "matchTimeline",
        "matchTimeInput",
        "rewind10",
        "rewind30",
        "forward10",
        "forward30",
    ):
        assert f'id="{identifier}"' in body


def test_tactical_match_clock_emits_time_only_commands_and_keeps_inspector(tmp_path: Path) -> None:
    factory, reason = find_sync_playwright()
    if factory is None:
        pytest.skip(f"Playwright unavailable: {reason}")

    stream, control_file = started_stream(tmp_path)
    try:
        metadata = tactical_metadata()
        metadata["replay"].update({"match_time_left": 420, "match_duration_sec": 420, "match_running": False})
        stream.update_metadata(metadata)
        with factory() as playwright:
            browser = playwright.chromium.launch(headless=True)
            try:
                page = browser.new_page(viewport={"width": 1440, "height": 900})
                page.goto(f"http://127.0.0.1:{stream.port}/tactical", wait_until="domcontentloaded")
                page.wait_for_selector("#matchClock")
                page.locator(".structure.base").click()
                page.wait_for_function("document.getElementById('inspectorTitle')?.textContent.includes('Friend Base')")

                page.locator("#rewind10").click()
                page.locator("#matchTimeInput").fill("03:15")
                page.locator("#matchTimeInput").press("Enter")

                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline:
                    if control_file.exists() and len(control_file.read_text(encoding="utf-8").splitlines()) >= 2:
                        break
                    page.wait_for_timeout(25)
                commands = [json.loads(line) for line in control_file.read_text(encoding="utf-8").splitlines()]
                assert [{key: value for key, value in command.items() if key != "ts"} for command in commands[-2:]] == [
                    {"command": "rewind", "seconds": 10},
                    {"command": "set_time_left", "seconds": 195},
                ]
                assert page.locator("#inspector").is_visible()
                assert page.locator("#inspectorTitle").text_content() == "Friend Base"
            finally:
                browser.close()
    finally:
        stream.stop()


def test_tactical_workspace_exposes_dockable_map_first_shell() -> None:
    body = build_tactical_html(0).decode("utf-8")

    assert 'id="workspaceShell"' in body
    assert 'id="activityRail"' in body
    assert 'id="operationsShelf"' in body
    assert 'id="viewportZoomSelection"' in body
    assert 'data-selection-kind="overview"' in body
    assert 'id="inspectorDockHandle"' in body
    assert 'id="shelfResizeHandle"' in body


def test_tactical_html_has_flight_deck_css_tokens() -> None:
    body = build_tactical_html(0).decode("utf-8")

    for value in ("#111418", "#171C22", "#1D232B", "#202832", "#4DB7FF"):
        assert value in body
    assert "--fd-border:rgba(255,255,255,.06)" in body


def test_tactical_html_has_flight_deck_card_icon_and_motion_contract() -> None:
    body = build_tactical_html(0).decode("utf-8")

    assert "flight-deck-card" in body
    assert 'data-lucide="crosshair"' in body
    assert "@media (prefers-reduced-motion: reduce)" in body
    assert "transition:transform 150ms ease-out" in body
    assert "border-radius:12px" in body


def test_tactical_state_uses_resolved_goal_position_when_trace_goal_has_no_coordinates() -> None:
    metadata = tactical_metadata()
    metadata["current_record"]["goal"]["pos_cm"] = None
    metadata["decision"] = {
        "goal": {"id": 25, "name": "PreRoadland", "position_cm": [457.0, 72.0]},
        "route_cm": [],
    }

    state = tactical_state_from_status(metadata)

    assert state["decision"]["goal"]["pos_cm"] == [457.0, 72.0]


def test_tactical_routes_use_shared_status_and_reject_manual_ros_mutation(tmp_path: Path) -> None:
    stream, control_file = started_stream(tmp_path)
    try:
        stream.update_metadata(tactical_metadata())

        status, body = get_text(stream, "/tactical")
        assert status == 200
        assert "Sentinel Flight Deck" in body
        assert "哨兵战术台" in body
        assert "MutationObserver" in body
        assert "inspectorEditState" in body
        assert "if(selection.kind==='overview'&&state.scene.selected_entity_id)" not in body
        assert ".area,.structure,.piece{position:absolute;transform:translate(-50%,-50%)}" not in body
        assert ".piece{position:absolute}" in body
        assert "inspectorNeedsRender" in body
        assert "if(forceInspector||!inspectorInitialized||inspectorNeedsRender)" in body
        assert "setInterval(refresh,1000)" in body
        assert "pointerdown" in body
        assert "aria-live" in body
        assert 'id="tactical"' in body
        assert 'id="viewSideRed"' in body
        assert 'id="viewSideBlue"' in body
        assert 'id="mapZoomIn"' in body
        assert 'id="resetView"' in body
        assert 'id="debugToggle"' in body
        assert 'id="mapViewport"' in body
        assert 'id="inspector"' in body
        assert 'id="fitView"' in body
        assert 'id="actualSize"' in body
        assert 'id="fullscreenToggle"' in body
        assert 'id="resetLayout"' in body
        assert 'id="inspectorTitle"' in body
        assert 'id="structureLayer"' in body
        assert 'id="overviewInspector"' in body
        assert "dataset.entityId" in body
        assert "drag.element.classList.add('dragging')" in body
        assert 'localStorage' in body
        assert 'aria-expanded="true"' in body

        status, state = get_json(stream, "/api/tactical-state")
        assert status == 200
        assert state["scene"]["units"][0]["entity_id"] == "enemy:hero:a"
        assert state["decision"]["goal"]["pos_cm"] == [457, 72]

        status, response = post_json(
            stream,
            "/api/control",
            {
                "command": "place_unit",
                "entity_id": "enemy:hero:b",
                "side": "enemy",
                "unit_key": "hero",
                "x": 1111,
                "y": 722,
            },
        )
        assert status == 200
        assert response["ok"] is True
        assert response["payload"]["x"] == 1111
        written = json.loads(control_file.read_text(encoding="utf-8").strip())
        assert written["command"] == "place_unit"

        stream.update_metadata(tactical_metadata(ownership_mode="manual_ros"))
        status, response = post_json(
            stream,
            "/api/control",
            {"command": "set_structure_health", "side": "friend", "structure": "base", "hp": 3000},
        )
        assert status == 409
        assert response["ok"] is False
        assert response["message"] == "manual_ros_observer_mode"
        assert len(control_file.read_text(encoding="utf-8").splitlines()) == 1
    finally:
        stream.stop()


def test_tactical_state_disables_scene_edits_without_a_control_consumer(tmp_path: Path) -> None:
    metadata = tactical_metadata()
    metadata["replay"]["controls_available"] = False

    state = tactical_state_from_status(metadata)

    assert state["scene"]["can_edit"] is False
    assert state["scene"]["edit_reason"] == "control_consumer_unavailable"

    stream, control_file = started_stream(tmp_path)
    try:
        stream.update_metadata(metadata)
        status, response = post_json(
            stream,
            "/api/control",
            {
                "command": "place_unit",
                "entity_id": "enemy:hero:b",
                "side": "enemy",
                "unit_key": "hero",
                "x": 1111,
                "y": 722,
            },
        )

        assert status == 409
        assert response["ok"] is False
        assert response["message"] == "control_consumer_unavailable"
        assert not control_file.exists()
    finally:
        stream.stop()


def test_tactical_browser_interactions_emit_scene_commands_when_playwright_available(tmp_path: Path) -> None:
    factory, reason = find_sync_playwright()
    if factory is None:
        pytest.skip(f"Playwright unavailable: {reason}")

    stream, control_file = started_stream(tmp_path)
    try:
        stream.update_metadata(tactical_metadata())
        url = f"http://127.0.0.1:{stream.port}/tactical"

        with factory() as playwright:
            try:
                browser = playwright.chromium.launch(headless=True)
            except Exception as exc:
                pytest.skip(f"Chromium unavailable: {exc}")
            try:
                context = browser.new_context(viewport={"width": 1440, "height": 900})
                try:
                    page = context.new_page()
                    page.set_default_timeout(5000)
                    page.goto(url, wait_until="domcontentloaded", timeout=5000)
                    page.wait_for_selector("#fieldBoard", timeout=5000)
                    page.locator('[data-surface="units"]').click()
                    page.wait_for_function(
                        "document.querySelectorAll('#surfacePalette button').length === 1 && "
                        "document.querySelectorAll('.piece').length === 1",
                        timeout=5000,
                    )

                    page.wait_for_selector(".structure.base", timeout=5000)

                    page.locator("#surfacePalette button").click()
                    map_box = page.locator("#mapCanvas").bounding_box()
                    assert map_box is not None
                    page.mouse.click(map_box["x"] + map_box["width"] * 0.40, map_box["y"] + map_box["height"] * 0.55)

                    page.wait_for_function(
                        """() => {
                          const piece = document.querySelector(".piece[data-entity-id='enemy:hero:a']");
                          return piece && piece.getBoundingClientRect().width > 0;
                        }""",
                        timeout=5000,
                    )

                    piece = page.locator(".piece[data-entity-id='enemy:hero:a']")
                    piece.wait_for(state="visible")
                    piece_box = piece.bounding_box()
                    assert piece_box is not None
                    drag_map_box = page.locator("#mapCanvas").bounding_box()
                    assert drag_map_box is not None
                    page.mouse.move(piece_box["x"] + piece_box["width"] / 2, piece_box["y"] + piece_box["height"] / 2)
                    page.mouse.down()
                    page.mouse.move(drag_map_box["x"] + drag_map_box["width"] * 0.72, drag_map_box["y"] + drag_map_box["height"] * 0.60, steps=4)
                    assert piece.evaluate(
                        "element => element.classList.contains('dragging')"
                    )
                    page.mouse.up()

                    page.locator(".structure.base").click()
                    health = page.locator("#inspectorCards details").nth(2)
                    ensure_open(health)
                    health_step = page.get_by_role("button", name="+500")
                    health_step.wait_for(state="visible")
                    health_step.click(force=True)
                    page.locator("[data-apply='structure']").click()

                    deadline = time.monotonic() + 2.0
                    while time.monotonic() < deadline:
                        if control_file.exists() and len(control_file.read_text(encoding="utf-8").splitlines()) >= 3:
                            break
                        page.wait_for_timeout(25)

                    commands = [json.loads(line) for line in control_file.read_text(encoding="utf-8").splitlines()]
                    assert len(commands) == 3
                    assert all(isinstance(command.get("ts"), float) for command in commands)
                    assert commands[0]["command"] == "place_unit"
                    assert commands[0]["side"] == "enemy"
                    assert commands[0]["unit_key"] == "hero"
                    assert 0 <= commands[0]["x"] <= 2800
                    assert 0 <= commands[0]["y"] <= 1500
                    moved = next(command for command in commands if command.get("entity_id") == "enemy:hero:a")
                    assert {key: value for key, value in moved.items() if key not in {"ts", "x", "y"}} == {
                        "command": "place_unit",
                        "entity_id": "enemy:hero:a",
                        "side": "enemy",
                        "unit_key": "hero",
                        "hp": 180,
                    }
                    assert 0 <= moved["x"] <= 2800
                    assert 0 <= moved["y"] <= 1500
                    assert (moved["x"], moved["y"]) != (1200, 700)
                    assert {key: value for key, value in commands[2].items() if key != "ts"} == {
                        "command": "set_structure_health",
                        "side": "friend",
                        "structure": "base",
                        "hp": 5000,
                    }
                    page.screenshot(path=(tmp_path / "tactical-interactions.png").as_posix(), full_page=True)
                    assert (tmp_path / "tactical-interactions.png").is_file()
                finally:
                    context.close()
            finally:
                browser.close()
    finally:
        stream.stop()


def test_inspector_stays_open_across_selection_until_explicitly_closed(tmp_path: Path) -> None:
    factory, reason = find_sync_playwright()
    if factory is None:
        pytest.skip(f"Playwright unavailable: {reason}")

    stream, _control_file = started_stream(tmp_path)
    try:
        stream.update_metadata(tactical_metadata())
        with factory() as playwright:
            browser = playwright.chromium.launch(headless=True)
            try:
                context = browser.new_context(viewport={"width": 1440, "height": 900})
                page = context.new_page()
                page.goto(f"http://127.0.0.1:{stream.port}/tactical", wait_until="domcontentloaded")
                page.wait_for_selector(".structure.base")

                page.locator(".structure.base").click()
                page.wait_for_selector("#inspectorCards details")
                assert page.locator("#inspector").is_visible()
                handle = page.locator("#inspectorDockHandle").bounding_box()
                assert handle is not None
                page.mouse.move(handle["x"] + handle["width"] / 2, handle["y"] + 80)
                page.mouse.down()
                page.mouse.move(handle["x"] - 56, handle["y"] + 80)
                page.mouse.up()
                assert int(page.locator("#workspaceShell").evaluate(
                    "element => parseInt(element.style.getPropertyValue('--inspector'), 10)"
                )) > 320

                piece_box = page.locator(".piece").bounding_box()
                assert piece_box is not None
                page.mouse.click(piece_box["x"] + piece_box["width"] / 2, piece_box["y"] + piece_box["height"] / 2)
                page.wait_for_selector("#hpValue")
                assert page.locator("#inspector").is_visible()

                page.locator("#inspectorClose").click()
                page.wait_for_function("document.getElementById('workspaceShell')?.classList.contains('inspector-collapsed')")
                page.locator("#inspectorToggle").click()
                page.wait_for_function("!document.getElementById('workspaceShell')?.classList.contains('inspector-collapsed')")
                assert page.locator("#hpValue").is_visible()
            finally:
                browser.close()
    finally:
        stream.stop()


def test_target_markers_hide_after_match_start(tmp_path: Path) -> None:
    factory, reason = find_sync_playwright()
    if factory is None:
        pytest.skip(f"Playwright unavailable: {reason}")

    markers = [
        {
            "key": "SelfBase:red",
            "label": "红方基地",
            "detail": "SelfBase · red",
            "field_side": "red",
            "x": 75,
            "y": 700,
        },
        {
            "key": "SelfBase:blue",
            "label": "蓝方基地",
            "detail": "SelfBase · blue",
            "field_side": "blue",
            "x": 1125,
            "y": 100,
        },
    ]
    stream, _control_file = started_stream(tmp_path, map_objects=markers)
    try:
        metadata = tactical_metadata()
        metadata["replay"]["match_started_user"] = False
        stream.update_metadata(metadata)
        with factory() as playwright:
            browser = playwright.chromium.launch(headless=True)
            try:
                page = browser.new_page(viewport={"width": 1440, "height": 900})
                page.goto(f"http://127.0.0.1:{stream.port}/tactical", wait_until="domcontentloaded")
                page.wait_for_function("Array.from(document.querySelectorAll('#areaLayer .area')).filter(element => getComputedStyle(element).display !== 'none').length === 1")
                assert page.locator("#areaLayer .area.red:visible").count() == 1
                assert page.locator("#areaLayer .area.blue:visible").count() == 0
                assert page.locator("#goalMarker:visible").count() == 0

                metadata["replay"]["match_started_user"] = True
                stream.update_metadata(metadata)
                page.wait_for_function("Array.from(document.querySelectorAll('#areaLayer .area')).filter(element => getComputedStyle(element).display !== 'none').length === 0")
                assert page.locator("#goalMarker:visible").count() == 1
            finally:
                browser.close()
    finally:
        stream.stop()


def test_inspector_health_steps_edit_draft_until_apply(tmp_path: Path) -> None:
    factory, reason = find_sync_playwright()
    if factory is None:
        pytest.skip(f"Playwright unavailable: {reason}")

    stream, control_file = started_stream(tmp_path)
    try:
        stream.update_metadata(tactical_metadata())
        with factory() as playwright:
            browser = playwright.chromium.launch(headless=True)
            try:
                page = browser.new_page(viewport={"width": 1440, "height": 900})
                page.goto(f"http://127.0.0.1:{stream.port}/tactical", wait_until="domcontentloaded")
                piece = page.locator(".piece[data-entity-id='enemy:hero:a']")
                piece.wait_for(state="visible")
                piece.evaluate("element => element.click()")
                page.wait_for_selector("#hpValue")

                cards = page.locator("#inspectorCards details")
                assert cards.nth(1).evaluate("element => element.open") is True
                assert cards.nth(2).evaluate("element => element.open") is True
                assert cards.nth(3).evaluate("element => element.open") is False

                draft = page.locator("#hpValue")
                draft.fill("120")
                assert not control_file.exists()

                page.locator("[data-hp-step='50']").click()
                assert draft.input_value() == "170"
                assert not control_file.exists()

                page.locator("[data-hp-step='-50']").click()
                assert draft.input_value() == "120"
                page.locator("[data-hp-step='-50']").click()
                assert draft.input_value() == "70"
                assert not control_file.exists()

                page.locator("[data-apply='unit']").click()
                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline and not control_file.exists():
                    page.wait_for_timeout(25)
                command = json.loads(control_file.read_text(encoding="utf-8").splitlines()[-1])
                assert command["command"] == "set_unit_hp"
                assert command["entity_id"] == "enemy:hero:a"
                assert command["hp"] == 70

                # Toggling the draft card must not expand the decision card.
                cards.nth(1).locator("summary").click()
                assert cards.nth(1).evaluate("element => element.open") is False
                assert cards.nth(3).evaluate("element => element.open") is False
            finally:
                browser.close()
    finally:
        stream.stop()


def test_inspector_health_card_and_editor_survive_live_refresh(tmp_path: Path) -> None:
    factory, reason = find_sync_playwright()
    if factory is None:
        pytest.skip(f"Playwright unavailable: {reason}")

    stream, _control_file = started_stream(tmp_path)
    try:
        stream.update_metadata(tactical_metadata())
        with factory() as playwright:
            browser = playwright.chromium.launch(headless=True)
            try:
                page = browser.new_page(viewport={"width": 1440, "height": 900})
                page.goto(f"http://127.0.0.1:{stream.port}/tactical", wait_until="domcontentloaded")
                page.wait_for_selector(".structure.base")
                page.locator(".structure.base").click()
                health = page.locator("#inspectorCards details").nth(1)
                ensure_open(health)
                page.locator("#hpValue").fill("4300")

                # State is refreshed every 1000 ms. A live editor must retain both its
                # expanded state and a user-entered, unapplied value across that refresh.
                page.wait_for_timeout(1200)
                assert health.evaluate("element => element.open")
                assert page.locator("#hpValue").input_value() == "4300"
            finally:
                browser.close()
    finally:
        stream.stop()


def test_piece_drag_updates_visual_position_while_live_refreshes(tmp_path: Path) -> None:
    factory, reason = find_sync_playwright()
    if factory is None:
        pytest.skip(f"Playwright unavailable: {reason}")

    stream, _control_file = started_stream(tmp_path)
    try:
        stream.update_metadata(tactical_metadata())
        with factory() as playwright:
            browser = playwright.chromium.launch(headless=True)
            try:
                page = browser.new_page(viewport={"width": 1440, "height": 900})
                page.goto(f"http://127.0.0.1:{stream.port}/tactical", wait_until="domcontentloaded")
                piece = page.locator(".piece[data-entity-id='enemy:hero:a']")
                piece.wait_for(state="visible")
                piece_box = piece.bounding_box()
                map_box = page.locator("#mapCanvas").bounding_box()
                assert piece_box is not None and map_box is not None
                before_left = piece.evaluate("element => element.style.left")

                page.mouse.move(piece_box["x"] + piece_box["width"] / 2, piece_box["y"] + piece_box["height"] / 2)
                page.mouse.down()
                page.mouse.move(map_box["x"] + map_box["width"] * 0.78, map_box["y"] + map_box["height"] * 0.64, steps=5)
                page.wait_for_timeout(700)

                assert piece.evaluate("element => element.classList.contains('dragging')")
                assert piece.evaluate("element => element.style.left") != before_left
                page.mouse.up()
            finally:
                browser.close()
    finally:
        stream.stop()


def test_outpost_inspector_emits_live_health_command(tmp_path: Path) -> None:
    factory, reason = find_sync_playwright()
    if factory is None:
        pytest.skip(f"Playwright unavailable: {reason}")

    stream, control_file = started_stream(tmp_path)
    try:
        stream.update_metadata(tactical_metadata())
        with factory() as playwright:
            browser = playwright.chromium.launch(headless=True)
            try:
                page = browser.new_page(viewport={"width": 1440, "height": 900})
                page.goto(f"http://127.0.0.1:{stream.port}/tactical", wait_until="domcontentloaded")
                outpost = page.locator(".structure.outpost")
                outpost.click()
                page.wait_for_selector("#hpValue")
                health = page.locator("#inspectorCards details").nth(2)
                ensure_open(health)
                health.locator("[data-hp-step='-100']").click()
                assert page.locator("#hpValue").input_value() == "1400"
                health.locator("[data-apply='structure']").click()

                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline and not control_file.exists():
                    page.wait_for_timeout(25)
                command = json.loads(control_file.read_text(encoding="utf-8").splitlines()[-1])
                assert {key: value for key, value in command.items() if key != "ts"} == {
                    "command": "set_structure_health",
                    "side": "friend",
                    "structure": "outpost",
                    "hp": 1400,
                }
            finally:
                browser.close()
    finally:
        stream.stop()


def test_focus_mode_keeps_the_persistent_inspector_visible(tmp_path: Path) -> None:
    factory, reason = find_sync_playwright()
    if factory is None:
        pytest.skip(f"Playwright unavailable: {reason}")

    stream, _control_file = started_stream(tmp_path)
    try:
        stream.update_metadata(tactical_metadata())
        with factory() as playwright:
            browser = playwright.chromium.launch(headless=True)
            try:
                page = browser.new_page(viewport={"width": 1440, "height": 900})
                page.goto(f"http://127.0.0.1:{stream.port}/tactical", wait_until="domcontentloaded")
                page.wait_for_selector("#inspector")
                page.locator("#focusToggle").click()
                page.wait_for_timeout(350)

                box = page.locator("#inspector").bounding_box()
                assert box is not None
                assert box["width"] >= 300
                assert page.locator("#inspectorTitle").is_visible()
            finally:
                browser.close()
    finally:
        stream.stop()


def test_piece_art_cannot_capture_the_browser_native_image_drag(tmp_path: Path) -> None:
    factory, reason = find_sync_playwright()
    if factory is None:
        pytest.skip(f"Playwright unavailable: {reason}")

    stream, _control_file = started_stream(tmp_path)
    try:
        stream.update_metadata(tactical_metadata())
        with factory() as playwright:
            browser = playwright.chromium.launch(headless=True)
            try:
                page = browser.new_page(viewport={"width": 1440, "height": 900})
                page.goto(f"http://127.0.0.1:{stream.port}/tactical", wait_until="domcontentloaded")
                art = page.locator(".piece img")
                art.wait_for(state="visible")
                assert art.evaluate("element => element.draggable") is False
                assert art.evaluate("element => getComputedStyle(element).pointerEvents") == "none"
            finally:
                browser.close()
    finally:
        stream.stop()
