import json
from types import SimpleNamespace

import pygame

from simulator.control_bus import normalize_api_control_payload
from simulator.field import FieldGeometry, relative_side_to_field_side
from simulator.inputs_panel import compact_decision_summary
from simulator.interactive_inputs import SimulatorInputState, load_unit_scene_file, unit_decision_summary
from simulator.viewer import Viewer
from simulator.workspace import Rect, ViewportState


def test_field_geometry_converts_position_data_y() -> None:
    field = FieldGeometry()

    assert field.position_data_raw_y(900) == 600
    assert field.position_data_official_y(600) == 900
    assert field.clamp_point((-10, 1800)) == (0.0, 1500.0)


def test_relative_side_maps_to_absolute_team_side() -> None:
    assert relative_side_to_field_side("friend", "red") == "red"
    assert relative_side_to_field_side("enemy", "red") == "blue"
    assert relative_side_to_field_side("friend", "blue") == "blue"
    assert relative_side_to_field_side("enemy", "blue") == "red"


def test_team_switch_updates_relative_scene_colors() -> None:
    state = SimulatorInputState.with_defaults()
    assert state.apply_command("set_unit", {"side": "friend", "type": "Sentry", "x": 245, "y": 755})
    assert state.apply_command("set_unit", {"side": "enemy", "type": "Sentry", "x": 2555, "y": 745})

    assert state.apply_command("set_team", {"team": "blue"})
    snapshot = state.snapshot()

    assert snapshot["team"] == "blue"
    assert next(item for item in snapshot["units"] if item["side"] == "friend")["field_side"] == "blue"
    assert next(item for item in snapshot["units"] if item["side"] == "enemy")["field_side"] == "red"


def test_structure_health_accepts_zero_and_uses_base_points() -> None:
    state = SimulatorInputState.with_defaults()

    assert state.apply_command("set_structure_health", {"side": "enemy", "structure": "outpost", "hp": 0})
    assert state.structure_hp("enemy", "outpost") == 0

    friend_base = state.find_structure("friend", "base")
    enemy_base = state.find_structure("enemy", "base")

    assert friend_base is not None
    assert enemy_base is not None
    assert state.structure_position(friend_base, "red", {}) == (245.0, 755.0)
    assert state.structure_position(enemy_base, "red", {}) == (2555.0, 745.0)


def test_default_outpost_health_uses_uncompressed_referee_hp() -> None:
    state = SimulatorInputState.with_defaults()

    friend_outpost = state.find_structure("friend", "outpost")
    enemy_outpost = state.find_structure("enemy", "outpost")

    assert friend_outpost is not None
    assert enemy_outpost is not None
    assert friend_outpost.hp == 1500
    assert friend_outpost.max_hp == 1500
    assert enemy_outpost.hp == 1500
    assert enemy_outpost.max_hp == 1500


def test_pygame_map_structures_are_hittable_at_catalog_coordinates() -> None:
    viewer = object.__new__(Viewer)
    viewer.sim_input_state = SimulatorInputState.with_defaults()
    viewer.records = [SimpleNamespace(team="red")]
    viewer.current_index = 0
    viewer.goals = {}
    viewer.field_to_screen = lambda point, _image_rect: (round(point[0]), round(point[1]))

    assert viewer.hit_sim_structure((245, 755), object()) == "friend_base"


def test_pygame_selected_structure_resolves_the_map_inspector_subject() -> None:
    viewer = object.__new__(Viewer)
    viewer.sim_input_state = SimulatorInputState.with_defaults()
    viewer.selected_structure_key = "friend_base"

    selected = viewer.selected_structure()

    assert selected is not None
    assert selected.label == "Friend Base"
    assert selected.structure == "base"


def test_pygame_structure_inspector_uses_existing_health_command() -> None:
    viewer = object.__new__(Viewer)
    calls: list[tuple[str, dict]] = []
    viewer.inspector_buttons = {
        "friend_base:+500": (pygame.Rect(10, 10, 50, 20), {"side": "friend", "structure": "base", "hp": 5000})
    }
    viewer.send_sim_command = lambda command, payload: calls.append((command, payload))

    assert viewer.handle_inspector_mouse_down((20, 15))
    assert calls == [("set_structure_health", {"side": "friend", "structure": "base", "hp": 5000})]


def test_pygame_map_camera_scales_the_fit_rect_without_changing_field_geometry() -> None:
    viewer = object.__new__(Viewer)
    viewer.pg = pygame
    viewer.width = 1500
    viewer.height = 900
    viewer.panel_w = 390
    viewer.timeline_h = 92
    viewer.map_size = (2094, 1122)
    viewer.field_geometry = lambda: FieldGeometry()
    content = viewer.map_content_rect()
    viewer.map_viewport = ViewportState.fit(
        field_width=2800,
        field_height=1500,
        viewport=Rect(content.x, content.y, content.width, content.height),
    )

    fit_rect = viewer.map_image_rect()
    viewer.adjust_map_zoom(2.0, fit_rect.center)
    zoomed_rect = viewer.map_image_rect()

    assert zoomed_rect.size == (fit_rect.width * 2, fit_rect.height * 2)
    assert zoomed_rect.center == fit_rect.center


def test_pygame_fullscreen_toggle_restores_the_windowed_viewport() -> None:
    class FakeScreen:
        def __init__(self, size: tuple[int, int]) -> None:
            self._size = size

        def get_size(self) -> tuple[int, int]:
            return self._size

    class FakeDisplay:
        def __init__(self) -> None:
            self.calls: list[tuple[tuple[int, int], int]] = []

        def set_mode(self, size: tuple[int, int], flags: int) -> FakeScreen:
            self.calls.append((size, flags))
            return FakeScreen((1920, 1080) if flags == 1 else size)

    display = FakeDisplay()
    viewer = object.__new__(Viewer)
    viewer.pg = SimpleNamespace(FULLSCREEN=1, RESIZABLE=2, display=display)
    viewer.width = 1500
    viewer.height = 900
    viewer.fullscreen = False
    viewer.windowed_size = (1500, 900)
    viewer.scaled_map = object()
    viewer.cached_map_key = (100, 100)
    refresh_calls: list[bool] = []
    viewer._refresh_map_viewport = lambda *, reset=False: refresh_calls.append(reset)

    viewer.toggle_fullscreen()

    assert viewer.fullscreen is True
    assert display.calls == [((0, 0), 1)]
    assert viewer.width == 1920
    assert viewer.height == 1080
    assert refresh_calls == [True]

    viewer.toggle_fullscreen()

    assert viewer.fullscreen is False
    assert display.calls[-1] == ((1500, 900), 2)


def test_unit_commands_feed_health_and_position_rows() -> None:
    state = SimulatorInputState.with_defaults(field=FieldGeometry())

    assert state.apply_command(
        "set_unit",
        {
            "side": "enemy",
            "type": "Hero",
            "hp": 123,
            "max_hp": 200,
            "x": 2555,
            "y": 900,
        },
    )

    health = state.health_fields("enemy", 400)
    assert health["hero"] == 123
    assert health["engineer"] == 400

    rows = state.position_rows()
    assert len(rows) == 1
    assert rows[0].enemy_car_id == 101
    assert rows[0].raw_x == 2555
    assert rows[0].raw_y == 600

    assert state.apply_command("set_unit_hp", {"side": "enemy", "type_id": 1, "hp": 0})
    assert state.health_fields("enemy", 400)["hero"] == 0
    assert tuple(state.scene.units) == ("enemy:hero",)


def test_legacy_hp_commands_without_a_value_keep_the_current_value() -> None:
    state = SimulatorInputState.with_defaults(field=FieldGeometry())
    assert state.apply_command("set_unit", {"side": "enemy", "type": "Hero", "hp": 123, "x": 100, "y": 200})
    assert state.apply_command("set_unit_hp", {"side": "enemy", "type": "Hero"})
    assert state.health_fields("enemy", 400)["hero"] == 123

    assert state.apply_command("set_structure_health", {"side": "friend", "structure": "base", "hp": 3000})
    assert state.apply_command("set_structure_health", {"side": "friend", "structure": "base"})
    assert state.structure_hp("friend", "base") == 3000


def test_legacy_tuple_view_does_not_choose_between_conflicting_scene_instances() -> None:
    state = SimulatorInputState.with_defaults(field=FieldGeometry())
    assert state.apply_command(
        "set_unit",
        {"entity_id": "enemy:hero:a", "side": "enemy", "type": "Hero", "x": 100, "y": 200},
    )
    assert state.apply_command(
        "set_unit",
        {"entity_id": "enemy:hero:b", "side": "enemy", "type": "Hero", "x": 300, "y": 400},
    )

    assert state.units == {}
    assert state.scene.project_ros_inputs().conflicts


def test_unit_scene_file_imports_units(tmp_path) -> None:
    scene_path = tmp_path / "unit_scene.json"
    scene_path.write_text(
        json.dumps(
            {
                "units": [
                    {"side": "enemy", "type": "Hero", "hp": 123, "max_hp": 200, "x": 2555, "y": 900},
                    {"side": "friend", "type_id": 7, "hp": 399, "position_cm": {"x": 300, "y": 400}},
                    {"side": "enemy", "type": "Unknown", "hp": 1, "x": 100, "y": 100},
                ],
            }
        ),
        encoding="utf-8",
    )
    state = SimulatorInputState.with_defaults(field=FieldGeometry())

    assert state.apply_unit_scene(load_unit_scene_file(scene_path)) == 2

    assert state.health_fields("enemy", 400)["hero"] == 123
    assert state.health_fields("friend", 400)["sentry"] == 399
    rows = sorted(state.position_rows(), key=lambda row: (row.side, row.type_id))
    assert [(row.side, row.type_id, row.raw_x, row.raw_y) for row in rows] == [
        ("enemy", 1, 2555, 600),
        ("friend", 7, 300, 1100),
    ]


def test_set_units_command_replaces_or_merges_units() -> None:
    state = SimulatorInputState.with_defaults(field=FieldGeometry())

    assert state.apply_command(
        "set_units",
        {"units": [{"side": "enemy", "type_id": 1, "x": 100, "y": 200}]},
    )
    assert len(state.units) == 1

    assert state.apply_command(
        "set_units",
        {"clear": False, "units": [{"side": "friend", "type_id": 7, "x": 300, "y": 400}]},
    )
    assert sorted(state.units) == [("enemy", 1), ("friend", 7)]


def test_default_palette_includes_extended_visual_units_without_new_health_fields() -> None:
    state = SimulatorInputState.with_defaults(field=FieldGeometry())
    palette = {(item.side, item.type_id, item.type_name) for item in state.unit_palette}

    assert ("friend", 5, "Infantry3") in palette
    assert ("friend", 6, "Drone") in palette
    assert ("enemy", 5, "Infantry3") in palette
    assert ("enemy", 6, "Drone") in palette

    assert state.apply_command("set_unit", {"side": "enemy", "type": "Drone", "hp": 99, "x": 1549, "y": 923})
    assert "drone" not in state.health_fields("enemy", 400)
    rows = state.position_rows()
    assert rows[0].enemy_car_id == 106
    assert rows[0].raw_x == 1549
    assert rows[0].raw_y == 577


def test_unit_decision_summary_separates_published_and_bt_consumed_channels() -> None:
    assert unit_decision_summary("enemy", 1) == "BT:HP,POS,UI"
    assert unit_decision_summary("friend", 7) == "BT:HP,POS,UI"
    assert unit_decision_summary("enemy", 5) == "PUB:HP,POS noUI"
    assert unit_decision_summary("enemy", 6) == "PUB:POS noUI"


def test_input_panel_compact_decision_summary_shortens_badges_for_dense_ui() -> None:
    assert compact_decision_summary("enemy", 1) == "BT HP/POS/UI"
    assert compact_decision_summary("enemy", 5) == "PUB HP/POS"
    assert compact_decision_summary("enemy", 6) == "PUB POS"


def test_control_bus_accepts_simulator_input_commands() -> None:
    command, payload, error = normalize_api_control_payload(
        {"command": "set_unit", "side": "friend", "type_id": 7, "x": 300, "y": 400},
        default_step_sec=10,
    )

    assert error is None
    assert command == "set_unit"
    assert payload == {"side": "friend", "type_id": 7, "x": 300, "y": 400}

    command, payload, error = normalize_api_control_payload(
        {"command": "set_units", "units": [{"side": "enemy", "type_id": 1, "x": 100, "y": 200}]},
        default_step_sec=10,
    )

    assert error is None
    assert command == "set_units"
    assert payload == {"units": [{"side": "enemy", "type_id": 1, "x": 100, "y": 200}]}

    command, payload, error = normalize_api_control_payload(
        {"command": "set_self_health", "hp": 210},
        default_step_sec=10,
    )
    assert error is None
    assert command == "set_self_health"
    assert payload == {"hp": 210}

    command, payload, error = normalize_api_control_payload(
        {"command": "set_self_position", "x": 820, "y": 830},
        default_step_sec=10,
    )
    assert error is None
    assert command == "set_self_position"
    assert payload == {"x": 820, "y": 830}

    command, payload, error = normalize_api_control_payload(
        {"command": "set_team", "team": "BLUE"},
        default_step_sec=10,
    )
    assert (command, payload, error) == ("set_team", {"team": "blue"}, None)

    command, _payload, error = normalize_api_control_payload(
        {"command": "set_team", "team": "green"},
        default_step_sec=10,
    )
    assert command is None
    assert error == "team must be red or blue"


def test_control_bus_accepts_and_validates_canonical_place_unit() -> None:
    command, payload, error = normalize_api_control_payload(
        {
            "command": "place_unit",
            "entity_id": "enemy:hero:a",
            "side": "enemy",
            "unit_key": "hero",
            "x": 1200,
            "y": 700,
        },
        default_step_sec=10,
    )

    assert error is None
    assert command == "place_unit"
    assert payload["entity_id"] == "enemy:hero:a"

    command, payload, error = normalize_api_control_payload(
        {"command": "place_unit", "entity_id": "enemy:hero:a", "side": "enemy"},
        default_step_sec=10,
    )

    assert command is None
    assert payload == {}
    assert error == "place_unit requires a known side and unit_key"


def test_canonical_place_unit_reaches_the_facade_command_path() -> None:
    command, payload, error = normalize_api_control_payload(
        {
            "command": "place_unit",
            "entity_id": "enemy:hero:manual",
            "side": "enemy",
            "unit_key": "hero",
            "x": 1200,
            "y": 700,
            "hp": 123,
        },
        default_step_sec=10,
    )
    assert error is None
    assert command is not None

    state = SimulatorInputState.with_defaults(field=FieldGeometry())

    assert state.apply_control_payload({"command": command, **payload})
    assert "enemy:hero:manual" in state.scene.units
    assert state.snapshot()["units"][0]["entity_id"] == "enemy:hero:manual"


def test_facade_position_rows_keep_catalog_resolved_formal_car_ids() -> None:
    state = SimulatorInputState.with_defaults(field=FieldGeometry())
    for side, unit_key, x in (
        ("friend", "Hero", 100),
        ("enemy", "Hero", 200),
        ("friend", "Drone", 300),
        ("enemy", "Drone", 400),
    ):
        assert state.apply_command("set_unit", {"side": side, "type": unit_key, "x": x, "y": 200})

    rows = {(row.side, row.type_id): row for row in state.position_rows()}

    assert rows[("friend", 1)].formal_car_id == 1
    assert rows[("enemy", 1)].formal_car_id == 101
    assert rows[("friend", 6)].formal_car_id == 6
    assert rows[("enemy", 6)].formal_car_id == 106


def test_manual_ros_compatibility_facade_rejects_scene_mutations() -> None:
    state = SimulatorInputState(ownership_mode="manual_ros")

    assert not state.apply_command("set_unit", {"side": "enemy", "type": "Hero", "x": 1200, "y": 700})
    assert not state.apply_command("set_structure_health", {"side": "friend", "structure": "base", "hp": 3000})
    assert state.units == {}


def test_input_state_config_preserves_manual_ros_ownership() -> None:
    state = SimulatorInputState.from_config({"input_owner": "manual_ros"}, field=FieldGeometry())

    assert state.snapshot()["ownership_mode"] == "manual_ros"
    assert not state.apply_command("set_structure_health", {"side": "friend", "structure": "base", "hp": 3000})


def test_input_state_config_empty_structures_disables_regional_overlay() -> None:
    state = SimulatorInputState.from_config(
        {"input_owner": "mock", "structures": []},
        field=FieldGeometry(),
    )

    assert state.snapshot()["structures"] == []


def test_input_state_reset_restores_initial_3v3_sentry_for_current_team() -> None:
    state = SimulatorInputState.from_config(
        {
            "input_owner": "mock",
            "initial_units": [
                {
                    "entity_id": "friend:sentry:offline",
                    "side": "friend",
                    "type": "Sentry",
                    "hp": 400,
                    "max_hp": 400,
                    "position_cm": {"x": 75, "y": 700},
                }
            ],
        },
        field=FieldGeometry(1200, 800),
    )
    state.apply_command("set_team", {"team": "blue"})
    state.apply_command(
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

    state.reset()
    snapshot = state.snapshot()
    sentry = snapshot["units"][0]

    assert snapshot["team"] == "blue"
    assert sentry["position_cm"] == {"x": 1125, "y": 100}
    assert sentry["hp"] == 400


def test_input_state_team_switch_mirrors_3v3_sentry() -> None:
    state = SimulatorInputState.from_config(
        {
            "input_owner": "mock",
            "initial_units": [
                {
                    "entity_id": "friend:sentry:offline",
                    "side": "friend",
                    "type": "Sentry",
                    "hp": 400,
                    "max_hp": 400,
                    "position_cm": {"x": 75, "y": 700},
                }
            ],
        },
        field=FieldGeometry(1200, 800),
    )

    assert state.snapshot()["units"][0]["position_cm"] == {"x": 75, "y": 700}
    assert state.apply_command("set_team", {"team": "blue"})
    assert state.snapshot()["units"][0]["position_cm"] == {"x": 1125, "y": 100}


def test_input_state_config_can_replace_palette_and_place_3v3_sentry() -> None:
    palette = [
        {"side": "friend", "type": "Hero", "hp": 350, "max_hp": 350},
        {"side": "friend", "type": "Infantry1", "hp": 300, "max_hp": 300},
        {"side": "friend", "type": "Sentry", "hp": 400, "max_hp": 400},
        {"side": "enemy", "type": "Hero", "hp": 350, "max_hp": 350},
        {"side": "enemy", "type": "Infantry1", "hp": 300, "max_hp": 300},
        {"side": "enemy", "type": "Sentry", "hp": 400, "max_hp": 400},
    ]
    state = SimulatorInputState.from_config(
        {
            "input_owner": "mock",
            "unit_palette": palette,
            "initial_units": [
                {
                    "entity_id": "friend:sentry:offline",
                    "side": "friend",
                    "type": "Sentry",
                    "hp": 400,
                    "max_hp": 400,
                    "position_cm": {"x": 245, "y": 750},
                }
            ],
        },
        field=FieldGeometry(),
    )

    snapshot = state.snapshot()
    assert len(snapshot["palette"]) == 6
    assert [(unit["side"], unit["type"]) for unit in snapshot["units"]] == [("friend", "Sentry")]
    assert snapshot["units"][0]["position_cm"] == {"x": 245, "y": 750}


def test_input_state_snapshot_exposes_json_safe_decision_context() -> None:
    state = SimulatorInputState.with_defaults(field=FieldGeometry())

    assert state.apply_command("set_structure_health", {"side": "enemy", "structure": "outpost", "hp": 0})
    assert state.apply_command("set_self_health", {"hp": 210})
    assert state.apply_command("set_ammo", {"ammo": 18})
    assert state.apply_command("set_posture", {"posture": 2})
    assert state.apply_command("set_self_position", {"x": 820, "y": 830})
    assert state.apply_command("set_unit", {"side": "enemy", "type": "Hero", "hp": 120, "x": 2555, "y": 900})
    assert state.apply_command("set_unit", {"side": "enemy", "type": "Drone", "hp": 30, "x": 1549, "y": 923})
    assert state.apply_command("set_unit", {"side": "friend", "type": "Sentry", "hp": 390, "x": 300, "y": 400})

    snapshot = state.snapshot(team="red", goals={})

    assert snapshot["team"] == "red"
    assert snapshot["ownership_mode"] == "mock"
    assert snapshot["field"] == {"width_cm": 2800, "height_cm": 1500, "frame": "left_bottom_origin_cm"}
    assert snapshot["summary"]["unit_count"] == 3
    assert snapshot["summary"]["friend_units"] == 1
    assert snapshot["summary"]["enemy_units"] == 2
    assert snapshot["summary"]["destroyed_structures"] == ["enemy_outpost"]
    assert "enemy:Drone" in snapshot["summary"]["low_hp_units"]
    assert snapshot["runtime"] == {
        "self_health": 210,
        "ammo_left": 18,
        "posture": 2,
        "self_position_cm": {"x": 820.0, "y": 830.0},
    }

    enemy_outpost = [item for item in snapshot["structures"] if item["key"] == "enemy_outpost"][0]
    assert enemy_outpost["field_side"] == "blue"
    assert enemy_outpost["hp"] == 0
    assert enemy_outpost["position_cm"] == {"x": 1707.0, "y": 1141.0}

    units = {(item["side"], item["type"]): item for item in snapshot["units"]}
    assert units[("enemy", "Hero")]["health_field"] == "hero"
    assert units[("enemy", "Hero")]["unit_key"] == "hero"
    assert units[("enemy", "Hero")]["asset_key"] == "Hero"
    assert units[("enemy", "Hero")]["position_data"] == {"car_id": 101, "raw_x": 2555, "raw_y": 600}
    assert units[("enemy", "Hero")]["decision_badges"] == ["BT-HP", "BT-POS", "UnitInfo"]
    assert units[("enemy", "Hero")]["decision_summary"] == "BT:HP,POS,UI"
    assert units[("enemy", "Hero")]["decision_channels"]["target_selection_consumed_by_bt"] is True
    assert units[("enemy", "Hero")]["decision_channels"]["regional_defense_position_used_by_bt"] is True
    assert units[("enemy", "Drone")]["health_field"] is None
    assert units[("enemy", "Drone")]["health_published"] is False
    assert units[("enemy", "Drone")]["decision_badges"] == ["POS-pub", "NoUnitInfo"]
    assert units[("enemy", "Drone")]["decision_summary"] == "PUB:POS noUI"
    assert units[("enemy", "Drone")]["decision_channels"]["position_data_published"] is True
    assert units[("enemy", "Drone")]["decision_channels"]["position_consumed_by_bt"] is False
    assert units[("friend", "Sentry")]["field_side"] == "red"
    assert units[("friend", "Sentry")]["position_data"] == {"car_id": 7, "raw_x": 300, "raw_y": 1100}


def test_viewer_hides_trace_units_when_an_editable_scene_is_present() -> None:
    viewer = object.__new__(Viewer)
    viewer.layers = {}
    viewer.show_trace_units_with_scene = False
    viewer.sim_input_state = SimulatorInputState.with_defaults(field=FieldGeometry())

    assert viewer.show_trace_units() is True

    assert viewer.sim_input_state.apply_command(
        "place_unit",
        {"entity_id": "enemy:hero:a", "side": "enemy", "unit_key": "hero", "x": 1200, "y": 700},
    )
    assert viewer.show_trace_units() is False

    viewer.show_trace_units_with_scene = True
    assert viewer.show_trace_units() is True


def test_viewer_scene_piece_list_keeps_same_class_instances_distinct() -> None:
    viewer = object.__new__(Viewer)
    viewer.sim_input_state = SimulatorInputState.with_defaults(field=FieldGeometry())

    assert viewer.sim_input_state.apply_command(
        "place_unit",
        {"entity_id": "enemy:hero:a", "side": "enemy", "unit_key": "hero", "x": 1200, "y": 700},
    )
    assert viewer.sim_input_state.apply_command(
        "place_unit",
        {"entity_id": "enemy:hero:b", "side": "enemy", "unit_key": "hero", "x": 1300, "y": 700},
    )

    pieces = viewer.sim_scene_units()

    assert [unit.entity_id for unit, _ in pieces] == ["enemy:hero:a", "enemy:hero:b"]
