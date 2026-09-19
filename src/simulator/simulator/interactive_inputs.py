from __future__ import annotations

import json
from copy import deepcopy
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Any

from .control_bus import command_name
from .field import FieldGeometry, relative_side_to_field_side
from .scene import SceneCommand, SceneState, normalize_scene_command
from .tactical_catalog import SceneCatalog, UnitArchetype, default_catalog_path


PointCm = tuple[float, float]


def as_dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def as_list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def parse_position(value: Any) -> PointCm | None:
    if isinstance(value, dict):
        raw_x = value.get("x")
        raw_y = value.get("y")
    elif isinstance(value, (list, tuple)) and len(value) >= 2:
        raw_x = value[0]
        raw_y = value[1]
    else:
        return None
    try:
        return (float(raw_x), float(raw_y))
    except (TypeError, ValueError):
        return None


def clamp_int(value: Any, low: int, high: int, default: int) -> int:
    try:
        parsed = int(round(float(value)))
    except (TypeError, ValueError):
        return default
    return max(low, min(high, parsed))


def normalize_side(value: Any) -> str | None:
    text = str(value).strip().lower()
    if text in {"friend", "self", "ally", "our", "me"}:
        return "friend"
    if text in {"enemy", "opponent"}:
        return "enemy"
    return None


def normalize_structure(value: Any) -> str | None:
    text = str(value).strip().lower()
    if text in {"outpost", "op", "op_hp"}:
        return "outpost"
    if text in {"base", "base_hp"}:
        return "base"
    return None


def normalize_bool(value: Any, default: bool) -> bool:
    if value is None:
        return default
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "y", "on"}:
        return True
    if text in {"0", "false", "no", "n", "off"}:
        return False
    return default


def point_payload(position: PointCm | None) -> dict[str, float] | None:
    if position is None:
        return None
    return {"x": float(position[0]), "y": float(position[1])}


def hp_ratio(hp: int, max_hp: int) -> float:
    return round(max(0.0, min(1.0, int(hp) / max(1, int(max_hp)))), 4)


@lru_cache(maxsize=1)
def default_scene_catalog() -> SceneCatalog:
    return SceneCatalog.load(default_catalog_path())


def _unit_for_type_id(
    type_id: int,
    catalog: SceneCatalog | None = None,
    side: str | None = None,
) -> UnitArchetype | None:
    active_catalog = catalog or default_scene_catalog()
    normalized = int(type_id)
    for unit in active_catalog.units:
        if unit.position_car_id == normalized:
            return unit
    resolved_side = normalize_side(side)
    candidate_sides = (resolved_side,) if resolved_side is not None else ("friend", "enemy")
    for candidate_side in candidate_sides:
        try:
            return active_catalog.unit_by_position_car_id(normalized, candidate_side)
        except KeyError:
            continue
    return None


def _unit_for_name(value: Any, catalog: SceneCatalog | None = None) -> UnitArchetype | None:
    text = str(value).strip().lower()
    if not text:
        return None
    active_catalog = catalog or default_scene_catalog()
    for unit in active_catalog.units:
        if text in {unit.key.lower(), unit.label.lower(), unit.asset_key.lower()}:
            return unit
    return None


def unit_type_id(payload: dict[str, Any], catalog: SceneCatalog | None = None) -> int | None:
    raw_type_id = payload.get(
        "type_id",
        payload.get("unit_type_id", payload.get("id", payload.get("car_id"))),
    )
    if raw_type_id is not None:
        try:
            type_id = int(raw_type_id)
        except (TypeError, ValueError):
            type_id = 0
        unit = _unit_for_type_id(type_id, catalog, normalize_side(payload.get("side")))
        if unit is not None:
            return int(unit.position_car_id or 0)
    unit = _unit_for_name(payload.get("type", payload.get("unit", payload.get("name", ""))), catalog)
    return None if unit is None else int(unit.position_car_id or 0)


def default_unit_hp(type_id: int, catalog: SceneCatalog | None = None) -> int:
    unit = _unit_for_type_id(type_id, catalog)
    return int(unit.default_hp) if unit is not None else 0


def default_unit_name(type_id: int, catalog: SceneCatalog | None = None) -> str:
    unit = _unit_for_type_id(type_id, catalog)
    return unit.label if unit is not None else f"Unit{type_id}"


def unit_decision_channels(side: str, type_id: int) -> dict[str, Any]:
    normalized_side = normalize_side(side) or str(side).strip().lower()
    catalog = default_scene_catalog()
    unit = _unit_for_type_id(type_id, catalog, normalized_side)
    if unit is None:
        return {
            "health_topic": "",
            "health_field": None,
            "health_published": False,
            "health_consumed_by_bt": False,
            "position_topic": "",
            "position_data_published": False,
            "position_consumed_by_bt": False,
            "unit_info_emitted_by_bt": False,
            "target_selection_consumed_by_bt": False,
            "regional_defense_position_used_by_bt": False,
            "visual_piece": False,
        }
    health_consumed = unit.decision_consumed and unit.health_published
    position_consumed = unit.decision_consumed and unit.position_published
    return {
        "health_topic": catalog.health_topic_for_side(normalized_side) if unit.health_published else "",
        "health_field": unit.health_field,
        "health_published": unit.health_published,
        "health_consumed_by_bt": health_consumed,
        "position_topic": catalog.position_data_topic if unit.position_published else "",
        "position_data_published": unit.position_published,
        "position_consumed_by_bt": position_consumed,
        "unit_info_emitted_by_bt": unit.decision_consumed,
        "target_selection_consumed_by_bt": normalized_side == "enemy" and health_consumed,
        "regional_defense_position_used_by_bt": normalized_side == "enemy" and position_consumed,
        "visual_piece": True,
    }


def unit_decision_badges(side: str, type_id: int) -> list[str]:
    channels = unit_decision_channels(side, type_id)
    badges: list[str] = []
    if channels["health_consumed_by_bt"]:
        badges.append("BT-HP")
    elif channels["health_published"]:
        badges.append("HP-pub")
    if channels["position_consumed_by_bt"]:
        badges.append("BT-POS")
    elif channels["position_data_published"]:
        badges.append("POS-pub")
    badges.append("UnitInfo" if channels["unit_info_emitted_by_bt"] else "NoUnitInfo")
    return badges


def unit_decision_summary(side: str, type_id: int) -> str:
    channels = unit_decision_channels(side, type_id)
    bt_parts: list[str] = []
    published_parts: list[str] = []
    if channels["health_consumed_by_bt"]:
        bt_parts.append("HP")
    elif channels["health_published"]:
        published_parts.append("HP")
    if channels["position_consumed_by_bt"]:
        bt_parts.append("POS")
    elif channels["position_data_published"]:
        published_parts.append("POS")
    if channels["unit_info_emitted_by_bt"]:
        bt_parts.append("UI")
    parts: list[str] = []
    if bt_parts:
        parts.append("BT:" + ",".join(bt_parts))
    if published_parts:
        parts.append("PUB:" + ",".join(published_parts))
    if not channels["unit_info_emitted_by_bt"]:
        parts.append("noUI")
    return " ".join(parts) if parts else "visual"


def unit_scene_payload(
    raw: dict[str, Any],
    fallback_side: str | None = None,
    catalog: SceneCatalog | None = None,
) -> dict[str, Any] | None:
    active_catalog = catalog or default_scene_catalog()
    side = normalize_side(raw.get("side", fallback_side))
    type_id = unit_type_id(raw, active_catalog)
    position = (
        parse_position(raw.get("position_cm"))
        or parse_position(raw.get("position"))
        or parse_position(raw.get("pos"))
        or parse_position(raw)
    )
    if side is None or type_id is None or position is None:
        return None
    archetype = _unit_for_type_id(type_id, active_catalog)
    if archetype is None:
        return None
    hp = clamp_int(raw.get("hp", raw.get("health")), 0, archetype.max_hp, archetype.default_hp)
    return {
        "side": side,
        "type_id": int(archetype.position_car_id or 0),
        "type": archetype.label,
        "hp": hp,
        "max_hp": int(archetype.max_hp),
        "x": position[0],
        "y": position[1],
        **({"entity_id": raw["entity_id"]} if isinstance(raw.get("entity_id"), str) else {}),
    }


def unit_scene_items(scene: Any, catalog: SceneCatalog | None = None) -> list[dict[str, Any]]:
    root = scene.get("units", scene) if isinstance(scene, dict) else scene
    raw_items: list[tuple[Any, str | None]] = []
    if isinstance(root, dict):
        for side in ("friend", "enemy"):
            for item in as_list(root.get(side)):
                raw_items.append((item, side))
        if any(key in root for key in ("side", "type", "type_id", "unit_type_id", "id", "car_id")):
            raw_items.append((root, None))
    else:
        for item in as_list(root):
            raw_items.append((item, None))

    out: list[dict[str, Any]] = []
    for raw, fallback_side in raw_items:
        item = as_dict(raw)
        if not item:
            continue
        payload = unit_scene_payload(item, fallback_side=fallback_side, catalog=catalog)
        if payload is not None:
            out.append(payload)
    return out


def load_unit_scene_file(path: str | Path) -> list[dict[str, Any]]:
    scene_path = Path(path).expanduser()
    with scene_path.open("r", encoding="utf-8") as stream:
        if scene_path.suffix.lower() in {".yaml", ".yml"}:
            try:
                import yaml
            except ImportError as exc:
                raise RuntimeError("YAML unit scenes require PyYAML") from exc
            try:
                scene = yaml.safe_load(stream) or {}
            except yaml.YAMLError as exc:
                raise ValueError(f"invalid YAML unit scene: {exc}") from exc
        else:
            try:
                scene = json.load(stream)
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid JSON unit scene: {exc}") from exc
    return unit_scene_items(scene)


@dataclass(frozen=True)
class StructureSpec:
    key: str
    label: str
    side: str
    structure: str
    hp: int
    max_hp: int
    step: int


@dataclass(frozen=True)
class UnitSpec:
    side: str
    type_id: int
    type_name: str
    hp: int
    max_hp: int

    def to_command_payload(self, x: int, y: int, hp: int | None = None) -> dict[str, Any]:
        return {
            "side": self.side,
            "type_id": self.type_id,
            "type": self.type_name,
            "hp": self.hp if hp is None else hp,
            "max_hp": self.max_hp,
            "x": x,
            "y": y,
        }


@dataclass(frozen=True)
class UnitState(UnitSpec):
    x: int
    y: int

    @property
    def key(self) -> tuple[str, int]:
        return (self.side, self.type_id)

    def to_command_payload(
        self,
        x: int | None = None,
        y: int | None = None,
        hp: int | None = None,
    ) -> dict[str, Any]:
        return {
            "side": self.side,
            "type_id": self.type_id,
            "type": self.type_name,
            "hp": self.hp if hp is None else hp,
            "max_hp": self.max_hp,
            "x": self.x if x is None else x,
            "y": self.y if y is None else y,
        }


@dataclass(frozen=True)
class PositionDataRow:
    side: str
    type_id: int
    formal_car_id: int
    raw_x: int
    raw_y: int

    @property
    def friend_car_id(self) -> int:
        return self.formal_car_id if self.side == "friend" else 0

    @property
    def enemy_car_id(self) -> int:
        return self.formal_car_id if self.side == "enemy" else 0


def _structure_specs(catalog: SceneCatalog) -> list[StructureSpec]:
    return [
        StructureSpec(
            key=item.key,
            label=item.label,
            side=item.side,
            structure=item.kind,
            hp=int(item.default_hp),
            max_hp=int(item.max_hp),
            step=int(item.step),
        )
        for item in catalog.structures
    ]


def _unit_palette(catalog: SceneCatalog) -> list[UnitSpec]:
    return [
        UnitSpec(
            side=side,
            type_id=int(item.position_car_id or 0),
            type_name=item.label,
            hp=int(item.default_hp),
            max_hp=int(item.max_hp),
        )
        for side in ("friend", "enemy")
        for item in catalog.units
        if item.position_car_id is not None
    ]


def _unit_palette_from_config(items: Any, catalog: SceneCatalog) -> list[UnitSpec]:
    if not isinstance(items, list):
        return []
    palette: list[UnitSpec] = []
    for item in items:
        if not isinstance(item, dict):
            continue
        side = normalize_side(item.get("side"))
        type_id = unit_type_id(item, catalog)
        if side is None or type_id is None:
            continue
        archetype = _unit_for_type_id(type_id, catalog, side)
        if archetype is None:
            continue
        try:
            hp = int(item.get("hp", archetype.default_hp))
            max_hp = int(item.get("max_hp", archetype.max_hp))
        except (TypeError, ValueError):
            continue
        palette.append(
            UnitSpec(
                side=side,
                type_id=type_id,
                type_name=str(item.get("type", archetype.label)),
                hp=hp,
                max_hp=max_hp,
            )
        )
    return palette


class SimulatorInputState:
    """Compatibility facade over :class:`SceneState` for existing simulator clients."""

    def __init__(
        self,
        structures: list[StructureSpec] | None = None,
        unit_palette: list[UnitSpec] | None = None,
        field: FieldGeometry | None = None,
        structure_positions: dict[str, dict[str, PointCm]] | None = None,
        structure_health_overrides: dict[tuple[str, str], int] | None = None,
        *,
        catalog: SceneCatalog | None = None,
        team: str = "red",
        ownership_mode: str = "mock",
    ) -> None:
        del structures, unit_palette, structure_positions
        self.catalog = catalog or default_scene_catalog()
        self.scene = SceneState.from_catalog(
            self.catalog,
            team,
            ownership_mode=ownership_mode,
            field=field,
        )
        self.field = self.scene.field
        self.structures = _structure_specs(self.catalog)
        self.unit_palette = _unit_palette(self.catalog)
        self.structure_positions = parse_structure_positions({}, catalog=self.catalog)
        sentry = _unit_for_name("sentry", self.catalog)
        self._self_health = int(sentry.default_hp) if sentry is not None else 0
        self._ammo_left = 200
        self._posture = 1
        self._self_position_cm: PointCm | None = None
        self._initial_units: Any = None
        self._initial_team = self.scene.team
        self._initial_structure_health = dict(self.scene.structure_health)
        for (side, structure), hp in (structure_health_overrides or {}).items():
            self.scene.apply(SceneCommand.set_structure_hp(f"{side}:{structure}", int(hp)))

    @classmethod
    def from_config(
        cls,
        simulator_inputs: dict[str, Any],
        field: FieldGeometry | None = None,
        structure_health_overrides: dict[tuple[str, str], int] | None = None,
    ) -> "SimulatorInputState":
        config = as_dict(simulator_inputs)
        state = cls(
            field=field,
            structure_health_overrides=structure_health_overrides,
            team=str(config.get("team", "red")),
            ownership_mode=str(config.get("input_owner", "mock")),
        )
        if "structures" in config and not config.get("structures"):
            state.structures = []
        if "unit_palette" in config:
            state.unit_palette = _unit_palette_from_config(config.get("unit_palette"), state.catalog)
        if "initial_units" in config:
            state._initial_units = deepcopy(config.get("initial_units"))
            state.apply_unit_scene(config.get("initial_units"), clear=True)
        return state

    @classmethod
    def with_defaults(
        cls,
        field: FieldGeometry | None = None,
        structure_health_overrides: dict[tuple[str, str], int] | None = None,
    ) -> "SimulatorInputState":
        return cls(field=field, structure_health_overrides=structure_health_overrides)

    @property
    def structure_health(self) -> dict[str, int]:
        return self.scene.structure_health

    @property
    def team(self) -> str:
        return self.scene.team

    @property
    def units(self) -> dict[tuple[str, int], UnitState]:
        grouped: dict[tuple[str, int], list[Any]] = {}
        for unit in self.scene.units.values():
            archetype = self.catalog.unit_by_key(unit.unit_key)
            if archetype.position_car_id is None:
                continue
            key = (unit.side, int(archetype.position_car_id))
            grouped.setdefault(key, []).append(unit)
        out: dict[tuple[str, int], UnitState] = {}
        for key, candidates in grouped.items():
            if len(candidates) != 1:
                continue
            unit = candidates[0]
            archetype = self.catalog.unit_by_key(unit.unit_key)
            out[key] = UnitState(
                side=unit.side,
                type_id=int(archetype.position_car_id),
                type_name=archetype.label,
                hp=int(unit.hp),
                max_hp=int(archetype.max_hp),
                x=int(unit.x),
                y=int(unit.y),
            )
        return out

    @property
    def self_health(self) -> int:
        return self._self_health

    @self_health.setter
    def self_health(self, value: int) -> None:
        self._self_health = clamp_int(value, 0, 65535, self._self_health)

    @property
    def ammo_left(self) -> int:
        return self._ammo_left

    @ammo_left.setter
    def ammo_left(self, value: int) -> None:
        self._ammo_left = clamp_int(value, 0, 65535, self._ammo_left)

    @property
    def posture(self) -> int:
        return self._posture

    @posture.setter
    def posture(self, value: int) -> None:
        self._posture = clamp_int(value, 0, 255, self._posture)

    @property
    def self_position_cm(self) -> PointCm | None:
        return self._self_position_cm

    def find_structure(self, side: str, structure: str) -> StructureSpec | None:
        normalized_side = normalize_side(side)
        normalized_structure = normalize_structure(structure)
        if normalized_side is None or normalized_structure is None:
            return None
        for item in self.structures:
            if item.side == normalized_side and item.structure == normalized_structure:
                return item
        return None

    def structure_hp(self, side: str, structure: str) -> int:
        item = self.find_structure(side, structure)
        return 0 if item is None else int(self.scene.structure_health[item.key])

    def structure_position(
        self,
        item: StructureSpec,
        team: str,
        goals: dict[int, dict[str, Any]],
    ) -> PointCm | None:
        del goals
        try:
            archetype = self.catalog.structure_by_key(item.key)
        except KeyError:
            return None
        field_side = relative_side_to_field_side(archetype.side, team)
        return archetype.position_for_field_side(field_side)

    def apply_control_payload(self, payload: dict[str, Any]) -> bool:
        return self.apply_command(command_name(payload), payload)

    def apply_command(self, command: str, payload: dict[str, Any]) -> bool:
        body = dict(payload or {})
        name = str(command).strip().lower()
        if self.scene.ownership_mode == "manual_ros":
            return False
        if name == "set_team":
            team = str(body.get("team", "")).strip().lower()
            if team not in {"red", "blue"}:
                return False
            if team != self.scene.team:
                for unit in list(self.scene.units.values()):
                    mirrored = SceneCommand.place_unit(
                        unit.entity_id,
                        unit.side,
                        unit.unit_key,
                        self.field.width - unit.x,
                        self.field.height - unit.y,
                        unit.hp,
                    )
                    self.scene.apply(mirrored)
                if self._self_position_cm is not None:
                    self._self_position_cm = (
                        self.field.width - self._self_position_cm[0],
                        self.field.height - self._self_position_cm[1],
                    )
            self.scene.team = team
            return True
        if name == "set_self_health":
            self.self_health = body.get("hp", body.get("health", body.get("self_health")))
            return True
        if name == "set_ammo":
            self.ammo_left = body.get("ammo", body.get("ammo_left", body.get("count")))
            return True
        if name == "set_posture":
            self.posture = body.get("posture", body.get("id", body.get("value")))
            return True
        if name == "set_self_position":
            position = (
                parse_position(body.get("position_cm"))
                or parse_position(body.get("position"))
                or parse_position(body.get("pos"))
                or parse_position(body)
            )
            if position is None:
                return False
            self._self_position_cm = (
                max(1, self.field.clamp_x(position[0])),
                max(1, self.field.clamp_y(position[1])),
            )
            return True
        if name == "set_units":
            self.apply_unit_scene(body.get("units", body), clear=normalize_bool(body.get("clear"), True))
            return True
        scene_command = self._legacy_scene_command(name, body)
        if scene_command is None:
            return False
        result = self.scene.apply(scene_command)
        return result.reason not in {
            "manual_ros_observer_mode",
            "invalid_scene_command",
            "invalid_unit_command",
            "unknown_structure",
            "unsupported_scene_command",
            "entity_not_found",
        }

    def reset(self) -> None:
        current_team = self.scene.team
        self.scene.team = self._initial_team
        if self._initial_units is not None:
            self.apply_unit_scene(deepcopy(self._initial_units), clear=True)
        else:
            self.apply_command("clear_units", {})
        self.scene.structure_health = dict(self._initial_structure_health)
        if current_team != self._initial_team:
            self.apply_command("set_team", {"team": current_team})
        sentry = next(
            (
                unit
                for unit in self.scene.units.values()
                if unit.side == "friend" and unit.unit_key == "sentry"
            ),
            None,
        )
        self._self_position_cm = None if sentry is None else (sentry.x, sentry.y)

    def apply_unit_scene(self, scene: Any, clear: bool = True) -> int:
        items = unit_scene_items(scene, catalog=self.catalog)
        if clear:
            self.apply_command("clear_units", {})
        count = 0
        for item in items:
            if self.apply_command("set_unit", item):
                count += 1
        return count

    def health_fields(self, side: str, base_hp: int) -> dict[str, int]:
        normalized_side = normalize_side(side)
        if normalized_side is None:
            return {}
        fields = {
            unit.health_field: int(base_hp)
            for unit in self.catalog.units
            if unit.health_published and unit.health_field is not None
        }
        projection = self.scene.project_ros_inputs()
        fields.update(projection.health[normalized_side])
        return fields

    def position_rows(self) -> list[PositionDataRow]:
        projection = self.scene.project_ros_inputs()
        rows: list[PositionDataRow] = []
        for side in ("friend", "enemy"):
            for car_id, point in projection.positions[side].items():
                try:
                    archetype = self.catalog.unit_by_position_car_id(car_id, side)
                except KeyError:
                    continue
                formal_car_id = self.catalog.position_car_id_for_side(archetype.key, side)
                if archetype.position_car_id is None or formal_car_id != car_id:
                    continue
                rows.append(
                    PositionDataRow(
                        side=side,
                        type_id=int(archetype.position_car_id),
                        formal_car_id=int(formal_car_id),
                        raw_x=int(point[0]),
                        raw_y=int(point[1]),
                    )
                )
        return sorted(rows, key=lambda item: (item.side, item.type_id))

    def snapshot(self, team: str | None = None, goals: dict[int, dict[str, Any]] | None = None) -> dict[str, Any]:
        del goals
        team = self.team if team is None else team
        projection = self.scene.project_ros_inputs()
        structures: list[dict[str, Any]] = []
        destroyed_structures: list[str] = []
        for item in self.structures:
            hp = int(self.scene.structure_health[item.key])
            if hp <= 0:
                destroyed_structures.append(item.key)
            structures.append(
                {
                    "key": item.key,
                    "label": item.label,
                    "side": item.side,
                    "field_side": relative_side_to_field_side(item.side, team),
                    "kind": item.structure,
                    "structure": item.structure,
                    "hp": hp,
                    "max_hp": item.max_hp,
                    "step": item.step,
                    "hp_ratio": hp_ratio(hp, item.max_hp),
                    "position_cm": point_payload(self.structure_position(item, team, {})),
                }
            )

        units: list[dict[str, Any]] = []
        counts = {"friend": 0, "enemy": 0}
        low_hp_units: list[str] = []
        for scene_unit in sorted(self.scene.units.values(), key=lambda item: item.entity_id):
            archetype = self.catalog.unit_by_key(scene_unit.unit_key)
            counts[scene_unit.side] += 1
            ratio = hp_ratio(scene_unit.hp, archetype.max_hp)
            if ratio <= 0.35:
                low_hp_units.append(f"{scene_unit.side}:{archetype.label}")
            type_id = int(archetype.position_car_id or 0)
            formal_id = self.catalog.position_car_id_for_side(archetype.key, scene_unit.side)
            units.append(
                {
                    "entity_id": scene_unit.entity_id,
                    "side": scene_unit.side,
                    "field_side": relative_side_to_field_side(scene_unit.side, team),
                    "unit_key": archetype.key,
                    "asset_key": archetype.asset_key,
                    "type_id": type_id,
                    "type": archetype.label,
                    "hp": int(scene_unit.hp),
                    "max_hp": int(archetype.max_hp),
                    "hp_ratio": ratio,
                    "health_field": archetype.health_field,
                    "health_published": archetype.health_published,
                    "decision_channels": unit_decision_channels(scene_unit.side, type_id),
                    "decision_badges": unit_decision_badges(scene_unit.side, type_id),
                    "decision_summary": unit_decision_summary(scene_unit.side, type_id),
                    "position_cm": {"x": scene_unit.x, "y": scene_unit.y},
                    "position_data": (
                        None
                        if formal_id is None
                        else {
                            "car_id": formal_id,
                            "raw_x": scene_unit.x,
                            "raw_y": self.field.position_data_raw_y(scene_unit.y),
                        }
                    ),
                }
            )
        palette: list[dict[str, Any]] = []
        for item in self.unit_palette:
            archetype = _unit_for_type_id(item.type_id, self.catalog)
            palette.append(
                {
                    "side": item.side,
                    "unit_key": archetype.key if archetype is not None else "",
                    "asset_key": archetype.asset_key if archetype is not None else "",
                    "type_id": item.type_id,
                    "type": item.type_name,
                    "hp": item.hp,
                    "max_hp": item.max_hp,
                    "health_field": archetype.health_field if archetype is not None else None,
                }
            )
        return {
            "team": "blue" if str(team).strip().lower() == "blue" else "red",
            "ownership_mode": self.scene.ownership_mode,
            "field": {
                "width_cm": self.field.width,
                "height_cm": self.field.height,
                "frame": self.catalog.field.frame,
            },
            "selected_entity_id": self.scene.selected_entity_id,
            "summary": {
                "unit_count": len(units),
                "friend_units": counts["friend"],
                "enemy_units": counts["enemy"],
                "structure_count": len(structures),
                "destroyed_structures": destroyed_structures,
                "low_hp_units": low_hp_units,
            },
            "runtime": {
                "self_health": self.self_health,
                "ammo_left": self.ammo_left,
                "posture": self.posture,
                "self_position_cm": point_payload(self.self_position_cm),
            },
            "structures": structures,
            "units": units,
            "palette": palette,
            "projection": projection.snapshot(),
        }

    def _legacy_scene_command(self, command: str, body: dict[str, Any]) -> SceneCommand | None:
        if command == "place_unit":
            try:
                return normalize_scene_command({"command": command, **body}, self.catalog)
            except ValueError:
                return None
        if command == "clear_units":
            return SceneCommand.clear_units()
        if command in {"set_structure_health", "set_structure_hp"}:
            side = normalize_side(body.get("side"))
            structure = normalize_structure(body.get("structure", body.get("kind")))
            if side is None or structure is None:
                return None
            item = self.find_structure(side, structure)
            if item is None:
                return None
            return SceneCommand.set_structure_hp(
                f"{side}:{structure}",
                clamp_int(
                    body.get("hp", body.get("health")),
                    0,
                    item.max_hp,
                    self.scene.structure_health[item.key],
                ),
            )
        if command == "set_unit":
            side = normalize_side(body.get("side"))
            type_id = unit_type_id(body, self.catalog)
            if side is None or type_id is None:
                return None
            archetype = _unit_for_type_id(type_id, self.catalog)
            if archetype is None:
                return None
            entity_id = body.get("entity_id") if isinstance(body.get("entity_id"), str) else f"{side}:{archetype.key}"
            return SceneCommand.place_unit(
                entity_id,
                side,
                archetype.key,
                self.field.clamp_x(body.get("x")),
                self.field.clamp_y(body.get("y")),
                clamp_int(body.get("hp", body.get("health")), 0, archetype.max_hp, archetype.default_hp),
            )
        if command == "set_unit_hp":
            entity_id = body.get("entity_id") if isinstance(body.get("entity_id"), str) else None
            if entity_id is None:
                side = normalize_side(body.get("side"))
                type_id = unit_type_id(body, self.catalog)
                archetype = _unit_for_type_id(type_id, self.catalog) if type_id is not None else None
                if side is None or archetype is None:
                    return None
                entity_id = f"{side}:{archetype.key}"
            existing = self.scene.units.get(entity_id)
            if existing is None:
                return None
            archetype = self.catalog.unit_by_key(existing.unit_key)
            return SceneCommand.set_unit_hp(
                entity_id,
                clamp_int(body.get("hp", body.get("health")), 0, archetype.max_hp, existing.hp),
            )
        if command == "remove_unit":
            entity_id = body.get("entity_id") if isinstance(body.get("entity_id"), str) else None
            if entity_id is None:
                side = normalize_side(body.get("side"))
                type_id = unit_type_id(body, self.catalog)
                archetype = _unit_for_type_id(type_id, self.catalog) if type_id is not None else None
                if side is None or archetype is None:
                    return None
                entity_id = f"{side}:{archetype.key}"
            return SceneCommand.remove_unit(entity_id)
        return None


def parse_structure_specs(config: dict[str, Any], catalog: SceneCatalog | None = None) -> list[StructureSpec]:
    del config
    return _structure_specs(catalog or default_scene_catalog())


def parse_unit_palette(config: dict[str, Any], catalog: SceneCatalog | None = None) -> list[UnitSpec]:
    del config
    return _unit_palette(catalog or default_scene_catalog())


def parse_structure_positions(
    config: dict[str, Any],
    catalog: SceneCatalog | None = None,
) -> dict[str, dict[str, PointCm]]:
    del config
    active_catalog = catalog or default_scene_catalog()
    positions: dict[str, dict[str, PointCm]] = {}
    for item in active_catalog.structures:
        kind_positions = positions.setdefault(item.kind, {})
        kind_positions.setdefault("red", item.position_for_field_side("red"))
        kind_positions.setdefault("blue", item.position_for_field_side("blue"))
    return positions
