from __future__ import annotations

import html
import json
import mimetypes
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping
from urllib.parse import quote, unquote

from .assets import UnitAssetCatalog, load_unit_asset_catalog
from .tactical_catalog import SceneCatalog, default_catalog_path


TACTICAL_STATE_SCHEMA = "ly_simulator_tactical_state_v1"


def as_dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def as_list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


@dataclass(frozen=True)
class TacticalAssetRegistry:
    """Expose only manifest-backed PNGs to the dependency-free tactical page."""

    root: Path | None
    catalog: UnitAssetCatalog

    @classmethod
    def load(cls) -> "TacticalAssetRegistry":
        catalog = load_unit_asset_catalog()
        root = catalog.manifest_path.parent.resolve() if catalog.manifest_path is not None else None
        return cls(root=root, catalog=catalog)

    def unit_url(self, field_side: str, asset_key: str) -> str | None:
        path = self.catalog.path_for(field_side, asset_key)
        if path is None or self.root is None:
            return None
        try:
            relative = path.resolve().relative_to(self.root)
        except ValueError:
            return None
        return "/assets/" + quote(relative.as_posix())

    def resolve_public_path(self, encoded_relative: str) -> tuple[Path | None, str | None]:
        if self.root is None:
            return None, None
        relative = Path(unquote(encoded_relative))
        if relative.is_absolute() or ".." in relative.parts or relative.suffix.lower() != ".png":
            return None, None
        candidate = (self.root / relative).resolve()
        try:
            candidate.relative_to(self.root)
        except ValueError:
            return None, None
        if not candidate.is_file():
            return None, None
        return candidate, mimetypes.guess_type(candidate.name)[0] or "image/png"


def _positive_int(value: Any, default: int) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return default
    return parsed if parsed > 0 else default


def _field(scene: Mapping[str, Any]) -> dict[str, Any]:
    raw = as_dict(scene.get("field"))
    return {
        "width_cm": _positive_int(raw.get("width_cm"), 2800),
        "height_cm": _positive_int(raw.get("height_cm"), 1500),
        "frame": str(raw.get("frame", "left_bottom_origin_cm")),
    }


def _scene_item(value: Any, registry: TacticalAssetRegistry | None) -> dict[str, Any]:
    item = dict(as_dict(value))
    field_side = str(item.get("field_side", "")).strip().lower()
    asset_key = str(item.get("asset_key", item.get("unit_key", item.get("type", "")))).strip()
    if registry is not None and field_side in {"red", "blue"} and asset_key:
        asset_url = registry.unit_url(field_side, asset_key)
        if asset_url is not None:
            item["asset_url"] = asset_url
    return item


def tactical_state_from_status(
    status: Mapping[str, Any], *, asset_registry: TacticalAssetRegistry | None = None
) -> dict[str, Any]:
    """Extract normalized scene facts; the page never derives simulation state."""

    root = as_dict(status)
    simulator_inputs = as_dict(root.get("simulator_inputs"))
    raw_scene = as_dict(simulator_inputs.get("state"))
    record = as_dict(root.get("current_record"))
    ownership_mode = str(raw_scene.get("ownership_mode", "mock")).strip().lower()
    if ownership_mode not in {"mock", "manual_ros"}:
        ownership_mode = "mock"
    enabled = bool(simulator_inputs.get("enabled", False))
    trace = as_dict(root.get("trace"))
    replay = as_dict(root.get("replay"))
    if ownership_mode != "mock":
        edit_reason = "manual_ros_observer_mode"
    elif not enabled:
        edit_reason = "simulator_inputs_disabled"
    elif not bool(root.get("control_enabled", False)):
        edit_reason = "control_file_unavailable"
    elif not bool(replay.get("controls_available", trace.get("controls_available", False))):
        edit_reason = "control_consumer_unavailable"
    else:
        edit_reason = "editable"
    goal = dict(as_dict(record.get("goal")))
    decision = as_dict(root.get("decision"))
    if goal.get("pos_cm") is None:
        resolved_goal = as_dict(decision.get("goal"))
        goal["pos_cm"] = resolved_goal.get("position_cm", resolved_goal.get("pos_cm"))
    return {
        "schema": TACTICAL_STATE_SCHEMA,
        "ready": bool(root.get("ready", False)),
        "field": _field(raw_scene),
        "scene": {
            "team": "blue" if str(raw_scene.get("team", "")).strip().lower() == "blue" else "red",
            "enabled": enabled,
            "ownership_mode": ownership_mode,
            "can_edit": edit_reason == "editable",
            "edit_reason": edit_reason,
            "show_trace_units": not bool(as_list(raw_scene.get("units"))),
            "selected_entity_id": raw_scene.get("selected_entity_id"),
            "units": [_scene_item(value, asset_registry) for value in as_list(raw_scene.get("units"))],
            "structures": [dict(as_dict(value)) for value in as_list(raw_scene.get("structures"))],
            "palette": [_scene_item(value, asset_registry) for value in as_list(raw_scene.get("palette"))],
            "projection": dict(as_dict(raw_scene.get("projection"))),
        },
        "decision": {
            "tick": record.get("tick"), "strategy": record.get("strategy"), "aim": record.get("aim"),
            "goal": goal, "intent": dict(as_dict(record.get("intent"))),
            "route_cm": list(as_list(decision.get("route_cm"))),
        },
        "gimbal_feedback": dict(as_dict(record.get("gimbal_feedback"))),
        "control_output": dict(as_dict(record.get("control_output"))),
        "tactical": dict(as_dict(record.get("tactical"))),
        "match": {
            "time_left": replay.get("match_time_left"),
            "duration_sec": _positive_int(replay.get("match_duration_sec"), 420),
            "running": bool(replay.get("match_running", False)),
            "started": bool(replay.get("match_started_user", replay.get("match_started", False))),
        },
        "stream": {"frame_url": "/frame.jpg", "map_url": "/tactical-map.png", "frame_id": root.get("frame_id")},
    }


def is_scene_mutation(command: str) -> bool:
    return str(command).strip().lower() in {
        "set_team",
        "place_unit", "set_unit", "set_units", "set_unit_hp", "remove_unit", "clear_units",
        "set_structure_health", "set_structure_hp", "set_self_health", "set_ammo", "set_posture",
        "set_self_position",
    }


def _map_objects() -> list[dict[str, Any]]:
    labels = {"home": "Spawn area", "recovery": "Supply area", "buff": "Buff area", "highland": "High ground"}
    objects: list[dict[str, Any]] = []
    for goal in SceneCatalog.load(default_catalog_path()).goals:
        label = labels.get(goal.category)
        if label is None:
            continue
        for field_side in ("red", "blue"):
            x, y = goal.position_for_field_side(field_side)
            objects.append({"key": f"{goal.key}:{field_side}", "label": label, "detail": goal.label, "field_side": field_side, "x": x, "y": y})
    return objects


def _build_workspace_html(map_objects: list[dict[str, Any]] | None = None) -> bytes:
    """The tactical browser's presentation-only docked workspace."""

    page = r'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>Sentinel Flight Deck</title>
<style>
:root{--fd-root:#111418;--fd-raised:#171C22;--fd-panel:#1D232B;--fd-card:#202832;--fd-border:rgba(255,255,255,.06);--fd-text:#F5F7FA;--fd-muted:#AEB7C2;--fd-accent:#4DB7FF;--fd-warning:#FFB648;--fd-danger:#FF6262;--fd-success:#58D68D;--bg:var(--fd-root);--chrome:var(--fd-raised);--surface:var(--fd-panel);--surface-2:var(--fd-card);--line:var(--fd-border);--text:var(--fd-text);--muted:var(--fd-muted);--accent:var(--fd-accent);--accent-soft:#16364C;--objective:var(--fd-warning);--danger:var(--fd-danger);--rail:56px;--inspector:320px;--shelf:40px;--gap:16px;--pad:24px}*{box-sizing:border-box}html,body{width:100%;min-width:320px;height:100%;margin:0;background:var(--bg);color:var(--text);font:15px/1.45 Inter,"Noto Sans",Arial,sans-serif}body{overflow:hidden}button,input{font:inherit;color:inherit}button{border:0;cursor:pointer}button:focus-visible,input:focus-visible{outline:2px solid var(--accent);outline-offset:2px}.hidden{display:none!important}.sr{position:absolute;width:1px;height:1px;overflow:hidden;clip:rect(0,0,0,0)}
#commandBar{height:80px;display:flex;align-items:center;gap:var(--gap);padding:12px var(--pad);border-bottom:1px solid var(--line);background:var(--chrome)}.brand{display:flex;align-items:center;gap:10px;min-width:238px}.mark{display:grid;place-items:center;width:36px;height:36px;border:1px solid var(--accent);border-radius:8px;background:#152c3b;color:#e1f2ff;font-weight:800}.brand strong{display:block;font-size:15px}.brand span{display:block;color:var(--muted);font-size:14px}.commands{display:flex;align-items:center;gap:8px;min-width:0;flex:1;justify-content:flex-end}.group{display:flex;align-items:center;gap:8px;padding-left:12px;border-left:1px solid var(--line)}.button,.icon-button,.seg button{height:40px;border:1px solid var(--line);border-radius:7px;background:var(--surface);padding:0 12px;transition:background .15s,border-color .15s,transform .15s}.icon-button{width:40px;padding:0}.button:hover,.icon-button:hover,.seg button:hover{background:var(--surface-2);border-color:var(--accent)}.button:active,.icon-button:active{transform:translateY(1px)}.button.primary{background:#214b69;border-color:var(--accent)}.button.danger{color:#ffd1d2;border-color:#834b50}.seg{display:flex;overflow:hidden;border:1px solid var(--line);border-radius:7px}.seg button{border:0;border-radius:0;min-width:52px}.seg button+button{border-left:1px solid var(--line)}.seg button[aria-pressed=true],.active{background:var(--accent-soft)!important;border-color:var(--accent)!important;color:#e4f4ff}.pill{padding:5px 9px;border:1px solid var(--line);border-radius:999px;color:var(--muted);white-space:nowrap}.pill.good{color:#c9e7ff;border-color:#4f7695}.pill.warn{color:#efc28d;border-color:#8d6849}
#workspaceShell{height:calc(100vh - 80px);display:grid;grid-template-columns:var(--rail) minmax(0,1fr) var(--inspector);grid-template-rows:minmax(0,1fr) var(--shelf);gap:var(--gap);padding:var(--pad);transition:grid-template-columns .18s ease,grid-template-rows .18s ease}#workspaceShell[data-inspector-dock=left]{grid-template-columns:var(--inspector) var(--rail) minmax(0,1fr)}#workspaceShell.inspector-collapsed{--inspector:48px}#workspaceShell.shelf-open{--shelf:180px}#workspaceShell.focus{grid-template-columns:0 minmax(0,1fr) 0}.panel{border:1px solid var(--line);border-radius:10px;background:var(--chrome);box-shadow:0 12px 30px #05080c45}
#activityRail{grid-row:1/3;display:flex;flex-direction:column;align-items:center;gap:10px;padding:10px 7px}.rail-button{display:grid;place-items:center;width:40px;height:40px;border:1px solid transparent;border-radius:7px;background:transparent;color:var(--muted);font-size:15px}.rail-button:hover{background:var(--surface);color:var(--text)}.rail-button[aria-pressed=true]{background:var(--accent-soft);border-color:var(--accent);color:#e2f3ff}.rail-spacer{flex:1}
#battlefieldWorkspace{grid-column:2;grid-row:1;display:flex;flex-direction:column;min-width:0;min-height:0}.workspace-head{display:flex;align-items:center;justify-content:space-between;min-height:40px;margin-bottom:10px;color:var(--muted)}.workspace-head strong{color:var(--text);font-size:15px}.viewport-tools{display:flex;gap:8px;align-items:center}.coordinate{font-variant-numeric:tabular-nums;color:var(--muted)}#mapViewport{position:relative;min-height:0;flex:1;overflow:hidden;border:1px solid var(--line);border-radius:10px;background:#202a32;box-shadow:0 14px 36px #05080c66;touch-action:none}#mapCanvas{position:absolute;top:50%;left:50%;width:100%;height:100%;transform-origin:center;will-change:transform}#fieldBoard{position:absolute;inset:0;overflow:hidden;background:#2a353d}#fieldImage{position:absolute;inset:0;width:100%;height:100%;pointer-events:none;user-select:none}body[data-side=blue] #fieldImage{transform:rotate(180deg)}#routeOverlay,#areaLayer,#structureLayer,#pieceLayer{position:absolute;inset:0}#routeOverlay{pointer-events:none;overflow:visible;z-index:2}#areaLayer{z-index:1}#structureLayer{z-index:3}#pieceLayer{z-index:4}.area,.structure{position:absolute;transform:translate(-50%,-50%)}.piece{position:absolute}.area{display:grid;place-items:center;width:28px;height:28px;border:1px solid #aab9c4;border-radius:50%;background:#17242de0;color:transparent}.area:after{content:"";width:8px;height:8px;border-radius:50%;background:var(--accent)}.area:hover,.area.selected,.structure.selected,.piece.selected .ring{outline:3px solid #69addf55;border-color:var(--accent)}.structure{display:grid;place-items:center;min-width:62px;min-height:42px;padding:5px 8px;border:1px solid #91a7b6;border-radius:6px;background:#121a21e9;color:var(--text);font-size:14px;font-weight:700}.structure.base{border-color:var(--accent)}.structure.outpost{border-color:var(--objective)}.piece{width:52px;height:52px;margin:-26px 0 0 -26px;padding:0;background:transparent;touch-action:none;cursor:grab}.piece.dragging{cursor:grabbing;z-index:10}.ring{position:absolute;inset:1px;overflow:hidden;border:2px solid #6d9ccb;border-radius:50%;background:#11191f}.piece.enemy .ring{border-color:#bd7074}.piece img,.fallback{display:block;width:100%;height:100%;object-fit:contain}.fallback{display:grid;place-items:center;background:var(--surface);font-weight:700}.hp{position:absolute;right:0;bottom:-6px;left:0;height:6px;overflow:hidden;border:1px solid #10161b;border-radius:4px;background:#53616c}.hp i{display:block;height:100%;background:var(--accent)}.low .hp i{background:var(--danger)}#goalMarker{position:absolute;z-index:5;width:38px;height:38px;margin:-19px 0 0 -19px;border:2px solid var(--objective);border-radius:50%;box-shadow:0 0 0 4px #d49a6133;pointer-events:none}#goalLabel{position:absolute;z-index:5;max-width:180px;padding:4px 7px;border:1px solid #8b6648;border-radius:5px;background:#282119;color:#f0c998;font-size:14px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;pointer-events:none}
#inspector{grid-column:3;grid-row:1;position:relative;min-width:0;overflow:auto;padding:20px;background:var(--chrome)}#workspaceShell[data-inspector-dock=left] #inspector{grid-column:1}#workspaceShell[data-inspector-dock=left] #activityRail{grid-column:2}#workspaceShell[data-inspector-dock=left] #battlefieldWorkspace{grid-column:3}#inspectorDockHandle{position:absolute;top:0;bottom:0;left:-5px;width:10px;cursor:col-resize}#workspaceShell[data-inspector-dock=left] #inspectorDockHandle{right:-5px;left:auto}.inspector-title{display:flex;align-items:flex-start;justify-content:space-between;gap:8px;margin-bottom:18px}.eyebrow{margin:0;color:var(--muted)}.inspector-title h2{margin:2px 0 0;font-size:20px}.card{margin-bottom:12px;overflow:hidden;border:1px solid var(--line);border-radius:9px;background:var(--surface)}.card summary{display:flex;align-items:center;justify-content:space-between;min-height:48px;padding:0 16px;cursor:pointer;list-style:none;font-weight:700}.card summary::-webkit-details-marker{display:none}.card summary small{margin-left:auto;margin-right:12px;color:var(--muted);font-weight:400}.card summary:after{content:"+";color:var(--muted);font-size:20px}.card[open] summary:after{content:"−";color:var(--accent)}.card-body{padding:16px;border-top:1px solid var(--line)}.kv{display:grid;grid-template-columns:minmax(98px,.8fr) minmax(0,1.2fr);gap:9px 14px;margin:0}.kv dt{color:var(--muted)}.kv dd{margin:0;overflow-wrap:anywhere}.meter{height:8px;margin-top:12px;overflow:hidden;border-radius:4px;background:#52616d}.meter i{display:block;height:100%;background:var(--accent)}.hp-editor{display:grid;gap:10px;margin-top:16px}.hp-editor input{height:40px;width:100%;padding:0 11px;border:1px solid var(--line);border-radius:6px;background:#11181e}.actions{display:flex;flex-wrap:wrap;gap:8px}.actions button{height:40px;min-width:56px;padding:0 10px;border:1px solid var(--line);border-radius:6px;background:var(--surface-2)}.actions .danger{color:#ffd0d2;border-color:#804a4e}.actions .primary{border-color:var(--accent);background:#214b69}.roster{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px}.roster button{display:flex;align-items:center;gap:8px;min-height:48px;padding:6px 8px;border:1px solid var(--line);border-radius:6px;background:var(--surface-2);text-align:left}.roster img{width:32px;height:32px;object-fit:contain}.notice{padding:10px;border-left:3px solid var(--objective);background:#2a241c;color:#eec28f}.debug{display:none}.debug-on .debug{display:block}
#operationsShelf{grid-column:2/4;grid-row:2;position:relative;overflow:auto;padding:12px 16px;background:var(--chrome)}#workspaceShell[data-inspector-dock=left] #operationsShelf{grid-column:1/4}#shelfResizeHandle{position:absolute;top:-5px;right:0;left:0;height:10px;cursor:row-resize}.shelf-head{display:flex;align-items:center;justify-content:space-between;gap:12px;height:18px}.shelf-head strong{font-size:14px}.shelf-data{display:none;grid-template-columns:repeat(4,minmax(0,1fr));gap:12px;margin-top:16px}.shelf-open .shelf-data{display:grid}.metric{padding:12px;border:1px solid var(--line);border-radius:7px;background:var(--surface)}.metric span{display:block;color:var(--muted)}.metric strong{display:block;margin-top:3px;font-size:16px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
@media(max-width:1100px){#commandBar{height:auto;min-height:80px;flex-wrap:wrap}.brand{min-width:200px}.commands{justify-content:flex-start;flex-wrap:wrap}#workspaceShell{height:calc(100vh - 128px);padding:16px;--pad:16px;--gap:12px;--inspector:300px}}@media(max-width:820px){body{overflow:auto}#workspaceShell,#workspaceShell[data-inspector-dock=left]{height:auto;min-height:calc(100vh - 128px);grid-template-columns:52px minmax(0,1fr);grid-template-rows:minmax(460px,70vh) auto auto}.panel{border-radius:8px}#activityRail,#workspaceShell[data-inspector-dock=left] #activityRail{grid-column:1;grid-row:1/3}#battlefieldWorkspace,#workspaceShell[data-inspector-dock=left] #battlefieldWorkspace{grid-column:2;grid-row:1}#inspector,#workspaceShell[data-inspector-dock=left] #inspector{grid-column:2;grid-row:2;min-height:320px}#operationsShelf,#workspaceShell[data-inspector-dock=left] #operationsShelf{grid-column:1/3;grid-row:3;min-height:40px}#inspectorDockHandle,#shelfResizeHandle{display:none}.shelf-open .shelf-data{grid-template-columns:repeat(2,minmax(0,1fr))}}@media(max-width:560px){#commandBar{padding:12px}.brand span,.pill{display:none}.group{border-left:0;padding-left:0}.button.label{padding:0 9px}.workspace-head{margin-bottom:8px}.roster{grid-template-columns:1fr}.structure{font-size:12px}}
</style></head><body data-side="red"><header id="commandBar"><div class="brand"><div class="mark">LY</div><div><strong>Sentinel Flight Deck</strong><span>Regional decision rehearsal</span></div></div><div class="commands"><div class="group seg" aria-label="Field perspective"><button id="viewSideRed" aria-pressed="true">Red</button><button id="viewSideBlue" aria-pressed="false">Blue</button></div><div class="group" id="matchActions"><button class="button primary" data-command="start">Start</button><button class="button" data-command="pause">Pause</button><button class="button" data-command="reset">Reset</button></div><div class="group"><button class="icon-button" id="fitView" title="Fit to window">⌗</button><button class="icon-button" id="actualSize" title="Actual size">1:1</button><button class="icon-button" id="mapZoomOut" title="Zoom out">−</button><button class="icon-button" id="mapZoomIn" title="Zoom in">+</button><button class="button label" id="viewportZoomSelection">Focus</button></div><div class="group"><button class="icon-button" id="debugToggle" aria-pressed="false" title="Debug">◈</button><button class="icon-button" id="focusToggle" aria-pressed="false" title="Focus mode">◫</button><button class="icon-button" id="fullscreenToggle" aria-pressed="false" title="Fullscreen">⛶</button><button class="icon-button" id="resetLayout" title="Reset layout">↺</button></div></div></header><main id="workspaceShell" data-inspector-dock="right" data-selection-kind="overview"><nav id="activityRail" class="panel" aria-label="Workspace surfaces"><button class="rail-button" data-surface="overview" aria-pressed="true" title="Overview">◎</button><button class="rail-button" data-surface="units" title="Units">♙</button><button class="rail-button" data-surface="decision" title="Decision">⌁</button><button class="rail-button" data-surface="events" title="Events">◌</button><button class="rail-button" data-surface="control" title="Control">⌘</button><button class="rail-button" data-surface="layers" title="Layers">▤</button><button class="rail-button debug" data-surface="runtime" title="Runtime">◫</button><span class="rail-spacer"></span><button class="rail-button" id="dockToggle" title="Move Inspector">⇄</button><button class="rail-button" id="shelfToggle" title="Operations shelf">▰</button></nav><section id="battlefieldWorkspace"><div class="workspace-head"><div><strong id="goalSummary">Waiting for decision trace</strong><span class="coordinate" id="coordinateReadout">0, 0 cm</span></div><div class="viewport-tools"><span class="pill" id="ownerPill">mock</span><span class="pill" id="decisionPill">goal -</span><span class="pill" id="controlPill">control -</span></div></div><div id="mapViewport"><div id="mapCanvas"><div id="fieldBoard"><img id="fieldImage" alt="Official RoboMaster field map"><svg id="routeOverlay"><polyline id="routeLine" fill="none" stroke="#69addf" stroke-width="3"></polyline></svg><div id="areaLayer"></div><div id="structureLayer"></div><div id="pieceLayer"></div><div id="goalMarker" hidden></div><div id="goalLabel" hidden></div></div></div></div></section><aside id="inspector" class="panel" aria-label="Inspector"><div id="inspectorDockHandle"></div><div class="inspector-title"><div><p class="eyebrow" id="inspectorEyebrow">BATTLEFIELD</p><h2 id="inspectorTitle">Overview</h2></div><button class="icon-button" id="undoButton" disabled title="Undo">↶</button></div><p id="ownerMessage" class="notice" hidden></p><div id="inspectorCards"></div></aside><section id="operationsShelf" class="panel"><div id="shelfResizeHandle"></div><div class="shelf-head"><strong>Operations shelf</strong><span id="operationSummary">Timeline and runtime are available here.</span></div><div class="shelf-data" id="shelfData"></div></section></main><div id="tactical" class="sr" aria-live="polite"></div>
<script>const MAP_OBJECTS=__MAP_OBJECTS__;let state=null,selection={kind:'overview',key:''},paletteChoice=null,lastUndo=null,drag=null,placement=null,pan=null;const storageKey='ly.simulator.workspace.v2';const defaults={side:'red',zoom:1,panX:0,panY:0,inspectorWidth:320,inspectorDock:'right',inspectorCollapsed:false,shelfHeight:180,shelfOpen:false,debug:false,focus:false,surface:'overview'};let view={...defaults,...JSON.parse(localStorage.getItem(storageKey)||'{}')};let inspectorInitialized=false,inspectorNeedsRender=true;const $=id=>document.getElementById(id),num=(v,d=1)=>typeof v==='number'&&Number.isFinite(v)?v.toFixed(d):'-',text=(v,f='-')=>v===null||v===undefined||v===''?f:String(v),clamp=(v,a,b)=>Math.max(a,Math.min(b,v)),dict=v=>v&&typeof v==='object'?v:{};const root=$('workspaceShell'),viewport=$('mapViewport'),canvas=$('mapCanvas'),board=$('fieldBoard'),areas=$('areaLayer'),structures=$('structureLayer'),pieces=$('pieceLayer'),route=$('routeLine'),goalMarker=$('goalMarker'),goalLabel=$('goalLabel');function persist(){localStorage.setItem(storageKey,JSON.stringify(view))}function field(){return state.field}function displayFraction(point){const x=Number(point.x)/field().width_cm,y=1-Number(point.y)/field().height_cm;return view.side==='red'?{x,y}:{x:1-x,y:1-y}}function position(el,point){const p=displayFraction(point);el.style.left=(p.x*100)+'%';el.style.top=(p.y*100)+'%'}function fitCanvas(){if(!state)return;const a=field().width_cm/field().height_cm,v=viewport.clientWidth/Math.max(1,viewport.clientHeight),w=v>a?viewport.clientHeight*a:viewport.clientWidth,h=w/a;canvas.style.width=w+'px';canvas.style.height=h+'px'}function applyView(){fitCanvas();root.dataset.inspectorDock=view.inspectorDock;root.dataset.selectionKind=selection.kind;root.classList.toggle('inspector-collapsed',view.inspectorCollapsed);root.classList.toggle('shelf-open',view.shelfOpen);root.classList.toggle('focus',view.focus);root.style.setProperty('--inspector',view.inspectorWidth+'px');root.style.setProperty('--shelf',view.shelfHeight+'px');document.body.dataset.side=view.side;document.body.classList.toggle('debug-on',view.debug);canvas.style.transform='translate(calc(-50% + '+view.panX+'px),calc(-50% + '+view.panY+'px)) scale('+view.zoom+')';$('viewSideRed').setAttribute('aria-pressed',String(view.side==='red'));$('viewSideBlue').setAttribute('aria-pressed',String(view.side==='blue'));$('debugToggle').setAttribute('aria-pressed',String(view.debug));$('focusToggle').setAttribute('aria-pressed',String(view.focus));$('fullscreenToggle').setAttribute('aria-pressed',String(Boolean(document.fullscreenElement)));for(const b of document.querySelectorAll('.rail-button[data-surface]'))b.setAttribute('aria-pressed',String(b.dataset.surface===view.surface));persist()}function readPoint(event){const r=canvas.getBoundingClientRect(),fx=clamp((event.clientX-r.left)/r.width,0,1),fy=clamp((event.clientY-r.top)/r.height,0,1);return view.side==='red'?{x:Math.round(fx*field().width_cm),y:Math.round((1-fy)*field().height_cm)}:{x:Math.round((1-fx)*field().width_cm),y:Math.round(fy*field().height_cm)}}function zoomAt(event,factor){const p=readPoint(event),vf=displayFraction(p),r=viewport.getBoundingClientRect(),baseW=canvas.offsetWidth,baseH=canvas.offsetHeight,next=clamp(view.zoom*factor,.45,4);view.zoom=next;view.panX=event.clientX-(r.left+r.width/2)-(vf.x-.5)*baseW*next;view.panY=event.clientY-(r.top+r.height/2)-(vf.y-.5)*baseH*next;applyView()}function centerOn(point,zoom){if(!point)return;const p=displayFraction(point),r=viewport.getBoundingClientRect();view.zoom=zoom||view.zoom;view.panX=-(p.x-.5)*canvas.offsetWidth*view.zoom;view.panY=-(p.y-.5)*canvas.offsetHeight*view.zoom;applyView()}function fit(){view.zoom=1;view.panX=0;view.panY=0;applyView()}function select(kind,key){selection={kind,key:key||''};paletteChoice=null;inspectorNeedsRender=true;render(true)}function selected(){if(!state)return null;if(selection.kind==='unit')return state.scene.units.find(x=>x.entity_id===selection.key);if(selection.kind==='structure')return state.scene.structures.find(x=>x.key===selection.key);if(selection.kind==='area')return MAP_OBJECTS.find(x=>x.key===selection.key);return null}function command(payload){if(!state.scene.can_edit){$('tactical').textContent='Scene editing is unavailable.';return}fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)}).then(r=>r.json()).then(reply=>{$('tactical').textContent=reply.ok?'Command applied':text(reply.message,'Command rejected')}).catch(()=>{$('tactical').textContent='Control endpoint unavailable'})}function undoable(payload,inverse){lastUndo={payload,inverse};$('undoButton').disabled=false;command(payload)}function card(title,summary,body,open=false,id=''){return '<details class="card" '+(open?'open':'')+' '+(id?'id="'+id+'"':'')+'><summary>'+title+'<small>'+summary+'</small></summary><div class="card-body">'+body+'</div></details>'}function kv(rows){return '<dl class="kv">'+rows.map(r=>'<dt>'+r[0]+'</dt><dd>'+r[1]+'</dd>').join('')+'</dl>'}function hpEditor(item,kind){const hp=Number(item.hp||0),max=Math.max(1,Number(item.max_hp||hp||1)),step=kind==='structure'?(item.kind==='base'?500:100):50;return '<div class="meter"><i style="width:'+(100*hp/max)+'%"></i></div><div class="hp-editor"><input id="hpValue" type="number" min="0" max="'+max+'" value="'+hp+'"><div class="actions"><button data-hp-step="'+(-step)+'">−'+step+'</button><button data-hp-step="'+step+'">+'+step+'</button><button data-hp-value="'+max+'">Restore</button><button class="danger" data-hp-value="0">Destroy</button><button class="primary" data-apply="'+kind+'">Apply</button></div></div>'}function overviewCards(){const d=dict(state.decision),g=dict(d.goal),i=dict(d.intent),o=dict(state.control_output);return card('Overview',g.name?text(g.name)+' · '+text(i.reason):'Waiting for decision trace',kv([['Goal',text(g.name)+' #'+text(g.id)],['Layer',text(i.layer)],['Reason',text(i.reason)],['Position',Array.isArray(g.pos_cm)?g.pos_cm.join(', ')+' cm':'-'],['Control',o.available?'#'+text(o.sequence):'Unavailable']]),true,'overviewInspector')+card('Decision & navigation',text(d.strategy),kv([['Strategy',text(d.strategy)],['Aim',text(d.aim)],['Route',text((d.route_cm||[]).length)+' points'],['Output',text(dict(o.fire_code).published?'FireCode published':'No FireCode')]]);}
function structureCards(s){return card('Overview',text(s.label),kv([['Team',text(s.side)],['Coordinate',s.position_cm?text(s.position_cm.x)+', '+text(s.position_cm.y)+' cm':'-'],['State',Number(s.hp)>0?'Alive':'Destroyed'],['Health',text(s.hp)+' / '+text(s.max_hp)]]),true)+card('Health',text(s.hp)+' / '+text(s.max_hp),hpEditor(s,'structure'))+card('Decision impact','Current trace evidence',kv([['Goal',text(dict(state.decision).goal&&dict(state.decision).goal.name)],['Tactical',text(dict(state.tactical).available?'Available':'Not recorded')]]);}
function unitCards(u){const d=dict(state.decision),g=dict(d.goal);return card('Overview',text(u.type),kv([['Side',text(u.side)],['Coordinate',u.position_cm?text(u.position_cm.x)+', '+text(u.position_cm.y)+' cm':'-'],['HP',text(u.hp)+' / '+text(u.max_hp)],['Current goal',text(g.name)]]),true)+card('Health',text(u.hp)+' / '+text(u.max_hp),hpEditor(u,'unit'))+card('Decision',text(d.strategy),kv([['Behavior',text(dict(d.intent).layer)],['Target',text(dict(d.intent).reason)],['Route',text((d.route_cm||[]).length)+' points']]);}
function areaCards(a){const d=dict(state.decision);return card('Overview',text(a.label),kv([['Category',text(a.label)],['Coordinate',text(a.x)+', '+text(a.y)+' cm'],['Field side',text(a.field_side)],['Current goal',text(dict(d.goal).name)]]),true)+card('Navigation context',text(dict(d.intent).reason),kv([['Strategy',text(d.strategy)],['Layer',text(dict(d.intent).layer)],['Priority',text(dict(d.intent).priority)]]);}
function surfaceCards(){const d=dict(state.decision),t=dict(state.tactical),out=dict(state.control_output),feedback=dict(state.gimbal_feedback);if(view.surface==='units'){const items=state.scene.palette||[];return card('Unit palette',paletteChoice?text(paletteChoice.type):'Choose a piece', '<div class="roster" id="palette">'+items.map((u,n)=>'<button data-palette="'+n+'">'+(u.asset_url?'<img src="'+u.asset_url+'">':'<span class="mark">'+text(u.type).slice(0,1)+'</span>')+'<span>'+text(u.type)+'</span></button>').join('')+'</div>',true)+card('Placed units',text(state.scene.units.length),kv(state.scene.units.map(u=>[text(u.type),text(u.hp)+' HP'])));if(view.surface==='decision')return card('Decision',text(d.strategy),kv([['Goal',text(dict(d.goal).name)],['Layer',text(dict(d.intent).layer)],['Reason',text(dict(d.intent).reason)],['Priority',text(dict(d.intent).priority)]]),true);if(view.surface==='events')return card('Events',text(t.available?'Tactical evidence':'No tactical trace'),kv([['Protect Castle',text(dict(t.protect_castle).enabled)],['Protect Outpost',text(dict(t.protect_outpost).active)],['Protect Hero',text(dict(t.protect_hero).active)],['Defense',text(dict(t.regional_defense).threat_active)]]),true);if(view.surface==='control')return card('Final control',text(out.source),kv([['Sequence',text(out.sequence)],['Angles',text(dict(out.angles).published?'Published':'Not published')],['FireCode',text(dict(out.fire_code).published?'Published':'Not published')],['Feedback age',text(feedback.age_ms)+' ms']]),true);if(view.surface==='layers')return card('Map layers','Current field presentation',kv([['Map',text(state.stream.map_url)],['Pieces',text(state.scene.units.length)],['Structures',text(state.scene.structures.length)],['Route',text((d.route_cm||[]).length)+' points']]),true);if(view.surface==='runtime')return card('Runtime diagnostics','Debug mode',kv([['Owner',text(state.scene.ownership_mode)],['Ready',text(state.ready)],['Control',text(out.available)],['Feedback',text(feedback.available)]]),true);return overviewCards()}
function inspectorEditState(){const details=[...document.querySelectorAll('#inspectorCards details')].map(d=>d.open),inputs={};document.querySelectorAll('#inspectorCards input').forEach((input,index)=>{const key=input.id||('input-'+index);if(input.dataset.dirty==='1')inputs[key]={value:input.value,start:input.selectionStart,end:input.selectionEnd}});const active=document.activeElement,focusId=active&&active.closest&&active.closest('#inspectorCards')?(active.id||''):'';return{details,inputs,focusId}}function restoreInspectorEditState(snapshot){[...document.querySelectorAll('#inspectorCards details')].forEach((d,index)=>{if(index<snapshot.details.length)d.open=snapshot.details[index]});document.querySelectorAll('#inspectorCards input').forEach((input,index)=>{const key=input.id||('input-'+index),saved=snapshot.inputs[key];if(!saved)return;input.value=saved.value;try{if(saved.start!=null)input.setSelectionRange(saved.start,saved.end)}catch(_){}});if(snapshot.focusId){const target=document.getElementById(snapshot.focusId);if(target&&target.closest('#inspectorCards'))target.focus()}}let renderedInspectorKey='';function renderInspector(){const item=selected(),title=$('inspectorTitle'),eyebrow=$('inspectorEyebrow'),key=selection.kind+'|'+selection.key+'|'+view.surface,sameSelection=key===renderedInspectorKey,editState=sameSelection?inspectorEditState():null,setCards=html=>{$('inspectorCards').innerHTML=html;if(editState)restoreInspectorEditState(editState)};renderedInspectorKey=key;if(selection.kind==='structure'&&item){eyebrow.textContent='STRUCTURE';title.textContent=text(item.label);setCards(structureCards(item))}else if(selection.kind==='unit'&&item){eyebrow.textContent='ROBOT';title.textContent=text(item.type);setCards(unitCards(item))}else if(selection.kind==='area'&&item){eyebrow.textContent='AREA';title.textContent=text(item.label);setCards(areaCards(item))}else{eyebrow.textContent=view.surface.toUpperCase();title.textContent=view.surface==='overview'?'Overview':view.surface[0].toUpperCase()+view.surface.slice(1);setCards(surfaceCards())}for(const b of document.querySelectorAll('[data-hp-step]'))b.onclick=()=>{const input=$('hpValue');if(!input)return;const min=Number(input.min)||0,max=Number(input.max)||min,next=clamp(Number(input.value||0)+Number(b.dataset.hpStep||0),min,max);input.value=String(next);input.dataset.dirty='1'};for(const b of document.querySelectorAll('[data-hp-value]'))b.onclick=()=>{const input=$('hpValue');if(!input)return;const min=Number(input.min)||0,max=Number(input.max)||min,next=clamp(Number(b.dataset.hpValue||0),min,max);input.value=String(next);input.dataset.dirty='1'};for(const b of document.querySelectorAll('[data-apply]'))b.onclick=()=>{const item=selected(),input=$('hpValue'),value=input?clamp(Number(input.value||0),Number(input.min)||0,Number(input.max)||0):0,payload=b.dataset.apply==='structure'?{command:'set_structure_health',side:item.side,structure:item.kind,hp:value}:{command:'set_unit_hp',entity_id:item.entity_id,hp:value};command(payload);inspectorNeedsRender=true};for(const b of document.querySelectorAll('[data-unit-select]'))b.onclick=()=>select('unit',b.dataset.unitSelect);for(const b of document.querySelectorAll('[data-palette]'))b.onclick=()=>{paletteChoice=state.scene.palette[Number(b.dataset.palette)];$('tactical').textContent='Placement armed: '+text(paletteChoice.type)};for(const input of document.querySelectorAll('#inspectorCards input')){input.addEventListener('input',()=>{input.dataset.dirty='1'});input.addEventListener('blur',()=>{delete input.dataset.dirty})}}function renderMap(){const f=field(),d=dict(state.decision),g=dict(d.goal),routePoints=(d.route_cm||[]).map(p=>Array.isArray(p)?{x:p[0],y:p[1]}:null).filter(Boolean);$('fieldImage').src=state.stream.map_url+'?frame='+encodeURIComponent(text(state.stream.frame_id,'0'));areas.innerHTML='';structures.innerHTML='';pieces.innerHTML='';for(const a of MAP_OBJECTS){const e=document.createElement('button');e.className='area '+(selection.kind==='area'&&selection.key===a.key?'selected':'');e.title=a.label+': '+a.detail;position(e,a);e.onclick=event=>{event.stopPropagation();select('area',a.key)};areas.append(e)}for(const s of state.scene.structures){if(!s.position_cm)continue;const e=document.createElement('button');e.className='structure '+text(s.kind).toLowerCase()+' '+(selection.kind==='structure'&&selection.key===s.key?'selected':'');e.textContent=text(s.kind)==='base'?'BASE':'OUTPOST';position(e,s.position_cm);e.onclick=event=>{event.stopPropagation();select('structure',s.key)};structures.append(e)}for(const u of state.scene.units){if(!u.position_cm)continue;const e=document.createElement('button');e.className='piece '+(u.side==='enemy'?'enemy ':'')+(selection.kind==='unit'&&selection.key===u.entity_id?'selected ':'')+(Number(u.hp)<=.25*Number(u.max_hp||1)?'low':'');e.innerHTML='<span class="ring">'+(u.asset_url?'<img src="'+u.asset_url+'" alt="'+text(u.type)+'">':'<span class="fallback">'+text(u.type).slice(0,1)+'</span>')+'</span><span class="hp"><i style="width:'+clamp(100*Number(u.hp)/Math.max(1,Number(u.max_hp)),0,100)+'%"></i></span>';position(e,u.position_cm);e.onpointerdown=event=>{if(!state.scene.can_edit)return;drag={id:event.pointerId,unit:u,startX:event.clientX,startY:event.clientY,moved:false};viewport.setPointerCapture(event.pointerId);event.stopPropagation()};e.onclick=event=>{event.stopPropagation();if(!drag)select('unit',u.entity_id)};pieces.append(e)}const points=routePoints.map(p=>{const q=displayFraction(p);return(q.x*100)+','+(q.y*100)});route.setAttribute('points',points.join(' '));if(Array.isArray(g.pos_cm)&&g.pos_cm.length>=2){const p={x:g.pos_cm[0],y:g.pos_cm[1]};position(goalMarker,p);const q=displayFraction(p);goalLabel.style.left=(q.x*100)+'%';goalLabel.style.top='calc('+(q.y*100)+'% - 34px)';goalLabel.textContent=text(g.name)+' #'+text(g.id);goalMarker.hidden=false;goalLabel.hidden=false}else{goalMarker.hidden=true;goalLabel.hidden=true}}
function renderShelf(){const d=dict(state.decision),o=dict(state.control_output);$('operationSummary').textContent='Tick '+text(d.tick)+' · '+text(d.strategy);$('shelfData').innerHTML=[['Goal',text(dict(d.goal).name)],['Reason',text(dict(d.intent).reason)],['Control',o.available?'#'+text(o.sequence):'Unavailable'],['Match',text(state.match.time_left)+' s']].map(x=>'<div class="metric"><span>'+x[0]+'</span><strong>'+x[1]+'</strong></div>').join('')}
function render(forceInspector=false){if(!state)return;const readonly=!state.scene.can_edit,labels={manual_ros_observer_mode:'Manual ROS observer mode: Foxglove or ROS CLI owns scene facts.',simulator_inputs_disabled:'Scene input controls are disabled.',control_file_unavailable:'Scene command file is unavailable.',control_consumer_unavailable:'Replay viewer is read-only. Start offline decision mode to edit this board.'};$('ownerMessage').hidden=!readonly;$('ownerMessage').textContent=readonly?(labels[state.scene.edit_reason]||'Scene editing is unavailable.'):'';for(const b of document.querySelectorAll('#matchActions button'))b.disabled=readonly;$('ownerPill').textContent=state.scene.ownership_mode;$('ownerPill').className='pill '+(readonly?'warn':'good');const d=dict(state.decision),g=dict(d.goal),o=dict(state.control_output);$('goalSummary').textContent=g.name?text(g.name)+' · '+text(dict(d.intent).reason):'Waiting for decision trace';$('decisionPill').textContent=g.name?'goal '+text(g.id):'goal -';$('controlPill').textContent=o.available?'control '+text(o.sequence):'control -';renderMap();if(forceInspector||!inspectorInitialized||inspectorNeedsRender){renderInspector();inspectorInitialized=true;inspectorNeedsRender=false}renderShelf();applyView()}function refresh(){fetch('/api/tactical-state',{cache:'no-store'}).then(r=>r.json()).then(next=>{state=next;render()}).catch(()=>{$('tactical').textContent='Tactical state unavailable'})}
$('viewSideRed').onclick=()=>{view.side='red';render()};$('viewSideBlue').onclick=()=>{view.side='blue';render()};$('fitView').onclick=fit;$('actualSize').onclick=()=>{view.zoom=clamp(field().width_cm/Math.max(1,canvas.offsetWidth),.45,4);view.panX=0;view.panY=0;applyView()};$('mapZoomIn').onclick=()=>{view.zoom=clamp(view.zoom*1.18,.45,4);applyView()};$('mapZoomOut').onclick=()=>{view.zoom=clamp(view.zoom/1.18,.45,4);applyView()};$('viewportZoomSelection').onclick=()=>{const item=selected();if(item&&item.position_cm)centerOn(item.position_cm,Math.max(view.zoom,1.7));else if(selection.kind==='area'&&item)centerOn(item,Math.max(view.zoom,1.7));else fit()};$('debugToggle').onclick=()=>{view.debug=!view.debug;applyView()};$('focusToggle').onclick=()=>{view.focus=!view.focus;applyView()};$('dockToggle').onclick=()=>{view.inspectorDock=view.inspectorDock==='right'?'left':'right';applyView()};$('shelfToggle').onclick=()=>{view.shelfOpen=!view.shelfOpen;applyView()};$('resetLayout').onclick=()=>{view={...defaults};localStorage.removeItem(storageKey);applyView()};$('fullscreenToggle').onclick=()=>document.fullscreenElement?document.exitFullscreen():document.documentElement.requestFullscreen();document.addEventListener('fullscreenchange',applyView);$('undoButton').onclick=()=>{if(lastUndo){command(lastUndo.inverse);lastUndo=null;$('undoButton').disabled=true}};for(const b of document.querySelectorAll('[data-surface]'))b.onclick=()=>{view.surface=b.dataset.surface;select('overview','')};viewport.addEventListener('wheel',event=>{event.preventDefault();zoomAt(event,event.deltaY<0?1.14:1/1.14)},{passive:false});viewport.addEventListener('pointerdown',event=>{if(event.button===1||event.altKey||event.shiftKey){pan={id:event.pointerId,x:event.clientX,y:event.clientY,startX:view.panX,startY:view.panY};viewport.setPointerCapture(event.pointerId);return}if(event.target.closest('.piece,.structure,.area'))return;if(!state.scene.can_edit||!paletteChoice){select('overview','');return}placement={id:event.pointerId};viewport.setPointerCapture(event.pointerId)});viewport.addEventListener('pointermove',event=>{if(!state)return;const p=readPoint(event);$('coordinateReadout').textContent=p.x+', '+p.y+' cm';if(pan&&pan.id===event.pointerId){view.panX=pan.startX+event.clientX-pan.x;view.panY=pan.startY+event.clientY-pan.y;applyView()}if(drag&&drag.id===event.pointerId){drag.moved=drag.moved||Math.hypot(event.clientX-drag.startX,event.clientY-drag.startY)>4}});viewport.addEventListener('pointerup',event=>{if(pan&&pan.id===event.pointerId){pan=null;return}if(drag&&drag.id===event.pointerId){const done=drag;drag=null;if(done.moved){const p=readPoint(event);command({command:'place_unit',entity_id:done.unit.entity_id,side:done.unit.side,unit_key:done.unit.unit_key,hp:done.unit.hp,x:p.x,y:p.y})}else select('unit',done.unit.entity_id);return}if(placement&&placement.id===event.pointerId){placement=null;const p=readPoint(event);command({command:'place_unit',entity_id:paletteChoice.side+':'+paletteChoice.unit_key+':'+Date.now().toString(36),side:paletteChoice.side,unit_key:paletteChoice.unit_key,hp:paletteChoice.hp,x:p.x,y:p.y})}});viewport.addEventListener('dblclick',()=>{$('viewportZoomSelection').click()});new ResizeObserver(()=>{if(state)applyView()}).observe(viewport);$('inspectorDockHandle').addEventListener('pointerdown',event=>{const start=event.clientX,width=view.inspectorWidth,sign=view.inspectorDock==='right'?-1:1;const move=e=>{view.inspectorWidth=clamp(width+sign*(e.clientX-start),280,460);applyView()},stop=e=>{move(e);window.removeEventListener('pointermove',move);window.removeEventListener('pointerup',stop)};window.addEventListener('pointermove',move);window.addEventListener('pointerup',stop)});$('shelfResizeHandle').addEventListener('pointerdown',event=>{view.shelfOpen=true;const start=event.clientY,height=view.shelfHeight;const move=e=>{view.shelfHeight=clamp(height+start-e.clientY,140,300);applyView()},stop=e=>{move(e);window.removeEventListener('pointermove',move);window.removeEventListener('pointerup',stop)};window.addEventListener('pointermove',move);window.addEventListener('pointerup',stop)});for(const b of document.querySelectorAll('#matchActions button'))b.onclick=()=>{if(b.dataset.command==='reset')inspectorNeedsRender=true;command({command:b.dataset.command})};refresh();setInterval(refresh,1000);
</script></body></html>'''
    page = page.replace('class="panel"', 'class="panel flight-deck-card"')
    page = page.replace('class="card"', 'class="card flight-deck-card"')
    page = page.replace(
        "</style>",
        """.flight-deck-card{border:1px solid var(--fd-border);border-radius:12px;background:linear-gradient(180deg,rgba(255,255,255,.025),transparent 40%),var(--fd-card);box-shadow:0 14px 34px rgba(0,0,0,.22);transition:transform 150ms ease-out,box-shadow 150ms ease-out,border-color 150ms ease-out}.flight-deck-card:not(#battlefieldWorkspace):hover{transform:translateY(-1px);border-color:rgba(77,183,255,.34);box-shadow:0 18px 38px rgba(0,0,0,.28)}.lucide{display:inline-grid;place-items:center;width:16px;height:16px;vertical-align:-3px;margin-right:6px}.lucide svg{width:16px;height:16px;fill:none;stroke:currentColor;stroke-width:1.8}@media (prefers-reduced-motion: reduce){*,*:before,*:after{transition:none!important;scroll-behavior:auto!important}}</style>""",
        1,
    ).replace(
        '>Focus</button>',
        '><span class="lucide" data-lucide="crosshair" aria-hidden="true"><svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="5"/><path d="M12 2v4m0 12v4M2 12h4m12 0h4"/></svg></span>Focus</button>',
        1,
    ).replace(
        "]]);}\nfunction structureCards",
        "]]));}\nfunction structureCards",
    ).replace(
        "]] );}\nfunction unitCards".replace("]] ", "]]"),
        "]] ));}\nfunction unitCards".replace("]]  ", "]]"),
    )
    page = page.replace(
        "]] );}\nfunction areaCards".replace("]] ", "]]"),
        "]] ));}\nfunction areaCards".replace("]]  ", "]]"),
    ).replace(
        "]] );}\nfunction surfaceCards".replace("]] ", "]]"),
        "]] ));}\nfunction surfaceCards".replace("]]  ", "]]"),
    )
    page = re.sub(
        r"function overviewCards\(\).*?(?=function structureCards)",
        r'''function overviewCards(){const d=dict(state.decision),g=dict(d.goal),i=dict(d.intent),o=dict(state.control_output);return card('Overview',g.name?text(g.name)+' · '+text(i.reason):'Waiting for decision trace',kv([['Goal',text(g.name)+' #'+text(g.id)],['Layer',text(i.layer)],['Reason',text(i.reason)],['Position',Array.isArray(g.pos_cm)?g.pos_cm.join(', ')+' cm':'-'],['Control',o.available?'#'+text(o.sequence):'Unavailable']]),true,'overviewInspector')+card('Decision & navigation',text(d.strategy),kv([['Strategy',text(d.strategy)],['Aim',text(d.aim)],['Route',text((d.route_cm||[]).length)+' points'],['Output',text(dict(o.fire_code).published?'FireCode published':'No FireCode')]]));}
''',
        page,
        flags=re.DOTALL,
    )
    page = re.sub(
        r"function structureCards\(s\).*?(?=function unitCards)",
        r'''function structureCards(s){return card('Overview',text(s.label),kv([['Team',text(s.side)],['Coordinate',s.position_cm?text(s.position_cm.x)+', '+text(s.position_cm.y)+' cm':'-'],['State',Number(s.hp)>0?'Alive':'Destroyed'],['Health',text(s.hp)+' / '+text(s.max_hp)]]),true)+card('当前血量','<span id="hpCurrentSummary">'+text(s.hp)+' / '+text(s.max_hp)+'</span>',hpMeter(s.hp,s.max_hp),true)+card('血量设置','手动设置',hpEditor(s,'structure'),true)+card('Decision impact','Current trace evidence',kv([['Goal',text(dict(state.decision).goal&&dict(state.decision).goal.name)],['Tactical',text(dict(state.tactical).available?'Available':'Not recorded')]]));}
''',
        page,
        flags=re.DOTALL,
    )
    page = re.sub(
        r"function unitCards\(u\).*?(?=function areaCards)",
        r'''function unitCards(u){const d=dict(state.decision),g=dict(d.goal);return card('Overview',text(u.type),kv([['Side',text(u.side)],['Coordinate',u.position_cm?text(u.position_cm.x)+', '+text(u.position_cm.y)+' cm':'-'],['HP',text(u.hp)+' / '+text(u.max_hp)],['Current goal',text(g.name)]]),true)+card('当前血量','<span id="hpCurrentSummary">'+text(u.hp)+' / '+text(u.max_hp)+'</span>',hpMeter(u.hp,u.max_hp),true)+card('血量设置','手动设置',hpEditor(u,'unit'),true)+card('Decision',text(d.strategy),kv([['Behavior',text(dict(d.intent).layer)],['Target',text(dict(d.intent).reason)],['Route',text((d.route_cm||[]).length)+' points']]));}
''',
        page,
        flags=re.DOTALL,
    )
    page = re.sub(
        r"function areaCards\(a\).*?(?=function surfaceCards)",
        r'''function areaCards(a){const d=dict(state.decision);return card('Overview',text(a.label),kv([['Category',text(a.label)],['Coordinate',text(a.x)+', '+text(a.y)+' cm'],['Field side',text(a.field_side)],['Current goal',text(dict(d.goal).name)]]),true)+card('Navigation context',text(dict(d.intent).reason),kv([['Strategy',text(d.strategy)],['Layer',text(dict(d.intent).layer)],['Priority',text(dict(d.intent).priority)]]));}
''',
        page,
        flags=re.DOTALL,
    )
    page = re.sub(
        r"function surfaceCards\(\).*?(?=function inspectorEditState|function restoreInspectorEditState|function renderInspector)",
        r'''function surfaceCards(){const d=dict(state.decision),t=dict(state.tactical),out=dict(state.control_output),feedback=dict(state.gimbal_feedback);if(view.surface==='units'){const items=state.scene.palette||[];return card('Unit palette',paletteChoice?text(paletteChoice.type):'Choose a piece','<div class="roster" id="surfacePalette">'+items.map((u,n)=>'<button data-palette="'+n+'">'+(u.asset_url?'<img src="'+u.asset_url+'">':'<span class="mark">'+text(u.type).slice(0,1)+'</span>')+'<span>'+text(u.type)+'</span></button>').join('')+'</div>',true)+card('Placed units',text(state.scene.units.length),'<div class="roster">'+state.scene.units.map(u=>'<button type="button" data-unit-select="'+u.entity_id+'"><span>'+text(u.type)+'</span><strong>'+text(u.hp)+' / '+text(u.max_hp)+'</strong></button>').join('')+'</div>',true);}if(view.surface==='decision')return card('Decision',text(d.strategy),kv([['Goal',text(dict(d.goal).name)],['Layer',text(dict(d.intent).layer)],['Reason',text(dict(d.intent).reason)],['Priority',text(dict(d.intent).priority)]]),true);if(view.surface==='events')return card('Events',text(t.available?'Tactical evidence':'No tactical trace'),kv([['Protect Castle',text(dict(t.protect_castle).enabled)],['Protect Outpost',text(dict(t.protect_outpost).active)],['Protect Hero',text(dict(t.protect_hero).active)],['Defense',text(dict(t.regional_defense).threat_active)]]),true);if(view.surface==='control')return card('Final control',text(out.source),kv([['Sequence',text(out.sequence)],['Angles',text(dict(out.angles).published?'Published':'Not published')],['FireCode',text(dict(out.fire_code).published?'Published':'Not published')],['Feedback age',text(feedback.age_ms)+' ms']]),true);if(view.surface==='layers')return card('Map layers','Current field presentation',kv([['Map',text(state.stream.map_url)],['Pieces',text(state.scene.units.length)],['Structures',text(state.scene.structures.length)],['Route',text((d.route_cm||[]).length)+' points']]),true);if(view.surface==='runtime')return card('Runtime diagnostics','Debug mode',kv([['Owner',text(state.scene.ownership_mode)],['Ready',text(state.ready)],['Control',text(out.available)],['Feedback',text(feedback.available)]]),true);return overviewCards()}
''',
        page,
        flags=re.DOTALL,
    )
    compatibility = r'''<script>
const resetViewButton=document.getElementById('resetView');if(resetViewButton)resetViewButton.onclick=fit;
function renderPersistentRoster(){if(!state)return;const palette=document.getElementById('palette');if(!palette)return;palette.innerHTML=(state.scene.palette||[]).map((unit,index)=>'<button type="button" data-unit-index="'+index+'">'+(unit.asset_url?'<img src="'+unit.asset_url+'" alt="">':'<span class="mark">'+text(unit.type).slice(0,1)+'</span>')+'<span>'+text(unit.type)+'</span></button>').join('');for(const button of palette.querySelectorAll('button'))button.onclick=()=>{paletteChoice=state.scene.palette[Number(button.dataset.unitIndex)];document.getElementById('tactical').textContent='Placement armed: '+text(paletteChoice.type)};}
function finishPieceDrag(event){if(!drag||(event.pointerId!==undefined&&drag.id!==event.pointerId))return;const done=drag;drag=null;event.stopPropagation();if(done.moved){const point=done.lastPoint||readPoint(event);command({command:'place_unit',entity_id:done.unit.entity_id,side:done.unit.side,unit_key:done.unit.unit_key,hp:done.unit.hp,x:point.x,y:point.y})}else select('unit',done.unit.entity_id)}
function preserveLegacyMapSelectors(){document.querySelectorAll('.structure').forEach((item,index)=>{const structure=state.scene.structures[index]||{};item.classList.add('map-structure');item.dataset.structureKey=structure.key||'';item.onclick=event=>{event.stopPropagation();if(drag)finishPieceDrag(event);select('structure',structure.key)};});document.querySelectorAll('.piece').forEach((item,index)=>{const unit=state.scene.units[index]||{};item.dataset.entityId=unit.entity_id||'';item.dataset.step='drag';item.onpointerdown=event=>{if(!state.scene.can_edit)return;drag={id:event.pointerId,unit,startX:event.clientX,startY:event.clientY,moved:false,lastPoint:readPoint(event)};item.setPointerCapture(event.pointerId);event.stopPropagation()};item.onpointerup=finishPieceDrag;item.onmouseup=finishPieceDrag;item.onclick=event=>{event.stopPropagation();if(drag)finishPieceDrag(event);else select('unit',unit.entity_id)};});}
window.addEventListener('pointermove',event=>{if(drag&&drag.id===event.pointerId&&event.target.closest('.piece')){drag.moved=drag.moved||Math.hypot(event.clientX-drag.startX,event.clientY-drag.startY)>4;drag.lastPoint=readPoint(event)}});window.addEventListener('mousemove',event=>{if(drag&&event.target.closest('.piece')){drag.moved=drag.moved||Math.hypot(event.clientX-drag.startX,event.clientY-drag.startY)>4;drag.lastPoint=readPoint(event)}});window.addEventListener('pointerup',finishPieceDrag);window.addEventListener('mouseup',finishPieceDrag);
viewport.addEventListener('pointerup',event=>{if(!drag)return;const structure=event.target.closest('.structure');event.stopImmediatePropagation();finishPieceDrag(event);if(structure)select('structure',structure.dataset.structureKey)},true);
setInterval(()=>{renderPersistentRoster();preserveLegacyMapSelectors()},120);
</script>'''
    # The current workspace owns piece drag handlers.  Only keep the palette
    # adapter required by the retained roster card; do not override pointer
    # handlers with the legacy compatibility layer.
    compatibility = r'''<script>
function renderPersistentRoster(){if(!state)return;const palette=document.getElementById('palette');if(!palette)return;palette.innerHTML=(state.scene.palette||[]).map((unit,index)=>'<button type="button" data-unit-index="'+index+'">'+(unit.asset_url?'<img src="'+unit.asset_url+'" alt="">':'<span class="mark">'+text(unit.type).slice(0,1)+'</span>')+'<span>'+text(unit.type)+'</span></button>').join('');for(const button of palette.querySelectorAll('button'))button.onclick=()=>{paletteChoice=state.scene.palette[Number(button.dataset.unitIndex)];document.getElementById('tactical').textContent='Placement armed: '+text(paletteChoice.type)};}
const workspaceRenderMap=renderMap;
renderMap=()=>{workspaceRenderMap();pieces.querySelectorAll('.piece img').forEach(image=>{image.draggable=false});for(const element of pieces.querySelectorAll('.piece')){const unit=(state.scene.units||[]).find(item=>item.entity_id===element.dataset.entityId),fieldSide=unit&&unit.field_side;element.classList.toggle('friend',unit&&unit.side==='friend');element.classList.toggle('enemy',unit&&unit.side==='enemy');element.classList.toggle('field-red',fieldSide==='red');element.classList.toggle('field-blue',fieldSide==='blue')}for(const element of structures.querySelectorAll('.structure')){const structure=(state.scene.structures||[]).find(item=>item.key===element.dataset.structureKey),fieldSide=structure&&structure.field_side;element.classList.toggle('field-red',fieldSide==='red');element.classList.toggle('field-blue',fieldSide==='blue')}};
document.addEventListener('dragstart',event=>{if(event.target.closest('.piece img'))event.preventDefault()});
setInterval(renderPersistentRoster,120);
</script>'''
    interaction = r'''<script>
const selectionBox=document.createElement('div');selectionBox.id='selectionBox';selectionBox.hidden=true;Object.assign(selectionBox.style,{position:'absolute',zIndex:'20',border:'1px solid #4DB7FF',background:'#4DB7FF22',pointerEvents:'none'});viewport.append(selectionBox);let boxStart=null;
function clearContextMenu(){document.getElementById('battleContextMenu')?.remove()}
function moveActivePiece(event){if(!drag||drag.id!==event.pointerId)return false;const point=readPoint(event);drag.lastPoint=point;drag.moved=drag.moved||Math.hypot(event.clientX-drag.startX,event.clientY-drag.startY)>4;if(drag.moved&&drag.element){drag.element.classList.add('dragging');position(drag.element,point)}return true}
function finishActivePieceDrag(event){if(!drag||drag.id!==event.pointerId)return false;const done=drag;drag=null;if(done.element)done.element.classList.remove('dragging');if(done.moved){const point=done.lastPoint||readPoint(event);command({command:'place_unit',entity_id:done.unit.entity_id,side:done.unit.side,unit_key:done.unit.unit_key,hp:done.unit.hp,x:point.x,y:point.y})}else{select('unit',done.unit.entity_id)}return true}
window.addEventListener('pointermove',moveActivePiece,true);
window.addEventListener('pointerup',finishActivePieceDrag,true);
window.addEventListener('pointercancel',event=>{if(drag&&drag.id===event.pointerId){if(drag.element)drag.element.classList.remove('dragging');drag=null}},true);
viewport.addEventListener('pointerdown',event=>{if(event.button!==0||paletteChoice||event.target.closest('.piece,.structure,.area'))return;boxStart={id:event.pointerId,x:event.clientX,y:event.clientY};selectionBox.hidden=false;event.stopImmediatePropagation()},{capture:true});
viewport.addEventListener('pointermove',event=>{if(!boxStart||boxStart.id!==event.pointerId)return;const left=Math.min(boxStart.x,event.clientX),top=Math.min(boxStart.y,event.clientY),width=Math.abs(event.clientX-boxStart.x),height=Math.abs(event.clientY-boxStart.y),r=viewport.getBoundingClientRect();Object.assign(selectionBox.style,{left:(left-r.left)+'px',top:(top-r.top)+'px',width:width+'px',height:height+'px'});},{capture:true});
viewport.addEventListener('pointerup',event=>{if(!boxStart||boxStart.id!==event.pointerId)return;const start=readPoint({clientX:boxStart.x,clientY:boxStart.y}),end=readPoint(event),minX=Math.min(start.x,end.x),maxX=Math.max(start.x,end.x),minY=Math.min(start.y,end.y),maxY=Math.max(start.y,end.y);for(const el of pieces.querySelectorAll('.piece')){const unit=(state.scene.units||[]).find(item=>item.entity_id===el.dataset.entityId),p=unit&&unit.position_cm;el.classList.toggle('selected',Boolean(p&&p.x>=minX&&p.x<=maxX&&p.y>=minY&&p.y<=maxY))}selectionBox.hidden=true;boxStart=null;event.stopImmediatePropagation()},{capture:true});
viewport.addEventListener('contextmenu',event=>{const piece=event.target.closest('.piece'),structure=event.target.closest('.structure');if(!piece&&!structure)return;event.preventDefault();clearContextMenu();const target=piece?(state.scene.units||[]).find(item=>item.entity_id===piece.dataset.entityId):(state.scene.structures||[]).find(item=>item.key===structure.dataset.structureKey);if(!target)return;const menu=document.createElement('div');menu.id='battleContextMenu';Object.assign(menu.style,{position:'fixed',zIndex:'100',left:event.clientX+'px',top:event.clientY+'px',minWidth:'150px',padding:'6px',background:'#202832',border:'1px solid rgba(255,255,255,.12)',borderRadius:'8px',boxShadow:'0 12px 30px #0008'});const focus=document.createElement('button');focus.textContent='Focus';focus.onclick=()=>{select(piece?'unit':'structure',piece?target.entity_id:target.key);centerOn(target.position_cm,Math.max(view.zoom,1.7));clearContextMenu()};menu.append(focus);if(piece&&state.scene.can_edit){const remove=document.createElement('button');remove.textContent='Remove';remove.onclick=()=>{command({command:'remove_unit',entity_id:target.entity_id});clearContextMenu()};menu.append(remove)}for(const button of menu.querySelectorAll('button'))Object.assign(button.style,{display:'block',width:'100%',height:'34px',border:0,background:'transparent',color:'#F5F7FA',textAlign:'left',cursor:'pointer'});document.body.append(menu)},{capture:true});
document.addEventListener('pointerdown',event=>{if(!event.target.closest('#battleContextMenu'))clearContextMenu()});
</script>'''
    clock_ui = r'''<script>
function matchClockText(seconds){const whole=Math.max(0,Math.round(Number(seconds)||0));return String(Math.floor(whole/60)).padStart(2,'0')+':'+String(whole%60).padStart(2,'0')}
function matchClockSeconds(value,limit){const match=String(value).trim().match(/^(\d{1,2}):(\d{2})$/);if(!match||Number(match[2])>=60)return null;return clamp(Number(match[1])*60+Number(match[2]),0,limit)}
function renderMatchClock(){if(!state)return;const match=state.match||{},limit=Math.max(1,Number(match.duration_sec)||420),time=clamp(Number(match.time_left)||0,0,limit),readonly=!state.scene.can_edit;const clock=document.getElementById('matchClock'),timeline=document.getElementById('matchTimelineRange'),input=document.getElementById('matchTimeInput'),stateLabel=document.getElementById('matchState');clock.textContent=matchClockText(time);clock.dataset.running=String(Boolean(match.running));timeline.max=String(limit);timeline.value=String(Math.round(time));if(document.activeElement!==input)input.value=matchClockText(time);stateLabel.textContent=match.running?'RUNNING':'PAUSED';stateLabel.className='pill '+(match.running?'good':'warn');for(const button of document.querySelectorAll('[data-match-command]'))button.disabled=readonly;timeline.disabled=readonly;input.disabled=readonly}
function sendMatchTime(commandName,seconds){command({command:commandName,seconds})}
for(const button of document.querySelectorAll('[data-match-command]'))button.onclick=()=>sendMatchTime(button.dataset.matchCommand,Number(button.dataset.seconds)||0);
const matchTimeline=document.getElementById('matchTimelineRange'),matchTimeInput=document.getElementById('matchTimeInput');matchTimeline.onchange=()=>sendMatchTime('set_time_left',Number(matchTimeline.value));matchTimeInput.onchange=()=>{const seconds=matchClockSeconds(matchTimeInput.value,Math.max(1,Number(state.match&&state.match.duration_sec)||420));if(seconds===null){matchTimeInput.value=matchClockText(state.match&&state.match.time_left);announce('Use mm:ss');return}sendMatchTime('set_time_left',seconds)};matchTimeInput.onkeydown=event=>{if(event.key==='Enter'){event.preventDefault();matchTimeInput.onchange()}};
	function renderTeamPill(){const pill=document.getElementById('teamPill');if(!pill||!state)return;const team=state.scene&&state.scene.team==='blue'?'BLUE':'RED';pill.textContent='MY TEAM · '+team;pill.className='pill team-'+team.toLowerCase()}
	const workspaceRender=render;render=()=>{if(state&&state.scene)view.side=state.scene.team==='blue'?'blue':'red';workspaceRender();renderMatchClock();renderTeamPill()};
	</script>'''
    target_visibility_ui = r'''<script>
(()=>{
const style=document.createElement('style');
style.textContent='.area.red:after{background:#e85d5d}.area.blue:after{background:#5d8ee8}.area.red{border-color:#e85d5d}.area.blue{border-color:#5d8ee8}';
document.head.append(style);
const baseRenderMap=renderMap;
renderMap=function(){
  baseRenderMap();
  const started=Boolean(state&&state.match&&state.match.started);
  const activeTeam=state&&state.scene&&state.scene.team==='blue'?'blue':'red';
  const nodes=[...document.querySelectorAll('#areaLayer .area')];
  nodes.forEach((node,index)=>{
    const item=(MAP_OBJECTS||[])[index]||{};
    const show=!started&&item.field_side===activeTeam;
    node.classList.toggle('red',item.field_side==='red');
    node.classList.toggle('blue',item.field_side==='blue');
    node.hidden=!show;node.style.display=show?'':'none';
  });
  const marker=document.getElementById('goalMarker');
  const label=document.getElementById('goalLabel');
  const goal=state&&state.decision?state.decision.goal:{};
  const hasGoal=Array.isArray(goal&&goal.pos_cm)&&goal.pos_cm.length>=2;
  if(!started||!hasGoal){marker.hidden=true;label.hidden=true}
};
if(state)renderMap();
})();
</script>'''
    localization_ui = r"""<script>
(()=>{
const exact={
"Sentinel Flight Deck":"哨兵战术台",
"Regional decision rehearsal":"哨兵决策离线演练",
"MY TEAM · RED":"我方 · 红方",
"MY TEAM · BLUE":"我方 · 蓝方",
"BATTLEFIELD":"战场",
"OVERVIEW":"总览",
"Red":"红方",
"Blue":"蓝方",
"Waiting for decision trace":"等待决策 trace",
"Operations shelf":"操作栏",
"Timeline and runtime are available here.":"时间轴与运行时信息显示在这里。",
"Tactical evidence is shown by the selected workspace surface.":"战术证据显示在所选工作区面板中。",
"Decision & navigation":"决策与导航",
"Robot roster":"机器人列表",
"Scene placement":"场景摆放",
"Tactical state":"战术状态",
"Current trace":"当前 trace",
"Official RoboMaster field map":"官方 RoboMaster 场地地图",
"Field perspective":"场地视角",
"Workspace surfaces":"工作区面板",
"Inspector":"检查器",
"Close Inspector":"关闭检查器",
"Show Inspector":"显示检查器",
"Move Inspector":"移动检查器",
"Reset view":"重置视图",
"Fit to window":"适应窗口",
"Actual size":"实际大小",
"Zoom in":"放大",
"Zoom out":"缩小",
"Debug mode":"调试模式",
"Focus mode":"专注模式",
"Reset layout":"重置布局",
"Match time":"比赛时间",
"Match time remaining":"比赛剩余时间",
"Match time in minutes and seconds":"比赛时间（分:秒）",
"Current field presentation":"当前场地图",
"Buff area":"增益区",
"High ground":"高地",
"Spawn area":"启动区",
"Supply area":"补给区",
"Control endpoint unavailable":"控制接口不可用",
"Command applied":"命令已应用",
"Command rejected":"命令被拒绝",
"Scene editing is unavailable.":"场景编辑不可用。",
"Scene input controls are disabled.":"场景输入控件已禁用。",
"Scene command file is unavailable.":"场景命令文件不可用。",
"Replay viewer is read-only. Start offline decision mode to edit this board.":"回放查看器为只读。请启动离线决策模式后再编辑场地。",
"Manual ROS observer mode: Foxglove or ROS CLI owns scene facts.":"手动 ROS 观察模式：由 Foxglove 或 ROS CLI 提供场景事实。",
"Use mm:ss":"请输入 mm:ss",
"Tactical state unavailable":"战术状态不可用",
"Control":"控制",
"Overview":"总览",
"Decision":"决策",
"Events":"事件",
"Runtime":"运行时",
"Layers":"图层",
"Units":"单位",
"Inputs":"输入",
"Goal":"目标",
"Layer":"层级",
"Reason":"原因",
"Position":"位置",
"Strategy":"策略",
"Aim":"瞄准",
"Route":"路径",
"Output":"输出",
"Team":"队伍",
"Coordinate":"坐标",
"State":"状态",
"Health":"血量",
"Alive":"存活",
"Destroyed":"已摧毁",
"Available":"可用",
"Unavailable":"不可用",
"Ready":"就绪",
"None":"无",
"Published":"已发布",
"Not published":"未发布",
"Not recorded":"未记录",
"Yes":"是",
"No":"否",
"Start":"开始",
"Pause":"暂停",
"Reset":"重置",
"Focus":"聚焦",
"Debug":"调试",
"Fullscreen":"全屏",
"Tick ":"帧 ",
"mock":"模拟",
"goal ":"目标 ",
"goal -":"目标 -",
"control ":"控制 ",
"control -":"控制 -",
"LEAGUE":"联盟赛",
"REGIONAL":"区域赛",
"BASE":"基地",
"OUTPOST":"前哨站",
"ROBOT":"机器人",
"STRUCTURE":"建筑",
"AREA":"区域",
"RUNNING":"运行中",
"PAUSED":"已暂停",
"+10":"+10秒",
"-10":"-10秒",
"+30":"+30秒",
"-30":"-30秒",
"+10s":"+10秒",
"-10s":"-10秒",
"+30s":"+30秒",
"-30s":"-30秒"
};
const tokens=[
["MY TEAM","我方"],
["goal ","目标 "],
["control ","控制 "],
["Tick ","帧 "],
["tactical","战术"],
["Trace","轨迹"],
["trace","轨迹"],
["LeagueSimple","联盟赛3V3"],
["Regional","区域赛"],
["center_anchor","中央控制区锚点"],
["center_own_side","己方控制区前沿"],
["center_top","控制区上侧巡逻点"],
["center_enemy_side","敌方控制区前沿"],
["enemy_start_approach","敌方基地外围"],
["self_base","己方基地"],
["league_3v3","联盟赛3V3"],
["center_patrol","中心巡逻"],
["hold_and_patrol_center","保持并巡逻中心"],
["contest_center","争夺中心"],
["contest_enemy_center","争夺敌方中心"],
["defend_center","防守中心"],
["center_contested","中心争夺"],
["center_unoccupied","中心未占领"],
["center_status_stale","中心状态过期"],
["recover","回血"],
["go_center","前往中心"],
["hold_center","驻守中心"],
["cover_weak_ally","掩护残血队友"],
["retreat_behind_ally","退至队友身后"],
["chase_enemy_hero","追击敌方英雄"],
["RotateScan","旋转扫描"],
["AutoAim","自动瞄准"],
["Hero","英雄"],
["Engineer","工程"],
["Infantry1","步兵"],
["Infantry2","步兵"],
["Infantry3","步兵"],
["Sentry","哨兵"],
["Drone","空中"],
[" cm"," 厘米"],
[" HP"," 血量"],
[" ms"," 毫秒"],
[" s"," 秒"]
];
function translateText(value){
 let out=String(value);
 if(Object.prototype.hasOwnProperty.call(exact,out)) return exact[out];
 for(const [en,zh] of tokens){if(en&&out.includes(en)) out=out.split(en).join(zh)}
 return out;
}
function translateNode(node){
 if(!node) return;
 if(node.nodeType===3){
  const next=translateText(node.nodeValue||"");
  if(next!==node.nodeValue) node.nodeValue=next;
  return;
 }
 if(node.nodeType!==1) return;
 for(const attr of ["title","aria-label","placeholder","alt"]){
  if(node.hasAttribute&&node.hasAttribute(attr)){
   const value=node.getAttribute(attr),next=translateText(value);
   if(next!==value) node.setAttribute(attr,next);
  }
 }
 for(const child of node.childNodes) translateNode(child);
}
function localizePage(){
 document.documentElement.lang="zh-CN";
 document.title=translateText(document.title);
 translateNode(document.body);
}
function observe(){
 const zhStyle=document.createElement("style");zhStyle.textContent="#goalSummary{margin-right:10px}button[data-seconds]{min-width:58px;white-space:nowrap}";document.head.append(zhStyle);
 localizePage();
 new MutationObserver(localizePage).observe(document.body,{subtree:true,childList:true,characterData:true,attributes:true,attributeFilter:["title","aria-label","placeholder","alt"]});
}
if(document.readyState==="loading") document.addEventListener("DOMContentLoaded",observe,{once:true}); else observe();
})();
</script>"""

    return (
        page.replace("__MAP_OBJECTS__", json.dumps(map_objects if map_objects is not None else _map_objects(), separators=(",", ":")))
        # `side` is friend/enemy relative to the selected sentry.  Border color is
        # deliberately reserved for physical red/blue field teams below.
        .replace(".piece.enemy .ring{border-color:#bd7074}", "")
        .replace(
            "</style>",
            "#pieceLayer{pointer-events:none}.piece{pointer-events:auto}"
            ".piece img{pointer-events:none;user-select:none;-webkit-user-drag:none}"
            ".team-red{color:#efc4c6;border-color:#855258;background:#2a1f22}.team-blue{color:#c9e7ff;border-color:#4f7695;background:#162936}"
            ".piece.field-red .ring,.structure.field-red{border-color:#bd7074}.piece.field-blue .ring,.structure.field-blue{border-color:#6d9ccb}"
            "#workspaceShell.focus{grid-template-columns:0 minmax(0,1fr) var(--inspector)}"
            "#workspaceShell.focus[data-inspector-dock=left]{grid-template-columns:var(--inspector) 0 minmax(0,1fr)}"
            ".match-clock{min-width:78px;font:700 20px/1 ui-monospace,SFMono-Regular,Consolas,monospace;letter-spacing:0;color:var(--fd-text);font-variant-numeric:tabular-nums}.match-clock[data-running=true]{color:#d9efff}.match-timeline{display:grid;grid-template-columns:auto minmax(120px,1fr) 74px auto;align-items:center;gap:12px;padding:12px;border:1px solid var(--line);border-radius:8px;background:var(--surface)}.match-timeline label{color:var(--muted)}.match-timeline input[type=range]{width:100%;accent-color:var(--accent)}.match-timeline input[type=text]{height:40px;width:74px;padding:0 8px;border:1px solid var(--line);border-radius:6px;background:#11181e;text-align:center;font-variant-numeric:tabular-nums}.match-adjust{display:flex;gap:8px}.match-adjust button{height:40px;min-width:52px;padding:0 10px;border:1px solid var(--line);border-radius:6px;background:var(--surface-2)}.match-adjust button:hover{border-color:var(--accent);background:var(--accent-soft)}"
            "@media(max-width:820px){#workspaceShell,#workspaceShell[data-inspector-dock=left]{grid-template-rows:auto auto auto}#battlefieldWorkspace{min-height:0}#mapViewport{flex:none;aspect-ratio:28/15}}"
            "@media(max-width:620px){.match-timeline{grid-template-columns:1fr 74px}.match-timeline label,.match-timeline .match-adjust{grid-column:1/-1}.match-timeline .match-adjust{display:grid;grid-template-columns:repeat(4,minmax(0,1fr))}}"
            "@media(max-width:560px){#commandBar .commands{flex:0 0 100%;width:100%}}"
            "</style>",
        )
        .replace(
            '<div class="group" id="matchActions"><button class="button primary" data-command="start">Start</button><button class="button" data-command="pause">Pause</button><button class="button" data-command="reset">Reset</button></div>',
            '<div class="group" id="matchActions"><span id="matchClock" class="match-clock" data-running="false">07:00</span><button class="button primary" data-command="start">Start</button><button class="button" data-command="pause">Pause</button><button id="rewind10" class="icon-button" type="button" data-match-command="rewind" data-seconds="10" title="Add 10 seconds" aria-label="Add 10 seconds">+10</button><button id="forward10" class="icon-button" type="button" data-match-command="forward" data-seconds="10" title="Subtract 10 seconds" aria-label="Subtract 10 seconds">-10</button><button class="button" data-command="reset">Reset</button></div>',
        )
        .replace(
            '<span class="pill" id="ownerPill">mock</span>',
            '<span class="pill" id="teamPill">MY TEAM · RED</span><span class="pill" id="ownerPill">mock</span>',
        )
        .replace('aria-label="Field perspective"', 'aria-label="My simulated team"')
        .replace(
            '<div class="shelf-data" id="shelfData"></div>',
            '<div id="matchTimeline" class="match-timeline"><label for="matchTimeInput">Match time</label><input id="matchTimelineRange" type="range" min="0" max="420" value="420" aria-label="Match time remaining"><input id="matchTimeInput" type="text" inputmode="numeric" value="07:00" aria-label="Match time in minutes and seconds"><span id="matchState" class="pill warn">PAUSED</span><div class="match-adjust"><button id="rewind30" type="button" data-match-command="rewind" data-seconds="30" title="Add 30 seconds">+30s</button><button type="button" data-match-command="rewind" data-seconds="10" title="Add 10 seconds">+10s</button><button type="button" data-match-command="forward" data-seconds="10" title="Subtract 10 seconds">-10s</button><button id="forward30" type="button" data-match-command="forward" data-seconds="30" title="Subtract 30 seconds">-30s</button></div></div><div class="shelf-data" id="shelfData"></div>',
        )
        .replace(
            "const p=readPoint(event);command({command:'place_unit',entity_id:done.unit.entity_id",
            "const p=done.lastPoint||readPoint(event);command({command:'place_unit',entity_id:done.unit.entity_id",
        )
        .replace("state=next;render()", "state=next;if(!drag)render()")
        .replace(
            "root.style.setProperty('--inspector',view.inspectorWidth+'px');root.style.setProperty('--shelf',view.shelfHeight+'px');",
            "root.style.setProperty('--inspector',(view.inspectorCollapsed?48:view.inspectorWidth)+'px');root.style.setProperty('--shelf',(view.shelfOpen?view.shelfHeight:40)+'px');",
        )
        .replace('title="Fit to window">⌗</button>', 'title="Fit to window">⌗</button><button class="sr" id="resetView" type="button">Reset view</button>')
        .replace('id="shelfToggle" title="Operations shelf">▰</button>', 'id="shelfToggle" aria-expanded="true" title="Operations shelf">▰</button>')
        .replace(
            '<div id="inspectorCards"></div></aside>',
            '<div id="inspectorCards"></div><span id="overviewInspector" hidden></span></aside>',
        )
        .replace(
            '<button class="icon-button" id="undoButton" disabled title="Undo">↶</button>',
            '<button class="icon-button" id="undoButton" disabled title="Undo">↶</button><button class="icon-button" id="inspectorClose" title="Close Inspector" aria-label="Close Inspector">×</button>',
        )
        .replace(
            '<button class="rail-button" id="dockToggle" title="Move Inspector">⇄</button>',
            '<button class="rail-button" id="inspectorToggle" title="Show Inspector" aria-label="Show Inspector">▤</button><button class="rail-button" id="dockToggle" title="Move Inspector">⇄</button>',
        )
        .replace(
            "</style>",
            "#workspaceShell.inspector-collapsed #inspector{padding:0;overflow:hidden}#workspaceShell.inspector-collapsed #inspector>*{visibility:hidden}</style>",
            1,
        )
        .replace(
            "$('dockToggle').onclick=()=>{view.inspectorDock=view.inspectorDock==='right'?'left':'right';applyView()};",
            "$('inspectorClose').onclick=()=>{view.inspectorCollapsed=true;applyView()};$('inspectorToggle').onclick=()=>{view.inspectorCollapsed=false;applyView()};$('dockToggle').onclick=()=>{view.inspectorDock=view.inspectorDock==='right'?'left':'right';applyView()};",
        )
        .replace(
            "$('viewSideRed').onclick=()=>{view.side='red';render()};$('viewSideBlue').onclick=()=>{view.side='blue';render()};",
            "$('viewSideRed').onclick=()=>{inspectorNeedsRender=true;command({command:'set_team',team:'red'})};$('viewSideBlue').onclick=()=>{inspectorNeedsRender=true;command({command:'set_team',team:'blue'})};",
        )
        .replace(
            "position(e,u.position_cm);e.onpointerdown=event=>{if(!state.scene.can_edit)return;drag={id:event.pointerId,unit:u,startX:event.clientX,startY:event.clientY,moved:false};viewport.setPointerCapture(event.pointerId);event.stopPropagation()};",
            "position(e,u.position_cm);e.dataset.entityId=u.entity_id;e.onpointerdown=event=>{if(!state.scene.can_edit)return;drag={id:event.pointerId,unit:u,element:e,startX:event.clientX,startY:event.clientY,moved:false,lastPoint:readPoint(event)};viewport.setPointerCapture(event.pointerId);event.stopPropagation()};",
        )
        .replace(
            "position(e,s.position_cm);e.onclick=event=>{event.stopPropagation();select('structure',s.key)};",
            "position(e,s.position_cm);e.dataset.structureKey=s.key;e.onclick=event=>{event.stopPropagation();select('structure',s.key)};",
        )
        .replace(
            "if(drag&&drag.id===event.pointerId){drag.moved=drag.moved||Math.hypot(event.clientX-drag.startX,event.clientY-drag.startY)>4}});",
            "if(drag&&drag.id===event.pointerId){drag.lastPoint=p;drag.moved=drag.moved||Math.hypot(event.clientX-drag.startX,event.clientY-drag.startY)>4;if(drag.moved){drag.element.classList.add('dragging');position(drag.element,p)}}});",
        )
        .replace(
            "if(drag&&drag.id===event.pointerId){const done=drag;drag=null;if(done.moved)",
            "if(drag&&drag.id===event.pointerId){const done=drag;done.element.classList.remove('dragging');drag=null;if(done.moved)",
        )
        .replace("</script></body>", "</script>" + compatibility + interaction + clock_ui + localization_ui + target_visibility_ui + "</body>")
        .replace(
            "function hpEditor(item,kind){",
            "function hpMeter(hp,max){return '<div class=\"meter\" id=\"hpMeter\"><i style=\"width:'+(100*clamp(Number(hp)/Math.max(1,Number(max)),0,1))+'%\"></i></div>'}function updateLiveHealthDisplay(){if(!state)return;const item=selected();if(!item)return;const hp=Number(item.hp||0),max=Math.max(1,Number(item.max_hp||hp||1)),summary=document.getElementById('hpCurrentSummary'),fill=document.querySelector('#hpMeter i');if(summary)summary.textContent=hp+' / '+max;if(fill)fill.style.width=(100*clamp(hp/max,0,1))+'%'}function hpEditor(item,kind){",
        )
        .replace(
            "<div class=\"meter\"><i style=\"width:'+(100*hp/max)+'%\"></i></div><div class=\"hp-editor\">",
            "<div class=\"hp-editor\">",
        )
        .replace(
            "+card('Health',text(u.hp)+' / '+text(u.max_hp),hpEditor(u,'unit'))+card('Decision'",
            "+card('当前血量','<span id=\"hpCurrentSummary\">'+text(u.hp)+' / '+text(u.max_hp)+'</span>',hpMeter(u.hp,u.max_hp),true)+card('血量设置','手动设置',hpEditor(u,'unit'))+card('Decision'",
        )
        .replace(
            "+card('Health',text(s.hp)+' / '+text(s.max_hp),hpEditor(s,'structure'))+card('Decision impact'",
            "+card('当前血量','<span id=\"hpCurrentSummary\">'+text(s.hp)+' / '+text(s.max_hp)+'</span>',hpMeter(s.hp,s.max_hp),true)+card('血量设置','手动设置',hpEditor(s,'structure'))+card('Decision impact'",
        )
        .replace(
            "renderMap();if(forceInspector",
            "renderMap();updateLiveHealthDisplay();if(forceInspector",
        )
        .encode("utf-8")
    )


def build_tactical_html(port: int, *, map_objects: list[dict[str, Any]] | None = None) -> bytes:
    """Return the dependency-free, map-first tactical command desk."""

    del port
    return _build_workspace_html(map_objects)

    del port
    body = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>LY Sentinel Tactical Simulator</title>
<style>
:root{--bg:#11161b;--chrome:#181f26;--surface:#202933;--surface-strong:#27323d;--line:#3a4854;--text:#edf2f7;--muted:#9aa9b7;--accent:#62a9e8;--accent-soft:#1d3850;--orange:#d8985a;--danger:#c96c6c;--red-team:#bd676c;--blue-team:#6599cc;--inspector-width:360px}*{box-sizing:border-box}html,body{min-width:320px;min-height:100%}body{margin:0;overflow:hidden;background:var(--bg);color:var(--text);font:14px/1.45 Inter,"Noto Sans",Arial,sans-serif}button,input{font:inherit;color:inherit}button{border:0}button:focus-visible,input:focus-visible{outline:2px solid var(--accent);outline-offset:2px}#topbar{display:flex;align-items:center;gap:24px;min-height:72px;padding:14px 24px;border-bottom:1px solid var(--line);background:var(--chrome)}.brand{display:flex;align-items:center;gap:12px;min-width:226px}.brand-mark{display:grid;place-items:center;width:38px;height:38px;border:1px solid var(--accent);border-radius:8px;background:#182c3d;color:#dceefd;font-weight:800}.brand-copy h1{margin:0;font-size:16px}.brand-copy p{margin:2px 0 0;color:var(--muted);font-size:14px}.commandbar{display:flex;flex:1;align-items:center;justify-content:flex-end;gap:16px;min-width:0}.command-group,.status-strip,.segmented,.inspector-actions,.hp-actions{display:flex;align-items:center;gap:8px}.command-group{padding-left:16px;border-left:1px solid var(--line)}.tool,.command,.segmented button,.hp-actions button,.undo-button{min-height:40px;border:1px solid var(--line);border-radius:6px;background:var(--surface);color:var(--text);cursor:pointer;transition:background .16s ease,border-color .16s ease,transform .16s ease}.tool{min-width:40px;padding:0 10px}.tool.label,.command{padding:0 13px}.tool:hover,.command:hover,.segmented button:hover,.hp-actions button:hover,.undo-button:hover{border-color:var(--accent);background:var(--surface-strong)}.tool:active,.command:active,.segmented button:active,.hp-actions button:active,.undo-button:active{transform:translateY(1px)}.tool[aria-pressed=true],.segmented button[aria-pressed=true]{border-color:var(--accent);background:var(--accent-soft);color:#d9edff}.command.primary{border-color:var(--accent);background:#204a6d;color:#e5f3ff}.command.danger,.hp-actions .danger{color:#ffc9c9;border-color:#834c50}.segmented{overflow:hidden;gap:0;border:1px solid var(--line);border-radius:6px}.segmented button{min-width:58px;border:0;border-radius:0}.segmented button+button{border-left:1px solid var(--line)}.pill{padding:5px 9px;border:1px solid var(--line);border-radius:999px;color:var(--muted);font-size:14px;white-space:nowrap}.pill.good{border-color:#5682a8;color:#b9d9f3}.pill.warn{border-color:#8f6849;color:#efc18d}.shell{display:grid;grid-template-columns:minmax(0,1fr) var(--inspector-width);height:calc(100vh - 72px);min-height:0}body.dock-left .shell{grid-template-columns:var(--inspector-width) minmax(0,1fr)}.workspace{display:flex;flex-direction:column;min-width:0;min-height:0;padding:24px;background:#12181d}.board-toolbar{display:flex;align-items:center;justify-content:space-between;gap:16px;min-height:32px;margin-bottom:16px;color:var(--muted)}.board-toolbar strong{color:var(--text);font-size:15px;font-weight:650}#mapViewport{position:relative;flex:1;min-height:0;overflow:hidden;border:1px solid var(--line);border-radius:10px;background:#1b242b;box-shadow:0 18px 44px #05080b66;touch-action:none}#mapCanvas{position:absolute;top:50%;left:50%;width:min(100%,calc((100vh - 160px)*28/15));aspect-ratio:28/15;transform-origin:center;transition:transform .16s ease}#fieldBoard{position:absolute;inset:0;overflow:hidden;background:#2a353d}#fieldImage{position:absolute;inset:0;width:100%;height:100%;user-select:none;pointer-events:none}body[data-view-side=blue] #fieldImage{transform:rotate(180deg)}#routeOverlay,#areaLayer,#structureLayer,#pieceLayer{position:absolute;inset:0}#routeOverlay{pointer-events:none;overflow:visible}#areaLayer{z-index:1}#structureLayer{z-index:3}#pieceLayer{z-index:4}.map-object,.map-structure{position:absolute;transform:translate(-50%,-50%);background:transparent;cursor:pointer}.map-object{width:26px;height:26px;border:1px solid #b9c7d2;border-radius:50%;background:#18242dcb;font-size:0}.map-object:after{content:"";position:absolute;inset:7px;border-radius:50%;background:var(--accent)}.map-object:hover,.map-object.selected{border-color:var(--accent);box-shadow:0 0 0 4px #62a9e838}.map-structure{display:grid;min-width:52px;min-height:42px;place-items:center;padding:4px 8px;border:1px solid #8ea1ae;border-radius:6px;background:#111a20e6;color:var(--text);font-size:12px;font-weight:700}.map-structure.base{min-width:68px;min-height:48px;border-color:var(--accent)}.map-structure.outpost{border-color:var(--orange)}.map-structure:hover,.map-structure.selected{box-shadow:0 0 0 4px #62a9e84d}.piece{position:absolute;width:52px;height:52px;margin:-26px 0 0 -26px;padding:0;border-radius:50%;background:transparent;cursor:grab;touch-action:none}.piece.dragging{cursor:grabbing;z-index:8}.piece.selected .piece-ring{border-color:#dceefd;box-shadow:0 0 0 4px #62a9e84d}.piece-ring{position:absolute;inset:1px;overflow:hidden;border:2px solid var(--blue-team);border-radius:50%;background:#11191f}.piece.enemy .piece-ring{border-color:var(--red-team)}.piece img{display:block;width:100%;height:100%;object-fit:contain}.piece-fallback{display:grid;width:100%;height:100%;place-items:center;background:var(--surface);font-weight:700}.piece-hp{position:absolute;right:0;bottom:-6px;left:0;height:6px;overflow:hidden;border:1px solid #10161b;border-radius:4px;background:#52606b}.piece-hp>i,.meter>i{display:block;height:100%;background:var(--accent)}.piece.low .piece-hp>i{background:var(--danger)}#goalMarker{position:absolute;z-index:2;width:38px;height:38px;margin:-19px 0 0 -19px;border:2px solid var(--orange);border-radius:50%;box-shadow:0 0 0 4px #d8985a33;pointer-events:none}#goalLabel{position:absolute;z-index:5;padding:5px 8px;border:1px solid #8f6849;border-radius:4px;background:#25201a;color:#f1c996;font-size:14px;pointer-events:none;white-space:nowrap}#resizeHandle{position:absolute;z-index:6;top:0;bottom:0;left:-4px;width:8px;cursor:col-resize}body.dock-left #resizeHandle{right:-4px;left:auto}#inspector{position:relative;min-width:0;overflow:auto;padding:24px;border-left:1px solid var(--line);background:var(--chrome)}body.dock-left #inspector{grid-column:1;grid-row:1;border-right:1px solid var(--line);border-left:0}body.dock-left .workspace{grid-column:2;grid-row:1}.inspector-head{display:flex;align-items:flex-start;justify-content:space-between;gap:12px;margin-bottom:24px}.eyebrow{margin:0 0 4px;color:var(--muted);font-size:14px}.inspector-head h2{margin:0;font-size:20px}.undo-button{min-height:40px;padding:0 12px}.undo-button:disabled{cursor:not-allowed;opacity:.45}details.section{margin:0 0 16px;border:1px solid var(--line);border-radius:10px;background:#1c252d;overflow:hidden}.section-toggle{display:flex;align-items:center;justify-content:space-between;width:100%;min-height:48px;padding:0 20px;background:#202b34;color:var(--text);cursor:pointer;font-size:15px;font-weight:650;list-style:none}.section-toggle::-webkit-details-marker{display:none}.section-toggle:after{content:"+";color:var(--muted);font-size:20px}details[open] .section-toggle:after{content:"−";color:var(--accent)}.section-body{padding:20px}.section-body h3{margin:0 0 12px;color:var(--muted);font-size:14px}.kv{display:grid;grid-template-columns:minmax(112px,.85fr) minmax(0,1.15fr);gap:10px 16px;margin:0}.kv dt{color:var(--muted)}.kv dd{min-width:0;margin:0;overflow-wrap:anywhere}.meter{height:8px;overflow:hidden;border-radius:4px;background:#53616c}.hp-editor{display:grid;gap:12px;margin-top:20px}.hp-readout{display:flex;align-items:baseline;justify-content:space-between;gap:12px;color:var(--muted)}.hp-readout strong{color:var(--text);font-size:18px}.hp-input{display:grid;grid-template-columns:minmax(0,1fr) auto;gap:12px}.hp-input input{width:100%;min-height:40px;padding:0 12px;border:1px solid var(--line);border-radius:6px;background:#141b21}.hp-actions{flex-wrap:wrap}.hp-actions button{min-width:58px;padding:0 10px}.hp-actions .restore{color:#d8ecff;border-color:#4d7494}.roster{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}.roster button{display:flex;align-items:center;min-height:48px;gap:10px;padding:6px 10px;border:1px solid var(--line);border-radius:6px;background:var(--surface);text-align:left;cursor:pointer}.roster button[aria-pressed=true]{border-color:var(--accent);background:var(--accent-soft)}.roster img{width:34px;height:34px;object-fit:contain}.roster button:disabled{cursor:not-allowed;opacity:.45}.read-only{color:#efc18d}.debug-only{display:none}body.debug-on .debug-only{display:block}body.compact #inspector{padding:16px}body.compact .section-body{padding:16px}body.compact details.section{margin-bottom:12px}body.focus-mode #inspector,body.focus-mode #resizeHandle{display:none}body.focus-mode .shell{grid-template-columns:minmax(0,1fr)}body.focus-mode .workspace{grid-column:1}body:fullscreen .shell{height:100vh}body:fullscreen #topbar{display:none}@media(max-width:1120px){body{overflow:auto}#topbar{align-items:flex-start;flex-direction:column}.commandbar{justify-content:flex-start;flex-wrap:wrap}.shell{height:auto;min-height:calc(100vh - 160px)}}@media(max-width:820px){body{overflow:auto}.shell,body.dock-left .shell{grid-template-columns:1fr;height:auto}.workspace{min-height:70vh;padding:16px}#inspector,body.dock-left #inspector{grid-column:1;grid-row:2;border-top:1px solid var(--line);border-right:0;border-left:0}body.dock-left .workspace{grid-column:1;grid-row:1}#resizeHandle{display:none}#mapCanvas{width:100%}}@media(max-width:560px){#topbar{padding:12px 16px}.brand{min-width:0}.command-group{padding-left:0;border-left:0}.status-strip{overflow-x:auto;max-width:100%}.workspace{padding:12px}#inspector{padding:16px}.piece{width:44px;height:44px;margin:-22px 0 0 -22px}.roster{grid-template-columns:1fr}}
</style></head><body><header id="topbar"><div class="brand"><div class="brand-mark">LY</div><div class="brand-copy"><h1>Sentinel Tactical Simulator</h1><p>Regional decision rehearsal</p></div></div><div class="commandbar" aria-label="tactical command bar"><div class="command-group"><div class="segmented" aria-label="field perspective"><button id="viewSideRed" type="button" aria-pressed="true">Red</button><button id="viewSideBlue" type="button" aria-pressed="false">Blue</button></div><button id="fitView" class="tool label" type="button">Fit</button><button id="actualSize" class="tool label" type="button">1:1</button><button id="mapZoomOut" class="tool" type="button" aria-label="Zoom out">−</button><button id="mapZoomIn" class="tool" type="button" aria-label="Zoom in">+</button><button id="resetView" class="tool" type="button" aria-label="Reset map view">↺</button></div><div class="command-group"><button id="fullscreenToggle" class="tool" type="button" aria-label="Fullscreen">⛶</button><button id="focusToggle" class="tool label" type="button" aria-pressed="false">Focus</button><button id="compactToggle" class="tool" type="button" aria-label="Compact inspector" aria-pressed="false">▤</button><button id="dockToggle" class="tool" type="button" aria-label="Move inspector dock">⇆</button><button id="resetLayout" class="tool" type="button" aria-label="Reset layout">↺</button><button id="debugToggle" class="tool label" type="button" aria-pressed="false">Debug</button></div><div class="command-group" id="matchActions"><button class="command primary" data-command="start">Start</button><button class="command" data-command="pause">Pause</button><button class="command" data-command="reset">Reset</button></div><div class="status-strip"><span id="ownerPill" class="pill">owner -</span><span id="decisionPill" class="pill">goal -</span><span id="controlPill" class="pill">control -</span></div></div></header><main class="shell"><section class="workspace"><div class="board-toolbar"><strong id="goalSummary">Waiting for decision trace</strong><span id="coordinateReadout">-</span></div><div id="mapViewport" aria-label="zoomable tactical field"><div id="mapCanvas"><div id="fieldBoard" role="application" aria-label="Tactical field. Select a robot or structure to inspect it. Drag a robot to move it."><img id="fieldImage" alt="Competition field map"><svg id="routeOverlay" aria-hidden="true"><polyline id="routeLine" fill="none" stroke="#d8985a" stroke-width="4" stroke-linejoin="round" stroke-linecap="round"></polyline></svg><div id="areaLayer"></div><div id="structureLayer"></div><div id="pieceLayer"></div><div id="goalMarker" hidden></div><div id="goalLabel" hidden></div></div></div></div></section><aside id="inspector" aria-label="selected object inspector"><div id="resizeHandle" aria-hidden="true"></div><div class="inspector-head"><div><p id="inspectorKind" class="eyebrow">Battlefield overview</p><h2 id="inspectorTitle">Regional field</h2></div><div class="inspector-actions"><button id="undoButton" class="undo-button" type="button" disabled>Undo</button></div></div><details id="overviewInspector" class="section" open><summary class="section-toggle" aria-expanded="true">Overview</summary><div class="section-body"><dl id="overview" class="kv"></dl></div></details><details id="objectInspector" class="section"><summary class="section-toggle" aria-expanded="false">Selected object</summary><div class="section-body"><div id="ownerMessage" class="read-only" hidden></div><div id="objectContent"></div></div></details><details id="rosterInspector" class="section"><summary class="section-toggle" aria-expanded="false">Robot roster</summary><div class="section-body"><div id="palette" class="roster"></div></div></details><details id="decisionInspector" class="section"><summary class="section-toggle" aria-expanded="false">Decision & navigation</summary><div class="section-body"><dl id="decision" class="kv"></dl><div style="height:20px"></div><dl id="navigation" class="kv"></dl></div></details><details id="tacticalInspector" class="section"><summary class="section-toggle" aria-expanded="false">Tactical state</summary><div class="section-body"><dl id="tactical" class="kv"></dl></div></details><details id="runtimeInspector" class="section debug-only"><summary class="section-toggle" aria-expanded="false">Runtime diagnostics</summary><div class="section-body"><h3>Final control output</h3><dl id="control" class="kv"></dl><div style="height:20px"></div><h3>Lower-machine feedback</h3><dl id="feedback" class="kv"></dl></div></details></aside></main><div id="liveRegion" role="status" aria-live="polite"></div>
<script>
const mapObjects=__MAP_OBJECTS__,board=document.getElementById('fieldBoard'),mapViewport=document.getElementById('mapViewport'),mapCanvas=document.getElementById('mapCanvas'),mapImage=document.getElementById('fieldImage'),pieceLayer=document.getElementById('pieceLayer'),structureLayer=document.getElementById('structureLayer'),areaLayer=document.getElementById('areaLayer'),palette=document.getElementById('palette'),goalMarker=document.getElementById('goalMarker'),goalLabel=document.getElementById('goalLabel'),routeLine=document.getElementById('routeLine'),liveRegion=document.getElementById('liveRegion');let state=null,paletteChoice=null,drag=null,placementPointer=null,pan=null,selection={kind:'overview',key:''},lastUndo=null;const storageKey='ly-simulator-tactical-layout-v2',saved=(()=>{try{return JSON.parse(localStorage.getItem(storageKey)||'{}')}catch(_){return{}}})(),view={side:saved.side||'red',zoom:Number(saved.zoom)||1,panX:Number(saved.panX)||0,panY:Number(saved.panY)||0,debug:Boolean(saved.debug),compact:Boolean(saved.compact),focus:Boolean(saved.focus),dock:saved.dock==='left'?'left':'right',inspectorWidth:Number(saved.inspectorWidth)||360};
const text=(v,f='-')=>v===null||v===undefined||v===''?f:String(v),numeric=(v,d=1)=>typeof v==='number'&&Number.isFinite(v)?v.toFixed(d):'-',clamp=(v,l,h)=>Math.max(l,Math.min(h,v));function pct(point){const f=state.field,left=100*Number(point.x)/f.width_cm,top=100*(1-Number(point.y)/f.height_cm);return view.side==='red'?{left,top}:{left:100-left,top:100-top}}function setPosition(el,point){const p=pct(point);el.style.left=p.left+'%';el.style.top=p.top+'%'}function persist(){localStorage.setItem(storageKey,JSON.stringify(view))}function applyView(){mapCanvas.style.transform='translate(calc(-50% + '+view.panX+'px),calc(-50% + '+view.panY+'px)) scale('+view.zoom+')';document.body.dataset.viewSide=view.side;document.body.classList.toggle('debug-on',view.debug);document.body.classList.toggle('compact',view.compact);document.body.classList.toggle('focus-mode',view.focus);document.body.classList.toggle('dock-left',view.dock==='left');document.documentElement.style.setProperty('--inspector-width',view.inspectorWidth+'px');for(const [id,value] of [['viewSideRed',view.side==='red'],['viewSideBlue',view.side==='blue'],['debugToggle',view.debug],['focusToggle',view.focus],['compactToggle',view.compact],['fullscreenToggle',Boolean(document.fullscreenElement)]])document.getElementById(id).setAttribute('aria-pressed',String(value));persist()}function announce(m){liveRegion.textContent=m}function command(payload){if(!state||!state.scene.can_edit)return Promise.resolve({ok:false,message:'manual_ros_observer_mode'});return fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)}).then(r=>r.json()).then(body=>{announce(body.ok?'Command accepted':'Command rejected: '+text(body.message));return body}).catch(error=>{announce('Command failed');return{ok:false,message:String(error)}})}function addKv(container,rows){container.textContent='';for(const[key,value]of rows){const dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=key;dd.textContent=text(value);container.append(dt,dd)}}function openInspector(id){for(const panel of document.querySelectorAll('details.section')){panel.open=panel.id===id;panel.querySelector('.section-toggle').setAttribute('aria-expanded',String(panel.open))}}function select(kind,key='',focus=true){selection={kind,key};if(focus&&kind!=='overview')openInspector('objectInspector');renderInspector();renderLayers()}function selectedUnit(){return selection.kind==='unit'?(state.scene.units||[]).find(x=>x.entity_id===selection.key):null}function selectedStructure(){return selection.kind==='structure'?(state.scene.structures||[]).find(x=>x.key===selection.key):null}function selectedArea(){return selection.kind==='area'?mapObjects.find(x=>x.key===selection.key):null}function mutate(payload,inverse){lastUndo={inverse};document.getElementById('undoButton').disabled=false;command(payload)}function hpEditor(current,maxValue,editable,onApply){const root=document.createElement('div'),meter=document.createElement('div'),fill=document.createElement('i'),readout=document.createElement('div'),pair=document.createElement('div'),input=document.createElement('input'),apply=document.createElement('button'),actions=document.createElement('div');root.className='hp-editor';meter.className='meter';fill.style.width=(100*clamp(current/Math.max(1,maxValue),0,1))+'%';meter.append(fill);readout.className='hp-readout';readout.innerHTML='<span>Current health</span><strong>'+current+' / '+maxValue+'</strong>';pair.className='hp-input';input.type='number';input.min='0';input.max=String(maxValue);input.value=String(current);input.disabled=!editable;input.setAttribute('aria-label','Health value');apply.type='button';apply.className='command primary';apply.textContent='Apply';apply.disabled=!editable;apply.onclick=()=>onApply(Math.round(clamp(Number(input.value),0,maxValue)));pair.append(input,apply);actions.className='hp-actions';for(const delta of[-500,-100,100,500]){const b=document.createElement('button');b.type='button';b.dataset.step=String(Math.abs(delta));b.textContent=(delta>0?'+':'')+delta;b.disabled=!editable;b.onclick=()=>onApply(clamp(current+delta,0,maxValue));actions.append(b)}for(const[name,value,style]of[['Restore',maxValue,'restore'],['Destroy',0,'danger']]){const b=document.createElement('button');b.type='button';b.className=style;b.textContent=name;b.disabled=!editable;b.onclick=()=>onApply(value);actions.append(b)}root.append(meter,readout,pair,actions);return root}
function renderPalette(){palette.textContent='';for(const item of state.scene.palette||[]){const b=document.createElement('button');b.type='button';b.disabled=!state.scene.can_edit;b.setAttribute('aria-pressed',String(paletteChoice&&paletteChoice.side===item.side&&paletteChoice.unit_key===item.unit_key));if(item.asset_url){const img=document.createElement('img');img.src=item.asset_url;img.alt='';b.append(img)}const label=document.createElement('span');label.textContent=text(item.side)+' '+text(item.type);b.append(label);b.onclick=()=>{paletteChoice=item;renderPalette();announce('Placement selected: '+text(item.type))};palette.append(b)}}function renderPieces(){pieceLayer.textContent='';for(const unit of state.scene.units||[]){if(!unit.position_cm)continue;const b=document.createElement('button'),ring=document.createElement('span'),meter=document.createElement('span'),fill=document.createElement('i');b.type='button';b.className='piece '+text(unit.side)+(selection.kind==='unit'&&selection.key===unit.entity_id?' selected':'')+(Number(unit.hp)/Math.max(1,Number(unit.max_hp))<.35?' low':'');b.dataset.entityId=unit.entity_id;b.setAttribute('aria-label',text(unit.side)+' '+text(unit.type)+' '+text(unit.hp)+' of '+text(unit.max_hp));setPosition(b,unit.position_cm);ring.className='piece-ring';if(unit.asset_url){const img=document.createElement('img');img.src=unit.asset_url;img.alt='';ring.append(img)}else{const fallback=document.createElement('span');fallback.className='piece-fallback';fallback.textContent=text(unit.type).slice(0,2);ring.append(fallback)}meter.className='piece-hp';fill.style.width=(100*clamp(Number(unit.hp)/Math.max(1,Number(unit.max_hp)),0,1))+'%';meter.append(fill);b.append(ring,meter);b.addEventListener('pointerdown',e=>{e.stopPropagation();select('unit',unit.entity_id);if(!state.scene.can_edit)return;drag={unit,pointerId:e.pointerId,element:b,startX:e.clientX,startY:e.clientY,moved:false};b.setPointerCapture(e.pointerId)});pieceLayer.append(b)}}function renderLayers(){areaLayer.textContent='';structureLayer.textContent='';if(!state)return;for(const item of mapObjects){const b=document.createElement('button');b.type='button';b.className='map-object'+(selection.kind==='area'&&selection.key===item.key?' selected':'');b.title=item.label+' · '+item.detail;b.setAttribute('aria-label',item.label+' '+item.field_side);setPosition(b,item);b.onclick=e=>{e.stopPropagation();select('area',item.key)};areaLayer.append(b)}for(const item of state.scene.structures||[]){if(!item.position_cm)continue;const b=document.createElement('button');b.type='button';b.className='map-structure '+text(item.kind)+(selection.kind==='structure'&&selection.key===item.key?' selected':'');b.textContent=text(item.kind)==='base'?'BASE':'OUTPOST';b.title=text(item.label)+' '+text(item.hp)+' / '+text(item.max_hp);b.setAttribute('aria-label',text(item.label));setPosition(b,item.position_cm);b.onclick=e=>{e.stopPropagation();select('structure',item.key)};structureLayer.append(b)}}
function renderInspector(){if(!state)return;const overview=state.decision||{},goal=overview.goal||{},kind=document.getElementById('inspectorKind'),title=document.getElementById('inspectorTitle'),content=document.getElementById('objectContent'),unit=selectedUnit(),structure=selectedStructure(),area=selectedArea(),editable=Boolean(state.scene.can_edit);addKv(document.getElementById('overview'),[['Match',state.match.running?'Running':'Paused'],['Time left',state.match.time_left==null?'-':text(state.match.time_left)+' s'],['Strategy',overview.strategy],['Current goal',goal.name],['Field',text(state.field.width_cm)+' × '+text(state.field.height_cm)+' cm'],['Input owner',state.scene.ownership_mode]]);content.textContent='';kind.textContent='Battlefield overview';title.textContent='Regional field';if(unit){kind.textContent='Robot inspector';title.textContent=text(unit.side)+' '+text(unit.type);const fields=document.createElement('dl');fields.className='kv';addKv(fields,[['Position',text(unit.position_cm.x)+', '+text(unit.position_cm.y)+' cm'],['Decision input',text(unit.decision_summary,'catalog mapping')],['Current strategy',overview.strategy],['Current goal',goal.name],['Runtime owner',state.scene.ownership_mode]]);content.append(fields,hpEditor(Number(unit.hp),Number(unit.max_hp),editable,hp=>mutate({command:'set_unit_hp',entity_id:unit.entity_id,hp},{command:'set_unit_hp',entity_id:unit.entity_id,hp:Number(unit.hp)})))}else if(structure){kind.textContent=text(structure.kind)==='base'?'Base inspector':'Outpost inspector';title.textContent=text(structure.label);const fields=document.createElement('dl'),tactical=state.tactical||{},castle=tactical.protect_castle||{};fields.className='kv';addKv(fields,[['Team',structure.field_side],['Coordinate',text(structure.position_cm.x)+', '+text(structure.position_cm.y)+' cm'],['Status',Number(structure.hp)>0?'Alive':'Destroyed'],['Decision evidence',text(structure.kind==='base'?castle.enabled:tactical.available?'Available':'Not recorded')],['Recent event',text(structure.kind==='base'?castle.enemy_pos_active:'Not recorded')]]);content.append(fields,hpEditor(Number(structure.hp),Number(structure.max_hp),editable,hp=>mutate({command:'set_structure_health',side:structure.side,structure:structure.kind,hp},{command:'set_structure_health',side:structure.side,structure:structure.kind,hp:Number(structure.hp)})))}else if(area){kind.textContent='Field area';title.textContent=area.label;const fields=document.createElement('dl');fields.className='kv';addKv(fields,[['Map point',area.detail],['Field side',area.field_side],['Coordinate',numeric(area.x,0)+', '+numeric(area.y,0)+' cm'],['Inspector mode','Reference only']]);content.append(fields)}const intent=overview.intent||{};addKv(document.getElementById('decision'),[['Strategy',overview.strategy],['Layer',intent.layer],['Reason',intent.reason],['Priority',intent.priority]]);addKv(document.getElementById('navigation'),[['Goal',text(goal.name)+' #'+text(goal.id)],['Coordinate',Array.isArray(goal.pos_cm)?text(goal.pos_cm[0])+', '+text(goal.pos_cm[1])+' cm':'-'],['Route points',Array.isArray(overview.route_cm)?overview.route_cm.length:0],['Output','/goal_pose or configured navi output']]);const tactical=state.tactical||{},castle=tactical.protect_castle||{},hero=tactical.protect_hero||{},defense=tactical.regional_defense||{};addKv(document.getElementById('tactical'),[['Evidence',tactical.available?'BT trace':'Not recorded'],['ProtectCastle',castle.enabled],['RFID stay',castle.stay_when_rfid_active],['Enemy in base',castle.enemy_pos_active],['ProtectHero',hero.active],['Defense threat',defense.threat_active],['Defense search',defense.search_kind],['Base enemies',defense.own_base_enemy_count]]);const output=state.control_output||{},fire=output.fire_code||{},trajectory=output.trajectory||{};addKv(document.getElementById('control'),[['Sequence',output.sequence],['Source',output.source],['Angles',output.angles&&output.angles.published?'yaw '+numeric(output.angles.yaw)+' pitch '+numeric(output.angles.pitch):'Not published'],['Fire',fire.published?'follow '+text(fire.follow_mode)+' rotate '+text(fire.rotate):'Not published'],['Trajectory',trajectory.available?'yaw '+numeric(trajectory.yaw)+' pitch '+numeric(trajectory.pitch):text(trajectory.unavailable_reason,'Unavailable')]]);const feedback=state.gimbal_feedback||{};addKv(document.getElementById('feedback'),[['Available',feedback.available],['Age',feedback.age_ms==null?'-':text(feedback.age_ms)+' ms'],['Rotate',feedback.fire_code&&feedback.fire_code.rotate]])}
function render(){if(!state)return;mapImage.src=state.stream.map_url+'?frame='+encodeURIComponent(text(state.stream.frame_id,'0'));if(selection.kind==='overview'&&state.scene.selected_entity_id)selection={kind:'unit',key:state.scene.selected_entity_id};const readonly=!state.scene.can_edit,output=state.control_output||{},message=document.getElementById('ownerMessage'),labels={manual_ros_observer_mode:'Manual ROS observer mode: Foxglove or ROS CLI owns scene facts.',simulator_inputs_disabled:'Scene input controls are disabled.',control_file_unavailable:'Scene command file is unavailable.',control_consumer_unavailable:'Replay viewer is read-only. Start offline decision mode to edit this board.'};message.hidden=!readonly;message.textContent=readonly?(labels[text(state.scene.edit_reason)]||'Scene editing is unavailable.'):'';for(const b of document.querySelectorAll('#matchActions button'))b.disabled=readonly;document.getElementById('ownerPill').textContent=state.scene.ownership_mode;document.getElementById('ownerPill').className='pill '+(readonly?'warn':'good');document.getElementById('controlPill').textContent=output.available?'control '+text(output.sequence):'control -';document.getElementById('controlPill').className='pill '+(output.available?'good':'warn');const decision=state.decision||{},goal=decision.goal||{};document.getElementById('goalSummary').textContent=goal.name?text(goal.name)+' · '+text(decision.intent&&decision.intent.reason):'Waiting for decision trace';document.getElementById('decisionPill').textContent=goal.name?'goal '+text(goal.id):'goal -';document.getElementById('decisionPill').className='pill '+(goal.name?'good':'warn');if(Array.isArray(goal.pos_cm)&&goal.pos_cm.length>=2){const p={x:Number(goal.pos_cm[0]),y:Number(goal.pos_cm[1])};setPosition(goalMarker,p);goalMarker.hidden=false;const q=pct(p);goalLabel.style.left=q.left+'%';goalLabel.style.top='calc('+q.top+'% - 34px)';goalLabel.textContent=text(goal.name)+' #'+text(goal.id);goalLabel.hidden=false}else{goalMarker.hidden=true;goalLabel.hidden=true}const points=(decision.route_cm||[]).map(p=>Array.isArray(p)&&p.length>=2?{x:Number(p[0]),y:Number(p[1])}:null).filter(Boolean).map(p=>{const q=pct(p);return q.left+','+q.top});routeLine.setAttribute('points',points.join(' '));renderPalette();renderPieces();renderLayers();renderInspector();applyView()}function readPoint(event){const rect=mapCanvas.getBoundingClientRect(),f=state.field,left=clamp((event.clientX-rect.left)/rect.width,0,1),top=clamp((event.clientY-rect.top)/rect.height,0,1);return view.side==='blue'?{x:Math.round((1-left)*f.width_cm),y:Math.round(top*f.height_cm)}:{x:Math.round(left*f.width_cm),y:Math.round((1-top)*f.height_cm)}}function refresh(){return fetch('/api/tactical-state',{cache:'no-store'}).then(r=>r.json()).then(next=>{state=next;render()}).catch(()=>announce('Tactical state unavailable'))}for(const section of document.querySelectorAll('details.section'))section.addEventListener('toggle',()=>{if(section.open)openInspector(section.id)});document.getElementById('viewSideRed').onclick=()=>{view.side='red';render()};document.getElementById('viewSideBlue').onclick=()=>{view.side='blue';render()};document.getElementById('mapZoomIn').onclick=()=>{view.zoom=clamp(view.zoom+.2,.45,4);applyView()};document.getElementById('mapZoomOut').onclick=()=>{view.zoom=clamp(view.zoom-.2,.45,4);applyView()};for(const id of['fitView','resetView'])document.getElementById(id).onclick=()=>{view.zoom=1;view.panX=0;view.panY=0;applyView()};document.getElementById('actualSize').onclick=()=>{view.zoom=clamp(state.field.width_cm/Math.max(1,mapViewport.clientWidth),.45,4);view.panX=0;view.panY=0;applyView()};document.getElementById('debugToggle').onclick=()=>{view.debug=!view.debug;applyView()};document.getElementById('focusToggle').onclick=()=>{view.focus=!view.focus;applyView()};document.getElementById('compactToggle').onclick=()=>{view.compact=!view.compact;applyView()};document.getElementById('dockToggle').onclick=()=>{view.dock=view.dock==='left'?'right':'left';applyView()};document.getElementById('resetLayout').onclick=()=>{Object.assign(view,{side:'red',zoom:1,panX:0,panY:0,debug:false,compact:false,focus:false,dock:'right',inspectorWidth:360});localStorage.removeItem(storageKey);applyView()};document.getElementById('fullscreenToggle').onclick=()=>{if(document.fullscreenElement)document.exitFullscreen();else document.documentElement.requestFullscreen()};document.addEventListener('fullscreenchange',applyView);document.getElementById('undoButton').onclick=()=>{if(!lastUndo)return;const undo=lastUndo;lastUndo=null;document.getElementById('undoButton').disabled=true;command(undo.inverse)};mapViewport.addEventListener('wheel',event=>{event.preventDefault();view.zoom=clamp(view.zoom+(event.deltaY<0?.12:-.12),.45,4);applyView()},{passive:false});mapViewport.addEventListener('pointerdown',event=>{if(event.button===1||event.altKey||event.shiftKey){pan={pointerId:event.pointerId,x:event.clientX,y:event.clientY,startX:view.panX,startY:view.panY};mapViewport.setPointerCapture(event.pointerId);event.preventDefault()}});board.addEventListener('pointerdown',event=>{if(!state||pan||event.target.closest('.piece,.map-structure,.map-object'))return;if(!state.scene.can_edit||!paletteChoice){select('overview','',false);return}placementPointer={pointerId:event.pointerId}});board.addEventListener('pointermove',event=>{if(!state)return;const point=readPoint(event);document.getElementById('coordinateReadout').textContent=point.x+', '+point.y+' cm';if(pan&&event.pointerId===pan.pointerId){view.panX=pan.startX+event.clientX-pan.x;view.panY=pan.startY+event.clientY-pan.y;applyView()}if(drag&&event.pointerId===drag.pointerId){drag.moved=drag.moved||Math.hypot(event.clientX-drag.startX,event.clientY-drag.startY)>4;if(drag.moved){drag.element.classList.add('dragging');setPosition(drag.element,point)}}});board.addEventListener('pointerup',event=>{if(pan&&event.pointerId===pan.pointerId){pan=null;return}if(drag&&event.pointerId===drag.pointerId){const moved=drag;drag=null;moved.element.classList.remove('dragging');if(moved.moved){const point=readPoint(event);command({command:'place_unit',entity_id:moved.unit.entity_id,side:moved.unit.side,unit_key:moved.unit.unit_key,hp:moved.unit.hp,x:point.x,y:point.y})}return}if(!placementPointer||event.pointerId!==placementPointer.pointerId)return;placementPointer=null;const point=readPoint(event);command({command:'place_unit',entity_id:paletteChoice.side+':'+paletteChoice.unit_key+':'+Date.now().toString(36),side:paletteChoice.side,unit_key:paletteChoice.unit_key,hp:paletteChoice.hp,x:point.x,y:point.y})});board.addEventListener('pointercancel',()=>{placementPointer=null;pan=null;if(drag){drag.element.classList.remove('dragging');drag=null}});board.addEventListener('dblclick',()=>{view.focus=true;applyView()});document.getElementById('resizeHandle').addEventListener('pointerdown',event=>{const handle=event.currentTarget,startWidth=view.inspectorWidth,startX=event.clientX;handle.setPointerCapture(event.pointerId);const move=next=>{const delta=next.clientX-startX;view.inspectorWidth=clamp(Math.round(startWidth+(view.dock==='right'?-delta:delta)),320,460);applyView()},stop=next=>{move(next);handle.removeEventListener('pointermove',move);handle.removeEventListener('pointerup',stop)};handle.addEventListener('pointermove',move);handle.addEventListener('pointerup',stop)});for(const button of document.querySelectorAll('#matchActions button'))button.onclick=()=>command({command:button.dataset.command});refresh();setInterval(refresh,500);
</script></body></html>"""
    return (
        body.replace("__MAP_OBJECTS__", json.dumps(_map_objects(), separators=(",", ":")))
        .replace(
            "</style>",
            "#pieceLayer{pointer-events:none}.piece{pointer-events:auto}"
            "@media(max-width:820px){.workspace{min-height:0}#mapViewport{flex:none;aspect-ratio:28/15}}"
            "@media(max-width:560px){#palette.roster{grid-template-columns:repeat(2,minmax(0,1fr))!important}}</style>",
        )
        .replace(
            "</script></body>",
            "</script><script>const stableRender=render;render=()=>{if(!drag)stableRender()};"
            "const primaryApplyView=applyView;applyView=()=>{if(state){const aspect=state.field.width_cm/"
            "state.field.height_cm,viewportAspect=mapViewport.clientWidth/Math.max(1,mapViewport.clientHeight),"
            "width=viewportAspect>aspect?mapViewport.clientHeight*aspect:mapViewport.clientWidth,height=width/aspect;"
            "mapCanvas.style.width=width+'px';mapCanvas.style.height=height+'px'}primaryApplyView()};"
            "window.addEventListener('resize',applyView);</script></body>",
        )
        .encode("utf-8")
    )


def safe_html_label(value: str) -> str:
    return html.escape(value, quote=True)
