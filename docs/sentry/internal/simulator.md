# Simulator Trace And Viewer

Updated: 2026-09-19

## Purpose

`simulator` is the maintained browser-first offline simulator for sentry decision behavior.
It replays JSONL rows written by `behavior_tree` and renders the current decision directly in the Tactical Board at the HTTP root.
Trace v4 adds BT-authored `tactical` evidence plus `control_output`, which is the final published
control snapshot. It remains separate from asynchronous lower-machine `gimbal_feedback`.
`tactical.protect_hero` preserves the effective YAML parameters and every activation gate
(profile/time, Hero position/health freshness, area membership, proactive-hold switch, and Base/Highland threat counts); its
`mode_gate_ready` is only the proactive-or-legacy-threat branch, not a replacement for the other gates,
so replay and Foxglove can explain an inactive Hero-protection decision without reimplementing BT policy.
The viewer shows whether the current navigation output is bridge `/goal_pose`, direct `UseXY` (`/ly/navi/goal_pos`), or goal-ID (`/ly/navi/goal`) mode.
The right panel shows live ROS topic values from `/goal_pose`, `/ly/navi/goal_pos_raw`, `/ly/navi/goal`, `/ly/navi/speed_level`, `/ly/navi/should_rotate`, and `/ly/control/vel` when started through the offline live wrapper.
In live mode, the current-goal marker and recent path prefer live `/goal_pose`; if that topic is absent, the viewer falls back to legacy `/ly/navi/goal_pos`, trace records, and labels the marker as `TRACE`.
The browser Tactical Board (`/`, with `/tactical` as an alias) is the single production workspace: its
**Activity rail** selects the existing Decision, Events, Runtime, Control, Inputs, and Layers surfaces; the
contextual Inspector reports the selected map object; and the collapsible **Operations shelf** retains
replay/timeline information. These are presentation regions, not new state or a second simulator.

The shared product identity is **Sentinel Flight Deck**: a presentation-only matte industrial surface with
`#4DB7FF` as the operation/selection accent. Team red and blue remain battlefield data colours, not general
UI accents; command bus, decision trace, ROS interfaces, and `manual_ros` observer-only behaviour are unchanged.
`SimulationRuntime` owns trace/match/scene updates without a display dependency and exposes shared state to the
browser. Native pygame is now an explicit debug renderer (`--debug-pygame` / `--live-debug-pygame`) only; it is
not captured, streamed, or embedded by the Tactical Board.

This is for decision review and offline decision simulation. The viewer itself does not publish ROS topics.
In offline mode, it can send file-based control commands to `simulator.mock_inputs` for match clock and mock referee/unit inputs.
It can also preload placed units from a JSON/YAML scene with `--unit-scene`; `simulator.start` passes that same file to the viewer and mock input publisher.
The offline mock publisher also covers formal decision inputs for referee event data, sentry info, RFID, team buff, gimbal state, bullet state, navigation status, self position, official target fallback, and optional external aim without changing formal ROS2 subscribers.

## Components

- Trace writer: `src/behavior_tree/src/DecisionTrace.cpp`
- Launch parameters: `decision_trace_enabled`, `decision_trace_file`, `decision_trace_every_n_ticks`
- Viewer package: `src/simulator`
- Simulator-facing trace adapter: `src/simulator/simulator/trace.py`
- Simulator-facing view model: `src/simulator/simulator/model.py`
- Foxglove offline exporter: `src/simulator/simulator/foxglove_export.py`
- Headless simulation runtime: `src/simulator/simulator/runtime.py`
- Tactical browser board: `src/simulator/simulator/tactical_web.py` (`/`, `/tactical` alias)
- Canonical scene catalog/state: `src/simulator/config/tactical_catalog.yaml`, `simulator.scene`, and `simulator.interactive_inputs`
- Viewer config: `src/simulator/config/default.yaml`
- Default map: `tools/maps/basemaps/buff_map_field.png`
- Asset manifest: `src/simulator/assets/manifest.yaml`
- Asset loader: `src/simulator/simulator/assets.py`
- Rule-aware structure overlay source: `src/simulator/config/default.yaml` -> `structures`
- The structure overlay includes formal `PreRoadland.*` and `ReadyRoadland.*` main-area polygons, plus `Recovery.*`, `CentralLeft.*`, and `SettleArea.*`. `ReadyRoadland` uses the former ReadyRoadLand boundary; `PreRoadland` is its same-level front-road peer.
- Scripted route overlay source: `src/simulator/config/default.yaml` -> `scripted_path`
- Interactive mock input source: `src/simulator/config/default.yaml` -> `simulator_inputs`
- Field coordinate conversion: `src/simulator/simulator/field.py`
- Offline input state and command application: `src/simulator/simulator/interactive_inputs.py`
- Inputs tab rendering and hitboxes: `src/simulator/simulator/inputs_panel.py`
- Right-panel scroll state and clamping: `src/simulator/simulator/panel_scroll.py`
- Full-roster clean visual asset QA: `src/simulator/simulator/visual_asset_qa.py`
- Clean visual asset QA config: `src/simulator/config/visual_asset_qa.yaml`
- File/API command bus schema: `src/simulator/simulator/control_bus.py`
- Energy mechanism debug fields are recorded under `referee`: `has_sentry_info`, `sentry_can_activate_energy`, and `energy_activate_confirm_pulse`.

## Tactical Sandbox

The browser `http://127.0.0.1:9000/` owns the production catalog-backed scene commands. Pieces,
base/outpost HP, target goal, route, tactical evidence, final control output, and lower-machine feedback
are intentionally shown as different facts. Pygame may inspect the same debug data only when explicitly started.

`mock` is the default input owner. Only this mode starts `simulator.mock_inputs`; browser/Pygame edits
are written to the command bus and then projected onto existing formal ROS input topics. The formal
Regional launch path is unchanged.

Browser edits are available only from the live runtime launched by
`simulator.start --offline-decision --live-view`, because that runtime follows and consumes the command
bus. A standalone `simulator.main ... --web-stream` trace replay exposes `/tactical` as read-only; it
cannot claim to recompute a recorded decision from a dragged piece.

The Tactical Board Red/Blue selector sends `set_team` on that same command bus. In mock mode it updates the
existing `/ly/friend/is_team_red` publication, so behavior_tree receives the matching `UnitTeam`. `friend` and
`enemy` remain relative data roles: their piece and structure borders are colored from each item's derived
physical `field_side` (`friend=blue`, `enemy=red` for a blue sentry). Switching team does not silently move
placed units or reset HP; their official coordinates remain the scene facts currently under test.

`manual_ros` is observer-only. It is for Foxglove or `ros2 topic pub` users who already own the input
topics. Both the browser UI and `/api/control` reject scene mutations in this mode, so mock and external
publishers cannot race each other.

The browser map is the interactive presentation of that same scene state. Its command bar and map toolbar
provide Fit, 1:1, **Zoom to Selection**, zoom, pan, Fullscreen, dock/collapse, and Reset Layout. Mouse wheel
zoom preserves the official coordinate under the pointer; middle-button drag pans. The Activity rail keeps
existing surfaces compact, while the Operations shelf can expand for the replay timeline. Clicking a Base,
Outpost, robot, or configured map area opens its contextual Inspector; clicking empty field returns to
battlefield Overview. Base/Outpost and robot health actions retain the existing `set_structure_health` /
`set_unit_hp` commands. Every map interaction uses official centimeter coordinates and does not bypass the
control bus or alter behavior-tree logic.

```bash
# Editable scene, mock input owner, browser Tactical Board.
PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision --mode regional --live-view --input-owner mock \
  --unit-scene src/simulator/sample/unit_scenes/tactical_board.yaml \
  --mock-sequence src/simulator/sample/mock_sequences/tactical_protection.json

# Observe external Foxglove/ROS input only. The acknowledgement label never starts a publisher.
PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision --mode regional --live-view --trace-on \
  --input-owner manual_ros --external-input-publisher foxglove
```

For the first command, open `http://127.0.0.1:9000/`. Map pointer coordinates are sent as
official field centimeters, never pixels. `manual_ros` can still display scene/trace data, but it cannot
alter it.

The Tactical Board is a map-first command desk rather than a second simulator implementation. `/tactical`
is only an alias. Its Red/Blue
selector changes only the browser's field perspective: Blue rotates the map and mirrors screen projection,
while pointer commands are converted back to the same official centimeter coordinates. Zoom, wheel zoom,
Alt/middle-button pan, Reset view, accordion expansion, and Debug visibility are likewise browser-local.
The top command bar keeps the `mm:ss` match countdown, Start/Pause/Reset, and `+/-10s` controls direct.
Expanding the Operations shelf exposes the same shared match clock as a slider, exact `mm:ss` entry, and
`+/-30s` controls. `rewind` only increases remaining match time, `forward` only decreases it, and
`set_time_left` assigns it within `match_control.duration_sec`; none of them restore unit positions, HP,
referee flags, or prior decision-trace rows. The Inspector starts with
only Overview expanded. Click a robot, Base, Outpost, or catalog area to switch its selected-object
content; click blank field to return to the battlefield overview. Robot, Base, and Outpost inspectors
retain the existing `set_unit_hp` and `set_structure_health` command payloads, including Apply, small
step, Restore, Destroy, and local Undo controls. Robot roster, Decision & navigation, and Tactical state
remain collapsed until requested; Runtime diagnostics appears only after Debug is enabled.
Inspector accordion state and an in-progress HP value survive the 500 ms live-state refresh. A pointer-up
without movement selects a robot; moving it updates its screen position immediately and emits the existing
`place_unit` command only on release.

The browser stores only presentation preferences locally (dock side, Inspector width, compact/focus
mode, perspective, and map view). It does not persist or synthesize simulator facts. Fit, 1:1, wheel
zoom, Alt/middle-button pan, double-click Focus, Fullscreen, Inspector resize/dock, and Reset layout are
browser-local presentation controls.

`tactical_catalog.yaml` remains the sole map catalog. The base markers use the confirmed official field
coordinates: Red C1 `(245, 755)` cm and Blue C2 `(2555, 745)` cm. BuffOutpost is Red `(1220, 1281)` cm
and Blue `(1580, 219)` cm. Scene command coordinates remain official left-bottom-origin centimeters
regardless of the browser perspective.

In trace v4, `tactical.protect_castle` records configuration state including `stay_when_rfid_enabled`, and separately records RFID raw/effective
activation, the resolved `stay_when_rfid_active` Castle-hold state, and enemy-position activation. The simulator never derives a Castle
stay lock from piece geometry; it only displays BT-authored evidence. `tactical.regional_defense` carries the exact BT threat observation,
last search kind, and counts. The simulator does not recreate those results from geometry. When reviewing
FollowMode, read `control_output.fire_code.follow_mode` and `control_output.fire_code.rotate`; do not infer
the final command from `gimbal_feedback`.

`tactical.protect_outpost` records the BT-owned own-outpost HP, freshness, destruction flag, phase, event
generation, damage window, damage threshold, search hold, and priority. Offline replay displays this evidence
without deriving damage from scene edits; `destroyed=true` means the formal BT revoked the C3/C4 task, while a
later positive HP sample is only a new baseline until the configured damage threshold is reached again.

Foxglove MCAP export writes the existing decision/metrics/goal channels plus:

- `/sentry/simulator/decision/control_output`
- `/sentry/simulator/decision/tactical`
- `/sentry/simulator/scene`

```bash
PYTHONPATH=src/simulator python3 -m simulator.foxglove_export \
  src/simulator/sample/scenarios/tactical_protect_castle.jsonl \
  --output /tmp/tactical_protect_castle.mcap
```

## Trace Recording

Tracing is opt-in and stays off for normal competition runs.
`decision_trace_enabled` defaults to `false`; `decision_trace_file` is only used after that switch is explicitly enabled.

```bash
./scripts/start.sh nogate --mode league \
  decision_trace_enabled:=true \
  decision_trace_file:=log/decision_trace.jsonl \
  decision_trace_every_n_ticks:=5
```

Direct launch:

```bash
ros2 launch behavior_tree sentry_all.launch.py mode:=league \
  decision_trace_enabled:=true \
  decision_trace_file:=log/decision_trace.jsonl \
  decision_trace_every_n_ticks:=5
```

Leaving `decision_trace_enabled:=false` means no trace file is opened or written, even if a path is accidentally supplied.

Trace is observational only. If JSON serialization or the trace file write fails after startup,
`behavior_tree` closes and disables the trace for the rest of that run; it does not alter the BT tick,
navigation output, Tactical policy, or control topics.

Optional indexed wrapper (reads `src/behavior_tree/Scripts/ConfigJson`):

```bash
PYTHONPATH=src/simulator python3 -m simulator.start --list-configs
PYTHONPATH=src/simulator python3 -m simulator.start --mode league --bt-config league/chase_only_competition.json --entry nogate
```

Offline decision test (not replay, behavior_tree + mock topics only):

```bash
PYTHONPATH=src/simulator python3 -m simulator.start --offline-decision --mode regional --bt-config regional_competition.json
```

Offline decision + browser Tactical Board:

```bash
PYTHONPATH=src/simulator python3 -m simulator.start --offline-decision --mode regional --live-view
```

Before launching live view, `simulator.start` auto-cleans stale old `simulator.main` viewer processes.

For regional super confrontation timing, use a 7-minute match clock (`420` seconds):

```bash
PYTHONPATH=src/simulator python3 -m simulator.start --offline-decision --mode regional --live-view --match-duration-sec 420
```

Preload units into the live viewer and mock input topics:

```bash
PYTHONPATH=src/simulator python3 -m simulator.start --list-unit-scenes

PYTHONPATH=src/simulator python3 -m simulator.unit_scene \
  src/simulator/sample/unit_scene.json \
  --team red

PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mode regional \
  --live-view \
  --unit-scene src/simulator/sample/unit_scene.json
```

Decision-context mock inputs can be combined with unit scenes. These knobs are forwarded only to `simulator.mock_inputs`; they do not alter the formal behavior-tree launch path.

Named mock presets make common input states reproducible without long command lines:

```bash
PYTHONPATH=src/simulator python3 -m simulator.start --list-mock-presets

PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mock-preset official-target-sentry \
  --live-view

PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mock-preset multi-unit-regional \
  --mock-ammo 120 \
  --live-view

PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mock-preset uwb-fusion \
  --live-view

PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mock-preset full-roster-regional \
  --live-view
```

Presets are simulator-only input overlays and require `--offline-decision`; explicit CLI flags override preset values.
The names describe mocked input state, not guaranteed behavior-tree outcomes. The selected BT config still controls task enablement, target selection, chase, recovery, and publication gates.
Current preset coverage:

- `buff-ready`: buff target, sentry activation flag, energy event, team buff energy, center/self-outpost RFID, and regional self position.
- `outpost-dead`: enemy outpost HP is zero while resource fields remain present.
- `nav-unreachable`: target-visible context with navigation reachability forced false.
- `official-target-sentry`: official target fallback with behavior-tree `ArmorType::Sentry` ID `6`.
- `uwb-fusion`: opt-in `/ly/friend/uwb_pos` self-position fusion rehearsal with CLI coordinates in official field centimeters.
- `bullet-resource`: BulletInfo resource snapshot with speed, shoot data, projectile allowance, and remaining gold coin fields.
- `multi-unit-regional`: loads `src/simulator/sample/unit_scene.json` so live view art and mock HP/position topics include multiple unit classes.
- `full-roster-regional`: loads `src/simulator/sample/unit_scenes/full_roster.json` for full packaged unit-art, formal health-unit HP mapping, and placed-unit PositionData coverage.
- `low-resource`: low sentry HP and low ammo recovery input state.

Scripted mock sequences cover temporal input facts that static presets cannot represent. `simulator.start --offline-decision --mock-sequence ...` starts a simulator-only control-bus producer after `simulator.mock_inputs` has opened the command file. The sequence emits existing JSONL control commands such as `start`, `set_time_left`, `set_self_position`, `set_self_health`, `set_ammo`, `set_posture`, `set_unit`, `set_unit_hp`, and `set_structure_health`; only `simulator.mock_inputs` publishes ROS topics.

```bash
PYTHONPATH=src/simulator python3 -m simulator.mock_sequence --list-samples

PYTHONPATH=src/simulator python3 -m simulator.start --list-mock-sequences

PYTHONPATH=src/simulator python3 -m simulator.mock_sequence \
  src/simulator/sample/mock_sequences/regional_timed_context.json \
  --control-file /tmp/simulator_match_control.jsonl \
  --dry-run

PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mode regional \
  --mock-preset multi-unit-regional \
  --mock-sequence src/simulator/sample/mock_sequences/regional_timed_context.json \
  --live-view
```

Bundled sequence examples:

- `regional_timed_context.json`: opens the start gate, updates match time and self position, places friend/enemy units, changes unit/self HP, lowers ammo, changes posture, and drops enemy outpost HP to zero.
- `buff_timeout_context.json`: starts from a buff-ready regional context, moves through posture/ammo/target HP changes, jumps match time, and removes the enemy Sentry target to rehearse timeout/fallback behavior.
- `low_resource_recovery_exit.json`: starts near base with low self HP/ammo, then restores HP/ammo, moves self position forward, and keeps an enemy Hero context to rehearse leaving recovery state.
- `official_target_fallback_companion.json`: companion script for `--mock-preset official-target-sentry`; opens the gate and changes nearby unit/self facts while the preset publishes `/ly/navi/target_official`.
- `multi_unit_target_priority_rehearsal.json`: multi-unit HP/position/resource rehearsal for watching target-priority behavior under changing enemy context.
- `league_3v3_center_status.json`: drives the RMUL 3V3 center status through ours, enemy, contested, ours, and unoccupied so the `LeagueSimple` policy can be replayed without a live referee link.

League 3V3 rehearsal uses `src/simulator/config/league_3v3.yaml`, `tools/maps/basemaps/RMUL2026_3V3_topdown_field.png`, and `scripts/simulator/league_3v3_demo.sh`. In 3V3 mode the JSON `Task.Outpost=false` is preserved against the regional `Task.yaml` default so the league policy is not preempted; the launcher also disables the runtime start gate so the center policy is exercised immediately.

Sequence `at_sec` values are monotonic scheduler elapsed seconds, not referee match time. Use `set_time_left` actions when a test needs an explicit match-clock transition.

Offline workflow playbooks pair presets, sequences, unit scenes, expected evidence, and verification commands for common regional debugging jobs:

```bash
PYTHONPATH=src/simulator python3 -m simulator.offline_workflow

PYTHONPATH=src/simulator python3 -m simulator.offline_workflow multi-unit-target-priority

PYTHONPATH=src/simulator python3 -m simulator.offline_workflow uwb-position-fusion --json
```

Current workflow IDs are `regional-buff-timeout`, `regional-outpost-collapse`, `official-target-fallback`, `uwb-position-fusion`, `bullet-info-resource-snapshot`, `multi-unit-target-priority`, `low-resource-recovery-exit`, and `full-roster-visual-inputs`.
Each workflow is simulator-only guidance: it prints a start command, preflight dry-runs, post-run trace checks, and evidence to inspect; it does not publish ROS topics and does not alter formal launch behavior.

Decision-input coverage is tracked as a simulator-owned catalog:

```bash
PYTHONPATH=src/simulator python3 -m simulator.decision_input_coverage

PYTHONPATH=src/simulator python3 -m simulator.decision_input_coverage unit_hp_position

PYTHONPATH=src/simulator python3 -m simulator.decision_input_coverage --json
```

The catalog maps formal behavior-tree input groups to mock flags, control-bus commands, trace fields, fixtures, workflows, viewer surfaces, and known gaps.
It covers match state, structure HP, unit HP/position, self position, navigation status/velocity, referee event/energy data, team buff, RFID, target streams, detector armor lists, official target fallback, gimbal/fire/posture state, optional external aim, and BulletInfo resource state.
Known non-complete areas are explicit: Drone and Infantry3 remain visual/offline context for current formal UnitInfo, and optional external aim depends on `sentry_msgs` and a matching BT config.
BulletInfo is traceable through `bullet_info` records, `/status.json.current_record.bullet_info`, the Runtime tab, and Foxglove export. Current behavior-tree decisions still use the existing legacy ammo/speed gates unless the formal logic changes separately.

Detector armor-list input can be published by the mock node on the formal `/ly/detector/armors` topic:

```bash
PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mode regional \
  --live-view \
  --mock-armors true \
  --mock-armor 1:6.0 \
  --mock-armor 3:4.5 \
  --mock-armor 6:5.2
```

`--mock-armor` uses `ArmorType:DISTANCE_M`; distance is meters and is copied by `behavior_tree` into target distance evidence. The subscriber intentionally ignores `/ly/detector/armors` when `ExternalAimSettings.Enable` is true. `ArmorType::Sentry` is ID `6`, while simulator draggable Sentry units use `UnitType` ID `7`.

```bash
PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mode regional \
  --live-view \
  --unit-scene src/simulator/sample/unit_scene.json \
  --mock-self-health 260 \
  --mock-enemy-outpost-health 0 \
  --mock-sentry-can-activate-energy true \
  --mock-event-self-small-energy-status 2 \
  --mock-team-buff-remaining-energy 35 \
  --mock-rfid-center-gain-point true \
  --mock-navi-reachable true
```

Official target fallback can be injected with `/ly/navi/target_official` semantics:

```bash
PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mode regional \
  --live-view \
  --mock-official-target-valid true \
  --mock-official-target-x 1505 \
  --mock-official-target-y 905 \
  --mock-official-target-armor-type 6
```

`--mock-official-target-armor-type` uses behavior-tree `ArmorType` IDs, where Sentry is `6`.
Draggable simulator units use `UnitType` IDs, where Drone is `6` and Sentry is `7`.
Do not treat Drone art as a formal `Health.msg` or `UnitInfo` decision field.

UWB self-position fusion can be rehearsed explicitly. The CLI takes official field centimeters, then `simulator.mock_inputs` publishes `/ly/friend/uwb_pos` with the raw-y convention expected by `behavior_tree/src/SubscribeMessage.cpp`:

```bash
PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mode regional \
  --live-view \
  --mock-publish-uwb-position true \
  --mock-uwb-position-x 1220 \
  --mock-uwb-position-y 760
```

External aim can be mocked when the selected BT config reads the sentry external aim path:

```bash
PYTHONPATH=src/simulator python3 -m simulator.start \
  --offline-decision \
  --mode regional \
  --live-view \
  --mock-external-aim true \
  --mock-external-aim-target-id 6 \
  --mock-external-aim-yaw 7.5 \
  --mock-external-aim-pitch -1.2
```

This publishes `/ly/aim/armor_targets` and `/ly/aim/result` from `simulator.mock_inputs` using existing `sentry_msgs` message types. It is optional, simulator-only, and requires `sentry_msgs` Python imports from the sourced workspace.

One-command fixed regional wrapper:

```bash
python3 scripts/python/start.py
```

The wrapper loads `tactical_board.yaml` by default. It contains the complete
14-piece red/blue roster and keeps one enemy Hero in MyBase so the first live
decision can exercise ProtectCastle. Pass `--unit-scene` only to replace that
default with a focused fixture.

This offline entry neither sources nor requires `sentry.aim` or the extended
`AimResult` velocity/acceleration fields. It uses the regular mock/referee
inputs and does not publish `/ly/aim/*` unless `--mock-external-aim true` is
explicitly chosen. The formal `gated` and `nogate` launch paths still require
the extended external AimResult contract.

Offline mode keeps `/ly/game/is_start` gate enabled by default; press `Start` in viewer/web to publish game-start and enter match phase.
Offline mode enables `runtime_rearm_start_gate:=true` by default. After `Reset`, behavior_tree re-enters start gate,
holds safe-control, publishes Home navigation goal, and waits for next `Start`.
Offline mode also forces `NaviSetting.ToNavi=false` via a generated temp config, so `/ly/navi/goal_pos` remains official map coordinates.
Use `--keep-to-navi` only when you need transformed bridge output.

Live view now also serves the same pygame frame to HTTP by default (port from YAML `web_stream.port`, default `9000`):

- `http://127.0.0.1:9000/` for local browser
- `http://<your-ip>:9000/` for LAN browser

Override live-view web port:

```bash
PYTHONPATH=src/simulator python3 -m simulator.start --offline-decision --mode regional --live-view --live-web-port 9010
```

With the default 100 Hz BT tick rate:

- `decision_trace_every_n_ticks:=1` records about 100 rows/s.
- `decision_trace_every_n_ticks:=5` records about 20 rows/s.
- `decision_trace_every_n_ticks:=10` records about 10 rows/s.

## Viewer

Run from source:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main log/decision_trace.jsonl
```

Open the sample:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main
```

Select a different map:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main \
  log/decision_trace.jsonl \
  --map tools/maps/basemaps/RMUC2026_V1.2.0_topview_cad_field.png
```

Validate a trace without opening pygame:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main log/decision_trace.jsonl --validate-only
```

`--validate-only` prints a simulator health report with:

- `Status: PASS`, `WARN`, or `FAIL`.
- record count, duration, tick range, schema-version counts, and output-kind counts.
- stable issue codes such as `schema.missing_decision_output`, `trace.time_monotonic`, and `output.goal_pos_bounds`.
- per-issue suggestions that describe what to fix before trusting replay, Foxglove export, or offline decision comparison.

Use `FAIL` reports as blockers for trace-based regression evidence.
Review `WARN` reports before comparing decision behavior.
Warnings are grouped by domain (`schema`, `trace`, `output`, `unit`, `runtime`, `referee`, `scenario`) so reviewers can scan likely root cause quickly.

For CI or scripts, use the JSON report format:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main \
  src/simulator/sample/scenarios/route_churn_warning.jsonl \
  --validate-only \
  --validate-format json
```

The JSON report schema is `ly_simulator_validation_report_v1`.
It includes `status`, `summary`, `issues`, `issue_groups`, and `next_actions`.
`--validate-format json` is only valid with `--validate-only`; text remains the default format.

Scenario-level diagnostics are warning-only and currently include:

- `scenario.route_churn`: rapid route changes inside a short trace window.
- `scenario.stale_chase_target`: chase/relative-target output while all target freshness flags are false.
- `output.goal_reach_mismatch`: detailed goal reach state does not match the emitted decision output goal.
- `output.relative_target_frame_missing`: valid relative target output lacks a frame ID.
- `output.navi_velocity_scale`: traced raw navigation velocity has a non-positive conversion scale.
- `output.should_rotate_missing`: `/ly/navi/should_rotate` is marked fresh without a value.
- `runtime.posture_lag`: posture command/feedback remains pending for multiple seconds.
- `scenario.match_time_jump`: referee match time delta does not match trace-time delta.
- `referee.missing_resource_state`: outpost/base HP fields are missing from rows that are otherwise usable.

## Scenario Fixtures

`src/simulator/sample/scenarios/` contains compact named fixtures for repeatable offline checks.
The fixture manifest is `src/simulator/sample/scenarios/manifest.json`.

The current suite covers:

- `startup_home_hold`: game-start home hold with no recent target.
- `target_acquisition`: fresh auto-aim target and regional patrol output.
- `buff_activation`: buff activation window and energy confirmation pulse.
- `relative_target_bridge`: bridge-mode relative target output to `/goal_pose`.
- `goal_id_output`: legacy goal-ID output path to `/ly/navi/goal`.
- `goal_pos_raw_bridge`: normal goal position routed through the raw bridge to `/goal_pose`.
- `chase_goal_pos`: official target chase output published directly as goal position.
- `chase_goal_pos_raw_bridge`: official target chase output routed through the raw bridge to `/goal_pose`.
- `outpost_attack`: enemy outpost attack window with outpost aim source.
- `low_resource_recovery`: low HP/ammo/damage recovery decision.
- `start_gate_lifecycle`: reset re-arms the start gate, Home is held, then the first active regional patrol goal is selected after game start.
- `route_churn_warning`: expected-WARN fixture for rapid route selection churn.
- `multi_unit_decision_context`: multi-unit HP/position/unit-info context, including reliable enemy position source evidence.
- `tactical_protect_castle`: trace v4 ProtectCastle EnemyPos evidence and castle defense goal.
- `tactical_follow_rotate`: trace v4 proof that final FollowMode forces final Rotate to zero while feedback may still report another gear.

The manifest records expected validation status, record count, schema-version coverage, output-kind coverage, intent layers/reasons, goals, aim modes, and important event/target/runtime flags.
For multi-record fixtures, the manifest can also assert event, goal, intent-reason, publish-allowed, match-time, and warning-code sequences.
`src/simulator/test/test_scenarios.py` reads the manifest and validates every bundled fixture.

Scripted mock sequence examples are separate from replay fixtures and live under `src/simulator/sample/mock_sequences/`.
They can be listed with `simulator.mock_sequence --list-samples` or `simulator.start --list-mock-sequences`.
After selecting a file, check it with `simulator.mock_sequence --dry-run`; when launched through `simulator.start`, the sequence is validated before dry-run or real process orchestration continues.
Current examples are `regional_timed_context.json`, `buff_timeout_context.json`, and `low_resource_recovery_exit.json`.

Run all fixture checks:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/simulator \
  python3 -m pytest src/simulator/test/test_scenarios.py -q
```

Run a single fixture through the CLI report:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main \
  src/simulator/sample/scenarios/relative_target_bridge.jsonl \
  --validate-only
```

Export a recorded trace to Foxglove-readable MCAP:

```bash
python3 -m pip install -r src/simulator/requirements-foxglove.txt
PYTHONPATH=src/simulator python3 -m simulator.foxglove_export \
  log/decision_trace.jsonl \
  -o log/decision_trace.mcap
```

The same export can be reached through the viewer entrypoint without opening pygame:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main \
  log/decision_trace.jsonl \
  --export-foxglove log/decision_trace.mcap
```

Foxglove export is an offline post-processing path. It reads JSONL through the simulator adapter and writes MCAP only when explicitly invoked; normal robot runs and BT ticks do not import or execute the exporter.

Disable HTTP stream:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main log/decision_trace.jsonl --no-web-stream
```

Change stream endpoint:

```bash
PYTHONPATH=src/simulator python3 -m simulator.main \
  log/decision_trace.jsonl \
  --web-host 0.0.0.0 \
  --web-port 9000
```

HTTP endpoints:

- `GET /`: compact browser dashboard with the latest pygame frame, match-control buttons, stream readiness, trace/replay/current-decision summaries, simulator-input state, placed units, and alert lists.
- `GET /frame.jpg`: latest pygame frame as JPEG; returns `503` until the first frame is published.
- `GET /status.json`: machine-readable stream status, including `ready`, `has_frame`, `frame_id`, `frame_age_sec`, control-file availability, host/port, FPS, JPEG quality, trace metadata, validation status, replay state, selected-record summary, and simulator input state.
- `GET /healthz`: readiness probe; returns `200` after the first frame is available and `503` before then.
- `POST /api/control`: appends simulator command-bus JSONL commands when `match_control.control_file` is configured.

`/healthz` reports readiness, not ongoing frame freshness. Automation that needs to detect a stalled viewer should also inspect `frame_age_sec` from `/status.json`.
HTTP responses expose only whether control is enabled and the control-file basename; the resolved local path is kept server-side.

When a trace is loaded, `/status.json` also includes simulator metadata:

- `trace`: basename-only trace name, follow mode, record count, bad-line count, duration, and tick range.
- `validation`: the same `ly_simulator_validation_report_v1` payload used by `--validate-format json`.
- `replay`: play/pause state, replay speed, current record index, total records, active panel tab, match-clock state, and whether local controls are available.
- `current_record`: compact selected-record debug summary with tick/event, team, strategy, aim, target, goal, output topic/frame, official chase metadata, goal reach state, navigation status/velocity, relative target payload, intent, posture, referee resources, and runtime guard state.
- `simulator_inputs`: whether offline input controls are enabled, last command status, runtime self HP/ammo/posture/self-position, structure HP, placed units, unit palette, low-HP summaries, and `PositionData` raw-coordinate rows.

The metadata payload is strict JSON: non-finite floats are serialized as `null`, and resolved local trace/control-file paths are not exposed.
The browser page escapes the control-file basename before rendering it as HTML.

Quick local readiness check:

```bash
curl http://127.0.0.1:9000/status.json
curl -f http://127.0.0.1:9000/healthz
```

Optional browser visual QA starts a temporary stream with a synthetic simulator frame, opens the dashboard in Chromium through Playwright, checks desktop and narrow layouts, and writes screenshots to `/tmp/ly-simulator-web-visual`:

```bash
PYTHONPATH=src/simulator python3 -m simulator.web_visual_check --json
```

If Playwright or its Chromium browser is not installed, the command reports `status: skip` by default. Install the optional browser tooling and make missing browser support a hard failure when you want a real-browser gate:

```bash
python3 -m pip install -r src/simulator/requirements-browser.txt
playwright install chromium
PYTHONPATH=src/simulator python3 -m simulator.web_visual_check --require-browser
```

The viewer includes a structure layer for quick rule-context checks:

- Walls / major barrier blocks
- Energy mechanism (rule section `4.3.2.2`)
- Outpost stations (rule section `4.3.2.3`)

These are currently approximated from map artwork + rule figure alignment and are intended for offline simulator, not millimeter-level navigation constraints.

YAML default stream config is under `src/simulator/config/default.yaml`:

- `web_stream.enabled`
- `web_stream.host`
- `web_stream.port`
- `web_stream.fps`
- `web_stream.jpeg_quality`
- `map_tags.goal_tags_expanded`
- `map_tags.hover_goal_tags`
- `map_tags.hover_radius_px`
- `ros_monitor.state_file`
- `ros_monitor.poll_sec`
- `ros_monitor.stale_sec`

Offline match-control defaults are in the same YAML:

- `match_control.duration_sec`
- `match_control.rewind_step_sec`
- `match_control.forward_step_sec`
- `match_control.control_file`

Scripted route defaults are in the same YAML:

- `scripted_path.enabled`
- `scripted_path.goal_ids` / `scripted_path.points_cm`
- `scripted_path.speed_cmps`
- `scripted_path.side` / `scripted_path.loop`
- `scripted_path.show_future`

Live follow mode panel includes `Start`, `Pause`, `+10s`, `-10s`, `Reset` controls for offline mock match timing.
The HTTP page (`/`) provides the same control buttons and writes commands to `match_control.control_file`.
Match clock is rendered as a real-time countdown; pressing `Start` starts countdown immediately.
Live view opens the full viewer immediately, then updates when real trace rows arrive.
The right panel tab can be changed with mouse clicks or keyboard `1` / `2` / `3` / `4` / `5`.
Mouse wheel over the right panel scrolls dense tab content without seeking the replay timeline.
When the mouse is over the right panel, `Up` / `Down` / `PageUp` / `PageDown` / `Home` / `End` scroll that tab; otherwise the timeline shortcuts keep their normal replay behavior.
The `Layers` tab toggles terrain, structures, simulator inputs, grid, goals, paths, units, HP bars, and recent changes without editing YAML.
The `Inputs` tab is only connected to behavior-tree decisions when running with `--offline-decision --live-view`;
in normal trace playback it remains a visual UI and reports that the live control bus is unavailable.

Interactive inputs use existing ROS contracts:

- Gimbal controls publish `/ly/gimbal/angles`, `/ly/gimbal/posture`, `/ly/gimbal/firecode`, `/ly/gimbal/chassis`, and `/ly/gimbal/capV`.
- Structure HP buttons publish `std_msgs/UInt16` through `simulator.mock_inputs` to `/ly/friend/op_hp`, `/ly/enemy/op_hp`, `/ly/friend/base_hp`, and `/ly/enemy/base_hp`.
- Dragged unit HP publishes through `gimbal_driver/msg/Health` on `/ly/friend/hp` or `/ly/enemy/hp`.
- Dragged unit positions publish through `gimbal_driver/msg/PositionData` on `/ly/position/data`.
- One `PositionData` message carries one friend row and one enemy row. The mock publisher pairs
  roster rows this way, so a full seven-versus-seven scene emits seven messages per timer cycle
  instead of fourteen and stays within the behavior-tree subscriber queue depth.
- Viewer coordinates are official field centimeters with left-bottom origin. `PositionData.friendy` / `enemyy` are sent as `1500 - official_y` because `behavior_tree/src/SubscribeMessage.cpp` normalizes them back to official-map y.
- Event and referee CLI knobs publish `/ly/game/event_data`, `/ly/game/sentry/info`, `/ly/team/buff`, and `/ly/game/rfid`.
- Bullet CLI knobs publish `/ly/game/bullet`.
- Detector armor-list CLI knobs publish `/ly/detector/armors`.
- Navigation CLI knobs publish `/ly/navi/reached`, `/ly/navi/reachable`, `/ly/navi/should_rotate`, `/ly/navi/vel`, `/ly/navi/lower_head`, `/ly/navi/position`, optional `/ly/friend/uwb_pos`, and optional `/ly/navi/target_official`.
- External aim CLI knobs publish `/ly/aim/armor_targets` and `/ly/aim/result` only when `--mock-external-aim true`.
- `/ly/navi/position` and `/ly/navi/target_official` use official field centimeters directly. `/ly/friend/uwb_pos` accepts official field centimeters in CLI flags but publishes raw y so behavior_tree reconstructs official y as `1500 - raw_y`. `/ly/position/data` also applies raw-y inversion before publish.

`--mock-navi-reached` is a static input, not simulated travel: it defaults to `false`; when set
to `true`, the mock publishes `true` on every timer tick for whichever goal is current. It does
not move `/ly/navi/position` or determine arrival from the emitted goal. For realistic patrol
checks leave it `false` and move the self position through the board/control bus; otherwise a
stationary simulated sentry will eventually exercise the Regional `MyBase.TravelTimeoutSec`
fallback and rotate to another Castle point.

Control-bus command groups:

- Match clock: `start`, `pause`, `reset`, `rewind`, `forward`, `set_time_left`.
- Self state: `set_self_health`, `set_ammo`, `set_posture`, `set_self_position`.
- Structures: `set_structure_health` / `set_structure_hp`.
- Units: `set_unit`, `set_units`, `set_unit_hp`, `remove_unit`, `clear_units`.

Structure HP uses the semantic ROS value, not the lower-machine packed value: each outpost starts at
`1500` HP and is edited/published as `0..1500` through `/ly/friend/op_hp` or `/ly/enemy/op_hp`.
The lower machine's legacy `GameCodeType` packs this as a 6-bit value in units of 25, but
`gimbal_driver` already expands it before publishing the ROS topic.

The pygame viewer map toolbar supports `Fit`, `1:1`, zoom, and `Full`; `F11` enters/exits fullscreen,
and `Esc` leaves fullscreen without closing the viewer.

`set_self_position` accepts `x`/`y`, `position_cm`, `position`, or `pos` payloads in official field centimeters. Invalid or non-finite coordinates are ignored instead of stopping the mock publisher.

`--unit-scene` accepts JSON or YAML as a top-level list, `{ "units": [...] }`, or `{ "units": { "friend": [...], "enemy": [...] } }`.
Each unit needs `side`, `type` or `type_id`, and either `x`/`y` or `position_cm`.
List bundled scenes with `simulator.unit_scene --list-samples` or `simulator.start --list-unit-scenes`.
Inspect a scene before launch with `simulator.unit_scene <scene> --team red|blue`; the summary shows the resulting `Health.msg` field, `/ly/position/data` car ID/raw-y value, field-side mapping, and sprite availability.
Use `--json` for a machine-readable `ly_simulator_unit_scene_summary_v1` payload.
The scene import only initializes simulator input state; publishing still uses the same mock topics above.
`src/simulator/sample/unit_scenes/full_roster.json` is a visual QA scene that covers red/blue Hero, Engineer, Infantry, Sentry, and Drone sprites.
Scene summaries, Inputs-tab rows, and map labels include decision-channel badges:

- `BT:HP,POS,UI`: Hero, Engineer, Infantry1, Infantry2, and Sentry publish mock HP/position facts that the current behavior-tree UnitInfo path consumes and can emit back into trace `unit_info`.
- `PUB:HP,POS noUI`: Infantry3 publishes `Health.msg.reserve` plus `/ly/position/data`, but the current formal `RobotLists`/UnitInfo path does not consume it as a decision unit.
- `PUB:POS noUI`: Drone publishes `/ly/position/data` and is drawn as tactical/visual context, but it has no formal `Health.msg` field and no current UnitInfo output.

For enemy units, `BT:HP` also means the current HP field can affect target/resource reasoning in the selected BT config.
For friend units, the same badge means the fact is available on the formal mock input channel; the selected BT config still decides whether that friend-side fact changes behavior.
Infantry3 and Drone are intentionally included in full-roster scenes because they are useful tactical context and asset coverage, but they are not current formal BT UnitInfo decision units.

After recording an offline trace, compare a scene against formal `unit_info` evidence with:

```bash
PYTHONPATH=src/simulator python3 -m simulator.unit_trace \
  src/simulator/sample/scenarios/multi_unit_decision_context.jsonl \
  --unit-scene src/simulator/sample/unit_scenes/multi_unit_trace_contract.json
```

`simulator.unit_trace` requires traceable units from the scene to appear in `unit_info` with matching HP and official-map position. `Infantry3` and `Drone` are reported as skipped because the current formal `RobotLists`/`UnitInfoArray` contract excludes those unit types.

Visual unit art is loaded from `src/simulator/assets/manifest.yaml`.
Red and blue Hero, Engineer, Infantry, Sentry, and Drone sprites are packaged under `src/simulator/assets/units/`.
`Infantry1`, `Infantry2`, and `Infantry3` share the Infantry sprite through manifest aliases.
Armor images from the same manifest are used in the right-panel target preview when available.
The source archive is not used at runtime; if assets are disabled or missing, the viewer falls back to circle markers.
The manifest records local provenance for `素材.zip`, including SHA-256 `8716faeaadce88422023023f923af5679c404241151b81d9a7a96b126fe963f4`.
The current license status is `unknown`, so redistribution is documented as `local_project_only_until_license_confirmed`.
The pygame `Layers` tab exposes asset catalog status, alias coverage, source archive name, import date, and license/redistribution status for operator review.

Pixel-level full-roster sprite visibility can be checked directly:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/simulator SDL_VIDEODRIVER=dummy \
  python3 -m simulator.main \
  --smoke-test \
  --web-port 9023 \
  --config src/simulator/config/visual_asset_qa.yaml \
  --unit-scene src/simulator/sample/unit_scenes/full_roster.json \
  --smoke-screenshot /tmp/ly-simulator-full-roster-visual-qa.png

PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/simulator SDL_VIDEODRIVER=dummy \
  python3 -m simulator.visual_asset_qa \
  --screenshot /tmp/ly-simulator-full-roster-visual-qa.png \
  --unit-scene src/simulator/sample/unit_scenes/full_roster.json \
  --config src/simulator/config/visual_asset_qa.yaml \
  --team red \
  --json
```

Use the clean QA screenshot for automated pass/fail decisions.
Use `/tmp/ly-simulator-full-roster-smoke.png` for manual review of the normal UI because normal overlays may intentionally cover pieces.

The `0` HP value is valid for outpost/base topics, so behavior_tree accepts zero on those four structure HP subscribers.
Self HP and per-unit HP are different from structure HP in the current formal subscribers.
`/ly/game/all.selfhealth` updates behavior_tree self health only when `selfhealth > 0`, and `/ly/friend/hp` / `/ly/enemy/hp` update formal Hero, Engineer, Infantry1, Infantry2, and Sentry HP only when each Health field is nonzero.
The simulator still allows `set_self_health` and `set_unit_hp` to publish `0` so operators can rehearse visual/offline states, but validation warns with `referee.zero_self_hp_runtime_mismatch` or `unit.zero_hp_runtime_mismatch` when a trace row could be mistaken for formal BT zero-HP evidence.
Use positive low-health values for recovery/posture rehearsal unless the formal behavior_tree subscriber contract is changed in a separate runtime patch.
Per-unit HP callbacks still follow the existing robot-health freshness behavior; removing a placed piece stops future mock position publishes, so downstream freshness timeout controls when that piece disappears from decisions.
`scripted_path` is disabled by default because it is a visual-only simulated route overlay, not a behavior_tree output.
When scripted path is enabled, route marker movement uses `speed_cmps` and elapsed match time.
By default, only the visited route plus the current target segment is drawn; set `show_future: true` to draw the full planned route.

Run the standard simulator-only quality gate from the workspace root:

```bash
PYTHONPATH=src/simulator python3 -m simulator.quality
```

The default gate runs the simulator pytest suite, Python bytecode compilation, sample validation, expected-WARN route-churn JSON validation, unit-scene catalog/summary checks, offline workflow catalog checks, decision-input coverage catalog checks, pygame asset decode/scale/blit smoke coverage, multi-unit UnitInfo trace evidence, headless web smoke tests including the full-roster scene, clean visual asset QA, and simulator-related text whitespace checks, including untracked files.
The normal full-roster smoke writes `/tmp/ly-simulator-full-roster-smoke.png` so asset scale, overlay interaction, and panel layout can be inspected after a headless run.
The automated pixel-level sprite check uses `src/simulator/config/visual_asset_qa.yaml` and `/tmp/ly-simulator-full-roster-visual-qa.png`, where non-essential overlays are disabled so map labels, current-goal markers, and HP bars do not hide unit art and create false failures.
Add the optional real-browser dashboard visual gate when Playwright/Chromium is installed:

```bash
PYTHONPATH=src/simulator python3 -m simulator.quality --with-browser-visual
```

Add package integration build when needed:

```bash
PYTHONPATH=src/simulator python3 -m simulator.quality --with-build
```

Preview the exact commands without running them:

```bash
PYTHONPATH=src/simulator python3 -m simulator.quality --dry-run --with-build
```

After build:

```bash
colcon build --packages-select simulator
source install/setup.bash
ros2 run simulator simulator log/decision_trace.jsonl
```

After `source install/setup.bash`, the same quality gate is available as:

```bash
ros2 run simulator simulator-quality --with-build
```

## Trace Schema

Each line is one JSON object. Important top-level fields:

- `schema`: currently `ly_decision_trace_v1`
- `schema_version`: `2` adds `decision_output`, `decision_intent`, `events`, `target_state`, `goal_reach_state`, `navi_status`, `navi_velocity`, `navi_relative_target`, `face_mode`, `gimbal`, and `runtime_guard`; `3` adds `gimbal_feedback` and `control_output`. The viewer still reads older rows without the newer fields.
- `event`: `game_start`, `tick`, or `stop`
- `t`: seconds from `gameStartTime`
- `field_cm`: map frame and field size
- `competition_profile`, `strategy_mode`, `team`; current live strategy names include `LeagueSimple` and `Regional`
- `aim_mode`, `target_armor`, `target_state`
- `events`: EventManager semantic snapshot, including Buff/Outpost/RegionalDefense/resource/damage/navigation event flags
- `decision_output`: stable viewer-facing output model; includes final output kind, topic, goal ID, `goal_pos_cm`, publish flags, bridge hints, relative-target validity, and official chase target metadata
- `decision_intent`: typed decision metadata for why the current output was selected; includes layer, reason, base goal ID, resolved goal ID, goal team, team-offset flag, priority, and detail
- `navi_goal`: ID, base ID, side, speed, publish flags, and `position_cm`
- `goal_reach_state`: detailed navigation reach result, including status/reason IDs, external reach/reachable freshness, position freshness, distance, arrive/face thresholds, and timeout; `has_position` means a valid fused self position has been received, while distance/threshold fields require both `has_position` and `position_fresh`
- `navi_status`: effective `/ly/navi/should_rotate`, `/ly/navi/reached`, and `/ly/navi/reachable` values with freshness flags
- `navi_velocity`: `/ly/navi/vel` input and `/ly/control/vel` output raw values plus the raw-to-m/s scale
- `navi_relative_target`: chase/bridge relative target, including frame ID, x/y/z, distance, yaw/pitch error, armor type, aim mode, and official target metadata
- `face_mode`: `FaceModeManager` 的本拍统一仲裁结果，包括 request 来源、是否接管、是否被视觉得分优先/导航兼容抑制、是否 fallback patrol，以及最终候选 yaw/pitch
- `posture`: command, state, runtime desired/current/pending, reason
- `referee`: HP, ammo, time, outpost/base HP, RFID/RFID2 raw state, `rfid_match`, event-data center/energy/fortress gain-point state, and buff state
- `unit_info`: optional formal FriendInfo/EnemyInfo-like unit records used by the viewer and validation when present
- `gimbal.fire_code.follow_mode`: semantic firecode bit4; old `hole_mode` naming should no longer be used in new traces/docs
- `gimbal_feedback`: callback-time `/ly/gimbal/firecode` feedback snapshot. `available` and `age_ms` distinguish no received frame from a real all-zero frame. Its nested fire-code fields are intentionally separate from legacy `gimbal.fire_code`, which remains a compatibility snapshot of mutable BT state.
- `control_output`: the most recently constructed and actually published BT control snapshot, not a recomputation from current state. It has a monotonic `sequence`, `age_ms`, and `source` (`normal` or `safe_control`), plus independent `angles`, `fire_code`, and `trajectory` subobjects. Each subobject records `published`; trajectory also records `available` and an explicit `unavailable_reason`. `game_start` and `stop` records retain the latest snapshot or `not_recorded`; they never imply a new control publication.
- `units`: friend/enemy unit records with type, HP, distance, and `position_cm`
- `runtime_guard`: current fault and recovery state

## Stable Simulator Contract

The simulator consumes `TraceRecord` from `src/simulator/simulator/model.py`, not behavior-tree internals directly.
`src/simulator/simulator/trace.py` is the adapter boundary from raw JSONL into that view model.

Stable viewer-facing fields are:

- `decision_output`: final navigation output and publish semantics.
- `decision_intent`: decision explanation for why that output was selected.
- `events`: semantic decision conditions, not BT node-local booleans.
- `goal_reach`: detailed reachability/reached/distance state for the emitted navigation goal.
- `navi_status`: traced navigation status inputs that affect rotate/reached/reachable behavior.
- `navi_velocity`: traced navigation velocity input/output values used to explain control velocity.
- `target_state`: active aim-source state, current/typed aim freshness, and target-set summaries.
- `navi_relative_target`: relative chase/bridge payload, including the source frame when valid.
- `posture`, `referee`, `gimbal`, `runtime_guard`, and `units`: runtime state needed to explain decisions.
- `gimbal_feedback`: lower-machine fire-code feedback evidence captured before BT-local mutation.
- `control_output`: final BT angle, fire-code, and optional trajectory publication evidence, including safe-control output.

Behavior-tree internals may add optional debug fields under names such as `debug` or `bt_debug`.
Adding or renaming those debug-only fields must not require simulator changes.
Simulator changes are required only when the stable contract above changes, when a new viewer-visible state is needed, or when validation/visual assumptions change.

The Foxglove exporter uses the same normalized `TraceRecord` contract and writes:

- `/sentry/simulator/decision`: full normalized decision frame.
- `/sentry/simulator/metrics`: numeric scalars useful for plots, including goal reach status/reason, goal distance, should-rotate, reachable/reached, and converted navigation velocity.
- `/sentry/simulator/goal_point_cm`: `foxglove.Point2`-shaped goal point in field centimeters when available.

The viewer uses coordinates in this order:

1. `decision_output.goal_pos_cm`, then legacy `navi_goal.position_cm`, and `units.*.position_cm` from trace.
2. `src/simulator/config/default.yaml` goal coordinates.
3. Optional `--points-json` map plugin coordinates when non-zero.

Terrain height is configured, not inferred. `config/default.yaml` contains an approximate 2D elevation overlay based on the visible map artwork. Treat it as a debug layer only until verified against official CAD/rule metadata.

Navigation goal ID `1` is `Base`, at red `(245, 750)` and blue `(2555, 750)`.
The simulator base HP structure badges use the same points.
Navigation goal ID `19` is `Highland`, a regional Highland compatibility/via point at red `(744, 1263)` and blue `(2056, 237)`. Current regional Highland compatibility arrival radius is 20 cm; during enter/via/leave transitions, behavior_tree may set `follow_mode` while routing through Highland or `CastleLeft1`. `CastleLeft1` is ID `5` at red `(510, 964)` and blue `(2290, 536)`; `CastleLeft2` is ID `20` at red `(831, 960)` and blue `(1969, 540)`.

## Maintenance Rules

When behavior-tree internals change but the stable simulator contract is unchanged, no simulator change is required.

The longer simulator work plan is tracked in `docs/sentry/internal/simulator_offline_debug_roadmap.md`.

Update the simulator in the same change only for these contract-level changes:

- New or renamed stable trace fields: update `src/behavior_tree/src/DecisionTrace.cpp`, `src/simulator/simulator/model.py`, `src/simulator/simulator/trace.py`, contract tests under `src/simulator/test/`, and this document.
- New output checks: update `src/simulator/simulator/validation.py`.
- New schema sample coverage: update `src/simulator/sample/sample_trace.jsonl` so parser and smoke tests exercise the current stable trace version.
- New scenario coverage: update `src/simulator/sample/scenarios/manifest.json`, the relevant scenario JSONL file, and `src/simulator/test/test_scenarios.py` if the expectation shape changes.
- New point IDs or map assumptions: update `src/simulator/config/default.yaml`.
- New visual layer or style: update `src/simulator/simulator/viewer.py` and this document.
- New right-panel scroll behavior: update `src/simulator/simulator/panel_scroll.py`, `src/simulator/simulator/viewer.py`, panel-scroll tests, and this document.
- New Foxglove-exported stable field: update `src/simulator/simulator/foxglove_export.py` and exporter tests.
- New offline mock input control: update `src/simulator/simulator/interactive_inputs.py`, `src/simulator/simulator/inputs_panel.py`, `src/simulator/simulator/mock_inputs.py`, `src/simulator/simulator/control_bus.py`, `src/simulator/config/default.yaml`, and this document.
- New formal decision input coverage, mock flag, trace-evidence mapping, workflow reference, or known simulator-only gap: update `src/simulator/simulator/decision_input_coverage.py`, tests, and this document.
- New scripted mock sequence command or sequence schema: update `src/simulator/simulator/mock_sequence.py`, `src/simulator/simulator/control_bus.py`, `src/simulator/simulator/start.py`, `src/simulator/sample/mock_sequences/`, package data in `src/simulator/setup.py`, tests, and this document.
- New unit art, asset mapping, or automated sprite visibility assumption: update `src/simulator/assets/manifest.yaml`, `src/simulator/config/visual_asset_qa.yaml` when overlay assumptions change, package data in `src/simulator/setup.py`, asset/visual-QA tests, and this document.
- New field coordinate or raw `PositionData` conversion: update `src/simulator/simulator/field.py` and simulator tests.
- New decision interface docs: update the current internal or regional document under `docs/sentry/` and add a dated implementation record under `docs/record/` when the change needs historical context.

Do not move this tool back under `tools/`; it is a maintained source package because it tracks the decision interface.
