from __future__ import annotations

import html
import io
import json
import math
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

from .control_bus import append_command, normalize_api_control_payload
from .tactical_web import (
    TacticalAssetRegistry,
    build_tactical_html,
    is_scene_mutation,
    tactical_state_from_status,
)


def json_safe(value: Any) -> Any:
    if value is None or isinstance(value, (bool, str)):
        return value
    if isinstance(value, int) and not isinstance(value, bool):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return str(value)


def build_index_html(port: int, control_enabled: bool, control_label: str, default_step_sec: int) -> bytes:
    disabled = "false" if control_enabled else "true"
    escaped_control_label = html.escape(control_label, quote=True)
    control_hint = (
        f"control: enabled ({escaped_control_label})"
        if control_enabled
        else "control: disabled (no control_file)"
    )
    body = f"""<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Simulator Live</title>
<style>
:root {{
  color-scheme: dark;
  --bg: #020617;
  --panel: #0f172a;
  --panel-2: #111827;
  --border: #243041;
  --muted: #94a3b8;
  --text: #e5edf7;
  --strong: #f8fafc;
  --good: #22c55e;
  --warn: #f59e0b;
  --bad: #ef4444;
  --info: #38bdf8;
}}
* {{ box-sizing: border-box; }}
body {{
  margin: 0;
  min-height: 100vh;
  background: var(--bg);
  color: var(--text);
  font: 13px/1.45 system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
}}
button {{
  min-height: 30px;
  padding: 6px 10px;
  color: var(--text);
  background: #1f2937;
  border: 1px solid #334155;
  border-radius: 4px;
  cursor: pointer;
}}
button:hover:not(:disabled), button:focus-visible {{
  border-color: var(--info);
  outline: none;
}}
button:disabled {{ opacity: .45; cursor: not-allowed; }}
#topbar {{
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 9px 12px;
  background: #07111f;
  border-bottom: 1px solid var(--border);
}}
#title {{
  font-weight: 700;
  color: var(--strong);
  white-space: nowrap;
}}
#pills {{
  display: flex;
  flex-wrap: wrap;
  gap: 6px;
  margin-left: auto;
}}
.pill {{
  border: 1px solid var(--border);
  border-radius: 999px;
  padding: 3px 8px;
  color: var(--muted);
  background: #0b1220;
  font-size: 12px;
}}
.pill.ready {{ color: #bbf7d0; border-color: #14532d; background: #052e16; }}
.pill.warn {{ color: #fde68a; border-color: #78350f; background: #2c1c03; }}
.pill.bad {{ color: #fecaca; border-color: #7f1d1d; background: #2b0808; }}
#ctrl {{
  display: flex;
  align-items: center;
  flex-wrap: wrap;
  gap: 8px;
  padding: 8px 12px;
  background: #0b1220;
  border-bottom: 1px solid var(--border);
}}
#msg {{ color: var(--muted); font-size: 12px; }}
#shell {{
  display: grid;
  grid-template-columns: minmax(360px, 1fr) minmax(340px, 430px);
  gap: 10px;
  padding: 10px;
}}
#framePane, #dashboard {{
  min-width: 0;
  border: 1px solid var(--border);
  background: var(--panel);
}}
#framePane {{ display: flex; align-items: flex-start; justify-content: center; }}
#f {{ width: 100%; height: auto; display: block; background: #000; }}
#dashboard {{
  display: grid;
  gap: 8px;
  padding: 8px;
  align-content: start;
  max-height: calc(100vh - 98px);
  overflow: auto;
}}
.card {{
  border: 1px solid var(--border);
  background: var(--panel-2);
  border-radius: 6px;
  padding: 8px;
}}
.card h2 {{
  margin: 0 0 6px;
  color: var(--strong);
  font-size: 12px;
  letter-spacing: 0;
}}
dl {{ display: grid; grid-template-columns: 94px minmax(0, 1fr); gap: 4px 8px; margin: 0; }}
dt {{ color: var(--muted); }}
dd {{ margin: 0; color: var(--text); overflow-wrap: anywhere; }}
ul {{ margin: 0; padding-left: 18px; }}
li {{ margin: 2px 0; }}
.status-pass {{ color: var(--good); }}
.status-warn {{ color: var(--warn); }}
.status-fail {{ color: var(--bad); }}
.empty {{ color: var(--muted); }}
@media (max-width: 940px) {{
  #topbar {{ align-items: flex-start; flex-direction: column; }}
  #pills {{ margin-left: 0; }}
  #shell {{ grid-template-columns: 1fr; }}
  #dashboard {{ max-height: none; }}
}}
:root {{
  --fd-root:#111418;
  --fd-raised:#171C22;
  --fd-panel:#1D232B;
  --fd-card:#202832;
  --fd-border:rgba(255,255,255,.06);
  --fd-text:#F5F7FA;
  --fd-muted:#AEB7C2;
  --fd-accent:#4DB7FF;
  --fd-warning:#FFB648;
  --fd-danger:#FF6262;
  --fd-success:#58D68D;
}}
body {{ background:var(--fd-root); color:var(--fd-text); font-size:15px; }}
button {{ min-height:40px; padding:0 13px; border-color:var(--fd-border); border-radius:8px; background:var(--fd-panel); transition:background 150ms ease-out,border-color 150ms ease-out,transform 150ms ease-out; }}
button:hover:not(:disabled),button:focus-visible {{ border-color:var(--fd-accent); background:var(--fd-card); transform:translateY(-1px); }}
#topbar {{ min-height:80px; padding:12px 24px; gap:16px; background:var(--fd-raised); border-color:var(--fd-border); }}
#title {{ display:flex; flex-direction:column; gap:2px; color:var(--fd-text); font-size:18px; line-height:1.2; }}
#title span {{ color:var(--fd-muted); font-size:13px; font-weight:400; }}
#pills {{ gap:8px; }}
.pill {{ padding:6px 10px; border-color:var(--fd-border); background:var(--fd-panel); color:var(--fd-muted); font-size:13px; }}
.pill.ready {{ color:var(--fd-success); border-color:rgba(88,214,141,.38); background:rgba(88,214,141,.08); }}
.pill.warn {{ color:var(--fd-warning); border-color:rgba(255,182,72,.38); background:rgba(255,182,72,.08); }}
.pill.bad {{ color:var(--fd-danger); border-color:rgba(255,98,98,.38); background:rgba(255,98,98,.08); }}
#ctrl {{ min-height:64px; padding:12px 24px; gap:8px; background:var(--fd-raised); border-color:var(--fd-border); }}
#msg {{ margin-left:8px; color:var(--fd-muted); font-size:13px; }}
#shell {{ grid-template-columns:minmax(0,1fr) minmax(300px,360px); gap:16px; padding:24px; }}
#framePane,#dashboard {{ border:1px solid var(--fd-border); border-radius:12px; background:var(--fd-panel); box-shadow:0 14px 34px rgba(0,0,0,.22); overflow:hidden; }}
#framePane {{ align-items:center; aspect-ratio:16/9; min-height:220px; }}
#f {{ max-width:100%; min-width:0; max-height:100%; object-fit:contain; background:#0b0f13; }}
#dashboard {{ gap:12px; padding:16px; max-height:calc(100vh - 168px); background:var(--fd-raised); }}
.card {{ padding:16px; border-color:var(--fd-border); border-radius:12px; background:linear-gradient(180deg,rgba(255,255,255,.025),transparent 42%),var(--fd-card); box-shadow:0 8px 18px rgba(0,0,0,.16); }}
.card h2 {{ margin:0 0 12px; color:var(--fd-text); font-size:15px; font-weight:600; }}
dl {{ grid-template-columns:96px minmax(0,1fr); gap:8px 12px; }}
dt,.empty {{ color:var(--fd-muted); }}
dd {{ color:var(--fd-text); }}
li {{ margin:5px 0; }}
@media (max-width:940px) {{ #topbar,#ctrl {{ padding-left:16px; padding-right:16px; }} #shell {{ grid-template-columns:1fr; gap:16px; padding:16px; }} #framePane {{ min-height:0; }} #dashboard {{ max-height:none; }} }}
</style>
</head>
<body>
<div id="topbar">
  <div id="title"><strong>Sentinel Flight Deck</strong><span>Live stream :{int(port)}</span></div>
  <div id="pills" aria-label="stream status">
    <span id="readyPill" class="pill warn">waiting</span>
    <span id="framePill" class="pill">frame 0</span>
    <span id="agePill" class="pill">age -</span>
    <span id="validationPill" class="pill">validation -</span>
    <span id="matchPill" class="pill">match -</span>
  </div>
</div>
<div id="ctrl">
  <button data-cmd="start">Start</button>
  <button data-cmd="pause">Pause</button>
  <button data-cmd="rewind" data-seconds="{int(default_step_sec)}">+{int(default_step_sec)}s</button>
  <button data-cmd="forward" data-seconds="{int(default_step_sec)}">-{int(default_step_sec)}s</button>
  <button data-cmd="reset">Reset</button>
  <span id="msg">{control_hint}</span>
</div>
<main id="shell">
  <section id="framePane" aria-label="pygame frame">
    <img id="f" alt="simulator frame">
  </section>
  <aside id="dashboard" aria-label="simulator status dashboard">
    <section class="card" aria-labelledby="traceHead">
      <h2 id="traceHead">Trace</h2>
      <dl>
        <dt>Name</dt><dd id="traceName">-</dd>
        <dt>Records</dt><dd id="traceRecords">-</dd>
        <dt>Ticks</dt><dd id="traceTicks">-</dd>
        <dt>Validation</dt><dd id="validationStatus">-</dd>
      </dl>
    </section>
    <section class="card" aria-labelledby="replayHead">
      <h2 id="replayHead">Replay</h2>
      <dl>
        <dt>Record</dt><dd id="replayRecord">-</dd>
        <dt>Speed</dt><dd id="replaySpeed">-</dd>
        <dt>Panel</dt><dd id="replayPanel">-</dd>
        <dt>Match</dt><dd id="replayMatch">-</dd>
      </dl>
    </section>
    <section class="card" aria-labelledby="decisionHead">
      <h2 id="decisionHead">Current Decision</h2>
      <dl>
        <dt>Tick</dt><dd id="recordTick">-</dd>
        <dt>Strategy</dt><dd id="recordStrategy">-</dd>
        <dt>Aim</dt><dd id="recordAim">-</dd>
        <dt>Goal</dt><dd id="recordGoal">-</dd>
        <dt>Output</dt><dd id="recordOutput">-</dd>
      </dl>
    </section>
    <section class="card" aria-labelledby="inputsHead">
      <h2 id="inputsHead">Simulator Inputs</h2>
      <dl>
        <dt>Runtime</dt><dd id="inputRuntime">-</dd>
        <dt>Units</dt><dd id="inputUnits">-</dd>
        <dt>Structures</dt><dd id="inputStructures">-</dd>
      </dl>
    </section>
    <section class="card" aria-labelledby="unitsHead">
      <h2 id="unitsHead">Placed Units</h2>
      <ul id="unitList"><li class="empty">No unit state</li></ul>
    </section>
    <section class="card" aria-labelledby="alertsHead">
      <h2 id="alertsHead">Alerts</h2>
      <ul id="alertList"><li class="empty">No alerts</li></ul>
    </section>
  </aside>
</main>
<script>
const img = document.getElementById('f');
const msg = document.getElementById('msg');
const controlsDisabled = {disabled};
const byId = (id) => document.getElementById(id);
function clean(value, fallback='-') {{
  if (value === null || value === undefined || value === '') return fallback;
  if (typeof value === 'number') return Number.isFinite(value) ? String(value) : fallback;
  if (typeof value === 'boolean') return value ? 'true' : 'false';
  return String(value);
}}
function numberText(value, digits=1) {{
  return (typeof value === 'number' && Number.isFinite(value)) ? value.toFixed(digits) : '-';
}}
function pointText(value) {{
  if (Array.isArray(value) && value.length >= 2) return numberText(value[0], 0) + ',' + numberText(value[1], 0);
  if (value && typeof value === 'object' && 'x' in value && 'y' in value) return numberText(value.x, 0) + ',' + numberText(value.y, 0);
  return '-';
}}
function setText(id, value, fallback='-') {{
  byId(id).textContent = clean(value, fallback);
}}
function setPill(id, text, kind='') {{
  const el = byId(id);
  el.className = 'pill' + (kind ? ' ' + kind : '');
  el.textContent = text;
}}
function setList(id, items, emptyText) {{
  const list = byId(id);
  list.textContent = '';
  const values = Array.isArray(items) ? items.filter(Boolean) : [];
  if (values.length === 0) {{
    const item = document.createElement('li');
    item.className = 'empty';
    item.textContent = emptyText;
    list.appendChild(item);
    return;
  }}
  for (const value of values.slice(0, 8)) {{
    const item = document.createElement('li');
    item.textContent = clean(value);
    list.appendChild(item);
  }}
}}
function statusClass(status) {{
  const text = clean(status).toUpperCase();
  if (text === 'PASS') return 'status-pass';
  if (text === 'WARN') return 'status-warn';
  if (text === 'FAIL') return 'status-fail';
  return '';
}}
function unitLine(unit) {{
  const hp = clean(unit.hp) + '/' + clean(unit.max_hp);
  const pos = unit.position_cm ? pointText(unit.position_cm) : '-';
  return clean(unit.side) + ' ' + clean(unit.type) + ' hp ' + hp + ' @ ' + pos;
}}
function updateDashboard(d) {{
  const age = (d.frame_age_sec === null || d.frame_age_sec === undefined) ? '-' : numberText(d.frame_age_sec, 1) + 's';
  setPill('readyPill', d.ready ? 'ready' : 'waiting', d.ready ? 'ready' : 'warn');
  setPill('framePill', 'frame ' + clean(d.frame_id, '0'));
  setPill('agePill', 'age ' + age, d.ready ? '' : 'warn');
  const validationStatusText = d.validation && d.validation.status ? clean(d.validation.status) : '-';
  setPill('validationPill', 'validation ' + validationStatusText, validationStatusText === 'PASS' ? 'ready' : (validationStatusText === 'WARN' ? 'warn' : (validationStatusText === 'FAIL' ? 'bad' : '')));
  const replay = d.replay || {{}};
  setPill('matchPill', 'match ' + clean(replay.match_time_left) + 's' + (replay.match_running ? ' running' : ''));

  const trace = d.trace || {{}};
  setText('traceName', trace.name);
  setText('traceRecords', trace.records);
  const ticks = trace.tick_range ? clean(trace.tick_range.first) + '..' + clean(trace.tick_range.last) : '-';
  setText('traceTicks', ticks);
  const validationEl = byId('validationStatus');
  validationEl.className = statusClass(validationStatusText);
  validationEl.textContent = validationStatusText;

  setText('replayRecord', clean(replay.current_record) + '/' + clean(replay.total_records));
  setText('replaySpeed', clean(replay.speed) + 'x');
  setText('replayPanel', replay.panel_tab);
  setText('replayMatch', clean(replay.match_time_left) + 's ' + (replay.match_running ? 'running' : 'paused'));

  const record = d.current_record || {{}};
  setText('recordTick', record.tick);
  setText('recordStrategy', record.strategy);
  setText('recordAim', record.aim);
  const goal = record.goal || {{}};
  setText('recordGoal', clean(goal.name) + ' #' + clean(goal.id) + ' ' + pointText(goal.pos_cm));
  const output = record.output || {{}};
  setText('recordOutput', clean(output.kind) + ' ' + clean(output.topic || output.final_topic));

  const inputState = d.simulator_inputs && d.simulator_inputs.state ? d.simulator_inputs.state : {{}};
  const runtime = inputState.runtime || {{}};
  setText('inputRuntime', 'hp ' + clean(runtime.self_health) + ' ammo ' + clean(runtime.ammo_left) + ' posture ' + clean(runtime.posture) + ' pos ' + pointText(runtime.self_position_cm));
  const summary = inputState.summary || {{}};
  setText('inputUnits', clean(summary.friend_units, '0') + ' friend / ' + clean(summary.enemy_units, '0') + ' enemy');
  const structures = Array.isArray(inputState.structures) ? inputState.structures : [];
  const damaged = structures.filter((s) => typeof s.hp_ratio === 'number' && s.hp_ratio < 1).map((s) => clean(s.key) + ' ' + clean(s.hp) + '/' + clean(s.max_hp));
  setText('inputStructures', damaged.length ? damaged.slice(0, 3).join(', ') : 'all nominal');
  setList('unitList', (Array.isArray(inputState.units) ? inputState.units : []).map(unitLine), 'No unit state');

  const alerts = [];
  if (validationStatusText === 'WARN' || validationStatusText === 'FAIL') alerts.push('validation ' + validationStatusText);
  if (Array.isArray(summary.destroyed_structures)) alerts.push(...summary.destroyed_structures.map((x) => 'destroyed ' + clean(x)));
  if (Array.isArray(summary.low_hp_units)) alerts.push(...summary.low_hp_units.map((x) => 'low hp ' + clean(x)));
  const issues = d.validation && Array.isArray(d.validation.issues) ? d.validation.issues : [];
  alerts.push(...issues.slice(0, 4).map((issue) => clean(issue.code) + ': ' + clean(issue.message)));
  setList('alertList', alerts, 'No alerts');
}}
function tickFrame() {{
  img.src = '/frame.jpg?t=' + Date.now();
}}
function pollStatus() {{
  fetch('/status.json', {{cache:'no-store'}})
    .then((r) => r.json())
    .then(updateDashboard)
    .catch((e) => {{
      setPill('readyPill', 'error', 'bad');
      msg.textContent = 'status failed: ' + e;
    }});
}}
function postControl(cmd, seconds) {{
  if (controlsDisabled) return;
  const body = (seconds === undefined) ? {{command: cmd}} : {{command: cmd, seconds: Number(seconds)}};
  fetch('/api/control', {{method:'POST', headers:{{'Content-Type':'application/json'}}, body:JSON.stringify(body)}})
    .then((r) => r.json().catch(() => ({{ok:false, message:'bad response'}})))
    .then((d) => {{ msg.textContent = (d && d.ok) ? ('ok: ' + cmd) : ('fail: ' + (d && d.message ? d.message : 'unknown')); }})
    .catch((e) => {{ msg.textContent = 'fail: ' + e; }});
}}
for (const btn of document.querySelectorAll('button[data-cmd]')) {{
  if (controlsDisabled) {{
    btn.disabled = true;
  }} else {{
    btn.addEventListener('click', () => postControl(btn.dataset.cmd, btn.dataset.seconds));
  }}
}}
setInterval(tickFrame, 80);
setInterval(pollStatus, 1000);
tickFrame();
pollStatus();
</script>
</body>
</html>"""
    return body.encode("utf-8")


class SimulatorWebStream:
    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 9000,
        fps: float = 12.0,
        jpeg_quality: int = 80,
        control_file: str = "",
        default_step_sec: int = 10,
        map_path: str = "",
        map_objects: list[dict[str, Any]] | None = None,
    ) -> None:
        fps_value = float(fps)
        if not math.isfinite(fps_value) or fps_value <= 0:
            raise ValueError("fps must be > 0")
        if not (1 <= jpeg_quality <= 100):
            raise ValueError("jpeg_quality must be in [1, 100]")
        self.host = host
        self.port = int(port)
        self.fps = fps_value
        self.frame_interval = 1.0 / fps_value
        self.jpeg_quality = int(jpeg_quality)

        self._httpd: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._frame_jpeg: bytes | None = None
        self._frame_id = 0
        self._last_publish_monotonic = 0.0
        self._last_frame_wall_time = 0.0
        self._last_state_wall_time = 0.0
        self._metadata: dict[str, Any] = {}
        self.control_file: Path | None = None
        if str(control_file).strip():
            self.control_file = Path(control_file).expanduser().resolve()
        self.default_step_sec = max(1, int(default_step_sec))
        self.map_path: Path | None = None
        if str(map_path).strip():
            candidate = Path(map_path).expanduser().resolve()
            if candidate.is_file() and candidate.suffix.lower() in {".png", ".jpg", ".jpeg"}:
                self.map_path = candidate
        self.map_objects = map_objects
        self.tactical_assets = TacticalAssetRegistry.load()

    def start(self) -> None:
        if self._httpd is not None:
            return
        handler_cls = self._build_handler()
        self._httpd = ThreadingHTTPServer((self.host, self.port), handler_cls)
        self.port = int(self._httpd.server_address[1])
        self._thread = threading.Thread(target=self._httpd.serve_forever, name="simulator-web-stream", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        httpd = self._httpd
        thread = self._thread
        self._httpd = None
        self._thread = None
        if httpd is not None:
            httpd.shutdown()
            httpd.server_close()
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)

    def publish_surface(self, surface: Any, pygame: Any) -> None:
        now = time.monotonic()
        if (now - self._last_publish_monotonic) < self.frame_interval:
            return
        self._last_publish_monotonic = now

        width, height = surface.get_size()
        raw_rgb = pygame.image.tostring(surface, "RGB")
        from PIL import Image

        image = Image.frombytes("RGB", (width, height), raw_rgb)
        out = io.BytesIO()
        image.save(out, format="JPEG", quality=self.jpeg_quality, optimize=False)
        payload = out.getvalue()

        with self._lock:
            self._frame_jpeg = payload
            self._frame_id += 1
            self._last_frame_wall_time = time.time()

    def snapshot(self) -> tuple[bytes | None, int, float]:
        with self._lock:
            return self._frame_jpeg, self._frame_id, self._last_frame_wall_time

    def status_snapshot(self) -> dict[str, Any]:
        frame, frame_id, last_frame_time = self.snapshot()
        has_frame = frame is not None
        frame_age_sec = max(0.0, time.time() - last_frame_time) if has_frame and last_frame_time > 0 else None
        with self._lock:
            metadata = dict(self._metadata)
            state_time = self._last_state_wall_time
        has_state = state_time > 0.0
        state_age_sec = max(0.0, time.time() - state_time) if has_state else None
        payload = {
            "ok": True,
            "ready": has_state,
            "has_state": has_state,
            "state_age_sec": state_age_sec,
            "has_frame": has_frame,
            "frame_id": frame_id,
            "last_frame_time": last_frame_time,
            "frame_age_sec": frame_age_sec,
            "control_enabled": self.control_file is not None,
            "control_file": self.control_label(),
            "host": self.host,
            "port": self.port,
            "fps": self.fps,
            "jpeg_quality": self.jpeg_quality,
            "default_step_sec": self.default_step_sec,
        }
        for key, value in metadata.items():
            if key not in payload:
                payload[key] = value
        return payload

    def update_metadata(self, metadata: dict[str, Any]) -> None:
        clean = json_safe(metadata)
        if not isinstance(clean, dict):
            return
        with self._lock:
            self._metadata.update(clean)
            self._last_state_wall_time = time.time()

    def control_label(self) -> str:
        return self.control_file.name if self.control_file is not None else ""

    def submit_control(self, command: str, payload: dict[str, Any] | None = None) -> tuple[bool, str]:
        if self.control_file is None:
            return (False, "control_file not configured")
        try:
            append_command(self.control_file, command, payload)
            return (True, "ok")
        except OSError as exc:
            return (False, str(exc))

    def tactical_state_snapshot(self) -> dict[str, Any]:
        return tactical_state_from_status(self.status_snapshot(), asset_registry=self.tactical_assets)

    def _build_handler(self):
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:  # noqa: N802
                path = urlparse(self.path).path
                if path in ("/", "/index.html"):
                    self._serve_index()
                    return
                if path == "/frame.jpg":
                    self._serve_frame()
                    return
                if path == "/status.json":
                    self._serve_status()
                    return
                if path == "/tactical":
                    self._serve_tactical()
                    return
                if path == "/api/tactical-state":
                    self._serve_tactical_state()
                    return
                if path == "/tactical-map.png":
                    self._serve_tactical_map()
                    return
                if path.startswith("/assets/"):
                    self._serve_asset(path[len("/assets/") :])
                    return
                if path == "/healthz":
                    self._serve_healthz()
                    return
                self.send_error(HTTPStatus.NOT_FOUND, "not found")

            def do_POST(self) -> None:  # noqa: N802
                path = urlparse(self.path).path
                if path == "/api/control":
                    self._serve_control()
                    return
                self.send_error(HTTPStatus.NOT_FOUND, "not found")

            def log_message(self, format: str, *args) -> None:  # noqa: A003
                return

            def _serve_index(self) -> None:
                # The browser Tactical Board is the only production UI.  The
                # optional /frame.jpg endpoint is native-pygame debug output.
                body = build_tactical_html(outer.port, map_objects=outer.map_objects)
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _serve_frame(self) -> None:
                frame, _, _ = outer.snapshot()
                if frame is None:
                    body = b"no frame yet"
                    self.send_response(HTTPStatus.SERVICE_UNAVAILABLE)
                    self.send_header("Content-Type", "text/plain; charset=utf-8")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(frame)))
                self.end_headers()
                self.wfile.write(frame)

            def _serve_status(self) -> None:
                self._json_response(HTTPStatus.OK, outer.status_snapshot())

            def _serve_tactical(self) -> None:
                body = build_tactical_html(outer.port, map_objects=outer.map_objects)
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _serve_tactical_state(self) -> None:
                self._json_response(HTTPStatus.OK, outer.tactical_state_snapshot())

            def _serve_tactical_map(self) -> None:
                map_path = outer.map_path
                if map_path is None:
                    self.send_error(HTTPStatus.NOT_FOUND, "tactical map not configured")
                    return
                try:
                    body = map_path.read_bytes()
                except OSError:
                    self.send_error(HTTPStatus.NOT_FOUND, "tactical map unavailable")
                    return
                content_type = "image/png" if map_path.suffix.lower() == ".png" else "image/jpeg"
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", content_type)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _serve_asset(self, relative_path: str) -> None:
                asset_path, content_type = outer.tactical_assets.resolve_public_path(relative_path)
                if asset_path is None or content_type is None:
                    self.send_error(HTTPStatus.NOT_FOUND, "asset not found")
                    return
                try:
                    body = asset_path.read_bytes()
                except OSError:
                    self.send_error(HTTPStatus.NOT_FOUND, "asset unavailable")
                    return
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", content_type)
                self.send_header("Cache-Control", "public, max-age=300")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _serve_healthz(self) -> None:
                status = outer.status_snapshot()
                ready = bool(status.get("ready"))
                status["ok"] = ready
                self._json_response(HTTPStatus.OK if ready else HTTPStatus.SERVICE_UNAVAILABLE, status)

            def _serve_control(self) -> None:
                raw_len = self.headers.get("Content-Length", "0").strip()
                try:
                    length = max(0, int(raw_len))
                except ValueError:
                    length = 0
                body = self.rfile.read(length) if length > 0 else b"{}"
                try:
                    data = json.loads(body.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    self._json_response(HTTPStatus.BAD_REQUEST, {"ok": False, "message": "invalid json"})
                    return
                if not isinstance(data, dict):
                    self._json_response(HTTPStatus.BAD_REQUEST, {"ok": False, "message": "payload must be object"})
                    return

                command, payload, error = normalize_api_control_payload(data, outer.default_step_sec)
                if error is not None or command is None:
                    self._json_response(HTTPStatus.BAD_REQUEST, {"ok": False, "message": error or "invalid command"})
                    return

                tactical_state = outer.tactical_state_snapshot()
                scene = tactical_state["scene"]
                if is_scene_mutation(command) and not scene.get("can_edit", False):
                    self._json_response(
                        HTTPStatus.CONFLICT,
                        {
                            "ok": False,
                            "message": scene.get("edit_reason", "scene_edit_unavailable"),
                            "command": command,
                            "payload": payload,
                        },
                    )
                    return

                ok, message = outer.submit_control(command, payload or None)
                status = HTTPStatus.OK if ok else HTTPStatus.SERVICE_UNAVAILABLE
                self._json_response(
                    status,
                    {
                        "ok": ok,
                        "message": message,
                        "command": command,
                        "payload": payload,
                        "control_enabled": outer.control_file is not None,
                        "control_file": outer.control_label(),
                    },
                )

            def _json_response(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
                body = json.dumps(json_safe(payload), ensure_ascii=True, allow_nan=False, separators=(",", ":")).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        return Handler
