#!/usr/bin/env python3
"""OpenDoge web dashboard — real-time monitoring and command console.

Reads the deploy binary's --status-file (JSON) and the command/IMU state
files, serves a single-page dashboard with live joint telemetry, IMU
visualisation, and bidirectional command controls.

Usage:
  python3 tools/web/opendoge_web.py
  python3 tools/web/opendoge_web.py --port 8080 --status-file /tmp/opendoge_status.json

Dependencies: Python 3.9+ standard library only (no pip installs).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import traceback
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlparse, parse_qs

# ── defaults ────────────────────────────────────────────────────────
DEFAULT_PORT = 8080
DEFAULT_STATUS_FILE = "/tmp/opendoge_status.json"
DEFAULT_COMMAND_FILE = "/tmp/opendoge_command.state"
DEFAULT_IMU_FILE = "/tmp/opendoge_imu.state"

STATUS_FILE = DEFAULT_STATUS_FILE
COMMAND_FILE = DEFAULT_COMMAND_FILE
IMU_FILE = DEFAULT_IMU_FILE

# ── file I/O helpers ─────────────────────────────────────────────────

def _read_kv(path: str) -> dict[str, str]:
    """Read a key=value file into a dict."""
    result: dict[str, str] = {}
    try:
        with open(path) as f:
            for line in f:
                line = line.split("#", 1)[0].strip()
                if "=" not in line:
                    continue
                k, v = line.split("=", 1)
                result[k.strip()] = v.strip()
    except FileNotFoundError:
        pass
    return result


def _atomic_write(path: str, text: str) -> None:
    tmp = f"{path}.tmp.{os.getpid()}"
    with open(tmp, "w") as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


# ── HTTP handler ─────────────────────────────────────────────────────

class DashboardHandler(BaseHTTPRequestHandler):
    """Serve the dashboard SPA and REST API endpoints."""

    def log_message(self, fmt, *args):
        # Suppress default stderr logging; we log to stdout compactly.
        pass

    def do_GET(self):
        path = urlparse(self.path).path

        if path == "/" or path == "/index.html":
            self._serve_html()
        elif path == "/api/status":
            self._serve_json(self._get_status())
        elif path == "/api/command":
            self._serve_json(self._get_command())
        elif path == "/api/imu":
            self._serve_json(self._get_imu())
        else:
            self.send_error(404)

    def do_POST(self):
        path = urlparse(self.path).path
        length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(length) if length else b""
        try:
            data = json.loads(body) if body else {}
        except json.JSONDecodeError:
            self._serve_json({"ok": False, "error": "invalid JSON"})
            return

        if path == "/api/command":
            self._handle_command_post(data)
        else:
            self.send_error(404)

    # ── API handlers ─────────────────────────────────────────────

    def _get_status(self) -> dict:
        try:
            with open(STATUS_FILE) as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            return {"state": "no_status", "joints": [], "imu": {}}

    def _get_command(self) -> dict:
        kv = _read_kv(COMMAND_FILE)
        return {
            "vx": float(kv.get("vx", 0)),
            "vy": float(kv.get("vy", 0)),
            "yaw_rate": float(kv.get("yaw_rate", 0)),
            "active": kv.get("active", "false") in ("true", "1"),
            "estop": kv.get("estop", "false") in ("true", "1"),
            "clear_fault": kv.get("clear_fault", "false") in ("true", "1"),
            "low_gain_mode": kv.get("low_gain_mode", "false") in ("true", "1"),
        }

    def _get_imu(self) -> dict:
        kv = _read_kv(IMU_FILE)
        try:
            return {
                "wx": float(kv.get("wx", 0)),
                "wy": float(kv.get("wy", 0)),
                "wz": float(kv.get("wz", 0)),
                "gx": float(kv.get("gx", 0)),
                "gy": float(kv.get("gy", 0)),
                "gz": float(kv.get("gz", -1)),
            }
        except (ValueError, KeyError):
            return {"wx": 0, "wy": 0, "wz": 0, "gx": 0, "gy": 0, "gz": -1}

    def _handle_command_post(self, data: dict) -> None:
        """Write updated command fields to the command file."""
        current = self._get_command()
        for key in ("vx", "vy", "yaw_rate", "active", "estop", "clear_fault", "low_gain_mode"):
            if key in data:
                current[key] = data[key]

        lines = [
            f"vx={float(current['vx']):.6f}",
            f"vy={float(current['vy']):.6f}",
            f"yaw_rate={float(current['yaw_rate']):.6f}",
            f"active={'true' if current['active'] else 'false'}",
            f"estop={'true' if current['estop'] else 'false'}",
            f"clear_fault={'true' if current['clear_fault'] else 'false'}",
            f"low_gain_mode={'true' if current['low_gain_mode'] else 'false'}",
        ]
        _atomic_write(COMMAND_FILE, "\n".join(lines) + "\n")
        self._serve_json({"ok": True})

    # ── response helpers ─────────────────────────────────────────

    def _serve_json(self, data: dict):
        body = json.dumps(data).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_html(self):
        html = DASHBOARD_HTML.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(html)))
        self.end_headers()
        self.wfile.write(html)


# ── dashboard HTML / CSS / JS (single-page app) ──────────────────────

DASHBOARD_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OpenDoge Dashboard</title>
<style>
:root {
  --bg: #0d1117; --card: #161b22; --border: #30363d;
  --text: #c9d1d9; --dim: #8b949e; --accent: #58a6ff;
  --green: #3fb950; --yellow: #d2991d; --red: #f85149; --blue: #58a6ff;
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
  background: var(--bg); color: var(--text);
}
* { box-sizing: border-box; margin: 0; padding: 0; }
body { padding: 16px; max-width: 1200px; margin: 0 auto; }
h1 { font-size: 1.3em; margin-bottom: 12px; }
h2 { font-size: 1.0em; color: var(--dim); margin: 16px 0 8px; text-transform: uppercase; letter-spacing: .05em; }

/* header bar */
.bar {
  display: flex; gap: 12px; align-items: center; flex-wrap: wrap;
  padding: 10px 14px; background: var(--card); border: 1px solid var(--border);
  border-radius: 8px; margin-bottom: 12px;
}
.bar .state { font-weight: 700; font-size: 1.1em; padding: 2px 10px; border-radius: 4px; }
.state-active, .state-low_gain_test { background: #1a3a1a; color: var(--green); }
.state-ready { background: #1a2a1a; color: var(--yellow); }
.state-damping_fault { background: #3a1a1a; color: var(--red); }
.state-wait_feedback { background: #1a1a3a; color: var(--blue); }
.state-no_status { color: var(--dim); }
.bar .info { font-size: 0.85em; color: var(--dim); }
.bar .fault { color: var(--red); font-weight: 600; max-width: 400px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }

/* grid layout */
.grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
@media (max-width: 800px) { .grid { grid-template-columns: 1fr; } }
.card {
  background: var(--card); border: 1px solid var(--border);
  border-radius: 8px; padding: 10px 14px;
}

/* joint table */
table { width: 100%; border-collapse: collapse; font-size: 0.82em; font-variant-numeric: tabular-nums; }
th, td { padding: 3px 6px; text-align: right; white-space: nowrap; }
th { color: var(--dim); font-weight: 600; text-transform: uppercase; font-size: 0.75em; }
td:first-child, th:first-child { text-align: left; }
td.warn { color: var(--yellow); }
td.danger { color: var(--red); font-weight: 700; }
td.good { color: var(--green); }

/* command panel */
.cmd-panel { display: flex; gap: 12px; flex-wrap: wrap; align-items: flex-end; }
.cmd-group { display: flex; flex-direction: column; gap: 2px; }
.cmd-group label { font-size: 0.75em; color: var(--dim); text-transform: uppercase; }
.cmd-group input[type=range] { width: 120px; accent-color: var(--accent); }
.cmd-group .val { font-size: 0.8em; text-align: center; }
.btn-row { display: flex; gap: 6px; flex-wrap: wrap; margin-top: 8px; }
.btn {
  padding: 5px 14px; border: 1px solid var(--border); border-radius: 4px;
  background: var(--card); color: var(--text); cursor: pointer;
  font-size: 0.82em; user-select: none;
}
.btn:hover { background: #21262d; }
.btn.on { background: var(--accent); color: #000; border-color: var(--accent); }
.btn.estop { border-color: var(--red); color: var(--red); }
.btn.estop.on { background: var(--red); color: #fff; }

/* IMU bars */
.imu-bar { display: flex; gap: 6px; align-items: center; margin: 2px 0; font-size: 0.82em; }
.imu-bar .label { width: 30px; color: var(--dim); text-align: right; }
.imu-bar .fill {
  height: 14px; border-radius: 3px; min-width: 2px;
  transition: width 0.3s, background 0.3s;
}
.imu-bar .num { width: 50px; text-align: right; font-variant-numeric: tabular-nums; }

.footer { margin-top: 12px; font-size: 0.72em; color: var(--dim); text-align: center; }
</style>
</head>
<body>
<h1>🐕 OpenDoge Dashboard</h1>

<!-- state bar -->
<div class="bar" id="bar">
  <span class="state" id="state-badge">---</span>
  <span class="info" id="bar-info"></span>
  <span class="fault" id="bar-fault"></span>
</div>

<div class="grid">
  <!-- joints -->
  <div class="card">
    <h2>Joints</h2>
    <table>
      <thead><tr>
        <th>Joint</th><th>q rad</th><th>dq rad/s</th><th>&tau; Nm</th><th>Temp °C</th><th>Fault</th>
      </tr></thead>
      <tbody id="joint-tbody"></tbody>
    </table>
  </div>

  <!-- IMU -->
  <div class="card">
    <h2>IMU</h2>
    <div style="margin-bottom:8px;font-size:0.82em"><b>Angular velocity</b> (rad/s)</div>
    <div id="imu-gyro"></div>
    <div style="margin:10px 0 6px;font-size:0.82em"><b>Projected gravity</b> (body frame)</div>
    <div id="imu-grav"></div>
    <div style="margin-top:10px;font-size:0.82em;color:var(--dim)" id="imu-valid"></div>
  </div>

  <!-- commands -->
  <div class="card" style="grid-column: 1 / -1">
    <h2>Command</h2>
    <div class="cmd-panel">
      <div class="cmd-group">
        <label>vx (m/s)</label>
        <input type="range" id="cmd-vx" min="-0.8" max="0.8" step="0.01" value="0">
        <span class="val" id="val-vx">0.00</span>
      </div>
      <div class="cmd-group">
        <label>vy (m/s)</label>
        <input type="range" id="cmd-vy" min="-0.6" max="0.6" step="0.01" value="0">
        <span class="val" id="val-vy">0.00</span>
      </div>
      <div class="cmd-group">
        <label>yaw (rad/s)</label>
        <input type="range" id="cmd-yaw" min="-1.5" max="1.5" step="0.01" value="0">
        <span class="val" id="val-yaw">0.00</span>
      </div>
    </div>
    <div class="btn-row">
      <button class="btn" id="btn-active">Active</button>
      <button class="btn" id="btn-lowgain">Low Gain</button>
      <button class="btn" id="btn-clear">Clear Fault</button>
      <button class="btn estop" id="btn-estop">E‑STOP</button>
      <span style="margin-left:12px;font-size:0.78em;color:var(--dim)" id="cmd-status"></span>
    </div>
  </div>

  <!-- CAN stats -->
  <div class="card" style="grid-column: 1 / -1">
    <h2>Loop &amp; CAN</h2>
    <div style="display:flex;gap:20px;flex-wrap:wrap;font-size:0.85em" id="can-stats"></div>
  </div>
</div>

<div class="footer">
  OpenDoge Web Console &mdash; polling every 500 ms
  &mdash; <span id="poll-status">●</span>
</div>

<script>
const POLL_MS = 500;
let cmdActive = false, cmdEstop = false, cmdLowGain = false, cmdClear = false;

function $ (id) { return document.getElementById(id); }

function stateClass (s) {
  if (!s) return 'state-no_status';
  if (s === 'active') return 'state-active';
  if (s === 'damping_fault') return 'state-damping_fault';
  if (s === 'low_gain_test') return 'state-low_gain_test';
  if (s === 'ready') return 'state-ready';
  if (s === 'wait_feedback') return 'state-wait_feedback';
  return 'state-no_status';
}

function tempClass (t) {
  if (t >= 80) return 'danger';
  if (t >= 65) return 'warn';
  return '';
}

function barColor (val, scale) {
  const v = Math.abs(val) / (scale || 1);
  const h = Math.max(0, 120 - v * 80);
  return `hsl(${h}, 70%, 50%)`;
}

function renderImuBar (container, vals, labels, scale) {
  let html = '';
  for (let i = 0; i < 3; i++) {
    const v = vals[i] || 0;
    const pct = Math.min(100, Math.abs(v) / scale * 100);
    html += `<div class="imu-bar">
      <span class="label">${labels[i]}</span>
      <span class="fill" style="width:${pct}%;background:${barColor(v, scale)}"></span>
      <span class="num">${v.toFixed(3)}</span>
    </div>`;
  }
  container.innerHTML = html;
}

async function update () {
  try {
    const st = await (await fetch('/api/status')).json();
    const cmd = await (await fetch('/api/command')).json();

    // State bar
    const badge = $('state-badge');
    badge.textContent = st.state || 'no_status';
    badge.className = 'state ' + stateClass(st.state);

    $('bar-info').textContent =
      `active=${st.active_cmd} estop=${st.estop} low_gain=${st.low_gain} imu=${st.imu_valid} | ` +
      `ctrl=${st.ctrl_ticks}Hz infer=${st.infer_ticks}Hz late=${st.max_late_us}us missed=${st.missed_ctrl}`;

    const faultDiv = $('bar-fault');
    faultDiv.textContent = st.fault_reason ? 'FAULT: ' + st.fault_reason : '';

    // Joints
    const joints = st.joints || [];
    let jhtml = '';
    for (const j of joints) {
      const tc = tempClass(j.temp);
      jhtml += `<tr>
        <td>${j.n}</td>
        <td>${(j.q||0).toFixed(4)}</td>
        <td>${(j.dq||0).toFixed(3)}</td>
        <td>${(j.tau||0).toFixed(3)}</td>
        <td class="${tc}">${(j.temp||0).toFixed(1)}</td>
        <td>${j.fault ? '⚡' : '✓'}</td>
      </tr>`;
    }
    $('joint-tbody').innerHTML = jhtml;

    // IMU
    const imu = st.imu || {};
    renderImuBar($('imu-gyro'), [imu.wx, imu.wy, imu.wz], ['x','y','z'], 3.0);
    renderImuBar($('imu-grav'), [imu.gx, imu.gy, imu.gz], ['x','y','z'], 1.0);
    $('imu-valid').textContent = st.imu_valid ? 'IMU valid' : 'IMU not available';

    // CAN stats
    $('can-stats').innerHTML =
      `<span>CAN TX: <b>${st.can_tx||0}</b>/s</span>` +
      `<span>CAN RX: <b>${st.can_rx||0}</b>/s</span>` +
      `<span>CAN err: <b>${st.can_err||0}</b></span>` +
      `<span>Time: <b>${(st.t||0).toFixed(1)}s</b></span>` +
      `<span>Cmd: [${(st.command||[0,0,0]).map(v=>v.toFixed(2)).join(', ')}]</span>`;

    // Sync command panel from file
    cmdActive = cmd.active;
    cmdEstop = cmd.estop;
    cmdLowGain = cmd.low_gain_mode;
    cmdClear = cmd.clear_fault;
    syncButtons();

    // Update slider values from file (don't fight joystick)
    $('val-vx').textContent = cmd.vx.toFixed(2);
    $('val-vy').textContent = cmd.vy.toFixed(2);
    $('val-yaw').textContent = cmd.yaw_rate.toFixed(2);

    $('poll-status').textContent = '●';
    $('poll-status').style.color = '#3fb950';
  } catch (e) {
    $('poll-status').textContent = '●';
    $('poll-status').style.color = '#f85149';
    console.error(e);
  }
}

function syncButtons () {
  const a = $('btn-active'); a.textContent = cmdActive ? 'Active ✓' : 'Active';
  a.className = 'btn' + (cmdActive ? ' on' : '');

  const l = $('btn-lowgain'); l.textContent = cmdLowGain ? 'Low Gain ✓' : 'Low Gain';
  l.className = 'btn' + (cmdLowGain ? ' on' : '');

  const e = $('btn-estop'); e.textContent = cmdEstop ? 'E‑STOP ⚡' : 'E‑STOP';
  e.className = 'btn estop' + (cmdEstop ? ' on' : '');
}

async function postCommand (fields) {
  try {
    const resp = await fetch('/api/command', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(fields),
    });
    const data = await resp.json();
    $('cmd-status').textContent = data.ok ? 'OK' : 'ERR';
    $('cmd-status').style.color = data.ok ? '#3fb950' : '#f85149';
    setTimeout(() => { $('cmd-status').textContent = ''; }, 1500);
  } catch (e) {
    $('cmd-status').textContent = 'NET ERR';
    $('cmd-status').style.color = '#f85149';
  }
}

// Slider → POST
['vx','vy','yaw'].forEach((axis, i) => {
  const ids = ['cmd-vx','cmd-vy','cmd-yaw'];
  const keys = ['vx','vy','yaw_rate'];
  const el = $(ids[i]);
  el.addEventListener('input', () => {
    $(`val-${axis}`).textContent = parseFloat(el.value).toFixed(2);
  });
  el.addEventListener('change', () => {
    postCommand({ [keys[i]]: parseFloat(el.value) });
  });
});

// Buttons
$('btn-active').addEventListener('click', () => {
  if (cmdEstop) return;
  postCommand({ active: !cmdActive, low_gain_mode: false });
});
$('btn-lowgain').addEventListener('click', () => {
  if (cmdEstop) return;
  postCommand({ low_gain_mode: !cmdLowGain, active: false });
});
$('btn-clear').addEventListener('click', () => {
  postCommand({ clear_fault: true });
});
$('btn-estop').addEventListener('click', () => {
  postCommand({ estop: !cmdEstop, active: false, low_gain_mode: false });
});

// Keyboard shortcuts
document.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT') return;
  switch (e.key.toLowerCase()) {
    case 'a': $('btn-active').click(); break;
    case 'l': $('btn-lowgain').click(); break;
    case 'c': $('btn-clear').click(); break;
    case 'escape': $('btn-estop').click(); break;
  }
});

// Kick off polling
setInterval(update, POLL_MS);
update();
</script>
</body>
</html>"""


# ── main ─────────────────────────────────────────────────────────────

def main() -> int:
    global STATUS_FILE, COMMAND_FILE, IMU_FILE

    p = argparse.ArgumentParser(description="OpenDoge web dashboard")
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--status-file", default=DEFAULT_STATUS_FILE)
    p.add_argument("--command-file", default=DEFAULT_COMMAND_FILE)
    p.add_argument("--imu-file", default=DEFAULT_IMU_FILE)
    p.add_argument("--bind", default="0.0.0.0")
    args = p.parse_args()

    STATUS_FILE = args.status_file
    COMMAND_FILE = args.command_file
    IMU_FILE = args.imu_file

    # Verify status file is reachable
    if not os.path.exists(STATUS_FILE):
        print(f"Note: status file not yet created: {STATUS_FILE}")
        print(f"      It will appear once the deploy binary writes its first snapshot.")

    server = HTTPServer((args.bind, args.port), DashboardHandler)
    print(f"OpenDoge web dashboard: http://localhost:{args.port}")
    print(f"  status-file:  {STATUS_FILE}")
    print(f"  command-file: {COMMAND_FILE}")
    print(f"  imu-file:     {IMU_FILE}")
    print(f"  Press Ctrl+C to stop.")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down.")
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
