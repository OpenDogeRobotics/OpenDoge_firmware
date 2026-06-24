#!/usr/bin/env python3
"""IMU 陀螺仪 Web 3D 可视化工具

实时展示 DM-IMU-L1 的姿态数据：
- 3D 方块实时跟随 IMU 姿态旋转
- 角速度 (wx, wy, wz) 柱状图
- 重力投影 (gx, gy, gz) 数值显示

用法:
  # 从 IMU 状态文件读取（配合 dm_imu_bridge.py 使用）
  ./tools/imu/imu_webviz.py --source file --file /tmp/opendoge_imu.state --port 8080

  # 直接从串口读取 IMU
  ./tools/imu/imu_webviz.py --source serial --device /dev/ttyACM0 --baud 921600 --port 8080

然后在浏览器打开 http://<orange-pi-ip>:8080
"""

from __future__ import annotations

import argparse
import json
import math
import os
import struct
import sys
import termios
import threading
import time
import tty
from http.server import HTTPServer, BaseHTTPRequestHandler
from typing import Optional, Tuple

# ─── IMU 协议常量 ──────────────────────────────────────────

QUAT_CAN_MIN, QUAT_CAN_MAX = -1.0, 1.0
CRC_POLY = 0x1021

# ─── HTML 页面 ─────────────────────────────────────────────

HTML_PAGE = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>IMU 陀螺仪 3D 可视化</title>
<style>
  :root { --bg: #1a1a2e; --panel: #16213e; --accent: #0f3460; --text: #e0e0e0; }
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body { font-family: 'Segoe UI', system-ui, sans-serif; background: var(--bg); color: var(--text);
         display: flex; height: 100vh; overflow: hidden; }
  /* 3D 视图 */
  #view3d { flex: 1; position: relative; cursor: grab; }
  #view3d:active { cursor: grabbing; }
  #view3d canvas { display: block; }
  #status-line { position: absolute; top: 12px; left: 16px; font-size: 13px;
                 background: rgba(0,0,0,0.55); padding: 6px 12px; border-radius: 6px; }
  #status-line.ok { color: #4caf50; }
  #status-line.stale { color: #ff9800; }
  #status-line.error { color: #f44336; }
  /* 右侧面板 */
  #panel { width: 320px; background: var(--panel); padding: 16px;
           display: flex; flex-direction: column; gap: 16px; overflow-y: auto; }
  h2 { font-size: 16px; font-weight: 600; border-bottom: 1px solid #333; padding-bottom: 6px; }
  .card { background: var(--accent); border-radius: 10px; padding: 14px; }
  .card h3 { font-size: 13px; color: #90caf9; margin-bottom: 10px; text-transform: uppercase;
             letter-spacing: 1px; }
  .row { display: flex; justify-content: space-between; align-items: center; padding: 3px 0; }
  .label { font-size: 12px; color: #aaa; width: 24px; font-weight: bold; }
  .value { font-size: 18px; font-family: 'JetBrains Mono', 'Cascadia Code', monospace;
            font-weight: 600; text-align: right; width: 90px; }
  .bar-wrap { flex: 1; height: 8px; background: #1a1a2e; border-radius: 4px; margin: 0 8px;
              overflow: hidden; position: relative; }
  .bar-fill { height: 100%; border-radius: 4px; transition: width 0.05s linear; position: absolute;
              left: 50%; top: 0; }
  .bar-fill.positive { background: linear-gradient(90deg, #4caf50, #8bc34a); }
  .bar-fill.negative { background: linear-gradient(270deg, #f44336, #ff5722); }
  .dot { display: inline-block; width: 10px; height: 10px; border-radius: 50%; margin-right: 6px; }
  .dot.x { background: #f44336; } .dot.y { background: #4caf50; } .dot.z { background: #2196f3; }
  .legend { font-size: 12px; color: #aaa; margin-bottom: 8px; display: flex; gap: 16px; }
  .axis-hint { position: absolute; bottom: 16px; left: 16px; font-size: 11px; color: #666;
               background: rgba(0,0,0,0.45); padding: 4px 8px; border-radius: 4px; }
  @media (max-width: 768px) {
    body { flex-direction: column; }
    #panel { width: 100%; max-height: 45vh; }
  }
</style>
</head>
<body>

<div id="view3d">
  <div id="status-line" class="error">● 等待 IMU 数据...</div>
  <div class="axis-hint">🖱 拖拽旋转 · 滚轮缩放 · 右键平移</div>
</div>

<div id="panel">
  <h2>📡 IMU 陀螺仪实时数据</h2>

  <div class="card">
    <h3>🔄 角速度 Gyro (rad/s)</h3>
    <div class="legend"><span><span class="dot x"></span> X</span><span><span class="dot y"></span> Y</span><span><span class="dot z"></span> Z</span></div>
    <div class="row"><span class="label" style="color:#f44336">wx</span><div class="bar-wrap"><div class="bar-fill positive" id="bar-wx" style="width:0%"></div></div><span class="value" id="val-wx">0.000</span></div>
    <div class="row"><span class="label" style="color:#4caf50">wy</span><div class="bar-wrap"><div class="bar-fill positive" id="bar-wy" style="width:0%"></div></div><span class="value" id="val-wy">0.000</span></div>
    <div class="row"><span class="label" style="color:#2196f3">wz</span><div class="bar-wrap"><div class="bar-fill positive" id="bar-wz" style="width:0%"></div></div><span class="value" id="val-wz">0.000</span></div>
  </div>

  <div class="card">
    <h3>🧲 重力投影 Gravity</h3>
    <div class="legend"><span><span class="dot x"></span> X</span><span><span class="dot y"></span> Y</span><span><span class="dot z"></span> Z</span></div>
    <div class="row"><span class="label" style="color:#f44336">gx</span><div class="bar-wrap"><div class="bar-fill positive" id="bar-gx" style="width:0%"></div></div><span class="value" id="val-gx">0.000</span></div>
    <div class="row"><span class="label" style="color:#4caf50">gy</span><div class="bar-wrap"><div class="bar-fill positive" id="bar-gy" style="width:0%"></div></div><span class="value" id="val-gy">0.000</span></div>
    <div class="row"><span class="label" style="color:#2196f3">gz</span><div class="bar-wrap"><div class="bar-fill positive" id="bar-gz" style="width:0%"></div></div><span class="value" id="val-gz">0.000</span></div>
  </div>

  <div class="card" id="quat-card" style="display:none">
    <h3>🔮 四元数 Quaternion</h3>
    <div class="row"><span class="label">w</span><span class="value" id="val-qw">0.000</span></div>
    <div class="row"><span class="label">x</span><span class="value" id="val-qx">0.000</span></div>
    <div class="row"><span class="label">y</span><span class="value" id="val-qy">0.000</span></div>
    <div class="row"><span class="label">z</span><span class="value" id="val-qz">0.000</span></div>
  </div>

  <div style="font-size:11px;color:#555;text-align:center;margin-top:auto;">
    FPS: <span id="fps">0</span> · 延迟: <span id="latency">0</span>ms
  </div>
</div>

<script type="importmap">
{
  "imports": {
    "three": "https://unpkg.com/three@0.160.0/build/three.module.js",
    "three/addons/": "https://unpkg.com/three@0.160.0/examples/jsm/"
  }
}
</script>

<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ─── 场景初始化 ────────────────────────────────
const container = document.getElementById('view3d');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a2e);
scene.fog = new THREE.Fog(0x1a1a2e, 3, 15);

const camera = new THREE.PerspectiveCamera(45, container.clientWidth / container.clientHeight, 0.1, 50);
camera.position.set(2.5, 1.8, 3.5);
camera.lookAt(0, 0, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(container.clientWidth, container.clientHeight);
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
container.appendChild(renderer.domElement);

// OrbitControls
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.minDistance = 1.2;
controls.maxDistance = 8;
controls.update();

// ─── 光照 ───────────────────────────────────────
const ambient = new THREE.AmbientLight(0x404060, 1.5);
scene.add(ambient);
const key = new THREE.DirectionalLight(0xffffff, 2.5);
key.position.set(5, 8, 5);
key.castShadow = true;
key.shadow.mapSize.set(512, 512);
key.shadow.camera.near = 0.5;
key.shadow.camera.far = 30;
key.shadow.camera.left = -5;
key.shadow.camera.right = 5;
key.shadow.camera.top = 5;
key.shadow.camera.bottom = -5;
scene.add(key);
const rim = new THREE.DirectionalLight(0x4488ff, 1.2);
rim.position.set(-3, 1, -3);
scene.add(rim);

// ─── 地面参考网格 ──────────────────────────────
const grid = new THREE.GridHelper(3, 20, 0x333355, 0x222244);
grid.position.y = -0.8;
scene.add(grid);

// 世界坐标轴指示 (细线)
function addWorldAxis(dir, color, length = 0.7) {
  const geo = new THREE.CylinderGeometry(0.015, 0.015, length, 8);
  const mat = new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.5 });
  const mesh = new THREE.Mesh(geo, mat);
  if (dir.x !== undefined) { // X axis
    mesh.rotation.z = -Math.PI / 2;
    mesh.position.set(length / 2, -0.75, 0);
  } else if (dir.y !== undefined) { // Y axis
    mesh.position.set(0, -0.75 + length / 2, 0);
  } else { // Z axis
    mesh.rotation.x = Math.PI / 2;
    mesh.position.set(0, -0.75, length / 2);
  }
  return mesh;
}
scene.add(addWorldAxis({ x: 1 }, 0xff4444));
scene.add(addWorldAxis({ y: 1 }, 0x44ff44));
scene.add(addWorldAxis({ z: 1 }, 0x4444ff));

// ─── IMU 3D 模型 ────────────────────────────────
const imuGroup = new THREE.Group();
scene.add(imuGroup);

// 主体方块 (带圆角效果用多个面表示)
const bodyGeo = new THREE.BoxGeometry(0.7, 0.2, 1.0, 2, 2, 2);
const bodyMat = new THREE.MeshStandardMaterial({
  color: 0x3a3a5c,
  roughness: 0.25,
  metalness: 0.6,
});
const body = new THREE.Mesh(bodyGeo, bodyMat);
body.castShadow = true;
body.receiveShadow = true;
imuGroup.add(body);

// 顶部装饰条
const stripeGeo = new THREE.BoxGeometry(0.5, 0.06, 0.8);
const stripeMat = new THREE.MeshStandardMaterial({ color: 0x5c6bc0, roughness: 0.2, metalness: 0.4 });
const stripe = new THREE.Mesh(stripeGeo, stripeMat);
stripe.position.y = 0.13;
imuGroup.add(stripe);

// 局部坐标轴
function addAxis(rotationAxis, angle, color, length = 0.75, radius = 0.02) {
  const geo = new THREE.CylinderGeometry(radius, radius, length, 8);
  const mat = new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.6, roughness: 0.3 });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.rotation.set(rotationAxis[0], rotationAxis[1], rotationAxis[2]);
  // 箭头锥体
  const coneGeo = new THREE.ConeGeometry(radius * 2.5, length * 0.18, 8);
  const cone = new THREE.Mesh(coneGeo, mat);
  cone.position.set(0, length / 2, 0);
  mesh.add(cone);
  return mesh;
}
// X 轴 (红) — 绕 Z 轴旋转 -PI/2 使默认 Y 轴指向 X
imuGroup.add(addAxis([0, 0, -Math.PI / 2], Math.PI / 2, 0xff4444));
// Y 轴 (绿)
imuGroup.add(addAxis([0, 0, 0], 0, 0x44ff44));
// Z 轴 (蓝) — 绕 X 轴旋转 PI/2
imuGroup.add(addAxis([Math.PI / 2, 0, 0], Math.PI / 2, 0x4488ff));

// ─── 陀螺仪指示环 ──────────────────────────────
const ringGroup = new THREE.Group();
imuGroup.add(ringGroup);
function createRing(rotation, color) {
  const torusGeo = new THREE.TorusGeometry(0.55, 0.015, 8, 48);
  const torusMat = new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.3, roughness: 0.4 });
  const torus = new THREE.Mesh(torusGeo, torusMat);
  torus.rotation.set(rotation[0], rotation[1], rotation[2]);
  return torus;
}
ringGroup.add(createRing([0, 0, 0], 0xff4444));           // X 环 (YZ 平面)
ringGroup.add(createRing([Math.PI / 2, 0, 0], 0x44ff44));  // Y 环 (XZ 平面)
ringGroup.add(createRing([0, 0, Math.PI / 2], 0x4488ff));  // Z 环 (XY 平面)

// ─── 重力方向指示线 ────────────────────────────
const gravityArrow = new THREE.Group();
scene.add(gravityArrow);
const arrowGeo = new THREE.CylinderGeometry(0.02, 0.02, 1.0, 8);
const arrowMat = new THREE.MeshStandardMaterial({ color: 0xffaa00, emissive: 0xff8800, emissiveIntensity: 0.6 });
const arrowShaft = new THREE.Mesh(arrowGeo, arrowMat);
arrowShaft.position.y = 0.5;
gravityArrow.add(arrowShaft);
const arrowHeadGeo = new THREE.ConeGeometry(0.06, 0.18, 8);
const arrowHead = new THREE.Mesh(arrowHeadGeo, arrowMat);
arrowHead.position.y = 1.0;
gravityArrow.add(arrowHead);

// ─── 窗口自适应 ────────────────────────────────
window.addEventListener('resize', () => {
  camera.aspect = container.clientWidth / container.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(container.clientWidth, container.clientHeight);
});

// ─── 数据元素映射 ──────────────────────────────
const els = {
  wx: document.getElementById('val-wx'), wy: document.getElementById('val-wy'), wz: document.getElementById('val-wz'),
  gx: document.getElementById('val-gx'), gy: document.getElementById('val-gy'), gz: document.getElementById('val-gz'),
  qw: document.getElementById('val-qw'), qx: document.getElementById('val-qx'), qy: document.getElementById('val-qy'), qz: document.getElementById('val-qz'),
  bar_wx: document.getElementById('bar-wx'), bar_wy: document.getElementById('bar-wy'), bar_wz: document.getElementById('bar-wz'),
  bar_gx: document.getElementById('bar-gx'), bar_gy: document.getElementById('bar-gy'), bar_gz: document.getElementById('bar-gz'),
};
const statusLine = document.getElementById('status-line');
const quatCard = document.getElementById('quat-card');

// ─── 重力方向指针 (世界空间中) ──────────────────
function updateGravityArrow(gx, gy, gz) {
  const len = Math.sqrt(gx * gx + gy * gy + gz * gz);
  if (len < 0.01) { gravityArrow.visible = false; return; }
  gravityArrow.visible = true;
  const nx = gx / len, ny = gy / len, nz = gz / len;
  // 箭头从原点指向重力方向
  const arrowLen = len * 1.2;
  arrowShaft.scale.y = arrowLen;
  arrowShaft.position.y = arrowLen / 2;
  arrowHead.position.y = arrowLen;
  // 默认箭头指向 +Y，转向 (nx, ny, nz)
  const dir = new THREE.Vector3(nx, ny, nz);
  const up = new THREE.Vector3(0, 1, 0);
  const quat = new THREE.Quaternion().setFromUnitVectors(up, dir);
  gravityArrow.quaternion.copy(quat);
}

// ─── IMU 模型旋转 ───────────────────────────────
// 重力向量在 IMU 局部坐标系中为 (gx, gy, gz)
// 世界参考重力指向 (0, -1, 0) 即 Y 轴负方向
// 将 IMU 旋转使得其局部重力方向与世界重力方向对齐
function updateImuRotation(gx, gy, gz) {
  const len = Math.sqrt(gx * gx + gy * gy + gz * gz);
  if (len < 0.01) return;
  const localDown = new THREE.Vector3(gx, gy, gz).normalize();
  const worldDown = new THREE.Vector3(0, -1, 0);

  const quat = new THREE.Quaternion().setFromUnitVectors(localDown, worldDown);
  imuGroup.quaternion.slerp(quat, 0.35);  // 平滑过渡
}

// ─── 更新条形图 ────────────────────────────────
const GYRO_MAX = 6.0;   // rad/s 满量程
const GRAV_MAX = 1.2;
function updateBar(el, val, maxVal) {
  const pct = Math.min(Math.abs(val) / maxVal * 100, 100);
  el.style.width = pct + '%';
  el.className = 'bar-fill ' + (val >= 0 ? 'positive' : 'negative');
  if (val < 0) el.style.left = (50 - pct) + '%';
  else el.style.left = '50%';
}

let lastDataTime = 0;
let frameCount = 0;
let fpsTime = performance.now();

// ─── 数据轮询 ───────────────────────────────────
async function poll() {
  frameCount++;
  const now = performance.now();
  if (now - fpsTime >= 1000) {
    document.getElementById('fps').textContent = Math.round(frameCount / ((now - fpsTime) / 1000));
    frameCount = 0;
    fpsTime = now;
  }
  const t0 = performance.now();
  try {
    const resp = await fetch('/data');
    const d = await resp.json();
    const lat = (performance.now() - t0).toFixed(0);
    document.getElementById('latency').textContent = lat;

    if (!d.valid) {
      statusLine.textContent = '● 等待 IMU 数据...';
      statusLine.className = 'error';
    } else {
      const age = d.age_ms || 0;
      if (age > 200) {
        statusLine.textContent = `● 数据延迟 ${age.toFixed(0)}ms`;
        statusLine.className = 'stale';
      } else {
        statusLine.textContent = `● IMU 连接正常 (${d.source})`;
        statusLine.className = 'ok';
      }
      lastDataTime = now;
    }

    // 角速度
    const wx = d.wx || 0, wy = d.wy || 0, wz = d.wz || 0;
    els.wx.textContent = wx.toFixed(4); els.wy.textContent = wy.toFixed(4); els.wz.textContent = wz.toFixed(4);
    updateBar(els.bar_wx, wx, GYRO_MAX); updateBar(els.bar_wy, wy, GYRO_MAX); updateBar(els.bar_wz, wz, GYRO_MAX);
    // 旋转指示环 (按角速度缩放变色)
    ringGroup.children.forEach((ring, i) => {
      const val = [wx, wy, wz][i];
      const intensity = Math.min(Math.abs(val) / GYRO_MAX, 1);
      ring.material.emissiveIntensity = 0.3 + intensity * 1.0;
      ring.scale.setScalar(1 + intensity * 0.15);
    });

    // 重力
    const gx = d.gx || 0, gy = d.gy || 0, gz = d.gz || 0;
    els.gx.textContent = gx.toFixed(4); els.gy.textContent = gy.toFixed(4); els.gz.textContent = gz.toFixed(4);
    updateBar(els.bar_gx, gx, GRAV_MAX); updateBar(els.bar_gy, gy, GRAV_MAX); updateBar(els.bar_gz, gz, GRAV_MAX);
    updateImuRotation(gx, gy, gz);
    updateGravityArrow(gx, gy, gz);

    // 四元数
    if (d.qw !== undefined) {
      quatCard.style.display = 'block';
      els.qw.textContent = d.qw.toFixed(4); els.qx.textContent = d.qx.toFixed(4);
      els.qy.textContent = d.qy.toFixed(4); els.qz.textContent = d.qz.toFixed(4);
    }
  } catch (e) {
    statusLine.textContent = '● 无法连接服务器';
    statusLine.className = 'error';
  }
}

// ─── 渲染循环 ───────────────────────────────────
function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}
animate();
setInterval(poll, 30); // ~30 Hz 轮询
</script>
</body>
</html>"""

# ─── IMU 数据读取 ────────────────────────────────────────────

class ImuData:
    __slots__ = ("wx", "wy", "wz", "gx", "gy", "gz",
                 "qw", "qx", "qy", "qz",
                 "valid", "timestamp", "source_name")
    def __init__(self):
        self.wx = self.wy = self.wz = 0.0
        self.gx = self.gy = self.gz = 0.0
        self.qw = self.qx = self.qy = self.qz = 0.0
        self.valid = False
        self.timestamp = 0.0
        self.source_name = "none"


class SerialImuReader:
    """直接从 DM-IMU-L1 串口读取，解析协议帧。"""

    def __init__(self, device: str, baud: int):
        self.device = device
        self.baud = baud
        self.fd: Optional[int] = None
        self.buffer = bytearray()

    def open(self) -> None:
        self.fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        attrs = termios.tcgetattr(self.fd)
        speed = getattr(termios, f"B{self.baud}", None)
        if speed is None:
            raise ValueError(f"unsupported baud: {self.baud}")
        attrs[4] = attrs[5] = speed
        attrs[2] |= termios.CLOCAL | termios.CREAD
        attrs[2] &= ~(termios.CSTOPB | termios.PARENB | termios.CSIZE)
        attrs[2] |= termios.CS8
        attrs[3] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def read(self, data: ImuData) -> None:
        if self.fd is None:
            return
        try:
            chunk = os.read(self.fd, 4096)
        except BlockingIOError:
            chunk = b""
        if chunk:
            self.buffer.extend(chunk)
        self._parse_frames(data)

    def _parse_frames(self, data: ImuData) -> None:
        buf = self.buffer
        while True:
            start = buf.find(b"\x55\xAA")
            if start < 0:
                buf.clear()
                return
            if start > 0:
                del buf[:start]
            if len(buf) < 5:
                return
            frame_type = buf[3]
            payload_len = 16 if frame_type == 0x04 else 12 if frame_type in (0x01, 0x02, 0x03) else None
            if payload_len is None:
                del buf[0]
                continue
            frame_len = 2 + 1 + 1 + payload_len + 2 + 1
            if len(buf) < frame_len:
                return
            if buf[frame_len - 1] != 0x0A:
                del buf[0]
                continue
            payload = bytes(buf[4: 4 + payload_len])
            del buf[:frame_len]
            self._apply(data, frame_type, payload)

    def _apply(self, data: ImuData, frame_type: int, payload: bytes) -> None:
        count = payload_len = len(payload)
        if count % 4 != 0:
            return
        values = struct.unpack("<" + "f" * (count // 4), payload)
        if frame_type == 0x02 and len(values) >= 3:
            data.wx, data.wy, data.wz = values[0], values[1], values[2]
            data.timestamp = time.monotonic()
        elif frame_type == 0x04 and len(values) >= 4:
            data.qw, data.qx, data.qy, data.qz = values[0], values[1], values[2], values[3]
            # 从四元数计算重力投影
            gravity = _quat_rotate_inverse((data.qw, data.qx, data.qy, data.qz), (0.0, 0.0, -1.0))
            data.gx, data.gy, data.gz = gravity
            data.valid = True
            data.timestamp = time.monotonic()


class FileImuReader:
    """从 dm_imu_bridge.py 输出的状态文件读取。"""

    def __init__(self, path: str):
        self.path = path

    def read(self, data: ImuData) -> None:
        try:
            with open(self.path, "r") as f:
                content = f.read()
        except (FileNotFoundError, PermissionError):
            return
        for line in content.splitlines():
            line = line.strip()
            if "=" not in line:
                continue
            key, _, val = line.partition("=")
            try:
                v = float(val.strip())
            except ValueError:
                continue
            key = key.strip()
            if key == "wx":
                data.wx = v
            elif key == "wy":
                data.wy = v
            elif key == "wz":
                data.wz = v
            elif key == "gx":
                data.gx = v
            elif key == "gy":
                data.gy = v
            elif key == "gz":
                data.gz = v
        if data.gz != 0.0 or data.gx != 0.0 or data.gy != 0.0:
            data.valid = True
        data.timestamp = time.monotonic()


def _quat_rotate_inverse(q, v):
    """四元数逆旋转：将世界系向量变换到 IMU 局部系。"""
    w, x, y, z = q
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm > 1e-9:
        w, x, y, z = w / norm, x / norm, y / norm, z / norm
    qv = (x, y, z)
    dot = qv[0] * v[0] + qv[1] * v[1] + qv[2] * v[2]
    cross = (
        qv[1] * v[2] - qv[2] * v[1],
        qv[2] * v[0] - qv[0] * v[2],
        qv[0] * v[1] - qv[1] * v[0],
    )
    scale = 2.0 * w * w - 1.0
    return (
        v[0] * scale - 2.0 * w * cross[0] + 2.0 * dot * qv[0],
        v[1] * scale - 2.0 * w * cross[1] + 2.0 * dot * qv[1],
        v[2] * scale - 2.0 * w * cross[2] + 2.0 * dot * qv[2],
    )


# ─── HTTP 服务器 ────────────────────────────────────────────

_shared_data = ImuData()
_shared_lock = threading.Lock()


class ImuHTTPHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # 静默模式

    def do_GET(self):
        if self.path == "/":
            self._serve_html()
        elif self.path == "/data":
            self._serve_json()
        else:
            self.send_response(404)
            self.end_headers()

    def _serve_html(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(HTML_PAGE.encode("utf-8"))

    def _serve_json(self):
        with _shared_lock:
            d = _shared_data
            age_ms = (time.monotonic() - d.timestamp) * 1000.0 if d.timestamp else float("inf")
            payload = {
                "wx": d.wx, "wy": d.wy, "wz": d.wz,
                "gx": d.gx, "gy": d.gy, "gz": d.gz,
                "qw": d.qw, "qx": d.qx, "qy": d.qy, "qz": d.qz,
                "valid": d.valid,
                "age_ms": age_ms,
                "source": d.source_name,
            }
        body = json.dumps(payload, ensure_ascii=False)
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body.encode("utf-8"))


def reader_thread(reader, interval_s: float):
    """后台线程：持续读取 IMU 数据到 _shared_data。"""
    global _shared_data
    while True:
        reader.read(_shared_data)
        time.sleep(interval_s)


def main():
    parser = argparse.ArgumentParser(description="IMU 陀螺仪 Web 3D 可视化")
    parser.add_argument("--source", choices=["file", "serial"], default="file",
                        help="数据来源: file (从状态文件) 或 serial (直接从串口)")
    parser.add_argument("--file", default="/tmp/opendoge_imu.state",
                        help="IMU 状态文件路径 (--source file 时使用)")
    parser.add_argument("--device", default="/dev/ttyACM0",
                        help="串口设备路径 (--source serial 时使用)")
    parser.add_argument("--baud", type=int, default=921600,
                        help="串口波特率")
    parser.add_argument("--port", type=int, default=8080,
                        help="Web 服务器端口 (默认 8080)")
    parser.add_argument("--host", default="0.0.0.0",
                        help="绑定地址 (默认 0.0.0.0)")
    args = parser.parse_args()

    # 创建读取器
    if args.source == "serial":
        reader = SerialImuReader(args.device, args.baud)
        reader.open()
        _shared_data.source_name = f"serial:{args.device}"
        print(f"📡 IMU 串口: {args.device} @ {args.baud} baud")
    else:
        reader = FileImuReader(args.file)
        _shared_data.source_name = f"file:{args.file}"
        print(f"📄 IMU 状态文件: {args.file}")

    # 启动读取线程
    poll_s = 0.002 if args.source == "serial" else 0.02
    t = threading.Thread(target=reader_thread, args=(reader, poll_s), daemon=True)
    t.start()

    # 启动 HTTP 服务器
    server = HTTPServer((args.host, args.port), ImuHTTPHandler)
    print(f"\n🌐 Web 可视化: http://localhost:{args.port}")
    print(f"   按 Ctrl+C 停止\n")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n停止服务器...")
        server.shutdown()
        if args.source == "serial":
            reader.close()


if __name__ == "__main__":
    main()
