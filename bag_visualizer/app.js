const ROBOT_COLORS = {
  root: '#e0a952',
  robot1: '#52b8e0',
};
const FORCE_COLORS = { avoid: '#e05252', quark: '#5588e0', dir: '#4caf50', net: '#222222' };
const TRAIL_MAX = 400;
// Force magnitudes are in whatever units the controller's k_* gains produce,
// not meters, so a fixed px-per-unit scale is either invisible or absurd
// depending on tuning. Instead pick a world-space (meters) length for the
// single largest force vector in the dataset and scale everything else
// relative to that, so arrows are always visible at the default fit-to-view
// zoom and shrink/grow together with the map when the user zooms.
let forceWorldScale = 1;

const canvas = document.getElementById('stage');
const ctx = canvas.getContext('2d');
const playBtn = document.getElementById('play-btn');
const scrubber = document.getElementById('scrubber');
const timeLabel = document.getElementById('time-label');
const speedSelect = document.getElementById('speed-select');
const dataUrlInput = document.getElementById('data-url');
const reloadBtn = document.getElementById('reload-btn');
const metaInfo = document.getElementById('meta-info');
const robotInfo = document.getElementById('robot-info');

const toggles = {
  field: document.getElementById('toggle-field'),
  peers: document.getElementById('toggle-peers'),
  trails: document.getElementById('toggle-trails'),
  forces: document.getElementById('toggle-forces'),
};

let dataset = null;
let frameIdx = 0;
let playing = false;
let lastTickMs = null;
let view = { scale: 1, offsetX: 0, offsetY: 0 }; // world meters -> screen px
let trails = {};
let fieldCanvas = null;

function resizeCanvas() {
  // Measure the canvas's own flexbox-computed CSS size (not the parent's,
  // which includes the side panel) - setting style.width from that would
  // pin the canvas wide and collapse the panel out of the flex row.
  const rect = canvas.getBoundingClientRect();
  canvas.width = rect.width * devicePixelRatio;
  canvas.height = rect.height * devicePixelRatio;
  ctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
  if (dataset) fitView();
  render();
}
window.addEventListener('resize', resizeCanvas);

async function loadData(url) {
  const res = await fetch(url, { cache: 'no-store' });
  if (!res.ok) throw new Error(`Failed to load ${url}: ${res.status}`);
  dataset = await res.json();
  frameIdx = 0;
  trails = {};
  for (const r of dataset.meta.robots) trails[r] = [];

  scrubber.max = String(dataset.frames.length - 1);
  scrubber.value = '0';
  metaInfo.textContent = `origin ${dataset.meta.origin_lat?.toFixed(6)}, ${dataset.meta.origin_lon?.toFixed(6)} · ${dataset.frames.length} frames @ ${(1 / dataset.meta.dt).toFixed(1)} Hz · ${dataset.meta.duration.toFixed(1)}s`;

  if (dataset.field) bakeFieldCanvas();
  computeForceScale();
  fitView();
  render();
}

function computeForceScale() {
  let maxMag = 0;
  for (const frame of dataset.frames) {
    for (const r of dataset.meta.robots) {
      const rob = frame.robots[r];
      if (!rob) continue;
      for (const vec of Object.values(rob.forces)) {
        const mag = Math.hypot(vec[0], vec[1]);
        if (mag > maxMag) maxMag = mag;
      }
    }
  }
  const { minX, maxX, minY, maxY } = worldExtent();
  const targetMeters = Math.max(maxX - minX, maxY - minY) * 0.15;
  forceWorldScale = maxMag > 1e-9 ? targetMeters / maxMag : 1;
}

function worldExtent() {
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  const consider = (x, y) => {
    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
  };
  if (dataset.field) {
    for (const row of dataset.field.x) for (const v of row) consider(v, 0);
    for (const row of dataset.field.y) for (const v of row) consider(0, v);
  }
  for (const frame of dataset.frames) {
    for (const r of dataset.meta.robots) {
      const rob = frame.robots[r];
      if (rob && rob.pos) consider(rob.pos[0], rob.pos[1]);
    }
  }
  if (!isFinite(minX)) { minX = -10; maxX = 10; minY = -10; maxY = 10; }
  const pad = Math.max(5, (maxX - minX) * 0.1, (maxY - minY) * 0.1);
  return { minX: minX - pad, maxX: maxX + pad, minY: minY - pad, maxY: maxY + pad };
}

function fitView() {
  const { minX, maxX, minY, maxY } = worldExtent();
  const rect = canvas.getBoundingClientRect();
  const w = rect.width, h = rect.height;
  const scaleX = w / (maxX - minX);
  const scaleY = h / (maxY - minY);
  view.scale = Math.min(scaleX, scaleY);
  view.offsetX = w / 2 - ((minX + maxX) / 2) * view.scale;
  view.offsetY = h / 2 + ((minY + maxY) / 2) * view.scale; // y flips (north = up)
}

function toScreen(x, y) {
  return [x * view.scale + view.offsetX, -y * view.scale + view.offsetY];
}

function bakeFieldCanvas() {
  const { x, y, z } = dataset.field;
  const rows = z.length, cols = z[0].length;
  let zmin = Infinity, zmax = -Infinity;
  for (const row of z) for (const v of row) { if (v < zmin) zmin = v; if (v > zmax) zmax = v; }

  const off = document.createElement('canvas');
  off.width = cols;
  off.height = rows;
  const octx = off.getContext('2d');
  const img = octx.createImageData(cols, rows);
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      const t = (z[r][c] - zmin) / (zmax - zmin + 1e-9);
      const [rr, gg, bb] = coolwarm(t);
      const idx = (r * cols + c) * 4;
      img.data[idx] = rr; img.data[idx + 1] = gg; img.data[idx + 2] = bb; img.data[idx + 3] = 200;
    }
  }
  octx.putImageData(img, 0, 0);
  fieldCanvas = { canvas: off, x, y, rows, cols };
}

function coolwarm(t) {
  // simple blue -> white -> red diverging map
  if (t < 0.5) {
    const u = t / 0.5;
    return [lerp(59, 245, u), lerp(76, 245, u), lerp(192, 245, u)];
  }
  const u = (t - 0.5) / 0.5;
  return [lerp(245, 200, u), lerp(245, 60, u), lerp(245, 60, u)];
}
function lerp(a, b, u) { return a + (b - a) * u; }

function drawGrid() {
  const rect = canvas.getBoundingClientRect();
  ctx.save();
  ctx.strokeStyle = 'rgba(128,128,128,0.15)';
  ctx.lineWidth = 1;
  const stepWorld = niceStep((rect.width / view.scale) / 10);
  const { minX, maxX, minY, maxY } = worldExtent();
  for (let gx = Math.floor(minX / stepWorld) * stepWorld; gx <= maxX; gx += stepWorld) {
    const [sx] = toScreen(gx, 0);
    ctx.beginPath(); ctx.moveTo(sx, 0); ctx.lineTo(sx, rect.height); ctx.stroke();
  }
  for (let gy = Math.floor(minY / stepWorld) * stepWorld; gy <= maxY; gy += stepWorld) {
    const [, sy] = toScreen(0, gy);
    ctx.beginPath(); ctx.moveTo(0, sy); ctx.lineTo(rect.width, sy); ctx.stroke();
  }
  ctx.restore();
}
function niceStep(target) {
  const pow10 = Math.pow(10, Math.floor(Math.log10(Math.max(target, 1e-6))));
  const candidates = [1, 2, 5, 10].map(m => m * pow10);
  return candidates.reduce((a, b) => (Math.abs(b - target) < Math.abs(a - target) ? b : a));
}

function drawField() {
  if (!fieldCanvas || !toggles.field.checked) return;
  const { x, y, rows, cols } = fieldCanvas;
  const x0 = x[0][0], x1 = x[0][cols - 1];
  const y0 = y[0][0], y1 = y[rows - 1][0];
  const [sx0, sy0] = toScreen(x0, y0);
  const [sx1, sy1] = toScreen(x1, y1);
  ctx.save();
  ctx.imageSmoothingEnabled = true;
  ctx.drawImage(fieldCanvas.canvas, Math.min(sx0, sx1), Math.min(sy0, sy1), Math.abs(sx1 - sx0), Math.abs(sy1 - sy0));
  ctx.restore();
}

function drawRobot(name, rob) {
  if (!rob || !rob.pos) return;
  const color = ROBOT_COLORS[name] || '#999';
  const [sx, sy] = toScreen(rob.pos[0], rob.pos[1]);
  const yaw = (rob.heading_deg || 0) * Math.PI / 180;

  // trail
  if (toggles.trails.checked && trails[name] && trails[name].length > 1) {
    ctx.save();
    ctx.strokeStyle = color;
    ctx.globalAlpha = 0.5;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    trails[name].forEach(([wx, wy], i) => {
      const [px, py] = toScreen(wx, wy);
      if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
    });
    ctx.stroke();
    ctx.restore();
  }

  // peer detections (absolute, already rotated into map frame)
  if (toggles.peers.checked) {
    ctx.save();
    ctx.fillStyle = color;
    ctx.globalAlpha = 0.5;
    for (const [px, py] of rob.peers) {
      const [sx2, sy2] = toScreen(px, py);
      ctx.beginPath();
      ctx.arc(sx2, sy2, 4, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.restore();
  }

  // force vectors - drawn in world space (meters) so they scale with the
  // map instead of staying a fixed pixel size regardless of zoom.
  if (toggles.forces.checked) {
    for (const [key, vec] of Object.entries(rob.forces)) {
      const [fx, fy] = vec;
      const mag = Math.hypot(fx, fy);
      if (mag < 1e-6) continue;
      const [ex, ey] = toScreen(rob.pos[0] + fx * forceWorldScale, rob.pos[1] + fy * forceWorldScale);
      drawArrow(sx, sy, ex, ey, FORCE_COLORS[key], key === 'net' ? 2.5 : 1.5);
    }
  }

  // robot icon: triangle pointing along heading (compass: 0=N up, clockwise)
  ctx.save();
  ctx.translate(sx, sy);
  ctx.rotate(yaw);
  ctx.fillStyle = color;
  ctx.strokeStyle = 'rgba(0,0,0,0.4)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(0, -10);
  ctx.lineTo(6, 8);
  ctx.lineTo(-6, 8);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();

  ctx.save();
  ctx.fillStyle = color;
  ctx.font = '11px -apple-system, sans-serif';
  ctx.fillText(name, sx + 10, sy - 10);
  ctx.restore();
}

function drawArrow(x0, y0, x1, y1, color, width) {
  ctx.save();
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = width;
  ctx.beginPath();
  ctx.moveTo(x0, y0);
  ctx.lineTo(x1, y1);
  ctx.stroke();
  const angle = Math.atan2(y1 - y0, x1 - x0);
  const headLen = 5 + width;
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x1 - headLen * Math.cos(angle - Math.PI / 6), y1 - headLen * Math.sin(angle - Math.PI / 6));
  ctx.lineTo(x1 - headLen * Math.cos(angle + Math.PI / 6), y1 - headLen * Math.sin(angle + Math.PI / 6));
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function updateRobotInfo(frame) {
  robotInfo.innerHTML = '';
  for (const name of dataset.meta.robots) {
    const rob = frame.robots[name];
    const block = document.createElement('div');
    block.className = 'robot-block';
    if (!rob) {
      block.innerHTML = `<div class="robot-name">${name}</div><div>no data yet</div>`;
    } else {
      const [x, y] = rob.pos;
      block.innerHTML = `<div class="robot-name">${name}</div>` +
        `x=${x.toFixed(2)} y=${y.toFixed(2)}<br>` +
        `heading=${rob.heading_deg.toFixed(1)}°<br>` +
        `peers=${rob.peers.length}<br>` +
        `net force=${Math.hypot(...rob.forces.net).toFixed(2)}`;
    }
    robotInfo.appendChild(block);
  }
}

function render() {
  const rect = canvas.getBoundingClientRect();
  ctx.clearRect(0, 0, rect.width, rect.height);
  if (!dataset || !dataset.frames.length) return;

  const frame = dataset.frames[frameIdx];

  drawField();
  drawGrid();

  for (const name of dataset.meta.robots) {
    const rob = frame.robots[name];
    if (rob && rob.pos) {
      const t = trails[name];
      const last = t[t.length - 1];
      if (!last || last[0] !== rob.pos[0] || last[1] !== rob.pos[1]) {
        t.push(rob.pos);
        if (t.length > TRAIL_MAX) t.shift();
      }
    }
  }

  for (const name of dataset.meta.robots) drawRobot(name, frame.robots[name]);

  timeLabel.textContent = `${frame.t.toFixed(1)} / ${dataset.meta.duration.toFixed(1)} s`;
  scrubber.value = String(frameIdx);
  updateRobotInfo(frame);
}

function tick(nowMs) {
  if (!playing) return;
  if (lastTickMs == null) lastTickMs = nowMs;
  const dtMs = nowMs - lastTickMs;
  const speed = parseFloat(speedSelect.value);
  const frameDtMs = dataset.meta.dt * 1000 / speed;
  if (dtMs >= frameDtMs) {
    lastTickMs = nowMs;
    frameIdx += 1;
    if (frameIdx >= dataset.frames.length) {
      frameIdx = dataset.frames.length - 1;
      playing = false;
      playBtn.textContent = 'Play';
    }
    render();
  }
  if (playing) requestAnimationFrame(tick);
}

playBtn.addEventListener('click', () => {
  if (!dataset) return;
  playing = !playing;
  playBtn.textContent = playing ? 'Pause' : 'Play';
  if (playing) {
    if (frameIdx >= dataset.frames.length - 1) frameIdx = 0;
    lastTickMs = null;
    requestAnimationFrame(tick);
  }
});

scrubber.addEventListener('input', () => {
  if (!dataset) return;
  playing = false;
  playBtn.textContent = 'Play';
  frameIdx = parseInt(scrubber.value, 10);
  render();
});

reloadBtn.addEventListener('click', () => {
  playing = false;
  playBtn.textContent = 'Play';
  loadData(dataUrlInput.value).catch(err => alert(err.message));
});

resizeCanvas();
loadData(dataUrlInput.value).catch(err => console.warn('No data loaded yet:', err.message));
