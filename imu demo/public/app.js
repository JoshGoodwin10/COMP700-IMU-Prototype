/* ============================================================
   IMU Biomechanics Lab — app.js
   Supports:
     - Real ESP32 via Web Serial API (Chrome/Edge, USB)
     - Simulation fallback when no device connected
   ============================================================ */

'use strict';

// ── State ──────────────────────────────────────────────────────────────────────
const state = {
  connected: false,
  recording: false,
  simMode: false,
  serialPort: null,
  serialReader: null,
  samples: [],
  sessionStart: null,
  assignedPart: null,
  simInterval: null,
  durationInterval: null,
  peakA: 0,
  t: 0,        // simulation time
  // Current orientation
  roll: 0, pitch: 0, yaw: 0,
  ax: 0, ay: 0, az: 1,
  // Sim targets (smooth random walk)
  rollTarget: 0, pitchTarget: 0, yawTarget: 0,
};

// Ring buffer for the chart (200 samples × 3 channels)
const CHART_LEN = 200;
const rollBuf = new Float32Array(CHART_LEN);
const pitchBuf = new Float32Array(CHART_LEN);
const yawBuf = new Float32Array(CHART_LEN);
let bufIdx = 0;

// ── Body part definitions ──────────────────────────────────────────────────────
const BODY_PARTS = [
  'L. Wrist', 'R. Wrist',
  'L. Elbow', 'R. Elbow',
  'L. Bicep', 'R. Bicep',
  'Chest', 'Waist',
  'L. Thigh', 'R. Thigh',
  'L. Knee', 'R. Knee',
  'L. Ankle', 'R. Ankle',
];

// Skeleton joint positions on the 240×340 canvas
const JOINTS = {
  head: [120, 28],
  neck: [120, 52],
  lsho: [88, 72], rsho: [152, 72],
  lbic: [78, 92], rbic: [162, 92],
  lelb: [70, 112], relb: [170, 112],
  lwri: [60, 148], rwri: [180, 148],
  chest: [120, 92],
  waist: [120, 132],
  lhip: [100, 152], rhip: [140, 152],
  lthy: [97, 176], rthy: [143, 176],
  lkne: [94, 206], rkne: [146, 206],
  lank: [91, 250], rank: [149, 250],
  lfoot: [82, 268], rfoot: [158, 268],
};

const BONES = [
  ['head', 'neck'],
  ['neck', 'lsho'], ['neck', 'rsho'],
  ['lsho', 'lbic'], ['rsho', 'rbic'],
  ['lbic', 'lelb'], ['rbic', 'relb'],
  ['lelb', 'lwri'], ['relb', 'rwri'],
  ['neck', 'chest'], ['chest', 'waist'],
  ['waist', 'lhip'], ['waist', 'rhip'],
  ['lhip', 'lthy'], ['rhip', 'rthy'],
  ['lthy', 'lkne'], ['rthy', 'rkne'],
  ['lkne', 'lank'], ['rkne', 'rank'],
  ['lank', 'lfoot'], ['rank', 'rfoot'],
];

const PART_TO_JOINT = {
  'L. Wrist': 'lwri', 'R. Wrist': 'rwri',
  'L. Elbow': 'lelb', 'R. Elbow': 'relb',
  'L. Bicep': 'lbic', 'R. Bicep': 'rbic',
  'Chest': 'chest', 'Waist': 'waist',
  'L. Thigh': 'lthy', 'R. Thigh': 'rthy',
  'L. Knee': 'lkne', 'R. Knee': 'rkne',
  'L. Ankle': 'lank', 'R. Ankle': 'rank',
};

// ── Build assignment chip grid ──────────────────────────────────────────────────
const assignGrid = document.getElementById('assign-grid');
BODY_PARTS.forEach(part => {
  const chip = document.createElement('div');
  chip.className = 'assign-chip';
  chip.textContent = part;
  chip.addEventListener('click', () => {
    document.querySelectorAll('.assign-chip').forEach(c => c.classList.remove('active'));
    chip.classList.add('active');
    state.assignedPart = part;
    document.getElementById('cube-label').textContent = 'IMU — ' + part;
    drawSkeleton();
  });
  assignGrid.appendChild(chip);
});

// ── Three.js setup ─────────────────────────────────────────────────────────────
const threeCanvas = document.getElementById('three-canvas');
const cubePanel = document.getElementById('cube-panel');

const renderer = new THREE.WebGLRenderer({ canvas: threeCanvas, antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setClearColor(0x000000, 0);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(38, 1, 0.1, 100);
camera.position.set(2.4, 1.6, 3.0);
camera.lookAt(0, 0, 0);

scene.add(new THREE.AmbientLight(0xffffff, 0.5));
const dirLight = new THREE.DirectionalLight(0xffffff, 0.95);
dirLight.position.set(3, 4, 3);
scene.add(dirLight);

// PCB board group
const imuGroup = new THREE.Group();

// Board body (green PCB)
const boardGeo = new THREE.BoxGeometry(1.9, 0.16, 1.25);
const boardMat = new THREE.MeshPhongMaterial({ color: 0x1a4a2e, shininess: 55 });
imuGroup.add(new THREE.Mesh(boardGeo, boardMat));

// IC chip
const chipGeo = new THREE.BoxGeometry(0.48, 0.09, 0.48);
const chipMat = new THREE.MeshPhongMaterial({ color: 0x1a1a1a });
const chip = new THREE.Mesh(chipGeo, chipMat);
chip.position.set(0, 0.125, 0);
imuGroup.add(chip);

// Connector strip (gold pins)
const pinGeo = new THREE.BoxGeometry(1.0, 0.12, 0.09);
const pinMat = new THREE.MeshPhongMaterial({ color: 0xc8a84b });
const pins = new THREE.Mesh(pinGeo, pinMat);
pins.position.set(0, 0.1, -0.56);
imuGroup.add(pins);

// Axis arrows (R=X, G=Y, B=Z)
const AXIS_COLORS = [0xE24B4A, 0x1D9E75, 0x378ADD];
const AXIS_DIRS = [
  new THREE.Vector3(1, 0, 0),
  new THREE.Vector3(0, 1, 0),
  new THREE.Vector3(0, 0, 1),
];
AXIS_DIRS.forEach((dir, i) => {
  const arrow = new THREE.ArrowHelper(dir, new THREE.Vector3(0, 0.1, 0), 1.15, AXIS_COLORS[i], 0.14, 0.07);
  imuGroup.add(arrow);
});

scene.add(imuGroup);

// Reference grid
const grid = new THREE.GridHelper(4, 8, 0xaaaaaa, 0xdddddd);
grid.position.y = -0.55;
scene.add(grid);

// Resize handler for Three canvas
function resizeThree() {
  const W = cubePanel.clientWidth;
  const H = cubePanel.clientHeight;
  if (!W || !H) return;
  renderer.setSize(W, H, false);
  camera.aspect = W / H;
  camera.updateProjectionMatrix();
}

// Animate Three.js
const euler = new THREE.Euler(0, 0, 0, 'XYZ');
function animateThree() {
  requestAnimationFrame(animateThree);
  resizeThree();
  euler.set(
    state.pitch * Math.PI / 180,
    state.yaw * Math.PI / 180,
    state.roll * Math.PI / 180,
    'XYZ'
  );
  imuGroup.setRotationFromEuler(euler);
  renderer.render(scene, camera);
}
animateThree();

// ── Skeleton canvas ─────────────────────────────────────────────────────────────
const skelCanvas = document.getElementById('skel-canvas');
const skelCtx = skelCanvas.getContext('2d');
const darkMode = () => window.matchMedia('(prefers-color-scheme: dark)').matches;

// Defines which bones belong to each body part, and the pivot joint
// (the joint that stays fixed while the distal end rotates).
const LIMB_CHAINS = {
  'L. Wrist': { pivot: 'lelb', bones: [['lelb', 'lwri']] },
  'R. Wrist': { pivot: 'relb', bones: [['relb', 'rwri']] },
  'L. Elbow': { pivot: 'lsho', bones: [['lsho', 'lbic'], ['lbic', 'lelb'], ['lelb', 'lwri']] },
  'R. Elbow': { pivot: 'rsho', bones: [['rsho', 'rbic'], ['rbic', 'relb'], ['relb', 'rwri']] },
  'L. Bicep': { pivot: 'lsho', bones: [['lsho', 'lbic'], ['lbic', 'lelb'], ['lelb', 'lwri']] },
  'R. Bicep': { pivot: 'rsho', bones: [['rsho', 'rbic'], ['rbic', 'relb'], ['relb', 'rwri']] },
  'Chest': { pivot: 'waist', bones: [['waist', 'chest'], ['chest', 'neck']] },
  'Waist': { pivot: 'chest', bones: [['chest', 'waist'], ['waist', 'lhip'], ['waist', 'rhip']] },
  'L. Thigh': { pivot: 'lhip', bones: [['lhip', 'lthy'], ['lthy', 'lkne'], ['lkne', 'lank'], ['lank', 'lfoot']] },
  'R. Thigh': { pivot: 'rhip', bones: [['rhip', 'rthy'], ['rthy', 'rkne'], ['rkne', 'rank'], ['rank', 'rfoot']] },
  'L. Knee': { pivot: 'lkne', bones: [['lkne', 'lank'], ['lank', 'lfoot']] },
  'R. Knee': { pivot: 'rkne', bones: [['rkne', 'rank'], ['rank', 'rfoot']] },
  'L. Ankle': { pivot: 'lank', bones: [['lank', 'lfoot']] },
  'R. Ankle': { pivot: 'rank', bones: [['rank', 'rfoot']] },
};

function drawSkeleton() {
  const ctx  = skelCtx;
  const W    = skelCanvas.width;
  const H    = skelCanvas.height;
  const dark = darkMode();
  ctx.clearRect(0, 0, W, H);
 
  const boneClr      = dark ? 'rgba(200,200,200,0.28)' : 'rgba(60,60,60,0.2)';
  const activeBoneClr = '#185FA5';
  const jointClr     = dark ? 'rgba(190,190,190,0.5)'  : 'rgba(50,50,50,0.4)';
  const activeJoint  = state.assignedPart ? PART_TO_JOINT[state.assignedPart] : null;
  const limbChain    = state.assignedPart ? LIMB_CHAINS[state.assignedPart]   : null;
 
  // Build a set of bone keys that belong to the active limb chain
  const activeBoneSet = new Set();
  if (limbChain) {
    limbChain.bones.forEach(([a, b]) => activeBoneSet.add(a + '|' + b));
  }
 
  // ── Compute animated joint positions ──────────────────────────
  // Start from the base JOINTS positions, then rotate the active
  // limb chain around its pivot using pitch (forward/back) and
  // roll (side tilt) from the IMU.
  const joints = {};
  Object.entries(JOINTS).forEach(([k, v]) => { joints[k] = [v[0], v[1]]; });
 
  if (limbChain && state.connected) {
    const pivotKey = limbChain.pivot;
    const pivot    = joints[pivotKey];
 
    // Map pitch → rotation in the sagittal plane (forward/back swings)
    // Map roll  → lateral tilt
    // We blend both into a single 2-D rotation angle for the canvas.
    // Pitch drives the primary swing; roll adds a tilt component.
    const angleRad = (state.pitch * 0.7 + state.roll * 0.3) * (Math.PI / 180);
 
    // Collect every joint that is reachable through the active bones
    const chainJoints = new Set();
    limbChain.bones.forEach(([a, b]) => { chainJoints.add(a); chainJoints.add(b); });
    // Remove the pivot itself — it stays fixed
    chainJoints.delete(pivotKey);
 
    // Rotate each chain joint around the pivot
    chainJoints.forEach(key => {
      const orig = JOINTS[key];
      const dx   = orig[0] - pivot[0];
      const dy   = orig[1] - pivot[1];
      const cos  = Math.cos(angleRad);
      const sin  = Math.sin(angleRad);
      joints[key] = [
        pivot[0] + dx * cos - dy * sin,
        pivot[1] + dx * sin + dy * cos,
      ];
    });
  }
 
  // ── Draw all bones ─────────────────────────────────────────────
  ctx.lineCap = 'round';
  BONES.forEach(([a, b]) => {
    const pa       = joints[a];
    const pb       = joints[b];
    const isActive = activeBoneSet.has(a + '|' + b);
    ctx.beginPath();
    ctx.moveTo(pa[0], pa[1]);
    ctx.lineTo(pb[0], pb[1]);
    ctx.strokeStyle = isActive ? activeBoneClr : boneClr;
    ctx.lineWidth   = isActive ? 2.8 : 2.2;
    ctx.stroke();
  });
 
  // ── Draw head ──────────────────────────────────────────────────
  ctx.beginPath();
  ctx.arc(joints.head[0], joints.head[1] - 12, 14, 0, Math.PI * 2);
  ctx.fillStyle   = dark ? 'rgba(180,180,180,0.16)' : 'rgba(100,100,100,0.11)';
  ctx.strokeStyle = boneClr;
  ctx.lineWidth   = 1.5;
  ctx.fill();
  ctx.stroke();
 
  // ── Draw joints ────────────────────────────────────────────────
  Object.entries(joints).forEach(([name, pos]) => {
    const isActive = name === activeJoint;
    ctx.beginPath();
    ctx.arc(pos[0], pos[1], isActive ? 7 : 3.5, 0, Math.PI * 2);
    ctx.fillStyle = isActive ? '#185FA5' : jointClr;
    ctx.fill();
    if (isActive) {
      ctx.strokeStyle = dark ? '#1c1c1a' : '#ffffff';
      ctx.lineWidth   = 1.8;
      ctx.stroke();
    }
  });
 
  // ── Label active joint ─────────────────────────────────────────
  if (activeJoint && joints[activeJoint]) {
    const pos   = joints[activeJoint];
    const right = JOINTS[activeJoint][0] >= 120; // use original side to pick label side
    ctx.fillStyle  = '#185FA5';
    ctx.font       = '500 10px var(--font, sans-serif)';
    ctx.textAlign  = right ? 'left' : 'right';
    ctx.fillText(state.assignedPart, pos[0] + (right ? 11 : -11), pos[1] + 4);
  }
}

// ── Line chart ──────────────────────────────────────────────────────────────────
const lineCanvas = document.getElementById('line-canvas');
const lineCtx = lineCanvas.getContext('2d');

function drawChart() {
  const parent = lineCanvas.parentElement;
  const W = parent.clientWidth - 32;
  const H = parent.clientHeight - 30;
  if (!W || !H || W < 10 || H < 10) return;
  lineCanvas.width = W;
  lineCanvas.height = H;

  const ctx = lineCtx;
  const dark = darkMode();
  ctx.clearRect(0, 0, W, H);

  const gridMinor = dark ? 'rgba(255,255,255,0.05)' : 'rgba(0,0,0,0.04)';
  const gridMid = dark ? 'rgba(255,255,255,0.10)' : 'rgba(0,0,0,0.09)';

  // Grid
  [-90, -60, -30, 0, 30, 60, 90].forEach(v => {
    const y = H / 2 - (v / 90) * (H / 2 - 6);
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(W, y);
    ctx.strokeStyle = v === 0 ? gridMid : gridMinor;
    ctx.lineWidth = v === 0 ? 1 : 0.5;
    ctx.stroke();
    if (v !== 0) {
      ctx.fillStyle = dark ? 'rgba(180,180,180,0.35)' : 'rgba(80,80,80,0.3)';
      ctx.font = '9px monospace';
      ctx.textAlign = 'left';
      ctx.fillText(v + '°', 2, y - 2);
    }
  });

  const series = [
    { buf: rollBuf, color: '#E24B4A', label: 'Roll' },
    { buf: pitchBuf, color: '#1D9E75', label: 'Pitch' },
    { buf: yawBuf, color: '#378ADD', label: 'Yaw' },
  ];

  series.forEach(({ buf, color }) => {
    ctx.beginPath();
    for (let i = 0; i < CHART_LEN; i++) {
      const idx = (bufIdx + i) % CHART_LEN;
      const x = (i / (CHART_LEN - 1)) * W;
      const y = H / 2 - (buf[idx] / 90) * (H / 2 - 6);
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    }
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.3;
    ctx.lineJoin = 'round';
    ctx.stroke();
  });

  // Legend (top-right)
  let lx = W - 8;
  series.slice().reverse().forEach(({ color, label }) => {
    ctx.fillStyle = color;
    ctx.font = '10px sans-serif';
    ctx.textAlign = 'right';
    const tw = ctx.measureText(label).width;
    ctx.fillText(label, lx, 12);
    lx -= tw + 18;
  });
}

// ── Data update — called from both serial and simulation ───────────────────────
function ingestSample(r, p, y, axv, ayv, azv, timestamp) {
  state.roll = r;
  state.pitch = p;
  state.yaw = y;
  state.ax = axv;
  state.ay = ayv;
  state.az = azv;

  const amag = Math.sqrt(axv * axv + ayv * ayv + azv * azv);
  if (amag > state.peakA) state.peakA = amag;

  // Ring buffer
  rollBuf[bufIdx] = r;
  pitchBuf[bufIdx] = p;
  yawBuf[bufIdx] = y;
  bufIdx = (bufIdx + 1) % CHART_LEN;

  // Update sidebar values
  document.getElementById('d-roll').textContent = r.toFixed(1) + '°';
  document.getElementById('d-pitch').textContent = p.toFixed(1) + '°';
  document.getElementById('d-yaw').textContent = y.toFixed(1) + '°';
  document.getElementById('d-ax').textContent = axv.toFixed(3) + ' g';
  document.getElementById('d-ay').textContent = ayv.toFixed(3) + ' g';
  document.getElementById('d-az').textContent = azv.toFixed(3) + ' g';
  document.getElementById('d-amag').textContent = amag.toFixed(3) + ' g';
  document.getElementById('s-peak').textContent = state.peakA.toFixed(2);

  // Record sample if recording
  if (state.recording && state.sessionStart !== null) {
    state.samples.push({
      time_s: +((Date.now() - state.sessionStart) / 1000).toFixed(4),
      roll_deg: +r.toFixed(3),
      pitch_deg: +p.toFixed(3),
      yaw_deg: +y.toFixed(3),
      ax_g: +axv.toFixed(4),
      ay_g: +ayv.toFixed(4),
      az_g: +azv.toFixed(4),
      amag_g: +amag.toFixed(4),
      body_part: state.assignedPart || 'unassigned',
    });
    document.getElementById('sample-count').textContent =
      state.samples.length.toLocaleString() + ' samples';
  }

  drawChart();
  drawSkeleton();
}

// ── Simulation mode ─────────────────────────────────────────────────────────────
function startSimulation() {
  state.simMode = true;
  if (state.simInterval) clearInterval(state.simInterval);
  state.simInterval = setInterval(() => {
    state.t += 0.02;
    state.rollTarget += (Math.random() - 0.5) * 3 + Math.sin(state.t * 0.7) * 0.9;
    state.pitchTarget += (Math.random() - 0.5) * 3 + Math.cos(state.t * 0.5) * 0.9;
    state.yawTarget += (Math.random() - 0.5) * 1.5 + Math.sin(state.t * 0.3) * 0.4;
    state.rollTarget = Math.max(-80, Math.min(80, state.rollTarget));
    state.pitchTarget = Math.max(-70, Math.min(70, state.pitchTarget));
    state.yawTarget = Math.max(-85, Math.min(85, state.yawTarget));
    state.roll += (state.rollTarget - state.roll) * 0.12;
    state.pitch += (state.pitchTarget - state.pitch) * 0.12;
    state.yaw += (state.yawTarget - state.yaw) * 0.12;
    const ax = Math.sin(state.pitch * Math.PI / 180) + (Math.random() - 0.5) * 0.04;
    const ay = Math.sin(state.roll * Math.PI / 180) + (Math.random() - 0.5) * 0.04;
    const az = Math.cos(state.pitch * Math.PI / 180) * Math.cos(state.roll * Math.PI / 180) + (Math.random() - 0.5) * 0.04;
    ingestSample(state.roll, state.pitch, state.yaw, ax, ay, az, Date.now());
  }, 10); // ~100 Hz
}

function stopSimulation() {
  if (state.simInterval) { clearInterval(state.simInterval); state.simInterval = null; }
  state.simMode = false;
}

// ── Web Serial API ──────────────────────────────────────────────────────────────
let serialLineBuffer = '';

async function connectSerial() {
  if (!('serial' in navigator)) {
    alert(
      'Web Serial is not supported in this browser.\n\n' +
      'Please use Google Chrome or Microsoft Edge.\n\n' +
      'Starting in simulation mode instead.'
    );
    connectFallback();
    return;
  }

  try {
    const port = await navigator.serial.requestPort();
    await port.open({ baudRate: 115200 });
    state.serialPort = port;

    setConnectedUI(false); // real device

    const decoder = new TextDecoderStream();
    port.readable.pipeTo(decoder.writable);
    const reader = decoder.readable.getReader();
    state.serialReader = reader;

    // Read loop
    (async () => {
      try {
        while (true) {
          const { value, done } = await reader.read();
          if (done) break;
          serialLineBuffer += value;
          const lines = serialLineBuffer.split('\n');
          serialLineBuffer = lines.pop(); // keep incomplete last line
          lines.forEach(line => {
            line = line.trim();
            if (!line.startsWith('{')) return;
            try {
              const d = JSON.parse(line);
              if (d.error) { console.error('ESP32:', d.error); return; }
              if (d.status) { console.log('ESP32 ready:', d); return; }
              ingestSample(d.r, d.p, d.y, d.ax, d.ay, d.az, d.t);
            } catch (_) { /* bad JSON fragment, skip */ }
          });
        }
      } catch (err) {
        console.warn('Serial read ended:', err.message);
      } finally {
        disconnectSerial();
      }
    })();

  } catch (err) {
    if (err.name === 'NotFoundError') {
      // User cancelled port picker — fall back to sim
      connectFallback();
    } else {
      console.error('Serial error:', err);
      alert('Could not open port: ' + err.message + '\n\nStarting in simulation mode.');
      connectFallback();
    }
  }
}

function connectFallback() {
  setConnectedUI(true); // sim mode
  startSimulation();
}

async function disconnectSerial() {
  if (state.serialReader) {
    try { await state.serialReader.cancel(); } catch (_) { }
    state.serialReader = null;
  }
  if (state.serialPort) {
    try { await state.serialPort.close(); } catch (_) { }
    state.serialPort = null;
  }
  setDisconnectedUI();
}

// ── UI state helpers ────────────────────────────────────────────────────────────
function setConnectedUI(isSim) {
  state.connected = true;
  const btn = document.getElementById('conn-btn');
  const dot = document.getElementById('conn-dot');
  const badge = document.getElementById('conn-badge');
  const info = document.getElementById('info-panel');
  btn.textContent = 'Disconnect';
  btn.classList.add('connected');
  dot.classList.add('live');
  if (isSim) {
    badge.textContent = 'Simulation mode';
    badge.className = 'badge sim';
    info.innerHTML = 'Running in <strong>simulation mode</strong>.<br>No ESP32 detected or browser<br>does not support Web Serial.<br><br>Real device: use Chrome/Edge.';
  } else {
    badge.textContent = 'Connected (ESP32)';
    badge.className = 'badge connected';
    info.innerHTML = '✓ ESP32 connected via USB.<br>Receiving at 100 Hz.';
  }
  // Reset buffers
  rollBuf.fill(0); pitchBuf.fill(0); yawBuf.fill(0); bufIdx = 0;
  state.peakA = 0;
  document.getElementById('s-peak').textContent = '0';
}

function setDisconnectedUI() {
  state.connected = false;
  if (state.recording) stopRecording();
  stopSimulation();
  const btn = document.getElementById('conn-btn');
  const dot = document.getElementById('conn-dot');
  const badge = document.getElementById('conn-badge');
  const info = document.getElementById('info-panel');
  btn.textContent = 'Connect ESP32';
  btn.classList.remove('connected');
  dot.classList.remove('live');
  dot.classList.remove('error');
  badge.textContent = 'Disconnected';
  badge.className = 'badge';
  info.innerHTML = 'Connect your ESP32 via USB.<br>Click "Connect ESP32" and select<br>the correct COM / tty port.<br><br>No device? The dashboard runs<br>in simulation mode automatically.';

  // Clear displays
  ['d-roll', 'd-pitch', 'd-yaw', 'd-ax', 'd-ay', 'd-az', 'd-amag'].forEach(id => {
    document.getElementById(id).textContent = '—';
  });
  document.getElementById('s-dur').textContent = '0s';
  state.roll = 0; state.pitch = 0; state.yaw = 0;
  drawChart();
}

// ── Record / Stop ───────────────────────────────────────────────────────────────
function startRecording() {
  if (!state.connected) return;
  state.recording = true;
  state.sessionStart = Date.now();
  state.samples = [];
  document.getElementById('sample-count').textContent = '0 samples';
  document.getElementById('s-dur').textContent = '0s';

  const btn = document.getElementById('rec-btn');
  const badge = document.getElementById('conn-badge');
  btn.className = 'btn btn-record recording';
  btn.innerHTML = '<div class="rec-dot"></div><span>Stop</span>';
  if (!state.simMode) {
    badge.textContent = 'Recording…';
    badge.className = 'badge recording';
  }
  state.durationInterval = setInterval(() => {
    if (!state.recording || !state.sessionStart) return;
    const secs = Math.floor((Date.now() - state.sessionStart) / 1000);
    document.getElementById('s-dur').textContent = secs + 's';
  }, 500);
}

function stopRecording() {
  state.recording = false;
  if (state.durationInterval) { clearInterval(state.durationInterval); state.durationInterval = null; }
  const btn = document.getElementById('rec-btn');
  const badge = document.getElementById('conn-badge');
  btn.className = 'btn btn-record';
  btn.innerHTML = '<div class="rec-dot"></div><span>Record</span>';
  if (state.simMode) {
    badge.textContent = 'Simulation mode';
    badge.className = 'badge sim';
  } else {
    badge.textContent = 'Connected (ESP32)';
    badge.className = 'badge connected';
  }
  if (state.sessionStart) {
    const secs = Math.floor((Date.now() - state.sessionStart) / 1000);
    document.getElementById('s-dur').textContent = secs + 's';
  }
}

// ── Export CSV ──────────────────────────────────────────────────────────────────
function exportCSV() {
  if (!state.samples.length) {
    alert('No data to export.\nConnect the sensor and press Record first.');
    return;
  }
  const headers = ['time_s', 'roll_deg', 'pitch_deg', 'yaw_deg', 'ax_g', 'ay_g', 'az_g', 'amag_g', 'body_part'];
  const rows = state.samples.map(s => headers.map(k => s[k]).join(','));
  const csv = [headers.join(','), ...rows].join('\r\n');
  const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `imu_session_${new Date().toISOString().slice(0, 19).replace(/[:]/g, '-')}.csv`;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(a.href);
}

// ── Wire up button event listeners ─────────────────────────────────────────────
document.getElementById('conn-btn').addEventListener('click', () => {
  if (state.connected) {
    disconnectSerial();
  } else {
    connectSerial();
  }
});

document.getElementById('rec-btn').addEventListener('click', () => {
  if (!state.connected) return;
  state.recording ? stopRecording() : startRecording();
});

document.getElementById('export-btn').addEventListener('click', exportCSV);

// ── Initial draw ────────────────────────────────────────────────────────────────
drawChart();
window.addEventListener('resize', () => { drawChart(); drawSkeleton(); });
