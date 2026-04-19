// ── Widget Definitions ─────────────────────────────────────────────────────
const WIDGET_DEFS = {
  imu: {
    title: 'IMU', icon: '▸',
    defaultGrid: { x: 0, y: 0, w: 3, h: 2 }, minW: 2, minH: 2,
    buildContent: () => `
      <div class="card-row"><span class="card-label">Yaw</span><span class="tv" id="iY">—</span></div>
      <div class="card-row"><span class="card-label">Pitch</span><span class="tv" id="iP">—</span></div>
      <div class="card-row"><span class="card-label">Roll</span><span class="tv" id="iR">—</span></div>
      <div class="card-row" style="margin-top:6px"><span class="card-label" style="font-size:10px">Age</span><span class="tv" id="iA" style="font-size:10px">—</span></div>`
  },
  gps: {
    title: 'GPS', icon: '▸',
    defaultGrid: { x: 3, y: 0, w: 3, h: 2 }, minW: 2, minH: 2,
    buildContent: () => `
      <div class="card-row"><span class="card-label">Lat</span><span class="tv" id="gLa">—</span></div>
      <div class="card-row"><span class="card-label">Lon</span><span class="tv" id="gLo">—</span></div>
      <div class="card-row"><span class="card-label">Alt</span><span class="tv" id="gAl">—</span></div>
      <div class="card-row"><span class="card-label">Speed</span><span class="tv" id="gSp">—</span></div>
      <div class="card-row"><span class="card-label">Sats / Fix</span><span class="tv" id="gSa">—</span></div>`
  },
  encoders: {
    title: 'ENCODERS', icon: '▸',
    defaultGrid: { x: 6, y: 0, w: 3, h: 2 }, minW: 2, minH: 2,
    buildContent: () => `
      <div class="card-row"><span class="card-label">Left</span><span class="tv" id="eL">—</span></div>
      <div class="card-row"><span class="card-label">Right</span><span class="tv" id="eR">—</span></div>
      <div class="card-row" style="margin-top:6px"><span class="card-label" style="font-size:10px">Age</span><span class="tv" id="eA" style="font-size:10px">—</span></div>`
  },
  distance: {
    title: 'DISTANCE', icon: '▸',
    defaultGrid: { x: 9, y: 0, w: 3, h: 2 }, minW: 2, minH: 2,
    buildContent: () => `
      <div class="dist-big"><span id="dV">—</span><small>metres</small></div>
      <div class="card-row" style="margin-top:4px"><span class="card-label" style="font-size:10px">Age</span><span class="tv" id="dA" style="font-size:10px">—</span></div>
      <div class="card-row" style="margin-top:6px"><span class="card-label">Det. FPS</span><span class="tv" id="detFps">—</span></div>`
  },
  system: {
    title: 'SYSTEM', icon: '▸',
    defaultGrid: { x: 0, y: 2, w: 3, h: 2 }, minW: 2, minH: 2,
    buildContent: () => `
      <div class="card-row"><span class="card-label">CPU</span><span class="tv" id="sysCpu">—</span></div>
      <div class="card-row"><span class="card-label">GPU</span><span class="tv" id="sysGpu">—</span></div>
      <div class="card-row"><span class="card-label">CPU Temp</span><span class="tv" id="sysCpuT">—</span></div>
      <div class="card-row"><span class="card-label">GPU Temp</span><span class="tv" id="sysGpuT">—</span></div>
      <div class="card-row"><span class="card-label">RAM</span><span class="tv" id="sysRam">—</span></div>`
  },
  lidar: {
    title: 'LIDAR', icon: '▸',
    defaultGrid: { x: 3, y: 2, w: 3, h: 4 }, minW: 3, minH: 3,
    buildContent: () => `
      <canvas id="radarCanvas" style="display:block;margin:auto"></canvas>
      <div class="card-row" style="margin-top:6px"><span class="card-label">Fwd min</span><span class="tv" id="lidarFwd">—</span></div>
      <div id="obstBadge" style="display:none;text-align:center;margin-top:4px;color:var(--danger);font-size:11px;letter-spacing:1px">&#9888; OBSTACLE</div>
      <div class="card-row" style="margin-top:6px"><span class="card-label">SLAM</span><span class="tv" id="slamPose">—</span></div>`,
    onAdd(el) {
      const body = el.querySelector('.widget-body');
      const canvas = el.querySelector('#radarCanvas');
      if (!body || !canvas) return;
      const resize = () => {
        const w = body.clientWidth - 24;
        const textH = 70;
        const availH = body.clientHeight - 20 - textH;
        const size = Math.max(60, Math.min(w, availH));
        if (canvas.width !== size || canvas.height !== size) {
          canvas.width = size;
          canvas.height = size;
        }
      };
      new ResizeObserver(resize).observe(body);
      resize();
    }
  },
  'slam-map': {
    title: 'SLAM MAP', icon: '▸',
    defaultGrid: { x: 6, y: 2, w: 6, h: 4 }, minW: 3, minH: 3,
    buildContent: () => `
      <canvas id="slamMapCanvas" style="display:block;width:100%;height:100%;image-rendering:pixelated;border-radius:4px"></canvas>`,
    onAdd(el) {
      const canvas = el.querySelector('#slamMapCanvas');
      if (!canvas) return;
      const resize = () => {
        const w = canvas.clientWidth;
        const h = canvas.clientHeight;
        if (w > 0 && h > 0 && (canvas.width !== w || canvas.height !== h)) {
          canvas.width = w;
          canvas.height = h;
        }
      };
      new ResizeObserver(resize).observe(canvas);
      resize();
    }
  },
  'wheel-ctrl': {
    title: 'WHEEL CONTROL', icon: '▸',
    defaultGrid: { x: 0, y: 6, w: 4, h: 4 }, minW: 3, minH: 3,
    buildContent: () => `
      <div class="dpad">
        <div></div><button id="bF" title="Forward (W)">▲</button><div></div>
        <button id="bL" title="Left (A)">◄</button>
        <button id="bS" class="sc" title="Stop (Space)">STOP</button>
        <button id="bR" title="Right (D)">►</button>
        <div></div><button id="bB" title="Backward (X)">▼</button><div></div>
      </div>
      <div class="srow">
        <label>Speed</label>
        <input type="range" id="ss" min="10" max="100" value="50" oninput="document.getElementById('sv').textContent=this.value+'%'">
        <span class="sv" id="sv">50%</span>
      </div>`,
    onAdd() { if (typeof initControls === 'function') initControls(); }
  },
  'ptu-ctrl': {
    title: 'PTU CONTROL', icon: '▸',
    defaultGrid: { x: 4, y: 6, w: 4, h: 4 }, minW: 3, minH: 3,
    buildContent: () => `
      <div class="ptupad">
        <div></div><button id="bTU" title="Tilt Up (↑)">▲</button><div></div>
        <button id="bPL" title="Pan Left (←)">◄</button>
        <button id="bPC" class="cb" title="Centre (C)">CTR</button>
        <button id="bPR" title="Pan Right (→)">►</button>
        <div></div><button id="bTD" title="Tilt Down (↓)">▼</button><div></div>
      </div>
      <div class="srow">
        <label>PTU Spd</label>
        <input type="range" id="ps" min="10" max="100" value="40" oninput="document.getElementById('pv').textContent=this.value+'%'">
        <span class="sv" id="pv">40%</span>
      </div>`,
    onAdd() { if (typeof initControls === 'function') initControls(); }
  },
  actions: {
    title: 'ACTIONS', icon: '▸',
    defaultGrid: { x: 8, y: 6, w: 4, h: 3 }, minW: 2, minH: 2,
    buildContent: () => `
      <div class="abtn">
        <button class="btn btn-accent" onclick="sendCmd('resume_follow')">▶ RESUME FOLLOW</button>
        <button class="btn btn-warn"   onclick="sendCmd('stop_follow')">⊘ STOP FOLLOW</button>
        <button class="btn btn-danger" onclick="eStop()">⚠ EMERGENCY STOP</button>
        <button class="btn btn-blue"   onclick="go('planner');newM()">⊕ NEW MISSION</button>
        <div class="kbhint">W/A/D/X = drive &nbsp;|&nbsp; Space = stop<br>Arrow keys = PTU &nbsp;|&nbsp; C = centre PTU</div>
      </div>`
  },
  'video-main': {
    title: 'DETECTION', icon: '▸',
    defaultGrid: { x: 0, y: 10, w: 5, h: 5 }, minW: 3, minH: 3,
    noPad: true,
    buildContent: () => `
      <div style="position:absolute;top:8px;right:10px;z-index:10;display:flex;align-items:center;gap:8px">
        <label style="font-size:10px;letter-spacing:1px;color:var(--muted);background:rgba(10,14,26,.7);padding:2px 8px;border-radius:3px;display:flex;align-items:center;gap:6px;cursor:pointer">
          Q <input id="sqSlider" type="range" min="10" max="95" value="55" step="5"
              style="width:60px;accent-color:var(--accent);cursor:pointer"
              oninput="onSQSlider(this.value)">
          <span id="sqVal">55</span>
        </label>
        <span id="streamStatus" style="font-size:10px;letter-spacing:1px;color:var(--muted);background:rgba(10,14,26,.7);padding:2px 8px;border-radius:3px">CONNECTING</span>
      </div>
      <img id="streamImg" alt="Camera feed" style="width:100%;height:100%;display:block;object-fit:contain"
        onerror="streamErr()" onload="streamOk()">
      <div id="streamPlaceholder" style="display:none;width:100%;height:100%;align-items:center;justify-content:center;flex-direction:column;gap:12px;background:#050810">
        <div style="font-size:11px;letter-spacing:2px;color:var(--danger)">NO SIGNAL</div>
        <div style="font-size:10px;color:var(--muted)">CAMERA FEED UNAVAILABLE</div>
      </div>`,
    onAdd() { setTimeout(startStream, 200); }
  },
  'video-thermal': {
    title: 'THERMAL', icon: '▸',
    defaultGrid: { x: 5, y: 10, w: 3, h: 5 }, minW: 2, minH: 3,
    noPad: true,
    buildContent: () => `
      <div style="position:absolute;top:8px;right:10px;z-index:10;display:flex;align-items:center;gap:8px">
        <span id="thermalStatus" style="font-size:10px;letter-spacing:1px;color:var(--muted);background:rgba(10,14,26,.7);padding:2px 8px;border-radius:3px">CONNECTING</span>
      </div>
      <img id="thermalImg" alt="Thermal feed" style="width:100%;height:100%;display:none;object-fit:contain"
        onerror="thermalErr()" onload="thermalOk()">
      <div id="thermalPlaceholder" style="width:100%;height:100%;display:flex;align-items:center;justify-content:center;flex-direction:column;gap:8px;background:#050810">
        <div style="font-size:11px;letter-spacing:2px;color:#333">CONNECTING</div>
      </div>`,
    onAdd() { setTimeout(startThermalStream, 300); }
  },
  'video-depth': {
    title: 'DEPTH', icon: '▸',
    defaultGrid: { x: 8, y: 10, w: 4, h: 5 }, minW: 3, minH: 3,
    noPad: true,
    buildContent: () => `
      <div style="position:absolute;top:8px;right:10px;z-index:10;display:flex;align-items:center;gap:8px">
        <label style="font-size:10px;letter-spacing:1px;color:var(--muted);background:rgba(10,14,26,.7);padding:2px 8px;border-radius:3px;display:flex;align-items:center;gap:6px;cursor:pointer">
          Q <input id="dqSlider" type="range" min="10" max="95" value="60" step="5"
              style="width:60px;accent-color:var(--blue);cursor:pointer"
              oninput="onDQSlider(this.value)">
          <span id="dqVal">60</span>
        </label>
        <span id="depthStatus" style="font-size:10px;letter-spacing:1px;color:var(--muted);background:rgba(10,14,26,.7);padding:2px 8px;border-radius:3px">STARTING</span>
      </div>
      <img id="depthImg" alt="Depth feed" style="width:100%;height:100%;display:none;object-fit:contain"
        onerror="depthErr()" onload="depthOk()">
      <div id="depthPlaceholder" style="width:100%;height:100%;display:flex;align-items:center;justify-content:center;flex-direction:column;gap:8px;background:#050810">
        <div style="font-size:11px;letter-spacing:2px;color:#333">STARTING DEPTH</div>
      </div>`,
    onAdd() { setTimeout(startDepthStream, 500); }
  },
  targets: {
    title: 'KNOWN TARGETS', icon: '▸',
    defaultGrid: { x: 0, y: 15, w: 12, h: 3 }, minW: 4, minH: 2,
    buildContent: () => `
      <div id="tgGrid" class="tg-grid">
        <div id="tgEmpty" style="color:var(--muted);font-size:11px;padding:4px 0">No targets identified yet</div>
      </div>`,
    bodyOverflow: 'auto'
  }
};

// ── Default Layout ─────────────────────────────────────────────────────────
const DEFAULT_LAYOUT = [
  { type: 'imu',         x: 0,  y: 0,  w: 3, h: 2 },
  { type: 'gps',         x: 3,  y: 0,  w: 3, h: 2 },
  { type: 'encoders',    x: 6,  y: 0,  w: 3, h: 2 },
  { type: 'distance',    x: 9,  y: 0,  w: 3, h: 2 },
  { type: 'system',      x: 0,  y: 2,  w: 3, h: 2 },
  { type: 'lidar',       x: 3,  y: 2,  w: 3, h: 4 },
  { type: 'slam-map',    x: 6,  y: 2,  w: 6, h: 4 },
  { type: 'wheel-ctrl',  x: 0,  y: 6,  w: 4, h: 4 },
  { type: 'ptu-ctrl',    x: 4,  y: 6,  w: 4, h: 4 },
  { type: 'actions',     x: 8,  y: 6,  w: 4, h: 3 },
  { type: 'video-main',    x: 0,  y: 10, w: 5, h: 5 },
  { type: 'video-thermal', x: 5,  y: 10, w: 3, h: 5 },
  { type: 'video-depth',   x: 8,  y: 10, w: 4, h: 5 },
  { type: 'targets',     x: 0,  y: 15, w: 12, h: 3 }
];

// ── Grid instance ──────────────────────────────────────────────────────────
let grid = null;
let activeWidgets = new Set();

function initDashboard() {
  const el = document.getElementById('dashboard-grid');
  grid = GridStack.init({
    column: 12,
    cellHeight: 70,
    margin: 5,
    float: false,
    animate: true,
    handle: '.widget-drag-handle',
    disableOneColumnMode: false,
    oneColumnSize: 768,
    removable: false,
    resizable: { handles: 'se' }
  }, el);

  loadDefaultLayout();
  initLayoutToolbar();
}

function loadDefaultLayout() {
  if (grid) grid.removeAll();
  activeWidgets.clear();
  DEFAULT_LAYOUT.forEach(item => addWidget(item.type, item));
}

function addWidget(type, pos) {
  const def = WIDGET_DEFS[type];
  if (!def || activeWidgets.has(type)) return;

  const g = pos || def.defaultGrid;
  const bodyClass = 'widget-body' + (def.noPad ? ' no-pad' : '');
  const bodyStyle = def.bodyOverflow ? `overflow-y:${def.bodyOverflow}` : '';

  const content = `
    <div class="widget-header">
      <span class="widget-drag-handle">⠿</span>
      <span class="widget-title">${def.icon} ${def.title}</span>
      <button class="widget-close" onclick="removeWidget('${type}')">✕</button>
    </div>
    <div class="${bodyClass}" style="${bodyStyle}">${def.buildContent()}</div>`;

  grid.addWidget({
    x: g.x, y: g.y, w: g.w, h: g.h,
    minW: def.minW, minH: def.minH,
    id: type,
    content: content
  });

  activeWidgets.add(type);

  // Run post-add callback if defined
  if (def.onAdd) {
    const el = grid.el.querySelector(`[gs-id="${type}"]`);
    if (el) def.onAdd(el);
  }
}

function removeWidget(type) {
  const el = grid.el.querySelector(`[gs-id="${type}"]`);
  if (el) {
    grid.removeWidget(el);
    activeWidgets.delete(type);
    updateDrawer();
  }
}

// ── Widget Drawer ──────────────────────────────────────────────────────────
function toggleDrawer() {
  const drawer = document.getElementById('widgetDrawer');
  drawer.classList.toggle('open');
  if (drawer.classList.contains('open')) updateDrawer();
}

function updateDrawer() {
  const drawer = document.getElementById('widgetDrawer');
  const items = drawer.querySelector('.widget-drawer-items');
  if (!items) return;
  items.innerHTML = '';
  for (const [type, def] of Object.entries(WIDGET_DEFS)) {
    if (activeWidgets.has(type)) continue;
    const el = document.createElement('div');
    el.className = 'widget-drawer-item';
    el.textContent = def.icon + ' ' + def.title;
    el.onclick = () => { addWidget(type); updateDrawer(); };
    items.appendChild(el);
  }
  // If all widgets are on the grid, show a message
  if (!items.children.length) {
    items.innerHTML = '<div style="color:var(--muted);font-size:11px;padding:8px 0;text-align:center">All widgets active</div>';
  }
}

// ── Layout serialization helpers ───────────────────────────────────────────
function getGridState() {
  if (!grid) return [];
  return grid.getGridItems().map(el => ({
    type: el.getAttribute('gs-id'),
    x: +el.getAttribute('gs-x'),
    y: +el.getAttribute('gs-y'),
    w: +el.getAttribute('gs-w'),
    h: +el.getAttribute('gs-h')
  }));
}

function applyLayout(widgets) {
  if (!grid) return;
  grid.removeAll();
  activeWidgets.clear();
  widgets.forEach(item => {
    if (WIDGET_DEFS[item.type]) addWidget(item.type, item);
  });
}

// ── Layout Management (server-side persistence) ────────────────────────────
let currentLayoutName = null;

async function listLayouts() {
  try {
    const r = await fetch('/api/layouts');
    return await r.json();
  } catch { return []; }
}

async function loadLayout(name) {
  try {
    const r = await fetch(`/api/layouts/${encodeURIComponent(name)}`);
    if (!r.ok) return false;
    const layout = await r.json();
    applyLayout(layout.widgets || []);
    currentLayoutName = name;
    // Re-bind controls after layout change
    if (typeof initControls === 'function') initControls();
    refreshLayoutDropdown();
    return true;
  } catch { return false; }
}

async function saveLayout(name) {
  if (!name) return;
  const layout = { widgets: getGridState() };
  try {
    await fetch(`/api/layouts/${encodeURIComponent(name)}`, {
      method: 'PUT',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(layout)
    });
    currentLayoutName = name;
    refreshLayoutDropdown();
  } catch (e) { console.error('[layout] save failed:', e); }
}

async function deleteLayout(name) {
  if (!name) return;
  try {
    await fetch(`/api/layouts/${encodeURIComponent(name)}`, { method: 'DELETE' });
    if (currentLayoutName === name) currentLayoutName = null;
    refreshLayoutDropdown();
  } catch (e) { console.error('[layout] delete failed:', e); }
}

async function refreshLayoutDropdown() {
  const sel = document.getElementById('layoutSelect');
  const layouts = await listLayouts();
  sel.innerHTML = '<option value="">Default</option>';
  layouts.forEach(l => {
    const opt = document.createElement('option');
    opt.value = l.name;
    opt.textContent = l.name;
    if (l.name === currentLayoutName) opt.selected = true;
    sel.appendChild(opt);
  });
}

// ── Toolbar event wiring (called once from initDashboard) ──────────────────
function initLayoutToolbar() {
  const sel = document.getElementById('layoutSelect');
  const saveBtn = document.getElementById('layoutSave');
  const saveAsBtn = document.getElementById('layoutSaveAs');
  const deleteBtn = document.getElementById('layoutDelete');

  sel.addEventListener('change', () => {
    const name = sel.value;
    if (name) loadLayout(name);
    else {
      loadDefaultLayout();
      currentLayoutName = null;
      if (typeof initControls === 'function') initControls();
    }
  });

  saveBtn.addEventListener('click', () => {
    if (currentLayoutName) {
      saveLayout(currentLayoutName);
    } else {
      // No layout selected — behave like Save As
      const name = prompt('Layout name:');
      if (name && name.trim()) saveLayout(name.trim());
    }
  });

  saveAsBtn.addEventListener('click', () => {
    const name = prompt('Save layout as:', currentLayoutName || '');
    if (name && name.trim()) saveLayout(name.trim());
  });

  deleteBtn.addEventListener('click', async () => {
    if (!currentLayoutName) return;
    if (!confirm(`Delete layout "${currentLayoutName}"?`)) return;
    await deleteLayout(currentLayoutName);
    loadDefaultLayout();
    currentLayoutName = null;
    if (typeof initControls === 'function') initControls();
  });

  // Load saved layouts into dropdown
  refreshLayoutDropdown();

  // Auto-load last-used layout from localStorage
  const lastLayout = localStorage.getItem('umbris_last_layout');
  if (lastLayout) {
    loadLayout(lastLayout).then(ok => {
      if (!ok) {
        loadDefaultLayout();
        if (typeof initControls === 'function') initControls();
      }
    });
  }

  // Save last-used layout name on change
  grid.on('change', () => {
    if (currentLayoutName) localStorage.setItem('umbris_last_layout', currentLayoutName);
  });
}
