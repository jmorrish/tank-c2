// ── Radar ──────────────────────────────────────────────────────────────────
function updateRadar(msg){
  const canvas=document.getElementById('radarCanvas');
  if(!canvas)return;
  const ctx=canvas.getContext('2d');
  const cx=canvas.width/2,cy=canvas.height/2;

  const pts=msg.pts||[];
  let maxDist=0;
  for(const[,d]of pts) if(d>0) maxDist=Math.max(maxDist,d);
  const range=Math.max(1.5, Math.ceil(maxDist*2)/2);
  const scale=cx/range;

  ctx.clearRect(0,0,canvas.width,canvas.height);

  ctx.strokeStyle='#1f2937';ctx.lineWidth=1;
  ctx.fillStyle='#4b5563';ctx.font='8px monospace';ctx.textAlign='center';
  [0.25,0.5,0.75,1.0].forEach(frac=>{
    const r=frac*range, px=r*scale;
    ctx.beginPath();ctx.arc(cx,cy,px,0,Math.PI*2);ctx.stroke();
    ctx.fillText(r.toFixed(1)+'m', cx, cy-px-2);
  });

  const arcRad=30*Math.PI/180;
  ctx.fillStyle='rgba(0,230,118,0.05)';
  ctx.beginPath();ctx.moveTo(cx,cy);
  ctx.arc(cx,cy,cx,-Math.PI/2-arcRad,-Math.PI/2+arcRad);
  ctx.closePath();ctx.fill();

  ctx.fillStyle=msg.obs?'#ff1744':'#00e676';
  for(const[a,d]of pts){
    if(d<=0)continue;
    const rad=(a-90)*Math.PI/180;
    const px=cx+Math.cos(rad)*d*scale,py=cy+Math.sin(rad)*d*scale;
    ctx.beginPath();ctx.arc(px,py,1.5,0,Math.PI*2);ctx.fill();
  }

  ctx.fillStyle='#00b0ff';
  ctx.beginPath();ctx.arc(cx,cy,4,0,Math.PI*2);ctx.fill();

  const fwdEl=document.getElementById('lidarFwd');
  const obsEl=document.getElementById('obstBadge');
  if(fwdEl){fwdEl.textContent=msg.fwd>0?f(msg.fwd,2)+' m':'—';fwdEl.style.color=msg.obs?'var(--danger)':'var(--accent)';}
  if(obsEl)obsEl.style.display=msg.obs?'block':'none';
}

// ── SLAM Map ───────────────────────────────────────────────────────────────
let lastSlamMapMeta = null;
let lastSlamPose    = null;

function updateSlamMap(msg){
  const canvas = document.getElementById('slamMapCanvas');
  if(!canvas) return;

  const CW = canvas.width;
  const CH = canvas.height;
  if(CW === 0 || CH === 0) return;

  lastSlamMapMeta = {res: msg.res, ox: msg.ox, oy: msg.oy, w: msg.w, h: msg.h};

  const ctx = canvas.getContext('2d');

  const scale = Math.min(CW / msg.w, CH / msg.h);
  const dw = Math.round(msg.w * scale);
  const dh = Math.round(msg.h * scale);
  const dx = Math.round((CW - dw) / 2);
  const dy = Math.round((CH - dh) / 2);

  const img = new Image();
  img.onload = () => {
    ctx.fillStyle = '#0a1a28';
    ctx.fillRect(0, 0, CW, CH);
    ctx.drawImage(img, dx, dy, dw, dh);
    drawRobotOnMap(ctx, CW, CH, dx, dy, dw, dh);
  };
  img.src = 'data:image/png;base64,' + msg.png;
}

function drawRobotOnMap(ctx, CW, CH, dx, dy, dw, dh){
  if(!lastSlamPose || !lastSlamMapMeta || !lastSlamPose.valid) return;
  const {res, ox, oy, w, h} = lastSlamMapMeta;

  const scaleX = dw / w, scaleY = dh / h;
  const px = dx + (lastSlamPose.x - ox) / res * scaleX;
  const py = dy + dh - (lastSlamPose.y - oy) / res * scaleY;

  if(px < dx || px > dx+dw || py < dy || py > dy+dh) return;

  const theta = lastSlamPose.theta;
  ctx.save();
  ctx.translate(px, py);
  ctx.rotate(-theta);
  ctx.strokeStyle = '#ff5252';
  ctx.lineWidth   = 2;
  ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(10,0); ctx.stroke();
  ctx.restore();

  ctx.fillStyle = '#00b0ff';
  ctx.beginPath(); ctx.arc(px, py, 5, 0, Math.PI*2); ctx.fill();
}

// ── Telemetry ──────────────────────────────────────────────────────────────
function updateTele(d){
  if(!d){
    ['iY','iP','iR','iA','gLa','gLo','gAl','gSp','gSa','eL','eR','eA','dV','dA','detFps','slamPose','sysCpu','sysGpu','sysCpuT','sysGpuT','sysRam'].forEach(id=>sv2(id,'—'));
    ['iY','gLa','eL','dV'].forEach(id=>setStale(id,false));
    return;
  }
  if(d.imu){sv2('iY',f(d.imu.yaw,1,'°'));sv2('iP',f(d.imu.pitch,1,'°'));sv2('iR',f(d.imu.roll,1,'°'));sv2('iA',f(d.imu.age_ms,0,'ms'));setStale('iY',d.imu.age_ms>2000)}
  if(d.gps){const g=d.gps;sv2('gLa',f(g.lat,6));sv2('gLo',f(g.lon,6));sv2('gAl',f(g.alt,1,'m'));sv2('gSp',f(g.speed_knots*1.852,1,'km/h'));sv2('gSa',`${g.sats} / Q${g.quality}`);setStale('gLa',g.age_ms>2000);if(g.quality>0&&g.lat!==0){lastGPS=[g.lat,g.lon];moveTank(g.lat,g.lon)}}
  if(d.encoders){sv2('eL',d.encoders.left);sv2('eR',d.encoders.right);sv2('eA',f(d.encoders.age_ms,0,'ms'));setStale('eL',d.encoders.age_ms>2000)}
  if(d.distance_m!=null&&d.distance_m>=0){sv2('dV',f(d.distance_m,2));sv2('dA',f(d.distance_age_ms,0,'ms'));setStale('dV',d.distance_age_ms>2000)}
  if(d.detection_fps!=null)sv2('detFps',f(d.detection_fps,1,' fps'));
  if(d.slam_pose?.valid){
    lastSlamPose = d.slam_pose;
    sv2('slamPose',`x:${f(d.slam_pose.x,1)} y:${f(d.slam_pose.y,1)} ${f(d.slam_pose.theta*180/Math.PI,0)}°`);
  } else sv2('slamPose','—');
  if(d.sensors){
    const s=d.sensors;
    if(s.cpu_usage_pct)sv2('sysCpu',f(s.cpu_usage_pct.value,1,'%'));
    if(s.gpu_usage_pct)sv2('sysGpu',f(s.gpu_usage_pct.value,1,'%'));
    if(s.cpu_temp_c){sv2('sysCpuT',f(s.cpu_temp_c.value,1,'°C'));setStale('sysCpuT',s.cpu_temp_c.value>90)}
    if(s.gpu_temp_c){sv2('sysGpuT',f(s.gpu_temp_c.value,1,'°C'));setStale('sysGpuT',s.gpu_temp_c.value>90)}
    if(s.ram_used_mb&&s.ram_total_mb)sv2('sysRam',`${f(s.ram_used_mb.value/1024,1)} / ${f(s.ram_total_mb.value/1024,1)} GB`);
  }
  if(d.stream_quality!=null&&!sqDebounce){const sl=document.getElementById('sqSlider');if(sl){sl.value=d.stream_quality;document.getElementById('sqVal').textContent=d.stream_quality;}}
  if(d.target_person_id!=null) highlightTarget(d.target_person_id);
  if(d.mission&&d.mission.running){
    const idx=(d.mission.waypoint_idx??0)+1, tot=d.mission.waypoint_count||'?';
    const name=d.mission.name||'MISSION';
    const fault=d.mission.fault||'';
    const faultLabels={'gps_lost':'⚠ GPS LOST','stuck':'⚠ STUCK','wheel_fail':'⚠ WHEEL FAULT'};
    if(fault&&faultLabels[fault])
      showBanner(`${faultLabels[fault]} — ${name} WP ${idx}/${tot} PAUSED`,'fault');
    else
      showBanner(`${name} — WP ${idx}/${tot}`,'ok');
    document.getElementById('skipBtn').style.display='';
    if(d.mission.target_lat&&map)moveMissionTarget(d.mission.target_lat,d.mission.target_lon);
  } else if(d.mode&&d.mode!=='mission'){
    const mb=document.getElementById('mb');
    if(!mb.classList.contains('pending')&&!mb.classList.contains('fault')){
      hideBanner();clearMissionTarget();
    }
  }
}

// ── Target gallery ─────────────────────────────────────────────────────────
let _activeTargetId = -1;

function loadTargets(){
  fetch('/api/targets').then(r=>r.ok?r.json():[]).then(list=>{
    const grid = document.getElementById('tgGrid');
    if(!grid) return;
    const empty = document.getElementById('tgEmpty');
    if(!list.length){ if(empty) empty.style.display=''; return; }
    empty.style.display='none';
    const existing = new Set([...grid.querySelectorAll('.tg-card')].map(c=>+c.dataset.pid));
    list.forEach(t=>{
      if(existing.has(t.id)){
        const lbl = grid.querySelector(`.tg-card[data-pid="${t.id}"] .tg-label`);
        if(lbl && !lbl.isContentEditable) lbl.textContent = t.name;
        const prof = grid.querySelector(`.tg-card[data-pid="${t.id}"] .tg-prof`);
        if(prof) setEmbedCount(prof, t.embed_count||0);
        return;
      }
      const card = document.createElement('div');
      card.className='tg-card'; card.dataset.pid=t.id;
      card.innerHTML=`<img class="tg-thumb" src="/api/targets/${t.id}/thumb?t=${Date.now()}" alt="P${t.id}" loading="lazy">
<span class="tg-label" title="Click to rename">${t.name}</span>
<button class="tg-follow" onclick="followTarget(${t.id})">FOLLOW</button>
<div class="tg-learn-prog"></div>
<button class="tg-learn" onclick="toggleLearn(this,${t.id})">DESCRIBE</button>
<div class="tg-prof"></div>
<div class="tg-desc"></div>`;
      setEmbedCount(card.querySelector('.tg-prof'), t.embed_count||0);
      const lbl = card.querySelector('.tg-label');
      lbl.addEventListener('click', () => startRename(lbl, t.id));
      grid.appendChild(card);
    });
    highlightTarget(_activeTargetId);
  }).catch(()=>{});
}

function startRename(lbl, id){
  const prev = lbl.textContent;
  lbl.contentEditable = 'true';
  lbl.focus();
  const sel = window.getSelection();
  const range = document.createRange();
  range.selectNodeContents(lbl);
  sel.removeAllRanges(); sel.addRange(range);

  const commit = () => {
    lbl.contentEditable = 'false';
    const name = lbl.textContent.trim().slice(0, 64) || prev;
    lbl.textContent = name;
    fetch(`/api/targets/${id}/name`, {
      method: 'PUT',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify({name})
    }).catch(()=>{});
  };
  lbl.addEventListener('blur', commit, {once:true});
  lbl.addEventListener('keydown', e => {
    if(e.key === 'Enter'){ e.preventDefault(); lbl.blur(); }
    if(e.key === 'Escape'){ lbl.textContent = prev; lbl.contentEditable='false'; }
  }, {once:true});
}

function followTarget(id){
  sendCmd('set_target:'+id);
}

function toggleLearn(btn, id){
  const prog=btn.previousElementSibling;
  if(btn.classList.contains('learning')){
    btn.classList.remove('learning');
    btn.textContent='DESCRIBE';
    if(prog){prog.textContent='Sending...';prog.style.display='block';}
    sendCmd('learn_stop');
    setTimeout(loadTargets, 1500);
  } else {
    btn.classList.add('learning');
    btn.textContent='■ STOP';
    if(prog){prog.textContent='0 / 20';prog.style.display='block';}
    sendCmd('learn_start:'+id);
  }
}

function setEmbedCount(el, n){
  if(!el) return;
  el.textContent = '◉ ' + n + ' embed' + (n===1?'':'s');
  el.dataset.level = n >= 16 ? '2' : n >= 6 ? '1' : '0';
}

function highlightTarget(id){
  _activeTargetId = id;
  document.querySelectorAll('.tg-card').forEach(c=>{
    c.classList.toggle('active-target', +c.dataset.pid === id);
  });
}

// ── Stream: MJPEG for Chrome/Firefox/Android, snapshot polling for Safari/iOS ─
const isSafari = /^((?!chrome|android).)*safari/i.test(navigator.userAgent);
let snapshotTimer = null;

function startStream() {
  const img = document.getElementById('streamImg');
  if (!img) return;
  if (isSafari) {
    img.src = '/snapshot?' + Date.now();
  } else {
    img.src = '/stream?' + Date.now();
  }
}

function snapshotTick() {
  if (!isSafari || !streamOnline) return;
  const img = document.getElementById('streamImg');
  const next = new Image();
  next.onload  = () => { img.src = next.src; snapshotTimer = setTimeout(snapshotTick, 100); };
  next.onerror = () => { streamErr(); };
  next.src = '/snapshot?' + Date.now();
}

// ── Stream quality slider ───────────────────────────────────────────────────
let sqDebounce = null;
function onSQSlider(v) {
  document.getElementById('sqVal').textContent = v;
  clearTimeout(sqDebounce);
  sqDebounce = setTimeout(() => sendCmd('stream_quality:' + v), 300);
}

// ── Stream state ──────────────────────────────────────────────────────────
let streamOnline = false;
let streamRetryTimer = null;

function streamOk() {
  streamOnline = true;
  const img = document.getElementById('streamImg');
  const ph = document.getElementById('streamPlaceholder');
  const s = document.getElementById('streamStatus');
  if (!img) return;
  img.style.display = 'block';
  if (ph) ph.style.display = 'none';
  if (s) { s.textContent = 'LIVE'; s.style.color = 'var(--accent)'; }
  if (isSafari) { clearTimeout(snapshotTimer); snapshotTimer = setTimeout(snapshotTick, 100); }
}

function streamErr() {
  streamOnline = false;
  clearTimeout(snapshotTimer);
  const img = document.getElementById('streamImg');
  const ph = document.getElementById('streamPlaceholder');
  const s = document.getElementById('streamStatus');
  if (!img) return;
  img.style.display = 'none';
  if (ph) ph.style.display = 'flex';
  if (s) { s.textContent = 'NO SIGNAL'; s.style.color = 'var(--danger)'; }
  scheduleStreamRetry(5000);
}

function scheduleStreamRetry(delay) {
  clearTimeout(streamRetryTimer);
  streamRetryTimer = setTimeout(() => startStream(), delay);
}

// ── Depth stream ──────────────────────────────────────────────────────────
let depthRunning = true, depthOnline = false, depthSnapshotTimer = null, depthRetryTimer = null;

let dqDebounce = null;
function onDQSlider(v) {
  document.getElementById('dqVal').textContent = v;
  clearTimeout(dqDebounce);
  dqDebounce = setTimeout(() => sendCmd('depth_quality:' + v), 300);
}

function startDepthStream() {
  if (!depthRunning) return;
  const img = document.getElementById('depthImg');
  if (!img) return;
  if (isSafari) {
    img.src = '/depth_snapshot?' + Date.now();
  } else {
    img.src = '/depth_stream?' + Date.now();
  }
}

function depthSnapshotTick() {
  if (!isSafari || !depthRunning || !depthOnline) return;
  const img = document.getElementById('depthImg');
  const next = new Image();
  next.onload  = () => { img.src = next.src; depthSnapshotTimer = setTimeout(depthSnapshotTick, 100); };
  next.onerror = () => depthErr();
  next.src = '/depth_snapshot?' + Date.now();
}

function depthOk() {
  depthOnline = true;
  const img = document.getElementById('depthImg');
  const ph = document.getElementById('depthPlaceholder');
  const s = document.getElementById('depthStatus');
  if (!img) return;
  img.style.display = 'block';
  if (ph) ph.style.display = 'none';
  if (s) { s.textContent = 'LIVE'; s.style.color = 'var(--accent)'; }
  if (isSafari) depthSnapshotTick();
}

function depthErr() {
  depthOnline = false;
  clearTimeout(depthSnapshotTimer);
  const img = document.getElementById('depthImg');
  const ph = document.getElementById('depthPlaceholder');
  const s = document.getElementById('depthStatus');
  if (!img) return;
  img.style.display = 'none';
  if (ph) { ph.style.display = 'flex'; ph.innerHTML = '<div style="font-size:11px;letter-spacing:2px;color:var(--danger)">NO SIGNAL</div>'; }
  if (s) { s.textContent = 'NO SIGNAL'; s.style.color = 'var(--danger)'; }
  if (depthRunning) depthRetryTimer = setTimeout(startDepthStream, 4000);
}

// ── Stale-stream watchdog ─────────────────────────────────────────────────
let _staleCanvas, _staleCtx, _lastPixel = null, _staleCount = 0;
function initStaleWatchdog() {
  _staleCanvas = document.createElement('canvas');
  _staleCanvas.width = _staleCanvas.height = 4;
  _staleCtx = _staleCanvas.getContext('2d');
}
setInterval(() => {
  if (!streamOnline) return;
  const img = document.getElementById('streamImg');
  if (!img || !img.naturalWidth) return;
  try {
    _staleCtx.drawImage(img, 0, 0, 4, 4);
    const px = _staleCtx.getImageData(0, 0, 1, 1).data.join(',');
    if (px === _lastPixel) {
      _staleCount++;
      if (_staleCount >= 4) {
        console.warn('[stream] Stale frame detected — reconnecting');
        _staleCount = 0; _lastPixel = null;
        streamErr();
      }
    } else {
      _staleCount = 0; _lastPixel = px;
    }
  } catch {}
}, 2000);
