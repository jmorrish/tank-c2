// ── Helpers ────────────────────────────��───────────────────────────────────
function esc(s){const d=document.createElement('div');d.textContent=s;return d.innerHTML}

// ── Shared state ───────────���──────────────────────────────────────────────
let lastGPS=null;

// ── Telemetry helpers (used by dashboard.js) ───────────────────────────���──
const f=(v,d=1,u='')=>(v==null||isNaN(v))?'—':v.toFixed(d)+(u?' '+u:'');
const sv2=(id,v)=>{const e=document.getElementById(id);if(e)e.textContent=v};
const cardOf=id=>{const e=document.getElementById(id);return e?e.closest('.grid-stack-item-content')||e.closest('.card'):null};
const setStale=(id,stale)=>{const c=cardOf(id);if(c)c.classList.toggle('stale',stale)};

// ── WebSocket — exponential backoff reconnect ──────────────────────��───────
let ws, wsDelay = 1000;
async function connectWS(){
  let tok = '';
  try {
    const r = await fetch('/api/ws-token');
    if (r.status === 401) { location.href = '/login'; return; }
    tok = (await r.json()).token;
  } catch { setTimeout(connectWS, wsDelay); wsDelay = Math.min(wsDelay * 1.5, 15000); return; }
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  ws = new WebSocket(`${proto}://${location.host}?tok=${tok}`);
  ws.onopen  = () => { wsDelay = 1000; };
  ws.onclose = () => { setTimeout(connectWS, wsDelay); wsDelay = Math.min(wsDelay * 1.5, 15000); };
  ws.onerror = () => {};
  ws.onmessage = e => { try { handle(JSON.parse(e.data)); } catch {} };
}
function sendCmd(c){if(ws?.readyState===1)ws.send(JSON.stringify({cmd:c}))}
function handle(m){
  if(m.type==='connection'){
    const on=m.status==='connected';
    document.getElementById('cb').className='conn-badge'+(on?' on':'');
    document.getElementById('ct').textContent=on?'ONLINE':'OFFLINE';
  } else if(m.type==='status'){
    updateTele(m.data);
  } else if(m.type==='new_target'){
    loadTargets();
  } else if(m.type==='learn_progress'){
    const card=document.querySelector(`.tg-card[data-pid="${m.target_id}"]`);
    if(card){
      const prog=card.querySelector('.tg-learn-prog');
      if(prog){prog.textContent=m.count+' / '+m.total;prog.style.display='block';}
    }
  } else if(m.type==='learn_status'){
    const card=document.querySelector(`.tg-card[data-pid="${m.target_id}"]`);
    if(card){
      const prog=card.querySelector('.tg-learn-prog');
      const btn=card.querySelector('.tg-learn');
      if(m.status==='analysing'){
        if(prog){prog.textContent='Asking AI (' +m.count+' crops)...';prog.style.display='block';prog.style.color='#9c6fff';}
      } else if(m.status==='error'){
        if(prog){prog.textContent='Error: '+m.message;prog.style.display='block';prog.style.color='#ff4444';}
        if(btn){btn.textContent='DESCRIBE';btn.classList.remove('learning');}
      }
    }
  } else if(m.type==='target_description'){
    const card=document.querySelector(`.tg-card[data-pid="${m.target_id}"]`);
    if(card){
      const desc=card.querySelector('.tg-desc');
      const btn=card.querySelector('.tg-learn');
      const prog=card.querySelector('.tg-learn-prog');
      if(desc){desc.textContent=m.description;desc.style.display='block';}
      if(btn){btn.textContent='DESCRIBE';btn.classList.remove('learning');}
      if(prog){prog.style.display='none';prog.style.color='';}
      card.classList.add('has-desc');
    }
    setTimeout(loadTargets, 500);
  } else if(m.type==='mission_status'){
    if(m.event==='executing') showBanner(m.name,'ok');
    else if(m.event==='completed'){
      console.log('[mission] completed:',m.name,'id:',m.missionId);
      hideBanner();clearMissionTarget();
    }
    else if(m.event==='aborted'){hideBanner();clearMissionTarget();}
    else if(m.event==='mission_faulted'){
      const fl={gps_lost:'⚠ GPS LOST',stuck:'⚠ STUCK',wheel_fail:'⚠ WHEEL FAULT'};
      console.warn('[mission] faulted:',m.fault,'mission:',m.name,'id:',m.missionId);
      showBanner(`${fl[m.fault]||'⚠ MISSION FAILED'} — ${m.name||''}`, 'fault');
      clearMissionTarget();
    }
    else if(m.event==='abort_pending') showBanner('ABORT QUEUED — RECONNECTING TO TANK','pending');
    else if(m.event==='pushed') console.log('[mission] pushed:',m.name);
  } else if(m.type==='scan'){
    updateRadar(m);
  } else if(m.type==='slam_map'){
    updateSlamMap(m);
  }
}

// ── Tab switching ────────────────���────────────────────────────────────────
function go(name){
  document.querySelectorAll('.tab-pane').forEach(p=>p.classList.remove('active'));
  document.querySelectorAll('.tab-btn').forEach(b=>b.classList.remove('active'));
  document.getElementById('tab-'+name).classList.add('active');
  document.querySelectorAll('.tab-btn').forEach(b=>{if(b.dataset.tab===name)b.classList.add('active')});
  if(name==='missions')loadLib();
  if(name==='planner')setTimeout(()=>map?.invalidateSize(),120);
  if(name==='dashboard'&&typeof grid!=='undefined'&&grid)setTimeout(()=>grid.onParentResize(),120);
  if(name==='docs'){const f=document.getElementById('docs-frame');if(f.src==='about:blank'||!f.src.includes('/docs/'))f.src='/docs/index.html';}
}
