// ── Map ────────────────────────────────────────────────────────────────────
let map,osmL,satL,satOn=false,addMode=false,tankMk=null,wpMks=[],wpPoly=null;
let userMk=null,userAccCircle=null,userLatLng=null;
let autoRouteEnabled=false,routeSegments=null,routeLayers=[],_routeTimer=null;
const COLORS=['#00e676','#00b0ff','#ffab00','#ff6e40','#e040fb','#40c4ff','#69f0ae','#ffd740','#ff6090','#a7ffeb'];

function initMap(){
  map=L.map('map',{zoomControl:true}).setView([20,0],2);
  osmL=L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{attribution:'© OSM',maxZoom:19}).addTo(map);
  satL=L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',{attribution:'© Esri',maxZoom:19});
  map.on('click',e=>{if(addMode){addWP(e.latlng.lat,e.latlng.lng);exitAdd()}});
  map.on('mousemove',e=>document.getElementById('mc').textContent=`${e.latlng.lat.toFixed(6)},  ${e.latlng.lng.toFixed(6)}`);
  locateUser();
}
function locateUser(){
  if(navigator.geolocation){
    navigator.geolocation.watchPosition(
      pos=>{
        const ll=[pos.coords.latitude,pos.coords.longitude];
        moveUser(ll, pos.coords.accuracy);
        if(!lastGPS) map.setView(ll, 15);
      },
      ()=>{ ipLocate(); },
      {enableHighAccuracy:true, timeout:10000, maximumAge:5000}
    );
  } else {
    ipLocate();
  }
}
function ipLocate(){
  fetch('https://ipapi.co/json/')
    .then(r=>r.json())
    .then(d=>{ if(!lastGPS && d.latitude) map.setView([d.latitude,d.longitude],12); })
    .catch(()=>{});
}
function moveUser(ll, accuracy){
  if(!map) return;
  userLatLng = ll;
  const icon = L.divIcon({className:'',iconSize:[18,18],iconAnchor:[9,9],
    html:'<div style="width:18px;height:18px;border-radius:50%;background:rgba(0,176,255,.25);border:2.5px solid #00b0ff;box-shadow:0 0 0 3px rgba(0,176,255,.15)"></div>'});
  if(!userMk){
    userMk = L.marker(ll,{icon, zIndexOffset:900, title:'Your location'}).addTo(map);
    userMk.bindTooltip('YOU', {permanent:false, direction:'top', className:'user-tip'});
  } else {
    userMk.setLatLng(ll);
  }
  if(accuracy > 20){
    if(!userAccCircle)
      userAccCircle = L.circle(ll,{radius:accuracy, color:'#00b0ff', weight:1, fillOpacity:.06}).addTo(map);
    else
      userAccCircle.setLatLng(ll).setRadius(accuracy);
  } else if(userAccCircle){
    map.removeLayer(userAccCircle); userAccCircle=null;
  }
  if(lastGPS) updateUserTankDist(ll);
}
function updateUserTankDist(userLL){
  const R=6371000, toRad=d=>d*Math.PI/180;
  const [lat1,lon1]=userLL, [lat2,lon2]=lastGPS;
  const dLat=toRad(lat2-lat1), dLon=toRad(lon2-lon1);
  const a=Math.sin(dLat/2)**2+Math.cos(toRad(lat1))*Math.cos(toRad(lat2))*Math.sin(dLon/2)**2;
  const dist=R*2*Math.atan2(Math.sqrt(a),Math.sqrt(1-a));
  const el=document.getElementById('userDist');
  if(!el) return;
  el.textContent = dist<1000 ? `YOU ↔ TANK  ${dist.toFixed(0)} m` : `YOU ↔ TANK  ${(dist/1000).toFixed(2)} km`;
  el.style.display='block';
}
function toggleSat(){satOn=!satOn;if(satOn){map.removeLayer(osmL);satL.addTo(map)}else{map.removeLayer(satL);osmL.addTo(map)}document.getElementById('satBtn').textContent=satOn?'MAP':'SATELLITE'}
function ctrTank(){if(lastGPS)map.setView(lastGPS,18)}
function ctrUser(){if(userLatLng)map.setView(userLatLng,17)}
function fitAll(){if(!wps.length)return;map.fitBounds(L.latLngBounds(wps.map(w=>[w.lat,w.lng])),{padding:[50,50]})}
function enterAdd(){addMode=true;document.getElementById('addbadge').classList.remove('hidden');map.getContainer().style.cursor='crosshair'}
function exitAdd(){addMode=false;document.getElementById('addbadge').classList.add('hidden');map.getContainer().style.cursor=''}
function moveTank(lat,lng){
  if(!map)return;
  const icon=L.divIcon({className:'',iconSize:[32,32],iconAnchor:[16,16],html:'<div style="width:32px;height:32px;border-radius:50%;background:rgba(0,230,118,.18);border:2px solid #00e676;display:flex;align-items:center;justify-content:center;font-size:18px">🚗</div>'});
  if(!tankMk){
    tankMk=L.marker([lat,lng],{icon,zIndexOffset:1000}).addTo(map);
    map.setView([lat,lng],17);
  } else tankMk.setLatLng([lat,lng]);
}
function mkIcon(i,action){
  const c=COLORS[i%COLORS.length];
  return L.divIcon({className:'',iconSize:[26,26],iconAnchor:[7,26],html:`<div style="width:26px;height:26px;border-radius:50% 50% 50% 0;transform:rotate(-45deg);background:${c};border:2px solid rgba(0,0,0,.4);display:flex;align-items:center;justify-content:center;box-shadow:0 2px 6px rgba(0,0,0,.5)"><span style="transform:rotate(45deg);font-size:11px;font-weight:bold;color:#0a0e1a">${i+1}</span></div>`});
}
function redrawMap(){
  wpMks.forEach(m=>map.removeLayer(m));wpMks=[];
  if(wpPoly){map.removeLayer(wpPoly);wpPoly=null}
  routeLayers.forEach(l=>map.removeLayer(l));routeLayers=[];
  wps.forEach((wp,i)=>{
    const mk=L.marker([wp.lat,wp.lng],{icon:mkIcon(i,wp.arrivalAction),draggable:true}).addTo(map)
      .bindPopup(`<b style="color:var(--accent)">WP${i+1}: ${esc(wp.name)}</b><br><span style="color:var(--muted);font-size:11px">${wp.lat.toFixed(5)}, ${wp.lng.toFixed(5)}</span><br><span style="color:var(--warn);font-size:11px">▸ ${esc(wp.arrivalAction||'continue')}</span>`);
    mk.on('click',()=>selWP(i));
    mk.on('dragend',e=>{const ll=e.target.getLatLng();wps[i].lat=+ll.lat.toFixed(7);wps[i].lng=+ll.lng.toFixed(7);if(sel===i){document.getElementById('wLa').value=wps[i].lat;document.getElementById('wLg').value=wps[i].lng}redrawMap();scheduleRouteRefresh()});
    wpMks.push(mk);
  });
  if(wps.length>1&&!autoRouteEnabled){const pts=wps.map(w=>[w.lat,w.lng]);if(document.getElementById('mRt').checked)pts.push(pts[0]);wpPoly=L.polyline(pts,{color:'#00e676',weight:2,opacity:.55,dashArray:'6,5'}).addTo(map)}
  if(autoRouteEnabled&&routeSegments) drawRouteLayer();
}

// ── Waypoints ──────────────────────────────────────────────────────────────
let wps=[],sel=-1,ctr=1;
const BDGE={continue:'bm MOVE',wait:'bw WAIT',scan:'bs SCAN',follow:'bf FLLOW',set_ptu:'bp PTU',custom:'bc CMD'};

function addWP(lat,lng){
  const wp={id:'wp_'+(ctr++),name:`WP ${wps.length+1}`,lat:+lat.toFixed(7),lng:+lng.toFixed(7),speed:+document.getElementById('mSp').value,arrivalRadius:3,arrivalAction:'continue',waitSeconds:5,scanFrom:-90,scanTo:90,scanSpeed:200,followDuration:0,ptuPan:0,ptuTilt:0,customCommand:'',notes:''};
  wps.push(wp);redrawList();redrawMap();selWP(wps.length-1);
  if(map&&wp.lat&&wp.lng)map.panTo([wp.lat,wp.lng]);
  scheduleRouteRefresh();
}
function removeWP(i){wps.splice(i,1);if(sel>=wps.length)sel=wps.length-1;redrawList();redrawMap();if(sel>=0)selWP(sel);else document.getElementById('wed').classList.add('hidden');scheduleRouteRefresh()}
function selWP(i){
  sel=i;
  document.querySelectorAll('.wp-item').forEach((e,j)=>e.classList.toggle('sel',j===i));
  const wp=wps[i];if(!wp)return;
  document.getElementById('wed').classList.remove('hidden');
  document.getElementById('wen').textContent=i+1;
  document.getElementById('wNm').value=wp.name;document.getElementById('wLa').value=wp.lat;document.getElementById('wLg').value=wp.lng;
  document.getElementById('wSp').value=wp.speed||60;document.getElementById('wSv').textContent=(wp.speed||60)+'%';
  document.getElementById('wRd').value=wp.arrivalRadius||3;document.getElementById('wAc').value=wp.arrivalAction||'continue';
  document.getElementById('wWt').value=wp.waitSeconds||5;document.getElementById('wSF').value=wp.scanFrom??-90;
  document.getElementById('wST').value=wp.scanTo??90;document.getElementById('wSS').value=wp.scanSpeed||200;
  document.getElementById('wFD').value=wp.followDuration||0;document.getElementById('wPP').value=wp.ptuPan||0;
  document.getElementById('wPT').value=wp.ptuTilt||0;document.getElementById('wCC').value=wp.customCommand||'';
  document.getElementById('wNo').value=wp.notes||'';showAF();
}
function syncWP(){
  if(sel<0||sel>=wps.length)return;const wp=wps[sel];
  wp.name=document.getElementById('wNm').value;wp.speed=+document.getElementById('wSp').value;
  wp.arrivalRadius=+document.getElementById('wRd').value||3;wp.arrivalAction=document.getElementById('wAc').value;
  wp.waitSeconds=+document.getElementById('wWt').value||5;wp.scanFrom=+document.getElementById('wSF').value||-90;
  wp.scanTo=+document.getElementById('wST').value||90;wp.scanSpeed=+document.getElementById('wSS').value||200;
  wp.followDuration=+document.getElementById('wFD').value||0;wp.ptuPan=+document.getElementById('wPP').value||0;
  wp.ptuTilt=+document.getElementById('wPT').value||0;wp.customCommand=document.getElementById('wCC').value;
  wp.notes=document.getElementById('wNo').value;redrawList();redrawMap();
}
function syncCoords(){
  if(sel<0)return;const lat=+document.getElementById('wLa').value,lng=+document.getElementById('wLg').value;
  if(!isNaN(lat)&&!isNaN(lng)){wps[sel].lat=lat;wps[sel].lng=lng;redrawMap();scheduleRouteRefresh()}
}
function showAF(){
  const v=document.getElementById('wAc').value;
  ['wait','scan','follow','set_ptu','custom'].forEach(k=>document.getElementById('af-'+k).classList.toggle('show',k===v));
}
function redrawList(){
  const c=document.getElementById('wpl');
  c.querySelectorAll('.wp-item').forEach(e=>e.remove());
  document.getElementById('wpe').style.display=wps.length?'none':'block';
  document.getElementById('wpc').textContent=wps.length;
  wps.forEach((wp,i)=>{
    const el=document.createElement('div');el.className='wp-item'+(i===sel?' sel':'');
    const[bc,bt]=(BDGE[wp.arrivalAction||'continue']||'bm MOVE').split(' ');
    el.innerHTML=`<span class="wph">⠿</span><span class="wpn">${i+1}</span><span class="wpname">${wp.name}</span><span class="badge ${bc}">${bt}</span><button class="wpdel" onclick="event.stopPropagation();removeWP(${i})">✕</button>`;
    el.addEventListener('click',()=>selWP(i));c.appendChild(el);
  });
  if(window._srt)window._srt.destroy();
  window._srt=Sortable.create(c,{handle:'.wph',animation:120,onEnd:ev=>{const m=wps.splice(ev.oldIndex,1)[0];wps.splice(ev.newIndex,0,m);sel=ev.newIndex;redrawList();redrawMap();scheduleRouteRefresh()}});
}

// ── Road Routing ─────────────────────────────────────────────────────────────
function toggleAutoRoute(){
  autoRouteEnabled=!autoRouteEnabled;
  const btn=document.getElementById('routeBtn');
  const sel=document.getElementById('routeProfile');
  btn.classList.toggle('rt-on',autoRouteEnabled);
  sel.style.display=autoRouteEnabled?'block':'none';
  if(autoRouteEnabled){
    fetchRoute();
  } else {
    routeSegments=null;
    routeLayers.forEach(l=>map.removeLayer(l));routeLayers=[];
    document.getElementById('routeStats').style.display='none';
    redrawMap();
  }
}

function scheduleRouteRefresh(){
  if(!autoRouteEnabled)return;
  clearTimeout(_routeTimer);
  _routeTimer=setTimeout(fetchRoute,700);
}

async function fetchRoute(){
  if(!autoRouteEnabled||wps.length<2){
    routeLayers.forEach(l=>map.removeLayer(l));routeLayers=[];
    document.getElementById('routeStats').style.display='none';
    return;
  }
  const statsEl=document.getElementById('routeStats');
  statsEl.textContent='Routing…';statsEl.style.display='block';
  const profile=document.getElementById('routeProfile').value;
  const pairs=[];
  for(let i=0;i<wps.length-1;i++) pairs.push([wps[i],wps[i+1]]);
  if(document.getElementById('mRt').checked&&wps.length>=2) pairs.push([wps[wps.length-1],wps[0]]);

  const segments=[];
  let totalDist=0,totalDur=0,anyFailed=false;
  for(const[a,b] of pairs){
    try{
      const r=await fetch('/api/route',{method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({points:[{lat:a.lat,lng:a.lng},{lat:b.lat,lng:b.lng}],profile})});
      const d=await r.json();
      if(d.points){segments.push(d.points);totalDist+=d.distance||0;totalDur+=d.duration||0;}
      else{segments.push(null);anyFailed=true;}
    }catch{segments.push(null);anyFailed=true;}
  }
  routeSegments=segments;
  drawRouteLayer();
  const km=(totalDist/1000).toFixed(2);
  const mins=Math.round(totalDur/60);
  const exp=buildExpandedWaypoints().length;
  statsEl.textContent=`${km} km · ~${mins} min · ${exp} wpts sent${anyFailed?' · ⚠ some segs straight-line':''}`;
}

function drawRouteLayer(){
  routeLayers.forEach(l=>map.removeLayer(l));routeLayers=[];
  if(!routeSegments)return;
  routeSegments.forEach(seg=>{
    if(!seg)return;
    const l=L.polyline(seg.map(p=>[p.lat,p.lng]),{color:'#ff9800',weight:4,opacity:.8,lineJoin:'round'}).addTo(map);
    routeLayers.push(l);
  });
}

function buildExpandedWaypoints(){
  if(!routeSegments||routeSegments.length<wps.length-1) return wps;
  const defSpeed=+document.getElementById('mSp').value;
  const expanded=[];
  for(let i=0;i<wps.length;i++){
    if(i>0&&routeSegments[i-1]){
      const seg=routeSegments[i-1];
      for(let j=1;j<seg.length-1;j++){
        expanded.push({id:`_r${i-1}_${j}`,name:`Route ${i}.${j}`,
          lat:seg[j].lat,lng:seg[j].lng,
          speed:wps[i-1].speed||defSpeed,arrivalRadius:5,arrivalAction:'continue',
          waitSeconds:5,scanFrom:-90,scanTo:90,scanSpeed:200,
          followDuration:0,ptuPan:0,ptuTilt:0,customCommand:'',notes:''});
      }
    }
    expanded.push(wps[i]);
  }
  if(document.getElementById('mRt').checked&&routeSegments.length===wps.length){
    const seg=routeSegments[wps.length-1];
    if(seg) for(let j=1;j<seg.length-1;j++){
      expanded.push({id:`_rret_${j}`,name:`Return ${j}`,
        lat:seg[j].lat,lng:seg[j].lng,
        speed:wps[0].speed||defSpeed,arrivalRadius:5,arrivalAction:'continue',
        waitSeconds:5,scanFrom:-90,scanTo:90,scanSpeed:200,
        followDuration:0,ptuPan:0,ptuTilt:0,customCommand:'',notes:''});
    }
  }
  return expanded;
}

// ── Mission CRUD ───────────────────────────────────────────────────────────
let curId=null;
function getMD(){return{id:curId,name:document.getElementById('mN').value||'Unnamed',type:document.getElementById('mT').value,loop:document.getElementById('mLp').checked,returnToStart:document.getElementById('mRt').checked,defaultSpeed:+document.getElementById('mSp').value,waypoints:wps}}
async function saveM(){
  const m=getMD();
  const r=await fetch(m.id?`/api/missions/${m.id}`:`/api/missions`,{method:m.id?'PUT':'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(m)});
  const j=await r.json();if(!m.id&&j.id)curId=j.id;
  alert(`Saved: ${m.name}`);
}
async function pushCurrent(btn){
  if(!wps.length){alert('Add at least one waypoint first.');return}
  await saveM();if(!curId)return;
  const orig=btn.textContent;btn.textContent='…';btn.disabled=true;
  try{
    const body=autoRouteEnabled&&routeSegments?{waypoints:buildExpandedWaypoints()}:{};
    const r=await fetch(`/api/missions/${curId}/push`,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
    const j=await r.json();
    btn.textContent=j.ok?'✓ SENT':'⚠ OFFLINE';
    setTimeout(()=>{btn.textContent=orig;btn.disabled=false},2000);
  }catch{btn.textContent=orig;btn.disabled=false}
}
async function runCurrent(){
  if(!wps.length){alert('Add at least one waypoint first.');return}
  await saveM();if(!curId)return;
  const body=autoRouteEnabled&&routeSegments?{waypoints:buildExpandedWaypoints()}:{};
  const r=await fetch(`/api/missions/${curId}/execute`,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  const j=await r.json();
  if(j.ok)showBanner(document.getElementById('mN').value);
  else alert('Mission queued — Jetson will execute when connected.');
}
function newM(){
  curId=null;wps=[];sel=-1;ctr=1;
  document.getElementById('mN').value='New Mission';document.getElementById('mT').value='patrol';
  document.getElementById('mLp').checked=false;document.getElementById('mRt').checked=false;
  document.getElementById('mSp').value=60;document.getElementById('mSv').textContent='60%';
  document.getElementById('wed').classList.add('hidden');redrawList();redrawMap();
}
function loadMD(m){
  curId=m.id;wps=m.waypoints||[];ctr=wps.length+1;sel=-1;
  document.getElementById('mN').value=m.name||'';document.getElementById('mT').value=m.type||'custom';
  document.getElementById('mLp').checked=!!m.loop;document.getElementById('mRt').checked=!!m.returnToStart;
  document.getElementById('mSp').value=m.defaultSpeed||60;document.getElementById('mSv').textContent=(m.defaultSpeed||60)+'%';
  document.getElementById('wed').classList.add('hidden');redrawList();redrawMap();
  if(wps.length)setTimeout(fitAll,300);go('planner');
}
document.getElementById('mRt').addEventListener('change',()=>{redrawMap();scheduleRouteRefresh()});

// ── Library ────────────────────────────────────────────────────────────────
async function resumeById(id,btn){
  const chk=await fetch(`/api/missions/${id}`);
  if(!chk.ok){alert('Mission not found on Jetson — push it first.');return}
  sendCmd(`mission_resume:${id}`);
  const name=btn.closest('tr').querySelector('td').textContent;
  showBanner(name,'ok');
}
async function loadLib(){
  const tb=document.getElementById('libbody');
  tb.innerHTML='<tr><td colspan="5" style="color:var(--muted);padding:16px;text-align:center">Loading…</td></tr>';
  const [list, stateRes] = await Promise.all([
    fetch('/api/missions').then(r=>r.json()),
    fetch('/api/missions/state').then(r=>r.ok?r.json():null).catch(()=>null)
  ]);
  const savedState = stateRes || {};
  list.forEach(m=>{
    if(savedState.mission_id===m.id && savedState.waypoint_idx>0){
      m._resumable=true; m._savedIdx=savedState.waypoint_idx;
    }
  });
  if(!list.length){tb.innerHTML='<tr><td colspan="5" style="color:var(--muted);padding:16px;text-align:center">No missions saved yet.</td></tr>';return}
  const tcls={patrol:'tp',search:'ts',escort:'te',surveillance:'tv2',custom:'tc'};
  tb.innerHTML=list.map(m=>`<tr>
    <td>${m.name}</td>
    <td><span class="tbadge ${tcls[m.type]||'tc'}">${(m.type||'custom').toUpperCase()}</span></td>
    <td>${m.waypointCount}</td>
    <td>${m.updated?new Date(m.updated).toLocaleString():'—'}</td>
    <td><div class="rowact">
      <button class="btn btn-accent btn-sm" onclick="loadById('${m.id}')">EDIT</button>
      <button class="btn btn-warn   btn-sm" onclick="pushById('${m.id}',this)">⬆ PUSH</button>
      <button class="btn btn-blue   btn-sm" onclick="runById('${m.id}',this)">▶ RUN</button>
      ${m._resumable?`<button class="btn btn-purple btn-sm" onclick="resumeById('${m.id}',this)" title="Resume from last saved waypoint WP${m._savedIdx}">↺ RESUME WP${m._savedIdx}</button>`:''}
      <button class="btn btn-muted  btn-sm" onclick="expM('${m.id}','${m.name}')">↓ EXP</button>
      <button class="btn btn-danger btn-sm" onclick="delM('${m.id}',this)">DEL</button>
    </div></td>
  </tr>`).join('');
}
async function loadById(id){const m=await fetch(`/api/missions/${id}`).then(r=>r.json());loadMD(m)}
async function pushById(id,btn){
  const orig=btn.textContent;btn.textContent='…';btn.disabled=true;
  try{
    const r=await fetch(`/api/missions/${id}/push`,{method:'POST'});
    const j=await r.json();
    btn.textContent=j.ok?'✓ SENT':'⚠ OFFLINE';
    setTimeout(()=>{btn.textContent=orig;btn.disabled=false},2000);
  }catch{btn.textContent=orig;btn.disabled=false}
}
async function runById(id,btn){
  const r=await fetch(`/api/missions/${id}/execute`,{method:'POST'});const j=await r.json();
  const name=btn.closest('tr').querySelector('td').textContent;
  if(j.ok)showBanner(name);else alert('Jetson appears offline — mission queued.');
}
async function delM(id,btn){if(!confirm('Delete this mission?'))return;await fetch(`/api/missions/${id}`,{method:'DELETE'});btn.closest('tr').remove()}
async function expM(id,name){
  const m=await fetch(`/api/missions/${id}`).then(r=>r.json());
  const a=document.createElement('a');
  a.href=URL.createObjectURL(new Blob([JSON.stringify(m,null,2)],{type:'application/json'}));
  a.download=`mission_${(name||id).replace(/\s+/g,'_')}.json`;a.click();
}
function importM(inp){
  const f=inp.files[0];if(!f)return;
  const rd=new FileReader();
  rd.onload=async e=>{try{const m=JSON.parse(e.target.result);m.id=null;await fetch('/api/missions',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(m)});alert('Imported: '+m.name);loadLib()}catch{alert('Invalid mission file.')}};
  rd.readAsText(f);inp.value='';
}

// ── Banner ─────────────────────────────────────────────────────────────────
function showBanner(name,state='ok'){
  const mb=document.getElementById('mb');
  mb.classList.add('show');
  mb.classList.toggle('fault',  state==='fault');
  mb.classList.toggle('pending',state==='pending');
  document.getElementById('mbt').textContent=(name||'').toUpperCase();
  document.getElementById('skipBtn').style.display=(state==='ok')?'':'none';
}
function hideBanner(){
  const mb=document.getElementById('mb');
  mb.classList.remove('show','fault','pending');
  document.getElementById('skipBtn').style.display='none';
}
async function abortM(){
  const r=await fetch('/api/missions/abort',{method:'POST'});
  const j=await r.json();
  if(j.ok){hideBanner();clearMissionTarget();}
}
async function skipWP(){
  sendCmd('mission_skip_wp');
}

// ── Mission target map marker ───────────────────────────────────────────────
let missionTargetMk=null;
function moveMissionTarget(lat,lon){
  if(!map)return;
  const icon=L.divIcon({className:'',iconSize:[20,20],iconAnchor:[10,10],html:'<div style="width:20px;height:20px;border-radius:50%;background:rgba(255,171,0,.25);border:2px solid #ffab00;box-shadow:0 0 8px #ffab00;animation:blink 1s infinite"></div>'});
  if(!missionTargetMk)missionTargetMk=L.marker([lat,lon],{icon,zIndexOffset:900}).addTo(map);
  else missionTargetMk.setLatLng([lat,lon]);
}
function clearMissionTarget(){if(missionTargetMk&&map){map.removeLayer(missionTargetMk);missionTargetMk=null}}
