// ── Controls ───────────────────────────────────────────────────────────────
const WM=1000,PM=5000;
const spd=()=>+document.getElementById('ss').value/100;
const pspd=()=>+document.getElementById('ps').value/100;
const clamp=(v,lo,hi)=>Math.max(lo,Math.min(hi,v));
const wc=(l,r)=>sendCmd(`manual_wheel:${Math.round(clamp(l*WM*spd(),-WM,WM))}:${Math.round(clamp(r*WM*spd(),-WM,WM))}`);
const pc=(p,t)=>sendCmd(`manual_ptu:${Math.round(p*PM*pspd())}:${Math.round(t*PM*pspd())}`);
function eStop(){sendCmd('emergency_stop');sendCmd('manual_wheel:0:0');sendCmd('manual_ptu:0:0')}

const held={};
function holdBtn(id,press,rel){
  const b=document.getElementById(id);if(!b||b.dataset.bound)return;
  b.dataset.bound='1';
  const up=()=>{clearInterval(held[id]);delete held[id];b.classList.remove('held');if(rel)rel()};
  b.addEventListener('mousedown',e=>{e.preventDefault();b.classList.add('held');press();held[id]=setInterval(press,100)});
  b.addEventListener('mouseup',up);b.addEventListener('mouseleave',up);
  b.addEventListener('touchstart',e=>{e.preventDefault();b.classList.add('held');press();held[id]=setInterval(press,100)},{passive:false});
  b.addEventListener('touchend',up);
}
// Bind D-pad buttons — must be called after widget DOM is created (initDashboard)
function initControls() {
  holdBtn('bF',()=>wc(1,1),()=>wc(0,0));holdBtn('bB',()=>wc(-1,-1),()=>wc(0,0));
  holdBtn('bL',()=>wc(-.6,.6),()=>wc(0,0));holdBtn('bR',()=>wc(.6,-.6),()=>wc(0,0));
  holdBtn('bS',()=>wc(0,0));
  holdBtn('bPL',()=>pc(-1,0),()=>pc(0,0));holdBtn('bPR',()=>pc(1,0),()=>pc(0,0));
  holdBtn('bTU',()=>pc(0,1),()=>pc(0,0));holdBtn('bTD',()=>pc(0,-1),()=>pc(0,0));
  holdBtn('bPC',()=>sendCmd('manual_ptu:centre'));
}

const kd=new Set();
document.addEventListener('keydown',e=>{
  if(['INPUT','SELECT','TEXTAREA'].includes(document.activeElement.tagName))return;
  if(kd.has(e.code))return;kd.add(e.code);
  if(e.code==='KeyW')wc(1,1);else if(e.code==='KeyX')wc(-1,-1);
  else if(e.code==='KeyA')wc(-.6,.6);else if(e.code==='KeyD')wc(.6,-.6);
  else if(e.code==='Space'){wc(0,0);e.preventDefault();}
  else if(e.code==='ArrowLeft'){pc(-1,0);e.preventDefault();}
  else if(e.code==='ArrowRight'){pc(1,0);e.preventDefault();}
  else if(e.code==='ArrowUp'){pc(0,1);e.preventDefault();}
  else if(e.code==='ArrowDown'){pc(0,-1);e.preventDefault();}
  else if(e.code==='KeyC')sendCmd('manual_ptu:centre');
  else if(e.code==='Escape')exitAdd();
});
document.addEventListener('keyup',e=>{
  kd.delete(e.code);
  if(['KeyW','KeyX','KeyA','KeyD','Space'].includes(e.code))wc(0,0);
  if(['ArrowLeft','ArrowRight','ArrowUp','ArrowDown'].includes(e.code))pc(0,0);
});
window.addEventListener('blur',()=>{kd.clear();wc(0,0);pc(0,0);});
