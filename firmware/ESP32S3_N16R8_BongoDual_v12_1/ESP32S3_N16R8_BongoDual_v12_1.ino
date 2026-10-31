/*
  SONIC BONGO / PRISMA
  ESP32-S3 N16R8 - Dual Sensor Bongo v12.1 CALIBRATION FIX

  CONFIRMED WIRING (unchanged):
    S1: GPIO13 -> TRIG direct
        HC-SR04 ECHO -> TXS B1 -> A1 -> GPIO14
    S2: GPIO15 -> TRIG direct
        HC-SR04 ECHO -> TXS B2 -> A2 -> GPIO16

  v12.1 fixes:
    - Telemetry refresh NEVER overwrites sliders while the user edits.
    - Sliders sync only on first load, explicit LOAD CURRENT, or after APPLY.
    - Dirty/pending state is visible.
    - START auto-applies pending settings before acquisition resumes.
    - Baseline resolution is 0.1 cm.
    - Sound test is independent from HIT detection.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

WebServer server(80);
static const char* FW_VERSION = "12.1-calibration-fix";
static const char* AP_SSID = "PRISMA-S3-DIAG";

struct SensorPins { uint8_t trig; uint8_t echo; };
const SensorPins PINS[2] = {{13,14},{15,16}};

float baselineCm = 1.0f;
float maxRangeCm = 50.0f;
float hitPlaneCm = 12.0f;
float hitSpeedCmS = 35.0f;
float rearmMarginCm = 3.0f;
float minCoherencePct = 18.0f;
float displayStepCm = 0.5f;
float coneHalfAngleDeg = 7.5f;
float fastAlpha = 0.72f;
float stableAlpha = 0.20f;
float sensorVelAlpha = 0.34f;
float pairVelAlpha = 0.42f;
float maxJumpCm = 20.0f;
uint32_t slotMs = 30;
uint32_t minHitIntervalMs = 100;
bool acquisitionRunning = true;

struct SensorState {
  bool have=false, ok=false, inRange=false;
  uint32_t lastGoodMs=0, pulseUs=0;
  float rawCm=0, medianCm=0, fastCm=0, stableCm=0, displayCm=0;
  float prevFastCm=0, velocityCmS=0, jitterCm=0;
  uint32_t prevFastMs=0;
  float hist[3]={0,0,0};
  uint8_t histCount=0, histHead=0;
  uint32_t valid=0, timeouts=0, rejected=0, validMark=0;
  float hz=0;
};
SensorState s[2];

struct PairState {
  bool valid=false, geoValid=false, armed=true;
  float zFast=0, zStable=0, zDisplay=0, prevZFast=0, velocity=0;
  uint32_t prevZMs=0;
  float delta=0, x=0, coherence=0, jitter=0;
  String lateral="INDETERMINADO";
  float lateralConfidence=0;
  uint32_t hits=0, lastHitMs=0, eventSeq=0;
  float lastHitVelocity=0, lastHitZ=0;
  int lastMidi=0;
  String lastZone="--";
  uint32_t pairUpdates=0, pairMark=0;
  float combinedHz=0;
};
PairState pair;

uint8_t nextSensor=0;
uint32_t nextMeasureMs=0, rateMarkMs=0;
String logBuffer;

float clampf(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);} 
uint32_t clampu(uint32_t v,uint32_t lo,uint32_t hi){return v<lo?lo:(v>hi?hi:v);} 
float quantize(float v,float step){return step<=0.001f?v:roundf(v/step)*step;}

void addLog(const String& x){
  String l="["+String(millis())+" ms] "+x+"\n";
  logBuffer+=l;
  if(logBuffer.length()>16000)logBuffer.remove(0,2500);
  Serial.print(l);
}

uint32_t echoTimeoutUs(){
  float u=maxRangeCm*58.3f+1200.0f;
  return (uint32_t)clampf(u,2500.0f,12000.0f);
}
uint32_t staleMs(){return clampu(slotMs*6,160,700);}

float med3(float a,float b,float c){
  if(a>b){float t=a;a=b;b=t;} if(b>c){float t=b;b=c;c=t;} if(a>b){float t=a;a=b;b=t;} return b;
}
float pushMedian(SensorState &q,float raw){
  q.hist[q.histHead]=raw; q.histHead=(q.histHead+1)%3; if(q.histCount<3)q.histCount++;
  if(q.histCount==1)return q.hist[0];
  if(q.histCount==2)return 0.5f*(q.hist[0]+q.hist[1]);
  return med3(q.hist[0],q.hist[1],q.hist[2]);
}
int midiFromVelocity(float v){
  float x=clampf((v-20.0f)/220.0f,0,1); return 1+(int)lroundf(x*126.0f);
}

void updateLateral(){
  float d=pair.delta,a=fabsf(d);
  if(a<=0.20f){pair.lateral="CENTRO";pair.lateralConfidence=clampf((0.20f-a)/0.20f*100.0f,0,100);}
  else if(a>=0.45f){pair.lateral=d<0?"S1 / IZQUIERDA":"S2 / DERECHA";pair.lateralConfidence=clampf(30.0f+(a-0.45f)*80.0f,0,100);}
  else {pair.lateral="INDETERMINADO";pair.lateralConfidence=0;}
}

void updateRates(){
  uint32_t now=millis();
  if(!rateMarkMs){rateMarkMs=now;for(int i=0;i<2;i++)s[i].validMark=s[i].valid;pair.pairMark=pair.pairUpdates;return;}
  uint32_t dt=now-rateMarkMs; if(dt<1000)return; float sec=dt/1000.0f;
  for(int i=0;i<2;i++){s[i].hz=(s[i].valid-s[i].validMark)/sec;s[i].validMark=s[i].valid;}
  pair.combinedHz=(pair.pairUpdates-pair.pairMark)/sec; pair.pairMark=pair.pairUpdates; rateMarkMs=now;
}

void resetTracking(bool keepHits=true){
  for(int i=0;i<2;i++){
    s[i].have=false;s[i].ok=false;s[i].inRange=false;s[i].histCount=0;s[i].histHead=0;s[i].velocityCmS=0;s[i].jitterCm=0;
  }
  pair.valid=false;pair.geoValid=false;pair.armed=true;pair.prevZMs=0;pair.velocity=0;pair.jitter=0;
  if(!keepHits){pair.hits=0;pair.eventSeq=0;pair.lastHitMs=0;pair.lastHitVelocity=0;pair.lastHitZ=0;pair.lastMidi=0;pair.lastZone="--";}
}

void updatePair(){
  uint32_t now=millis(), stale=staleMs();
  bool f0=s[0].ok&&s[0].inRange&&(now-s[0].lastGoodMs<=stale);
  bool f1=s[1].ok&&s[1].inRange&&(now-s[1].lastGoodMs<=stale);
  pair.valid=f0&&f1; if(!pair.valid)return;

  float maxPredict=clampf(slotMs/1000.0f*1.5f,0.03f,0.12f);
  float age0=clampf((now-s[0].lastGoodMs)/1000.0f,0,maxPredict);
  float age1=clampf((now-s[1].lastGoodMs)/1000.0f,0,maxPredict);
  float p0=s[0].fastCm+clampf(s[0].velocityCmS*age0,-5,5);
  float p1=s[1].fastCm+clampf(s[1].velocityCmS*age1,-5,5);
  float zNew=0.5f*(p0+p1);

  pair.zStable=0.5f*(s[0].stableCm+s[1].stableCm);
  pair.zDisplay=quantize(pair.zStable,displayStepCm);
  pair.delta=s[0].stableCm-s[1].stableCm;
  pair.jitter=0.5f*(s[0].jitterCm+s[1].jitterCm);

  if(pair.prevZMs&&now>pair.prevZMs){
    float dt=(now-pair.prevZMs)/1000.0f;
    if(dt>0.008f&&dt<0.30f){
      float rv=clampf((zNew-pair.prevZFast)/dt,-600,600);
      pair.velocity=pairVelAlpha*rv+(1.0f-pairVelAlpha)*pair.velocity;
    }
  }
  pair.zFast=zNew;pair.prevZFast=zNew;pair.prevZMs=now;pair.pairUpdates++;
  pair.coherence=clampf(100.0f-fabsf(pair.delta)*22.0f-pair.jitter*10.0f,0,100);

  pair.geoValid=false;pair.x=0;
  if(baselineCm>0.01f&&fabsf(pair.delta)<=baselineCm){
    float d1=s[0].stableCm,d2=s[1].stableCm;
    float x=(d1*d1-d2*d2)/(2.0f*baselineCm);
    float z2=d1*d1-(x+baselineCm*0.5f)*(x+baselineCm*0.5f);
    if(z2>=0){pair.x=x;pair.geoValid=true;}
  }
  updateLateral();

  bool intervalOK=(now-pair.lastHitMs)>=minHitIntervalMs;
  if(pair.armed&&pair.zFast<=hitPlaneCm&&pair.velocity<=-hitSpeedCmS&&pair.coherence>=minCoherencePct&&intervalOK){
    pair.hits++;pair.eventSeq++;pair.lastHitMs=now;pair.lastHitVelocity=-pair.velocity;pair.lastHitZ=pair.zFast;
    pair.lastMidi=midiFromVelocity(pair.lastHitVelocity);pair.lastZone=pair.lateral;pair.armed=false;
    addLog("HIT seq="+String(pair.eventSeq)+" z="+String(pair.lastHitZ,2)+" v="+String(pair.lastHitVelocity,1));
  }
  if(!pair.armed&&intervalOK&&pair.zFast>=hitPlaneCm+rearmMarginCm)pair.armed=true;
}

void processPulse(uint8_t i,uint32_t pulse){
  SensorState &q=s[i];uint32_t now=millis();float raw=((float)pulse*0.0343f)/2.0f;
  q.pulseUs=pulse;q.rawCm=raw;
  if(raw<2.0f||raw>maxRangeCm){q.ok=false;q.inRange=false;q.rejected++;return;}
  q.ok=true;q.inRange=true;q.lastGoodMs=now;q.valid++;
  float med=pushMedian(q,raw);q.medianCm=med;
  if(!q.have){q.have=true;q.fastCm=raw;q.stableCm=med;q.displayCm=quantize(med,displayStepCm);q.prevFastCm=raw;q.prevFastMs=now;q.velocityCmS=0;q.jitterCm=0;return;}

  float guarded=raw,diff=raw-q.fastCm;
  if(diff>maxJumpCm)guarded=q.fastCm+maxJumpCm;if(diff<-maxJumpCm)guarded=q.fastCm-maxJumpCm;
  float fastNew=fastAlpha*guarded+(1.0f-fastAlpha)*q.fastCm;
  q.stableCm=stableAlpha*med+(1.0f-stableAlpha)*q.stableCm;
  q.displayCm=quantize(q.stableCm,displayStepCm);
  q.jitterCm=0.90f*q.jitterCm+0.10f*fabsf(raw-med);
  if(now>q.prevFastMs){float dt=(now-q.prevFastMs)/1000.0f;if(dt>0.025f&&dt<0.35f){float rv=clampf((fastNew-q.prevFastCm)/dt,-600,600);q.velocityCmS=sensorVelAlpha*rv+(1.0f-sensorVelAlpha)*q.velocityCmS;}}
  q.fastCm=fastNew;q.prevFastCm=fastNew;q.prevFastMs=now;
}

void measureOne(uint8_t i){
  const SensorPins&p=PINS[i];digitalWrite(p.trig,LOW);delayMicroseconds(3);digitalWrite(p.trig,HIGH);delayMicroseconds(10);digitalWrite(p.trig,LOW);
  uint32_t pulse=pulseIn(p.echo,HIGH,echoTimeoutUs());
  if(pulse)processPulse(i,pulse);else{s[i].ok=false;s[i].inRange=false;s[i].pulseUs=0;s[i].timeouts++;}
  updatePair();updateRates();
}
void serviceSensors(){
  if(!acquisitionRunning)return;uint32_t now=millis();if((int32_t)(now-nextMeasureMs)<0)return;
  uint8_t i=nextSensor;nextSensor^=1;measureOne(i);nextMeasureMs=millis()+slotMs;
}
String motionLabel(){if(!pair.valid)return"SIN PAR";if(pair.velocity<=-18)return"ACERCANDO";if(pair.velocity>=18)return"ALEJANDO";return"ESTABLE";}

const char PAGE[] PROGMEM=R"HTML(
<!doctype html><html lang="es"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SONIC BONGO v12.1</title><style>
:root{--bg:#070c12;--panel:#0e1821;--line:#263846;--text:#ecf4fb;--muted:#8296a7;--a:#5be8cd;--b:#739fff;--hit:#ffc65a;--ok:#48df86;--bad:#ff6868;--edit:#69b9ff}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font-family:system-ui,-apple-system,Segoe UI,sans-serif}.wrap,header{max-width:1200px;margin:auto;padding:12px}header{display:flex;justify-content:space-between;gap:12px;align-items:center}h1{font-size:20px;margin:0}.sub{font-size:10px;color:var(--muted)}.pill{padding:5px 8px;border:1px solid var(--line);border-radius:999px;font-size:10px;font-weight:800}.on{color:var(--ok)}.off{color:var(--bad)}.pending{color:var(--hit)}.cards{display:grid;grid-template-columns:repeat(6,1fr);gap:7px}.card,.panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:10px}.k{font-size:9px;color:var(--muted);letter-spacing:.07em}.v{font-size:19px;font-weight:900;margin-top:3px}.a{color:var(--a)}.b{color:var(--b)}.hit{color:var(--hit)}.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:8px}.row{display:flex;justify-content:space-between;gap:12px;margin:5px 0;font-size:11px}.row b{font-size:12px}.controls{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:8px}.ctrl{padding:8px;border:1px solid #223441;border-radius:8px;background:#09121a}.ctrlHead{display:flex;justify-content:space-between;font-size:10px}.ctrl input{width:100%;margin-top:7px;accent-color:#6bbdff}.ctrl.disabled{opacity:.35}.buttons{display:flex;gap:7px;flex-wrap:wrap;margin-top:9px}button{padding:9px 11px;border:1px solid #3a5265;border-radius:8px;background:#132330;color:#fff;font-weight:800}.primary{background:#123c2b;border-color:#2a7752}.danger{background:#35191b;border-color:#70383d}.blue{background:#152b40;border-color:#376487}.status{font-size:10px;color:var(--muted);margin-top:7px;min-height:16px}canvas{width:100%;height:260px;background:#050a0e;border:1px solid #1d2d38;border-radius:8px}.meter{height:7px;background:#071017;border:1px solid #1e303c;border-radius:8px;overflow:hidden}.meter i{display:block;height:100%;background:#55d68f;width:0}@media(max-width:900px){.cards{grid-template-columns:repeat(2,1fr)}.grid{grid-template-columns:1fr}.controls{grid-template-columns:1fr}}
</style></head><body>
<header><div><h1>SONIC BONGO · CALIBRATION v12.1</h1><div class="sub">Controles editables reales · telemetría separada · 2 HC-SR04 / 1 parche</div></div><div><span id="runPill" class="pill">...</span> <span id="editPill" class="pill">...</span></div></header>
<div class="wrap"><div class="cards">
<div class="card"><div class="k">S1</div><div id="d1" class="v a">--</div></div><div class="card"><div class="k">S2</div><div id="d2" class="v b">--</div></div><div class="card"><div class="k">Z</div><div id="z" class="v">--</div></div><div class="card"><div class="k">VELOCIDAD</div><div id="vel" class="v">--</div></div><div class="card"><div class="k">HITS</div><div id="hits" class="v hit">0</div></div><div class="card"><div class="k">Hz COMBINADOS</div><div id="hz" class="v">--</div></div></div>
<div class="grid"><section class="panel"><b>SEÑAL</b><canvas id="chart" width="560" height="260"></canvas><div class="sub">S1 verde · S2 azul · Z blanca · línea HIT amarilla</div></section><section class="panel"><b>CALIDAD</b><div class="row"><span>Movimiento</span><b id="motion">--</b></div><div class="row"><span>Jitter</span><b id="jitter">--</b></div><div class="row"><span>Coherencia</span><b id="coh">--</b></div><div class="meter"><i id="cohBar"></i></div><div class="row"><span>Hz S1 / S2</span><b id="shz">--</b></div><div class="row"><span>Timeouts</span><b id="timeouts">--</b></div><div class="row"><span>Fuera de rango</span><b id="rejects">--</b></div><div class="row"><span>Sesgo lateral</span><b id="lat">--</b></div><div class="row"><span>ARMADO</span><b id="armed">--</b></div><div class="row"><span>Último HIT</span><b id="last">--</b></div><div class="buttons"><button class="danger" onclick="stopAcq()">DETENER</button><button class="primary" onclick="startAcq()">INICIAR</button><button id="soundBtn" class="blue" onclick="toggleSound()">SONIDO OFF</button><button onclick="testSound()">PROBAR BIP</button></div></section></div>
<section class="panel" style="margin-top:8px"><div class="row"><b>CALIBRACIÓN</b><span id="editHint">Detené para editar</span></div><div id="controls" class="controls">
<div class="ctrl"><div class="ctrlHead"><span>Rango máximo</span><b><span id="maxRangeV">50</span> cm</b></div><input id="maxRange" type="range" min="20" max="100" step="5" value="50"></div>
<div class="ctrl"><div class="ctrlHead"><span>Plano HIT</span><b><span id="hitPlaneV">12</span> cm</b></div><input id="hitPlane" type="range" min="5" max="30" step="0.5" value="12"></div>
<div class="ctrl"><div class="ctrlHead"><span>Velocidad HIT</span><b><span id="hitSpeedV">35</span> cm/s</b></div><input id="hitSpeed" type="range" min="10" max="180" step="5" value="35"></div>
<div class="ctrl"><div class="ctrlHead"><span>Rearm</span><b><span id="rearmV">3</span> cm</b></div><input id="rearm" type="range" min="1" max="10" step="0.5" value="3"></div>
<div class="ctrl"><div class="ctrlHead"><span>Mínimo entre HIT</span><b><span id="hitIntervalV">100</span> ms</b></div><input id="hitInterval" type="range" min="70" max="300" step="10" value="100"></div>
<div class="ctrl"><div class="ctrlHead"><span>Ping alternado</span><b><span id="slotV">30</span> ms</b></div><input id="slot" type="range" min="30" max="100" step="5" value="30"></div>
<div class="ctrl"><div class="ctrlHead"><span>FAST alpha</span><b><span id="fastAlphaV">0.72</span></b></div><input id="fastAlpha" type="range" min="0.45" max="0.90" step="0.01" value="0.72"></div>
<div class="ctrl"><div class="ctrlHead"><span>STABLE alpha</span><b><span id="stableAlphaV">0.20</span></b></div><input id="stableAlpha" type="range" min="0.05" max="0.50" step="0.01" value="0.20"></div>
<div class="ctrl"><div class="ctrlHead"><span>Coherencia mínima</span><b><span id="cohMinV">18</span>%</b></div><input id="cohMin" type="range" min="0" max="80" step="5" value="18"></div>
<div class="ctrl"><div class="ctrlHead"><span>Granularidad</span><b><span id="displayStepV">0.5</span> cm</b></div><input id="displayStep" type="range" min="0.1" max="2" step="0.1" value="0.5"></div>
<div class="ctrl"><div class="ctrlHead"><span>Distancia sensores</span><b><span id="baselineV">1.0</span> cm</b></div><input id="baseline" type="range" min="0.5" max="10" step="0.1" value="1.0"></div>
<div class="ctrl"><div class="ctrlHead"><span>Semiángulo visual</span><b><span id="coneV">7.5</span>°</b></div><input id="cone" type="range" min="3" max="20" step="0.5" value="7.5"></div>
</div><div class="buttons"><button id="applyBtn" class="primary" onclick="applyCfg()">APLICAR</button><button id="loadBtn" onclick="loadCurrent()">CARGAR ACTUALES</button><button onclick="resetHits()">RESET HITS</button></div><div id="cfgStatus" class="status"></div></section></div>
<script>
const $=id=>document.getElementById(id), C=$('chart').getContext('2d');
const ids=['maxRange','hitPlane','hitSpeed','rearm','hitInterval','slot','fastAlpha','stableAlpha','cohMin','displayStep','baseline','cone'];
const labs={maxRange:'maxRangeV',hitPlane:'hitPlaneV',hitSpeed:'hitSpeedV',rearm:'rearmV',hitInterval:'hitIntervalV',slot:'slotV',fastAlpha:'fastAlphaV',stableAlpha:'stableAlphaV',cohMin:'cohMinV',displayStep:'displayStepV',baseline:'baselineV',cone:'coneV'};
let dirty=false,firstSync=true,forceSync=false,running=true,lastSeq=null,soundOn=false,audioCtx=null,lastCfg=null;const H=[];
ids.forEach(id=>$(id).addEventListener('input',()=>{ $(labs[id]).textContent=$(id).value; dirty=true; $('editPill').textContent='CAMBIOS PENDIENTES';$('editPill').className='pill pending';$('cfgStatus').textContent='Cambios locales sin aplicar'; }));
function setEditable(on){ids.forEach(id=>$(id).disabled=!on);$('applyBtn').disabled=!on;$('loadBtn').disabled=!on;document.querySelectorAll('.ctrl').forEach(e=>e.classList.toggle('disabled',!on));if(!dirty)$('editHint').textContent=on?'Editá sliders y pulsá APLICAR':'Detené para editar';}
function syncFromServer(j){const c=j.cfg,v={maxRange:c.maxRange,hitPlane:c.hitPlane,hitSpeed:c.hitSpeed,rearm:c.rearm,hitInterval:c.minHitInterval,slot:c.slotMs,fastAlpha:c.fastAlpha,stableAlpha:c.stableAlpha,cohMin:c.minCoherence,displayStep:c.displayStep,baseline:c.baseline,cone:c.coneHalfAngle};Object.entries(v).forEach(([id,x])=>{$(id).value=x;$(labs[id]).textContent=x});dirty=false;firstSync=false;forceSync=false;$('editPill').textContent='CONFIG SIN CAMBIOS';$('editPill').className='pill';$('cfgStatus').textContent='Valores sincronizados con el ESP';}
async function loadCurrent(){forceSync=true;dirty=false;await refresh();}
function queryCfg(){return new URLSearchParams({range:$('maxRange').value,plane:$('hitPlane').value,speed:$('hitSpeed').value,rearm:$('rearm').value,interval:$('hitInterval').value,slot:$('slot').value,fa:$('fastAlpha').value,sa:$('stableAlpha').value,coh:$('cohMin').value,step:$('displayStep').value,base:$('baseline').value,cone:$('cone').value});}
async function applyCfg(){if(running){$('cfgStatus').textContent='Primero DETENER la adquisición';return false;}const r=await fetch('/api/config?'+queryCfg().toString(),{cache:'no-store'});if(!r.ok){$('cfgStatus').textContent='ERROR aplicando configuración: '+await r.text();return false;}dirty=false;forceSync=true;$('cfgStatus').textContent='Configuración aplicada';await refresh();return true;}
async function stopAcq(){await fetch('/api/run?on=0',{cache:'no-store'});await refresh();$('cfgStatus').textContent='Detenido: ahora podés editar';}
async function startAcq(){if(dirty){const ok=await applyCfg();if(!ok)return;}await fetch('/api/run?on=1',{cache:'no-store'});await refresh();$('cfgStatus').textContent='Detección iniciada';}
async function resetHits(){await fetch('/api/reset',{cache:'no-store'});lastSeq=null;await refresh();}
async function ensureAudio(){if(!audioCtx)audioCtx=new(window.AudioContext||window.webkitAudioContext)();await audioCtx.resume();}
async function toggleSound(){await ensureAudio();soundOn=!soundOn;$('soundBtn').textContent=soundOn?'SONIDO ON':'SONIDO OFF';if(soundOn)playHit(60);}
async function testSound(){await ensureAudio();soundOn=true;$('soundBtn').textContent='SONIDO ON';playHit(90);}
function playHit(midi){if(!soundOn||!audioCtx)return;const n=audioCtx.currentTime,o=audioCtx.createOscillator(),g=audioCtx.createGain();o.type='triangle';o.frequency.value=520+Math.min(127,midi)*3;g.gain.setValueAtTime(.0001,n);g.gain.exponentialRampToValueAtTime(.18,n+.004);g.gain.exponentialRampToValueAtTime(.0001,n+.07);o.connect(g);g.connect(audioCtx.destination);o.start(n);o.stop(n+.08);}
function line(x1,y1,x2,y2,col,w=1){C.beginPath();C.moveTo(x1,y1);C.lineTo(x2,y2);C.strokeStyle=col;C.lineWidth=w;C.stroke();}
function draw(j){const W=560,H=260,p=30,max=j.cfg.maxRange,y=v=>H-p-v/max*(H-2*p);C.clearRect(0,0,W,H);for(let q=10;q<=max;q+=10)line(p,y(q),W-8,y(q),'#14242e');line(p,y(j.cfg.hitPlane),W-8,y(j.cfg.hitPlane),'#ffc65a',2);function ser(k,col,w){C.beginPath();let st=false;HIST:for(let i=0;i<H.length;i++){const v=H[i][k];if(!Number.isFinite(v)||v>max){st=false;continue HIST;}const x=p+i/Math.max(1,H.length-1)*(W-p-10);if(!st){C.moveTo(x,y(v));st=true}else C.lineTo(x,y(v));}C.strokeStyle=col;C.lineWidth=w;C.stroke();}ser('s1','#5be8cd',1.5);ser('s2','#739fff',1.5);ser('z','#fff',2.5);H.forEach((r,i)=>{if(r.hit){const x=p+i/Math.max(1,H.length-1)*(W-p-10);line(x,8,x,H-p,'#ffc65a',2)}});}
async function refresh(){try{const r=await fetch('/api/status',{cache:'no-store'}),j=await r.json();running=j.running;lastCfg=j.cfg;setEditable(!running);$('runPill').textContent=running?'DETECCIÓN ACTIVA':'DETECCIÓN DETENIDA';$('runPill').className='pill '+(running?'on':'off');if(firstSync||forceSync)syncFromServer(j);$('d1').textContent=j.s1.ok?j.s1.display.toFixed(1)+' cm':'--';$('d2').textContent=j.s2.ok?j.s2.display.toFixed(1)+' cm':'--';$('z').textContent=j.pair.valid?j.pair.zDisplay.toFixed(1)+' cm':'--';$('vel').textContent=j.pair.valid?j.pair.velocity.toFixed(0)+' cm/s':'--';$('hits').textContent=j.pair.hits;$('hz').textContent=j.pair.combinedHz.toFixed(1);$('motion').textContent=j.pair.motion;$('jitter').textContent=j.pair.jitter.toFixed(2)+' cm';$('coh').textContent=j.pair.coherence.toFixed(0)+'%';$('cohBar').style.width=j.pair.coherence.toFixed(0)+'%';$('shz').textContent=j.s1.hz.toFixed(1)+' / '+j.s2.hz.toFixed(1);$('timeouts').textContent=j.s1.timeouts+' / '+j.s2.timeouts;$('rejects').textContent=j.s1.rejected+' / '+j.s2.rejected;$('lat').textContent=j.pair.lateral+' '+j.pair.lateralConfidence.toFixed(0)+'%';$('armed').textContent=j.pair.armed?'SÍ':'NO';$('last').textContent=j.pair.lastHitMs?j.pair.lastHitZ.toFixed(1)+'cm · '+j.pair.lastHitVelocity.toFixed(0)+'cm/s · MIDI '+j.pair.lastMidi:'--';let hit=false;if(lastSeq===null)lastSeq=j.event.seq;else if(j.event.seq!==lastSeq){hit=true;lastSeq=j.event.seq;playHit(j.event.midi||80);}H.push({s1:j.s1.ok?j.s1.display:NaN,s2:j.s2.ok?j.s2.display:NaN,z:j.pair.valid?j.pair.zDisplay:NaN,hit});if(H.length>180)H.shift();draw(j);}catch(e){$('cfgStatus').textContent='Error de comunicación con ESP';}}
setInterval(refresh,100);refresh();
</script></body></html>
)HTML";

void sendStatus(){
  String j="{";
  j+="\"fw\":\""+String(FW_VERSION)+"\",\"running\":"+String(acquisitionRunning?"true":"false")+",";
  for(int i=0;i<2;i++){
    SensorState&q=s[i];j+="\"s"+String(i+1)+"\":{";
    j+="\"ok\":"+String(q.ok?"true":"false")+",\"raw\":"+String(q.rawCm,2)+",\"median\":"+String(q.medianCm,2)+",\"fast\":"+String(q.fastCm,2)+",\"stable\":"+String(q.stableCm,2)+",\"display\":"+String(q.displayCm,2)+",\"velocity\":"+String(q.velocityCmS,2)+",\"jitter\":"+String(q.jitterCm,2)+",\"timeouts\":"+String(q.timeouts)+",\"rejected\":"+String(q.rejected)+",\"hz\":"+String(q.hz,2)+"}";if(i==0)j+=",";
  }
  j+=",\"pair\":{";
  j+="\"valid\":"+String(pair.valid?"true":"false")+",\"geoValid\":"+String(pair.geoValid?"true":"false")+",\"zFast\":"+String(pair.zFast,2)+",\"zStable\":"+String(pair.zStable,2)+",\"zDisplay\":"+String(pair.zDisplay,2)+",\"x\":"+String(pair.x,2)+",\"velocity\":"+String(pair.velocity,2)+",\"jitter\":"+String(pair.jitter,2)+",\"coherence\":"+String(pair.coherence,1)+",\"lateral\":\""+pair.lateral+"\",\"lateralConfidence\":"+String(pair.lateralConfidence,1)+",\"armed\":"+String(pair.armed?"true":"false")+",\"hits\":"+String(pair.hits)+",\"lastHitMs\":"+String(pair.lastHitMs)+",\"lastHitVelocity\":"+String(pair.lastHitVelocity,1)+",\"lastHitZ\":"+String(pair.lastHitZ,2)+",\"lastMidi\":"+String(pair.lastMidi)+",\"combinedHz\":"+String(pair.combinedHz,2)+",\"motion\":\""+motionLabel()+"\"},";
  j+="\"event\":{\"seq\":"+String(pair.eventSeq)+",\"pad\":1,\"midi\":"+String(pair.lastMidi)+",\"velocity\":"+String(pair.lastHitVelocity,1)+",\"z\":"+String(pair.lastHitZ,2)+"},";
  j+="\"cfg\":{\"maxRange\":"+String(maxRangeCm,1)+",\"hitPlane\":"+String(hitPlaneCm,1)+",\"hitSpeed\":"+String(hitSpeedCmS,1)+",\"rearm\":"+String(rearmMarginCm,1)+",\"minHitInterval\":"+String(minHitIntervalMs)+",\"slotMs\":"+String(slotMs)+",\"fastAlpha\":"+String(fastAlpha,2)+",\"stableAlpha\":"+String(stableAlpha,2)+",\"minCoherence\":"+String(minCoherencePct,0)+",\"displayStep\":"+String(displayStepCm,1)+",\"baseline\":"+String(baselineCm,1)+",\"coneHalfAngle\":"+String(coneHalfAngleDeg,1)+",\"echoTimeoutUs\":"+String(echoTimeoutUs())+"}}";
  server.send(200,"application/json",j);
}

void handleConfig(){
  if(acquisitionRunning){server.send(409,"text/plain","STOP acquisition before config");return;}
  if(server.hasArg("range"))maxRangeCm=clampf(server.arg("range").toFloat(),20,100);
  if(server.hasArg("plane"))hitPlaneCm=clampf(server.arg("plane").toFloat(),5,30);
  if(server.hasArg("speed"))hitSpeedCmS=clampf(server.arg("speed").toFloat(),10,180);
  if(server.hasArg("rearm"))rearmMarginCm=clampf(server.arg("rearm").toFloat(),1,10);
  if(server.hasArg("interval"))minHitIntervalMs=clampu(server.arg("interval").toInt(),70,300);
  if(server.hasArg("slot"))slotMs=clampu(server.arg("slot").toInt(),30,100);
  if(server.hasArg("fa"))fastAlpha=clampf(server.arg("fa").toFloat(),0.45f,0.90f);
  if(server.hasArg("sa"))stableAlpha=clampf(server.arg("sa").toFloat(),0.05f,0.50f);
  if(server.hasArg("coh"))minCoherencePct=clampf(server.arg("coh").toFloat(),0,80);
  if(server.hasArg("step"))displayStepCm=clampf(server.arg("step").toFloat(),0.1f,2.0f);
  if(server.hasArg("base"))baselineCm=clampf(server.arg("base").toFloat(),0.5f,10.0f);
  if(server.hasArg("cone"))coneHalfAngleDeg=clampf(server.arg("cone").toFloat(),3,20);
  resetTracking(true);addLog("CFG range="+String(maxRangeCm,0)+" hit="+String(hitPlaneCm,1)+" base="+String(baselineCm,1)+" slot="+String(slotMs));server.send(200,"text/plain","OK");
}
void handleRun(){
  if(server.hasArg("on"))acquisitionRunning=server.arg("on")!="0";
  if(acquisitionRunning){resetTracking(true);nextSensor=0;nextMeasureMs=millis()+80;addLog("ACQ START");}else addLog("ACQ STOP");
  server.send(200,"text/plain","OK");
}
void setup(){
  Serial.begin(115200);delay(120);for(int i=0;i<2;i++){pinMode(PINS[i].echo,INPUT);pinMode(PINS[i].trig,OUTPUT);digitalWrite(PINS[i].trig,LOW);}WiFi.mode(WIFI_AP);WiFi.softAP(AP_SSID);
  server.on("/",[](){server.send_P(200,"text/html",PAGE);});server.on("/api/status",sendStatus);server.on("/api/config",handleConfig);server.on("/api/run",handleRun);server.on("/api/reset",[](){resetTracking(false);server.send(200,"text/plain","OK");});server.begin();nextMeasureMs=millis()+300;addLog("BOOT "+String(FW_VERSION));
}
void loop(){server.handleClient();serviceSensors();delay(1);}
