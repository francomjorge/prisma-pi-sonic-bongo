/*
  SONIC BONGO / PRISMA
  ESP32-S3 N16R8 - Dual Sensor Bongo v11

  CONFIRMED WIRING:
    S1: GPIO13 -> TRIG direct
        HC-SR04 ECHO -> TXS B1 -> A1 -> GPIO14
    S2: GPIO15 -> TRIG direct
        HC-SR04 ECHO -> TXS B2 -> A2 -> GPIO16

  Goals v11:
    - Stabilize display without slowing the hit engine.
    - Alternate sensors every 30 ms (~60 ms per sensor nominal).
    - Keep RAW / FAST / STABLE channels separate.
    - Quantize only UI/stable distance (default 0.5 cm).
    - Hit detection uses fast fused distance + fused velocity.
    - Faster re-arm for repeated hits.
    - Lateral result is diagnostic only with 1 cm baseline.
    - Browser audio click on HIT; event object ready for future samples.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

WebServer server(80);

static const char* FW_VERSION = "11.0-stable-bongo";
static const char* AP_SSID = "PRISMA-S3-DIAG";

struct SensorPins { uint8_t trig; uint8_t echo; };
const SensorPins PINS[2] = { {13,14}, {15,16} };

static const uint32_t ECHO_TIMEOUT_US = 26000;
static const uint32_t SLOT_MS = 30;
static const uint32_t STALE_MS = 180;
static const uint32_t MIN_HIT_INTERVAL_MS = 110;

static const float FAST_ALPHA = 0.68f;
static const float STABLE_ALPHA = 0.24f;
static const float SENSOR_V_ALPHA = 0.34f;
static const float PAIR_V_ALPHA = 0.42f;
static const float MAX_STEP_CM = 25.0f;

float baselineCm = 1.0f;
float hitPlaneCm = 12.0f;
float hitSpeedCmS = 38.0f;
float rearmMarginCm = 3.0f;
float coneHalfAngleDeg = 7.5f;
float displayStepCm = 0.5f;

struct SensorState {
  bool have = false;
  bool ok = false;
  uint32_t lastMs = 0;
  uint32_t pulseUs = 0;
  int idle = 0;
  float rawCm = 0;
  float fastCm = 0;
  float stableCm = 0;
  float displayCm = 0;
  float prevFastCm = 0;
  uint32_t prevMs = 0;
  float velocityCmS = 0;
  float jitterCm = 0;
  uint32_t valid = 0;
  uint32_t timeouts = 0;
  uint32_t validAtRateMark = 0;
  float hz = 0;
};
SensorState s[2];

struct PairState {
  bool valid = false;
  bool geoValid = false;
  bool armed = true;
  float zFastCm = 0;
  float zStableCm = 0;
  float zDisplayCm = 0;
  float prevZFastCm = 0;
  uint32_t prevZMs = 0;
  float velocityCmS = 0;
  float deltaFastCm = 0;
  float deltaStableCm = 0;
  float xCm = 0;
  float coherencePct = 0;
  float jitterCm = 0;
  String lateral = "INDETERMINADO";
  float lateralConfidence = 0;
  float peakApproachCmS = 0;
  uint32_t hits = 0;
  uint32_t lastHitMs = 0;
  float lastHitVelocity = 0;
  float lastHitZ = 0;
  int lastHitMidi = 0;
  String lastZone = "--";
  float combinedHz = 0;
  uint32_t pairUpdates = 0;
  uint32_t pairUpdatesAtRateMark = 0;
};
PairState pair;

uint8_t nextSensor = 0;
uint32_t nextMeasureMs = 0;
uint32_t rateMarkMs = 0;
String logBuffer;

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
float quantize(float v, float step) { if (step <= 0.001f) return v; return roundf(v / step) * step; }

void addLog(const String& text) {
  String line = "[" + String(millis()) + " ms] " + text + "\n";
  logBuffer += line;
  if (logBuffer.length() > 15000) logBuffer.remove(0, 2500);
  Serial.print(line);
}

int midiFromVelocity(float approachSpeed) {
  float x = clampf((approachSpeed - 25.0f) / (220.0f - 25.0f), 0.0f, 1.0f);
  return 1 + (int)lroundf(x * 126.0f);
}

void updateLateral() {
  float a = fabsf(pair.deltaStableCm);
  if (a < 0.18f) {
    pair.lateral = "CENTRO";
    pair.lateralConfidence = clampf((0.18f - a) / 0.18f * 100.0f, 0.0f, 100.0f);
  } else if (a >= 0.35f) {
    pair.lateral = pair.deltaStableCm < 0 ? "S1 / IZQUIERDA" : "S2 / DERECHA";
    pair.lateralConfidence = clampf((a - 0.35f) / 0.65f * 100.0f + 25.0f, 0.0f, 100.0f);
  } else {
    pair.lateral = "INDETERMINADO";
    pair.lateralConfidence = 0;
  }
}

void updateRates() {
  uint32_t now = millis();
  if (rateMarkMs == 0) {
    rateMarkMs = now;
    for (uint8_t i=0;i<2;i++) s[i].validAtRateMark = s[i].valid;
    pair.pairUpdatesAtRateMark = pair.pairUpdates;
    return;
  }
  uint32_t dt = now - rateMarkMs;
  if (dt < 1000) return;
  float sec = dt / 1000.0f;
  for (uint8_t i=0;i<2;i++) {
    s[i].hz = (s[i].valid - s[i].validAtRateMark) / sec;
    s[i].validAtRateMark = s[i].valid;
  }
  pair.combinedHz = (pair.pairUpdates - pair.pairUpdatesAtRateMark) / sec;
  pair.pairUpdatesAtRateMark = pair.pairUpdates;
  rateMarkMs = now;
}

void updatePair() {
  uint32_t now = millis();
  bool fresh0 = s[0].ok && (now - s[0].lastMs <= STALE_MS);
  bool fresh1 = s[1].ok && (now - s[1].lastMs <= STALE_MS);
  pair.valid = fresh0 && fresh1;
  if (!pair.valid) return;

  float a0 = clampf((now - s[0].lastMs) / 1000.0f, 0.0f, 0.06f);
  float a1 = clampf((now - s[1].lastMs) / 1000.0f, 0.0f, 0.06f);
  float p0 = s[0].fastCm + clampf(s[0].velocityCmS * a0, -4.0f, 4.0f);
  float p1 = s[1].fastCm + clampf(s[1].velocityCmS * a1, -4.0f, 4.0f);

  float zFastNew = 0.5f * (p0 + p1);
  pair.zStableCm = 0.5f * (s[0].stableCm + s[1].stableCm);
  pair.zDisplayCm = quantize(pair.zStableCm, displayStepCm);
  pair.deltaFastCm = p0 - p1;
  pair.deltaStableCm = s[0].stableCm - s[1].stableCm;
  pair.jitterCm = 0.5f * (s[0].jitterCm + s[1].jitterCm);

  if (pair.prevZMs != 0 && now > pair.prevZMs) {
    float dt = (now - pair.prevZMs) / 1000.0f;
    if (dt > 0.015f && dt < 0.20f) {
      float rawV = (zFastNew - pair.prevZFastCm) / dt;
      pair.velocityCmS = PAIR_V_ALPHA * rawV + (1.0f - PAIR_V_ALPHA) * pair.velocityCmS;
    }
  }
  pair.zFastCm = zFastNew;
  pair.prevZFastCm = zFastNew;
  pair.prevZMs = now;
  pair.pairUpdates++;

  pair.coherencePct = clampf(100.0f - fabsf(pair.deltaStableCm) * 22.0f - pair.jitterCm * 8.0f, 0.0f, 100.0f);

  pair.geoValid = false;
  pair.xCm = 0;
  if (baselineCm > 0.01f && fabsf(pair.deltaStableCm) <= baselineCm) {
    float d1 = s[0].stableCm;
    float d2 = s[1].stableCm;
    float x = (d1*d1 - d2*d2) / (2.0f * baselineCm);
    float z2 = d1*d1 - (x + baselineCm*0.5f) * (x + baselineCm*0.5f);
    if (z2 >= 0.0f) { pair.xCm = x; pair.geoValid = true; }
  }

  updateLateral();

  if (pair.velocityCmS < 0 && -pair.velocityCmS > pair.peakApproachCmS) pair.peakApproachCmS = -pair.velocityCmS;

  bool enteredPlane = pair.zFastCm <= hitPlaneCm;
  bool fastEnough = pair.velocityCmS <= -hitSpeedCmS;
  bool coherentEnough = pair.coherencePct >= 18.0f;
  bool intervalOK = (now - pair.lastHitMs) >= MIN_HIT_INTERVAL_MS;

  if (pair.armed && enteredPlane && fastEnough && coherentEnough && intervalOK) {
    pair.hits++;
    pair.lastHitMs = now;
    pair.lastHitVelocity = -pair.velocityCmS;
    pair.lastHitZ = pair.zFastCm;
    pair.lastHitMidi = midiFromVelocity(pair.lastHitVelocity);
    pair.lastZone = pair.lateral;
    pair.armed = false;
    addLog("HIT #" + String(pair.hits) + " z=" + String(pair.lastHitZ,1) + " v=" + String(pair.lastHitVelocity,1) + " midi=" + String(pair.lastHitMidi) + " zone=" + pair.lastZone + " coh=" + String(pair.coherencePct,0) + "%");
  }

  if (!pair.armed && (now - pair.lastHitMs) >= MIN_HIT_INTERVAL_MS && pair.zFastCm >= hitPlaneCm + rearmMarginCm) {
    pair.armed = true;
    pair.peakApproachCmS = 0;
  }
}

void processValid(uint8_t i, uint32_t pulse) {
  SensorState &q = s[i];
  uint32_t now = millis();
  float raw = ((float)pulse * 0.0343f) / 2.0f;
  q.ok = true;
  q.pulseUs = pulse;
  q.rawCm = raw;
  q.lastMs = now;
  q.valid++;

  if (!q.have) {
    q.have = true;
    q.fastCm = raw;
    q.stableCm = raw;
    q.displayCm = quantize(raw, displayStepCm);
    q.prevFastCm = raw;
    q.prevMs = now;
    q.velocityCmS = 0;
    q.jitterCm = 0;
  } else {
    float guarded = raw;
    float diff = guarded - q.fastCm;
    if (diff > MAX_STEP_CM) guarded = q.fastCm + MAX_STEP_CM;
    if (diff < -MAX_STEP_CM) guarded = q.fastCm - MAX_STEP_CM;

    float fastNew = FAST_ALPHA * guarded + (1.0f - FAST_ALPHA) * q.fastCm;
    q.stableCm = STABLE_ALPHA * guarded + (1.0f - STABLE_ALPHA) * q.stableCm;
    q.displayCm = quantize(q.stableCm, displayStepCm);
    q.jitterCm = 0.88f * q.jitterCm + 0.12f * fabsf(raw - q.stableCm);

    if (now > q.prevMs) {
      float dt = (now - q.prevMs) / 1000.0f;
      if (dt > 0.035f && dt < 0.25f) {
        float rawV = (fastNew - q.prevFastCm) / dt;
        q.velocityCmS = SENSOR_V_ALPHA * rawV + (1.0f - SENSOR_V_ALPHA) * q.velocityCmS;
      }
    }
    q.fastCm = fastNew;
    q.prevFastCm = fastNew;
    q.prevMs = now;
  }
}

void processTimeout(uint8_t i) {
  s[i].ok = false;
  s[i].pulseUs = 0;
  s[i].lastMs = millis();
  s[i].timeouts++;
}

void measureOne(uint8_t i) {
  const SensorPins &p = PINS[i];
  s[i].idle = digitalRead(p.echo);
  digitalWrite(p.trig, LOW); delayMicroseconds(4);
  digitalWrite(p.trig, HIGH); delayMicroseconds(10);
  digitalWrite(p.trig, LOW);
  uint32_t pulse = pulseIn(p.echo, HIGH, ECHO_TIMEOUT_US);
  if (pulse) processValid(i, pulse); else processTimeout(i);
  updatePair();
  updateRates();
}

void serviceSensors() {
  uint32_t now = millis();
  if ((int32_t)(now - nextMeasureMs) < 0) return;
  uint8_t i = nextSensor;
  nextSensor ^= 1;
  measureOne(i);
  nextMeasureMs = millis() + SLOT_MS;
}

String motionLabel() {
  if (!pair.valid) return "SIN PAR";
  if (pair.velocityCmS <= -18.0f) return "ACERCANDO";
  if (pair.velocityCmS >= 18.0f) return "ALEJANDO";
  return "ESTABLE";
}

const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html lang="es"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>SONIC BONGO v11</title>
<style>
:root{--bg:#071019;--p:#111923;--l:#2c3d4c;--t:#edf5fb;--m:#8ea1b3;--a:#68f0d1;--b:#79a7ff;--hit:#ffcc66;--ok:#50e98a;--bad:#ff6969}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--t);font-family:system-ui,-apple-system,Segoe UI,sans-serif}header,.w{max-width:1220px;margin:auto;padding:13px}h1{font-size:21px;margin:0}.sub,.note{color:var(--m);font-size:11px}.cards{display:grid;grid-template-columns:repeat(6,minmax(105px,1fr));gap:8px;margin-bottom:9px}.c,.panel{background:var(--p);border:1px solid var(--l);border-radius:11px;padding:11px}.k{font-size:9px;letter-spacing:.07em;color:var(--m)}.v{font-size:20px;font-weight:900;margin-top:4px}.a{color:var(--a)}.b{color:var(--b)}.hit{color:var(--hit)}.grid{display:grid;grid-template-columns:1.18fr .82fr;gap:9px}.row{display:flex;justify-content:space-between;gap:10px;margin:6px 0;font-size:12px}.row b{font-size:13px}canvas{display:block;width:100%;height:auto;background:#060c12;border:1px solid #263441;border-radius:9px}.controls{display:grid;grid-template-columns:1fr 1fr;gap:7px;margin-top:8px}label{font-size:10px;color:var(--m)}input{width:100%;margin-top:3px;background:#091119;color:#fff;border:1px solid #3b4c5d;border-radius:7px;padding:7px}button{padding:9px 11px;border-radius:8px;border:1px solid #40566c;background:#182635;color:#fff;font-weight:800}.on{border-color:#3b8f69;background:#12382d}.badge{padding:4px 7px;border:1px solid #40566c;border-radius:7px;font-size:10px;font-weight:900}.ok{color:var(--ok)}.bad{color:var(--bad)}.legend{display:flex;gap:12px;flex-wrap:wrap;margin-top:6px;font-size:10px;color:var(--m)}.dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:4px}@media(max-width:900px){.grid{grid-template-columns:1fr}.cards{grid-template-columns:repeat(2,1fr)}}
</style></head><body>
<header><h1>SONIC BONGO · STABLE DUAL PATCH v11</h1><div class="sub">2 HC-SR04 = 1 bongó · RAW/FAST/STABLE separados · display 0,5 cm · HIT rápido · audio web opcional</div></header><div class="w">
<div class="cards"><div class="c"><div class="k">S1 ESTABLE</div><div class="v a" id="d1">--</div></div><div class="c"><div class="k">S2 ESTABLE</div><div class="v b" id="d2">--</div></div><div class="c"><div class="k">Z BONGÓ</div><div class="v" id="z">--</div></div><div class="c"><div class="k">VELOCIDAD</div><div class="v" id="velTop">--</div></div><div class="c"><div class="k">HITS</div><div class="v hit" id="hits">0</div></div><div class="c"><div class="k">MUESTRAS</div><div class="v" id="hz">--</div></div></div>
<div class="grid"><section class="panel"><div class="row"><b>PARCHE VIRTUAL</b><span id="coh" class="badge">--</span></div><canvas id="geo" width="760" height="500"></canvas><div class="legend"><span><i class="dot" style="background:#68f0d1"></i>S1 estable</span><span><i class="dot" style="background:#79a7ff"></i>S2 estable</span><span><i class="dot" style="background:#fff"></i>Z estable</span><span><i class="dot" style="background:#ffcc66"></i>plano HIT</span></div><div class="note">La triangulación X es diagnóstica. Con baseline 1 cm, izquierda/derecha es un sesgo estimado, no posición 3D confiable.</div></section>
<section class="panel"><div class="row"><b>MOTOR / ESTABILIDAD</b><button id="soundBtn" onclick="toggleSound()">SONIDO OFF</button></div><div class="row"><span>RAW S1 / S2</span><b id="raw">--</b></div><div class="row"><span>FAST Z interna</span><b id="fastz">--</b></div><div class="row"><span>STABLE Z</span><b id="stablez">--</b></div><div class="row"><span>Jitter estimado</span><b id="jitter">--</b></div><div class="row"><span>Coherencia</span><b id="cohTxt">--</b></div><div class="row"><span>Sesgo lateral</span><b id="lateral">--</b></div><div class="row"><span>Conf. lateral</span><b id="latConf">--</b></div><div class="row"><span>ARMADO</span><b id="armed">--</b></div><div class="row"><span>Último HIT</span><b id="last">--</b></div><div class="row"><span>MIDI velocity</span><b id="midi">--</b></div><div class="row"><span>Hz S1 / S2</span><b id="sensorHz">--</b></div>
<div class="controls"><label>Baseline cm<input id="base" type="number" min="0.2" max="20" step="0.1" value="1"></label><label>Plano HIT cm<input id="plane" type="number" min="4" max="40" step="0.5" value="12"></label><label>Vel. mínima cm/s<input id="speed" type="number" min="5" max="300" step="1" value="38"></label><label>Rearm cm<input id="rearm" type="number" min="1" max="12" step="0.5" value="3"></label><label>Granularidad display cm<input id="step" type="number" min="0.1" max="2" step="0.1" value="0.5"></label><label>Semiángulo cono °<input id="angle" type="number" min="2" max="30" step="0.5" value="7.5"></label></div><div style="margin-top:9px"><button onclick="saveCfg()">APLICAR</button> <button onclick="resetHits()">RESET HITS</button></div><div class="note" style="margin-top:8px">El display se cuantiza; la detección de HIT no. El bip se genera en el navegador y luego esta misma función podrá disparar samples.</div></section></div>
<section class="panel" style="margin-top:9px"><div class="row"><b>TIMELINE ESTABILIZADO</b><span class="note">RAW tenue · FAST · STABLE · HIT</span></div><canvas id="tl" width="1160" height="290"></canvas></section></div>
<script>
const G=document.getElementById('geo').getContext('2d'),T=document.getElementById('tl').getContext('2d');const H=[];let lastSeq=0,soundOn=false,audioCtx=null;
function ln(c,x1,y1,x2,y2,col='#40505d',w=1){c.beginPath();c.moveTo(x1,y1);c.lineTo(x2,y2);c.strokeStyle=col;c.lineWidth=w;c.stroke()}function tx(c,s,x,y,col='#91a4b5',sz=10,a='left'){c.fillStyle=col;c.font=sz+'px system-ui';c.textAlign=a;c.fillText(s,x,y)}
function beep(vel=80){if(!soundOn||!audioCtx)return;let o=audioCtx.createOscillator(),g=audioCtx.createGain(),now=audioCtx.currentTime;o.type='triangle';o.frequency.setValueAtTime(520+vel*3.0,now);g.gain.setValueAtTime(0.0001,now);g.gain.exponentialRampToValueAtTime(0.12,now+0.004);g.gain.exponentialRampToValueAtTime(0.0001,now+0.055);o.connect(g);g.connect(audioCtx.destination);o.start(now);o.stop(now+0.06)}
async function toggleSound(){if(!audioCtx)audioCtx=new (window.AudioContext||window.webkitAudioContext)();await audioCtx.resume();soundOn=!soundOn;soundBtn.textContent=soundOn?'SONIDO ON':'SONIDO OFF';soundBtn.className=soundOn?'on':'';if(soundOn)beep(55)}
function onHitEvent(ev){beep(ev.velocity||80)}
function drawGeo(j){let W=760,Hh=500,by=452,cx=W/2,max=32,sc=(Hh-72)/max;G.clearRect(0,0,W,Hh);for(let q=5;q<=30;q+=5){let y=by-q*sc;ln(G,45,y,W-25,y,'#17232d');tx(G,q+' cm',8,y+3)}let x1=cx-j.cfg.baseline/2*sc,x2=cx+j.cfg.baseline/2*sc,hy=by-j.cfg.hitPlane*sc;ln(G,45,hy,W-25,hy,'#ffcc66',2);tx(G,'HIT '+j.cfg.hitPlane.toFixed(1)+' cm',W-28,hy-6,'#ffcc66',11,'right');let len=max*sc,spread=Math.tan(j.cfg.coneHalfAngle*Math.PI/180)*len;[[x1,'#68f0d1'],[x2,'#79a7ff']].forEach(v=>{G.save();G.globalAlpha=.11;G.beginPath();G.moveTo(v[0],by);G.lineTo(v[0]-spread,by-len);G.lineTo(v[0]+spread,by-len);G.closePath();G.fillStyle=v[1];G.fill();G.restore();ln(G,v[0],by,v[0]-spread,by-len,v[1]);ln(G,v[0],by,v[0]+spread,by-len,v[1])});G.fillStyle='#68f0d1';G.fillRect(x1-7,by-6,14,12);G.fillStyle='#79a7ff';G.fillRect(x2-7,by-6,14,12);tx(G,'S1',x1,by+23,'#68f0d1',11,'center');tx(G,'S2',x2,by+23,'#79a7ff',11,'center');tx(G,j.cfg.baseline.toFixed(1)+' cm',cx,by+23,'#dfe9f0',10,'center');if(j.pair.valid){let py=by-j.pair.zDisplay*sc;G.beginPath();G.arc(cx,py,8,0,Math.PI*2);G.fillStyle='#fff';G.fill();tx(G,j.pair.zDisplay.toFixed(1)+' cm',cx+13,py-8,'#fff',11);if(j.pair.geoValid){let px=cx+j.pair.x*sc;G.beginPath();G.arc(px,py,4,0,Math.PI*2);G.fillStyle='#ffcc66';G.fill()}}}
function drawTL(j){let W=1160,Hh=290,p=36,max=32;T.clearRect(0,0,W,Hh);let y=v=>Hh-p-v/max*(Hh-2*p);for(let q=5;q<=30;q+=5){ln(T,p,y(q),W-10,y(q),'#17232d');tx(T,q,8,y(q)+3)}ln(T,p,y(j.cfg.hitPlane),W-10,y(j.cfg.hitPlane),'#ffcc66',1.5);function ser(k,col,w,alpha){T.save();T.globalAlpha=alpha;T.beginPath();let st=false;H.forEach((r,i)=>{let v=r[k];if(!Number.isFinite(v)){st=false;return}let x=p+i/(Math.max(1,H.length-1))*(W-p-15);if(!st){T.moveTo(x,y(v));st=true}else T.lineTo(x,y(v))});T.strokeStyle=col;T.lineWidth=w;T.stroke();T.restore()}ser('r1','#68f0d1',1,.24);ser('r2','#79a7ff',1,.24);ser('fast','#d1d7dc',1.2,.55);ser('stable','#fff',2.6,1);H.forEach((r,i)=>{if(r.hit){let x=p+i/(Math.max(1,H.length-1))*(W-p-15);ln(T,x,15,x,Hh-p,'#ffcc66',2)}})}
async function refresh(){try{let r=await fetch('/api/status',{cache:'no-store'}),j=await r.json();d1.textContent=j.s1.ok?j.s1.display.toFixed(1)+' cm':'TIMEOUT';d2.textContent=j.s2.ok?j.s2.display.toFixed(1)+' cm':'TIMEOUT';z.textContent=j.pair.valid?j.pair.zDisplay.toFixed(1)+' cm':'--';velTop.textContent=j.pair.valid?j.pair.velocity.toFixed(0)+' cm/s':'--';hits.textContent=j.pair.hits;hz.textContent=j.pair.combinedHz.toFixed(1)+' Hz';raw.textContent=j.s1.raw.toFixed(1)+' / '+j.s2.raw.toFixed(1)+' cm';fastz.textContent=j.pair.valid?j.pair.zFast.toFixed(2)+' cm':'--';stablez.textContent=j.pair.valid?j.pair.zStable.toFixed(2)+' cm':'--';jitter.textContent=j.pair.jitter.toFixed(2)+' cm';cohTxt.textContent=j.pair.coherence.toFixed(0)+'%';coh.textContent='COH '+j.pair.coherence.toFixed(0)+'%';coh.className='badge '+(j.pair.coherence>=55?'ok':'bad');lateral.textContent=j.pair.lateral;latConf.textContent=j.pair.lateralConfidence.toFixed(0)+'%';armed.textContent=j.pair.armed?'SÍ':'NO';last.textContent=j.pair.lastHitMs?('#'+j.pair.hits+' · '+j.pair.lastHitZ.toFixed(1)+' cm · '+j.pair.lastHitVelocity.toFixed(0)+' cm/s'):'--';midi.textContent=j.pair.lastMidi||'--';sensorHz.textContent=j.s1.hz.toFixed(1)+' / '+j.s2.hz.toFixed(1);base.value=j.cfg.baseline;plane.value=j.cfg.hitPlane;speed.value=j.cfg.hitSpeed;rearm.value=j.cfg.rearm;step.value=j.cfg.displayStep;angle.value=j.cfg.coneHalfAngle;let hit=j.event.seq>lastSeq;if(hit&&lastSeq!==0)onHitEvent(j.event);lastSeq=j.event.seq;H.push({r1:j.s1.ok?j.s1.raw:NaN,r2:j.s2.ok?j.s2.raw:NaN,fast:j.pair.valid?j.pair.zFast:NaN,stable:j.pair.valid?j.pair.zDisplay:NaN,hit});if(H.length>210)H.shift();drawGeo(j);drawTL(j)}catch(e){}}
async function saveCfg(){await fetch('/api/config?b='+encodeURIComponent(base.value)+'&p='+encodeURIComponent(plane.value)+'&s='+encodeURIComponent(speed.value)+'&r='+encodeURIComponent(rearm.value)+'&q='+encodeURIComponent(step.value)+'&a='+encodeURIComponent(angle.value));refresh()}async function resetHits(){await fetch('/api/reset');lastSeq=0;refresh()}setInterval(refresh,90);refresh();
</script></body></html>
)HTML";

void sendStatus() {
  String j="{";
  j += "\"fw\":\""+String(FW_VERSION)+"\",";
  for(uint8_t i=0;i<2;i++){
    SensorState &q=s[i];
    j += "\"s"+String(i+1)+"\":{";
    j += "\"ok\":"+String(q.ok?"true":"false")+",";
    j += "\"raw\":"+String(q.rawCm,2)+",";
    j += "\"fast\":"+String(q.fastCm,2)+",";
    j += "\"stable\":"+String(q.stableCm,2)+",";
    j += "\"display\":"+String(q.displayCm,2)+",";
    j += "\"velocity\":"+String(q.velocityCmS,2)+",";
    j += "\"jitter\":"+String(q.jitterCm,2)+",";
    j += "\"hz\":"+String(q.hz,2)+",";
    j += "\"pulse\":"+String(q.pulseUs)+",";
    j += "\"valid\":"+String(q.valid)+",";
    j += "\"timeouts\":"+String(q.timeouts)+"}";
    if(i==0)j+=",";
  }
  j += ",\"pair\":{";
  j += "\"valid\":"+String(pair.valid?"true":"false")+",";
  j += "\"geoValid\":"+String(pair.geoValid?"true":"false")+",";
  j += "\"zFast\":"+String(pair.zFastCm,2)+",";
  j += "\"zStable\":"+String(pair.zStableCm,2)+",";
  j += "\"zDisplay\":"+String(pair.zDisplayCm,2)+",";
  j += "\"x\":"+String(pair.xCm,2)+",";
  j += "\"deltaFast\":"+String(pair.deltaFastCm,2)+",";
  j += "\"deltaStable\":"+String(pair.deltaStableCm,2)+",";
  j += "\"velocity\":"+String(pair.velocityCmS,2)+",";
  j += "\"jitter\":"+String(pair.jitterCm,2)+",";
  j += "\"coherence\":"+String(pair.coherencePct,1)+",";
  j += "\"lateral\":\""+pair.lateral+"\",";
  j += "\"lateralConfidence\":"+String(pair.lateralConfidence,1)+",";
  j += "\"armed\":"+String(pair.armed?"true":"false")+",";
  j += "\"hits\":"+String(pair.hits)+",";
  j += "\"lastHitMs\":"+String(pair.lastHitMs)+",";
  j += "\"lastHitVelocity\":"+String(pair.lastHitVelocity,1)+",";
  j += "\"lastHitZ\":"+String(pair.lastHitZ,2)+",";
  j += "\"lastMidi\":"+String(pair.lastHitMidi)+",";
  j += "\"lastZone\":\""+pair.lastZone+"\",";
  j += "\"combinedHz\":"+String(pair.combinedHz,2)+",";
  j += "\"motion\":\""+motionLabel()+"\"},";
  j += "\"event\":{";
  j += "\"seq\":"+String(pair.hits)+",";
  j += "\"pad\":1,";
  j += "\"velocity\":"+String(pair.lastHitMidi)+",";
  j += "\"approachSpeed\":"+String(pair.lastHitVelocity,1)+",";
  j += "\"z\":"+String(pair.lastHitZ,2)+",";
  j += "\"zone\":\""+pair.lastZone+"\",";
  j += "\"timestampMs\":"+String(pair.lastHitMs)+"},";
  j += "\"cfg\":{";
  j += "\"baseline\":"+String(baselineCm,2)+",";
  j += "\"hitPlane\":"+String(hitPlaneCm,2)+",";
  j += "\"hitSpeed\":"+String(hitSpeedCmS,1)+",";
  j += "\"rearm\":"+String(rearmMarginCm,1)+",";
  j += "\"displayStep\":"+String(displayStepCm,1)+",";
  j += "\"coneHalfAngle\":"+String(coneHalfAngleDeg,1)+"}";
  j += "}";
  server.send(200,"application/json",j);
}

void handleConfig() {
  if(server.hasArg("b")) baselineCm=clampf(server.arg("b").toFloat(),0.2f,20.0f);
  if(server.hasArg("p")) hitPlaneCm=clampf(server.arg("p").toFloat(),4.0f,40.0f);
  if(server.hasArg("s")) hitSpeedCmS=clampf(server.arg("s").toFloat(),5.0f,300.0f);
  if(server.hasArg("r")) rearmMarginCm=clampf(server.arg("r").toFloat(),1.0f,12.0f);
  if(server.hasArg("q")) displayStepCm=clampf(server.arg("q").toFloat(),0.1f,2.0f);
  if(server.hasArg("a")) coneHalfAngleDeg=clampf(server.arg("a").toFloat(),2.0f,30.0f);
  addLog("CFG hit="+String(hitPlaneCm,1)+" speed="+String(hitSpeedCmS,0)+" rearm="+String(rearmMarginCm,1)+" q="+String(displayStepCm,1));
  server.send(200,"text/plain","OK");
}

void setup() {
  Serial.begin(115200); delay(100);
  for(uint8_t i=0;i<2;i++){ pinMode(PINS[i].echo,INPUT); pinMode(PINS[i].trig,OUTPUT); digitalWrite(PINS[i].trig,LOW); }
  WiFi.mode(WIFI_AP); WiFi.softAP(AP_SSID);
  server.on("/",[](){server.send_P(200,"text/html",PAGE);});
  server.on("/api/status",sendStatus);
  server.on("/api/config",handleConfig);
  server.on("/api/reset",[](){ pair.hits=0;pair.lastHitMs=0;pair.lastHitVelocity=0;pair.lastHitZ=0;pair.lastHitMidi=0;pair.lastZone="--";pair.peakApproachCmS=0;pair.armed=true;server.send(200,"text/plain","OK"); });
  server.on("/api/log",[](){server.send(200,"text/plain",logBuffer);});
  server.begin();
  nextMeasureMs=millis()+300;
  addLog("BOOT "+String(FW_VERSION));
  addLog("ONE BONGO / 2 HC-SR04 / GPIO13-16 / slot=30ms");
  addLog("AP "+String(AP_SSID)+" http://192.168.4.1");
}

void loop() { server.handleClient(); serviceSensors(); delay(1); }
