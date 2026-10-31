/*
  SONIC BONGO / PRISMA
  ESP32-S3 N16R8 - Dual Sensor Bongo v12 CALIBRATION LAB

  CONFIRMED WIRING (unchanged):
    S1: GPIO13 -> TRIG direct
        HC-SR04 ECHO -> TXS B1 -> A1 -> GPIO14
    S2: GPIO15 -> TRIG direct
        HC-SR04 ECHO -> TXS B2 -> A2 -> GPIO16

  Goals v12:
    - Work only inside a configurable useful range (default 50 cm).
    - Dynamic pulseIn timeout derived from max range, avoiding long blocking waits.
    - Start/stop acquisition from web; sliders editable while stopped.
    - Median-of-3 + FAST + STABLE paths.
    - HIT engine stays on fast path; UI uses stable/quantized path.
    - Sampling interval configurable (30..100 ms global alternating slot).
    - Configurable hit plane, speed, rearm, min hit interval, smoothing,
      coherence threshold, display granularity, baseline and cone model.
    - Explicit hit event sequence for reliable browser beep/sample triggering.
    - Geometry/lateral result remains diagnostic with 1 cm baseline.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

WebServer server(80);

static const char* FW_VERSION = "12.0-calibration-bongo";
static const char* AP_SSID = "PRISMA-S3-DIAG";

struct SensorPins { uint8_t trig; uint8_t echo; };
const SensorPins PINS[2] = { {13,14}, {15,16} };

// Runtime configuration
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
  bool have = false;
  bool ok = false;
  bool inRange = false;
  uint32_t lastGoodMs = 0;
  uint32_t lastAttemptMs = 0;
  uint32_t pulseUs = 0;
  float rawCm = 0;
  float medianCm = 0;
  float fastCm = 0;
  float stableCm = 0;
  float displayCm = 0;
  float prevFastCm = 0;
  uint32_t prevFastMs = 0;
  float velocityCmS = 0;
  float jitterCm = 0;
  float rawHist[3] = {0,0,0};
  uint8_t rawHistCount = 0;
  uint8_t rawHistHead = 0;
  uint32_t valid = 0;
  uint32_t timeouts = 0;
  uint32_t rejectedRange = 0;
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
  float deltaStableCm = 0;
  float xCm = 0;
  float coherencePct = 0;
  float jitterCm = 0;
  String lateral = "INDETERMINADO";
  float lateralConfidence = 0;
  uint32_t hits = 0;
  uint32_t lastHitMs = 0;
  float lastHitVelocity = 0;
  float lastHitZ = 0;
  int lastHitMidi = 0;
  String lastZone = "--";
  uint32_t eventSeq = 0;
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
uint32_t clampu(uint32_t v, uint32_t lo, uint32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }
float quantize(float v, float step) { return step <= 0.001f ? v : roundf(v / step) * step; }

void addLog(const String& text) {
  String line = "[" + String(millis()) + " ms] " + text + "\n";
  logBuffer += line;
  if (logBuffer.length() > 18000) logBuffer.remove(0, 3000);
  Serial.print(line);
}

uint32_t echoTimeoutUs() {
  // HC-SR04 echo time is ~58.3 us/cm round trip. Add margin, but do not
  // permit a very long blocking timeout in the bongo calibration range.
  float u = maxRangeCm * 58.3f + 1200.0f;
  return (uint32_t)clampf(u, 2500.0f, 12000.0f);
}

uint32_t staleMs() {
  return clampu(slotMs * 6, 160, 700);
}

float median3(float a, float b, float c) {
  if (a > b) { float t=a; a=b; b=t; }
  if (b > c) { float t=b; b=c; c=t; }
  if (a > b) { float t=a; a=b; b=t; }
  return b;
}

float pushMedian(SensorState &q, float raw) {
  q.rawHist[q.rawHistHead] = raw;
  q.rawHistHead = (q.rawHistHead + 1) % 3;
  if (q.rawHistCount < 3) q.rawHistCount++;
  if (q.rawHistCount == 1) return q.rawHist[0];
  if (q.rawHistCount == 2) return 0.5f * (q.rawHist[0] + q.rawHist[1]);
  return median3(q.rawHist[0], q.rawHist[1], q.rawHist[2]);
}

int midiFromVelocity(float approachSpeed) {
  float x = clampf((approachSpeed - 20.0f) / 220.0f, 0.0f, 1.0f);
  return 1 + (int)lroundf(x * 126.0f);
}

void updateLateral() {
  float d = pair.deltaStableCm;
  float a = fabsf(d);
  float dead = 0.20f;
  float decisive = 0.45f;
  if (a <= dead) {
    pair.lateral = "CENTRO";
    pair.lateralConfidence = clampf((dead - a) / dead * 100.0f, 0, 100);
  } else if (a >= decisive) {
    pair.lateral = d < 0 ? "S1 / IZQUIERDA" : "S2 / DERECHA";
    pair.lateralConfidence = clampf(30.0f + (a - decisive) * 80.0f, 0, 100);
  } else {
    pair.lateral = "INDETERMINADO";
    pair.lateralConfidence = 0;
  }
}

void updateRates() {
  uint32_t now = millis();
  if (!rateMarkMs) {
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

void resetTracking(bool keepHits=true) {
  for (uint8_t i=0;i<2;i++) {
    s[i].have = false;
    s[i].ok = false;
    s[i].inRange = false;
    s[i].rawHistCount = 0;
    s[i].rawHistHead = 0;
    s[i].velocityCmS = 0;
    s[i].jitterCm = 0;
  }
  pair.valid = false;
  pair.geoValid = false;
  pair.armed = true;
  pair.prevZMs = 0;
  pair.velocityCmS = 0;
  pair.jitterCm = 0;
  if (!keepHits) {
    pair.hits = 0;
    pair.eventSeq = 0;
    pair.lastHitMs = 0;
    pair.lastHitVelocity = 0;
    pair.lastHitZ = 0;
    pair.lastHitMidi = 0;
    pair.lastZone = "--";
  }
}

void updatePair() {
  uint32_t now = millis();
  uint32_t stale = staleMs();
  bool fresh0 = s[0].ok && s[0].inRange && (now - s[0].lastGoodMs <= stale);
  bool fresh1 = s[1].ok && s[1].inRange && (now - s[1].lastGoodMs <= stale);
  pair.valid = fresh0 && fresh1;
  if (!pair.valid) return;

  // Predict each channel slightly to compensate alternating timestamps.
  float maxPredict = clampf(slotMs / 1000.0f * 1.5f, 0.03f, 0.12f);
  float age0 = clampf((now - s[0].lastGoodMs) / 1000.0f, 0, maxPredict);
  float age1 = clampf((now - s[1].lastGoodMs) / 1000.0f, 0, maxPredict);
  float p0 = s[0].fastCm + clampf(s[0].velocityCmS * age0, -5.0f, 5.0f);
  float p1 = s[1].fastCm + clampf(s[1].velocityCmS * age1, -5.0f, 5.0f);
  float zFastNew = 0.5f * (p0 + p1);

  pair.zStableCm = 0.5f * (s[0].stableCm + s[1].stableCm);
  pair.zDisplayCm = quantize(pair.zStableCm, displayStepCm);
  pair.deltaStableCm = s[0].stableCm - s[1].stableCm;
  pair.jitterCm = 0.5f * (s[0].jitterCm + s[1].jitterCm);

  if (pair.prevZMs && now > pair.prevZMs) {
    float dt = (now - pair.prevZMs) / 1000.0f;
    if (dt > 0.008f && dt < 0.30f) {
      float rawV = (zFastNew - pair.prevZFastCm) / dt;
      // Reject absurd derivatives caused by echo swaps/outliers.
      rawV = clampf(rawV, -600.0f, 600.0f);
      pair.velocityCmS = pairVelAlpha * rawV + (1.0f - pairVelAlpha) * pair.velocityCmS;
    }
  }
  pair.zFastCm = zFastNew;
  pair.prevZFastCm = zFastNew;
  pair.prevZMs = now;
  pair.pairUpdates++;

  pair.coherencePct = clampf(100.0f - fabsf(pair.deltaStableCm)*22.0f - pair.jitterCm*10.0f, 0, 100);

  // Diagnostic 2D geometry only.
  pair.geoValid = false;
  pair.xCm = 0;
  if (baselineCm > 0.01f && fabsf(pair.deltaStableCm) <= baselineCm) {
    float d1=s[0].stableCm, d2=s[1].stableCm;
    float x=(d1*d1-d2*d2)/(2.0f*baselineCm);
    float z2=d1*d1-(x+baselineCm*0.5f)*(x+baselineCm*0.5f);
    if (z2 >= 0) { pair.xCm=x; pair.geoValid=true; }
  }
  updateLateral();

  bool enteredPlane = pair.zFastCm <= hitPlaneCm;
  bool fastEnough = pair.velocityCmS <= -hitSpeedCmS;
  bool coherentEnough = pair.coherencePct >= minCoherencePct;
  bool intervalOK = (now - pair.lastHitMs) >= minHitIntervalMs;

  if (pair.armed && enteredPlane && fastEnough && coherentEnough && intervalOK) {
    pair.hits++;
    pair.eventSeq++;
    pair.lastHitMs = now;
    pair.lastHitVelocity = -pair.velocityCmS;
    pair.lastHitZ = pair.zFastCm;
    pair.lastHitMidi = midiFromVelocity(pair.lastHitVelocity);
    pair.lastZone = pair.lateral;
    pair.armed = false;
    addLog("HIT seq="+String(pair.eventSeq)+" #"+String(pair.hits)+" z="+String(pair.lastHitZ,2)+" v="+String(pair.lastHitVelocity,1)+" midi="+String(pair.lastHitMidi)+" zone="+pair.lastZone+" coh="+String(pair.coherencePct,0));
  }

  if (!pair.armed && intervalOK && pair.zFastCm >= hitPlaneCm + rearmMarginCm) {
    pair.armed = true;
  }
}

void processValid(uint8_t i, uint32_t pulse) {
  SensorState &q=s[i];
  uint32_t now=millis();
  float raw=((float)pulse*0.0343f)/2.0f;
  q.lastAttemptMs=now;
  q.pulseUs=pulse;
  q.rawCm=raw;

  if (raw < 2.0f || raw > maxRangeCm) {
    q.ok=false;
    q.inRange=false;
    q.rejectedRange++;
    return;
  }

  q.ok=true;
  q.inRange=true;
  q.lastGoodMs=now;
  q.valid++;
  float med=pushMedian(q,raw);
  q.medianCm=med;

  if (!q.have) {
    q.have=true;
    q.fastCm=raw;
    q.stableCm=med;
    q.displayCm=quantize(med,displayStepCm);
    q.prevFastCm=q.fastCm;
    q.prevFastMs=now;
    q.velocityCmS=0;
    q.jitterCm=0;
    return;
  }

  // FAST follows raw closely, but clips implausible one-sample jumps.
  float diff=raw-q.fastCm;
  float guarded=raw;
  if (diff > maxJumpCm) guarded=q.fastCm+maxJumpCm;
  if (diff < -maxJumpCm) guarded=q.fastCm-maxJumpCm;
  float fastNew=fastAlpha*guarded+(1.0f-fastAlpha)*q.fastCm;

  // STABLE uses median-of-3 and a slower EMA.
  q.stableCm=stableAlpha*med+(1.0f-stableAlpha)*q.stableCm;
  q.displayCm=quantize(q.stableCm,displayStepCm);
  q.jitterCm=0.90f*q.jitterCm+0.10f*fabsf(raw-med);

  if (now > q.prevFastMs) {
    float dt=(now-q.prevFastMs)/1000.0f;
    if (dt > 0.025f && dt < 0.35f) {
      float rawV=(fastNew-q.prevFastCm)/dt;
      rawV=clampf(rawV,-600.0f,600.0f);
      q.velocityCmS=sensorVelAlpha*rawV+(1.0f-sensorVelAlpha)*q.velocityCmS;
    }
  }
  q.fastCm=fastNew;
  q.prevFastCm=fastNew;
  q.prevFastMs=now;
}

void processTimeout(uint8_t i) {
  SensorState &q=s[i];
  q.ok=false;
  q.inRange=false;
  q.pulseUs=0;
  q.lastAttemptMs=millis();
  q.timeouts++;
}

void measureOne(uint8_t i) {
  const SensorPins &p=PINS[i];
  digitalWrite(p.trig,LOW); delayMicroseconds(3);
  digitalWrite(p.trig,HIGH); delayMicroseconds(10);
  digitalWrite(p.trig,LOW);
  uint32_t pulse=pulseIn(p.echo,HIGH,echoTimeoutUs());
  if (pulse) processValid(i,pulse); else processTimeout(i);
  updatePair();
  updateRates();
}

void serviceSensors() {
  if (!acquisitionRunning) return;
  uint32_t now=millis();
  if ((int32_t)(now-nextMeasureMs)<0) return;
  uint8_t i=nextSensor;
  nextSensor^=1;
  measureOne(i);
  nextMeasureMs=millis()+slotMs;
}

String motionLabel() {
  if (!pair.valid) return "SIN PAR";
  if (pair.velocityCmS <= -18) return "ACERCANDO";
  if (pair.velocityCmS >= 18) return "ALEJANDO";
  return "ESTABLE";
}

const char PAGE[] PROGMEM=R"HTML(
<!doctype html><html lang="es"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>SONIC BONGO v12</title>
<style>
:root{--bg:#070b10;--p:#0e151d;--p2:#121d27;--l:#263746;--t:#ecf4fb;--m:#7f93a5;--s1:#59e8cc;--s2:#719dff;--hit:#ffc85a;--ok:#45df84;--bad:#ff6565;--blue:#59b8ff}*{box-sizing:border-box}body{margin:0;background:linear-gradient(180deg,#070b10,#09121b 65%,#070c12);color:var(--t);font-family:Inter,system-ui,-apple-system,Segoe UI,sans-serif}header,.wrap{max-width:1240px;margin:auto;padding:13px 14px}header{display:flex;justify-content:space-between;align-items:flex-end;gap:12px}h1{font-size:20px;margin:0}.sub,.small{font-size:10px;color:var(--m)}.statusbar{display:flex;gap:7px;align-items:center;flex-wrap:wrap}.pill{padding:5px 8px;border:1px solid var(--l);background:#0b131b;border-radius:999px;font-size:10px;font-weight:800}.on{color:var(--ok);border-color:#286044}.off{color:var(--bad);border-color:#633336}.cards{display:grid;grid-template-columns:repeat(7,minmax(100px,1fr));gap:7px;margin-bottom:8px}.card,.panel{background:rgba(14,21,29,.96);border:1px solid var(--l);border-radius:11px;padding:10px}.k{font-size:9px;letter-spacing:.08em;color:var(--m)}.v{font-size:18px;font-weight:900;margin-top:3px}.s1{color:var(--s1)}.s2{color:var(--s2)}.hit{color:var(--hit)}.grid{display:grid;grid-template-columns:1.15fr .85fr;gap:8px}.row{display:flex;justify-content:space-between;gap:12px;margin:5px 0;font-size:11px}.row b{font-size:12px}.sectionTitle{font-size:11px;font-weight:900;letter-spacing:.05em}.controls{display:grid;grid-template-columns:1fr 1fr;gap:9px 12px;margin-top:8px}.ctrl{background:#0a1118;border:1px solid #223341;border-radius:8px;padding:8px}.ctrlHead{display:flex;justify-content:space-between;gap:8px;font-size:10px}.ctrlHead b{color:#dce8f0}.ctrl input[type=range]{width:100%;margin-top:7px;accent-color:#82bfff}.ctrl.disabled{opacity:.38}.buttons{display:flex;gap:7px;flex-wrap:wrap;margin-top:9px}button{border:1px solid #385064;background:#132230;color:white;padding:9px 11px;border-radius:8px;font-weight:850;cursor:pointer}.primary{background:#123b2b;border-color:#2a7652}.danger{background:#34191b;border-color:#72383d}.audio{background:#18263a;border-color:#405f87}.audio.on{background:#203b24;border-color:#4c8a53;color:white}canvas{display:block;width:100%;height:auto;background:#050a0f;border:1px solid #1e2d38;border-radius:9px}.legend{display:flex;gap:12px;flex-wrap:wrap;margin-top:6px;font-size:9px;color:var(--m)}.dot{display:inline-block;width:7px;height:7px;border-radius:50%;margin-right:4px}.warn{color:#ffb35b}.good{color:var(--ok)}.bad{color:var(--bad)}.meter{height:7px;background:#071017;border-radius:8px;overflow:hidden;border:1px solid #1f303d}.meter>i{display:block;height:100%;width:0;background:#60d698}.advanced{margin-top:8px;border-top:1px solid #20303c;padding-top:8px}@media(max-width:980px){header{align-items:flex-start;flex-direction:column}.grid{grid-template-columns:1fr}.cards{grid-template-columns:repeat(2,1fr)}.controls{grid-template-columns:1fr}}
</style></head><body>
<header><div><h1>SONIC BONGO · CALIBRATION LAB v12</h1><div class="sub">2 HC-SR04 = 1 parche · rango útil configurable · median/FAST/STABLE · eventos HIT preparados para samples</div></div><div class="statusbar"><span id="runPill" class="pill">...</span><span id="audioPill" class="pill off">AUDIO OFF</span></div></header>
<div class="wrap">
<div class="cards"><div class="card"><div class="k">S1</div><div id="d1" class="v s1">--</div></div><div class="card"><div class="k">S2</div><div id="d2" class="v s2">--</div></div><div class="card"><div class="k">Z ESTABLE</div><div id="z" class="v">--</div></div><div class="card"><div class="k">VELOCIDAD</div><div id="velTop" class="v">--</div></div><div class="card"><div class="k">HITS</div><div id="hits" class="v hit">0</div></div><div class="card"><div class="k">Hz COMBINADOS</div><div id="hz" class="v">--</div></div><div class="card"><div class="k">JITTER</div><div id="jitTop" class="v">--</div></div></div>
<div class="grid">
<section class="panel"><div class="row"><span class="sectionTitle">PARCHE VIRTUAL · 0–<span id="graphMax">50</span> cm</span><b id="motion">--</b></div><canvas id="geo" width="760" height="470"></canvas><div class="legend"><span><i class="dot" style="background:#59e8cc"></i>S1</span><span><i class="dot" style="background:#719dff"></i>S2</span><span><i class="dot" style="background:#fff"></i>Z estable</span><span><i class="dot" style="background:#ffc85a"></i>plano HIT</span></div><div class="small">Fuera del rango máximo la lectura se descarta antes de entrar al filtro. Izquierda/derecha sigue siendo un sesgo diagnóstico con baseline de 1 cm.</div></section>
<section class="panel"><div class="row"><span class="sectionTitle">ESTADO / CALIDAD</span><b id="cohTxt">--</b></div><div class="row"><span>RAW S1 / S2</span><b id="raw">--</b></div><div class="row"><span>MEDIAN S1 / S2</span><b id="median">--</b></div><div class="row"><span>FAST Z</span><b id="fastz">--</b></div><div class="row"><span>STABLE Z</span><b id="stablez">--</b></div><div class="row"><span>Rechazadas por rango</span><b id="rejects">--</b></div><div class="row"><span>Timeouts</span><b id="timeouts">--</b></div><div class="row"><span>Hz S1 / S2</span><b id="sensorHz">--</b></div><div class="row"><span>Nominal S1/S2</span><b id="nominalHz">--</b></div><div class="row"><span>Timeout actual</span><b id="timeoutUs">--</b></div><div class="row"><span>Sesgo lateral</span><b id="lateral">--</b></div><div class="row"><span>Conf. lateral</span><b id="latConf">--</b></div><div class="row"><span>ARMADO</span><b id="armed">--</b></div><div class="row"><span>Último HIT</span><b id="last">--</b></div><div class="meter"><i id="cohMeter"></i></div><div class="buttons"><button id="stopBtn" class="danger" onclick="setRun(false)">DETENER</button><button id="startBtn" class="primary" onclick="setRun(true)">INICIAR</button><button id="soundBtn" class="audio" onclick="toggleSound()">SONIDO OFF</button><button onclick="resetHits()">RESET HITS</button></div></section>
</div>
<section class="panel" style="margin-top:8px"><div class="row"><span class="sectionTitle">CALIBRACIÓN</span><span id="editHint" class="small">Detené para editar</span></div><div id="controls" class="controls">
<div class="ctrl"><div class="ctrlHead"><span>Rango máximo</span><b><span id="maxRangeV">50</span> cm</b></div><input id="maxRange" type="range" min="20" max="100" step="5" value="50"></div>
<div class="ctrl"><div class="ctrlHead"><span>Plano HIT</span><b><span id="hitPlaneV">12</span> cm</b></div><input id="hitPlane" type="range" min="5" max="30" step="0.5" value="12"></div>
<div class="ctrl"><div class="ctrlHead"><span>Velocidad mínima HIT</span><b><span id="hitSpeedV">35</span> cm/s</b></div><input id="hitSpeed" type="range" min="10" max="180" step="5" value="35"></div>
<div class="ctrl"><div class="ctrlHead"><span>Retirada para rearmar</span><b><span id="rearmV">3</span> cm</b></div><input id="rearm" type="range" min="1" max="10" step="0.5" value="3"></div>
<div class="ctrl"><div class="ctrlHead"><span>Intervalo mínimo HIT</span><b><span id="hitIntervalV">100</span> ms</b></div><input id="hitInterval" type="range" min="70" max="300" step="10" value="100"></div>
<div class="ctrl"><div class="ctrlHead"><span>Intervalo entre pings</span><b><span id="slotV">30</span> ms</b></div><input id="slot" type="range" min="30" max="100" step="5" value="30"></div>
<div class="ctrl"><div class="ctrlHead"><span>FAST alpha</span><b><span id="fastAlphaV">0.72</span></b></div><input id="fastAlpha" type="range" min="0.45" max="0.90" step="0.01" value="0.72"></div>
<div class="ctrl"><div class="ctrlHead"><span>STABLE alpha</span><b><span id="stableAlphaV">0.20</span></b></div><input id="stableAlpha" type="range" min="0.05" max="0.50" step="0.01" value="0.20"></div>
<div class="ctrl"><div class="ctrlHead"><span>Coherencia mínima HIT</span><b><span id="cohMinV">18</span>%</b></div><input id="cohMin" type="range" min="0" max="80" step="5" value="18"></div>
<div class="ctrl"><div class="ctrlHead"><span>Granularidad display</span><b><span id="displayStepV">0.5</span> cm</b></div><input id="displayStep" type="range" min="0.1" max="2" step="0.1" value="0.5"></div>
<div class="ctrl"><div class="ctrlHead"><span>Baseline</span><b><span id="baselineV">1.0</span> cm</b></div><input id="baseline" type="range" min="0.5" max="10" step="0.5" value="1"></div>
<div class="ctrl"><div class="ctrlHead"><span>Semiángulo visual</span><b><span id="coneV">7.5</span>°</b></div><input id="cone" type="range" min="3" max="20" step="0.5" value="7.5"></div>
</div><div class="buttons"><button id="applyBtn" onclick="applyCfg()">APLICAR CONFIGURACIÓN</button></div><div class="small">30 ms es el mínimo permitido: equivale nominalmente a ~16,7 Hz por sensor y ~33,3 Hz combinados. Valores mayores reducen interferencia pero también la resolución temporal.</div></section>
<section class="panel" style="margin-top:8px"><div class="row"><span class="sectionTitle">TIMELINE</span><span class="small">RAW tenue · FAST · STABLE · HIT</span></div><canvas id="tl" width="1160" height="280"></canvas></section>
</div>
<script>
const $=id=>document.getElementById(id);const G=$('geo').getContext('2d'),T=$('tl').getContext('2d');const hist=[];let audioCtx=null,soundOn=false,lastEventSeq=null,firstStatus=true,currentRunning=true;
const sliderIds=['maxRange','hitPlane','hitSpeed','rearm','hitInterval','slot','fastAlpha','stableAlpha','cohMin','displayStep','baseline','cone'];
const labels={maxRange:'maxRangeV',hitPlane:'hitPlaneV',hitSpeed:'hitSpeedV',rearm:'rearmV',hitInterval:'hitIntervalV',slot:'slotV',fastAlpha:'fastAlphaV',stableAlpha:'stableAlphaV',cohMin:'cohMinV',displayStep:'displayStepV',baseline:'baselineV',cone:'coneV'};
sliderIds.forEach(id=>$(id).addEventListener('input',()=>$(labels[id]).textContent=$(id).value));
function ln(c,x1,y1,x2,y2,col='#334754',w=1){c.beginPath();c.moveTo(x1,y1);c.lineTo(x2,y2);c.strokeStyle=col;c.lineWidth=w;c.stroke()}function tx(c,s,x,y,col='#8094a5',sz=10,a='left'){c.fillStyle=col;c.font=sz+'px system-ui';c.textAlign=a;c.fillText(s,x,y)}
function enableControls(on){sliderIds.forEach(id=>$(id).disabled=!on);$('applyBtn').disabled=!on;document.querySelectorAll('.ctrl').forEach(e=>e.classList.toggle('disabled',!on));$('editHint').textContent=on?'Editá y aplicá antes de iniciar':'Detené para editar'}
async function toggleSound(){if(!audioCtx)audioCtx=new(window.AudioContext||window.webkitAudioContext)();await audioCtx.resume();soundOn=!soundOn;$('soundBtn').textContent=soundOn?'SONIDO ON':'SONIDO OFF';$('soundBtn').classList.toggle('on',soundOn);$('audioPill').textContent=soundOn?'AUDIO ON':'AUDIO OFF';$('audioPill').className='pill '+(soundOn?'on':'off');if(soundOn)playHit(55)}
function playHit(midi){if(!soundOn||!audioCtx)return;const n=audioCtx.currentTime,o=audioCtx.createOscillator(),g=audioCtx.createGain();o.type='triangle';o.frequency.setValueAtTime(540+Math.min(127,midi)*3.2,n);g.gain.setValueAtTime(.0001,n);g.gain.exponentialRampToValueAtTime(.16,n+.003);g.gain.exponentialRampToValueAtTime(.0001,n+.07);o.connect(g);g.connect(audioCtx.destination);o.start(n);o.stop(n+.075)}
async function setRun(on){await fetch('/api/run?on='+(on?1:0),{cache:'no-store'});await refresh()}
async function resetHits(){await fetch('/api/reset',{cache:'no-store'});lastEventSeq=null;await refresh()}
async function applyCfg(){const q=new URLSearchParams({range:$('maxRange').value,plane:$('hitPlane').value,speed:$('hitSpeed').value,rearm:$('rearm').value,interval:$('hitInterval').value,slot:$('slot').value,fa:$('fastAlpha').value,sa:$('stableAlpha').value,coh:$('cohMin').value,step:$('displayStep').value,base:$('baseline').value,cone:$('cone').value});await fetch('/api/config?'+q.toString(),{cache:'no-store'});hist.length=0;await refresh()}
function syncSliders(j){const c=j.cfg;const vals={maxRange:c.maxRange,hitPlane:c.hitPlane,hitSpeed:c.hitSpeed,rearm:c.rearm,hitInterval:c.minHitInterval,slot:c.slotMs,fastAlpha:c.fastAlpha,stableAlpha:c.stableAlpha,cohMin:c.minCoherence,displayStep:c.displayStep,baseline:c.baseline,cone:c.coneHalfAngle};Object.entries(vals).forEach(([id,v])=>{if(firstStatus||!currentRunning){$(id).value=v;$(labels[id]).textContent=v}})}
function drawGeo(j){const W=760,H=470,by=430,cx=W/2,max=j.cfg.maxRange,sc=(H-66)/max;G.clearRect(0,0,W,H);for(let q=10;q<=max;q+=10){const y=by-q*sc;ln(G,44,y,W-20,y,'#13222d');tx(G,q+' cm',7,y+3)}const x1=cx-j.cfg.baseline*.5*sc,x2=cx+j.cfg.baseline*.5*sc,hy=by-j.cfg.hitPlane*sc;ln(G,44,hy,W-20,hy,'#ffc85a',2);tx(G,'HIT '+j.cfg.hitPlane.toFixed(1)+' cm',W-24,hy-6,'#ffc85a',11,'right');const len=max*sc,spread=Math.tan(j.cfg.coneHalfAngle*Math.PI/180)*len;[[x1,'#59e8cc'],[x2,'#719dff']].forEach(v=>{G.save();G.globalAlpha=.08;G.beginPath();G.moveTo(v[0],by);G.lineTo(v[0]-spread,by-len);G.lineTo(v[0]+spread,by-len);G.closePath();G.fillStyle=v[1];G.fill();G.restore();ln(G,v[0],by,v[0]-spread,by-len,v[1]);ln(G,v[0],by,v[0]+spread,by-len,v[1])});G.fillStyle='#59e8cc';G.fillRect(x1-7,by-5,14,10);G.fillStyle='#719dff';G.fillRect(x2-7,by-5,14,10);tx(G,'S1',x1,by+20,'#59e8cc',10,'center');tx(G,'S2',x2,by+20,'#719dff',10,'center');if(j.pair.valid&&j.pair.zDisplay<=max){const py=by-j.pair.zDisplay*sc;G.beginPath();G.arc(cx,py,8,0,Math.PI*2);G.fillStyle='#fff';G.fill();tx(G,j.pair.zDisplay.toFixed(1)+' cm',cx+13,py-8,'#fff',11);if(j.pair.geoValid){let px=cx+j.pair.x*sc;G.beginPath();G.arc(px,py,4,0,Math.PI*2);G.fillStyle='#ffc85a';G.fill()}}}
function drawTL(j){const W=1160,H=280,p=36,max=j.cfg.maxRange,y=v=>H-p-v/max*(H-2*p);T.clearRect(0,0,W,H);for(let q=10;q<=max;q+=10){ln(T,p,y(q),W-10,y(q),'#13222d');tx(T,q,8,y(q)+3)}ln(T,p,y(j.cfg.hitPlane),W-10,y(j.cfg.hitPlane),'#ffc85a',1.5);function ser(k,col,w,a){T.save();T.globalAlpha=a;T.beginPath();let started=false;hist.forEach((r,i)=>{let v=r[k];if(!Number.isFinite(v)||v>max){started=false;return}let x=p+i/Math.max(1,hist.length-1)*(W-p-15);if(!started){T.moveTo(x,y(v));started=true}else T.lineTo(x,y(v))});T.strokeStyle=col;T.lineWidth=w;T.stroke();T.restore()}ser('r1','#59e8cc',1,.18);ser('r2','#719dff',1,.18);ser('fast','#b7c5d0',1.2,.55);ser('stable','#fff',2.4,1);hist.forEach((r,i)=>{if(r.hit){let x=p+i/Math.max(1,hist.length-1)*(W-p-15);ln(T,x,12,x,H-p,'#ffc85a',2)}})}
async function refresh(){try{const r=await fetch('/api/status',{cache:'no-store'}),j=await r.json();currentRunning=j.running;enableControls(!j.running);$('runPill').textContent=j.running?'DETECCIÓN ACTIVA':'DETECCIÓN DETENIDA';$('runPill').className='pill '+(j.running?'on':'off');$('d1').textContent=j.s1.ok?j.s1.display.toFixed(1)+' cm':'--';$('d2').textContent=j.s2.ok?j.s2.display.toFixed(1)+' cm':'--';$('z').textContent=j.pair.valid?j.pair.zDisplay.toFixed(1)+' cm':'--';$('velTop').textContent=j.pair.valid?j.pair.velocity.toFixed(0)+' cm/s':'--';$('hits').textContent=j.pair.hits;$('hz').textContent=j.pair.combinedHz.toFixed(1);$('jitTop').textContent=j.pair.jitter.toFixed(2)+' cm';$('motion').textContent=j.pair.motion;$('graphMax').textContent=j.cfg.maxRange.toFixed(0);$('raw').textContent=j.s1.raw.toFixed(1)+' / '+j.s2.raw.toFixed(1)+' cm';$('median').textContent=j.s1.median.toFixed(1)+' / '+j.s2.median.toFixed(1)+' cm';$('fastz').textContent=j.pair.valid?j.pair.zFast.toFixed(2)+' cm':'--';$('stablez').textContent=j.pair.valid?j.pair.zStable.toFixed(2)+' cm':'--';$('rejects').textContent=j.s1.rejected+' / '+j.s2.rejected;$('timeouts').textContent=j.s1.timeouts+' / '+j.s2.timeouts;$('sensorHz').textContent=j.s1.hz.toFixed(1)+' / '+j.s2.hz.toFixed(1);$('nominalHz').textContent=j.cfg.nominalPerSensorHz.toFixed(1)+' Hz';$('timeoutUs').textContent=j.cfg.echoTimeoutUs+' us';$('lateral').textContent=j.pair.lateral;$('latConf').textContent=j.pair.lateralConfidence.toFixed(0)+'%';$('armed').textContent=j.pair.armed?'SÍ':'NO';$('last').textContent=j.pair.lastHitMs?('#'+j.pair.hits+' · '+j.pair.lastHitZ.toFixed(1)+' cm · '+j.pair.lastHitVelocity.toFixed(0)+' cm/s · MIDI '+j.pair.lastMidi):'--';$('cohTxt').textContent='COHERENCIA '+j.pair.coherence.toFixed(0)+'%';$('cohMeter').style.width=j.pair.coherence.toFixed(0)+'%';syncSliders(j);let hit=false;if(lastEventSeq===null)lastEventSeq=j.event.seq;else if(j.event.seq!==lastEventSeq){hit=true;lastEventSeq=j.event.seq;playHit(j.event.midi||80)}hist.push({r1:j.s1.ok?j.s1.raw:NaN,r2:j.s2.ok?j.s2.raw:NaN,fast:j.pair.valid?j.pair.zFast:NaN,stable:j.pair.valid?j.pair.zDisplay:NaN,hit});if(hist.length>220)hist.shift();drawGeo(j);drawTL(j);firstStatus=false}catch(e){console.log(e)}}
setInterval(refresh,70);refresh();
</script></body></html>
)HTML";

void sendStatus() {
  String j="{";
  j += "\"fw\":\""+String(FW_VERSION)+"\",";
  j += "\"running\":"+String(acquisitionRunning?"true":"false")+",";
  for (uint8_t i=0;i<2;i++) {
    SensorState &q=s[i];
    j += "\"s"+String(i+1)+"\":{";
    j += "\"ok\":"+String(q.ok?"true":"false")+",";
    j += "\"raw\":"+String(q.rawCm,2)+",";
    j += "\"median\":"+String(q.medianCm,2)+",";
    j += "\"fast\":"+String(q.fastCm,2)+",";
    j += "\"stable\":"+String(q.stableCm,2)+",";
    j += "\"display\":"+String(q.displayCm,2)+",";
    j += "\"velocity\":"+String(q.velocityCmS,2)+",";
    j += "\"jitter\":"+String(q.jitterCm,2)+",";
    j += "\"valid\":"+String(q.valid)+",";
    j += "\"timeouts\":"+String(q.timeouts)+",";
    j += "\"rejected\":"+String(q.rejectedRange)+",";
    j += "\"hz\":"+String(q.hz,2)+"}";
    if (i==0) j += ",";
  }
  j += ",\"pair\":{";
  j += "\"valid\":"+String(pair.valid?"true":"false")+",";
  j += "\"geoValid\":"+String(pair.geoValid?"true":"false")+",";
  j += "\"zFast\":"+String(pair.zFastCm,2)+",";
  j += "\"zStable\":"+String(pair.zStableCm,2)+",";
  j += "\"zDisplay\":"+String(pair.zDisplayCm,2)+",";
  j += "\"velocity\":"+String(pair.velocityCmS,2)+",";
  j += "\"delta\":"+String(pair.deltaStableCm,2)+",";
  j += "\"x\":"+String(pair.xCm,2)+",";
  j += "\"coherence\":"+String(pair.coherencePct,1)+",";
  j += "\"jitter\":"+String(pair.jitterCm,2)+",";
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
  j += "\"seq\":"+String(pair.eventSeq)+",";
  j += "\"pad\":1,";
  j += "\"midi\":"+String(pair.lastHitMidi)+",";
  j += "\"velocity\":"+String(pair.lastHitVelocity,1)+",";
  j += "\"z\":"+String(pair.lastHitZ,2)+",";
  j += "\"zone\":\""+pair.lastZone+"\"},";
  j += "\"cfg\":{";
  j += "\"maxRange\":"+String(maxRangeCm,1)+",";
  j += "\"hitPlane\":"+String(hitPlaneCm,1)+",";
  j += "\"hitSpeed\":"+String(hitSpeedCmS,1)+",";
  j += "\"rearm\":"+String(rearmMarginCm,1)+",";
  j += "\"minHitInterval\":"+String(minHitIntervalMs)+",";
  j += "\"slotMs\":"+String(slotMs)+",";
  j += "\"fastAlpha\":"+String(fastAlpha,2)+",";
  j += "\"stableAlpha\":"+String(stableAlpha,2)+",";
  j += "\"minCoherence\":"+String(minCoherencePct,0)+",";
  j += "\"displayStep\":"+String(displayStepCm,1)+",";
  j += "\"baseline\":"+String(baselineCm,1)+",";
  j += "\"coneHalfAngle\":"+String(coneHalfAngleDeg,1)+",";
  j += "\"echoTimeoutUs\":"+String(echoTimeoutUs())+",";
  j += "\"nominalPerSensorHz\":"+String(500.0f/slotMs,2)+",";
  j += "\"nominalCombinedHz\":"+String(1000.0f/slotMs,2)+"}";
  j += "}";
  server.send(200,"application/json",j);
}

void handleConfig() {
  if (acquisitionRunning) { server.send(409,"text/plain","STOP acquisition before config"); return; }
  if (server.hasArg("range")) maxRangeCm=clampf(server.arg("range").toFloat(),20,100);
  if (server.hasArg("plane")) hitPlaneCm=clampf(server.arg("plane").toFloat(),5,30);
  if (server.hasArg("speed")) hitSpeedCmS=clampf(server.arg("speed").toFloat(),10,180);
  if (server.hasArg("rearm")) rearmMarginCm=clampf(server.arg("rearm").toFloat(),1,10);
  if (server.hasArg("interval")) minHitIntervalMs=clampu(server.arg("interval").toInt(),70,300);
  if (server.hasArg("slot")) slotMs=clampu(server.arg("slot").toInt(),30,100);
  if (server.hasArg("fa")) fastAlpha=clampf(server.arg("fa").toFloat(),0.45f,0.90f);
  if (server.hasArg("sa")) stableAlpha=clampf(server.arg("sa").toFloat(),0.05f,0.50f);
  if (server.hasArg("coh")) minCoherencePct=clampf(server.arg("coh").toFloat(),0,80);
  if (server.hasArg("step")) displayStepCm=clampf(server.arg("step").toFloat(),0.1f,2.0f);
  if (server.hasArg("base")) baselineCm=clampf(server.arg("base").toFloat(),0.5f,10.0f);
  if (server.hasArg("cone")) coneHalfAngleDeg=clampf(server.arg("cone").toFloat(),3,20);
  resetTracking(true);
  addLog("CFG range="+String(maxRangeCm,0)+" hit="+String(hitPlaneCm,1)+" speed="+String(hitSpeedCmS,0)+" slot="+String(slotMs)+" timeout="+String(echoTimeoutUs()));
  server.send(200,"text/plain","OK");
}

void handleRun() {
  if (server.hasArg("on")) acquisitionRunning = server.arg("on") != "0";
  if (acquisitionRunning) {
    resetTracking(true);
    nextSensor=0;
    nextMeasureMs=millis()+80;
    addLog("ACQUISITION START slot="+String(slotMs)+"ms range="+String(maxRangeCm,0)+"cm");
  } else {
    addLog("ACQUISITION STOP");
  }
  server.send(200,"text/plain","OK");
}

void setup() {
  Serial.begin(115200); delay(120);
  for (uint8_t i=0;i<2;i++) {
    pinMode(PINS[i].echo,INPUT);
    pinMode(PINS[i].trig,OUTPUT);
    digitalWrite(PINS[i].trig,LOW);
  }
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID);
  server.on("/",[](){server.send_P(200,"text/html",PAGE);});
  server.on("/api/status",sendStatus);
  server.on("/api/config",handleConfig);
  server.on("/api/run",handleRun);
  server.on("/api/reset",[](){resetTracking(false);server.send(200,"text/plain","OK");});
  server.on("/api/log",[](){server.send(200,"text/plain",logBuffer);});
  server.begin();
  nextMeasureMs=millis()+300;
  addLog("BOOT "+String(FW_VERSION));
  addLog("GPIO13/14 + GPIO15/16; default range 50cm; slot 30ms");
  addLog("AP "+String(AP_SSID)+" http://192.168.4.1");
}

void loop() {
  server.handleClient();
  serviceSensors();
  delay(1);
}
