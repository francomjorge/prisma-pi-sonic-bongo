/*
  SONIC BONGO / PRISMA
  ESP32-S3 N16R8 - Dual Sensor Bongo v13

  CONFIRMED WIRING (unchanged):
    S1: GPIO13 -> TRIG direct
        HC-SR04 ECHO -> TXS B1 -> A1 -> GPIO14
    S2: GPIO15 -> TRIG direct
        HC-SR04 ECHO -> TXS B2 -> A2 -> GPIO16

  v13 goals:
    - Keep proven dual HC-SR04 acquisition path.
    - Separate GEOMETRY / ACQUISITION / HIT configuration.
    - Geometry preview updates locally before save.
    - POST /api/config, auto-pauses acquisition while applying.
    - Persist calibration in ESP32 NVS via Preferences.
    - Read-back verification before reporting success.
    - Professional geometry canvas with cones, circles, overlap and estimated point.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <math.h>

WebServer server(80);
Preferences prefs;

static const char* FW_VERSION = "13.0-geometry-lab";
static const char* AP_SSID = "PRISMA-S3-DIAG";
static const char* NVS_NS = "bongo13";

struct SensorPins { uint8_t trig; uint8_t echo; };
const SensorPins PINS[2] = {{13,14},{15,16}};

// Persisted calibration
float baselineCm = 1.0f;
float maxRangeCm = 50.0f;
float hitPlaneCm = 12.0f;
float coneHalfAngleDeg = 7.5f;
float targetHzPerSensor = 16.0f;
float smoothingPct = 55.0f;
float hitSpeedCmS = 35.0f;
float rearmMarginCm = 3.0f;

// Fixed engine constants: intentionally not exposed in this calibration pass.
static const float FAST_ALPHA = 0.78f;
static const float SENSOR_V_ALPHA = 0.36f;
static const float PAIR_V_ALPHA = 0.48f;
static const float MAX_JUMP_CM = 18.0f;
static const float MIN_COHERENCE_PCT = 15.0f;
static const uint32_t MIN_HIT_INTERVAL_MS = 90;

bool acquisitionRunning = true;

struct SensorState {
  bool have=false, ok=false, inRange=false;
  uint32_t lastGoodMs=0, pulseUs=0;
  float rawCm=0, medianCm=0, fastCm=0, stableCm=0;
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
  float zFast=0, zStable=0, prevZFast=0, velocity=0;
  uint32_t prevZMs=0;
  float delta=0, x=0, zGeo=0, coherence=0, jitter=0;
  String lateral="INDETERMINADO";
  float lateralConfidence=0;
  uint32_t hits=0, lastHitMs=0, eventSeq=0;
  float lastHitVelocity=0, lastHitZ=0;
  int lastMidi=0;
  uint32_t pairUpdates=0, pairMark=0;
  float combinedHz=0;
};
PairState pair;

uint8_t nextSensor=0;
uint32_t nextMeasureMs=0, rateMarkMs=0;

float clampf(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);} 
uint32_t clampu(uint32_t v,uint32_t lo,uint32_t hi){return v<lo?lo:(v>hi?hi:v);} 

float stableAlpha(){
  // 0% = fast visual response, 100% = maximum smoothing.
  return 0.45f - (clampf(smoothingPct,0,100) * 0.0037f);
}
uint32_t slotMs(){
  float hz=clampf(targetHzPerSensor,5.0f,16.0f);
  return (uint32_t)lroundf(500.0f/hz); // alternating: same sensor every 2 slots
}
uint32_t echoTimeoutUs(){
  float u=maxRangeCm*58.3f+1100.0f;
  return (uint32_t)clampf(u,2500.0f,9000.0f);
}
uint32_t staleMs(){return clampu(slotMs()*6,170,800);}

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

void loadConfig(){
  prefs.begin(NVS_NS,true);
  baselineCm=clampf(prefs.getFloat("base",1.0f),0.5f,10.0f);
  maxRangeCm=clampf(prefs.getFloat("range",50.0f),20.0f,100.0f);
  hitPlaneCm=clampf(prefs.getFloat("hit",12.0f),5.0f,30.0f);
  coneHalfAngleDeg=clampf(prefs.getFloat("angle",7.5f),3.0f,20.0f);
  targetHzPerSensor=clampf(prefs.getFloat("rate",16.0f),5.0f,16.0f);
  smoothingPct=clampf(prefs.getFloat("smooth",55.0f),0.0f,100.0f);
  hitSpeedCmS=clampf(prefs.getFloat("speed",35.0f),10.0f,180.0f);
  rearmMarginCm=clampf(prefs.getFloat("rearm",3.0f),1.0f,10.0f);
  prefs.end();
}

bool saveAndVerifyConfig(String &detail){
  prefs.begin(NVS_NS,false);
  prefs.putFloat("base",baselineCm);
  prefs.putFloat("range",maxRangeCm);
  prefs.putFloat("hit",hitPlaneCm);
  prefs.putFloat("angle",coneHalfAngleDeg);
  prefs.putFloat("rate",targetHzPerSensor);
  prefs.putFloat("smooth",smoothingPct);
  prefs.putFloat("speed",hitSpeedCmS);
  prefs.putFloat("rearm",rearmMarginCm);
  prefs.end();

  prefs.begin(NVS_NS,true);
  float rbBase=prefs.getFloat("base",-999), rbRange=prefs.getFloat("range",-999);
  float rbHit=prefs.getFloat("hit",-999), rbAngle=prefs.getFloat("angle",-999);
  float rbRate=prefs.getFloat("rate",-999), rbSmooth=prefs.getFloat("smooth",-999);
  float rbSpeed=prefs.getFloat("speed",-999), rbRearm=prefs.getFloat("rearm",-999);
  prefs.end();

  bool ok=fabsf(rbBase-baselineCm)<0.01f && fabsf(rbRange-maxRangeCm)<0.01f &&
          fabsf(rbHit-hitPlaneCm)<0.01f && fabsf(rbAngle-coneHalfAngleDeg)<0.01f &&
          fabsf(rbRate-targetHzPerSensor)<0.01f && fabsf(rbSmooth-smoothingPct)<0.01f &&
          fabsf(rbSpeed-hitSpeedCmS)<0.01f && fabsf(rbRearm-rearmMarginCm)<0.01f;
  detail=ok?"verified":"readback_mismatch";
  return ok;
}

void resetTracking(bool keepHits=true){
  for(int i=0;i<2;i++){
    s[i].have=false;s[i].ok=false;s[i].inRange=false;s[i].histCount=0;s[i].histHead=0;
    s[i].velocityCmS=0;s[i].jitterCm=0;
  }
  pair.valid=false;pair.geoValid=false;pair.armed=true;pair.prevZMs=0;pair.velocity=0;pair.jitter=0;
  if(!keepHits){pair.hits=0;pair.eventSeq=0;pair.lastHitMs=0;pair.lastHitVelocity=0;pair.lastHitZ=0;pair.lastMidi=0;}
}

void updateRates(){
  uint32_t now=millis();
  if(!rateMarkMs){rateMarkMs=now;for(int i=0;i<2;i++)s[i].validMark=s[i].valid;pair.pairMark=pair.pairUpdates;return;}
  uint32_t dt=now-rateMarkMs;if(dt<1000)return;float sec=dt/1000.0f;
  for(int i=0;i<2;i++){s[i].hz=(s[i].valid-s[i].validMark)/sec;s[i].validMark=s[i].valid;}
  pair.combinedHz=(pair.pairUpdates-pair.pairMark)/sec;pair.pairMark=pair.pairUpdates;rateMarkMs=now;
}

void updateLateral(){
  float a=fabsf(pair.delta);
  if(a<=0.20f){pair.lateral="CENTRO";pair.lateralConfidence=clampf((0.20f-a)/0.20f*100.0f,0,100);}
  else if(a>=0.45f){pair.lateral=pair.delta<0?"S1 / IZQUIERDA":"S2 / DERECHA";pair.lateralConfidence=clampf(30.0f+(a-0.45f)*80.0f,0,100);}
  else{pair.lateral="INDETERMINADO";pair.lateralConfidence=0;}
}

void updatePair(){
  uint32_t now=millis(),stale=staleMs();
  bool f0=s[0].ok&&s[0].inRange&&(now-s[0].lastGoodMs<=stale);
  bool f1=s[1].ok&&s[1].inRange&&(now-s[1].lastGoodMs<=stale);
  pair.valid=f0&&f1;if(!pair.valid)return;

  float maxPredict=clampf(slotMs()/1000.0f*1.5f,0.03f,0.12f);
  float age0=clampf((now-s[0].lastGoodMs)/1000.0f,0,maxPredict);
  float age1=clampf((now-s[1].lastGoodMs)/1000.0f,0,maxPredict);
  float p0=s[0].fastCm+clampf(s[0].velocityCmS*age0,-5,5);
  float p1=s[1].fastCm+clampf(s[1].velocityCmS*age1,-5,5);
  float zNew=0.5f*(p0+p1);

  pair.zStable=0.5f*(s[0].stableCm+s[1].stableCm);
  pair.delta=s[0].stableCm-s[1].stableCm;
  pair.jitter=0.5f*(s[0].jitterCm+s[1].jitterCm);

  if(pair.prevZMs&&now>pair.prevZMs){
    float dt=(now-pair.prevZMs)/1000.0f;
    if(dt>0.008f&&dt<0.30f){
      float rv=clampf((zNew-pair.prevZFast)/dt,-600,600);
      pair.velocity=PAIR_V_ALPHA*rv+(1.0f-PAIR_V_ALPHA)*pair.velocity;
    }
  }
  pair.zFast=zNew;pair.prevZFast=zNew;pair.prevZMs=now;pair.pairUpdates++;
  pair.coherence=clampf(100.0f-fabsf(pair.delta)*22.0f-pair.jitter*10.0f,0,100);

  pair.geoValid=false;pair.x=0;pair.zGeo=0;
  if(baselineCm>0.01f&&fabsf(pair.delta)<=baselineCm){
    float d1=s[0].stableCm,d2=s[1].stableCm;
    float x=(d1*d1-d2*d2)/(2.0f*baselineCm);
    float z2=d1*d1-(x+baselineCm*0.5f)*(x+baselineCm*0.5f);
    if(z2>=0){pair.x=x;pair.zGeo=sqrtf(z2);pair.geoValid=true;}
  }
  updateLateral();

  bool intervalOK=(now-pair.lastHitMs)>=MIN_HIT_INTERVAL_MS;
  if(pair.armed&&pair.zFast<=hitPlaneCm&&pair.velocity<=-hitSpeedCmS&&pair.coherence>=MIN_COHERENCE_PCT&&intervalOK){
    pair.hits++;pair.eventSeq++;pair.lastHitMs=now;pair.lastHitVelocity=-pair.velocity;pair.lastHitZ=pair.zFast;
    pair.lastMidi=midiFromVelocity(pair.lastHitVelocity);pair.armed=false;
  }
  if(!pair.armed&&intervalOK&&pair.zFast>=hitPlaneCm+rearmMarginCm)pair.armed=true;
}

void processPulse(uint8_t i,uint32_t pulse){
  SensorState &q=s[i];uint32_t now=millis();float raw=((float)pulse*0.0343f)/2.0f;
  q.pulseUs=pulse;q.rawCm=raw;
  if(raw<2.0f||raw>maxRangeCm){q.ok=false;q.inRange=false;q.rejected++;return;}
  q.ok=true;q.inRange=true;q.lastGoodMs=now;q.valid++;
  float med=pushMedian(q,raw);q.medianCm=med;
  if(!q.have){q.have=true;q.fastCm=raw;q.stableCm=med;q.prevFastCm=raw;q.prevFastMs=now;q.velocityCmS=0;q.jitterCm=0;return;}

  float guarded=raw,diff=raw-q.fastCm;
  if(diff>MAX_JUMP_CM)guarded=q.fastCm+MAX_JUMP_CM;
  if(diff<-MAX_JUMP_CM)guarded=q.fastCm-MAX_JUMP_CM;
  float fastNew=FAST_ALPHA*guarded+(1.0f-FAST_ALPHA)*q.fastCm;
  float a=stableAlpha();
  q.stableCm=a*med+(1.0f-a)*q.stableCm;
  q.jitterCm=0.90f*q.jitterCm+0.10f*fabsf(raw-med);
  if(now>q.prevFastMs){
    float dt=(now-q.prevFastMs)/1000.0f;
    if(dt>0.025f&&dt<0.35f){
      float rv=clampf((fastNew-q.prevFastCm)/dt,-600,600);
      q.velocityCmS=SENSOR_V_ALPHA*rv+(1.0f-SENSOR_V_ALPHA)*q.velocityCmS;
    }
  }
  q.fastCm=fastNew;q.prevFastCm=fastNew;q.prevFastMs=now;
}

void measureOne(uint8_t i){
  const SensorPins &p=PINS[i];
  digitalWrite(p.trig,LOW);delayMicroseconds(3);
  digitalWrite(p.trig,HIGH);delayMicroseconds(10);
  digitalWrite(p.trig,LOW);
  uint32_t pulse=pulseIn(p.echo,HIGH,echoTimeoutUs());
  if(pulse)processPulse(i,pulse);else{s[i].ok=false;s[i].inRange=false;s[i].pulseUs=0;s[i].timeouts++;}
  updatePair();updateRates();
}
void serviceSensors(){
  if(!acquisitionRunning)return;
  uint32_t now=millis();if((int32_t)(now-nextMeasureMs)<0)return;
  uint8_t i=nextSensor;nextSensor^=1;measureOne(i);nextMeasureMs=millis()+slotMs();
}
String motionLabel(){if(!pair.valid)return"SIN PAR";if(pair.velocity<=-18)return"ACERCANDO";if(pair.velocity>=18)return"ALEJANDO";return"ESTABLE";}

const char PAGE[] PROGMEM=R"HTML(
<!doctype html><html lang="es"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>SONIC BONGO v13</title>
<style>
:root{--bg:#060b10;--p:#0d151d;--p2:#101d27;--line:#243746;--text:#edf5fb;--muted:#8195a6;--s1:#58e8cd;--s2:#76a0ff;--hit:#ffc85e;--ok:#4ade88;--bad:#ff6868;--edit:#69b8ff}*{box-sizing:border-box}body{margin:0;background:linear-gradient(180deg,#060b10,#09131c);color:var(--text);font-family:system-ui,-apple-system,Segoe UI,sans-serif}header,.wrap{max-width:1280px;margin:auto;padding:13px}header{display:flex;justify-content:space-between;gap:12px;align-items:center}h1{font-size:21px;margin:0}.sub,.note{font-size:10px;color:var(--muted)}.pill{padding:5px 8px;border:1px solid var(--line);border-radius:999px;font-size:10px;font-weight:800}.on{color:var(--ok)}.off{color:var(--bad)}.pending{color:var(--hit)}.cards{display:grid;grid-template-columns:repeat(6,1fr);gap:7px}.card,.panel{background:rgba(13,21,29,.97);border:1px solid var(--line);border-radius:11px;padding:10px}.k{font-size:9px;letter-spacing:.07em;color:var(--muted)}.v{font-size:18px;font-weight:900;margin-top:3px}.s1{color:var(--s1)}.s2{color:var(--s2)}.hit{color:var(--hit)}.main{display:grid;grid-template-columns:1.35fr .65fr;gap:8px;margin-top:8px}.row{display:flex;justify-content:space-between;gap:12px;margin:5px 0;font-size:11px}.row b{font-size:12px}.section{font-size:11px;font-weight:900;letter-spacing:.06em}.controls{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:8px}.ctrl{padding:8px;border:1px solid #223441;border-radius:8px;background:#09121a}.ctrlHead{display:flex;justify-content:space-between;gap:8px;font-size:10px}.ctrl input{width:100%;margin-top:7px;accent-color:#6bbdff}.blocks{display:grid;grid-template-columns:1.15fr .85fr;gap:8px;margin-top:8px}.buttons{display:flex;gap:7px;flex-wrap:wrap;margin-top:9px}button{padding:9px 11px;border:1px solid #3a5265;border-radius:8px;background:#132330;color:#fff;font-weight:800;cursor:pointer}.primary{background:#123c2b;border-color:#2a7752}.danger{background:#35191b;border-color:#70383d}.blue{background:#152b40;border-color:#376487}.status{font-size:10px;color:var(--muted);min-height:16px;margin-top:7px}canvas{display:block;width:100%;height:auto;background:#050a0e;border:1px solid #1d2d38;border-radius:9px}.mathGrid{display:grid;grid-template-columns:1fr 1fr;gap:6px}.metric{padding:7px;border:1px solid #213442;border-radius:8px;background:#09121a}.metric span{display:block;font-size:9px;color:var(--muted)}.metric b{display:block;font-size:13px;margin-top:2px}.meter{height:7px;background:#071017;border:1px solid #1e303c;border-radius:8px;overflow:hidden}.meter i{display:block;height:100%;background:#55d68f;width:0}@media(max-width:980px){header{align-items:flex-start;flex-direction:column}.cards{grid-template-columns:repeat(2,1fr)}.main,.blocks{grid-template-columns:1fr}.controls{grid-template-columns:1fr}}
</style></head><body>
<header><div><h1>SONIC BONGO · GEOMETRY LAB v13</h1><div class="sub">Geometría, adquisición y HIT separados · configuración persistente verificada</div></div><div><span id="runPill" class="pill">...</span> <span id="cfgPill" class="pill">...</span></div></header>
<div class="wrap"><div class="cards">
<div class="card"><div class="k">S1 STABLE</div><div id="d1" class="v s1">--</div></div><div class="card"><div class="k">S2 STABLE</div><div id="d2" class="v s2">--</div></div><div class="card"><div class="k">Z FUSIONADA</div><div id="z" class="v">--</div></div><div class="card"><div class="k">VELOCIDAD</div><div id="vel" class="v">--</div></div><div class="card"><div class="k">HITS</div><div id="hits" class="v hit">0</div></div><div class="card"><div class="k">Hz COMBINADOS</div><div id="hz" class="v">--</div></div></div>
<div class="main"><section class="panel"><div class="row"><span class="section">GEOMETRÍA EN TIEMPO REAL</span><b id="geoStatus">--</b></div><canvas id="geo" width="820" height="560"></canvas><div class="note">Los círculos son las dos distancias estables. Los conos son el modelo angular configurable. El punto X/Z sólo aparece cuando ambas distancias admiten una intersección física.</div></section>
<section class="panel"><div class="section">MATEMÁTICA</div><div class="mathGrid" style="margin-top:8px">
<div class="metric"><span>Baseline b</span><b id="mBase">--</b></div><div class="metric"><span>Plano HIT h</span><b id="mHit">--</b></div>
<div class="metric"><span>Radio cono @ HIT</span><b id="mRadius">--</b></div><div class="metric"><span>Ancho solapado</span><b id="mWidth">--</b></div>
<div class="metric"><span>Área solapada @ HIT</span><b id="mArea">--</b></div><div class="metric"><span>Δ d1-d2</span><b id="mDelta">--</b></div>
<div class="metric"><span>X estimada</span><b id="mX">--</b></div><div class="metric"><span>Z geométrica</span><b id="mZgeo">--</b></div>
</div><div class="row"><span>Sesgo lateral</span><b id="lateral">--</b></div><div class="row"><span>Coherencia</span><b id="coh">--</b></div><div class="meter"><i id="cohBar"></i></div></section></div>

<section class="panel" style="margin-top:8px"><div class="section">GEOMETRÍA</div><div class="controls">
<div class="ctrl"><div class="ctrlHead"><span>Distancia sensores b</span><b><span id="baselineV">1.0</span> cm</b></div><input id="baseline" type="range" min="0.5" max="10" step="0.1" value="1"></div>
<div class="ctrl"><div class="ctrlHead"><span>Plano HIT h</span><b><span id="hitPlaneV">12.0</span> cm</b></div><input id="hitPlane" type="range" min="5" max="30" step="0.5" value="12"></div>
<div class="ctrl"><div class="ctrlHead"><span>Semiángulo θ</span><b><span id="angleV">7.5</span>°</b></div><input id="angle" type="range" min="3" max="20" step="0.5" value="7.5"></div>
</div></section>

<div class="blocks"><section class="panel"><div class="section">ADQUISICIÓN</div><div class="controls">
<div class="ctrl"><div class="ctrlHead"><span>Rango máximo</span><b><span id="rangeV">50</span> cm</b></div><input id="range" type="range" min="20" max="100" step="5" value="50"></div>
<div class="ctrl"><div class="ctrlHead"><span>Objetivo por sensor</span><b><span id="rateV">16</span> Hz</b></div><input id="rate" type="range" min="5" max="16" step="1" value="16"></div>
<div class="ctrl"><div class="ctrlHead"><span>Estabilidad visual</span><b><span id="smoothV">55</span>%</b></div><input id="smooth" type="range" min="0" max="100" step="5" value="55"></div>
</div><div class="row"><span>Hz reales S1 / S2</span><b id="sensorHz">--</b></div><div class="row"><span>Timeouts S1 / S2</span><b id="timeouts">--</b></div><div class="row"><span>Fuera de rango</span><b id="rejects">--</b></div><div class="row"><span>Jitter</span><b id="jitter">--</b></div><div class="row"><span>RAW S1 / S2</span><b id="raw">--</b></div><div class="row"><span>FAST S1 / S2</span><b id="fast">--</b></div></section>
<section class="panel"><div class="section">HIT</div><div class="controls" style="grid-template-columns:1fr 1fr">
<div class="ctrl"><div class="ctrlHead"><span>Velocidad mínima</span><b><span id="speedV">35</span> cm/s</b></div><input id="speed" type="range" min="10" max="180" step="5" value="35"></div>
<div class="ctrl"><div class="ctrlHead"><span>Retirada / rearm</span><b><span id="rearmV">3.0</span> cm</b></div><input id="rearm" type="range" min="1" max="10" step="0.5" value="3"></div>
</div><div class="row"><span>Estado</span><b id="motion">--</b></div><div class="row"><span>ARMADO</span><b id="armed">--</b></div><div class="row"><span>Último HIT</span><b id="lastHit">--</b></div><div class="buttons"><button class="danger" onclick="setRun(false)">DETENER</button><button class="primary" onclick="setRun(true)">INICIAR</button><button id="soundBtn" class="blue" onclick="toggleSound()">SONIDO OFF</button><button onclick="testSound()">PROBAR HIT</button></div></section></div>

<section class="panel" style="margin-top:8px"><div class="row"><span class="section">CONFIGURACIÓN</span><span class="note">La gráfica usa el preview local; GUARDAR Y APLICAR escribe NVS y verifica read-back.</span></div><div class="buttons"><button class="primary" onclick="saveConfig()">GUARDAR Y APLICAR</button><button onclick="loadCurrent()">CARGAR ACTUALES</button><button onclick="resetHits()">RESET HITS</button></div><div id="status" class="status"></div></section>
<section class="panel" style="margin-top:8px"><div class="row"><span class="section">TIMELINE</span><span class="note">S1 · S2 · Z estable · HIT</span></div><canvas id="timeline" width="1200" height="260"></canvas></section>
</div>
<script>
const $=id=>document.getElementById(id),G=$('geo').getContext('2d'),T=$('timeline').getContext('2d');
const ids=['baseline','hitPlane','angle','range','rate','smooth','speed','rearm'];
const labels={baseline:'baselineV',hitPlane:'hitPlaneV',angle:'angleV',range:'rangeV',rate:'rateV',smooth:'smoothV',speed:'speedV',rearm:'rearmV'};
let first=true,dirty=false,lastData=null,lastSeq=null,soundOn=false,audioCtx=null;const hist=[];
ids.forEach(id=>$(id).addEventListener('input',()=>{$(labels[id]).textContent=$(id).value;dirty=true;setCfgState();if(lastData)drawAll(lastData);}));
function setCfgState(){ $('cfgPill').textContent=dirty?'CAMBIOS PENDIENTES':'CONFIG GUARDADA';$('cfgPill').className='pill '+(dirty?'pending':'on'); }
function localCfg(){return{baseline:+$('baseline').value,hitPlane:+$('hitPlane').value,angle:+$('angle').value,maxRange:+$('range').value,rate:+$('rate').value,smooth:+$('smooth').value,speed:+$('speed').value,rearm:+$('rearm').value};}
function syncCfg(c){const m={baseline:c.baseline,hitPlane:c.hitPlane,angle:c.coneHalfAngle,range:c.maxRange,rate:c.targetHz,smooth:c.smoothing,speed:c.hitSpeed,rearm:c.rearm};Object.entries(m).forEach(([id,v])=>{$(id).value=v;$(labels[id]).textContent=v});dirty=false;setCfgState();}
function line(c,x1,y1,x2,y2,col='#2b3f4d',w=1){c.beginPath();c.moveTo(x1,y1);c.lineTo(x2,y2);c.strokeStyle=col;c.lineWidth=w;c.stroke();}
function text(c,s,x,y,col='#8094a5',sz=10,a='left'){c.fillStyle=col;c.font=sz+'px system-ui';c.textAlign=a;c.fillText(s,x,y);}
function circle(c,x,y,r,col,w=1,alpha=1){c.save();c.globalAlpha=alpha;c.beginPath();c.arc(x,y,r,0,Math.PI*2);c.strokeStyle=col;c.lineWidth=w;c.stroke();c.restore();}
function solveGeo(d1,d2,b){if(!(d1>0&&d2>0&&b>0))return{ok:false};const x=(d1*d1-d2*d2)/(2*b),z2=d1*d1-(x+b/2)*(x+b/2);return z2>=0?{ok:true,x,z:Math.sqrt(z2)}:{ok:false};}
function overlapMath(c){const h=c.hitPlane,r=h*Math.tan(c.angle*Math.PI/180),b=c.baseline,w=Math.max(0,2*r-b);let area=0;if(r>0&&b<2*r){const q=Math.max(-1,Math.min(1,b/(2*r)));area=2*r*r*Math.acos(q)-.5*b*Math.sqrt(Math.max(0,4*r*r-b*b));}return{r,w,area};}
function drawGeometry(j){const c=localCfg(),W=820,H=560,baseY=510,cx=W/2,max=c.maxRange,sc=(H-90)/max;G.clearRect(0,0,W,H);for(let q=5;q<=max;q+=5){const y=baseY-q*sc;line(G,48,y,W-24,y,q%10===0?'#19303d':'#10212b');if(q%10===0)text(G,q+' cm',8,y+3,'#60798a',10)}const sx1=cx-c.baseline*.5*sc,sx2=cx+c.baseline*.5*sc,hy=baseY-c.hitPlane*sc;line(G,46,hy,W-24,hy,'#ffc85e',2);text(G,'PLANO HIT '+c.hitPlane.toFixed(1)+' cm',W-28,hy-7,'#ffc85e',11,'right');const len=max*sc,spread=Math.tan(c.angle*Math.PI/180)*len;[[sx1,'#58e8cd'],[sx2,'#76a0ff']].forEach(v=>{G.save();G.globalAlpha=.10;G.beginPath();G.moveTo(v[0],baseY);G.lineTo(v[0]-spread,baseY-len);G.lineTo(v[0]+spread,baseY-len);G.closePath();G.fillStyle=v[1];G.fill();G.restore();line(G,v[0],baseY,v[0]-spread,baseY-len,v[1]);line(G,v[0],baseY,v[0]+spread,baseY-len,v[1])});const om=overlapMath(c),hr=om.r*sc;if(om.w>0){const y=hy;G.save();G.globalAlpha=.18;G.fillStyle='#ffc85e';G.fillRect(cx-om.w*.5*sc,y-5,om.w*sc,10);G.restore();}G.fillStyle='#58e8cd';G.fillRect(sx1-9,baseY-7,18,14);G.fillStyle='#76a0ff';G.fillRect(sx2-9,baseY-7,18,14);text(G,'S1',sx1,baseY+27,'#58e8cd',11,'center');text(G,'S2',sx2,baseY+27,'#76a0ff',11,'center');text(G,c.baseline.toFixed(1)+' cm',cx,baseY+27,'#d8e5ed',10,'center');if(j&&j.s1.ok)circle(G,sx1,baseY,j.s1.stable*sc,'#58e8cd',1.5,.55);if(j&&j.s2.ok)circle(G,sx2,baseY,j.s2.stable*sc,'#76a0ff',1.5,.55);let sol={ok:false};if(j&&j.s1.ok&&j.s2.ok)sol=solveGeo(j.s1.stable,j.s2.stable,c.baseline);if(sol.ok&&sol.z<=max){const px=cx+sol.x*sc,py=baseY-sol.z*sc;line(G,sx1,baseY,px,py,'#58e8cd',1);line(G,sx2,baseY,px,py,'#76a0ff',1);G.beginPath();G.arc(px,py,8,0,Math.PI*2);G.fillStyle='#fff';G.fill();text(G,'X '+sol.x.toFixed(2)+' / Z '+sol.z.toFixed(2),px+12,py-10,'#fff',11);$('geoStatus').textContent='INTERSECCIÓN VÁLIDA';}else $('geoStatus').textContent='SIN INTERSECCIÓN';$('mBase').textContent=c.baseline.toFixed(1)+' cm';$('mHit').textContent=c.hitPlane.toFixed(1)+' cm';$('mRadius').textContent=om.r.toFixed(2)+' cm';$('mWidth').textContent=om.w.toFixed(2)+' cm';$('mArea').textContent=om.area.toFixed(2)+' cm²';$('mDelta').textContent=j&&j.s1.ok&&j.s2.ok?(j.s1.stable-j.s2.stable).toFixed(2)+' cm':'--';$('mX').textContent=sol.ok?sol.x.toFixed(2)+' cm':'--';$('mZgeo').textContent=sol.ok?sol.z.toFixed(2)+' cm':'--';}
function drawTimeline(j){const c=localCfg(),W=1200,H=260,p=34,max=c.maxRange,y=v=>H-p-v/max*(H-2*p);T.clearRect(0,0,W,H);for(let q=10;q<=max;q+=10){line(T,p,y(q),W-10,y(q),'#14242e');text(T,q,8,y(q)+3)}line(T,p,y(c.hitPlane),W-10,y(c.hitPlane),'#ffc85e',1.5);function ser(k,col,w){T.beginPath();let st=false;hist.forEach((r,i)=>{const v=r[k];if(!Number.isFinite(v)||v>max){st=false;return}const x=p+i/Math.max(1,hist.length-1)*(W-p-15);if(!st){T.moveTo(x,y(v));st=true}else T.lineTo(x,y(v));});T.strokeStyle=col;T.lineWidth=w;T.stroke();}ser('s1','#58e8cd',1.4);ser('s2','#76a0ff',1.4);ser('z','#fff',2.2);hist.forEach((r,i)=>{if(r.hit){const x=p+i/Math.max(1,hist.length-1)*(W-p-15);line(T,x,12,x,H-p,'#ffc85e',2)}});}
function drawAll(j){drawGeometry(j);drawTimeline(j);}
async function ensureAudio(){if(!audioCtx)audioCtx=new(window.AudioContext||window.webkitAudioContext)();await audioCtx.resume();}
function playHit(midi=80){if(!soundOn||!audioCtx)return;const n=audioCtx.currentTime,o=audioCtx.createOscillator(),g=audioCtx.createGain();o.type='triangle';o.frequency.value=500+Math.min(127,midi)*3.2;g.gain.setValueAtTime(.0001,n);g.gain.exponentialRampToValueAtTime(.16,n+.004);g.gain.exponentialRampToValueAtTime(.0001,n+.07);o.connect(g);g.connect(audioCtx.destination);o.start(n);o.stop(n+.08);}
async function toggleSound(){await ensureAudio();soundOn=!soundOn;$('soundBtn').textContent=soundOn?'SONIDO ON':'SONIDO OFF';if(soundOn)playHit(60);}
async function testSound(){await ensureAudio();soundOn=true;$('soundBtn').textContent='SONIDO ON';playHit(95);}
async function setRun(on){const r=await fetch('/api/run?on='+(on?1:0),{cache:'no-store'});if(!r.ok){$('status').textContent='Error cambiando adquisición';return}await refresh();}
async function saveConfig(){const c=localCfg(),q=new URLSearchParams({base:c.baseline,hit:c.hitPlane,angle:c.angle,range:c.maxRange,rate:c.rate,smooth:c.smooth,speed:c.speed,rearm:c.rearm});$('status').textContent='Guardando y verificando...';const r=await fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:q.toString(),cache:'no-store'});const txt=await r.text();if(!r.ok){$('status').textContent='ERROR: '+txt;return}let out;try{out=JSON.parse(txt)}catch(e){$('status').textContent='ERROR: respuesta inválida';return}if(!out.ok){$('status').textContent='ERROR DE READ-BACK';return}syncCfg(out.cfg);$('status').textContent='GUARDADO ✓ · NVS verificado · baseline '+out.cfg.baseline.toFixed(1)+' cm · HIT '+out.cfg.hitPlane.toFixed(1)+' cm · '+out.cfg.targetHz.toFixed(0)+' Hz/sensor';await refresh();}
async function loadCurrent(){const r=await fetch('/api/status',{cache:'no-store'}),j=await r.json();syncCfg(j.cfg);lastData=j;drawAll(j);$('status').textContent='Valores actuales cargados';}
async function resetHits(){await fetch('/api/reset',{cache:'no-store'});lastSeq=null;await refresh();}
async function refresh(){try{const r=await fetch('/api/status',{cache:'no-store'}),j=await r.json();lastData=j;if(first){syncCfg(j.cfg);first=false}$('runPill').textContent=j.running?'DETECCIÓN ACTIVA':'DETECCIÓN DETENIDA';$('runPill').className='pill '+(j.running?'on':'off');$('d1').textContent=j.s1.ok?j.s1.stable.toFixed(2)+' cm':'--';$('d2').textContent=j.s2.ok?j.s2.stable.toFixed(2)+' cm':'--';$('z').textContent=j.pair.valid?j.pair.zStable.toFixed(2)+' cm':'--';$('vel').textContent=j.pair.valid?j.pair.velocity.toFixed(0)+' cm/s':'--';$('hits').textContent=j.pair.hits;$('hz').textContent=j.pair.combinedHz.toFixed(1);$('lateral').textContent=j.pair.lateral+' '+j.pair.lateralConfidence.toFixed(0)+'%';$('coh').textContent=j.pair.coherence.toFixed(0)+'%';$('cohBar').style.width=j.pair.coherence.toFixed(0)+'%';$('sensorHz').textContent=j.s1.hz.toFixed(1)+' / '+j.s2.hz.toFixed(1);$('timeouts').textContent=j.s1.timeouts+' / '+j.s2.timeouts;$('rejects').textContent=j.s1.rejected+' / '+j.s2.rejected;$('jitter').textContent=j.pair.jitter.toFixed(2)+' cm';$('raw').textContent=j.s1.raw.toFixed(1)+' / '+j.s2.raw.toFixed(1)+' cm';$('fast').textContent=j.s1.fast.toFixed(1)+' / '+j.s2.fast.toFixed(1)+' cm';$('motion').textContent=j.pair.motion;$('armed').textContent=j.pair.armed?'SÍ':'NO';$('lastHit').textContent=j.pair.lastHitMs?(j.pair.lastHitZ.toFixed(1)+' cm · '+j.pair.lastHitVelocity.toFixed(0)+' cm/s · MIDI '+j.pair.lastMidi):'--';let hit=false;if(lastSeq===null)lastSeq=j.event.seq;else if(j.event.seq!==lastSeq){hit=true;lastSeq=j.event.seq;playHit(j.event.midi||80)}hist.push({s1:j.s1.ok?j.s1.stable:NaN,s2:j.s2.ok?j.s2.stable:NaN,z:j.pair.valid?j.pair.zStable:NaN,hit});if(hist.length>200)hist.shift();drawAll(j);}catch(e){$('status').textContent='Error de comunicación con ESP';}}
setInterval(refresh,100);refresh();
</script></body></html>
)HTML";

String cfgJson(){
  String j="{";
  j+="\"baseline\":"+String(baselineCm,2)+",";
  j+="\"maxRange\":"+String(maxRangeCm,1)+",";
  j+="\"hitPlane\":"+String(hitPlaneCm,1)+",";
  j+="\"coneHalfAngle\":"+String(coneHalfAngleDeg,1)+",";
  j+="\"targetHz\":"+String(targetHzPerSensor,1)+",";
  j+="\"smoothing\":"+String(smoothingPct,0)+",";
  j+="\"hitSpeed\":"+String(hitSpeedCmS,1)+",";
  j+="\"rearm\":"+String(rearmMarginCm,1)+",";
  j+="\"slotMs\":"+String(slotMs())+",";
  j+="\"stableAlpha\":"+String(stableAlpha(),3)+",";
  j+="\"echoTimeoutUs\":"+String(echoTimeoutUs());
  j+="}";
  return j;
}

void sendStatus(){
  String j="{";
  j+="\"fw\":\""+String(FW_VERSION)+"\",\"running\":"+String(acquisitionRunning?"true":"false")+",";
  for(int i=0;i<2;i++){
    SensorState&q=s[i];j+="\"s"+String(i+1)+"\":{";
    j+="\"ok\":"+String(q.ok?"true":"false")+",\"raw\":"+String(q.rawCm,2)+",\"median\":"+String(q.medianCm,2)+",\"fast\":"+String(q.fastCm,2)+",\"stable\":"+String(q.stableCm,2)+",\"velocity\":"+String(q.velocityCmS,2)+",\"jitter\":"+String(q.jitterCm,2)+",\"timeouts\":"+String(q.timeouts)+",\"rejected\":"+String(q.rejected)+",\"hz\":"+String(q.hz,2)+"}";if(i==0)j+=",";
  }
  j+=",\"pair\":{";
  j+="\"valid\":"+String(pair.valid?"true":"false")+",\"geoValid\":"+String(pair.geoValid?"true":"false")+",\"zFast\":"+String(pair.zFast,2)+",\"zStable\":"+String(pair.zStable,2)+",\"x\":"+String(pair.x,2)+",\"zGeo\":"+String(pair.zGeo,2)+",\"velocity\":"+String(pair.velocity,2)+",\"jitter\":"+String(pair.jitter,2)+",\"coherence\":"+String(pair.coherence,1)+",\"lateral\":\""+pair.lateral+"\",\"lateralConfidence\":"+String(pair.lateralConfidence,1)+",\"armed\":"+String(pair.armed?"true":"false")+",\"hits\":"+String(pair.hits)+",\"lastHitMs\":"+String(pair.lastHitMs)+",\"lastHitVelocity\":"+String(pair.lastHitVelocity,1)+",\"lastHitZ\":"+String(pair.lastHitZ,2)+",\"lastMidi\":"+String(pair.lastMidi)+",\"combinedHz\":"+String(pair.combinedHz,2)+",\"motion\":\""+motionLabel()+"\"},";
  j+="\"event\":{\"seq\":"+String(pair.eventSeq)+",\"pad\":1,\"midi\":"+String(pair.lastMidi)+",\"velocity\":"+String(pair.lastHitVelocity,1)+",\"z\":"+String(pair.lastHitZ,2)+"},";
  j+="\"cfg\":"+cfgJson();
  j+="}";
  server.send(200,"application/json",j);
}

void handleConfig(){
  bool wasRunning=acquisitionRunning;
  acquisitionRunning=false;

  if(server.hasArg("base"))baselineCm=clampf(server.arg("base").toFloat(),0.5f,10.0f);
  if(server.hasArg("range"))maxRangeCm=clampf(server.arg("range").toFloat(),20.0f,100.0f);
  if(server.hasArg("hit"))hitPlaneCm=clampf(server.arg("hit").toFloat(),5.0f,30.0f);
  if(server.hasArg("angle"))coneHalfAngleDeg=clampf(server.arg("angle").toFloat(),3.0f,20.0f);
  if(server.hasArg("rate"))targetHzPerSensor=clampf(server.arg("rate").toFloat(),5.0f,16.0f);
  if(server.hasArg("smooth"))smoothingPct=clampf(server.arg("smooth").toFloat(),0.0f,100.0f);
  if(server.hasArg("speed"))hitSpeedCmS=clampf(server.arg("speed").toFloat(),10.0f,180.0f);
  if(server.hasArg("rearm"))rearmMarginCm=clampf(server.arg("rearm").toFloat(),1.0f,10.0f);

  String detail;
  bool ok=saveAndVerifyConfig(detail);
  resetTracking(true);
  nextSensor=0;
  nextMeasureMs=millis()+80;
  acquisitionRunning=wasRunning;

  String j="{\"ok\":"+String(ok?"true":"false")+",\"detail\":\""+detail+"\",\"cfg\":"+cfgJson()+"}";
  server.send(ok?200:500,"application/json",j);
}

void handleRun(){
  if(server.hasArg("on"))acquisitionRunning=server.arg("on")!="0";
  if(acquisitionRunning){resetTracking(true);nextSensor=0;nextMeasureMs=millis()+80;}
  server.send(200,"text/plain","OK");
}

void setup(){
  Serial.begin(115200);delay(120);
  loadConfig();
  for(int i=0;i<2;i++){pinMode(PINS[i].echo,INPUT);pinMode(PINS[i].trig,OUTPUT);digitalWrite(PINS[i].trig,LOW);}
  WiFi.mode(WIFI_AP);WiFi.softAP(AP_SSID);
  server.on("/",[](){server.send_P(200,"text/html",PAGE);});
  server.on("/api/status",HTTP_GET,sendStatus);
  server.on("/api/config",HTTP_POST,handleConfig);
  server.on("/api/run",HTTP_GET,handleRun);
  server.on("/api/reset",HTTP_GET,[](){resetTracking(false);server.send(200,"text/plain","OK");});
  server.begin();nextMeasureMs=millis()+300;
}

void loop(){server.handleClient();serviceSensors();delay(1);}
