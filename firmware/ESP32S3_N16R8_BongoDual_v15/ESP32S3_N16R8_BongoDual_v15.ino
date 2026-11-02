/*
  SONIC BONGO / PRISMA
  ESP32-S3 N16R8 - Dual Sensor Bongo v15 ANY-SENSOR POINT

  CONFIRMED WIRING (unchanged):
    S1: GPIO13 -> TRIG direct
        HC-SR04 ECHO -> TXS B1 -> A1 -> GPIO14
    S2: GPIO15 -> TRIG direct
        HC-SR04 ECHO -> TXS B2 -> A2 -> GPIO16

  v15 focus:
    - Either fresh sensor can drive the white point.
    - When both are fresh, time-align and fuse both.
    - White point Y and HIT use EXACTLY the same signal.
    - HIT occurs when armed white point enters/belongs below the HIT plane.
      Velocity no longer blocks a hit; it only determines intensity/MIDI.
    - Preserve v13/v14 NVS calibration and POST/read-back workflow.
    - Show S1/S2 freshness/age and point source to diagnose asymmetry.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <math.h>

WebServer server(80);
Preferences prefs;

static const char* FW_VERSION = "15.0-any-sensor-point";
static const char* AP_SSID = "PRISMA-S3-DIAG";
static const char* NVS_NS = "bongo13"; // preserve existing calibration

struct SensorPins { uint8_t trig; uint8_t echo; };
const SensorPins PINS[2] = {{13,14},{15,16}};

// Persisted calibration
float baselineCm = 1.0f;
float maxRangeCm = 50.0f;
float hitPlaneCm = 12.0f;
float coneHalfAngleDeg = 7.5f;
float targetHzPerSensor = 16.0f;
float smoothingPct = 55.0f;
float hitSpeedCmS = 35.0f; // v15: reference for MIDI/intensity only
float rearmMarginCm = 3.0f;

// Engine constants
static const float FAST_ALPHA = 0.80f;
static const float SENSOR_V_ALPHA = 0.38f;
static const float POINT_V_ALPHA = 0.52f;
static const float MAX_JUMP_CM = 18.0f;
static const uint32_t MIN_HIT_INTERVAL_MS = 85;

bool acquisitionRunning = true;

struct SensorState {
  bool have=false, ok=false;
  uint32_t lastGoodMs=0, lastAttemptMs=0, pulseUs=0;
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
  bool valid=false, both=false, geoValid=false, armed=true, havePoint=false;
  uint8_t sourceMask=0; // 1=S1, 2=S2, 3=S1+S2
  float yPoint=0, yCandidate=0, yStable=0, fastCandidate=0;
  float prevPoint=0, velocity=0;
  uint32_t prevPointMs=0;
  float delta=0, x=0, zGeo=0, coherence=0, jitter=0;
  String lateral="INDETERMINADO";
  float lateralConfidence=0;
  uint32_t hits=0, lastHitMs=0, eventSeq=0, pointSeq=0;
  float lastHitVelocity=0, lastHitZ=0;
  int lastMidi=0;
  uint32_t pointUpdates=0, pointMark=0;
  float pointHz=0;
};
PairState pair;

uint8_t nextSensor=0;
uint32_t nextMeasureMs=0, rateMarkMs=0;

float clampf(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);} 
uint32_t clampu(uint32_t v,uint32_t lo,uint32_t hi){return v<lo?lo:(v>hi?hi:v);} 

float sensorStableAlpha(){return clampf(0.52f-smoothingPct*0.0040f,0.12f,0.52f);} // 55% -> .30
float pointAlpha(){return clampf(0.90f-smoothingPct*0.0045f,0.45f,0.90f);}       // 55% -> .65
uint32_t slotMs(){float hz=clampf(targetHzPerSensor,5.0f,16.0f);return (uint32_t)lroundf(500.0f/hz);} // alternating
uint32_t echoTimeoutUs(){float u=maxRangeCm*58.3f+1100.0f;return (uint32_t)clampf(u,2500.0f,9000.0f);}
uint32_t freshWindowMs(){return clampu(slotMs()*4,130,500);} // tolerate one missed ping, not long ghosts

float med3(float a,float b,float c){if(a>b){float t=a;a=b;b=t;}if(b>c){float t=b;b=c;c=t;}if(a>b){float t=a;a=b;b=t;}return b;}
float pushMedian(SensorState &q,float raw){q.hist[q.histHead]=raw;q.histHead=(q.histHead+1)%3;if(q.histCount<3)q.histCount++;if(q.histCount==1)return q.hist[0];if(q.histCount==2)return .5f*(q.hist[0]+q.hist[1]);return med3(q.hist[0],q.hist[1],q.hist[2]);}

int midiFromVelocity(float approachSpeed){
  float ref=clampf(hitSpeedCmS,10.0f,180.0f);
  float upper=clampf(ref*4.0f,80.0f,500.0f);
  float x=clampf(approachSpeed/upper,0.0f,1.0f);
  return 1+(int)lroundf(x*126.0f);
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
  prefs.putFloat("base",baselineCm);prefs.putFloat("range",maxRangeCm);prefs.putFloat("hit",hitPlaneCm);prefs.putFloat("angle",coneHalfAngleDeg);
  prefs.putFloat("rate",targetHzPerSensor);prefs.putFloat("smooth",smoothingPct);prefs.putFloat("speed",hitSpeedCmS);prefs.putFloat("rearm",rearmMarginCm);
  prefs.end();
  prefs.begin(NVS_NS,true);
  float a=prefs.getFloat("base",-999),b=prefs.getFloat("range",-999),c=prefs.getFloat("hit",-999),d=prefs.getFloat("angle",-999);
  float e=prefs.getFloat("rate",-999),f=prefs.getFloat("smooth",-999),g=prefs.getFloat("speed",-999),h=prefs.getFloat("rearm",-999);
  prefs.end();
  bool ok=fabsf(a-baselineCm)<.01f&&fabsf(b-maxRangeCm)<.01f&&fabsf(c-hitPlaneCm)<.01f&&fabsf(d-coneHalfAngleDeg)<.01f&&fabsf(e-targetHzPerSensor)<.01f&&fabsf(f-smoothingPct)<.01f&&fabsf(g-hitSpeedCmS)<.01f&&fabsf(h-rearmMarginCm)<.01f;
  detail=ok?"verified":"readback_mismatch";return ok;
}

void resetTracking(bool keepHits=true){
  for(int i=0;i<2;i++){s[i].have=false;s[i].ok=false;s[i].histCount=0;s[i].histHead=0;s[i].velocityCmS=0;s[i].jitterCm=0;s[i].lastGoodMs=0;}
  pair.valid=false;pair.both=false;pair.geoValid=false;pair.armed=true;pair.havePoint=false;pair.sourceMask=0;pair.prevPointMs=0;pair.velocity=0;pair.yPoint=0;pair.yCandidate=0;pair.pointSeq=0;
  if(!keepHits){pair.hits=0;pair.eventSeq=0;pair.lastHitMs=0;pair.lastHitVelocity=0;pair.lastHitZ=0;pair.lastMidi=0;}
}

bool sensorFresh(uint8_t i,uint32_t now){return s[i].have&&s[i].lastGoodMs&&((now-s[i].lastGoodMs)<=freshWindowMs());}
uint32_t sensorAge(uint8_t i,uint32_t now){return s[i].lastGoodMs?(now-s[i].lastGoodMs):999999;}

void updateRates(){
  uint32_t now=millis();
  if(!rateMarkMs){rateMarkMs=now;for(int i=0;i<2;i++)s[i].validMark=s[i].valid;pair.pointMark=pair.pointUpdates;return;}
  uint32_t dt=now-rateMarkMs;if(dt<1000)return;float sec=dt/1000.0f;
  for(int i=0;i<2;i++){s[i].hz=(s[i].valid-s[i].validMark)/sec;s[i].validMark=s[i].valid;}
  pair.pointHz=(pair.pointUpdates-pair.pointMark)/sec;pair.pointMark=pair.pointUpdates;rateMarkMs=now;
}

void updateLateral(){
  if(!pair.both){pair.lateral=pair.sourceMask==1?"S1 SOLO":pair.sourceMask==2?"S2 SOLO":"--";pair.lateralConfidence=0;return;}
  float a=fabsf(pair.delta);
  if(a<=.20f){pair.lateral="CENTRO";pair.lateralConfidence=clampf((.20f-a)/.20f*100.0f,0,100);}
  else if(a>=.45f){pair.lateral=pair.delta<0?"S1 / IZQUIERDA":"S2 / DERECHA";pair.lateralConfidence=clampf(30.0f+(a-.45f)*80.0f,0,100);}
  else{pair.lateral="INDETERMINADO";pair.lateralConfidence=0;}
}

void updatePoint(){
  uint32_t now=millis();
  bool f0=sensorFresh(0,now),f1=sensorFresh(1,now);
  pair.valid=f0||f1;pair.both=f0&&f1;pair.sourceMask=(f0?1:0)|(f1?2:0);
  if(!pair.valid)return;

  float maxPredict=clampf(slotMs()/1000.0f*1.5f,.03f,.12f);
  float age0=f0?clampf(sensorAge(0,now)/1000.0f,0,maxPredict):0;
  float age1=f1?clampf(sensorAge(1,now)/1000.0f,0,maxPredict):0;

  float fast0=f0?s[0].fastCm+clampf(s[0].velocityCmS*age0,-5,5):0;
  float fast1=f1?s[1].fastCm+clampf(s[1].velocityCmS*age1,-5,5):0;
  float st0=f0?s[0].stableCm+clampf(s[0].velocityCmS*age0,-2.5f,2.5f):0;
  float st1=f1?s[1].stableCm+clampf(s[1].velocityCmS*age1,-2.5f,2.5f):0;

  if(pair.both){pair.fastCandidate=.5f*(fast0+fast1);pair.yStable=.5f*(st0+st1);}
  else if(f0){pair.fastCandidate=fast0;pair.yStable=st0;}
  else{pair.fastCandidate=fast1;pair.yStable=st1;}

  pair.yCandidate=pair.fastCandidate; // same candidate drives point and hit
  float pa=pointAlpha();
  if(!pair.havePoint){pair.yPoint=pair.yCandidate;pair.prevPoint=pair.yPoint;pair.prevPointMs=now;pair.havePoint=true;}
  else{
    float newPoint=pa*pair.yCandidate+(1.0f-pa)*pair.yPoint;
    if(pair.prevPointMs&&now>pair.prevPointMs){float dt=(now-pair.prevPointMs)/1000.0f;if(dt>.008f&&dt<.30f){float rv=clampf((newPoint-pair.prevPoint)/dt,-600,600);pair.velocity=POINT_V_ALPHA*rv+(1.0f-POINT_V_ALPHA)*pair.velocity;}}
    pair.prevPoint=pair.yPoint=newPoint;pair.prevPointMs=now;
  }

  if(pair.both){
    pair.delta=s[0].stableCm-s[1].stableCm;
    pair.jitter=.5f*(s[0].jitterCm+s[1].jitterCm);
    pair.coherence=clampf(100.0f-fabsf(pair.delta)*22.0f-pair.jitter*10.0f,0,100);
  }else{
    pair.delta=0;pair.jitter=f0?s[0].jitterCm:s[1].jitterCm;pair.coherence=50.0f;
  }

  // Diagnostic geometry only when both are fresh.
  pair.geoValid=false;pair.x=0;pair.zGeo=0;
  if(pair.both&&baselineCm>.01f&&fabsf(pair.delta)<=baselineCm){
    float d1=s[0].stableCm,d2=s[1].stableCm;float x=(d1*d1-d2*d2)/(2.0f*baselineCm);float z2=d1*d1-(x+baselineCm*.5f)*(x+baselineCm*.5f);
    if(z2>=0){pair.x=x;pair.zGeo=sqrtf(z2);pair.geoValid=true;}
  }
  updateLateral();

  pair.pointUpdates++;pair.pointSeq++;
  bool intervalOK=(now-pair.lastHitMs)>=MIN_HIT_INTERVAL_MS;
  bool below=pair.yPoint<=hitPlaneCm;
  // Visual parity rule: if the white point is below the yellow line and armed, HIT.
  if(pair.armed&&pair.pointSeq>2&&below&&intervalOK){
    pair.hits++;pair.eventSeq++;pair.lastHitMs=now;pair.lastHitVelocity=fmaxf(0.0f,-pair.velocity);pair.lastHitZ=pair.yPoint;pair.lastMidi=midiFromVelocity(pair.lastHitVelocity);pair.armed=false;
  }
  if(!pair.armed&&intervalOK&&pair.yPoint>=hitPlaneCm+rearmMarginCm)pair.armed=true;
}

void processPulse(uint8_t i,uint32_t pulse){
  SensorState&q=s[i];uint32_t now=millis();float raw=((float)pulse*.0343f)/2.0f;q.lastAttemptMs=now;q.pulseUs=pulse;q.rawCm=raw;
  if(raw<2.0f||raw>maxRangeCm){q.ok=false;q.rejected++;return;}
  q.ok=true;q.lastGoodMs=now;q.valid++;float med=pushMedian(q,raw);q.medianCm=med;
  if(!q.have){q.have=true;q.fastCm=raw;q.stableCm=med;q.prevFastCm=raw;q.prevFastMs=now;q.velocityCmS=0;q.jitterCm=0;return;}
  float guarded=raw,diff=raw-q.fastCm;if(diff>MAX_JUMP_CM)guarded=q.fastCm+MAX_JUMP_CM;if(diff<-MAX_JUMP_CM)guarded=q.fastCm-MAX_JUMP_CM;
  float fastNew=FAST_ALPHA*guarded+(1.0f-FAST_ALPHA)*q.fastCm;float sa=sensorStableAlpha();q.stableCm=sa*med+(1.0f-sa)*q.stableCm;q.jitterCm=.90f*q.jitterCm+.10f*fabsf(raw-med);
  if(now>q.prevFastMs){float dt=(now-q.prevFastMs)/1000.0f;if(dt>.025f&&dt<.35f){float rv=clampf((fastNew-q.prevFastCm)/dt,-600,600);q.velocityCmS=SENSOR_V_ALPHA*rv+(1.0f-SENSOR_V_ALPHA)*q.velocityCmS;}}
  q.fastCm=fastNew;q.prevFastCm=fastNew;q.prevFastMs=now;
}

void measureOne(uint8_t i){
  const SensorPins&p=PINS[i];digitalWrite(p.trig,LOW);delayMicroseconds(3);digitalWrite(p.trig,HIGH);delayMicroseconds(10);digitalWrite(p.trig,LOW);
  uint32_t pulse=pulseIn(p.echo,HIGH,echoTimeoutUs());
  if(pulse)processPulse(i,pulse);else{s[i].ok=false;s[i].lastAttemptMs=millis();s[i].pulseUs=0;s[i].timeouts++;}
  updatePoint();updateRates();
}

void serviceSensors(){
  if(!acquisitionRunning)return;uint32_t now=millis();if((int32_t)(now-nextMeasureMs)<0)return;
  uint32_t scheduled=nextMeasureMs;uint8_t i=nextSensor;nextSensor^=1;measureOne(i);
  uint32_t step=slotMs();nextMeasureMs=scheduled+step;if((int32_t)(millis()-nextMeasureMs)>=0)nextMeasureMs=millis()+2;
}
String motionLabel(){if(!pair.valid)return"SIN SEÑAL";if(pair.velocity<=-18)return"ACERCANDO";if(pair.velocity>=18)return"ALEJANDO";return"ESTABLE";}
String sourceLabel(){return pair.sourceMask==3?"S1+S2":pair.sourceMask==1?"S1":pair.sourceMask==2?"S2":"--";}

const char PAGE[] PROGMEM=R"HTML(
<!doctype html><html lang="es"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>SONIC BONGO v15</title>
<style>
:root{--bg:#060b10;--p:#0d151d;--line:#243746;--text:#edf5fb;--muted:#8195a6;--s1:#58e8cd;--s2:#76a0ff;--hit:#ffc85e;--ok:#4ade88;--bad:#ff6868}*{box-sizing:border-box}body{margin:0;background:linear-gradient(180deg,#060b10,#09131c);color:var(--text);font-family:system-ui,-apple-system,Segoe UI,sans-serif}header,.wrap{max-width:1280px;margin:auto;padding:13px}header{display:flex;justify-content:space-between;gap:12px;align-items:center}h1{font-size:21px;margin:0}.sub,.note{font-size:10px;color:var(--muted)}.pill{padding:5px 8px;border:1px solid var(--line);border-radius:999px;font-size:10px;font-weight:800}.on{color:var(--ok)}.pending{color:var(--hit)}.cards{display:grid;grid-template-columns:repeat(6,1fr);gap:7px}.card,.panel{background:rgba(13,21,29,.97);border:1px solid var(--line);border-radius:11px;padding:10px}.k{font-size:9px;letter-spacing:.07em;color:var(--muted)}.v{font-size:18px;font-weight:900;margin-top:3px}.s1{color:var(--s1)}.s2{color:var(--s2)}.hit{color:var(--hit)}.main{display:grid;grid-template-columns:1.45fr .55fr;gap:8px;margin-top:8px}.row{display:flex;justify-content:space-between;gap:12px;margin:5px 0;font-size:11px}.row b{font-size:12px}.section{font-size:11px;font-weight:900;letter-spacing:.06em}.controls{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:8px}.ctrl{padding:8px;border:1px solid #223441;border-radius:8px;background:#09121a}.ctrlHead{display:flex;justify-content:space-between;gap:8px;font-size:10px}.ctrl input{width:100%;margin-top:7px;accent-color:#6bbdff}.blocks{display:grid;grid-template-columns:1.15fr .85fr;gap:8px;margin-top:8px}.buttons{display:flex;gap:7px;flex-wrap:wrap;margin-top:9px}button{padding:9px 11px;border:1px solid #3a5265;border-radius:8px;background:#132330;color:#fff;font-weight:800}.primary{background:#123c2b;border-color:#2a7752}.danger{background:#35191b;border-color:#70383d}.blue{background:#152b40;border-color:#376487}.status{font-size:10px;color:var(--muted);min-height:16px;margin-top:7px}canvas{display:block;width:100%;height:auto;background:#050a0e;border:1px solid #1d2d38;border-radius:9px}.metric{padding:7px;border:1px solid #213442;border-radius:8px;background:#09121a;margin-top:6px}.metric span{display:block;font-size:9px;color:var(--muted)}.metric b{display:block;font-size:13px;margin-top:2px}.meter{height:7px;background:#071017;border:1px solid #1e303c;border-radius:8px;overflow:hidden}.meter i{display:block;height:100%;background:#55d68f;width:0}@media(max-width:980px){header{align-items:flex-start;flex-direction:column}.cards{grid-template-columns:repeat(2,1fr)}.main,.blocks{grid-template-columns:1fr}.controls{grid-template-columns:1fr}}
</style></head><body>
<header><div><h1>SONIC BONGO · ANY SENSOR v15</h1><div class="sub">Cualquiera de los dos sostiene el punto · ambos se fusionan cuando están disponibles · punto y HIT son la misma Y</div></div><div><span id="runPill" class="pill">...</span> <span id="cfgPill" class="pill">...</span></div></header>
<div class="wrap"><div class="cards">
<div class="card"><div class="k">S1 STABLE</div><div id="d1" class="v s1">--</div></div><div class="card"><div class="k">S2 STABLE</div><div id="d2" class="v s2">--</div></div><div class="card"><div class="k">PUNTO Y</div><div id="yPoint" class="v">--</div></div><div class="card"><div class="k">FUENTE PUNTO</div><div id="source" class="v">--</div></div><div class="card"><div class="k">HITS</div><div id="hits" class="v hit">0</div></div><div class="card"><div class="k">POINT Hz REAL</div><div id="hz" class="v">--</div></div></div>
<div class="main"><section class="panel"><div class="row"><span class="section">ARCOS + PUNTO CENTRAL</span><b id="zone">--</b></div><canvas id="geo" width="860" height="570"></canvas><div class="note">El punto blanco y el detector HIT usan exactamente la misma Y. Si el punto entra debajo de la línea amarilla estando armado, genera HIT. S1 o S2 pueden sostenerlo solos; con ambos frescos se fusionan.</div></section>
<section class="panel"><div class="section">SEÑAL CENTRAL</div><div class="metric"><span>Y punto</span><b id="mPoint">--</b></div><div class="metric"><span>Candidato actual</span><b id="mCandidate">--</b></div><div class="metric"><span>Fuente</span><b id="mSource">--</b></div><div class="metric"><span>Edad S1 / S2</span><b id="ages">--</b></div><div class="metric"><span>Objetivo combinado</span><b id="targetCombined">--</b></div><div class="metric"><span>X matemática diagnóstico</span><b id="mX">--</b></div><div class="row"><span>Coherencia (ambos)</span><b id="coh">--</b></div><div class="meter"><i id="cohBar"></i></div><div class="row"><span>Sesgo lateral</span><b id="lateral">--</b></div></section></div>
<section class="panel" style="margin-top:8px"><div class="section">GEOMETRÍA</div><div class="controls"><div class="ctrl"><div class="ctrlHead"><span>Distancia sensores</span><b><span id="baselineV">1.0</span> cm</b></div><input id="baseline" type="range" min="0.5" max="10" step="0.1" value="1"></div><div class="ctrl"><div class="ctrlHead"><span>Plano HIT</span><b><span id="hitPlaneV">12.0</span> cm</b></div><input id="hitPlane" type="range" min="5" max="30" step="0.5" value="12"></div><div class="ctrl"><div class="ctrlHead"><span>Semiángulo</span><b><span id="angleV">7.5</span>°</b></div><input id="angle" type="range" min="3" max="20" step="0.5" value="7.5"></div></div></section>
<div class="blocks"><section class="panel"><div class="section">ADQUISICIÓN</div><div class="controls"><div class="ctrl"><div class="ctrlHead"><span>Rango máximo</span><b><span id="rangeV">50</span> cm</b></div><input id="range" type="range" min="20" max="100" step="5" value="50"></div><div class="ctrl"><div class="ctrlHead"><span>Objetivo por sensor</span><b><span id="rateV">16</span> Hz</b></div><input id="rate" type="range" min="5" max="16" step="1" value="16"></div><div class="ctrl"><div class="ctrlHead"><span>Estabilidad</span><b><span id="smoothV">55</span>%</b></div><input id="smooth" type="range" min="0" max="100" step="5" value="55"></div></div><div class="row"><span>Hz reales S1 / S2</span><b id="sensorHz">--</b></div><div class="row"><span>Edad válida S1 / S2</span><b id="ageRow">--</b></div><div class="row"><span>Timeouts</span><b id="timeouts">--</b></div><div class="row"><span>Fuera de rango</span><b id="rejects">--</b></div><div class="row"><span>Jitter</span><b id="jitter">--</b></div><div class="row"><span>RAW S1 / S2</span><b id="raw">--</b></div></section>
<section class="panel"><div class="section">HIT</div><div class="controls" style="grid-template-columns:1fr 1fr"><div class="ctrl"><div class="ctrlHead"><span>Referencia intensidad</span><b><span id="speedV">35</span> cm/s</b></div><input id="speed" type="range" min="10" max="180" step="5" value="35"></div><div class="ctrl"><div class="ctrlHead"><span>Retirada / rearm</span><b><span id="rearmV">3.0</span> cm</b></div><input id="rearm" type="range" min="1" max="10" step="0.5" value="3"></div></div><div class="row"><span>Estado</span><b id="motion">--</b></div><div class="row"><span>Velocidad punto</span><b id="vel">--</b></div><div class="row"><span>ARMADO</span><b id="armed">--</b></div><div class="row"><span>Último HIT</span><b id="lastHit">--</b></div><div class="buttons"><button class="danger" onclick="setRun(false)">DETENER</button><button class="primary" onclick="setRun(true)">INICIAR</button><button id="soundBtn" class="blue" onclick="toggleSound()">SONIDO OFF</button><button onclick="testSound()">PROBAR HIT</button></div></section></div>
<section class="panel" style="margin-top:8px"><div class="buttons"><button class="primary" onclick="saveConfig()">GUARDAR Y APLICAR</button><button onclick="loadCurrent()">CARGAR ACTUALES</button><button onclick="resetHits()">RESET HITS</button></div><div id="status" class="status"></div></section>
<section class="panel" style="margin-top:8px"><div class="row"><span class="section">TIMELINE</span><span class="note">S1 · S2 · punto Y · HIT</span></div><canvas id="timeline" width="1200" height="260"></canvas></section></div>
<script>
const $=id=>document.getElementById(id),G=$('geo').getContext('2d'),T=$('timeline').getContext('2d');const ids=['baseline','hitPlane','angle','range','rate','smooth','speed','rearm'];const labels={baseline:'baselineV',hitPlane:'hitPlaneV',angle:'angleV',range:'rangeV',rate:'rateV',smooth:'smoothV',speed:'speedV',rearm:'rearmV'};let first=true,dirty=false,lastData=null,lastSeq=null,soundOn=false,audioCtx=null;const hist=[],trail=[];
ids.forEach(id=>$(id).addEventListener('input',()=>{$(labels[id]).textContent=$(id).value;dirty=true;setCfgState();if(lastData)drawAll(lastData);}));
function setCfgState(){$('cfgPill').textContent=dirty?'CAMBIOS PENDIENTES':'CONFIG GUARDADA';$('cfgPill').className='pill '+(dirty?'pending':'on');}
function localCfg(){return{baseline:+$('baseline').value,hitPlane:+$('hitPlane').value,angle:+$('angle').value,maxRange:+$('range').value,rate:+$('rate').value,smooth:+$('smooth').value,speed:+$('speed').value,rearm:+$('rearm').value};}
function syncCfg(c){const m={baseline:c.baseline,hitPlane:c.hitPlane,angle:c.coneHalfAngle,range:c.maxRange,rate:c.targetHz,smooth:c.smoothing,speed:c.hitSpeed,rearm:c.rearm};Object.entries(m).forEach(([id,v])=>{$(id).value=v;$(labels[id]).textContent=v});dirty=false;setCfgState();}
function line(c,x1,y1,x2,y2,col='#2b3f4d',w=1){c.beginPath();c.moveTo(x1,y1);c.lineTo(x2,y2);c.strokeStyle=col;c.lineWidth=w;c.stroke();}function text(c,s,x,y,col='#8094a5',sz=10,a='left'){c.fillStyle=col;c.font=sz+'px system-ui';c.textAlign=a;c.fillText(s,x,y);}function arc(c,x,y,r,col,w=1,alpha=1){c.save();c.globalAlpha=alpha;c.beginPath();c.arc(x,y,r,Math.PI,Math.PI*2);c.strokeStyle=col;c.lineWidth=w;c.stroke();c.restore();}
function drawGeometry(j){const c=localCfg(),W=860,H=570,baseY=520,cx=W/2,max=c.maxRange,sc=(H-92)/max;G.clearRect(0,0,W,H);for(let q=5;q<=max;q+=5){const y=baseY-q*sc;line(G,48,y,W-24,y,q%10===0?'#19303d':'#10212b');if(q%10===0)text(G,q+' cm',8,y+3,'#60798a',10)}line(G,cx,34,cx,baseY,'#1b3340');const sx1=cx-c.baseline*.5*sc,sx2=cx+c.baseline*.5*sc,hy=baseY-c.hitPlane*sc;line(G,46,hy,W-24,hy,'#ffc85e',2);text(G,'HIT '+c.hitPlane.toFixed(1)+' cm',W-28,hy-7,'#ffc85e',11,'right');const len=max*sc,spread=Math.tan(c.angle*Math.PI/180)*len;[[sx1,'#58e8cd'],[sx2,'#76a0ff']].forEach(v=>{G.save();G.globalAlpha=.06;G.beginPath();G.moveTo(v[0],baseY);G.lineTo(v[0]-spread,baseY-len);G.lineTo(v[0]+spread,baseY-len);G.closePath();G.fillStyle=v[1];G.fill();G.restore();line(G,v[0],baseY,v[0]-spread,baseY-len,v[1]);line(G,v[0],baseY,v[0]+spread,baseY-len,v[1])});G.fillStyle='#58e8cd';G.fillRect(sx1-9,baseY-7,18,14);G.fillStyle='#76a0ff';G.fillRect(sx2-9,baseY-7,18,14);text(G,'S1',sx1,baseY+27,'#58e8cd',11,'center');text(G,'S2',sx2,baseY+27,'#76a0ff',11,'center');if(j.s1.fresh)arc(G,sx1,baseY,j.s1.stable*sc,'#58e8cd',2.3,.85);if(j.s2.fresh)arc(G,sx2,baseY,j.s2.stable*sc,'#76a0ff',2.3,.85);if(j.pair.valid){const py=baseY-j.pair.yPoint*sc;trail.push(py);if(trail.length>10)trail.shift();trail.forEach((yy,i)=>{G.save();G.globalAlpha=(i+1)/trail.length*.16;G.beginPath();G.arc(cx,yy,3,0,Math.PI*2);G.fillStyle='#fff';G.fill();G.restore()});G.beginPath();G.arc(cx,py,9,0,Math.PI*2);G.fillStyle=j.pair.below?'#ffc85e':'#fff';G.fill();G.beginPath();G.arc(cx,py,14,0,Math.PI*2);G.strokeStyle=j.pair.below?'#ffc85e':'#fff';G.globalAlpha=.35;G.stroke();G.globalAlpha=1;text(G,j.pair.yPoint.toFixed(2)+' cm',cx+15,py-10,j.pair.below?'#ffc85e':'#fff',12);$('zone').textContent=j.pair.below?'DEBAJO DEL HIT':'SOBRE EL HIT';}else{$('zone').textContent='SIN SEÑAL';}}
function drawTimeline(j){const c=localCfg(),W=1200,H=260,p=34,max=c.maxRange,y=v=>H-p-v/max*(H-2*p);T.clearRect(0,0,W,H);for(let q=10;q<=max;q+=10){line(T,p,y(q),W-10,y(q),'#14242e');text(T,q,8,y(q)+3)}line(T,p,y(c.hitPlane),W-10,y(c.hitPlane),'#ffc85e',1.5);function ser(k,col,w){T.beginPath();let st=false;hist.forEach((r,i)=>{const v=r[k];if(!Number.isFinite(v)||v>max){st=false;return}const x=p+i/Math.max(1,hist.length-1)*(W-p-15);if(!st){T.moveTo(x,y(v));st=true}else T.lineTo(x,y(v));});T.strokeStyle=col;T.lineWidth=w;T.stroke();}ser('s1','#58e8cd',1.1);ser('s2','#76a0ff',1.1);ser('point','#fff',2.5);hist.forEach((r,i)=>{if(r.hit){const x=p+i/Math.max(1,hist.length-1)*(W-p-15);line(T,x,12,x,H-p,'#ffc85e',2)}});}
function drawAll(j){drawGeometry(j);drawTimeline(j);}
async function ensureAudio(){if(!audioCtx)audioCtx=new(window.AudioContext||window.webkitAudioContext)();await audioCtx.resume();}function playHit(m=80){if(!soundOn||!audioCtx)return;const n=audioCtx.currentTime,o=audioCtx.createOscillator(),g=audioCtx.createGain();o.type='triangle';o.frequency.value=500+Math.min(127,m)*3.2;g.gain.setValueAtTime(.0001,n);g.gain.exponentialRampToValueAtTime(.16,n+.004);g.gain.exponentialRampToValueAtTime(.0001,n+.07);o.connect(g);g.connect(audioCtx.destination);o.start(n);o.stop(n+.08);}async function toggleSound(){await ensureAudio();soundOn=!soundOn;$('soundBtn').textContent=soundOn?'SONIDO ON':'SONIDO OFF';if(soundOn)playHit(60);}async function testSound(){await ensureAudio();soundOn=true;$('soundBtn').textContent='SONIDO ON';playHit(95);}
async function setRun(on){await fetch('/api/run?on='+(on?1:0),{cache:'no-store'});await refresh();}async function resetHits(){await fetch('/api/reset',{cache:'no-store'});lastSeq=null;await refresh();}
async function saveConfig(){const c=localCfg(),q=new URLSearchParams({base:c.baseline,hit:c.hitPlane,angle:c.angle,range:c.maxRange,rate:c.rate,smooth:c.smooth,speed:c.speed,rearm:c.rearm});$('status').textContent='Guardando...';const r=await fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:q.toString(),cache:'no-store'});const t=await r.text();if(!r.ok){$('status').textContent='ERROR: '+t;return}const o=JSON.parse(t);if(!o.ok){$('status').textContent='ERROR READ-BACK';return}syncCfg(o.cfg);$('status').textContent='GUARDADO ✓ · NVS verificado';await refresh();}
async function loadCurrent(){const r=await fetch('/api/status',{cache:'no-store'}),j=await r.json();syncCfg(j.cfg);lastData=j;drawAll(j);$('status').textContent='Valores actuales cargados';}
async function refresh(){try{const r=await fetch('/api/status',{cache:'no-store'}),j=await r.json();lastData=j;if(first){syncCfg(j.cfg);first=false}$('runPill').textContent=j.running?'DETECCIÓN ACTIVA':'DETENIDA';$('runPill').className='pill '+(j.running?'on':'');$('d1').textContent=j.s1.fresh?j.s1.stable.toFixed(2)+' cm':'--';$('d2').textContent=j.s2.fresh?j.s2.stable.toFixed(2)+' cm':'--';$('yPoint').textContent=j.pair.valid?j.pair.yPoint.toFixed(2)+' cm':'--';$('source').textContent=j.pair.source;$('hits').textContent=j.pair.hits;$('hz').textContent=j.pair.pointHz.toFixed(1);$('mPoint').textContent=j.pair.valid?j.pair.yPoint.toFixed(2)+' cm':'--';$('mCandidate').textContent=j.pair.valid?j.pair.yCandidate.toFixed(2)+' cm':'--';$('mSource').textContent=j.pair.source;$('ages').textContent=j.s1.ageMs+' / '+j.s2.ageMs+' ms';$('targetCombined').textContent=(j.cfg.targetHz*2).toFixed(0)+' Hz';$('mX').textContent=j.pair.geoValid?j.pair.x.toFixed(2)+' cm':'--';$('coh').textContent=j.pair.both?j.pair.coherence.toFixed(0)+'%':'1 SENSOR';$('cohBar').style.width=(j.pair.both?j.pair.coherence:50)+'%';$('lateral').textContent=j.pair.lateral;$('sensorHz').textContent=j.s1.hz.toFixed(1)+' / '+j.s2.hz.toFixed(1);$('ageRow').textContent=j.s1.ageMs+' / '+j.s2.ageMs+' ms';$('timeouts').textContent=j.s1.timeouts+' / '+j.s2.timeouts;$('rejects').textContent=j.s1.rejected+' / '+j.s2.rejected;$('jitter').textContent=j.pair.jitter.toFixed(2)+' cm';$('raw').textContent=j.s1.raw.toFixed(1)+' / '+j.s2.raw.toFixed(1)+' cm';$('motion').textContent=j.pair.motion;$('vel').textContent=j.pair.velocity.toFixed(0)+' cm/s';$('armed').textContent=j.pair.armed?'SÍ':'NO';$('lastHit').textContent=j.pair.lastHitMs?(j.pair.lastHitZ.toFixed(1)+' cm · MIDI '+j.pair.lastMidi):'--';let hit=false;if(lastSeq===null)lastSeq=j.event.seq;else if(j.event.seq!==lastSeq){hit=true;lastSeq=j.event.seq;playHit(j.event.midi||80)}hist.push({s1:j.s1.fresh?j.s1.stable:NaN,s2:j.s2.fresh?j.s2.stable:NaN,point:j.pair.valid?j.pair.yPoint:NaN,hit});if(hist.length>220)hist.shift();drawAll(j);}catch(e){$('status').textContent='Error de comunicación';}}
setInterval(refresh,40);refresh();
</script></body></html>
)HTML";

String cfgJson(){String j="{";j+="\"baseline\":"+String(baselineCm,2)+",\"maxRange\":"+String(maxRangeCm,1)+",\"hitPlane\":"+String(hitPlaneCm,1)+",\"coneHalfAngle\":"+String(coneHalfAngleDeg,1)+",\"targetHz\":"+String(targetHzPerSensor,1)+",\"smoothing\":"+String(smoothingPct,0)+",\"hitSpeed\":"+String(hitSpeedCmS,1)+",\"rearm\":"+String(rearmMarginCm,1)+",\"slotMs\":"+String(slotMs())+",\"echoTimeoutUs\":"+String(echoTimeoutUs())+"}";return j;}

void sendStatus(){
  uint32_t now=millis();String j="{";j+="\"fw\":\""+String(FW_VERSION)+"\",\"running\":"+String(acquisitionRunning?"true":"false")+",";
  for(int i=0;i<2;i++){SensorState&q=s[i];bool fresh=sensorFresh(i,now);uint32_t age=sensorAge(i,now);j+="\"s"+String(i+1)+"\":{";j+="\"ok\":"+String(q.ok?"true":"false")+",\"fresh\":"+String(fresh?"true":"false")+",\"ageMs\":"+String(age)+",\"raw\":"+String(q.rawCm,2)+",\"fast\":"+String(q.fastCm,2)+",\"stable\":"+String(q.stableCm,2)+",\"velocity\":"+String(q.velocityCmS,2)+",\"jitter\":"+String(q.jitterCm,2)+",\"timeouts\":"+String(q.timeouts)+",\"rejected\":"+String(q.rejected)+",\"hz\":"+String(q.hz,2)+"}";if(i==0)j+=",";}
  j+=",\"pair\":{";j+="\"valid\":"+String(pair.valid?"true":"false")+",\"both\":"+String(pair.both?"true":"false")+",\"source\":\""+sourceLabel()+"\",\"sourceMask\":"+String(pair.sourceMask)+",\"yPoint\":"+String(pair.yPoint,2)+",\"yCandidate\":"+String(pair.yCandidate,2)+",\"yStable\":"+String(pair.yStable,2)+",\"velocity\":"+String(pair.velocity,2)+",\"below\":"+String((pair.valid&&pair.yPoint<=hitPlaneCm)?"true":"false")+",\"geoValid\":"+String(pair.geoValid?"true":"false")+",\"x\":"+String(pair.x,2)+",\"zGeo\":"+String(pair.zGeo,2)+",\"jitter\":"+String(pair.jitter,2)+",\"coherence\":"+String(pair.coherence,1)+",\"lateral\":\""+pair.lateral+"\",\"armed\":"+String(pair.armed?"true":"false")+",\"hits\":"+String(pair.hits)+",\"lastHitMs\":"+String(pair.lastHitMs)+",\"lastHitVelocity\":"+String(pair.lastHitVelocity,1)+",\"lastHitZ\":"+String(pair.lastHitZ,2)+",\"lastMidi\":"+String(pair.lastMidi)+",\"pointHz\":"+String(pair.pointHz,2)+",\"motion\":\""+motionLabel()+"\"},";
  j+="\"event\":{\"seq\":"+String(pair.eventSeq)+",\"pad\":1,\"midi\":"+String(pair.lastMidi)+",\"velocity\":"+String(pair.lastHitVelocity,1)+",\"z\":"+String(pair.lastHitZ,2)+"},\"cfg\":"+cfgJson()+"}";server.send(200,"application/json",j);
}

void handleConfig(){bool wasRunning=acquisitionRunning;acquisitionRunning=false;if(server.hasArg("base"))baselineCm=clampf(server.arg("base").toFloat(),.5f,10);if(server.hasArg("range"))maxRangeCm=clampf(server.arg("range").toFloat(),20,100);if(server.hasArg("hit"))hitPlaneCm=clampf(server.arg("hit").toFloat(),5,30);if(server.hasArg("angle"))coneHalfAngleDeg=clampf(server.arg("angle").toFloat(),3,20);if(server.hasArg("rate"))targetHzPerSensor=clampf(server.arg("rate").toFloat(),5,16);if(server.hasArg("smooth"))smoothingPct=clampf(server.arg("smooth").toFloat(),0,100);if(server.hasArg("speed"))hitSpeedCmS=clampf(server.arg("speed").toFloat(),10,180);if(server.hasArg("rearm"))rearmMarginCm=clampf(server.arg("rearm").toFloat(),1,10);String detail;bool ok=saveAndVerifyConfig(detail);resetTracking(true);nextSensor=0;nextMeasureMs=millis()+80;acquisitionRunning=wasRunning;String j="{\"ok\":"+String(ok?"true":"false")+",\"detail\":\""+detail+"\",\"cfg\":"+cfgJson()+"}";server.send(ok?200:500,"application/json",j);}
void handleRun(){if(server.hasArg("on"))acquisitionRunning=server.arg("on")!="0";if(acquisitionRunning){resetTracking(true);nextSensor=0;nextMeasureMs=millis()+80;}server.send(200,"text/plain","OK");}
void setup(){Serial.begin(115200);delay(120);loadConfig();for(int i=0;i<2;i++){pinMode(PINS[i].echo,INPUT);pinMode(PINS[i].trig,OUTPUT);digitalWrite(PINS[i].trig,LOW);}WiFi.mode(WIFI_AP);WiFi.softAP(AP_SSID);server.on("/",[](){server.send_P(200,"text/html",PAGE);});server.on("/api/status",HTTP_GET,sendStatus);server.on("/api/config",HTTP_POST,handleConfig);server.on("/api/run",HTTP_GET,handleRun);server.on("/api/reset",HTTP_GET,[](){resetTracking(false);server.send(200,"text/plain","OK");});server.begin();nextMeasureMs=millis()+300;}
void loop(){server.handleClient();serviceSensors();delay(1);}
