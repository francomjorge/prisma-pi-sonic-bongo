/*
 SONIC BONGO v19.0 - HYBRID SENSOR DIAGNOSTIC
 New S1: HC-SR04 GPIO13 TRIG, GPIO14 ECHO (through TXS0108E B1->A1).
 New S2: VL53L1X I2C GPIO8 SDA, GPIO9 SCL (0x29).
 Old HC-SR04 S2 on GPIO15/16 is DISABLED. Never set it to distance zero.
 Buzzer NOT connected. Legacy v18 source included, not executed in stage 1.
*/
#include <Wire.h>
#include <VL53L1X.h>
#include "base_v18.h"

static const char* HYBRID_VER = "19.1-hybrid-function";
static const uint8_t TOF_ADDR=0x29, TOF_SDA=8, TOF_SCL=9;
VL53L1X tof;
bool hybridRunning=true, tofReady=false;
uint32_t lastS1At=0, lastTofInitAt=0, rateAt=0;
uint32_t previousS1Count=0,previousS2Count=0;
uint16_t tofStatus=255;
String tofInitStatus="PENDIENTE";

struct HybridRead {
 bool have=false, valid=false;
 float cm=0, filtered=0, hz=0;
 uint32_t seq=0, atMs=0, attempts=0, rejected=0, timeouts=0;
 float signal=0, ambient=0;
};
HybridRead u, t;

// --------- HYBRID FUSION / LEARNING BRIDGE (v19.1) ----------
static const size_t TRACE_CAP=480, BEAT_CAP=256, HYBRID_CFG_CAP=32;
struct HybridTrace {
 uint32_t seq,tMs,eventSeq;
 float y,v,a,ultra,tof,quality;
 uint8_t source;
};
HybridTrace traceRows[TRACE_CAP];
size_t traceWrite=0,traceCount=0;
uint32_t traceSeq=0;
struct BeatRef {uint32_t espMs,pointSeq,eventSeq,index;float bpm,clientMs;};
BeatRef beatRows[BEAT_CAP];
size_t beatWrite=0,beatCount=0;
struct HConfig{uint32_t revision,tMs;float tofWeight,agreeCm,tofOffset,bpm,minSpeed,minAmp;};
HConfig hCfgRows[HYBRID_CFG_CAP];
size_t hCfgWrite=0,hCfgCount=0;
float tofWeight=0.85f, agreementCm=7.0f, tofOffsetCm=0, metronomeBpm=120.0f;
uint32_t lastHybridEventSeq=0,calibrationChanges=0,tooDifferent=0;
bool prevFusedReady=false;
void recordHybridConfig() {
 captureConfigIfChanged(true);
 HConfig r={configRevision,millis(),tofWeight,agreementCm,tofOffsetCm,metronomeBpm,funcMinDownSpeed,funcMinAmplitude};
 hCfgRows[hCfgWrite]=r;
 hCfgWrite=(hCfgWrite+1)%HYBRID_CFG_CAP;
 if(hCfgCount<HYBRID_CFG_CAP)hCfgCount++;
}
void loadHybridConfig(){
 prefs.begin("hybrid19",true);
 tofWeight=clampf(prefs.getFloat("wtof",.85f),.5f,1.f);
 agreementCm=clampf(prefs.getFloat("agree",7.f),1.f,20.f);
 tofOffsetCm=clampf(prefs.getFloat("offset",0),-10.f,10.f);
 metronomeBpm=clampf(prefs.getFloat("bpm",120),40,280);
 prefs.end();
 recordHybridConfig();
}
void saveHybridConfig(){
 prefs.begin("hybrid19",false);
 prefs.putFloat("wtof",tofWeight);
 prefs.putFloat("agree",agreementCm);
 prefs.putFloat("offset",tofOffsetCm);
 prefs.putFloat("bpm",metronomeBpm);
 prefs.end();
}
void recordTrace(){
 HybridTrace r={};
 r.seq=++traceSeq;r.tMs=millis();r.eventSeq=motion.eventSeq;
 r.y=pair.yPoint;r.v=pair.velocity;r.a=pair.acceleration;
 r.ultra=u.have?u.cm:NAN;r.tof=t.have?t.cm:NAN;
 r.quality=pair.coherence;r.source=pair.sourceMask;
 traceRows[traceWrite]=r;traceWrite=(traceWrite+1)%TRACE_CAP;
 if(traceCount<TRACE_CAP)traceCount++;
}
void hybridFusionTick() {
 uint32_t now=millis();
 // Only fresh valid data. A missing measurement never becomes zero.
 bool us=u.have && u.valid && (now-u.atMs)<=160;
 bool optical=t.have && t.valid && tofReady && (now-t.atMs)<=105;
 if(!us&&!optical){pair.valid=false;return;}
 float uFast=s[0].fastCm, uStable=s[0].stableCm;
 float tFast=t.cm+tofOffsetCm,tStable=t.filtered+tofOffsetCm;
 if(us&&optical&&fabsf(uFast-tFast)>agreementCm){
   // A single ultrasonic beam and narrow ToF beam may see different targets.
   // Prefer ToF rather than averaging different physical surfaces.
   us=false;tooDifferent++;
 }
 float fast,stable;
 if(optical && us) {
   fast=tofWeight*tFast+(1-tofWeight)*uFast;
   stable=tofWeight*tStable+(1-tofWeight)*uStable;
 } else if(optical) {fast=tFast;stable=tStable;}
 else {fast=uFast;stable=uStable;}
 uint8_t source=(us?1:0)|(optical?2:0);
 pair.valid=true;pair.havePoint=true;pair.both=(source==3);pair.sourceMask=source;
 pair.yCandidate=fast;
 pair.yStable=stable;
 if(!prevFusedReady) {
   pair.yPoint=fast;pair.prevPoint=fast;pair.prevPointMs=now;
   pair.velocity=0;pair.acceleration=0;prevFusedReady=true;
   motion.peakY=fast;
 } else {
   float alpha=clampf(pointAlpha(),.55f,.95f);
   float newPoint=alpha*fast+(1-alpha)*pair.yPoint;
   float dt=(now-pair.prevPointMs)/1000.0f;
   if(dt>=.008f && dt<=.35f){
     float rawV=clampf((newPoint-pair.yPoint)/dt,-650,650);
     float newV=.55f*rawV+.45f*pair.velocity;
     float rawA=clampf((newV-pair.velocity)/dt,-11000,11000);
     pair.acceleration=.34f*rawA+.66f*pair.acceleration;
     pair.prevVelocity=pair.velocity;
     pair.velocity=newV;
   }
   pair.prevPoint=pair.yPoint;pair.yPoint=newPoint;pair.prevPointMs=now;
 }
 pair.delta=(us&&optical)?uStable-tStable:0;
 pair.jitter=us?(optical?0.5f*(s[0].jitterCm+fabsf(t.cm-t.filtered)):s[0].jitterCm):fabsf(t.cm-t.filtered);
 pair.coherence=pair.both?clampf(100.f-fabsf(pair.delta)*10.f-pair.jitter*6.f,0,100):(optical?75.f:40.f);
 // 2 different FoVs at unknown optical-center separation: no bogus trilateration.
 pair.geoValid=false;pair.x=0;pair.zGeo=0;pair.lateral="NO APLICA";
 pair.pointUpdates++;pair.pointSeq++;
 updateMotion(now);
 updateAdaptiveBaseline();
 datasetTick();                  // v17 SAMPLES/EVENTS and config snapshots preserved.
 if(motion.eventSeq!=lastHybridEventSeq){lastHybridEventSeq=motion.eventSeq;}
 recordTrace();
}
void onTofValid() {
 // Feed original dataset columns with VL53L1X values, no fabricated pulse.
 SensorState &q=s[1];uint32_t now=millis();
 q.have=true;q.ok=true;q.rawCm=t.cm;q.fastCm=t.cm;q.stableCm=t.filtered;
 q.lastGoodMs=now;q.valid++;q.lastAttemptMs=now;
 q.jitterCm=.9f*q.jitterCm+.1f*fabsf(t.cm-t.filtered);
 hybridFusionTick();
}
void appendBeatReference() {
 if(!server.hasArg("i")||!server.hasArg("bpm")){
   server.send(400,"application/json","{\"ok\":false,\"error\":\"missing beat index or bpm\"}");return;
 }
 BeatRef b={};
 b.espMs=millis();
 b.pointSeq=pair.pointSeq;b.eventSeq=motion.eventSeq;
 b.index=(uint32_t)server.arg("i").toInt();
 b.bpm=clampf(server.arg("bpm").toFloat(),40,280);
 b.clientMs=server.hasArg("client")?server.arg("client").toFloat():0;
 beatRows[beatWrite]=b;beatWrite=(beatWrite+1)%BEAT_CAP;
 if(beatCount<BEAT_CAP)beatCount++;
 server.send(200,"application/json","{\"ok\":true,\"note\":\"metronome_reference_not_human_ground_truth\"}");
}
void handleHybridSettings() {
 if(server.hasArg("w"))tofWeight=clampf(server.arg("w").toFloat(),.5f,1.f);
 if(server.hasArg("agree"))agreementCm=clampf(server.arg("agree").toFloat(),1.f,20.f);
 if(server.hasArg("offset"))tofOffsetCm=clampf(server.arg("offset").toFloat(),-10.f,10.f);
 if(server.hasArg("bpm"))metronomeBpm=clampf(server.arg("bpm").toFloat(),40,280);
 if(server.hasArg("down"))funcMinDownSpeed=clampf(server.arg("down").toFloat(),10,250);
 if(server.hasArg("amp"))funcMinAmplitude=clampf(server.arg("amp").toFloat(),.4f,10.f);
 saveHybridConfig();
 String detail;
 bool okBase=saveAndVerifyConfig(detail);
 prefs.begin("hybrid19",true);
 bool ok=okBase && fabsf(prefs.getFloat("wtof",-999)-tofWeight)<.01f &&
  fabsf(prefs.getFloat("agree",-999)-agreementCm)<.01f &&
  fabsf(prefs.getFloat("offset",-999)-tofOffsetCm)<.01f &&
  fabsf(prefs.getFloat("bpm",-999)-metronomeBpm)<.01f;
 prefs.end();
 if(ok){recordHybridConfig();calibrationChanges++;}
 String j=String("{\"ok\":")+(ok?"true":"false")+
  ",\"revision\":"+String(configRevision)+",\"detail\":\""+detail+"\"}";
 server.send(ok?200:500,"application/json",j);
}
void hybridTrace() {
 uint32_t after=server.hasArg("after")?(uint32_t)server.arg("after").toInt():0;
 String j="{\"points\":[";
 bool first=true;size_t sent=0;
 for(size_t i=0;i<traceCount;i++) {
   const HybridTrace &q=traceRows[(traceWrite+TRACE_CAP-traceCount+i)%TRACE_CAP];
   if(q.seq<=after)continue;
   if(++sent>64)break;
   if(!first)j+=",";first=false;
   j+="["+String(q.seq)+","+String(q.tMs)+","+String(q.y,2)+","+String(q.v,1)+","+String(q.a,0)+","+
    (isfinite(q.ultra)?String(q.ultra,2):String("null"))+","+
    (isfinite(q.tof)?String(q.tof,2):String("null"))+","+String(q.eventSeq)+","+String(q.source)+"]";
 }
 j+="],\"lastSeq\":"+String(traceSeq)+"}";
 server.sendHeader("Cache-Control","no-store");
 server.send(200,"application/json",j);
}
void handleHybridData() {
 String j="{\"revision\":"+String(configRevision)+",\"sampleCount\":"+String(sampleCount)+
 ",\"eventCount\":"+String(eventCount)+",\"beatRefCount\":"+String(beatCount)+
 ",\"sessionId\":"+String(datasetSessionId)+",\"note\":\"beat_clicks_are_reference_not_labels\"}";
 server.send(200,"application/json",j);
}
void exportBeatRefs(){
 server.setContentLength(CONTENT_LENGTH_UNKNOWN);
 server.send(200,"text/csv","");
 server.sendContent("session_id,esp_received_ms,client_time_ms,beat_index,target_bpm,point_seq,event_seq\n");
 for(size_t i=0;i<beatCount;i++){
   const BeatRef &q=beatRows[(beatWrite+BEAT_CAP-beatCount+i)%BEAT_CAP];
   server.sendContent(String(datasetSessionId)+","+q.espMs+","+String(q.clientMs,1)+","+q.index+","+String(q.bpm,1)+","+q.pointSeq+","+q.eventSeq+"\n");
 }
 server.sendContent("");
}


void updateHybridRate() {
 uint32_t now=millis();
 if(rateAt==0){ rateAt=now;previousS1Count=u.seq;previousS2Count=t.seq;return; }
 uint32_t dt=now-rateAt;
 if(dt<1000)return;
 float seconds=dt/1000.0f;
 u.hz=(u.seq-previousS1Count)/seconds;
 t.hz=(t.seq-previousS2Count)/seconds;
 previousS1Count=u.seq;previousS2Count=t.seq;rateAt=now;
}

void initTof() {
 lastTofInitAt=millis();
 Wire.beginTransmission(TOF_ADDR);
 if(Wire.endTransmission()!=0){
  tofReady=false;tofInitStatus="NO_I2C_0x29";return;
 }
 tof.setTimeout(100);
 if(!tof.init()) {
  tofReady=false;tofInitStatus="FALLO_INIT";return;
 }
 if(!tof.setDistanceMode(VL53L1X::Short) || !tof.setMeasurementTimingBudget(20000)) {
  tofReady=false;tofInitStatus="FALLO_MODO";return;
 }
 tof.startContinuous(35);
 tofReady=true;tofInitStatus="OK_SHORT_20MS_35MS";
}

// S1 diagnostic: extended echo window and independent GPIO state sampling.
// This changes HC-SR04 only. The VL53L1X path is unchanged.
static const uint32_t S1_DIAG_TIMEOUT_US=12000; // ~2 m theoretical; not an acquisition claim
uint32_t s1LastPulseUs=0,s1LastPingMs=0,s1NoPulse=0,s1PinHighBefore=0;
uint8_t s1EchoBefore=0,s1EchoAfter=0;
float s1LastEchoCm=0;
void sampleS1() {
 uint32_t before=s[0].valid;
 u.attempts++;
 s1EchoBefore=digitalRead(PINS[0].echo);
 if(s1EchoBefore) s1PinHighBefore++;
 digitalWrite(PINS[0].trig,LOW);delayMicroseconds(3);
 digitalWrite(PINS[0].trig,HIGH);delayMicroseconds(10);
 digitalWrite(PINS[0].trig,LOW);
 s1LastPingMs=millis();
 uint32_t pulse=pulseIn(PINS[0].echo,HIGH,S1_DIAG_TIMEOUT_US);
 s1EchoAfter=digitalRead(PINS[0].echo);
 s1LastPulseUs=pulse;
 if(pulse){
   s1LastEchoCm=pulse*0.0343f*0.5f;
   processPulse(0,pulse);
   if(s[0].valid==before)u.rejected++;
 }else{
   s[0].ok=false;s[0].lastAttemptMs=millis();s[0].pulseUs=0;
   s[0].timeouts++;u.timeouts++;s1NoPulse++;
 }
 if(s[0].valid!=before) {
   u.cm=s[0].rawCm;
   u.filtered=s[0].stableCm;
   u.seq++;u.atMs=millis();u.have=true;u.valid=true;
  hybridFusionTick();
 } else {
   u.valid=false;
 }
}

void sampleTof() {
 if(!tofReady)return;
 if(!tof.dataReady()){
  if(tof.last_status != 0) {
   tofInitStatus="I2C_ERROR";tofReady=false;
  }
  return;
 }
 uint16_t mm=tof.read(false); // non-blocking, only when dataReady() is true.
 t.attempts++;tofStatus=(uint16_t)tof.ranging_data.range_status;
 if(tof.timeoutOccurred() || tof.last_status != 0) {
  t.timeouts++;t.valid=false;
  return;
 }
 // RangeValid is status 0. Keep raw data distinct from filtered; invalid never means 0 cm.
 bool good=(tofStatus==VL53L1X::RangeValid && mm>=40 && mm<=maxRangeCm*10.0f);
 if(!good) {
  t.rejected++;t.valid=false;
  return;
 }
 t.cm=mm*0.1f;
 t.filtered=t.have?(0.42f*t.cm+0.58f*t.filtered):t.cm;
 t.atMs=millis();t.seq++;t.valid=t.have=true;
 t.signal=tof.ranging_data.peak_signal_count_rate_MCPS;
 t.ambient=tof.ranging_data.ambient_count_rate_MCPS;
 onTofValid();
}

void serviceHybrid() {
 if(!hybridRunning){updateHybridRate();return;}
 uint32_t now=millis();
 uint32_t period=2*slotMs(); // from v18 config, nominal 62 ms at 16 Hz/sensor.
 if((int32_t)(now-lastS1At)>=(int32_t)period){
  lastS1At=now;
  sampleS1();
 }
 if(!tofReady && now-lastTofInitAt>=5000)initTof();
 if(tofReady)sampleTof();
 updateHybridRate();
}

String hybridSensorJson(const HybridRead &q,bool fresh){
 String j="{";
 j+="\"seq\":"+String(q.seq)+",";
 j+="\"valid\":"+String(fresh?"true":"false")+",";
 j+="\"lastValid\":"+String(q.have?"true":"false")+",";
 j+="\"rawCm\":"+String(q.cm,2)+",";
 j+="\"smoothCm\":"+String(q.filtered,2)+",";
 j+="\"ageMs\":"+String(q.have?millis()-q.atMs:999999)+",";
 j+="\"hz\":"+String(q.hz,2)+",";
 j+="\"attempts\":"+String(q.attempts)+",";
 j+="\"rejected\":"+String(q.rejected)+",";
 j+="\"timeouts\":"+String(q.timeouts)+",";
 j+="\"signal\":"+String(q.signal,2)+",";
 j+="\"ambient\":"+String(q.ambient,2);
 return j+"}";
}

void hybridStatus() {
 uint32_t now=millis();
 bool f1=hybridRunning&&u.valid&&u.have&&now-u.atMs<250;
 bool f2=hybridRunning&&t.valid&&t.have&&now-t.atMs<180&&tofReady;
 String j="{\"fw\":\""+String(HYBRID_VER)+"\",";
 j+="\"running\":"+String(hybridRunning?"true":"false")+",";
 j+="\"s1\":"+hybridSensorJson(u,f1)+",";
 j+="\"s1Diag\":{\"pulseUs\":"+String(s1LastPulseUs)+",\"echoCm\":"+String(s1LastEchoCm,2)+",\"timeoutUs\":"+String(S1_DIAG_TIMEOUT_US)+",\"echoBefore\":"+String(s1EchoBefore)+",\"echoAfter\":"+String(s1EchoAfter)+",\"highBeforeCount\":"+String(s1PinHighBefore)+",\"noPulseCount\":"+String(s1NoPulse)+"},";
 j+="\"s2\":"+hybridSensorJson(t,f2)+",";
 j+="\"tofReady\":"+String(tofReady?"true":"false")+",";
 j+="\"tofInit\":\""+tofInitStatus+"\",";
 j+="\"tofStatus\":"+String(tofStatus)+",";
 j+="\"i2cAddress\":\"0x29\",";
 j+="\"tofPeriodMs\":35,";
 j+="\"hcPeriodMs\":"+String(2*slotMs())+",";
 j+="\"maxRangeCm\":"+String(maxRangeCm,0)+",";
 j+="\"oldS2\":\"DISABLED\",";
 j+="\"fusion\":{\"valid\":"+String(pair.valid?"true":"false")+",\"y\":"+String(pair.yPoint,2)+",\"velocity\":"+String(pair.velocity,1)+",\"acceleration\":"+String(pair.acceleration,0)+",\"sourceMask\":"+String(pair.sourceMask)+",\"quality\":"+String(pair.coherence,1)+",\"seq\":"+String(pair.pointSeq)+",\"hits\":"+String(motion.candidates)+",\"eventSeq\":"+String(motion.eventSeq)+",\"midi\":"+String(motion.midi)+",\"phase\":\""+motion.phase+"\",\"baseline\":"+String(learnedBaseline,2)+",\"roll\":"+String(motion.roll?"true":"false")+"},";
 j+="\"cfgHybrid\":{\"tofWeight\":"+String(tofWeight,2)+",\"agreement\":"+String(agreementCm,1)+",\"offset\":"+String(tofOffsetCm,2)+",\"bpm\":"+String(metronomeBpm,1)+",\"down\":"+String(funcMinDownSpeed,1)+",\"amp\":"+String(funcMinAmplitude,2)+",\"revision\":"+String(configRevision)+"},";
 j+="\"legacyEngine\":\"preserved_not_executing\",";
 j+="\"uptimeMs\":"+String(now)+"}";
 server.sendHeader("Cache-Control","no-store");
 server.send(200,"application/json",j);
}
void hybridRun(){
 if(!server.hasArg("on")){server.send(400,"text/plain","on requerido");return;}
 bool next=server.arg("on")!="0";
 if(next && !hybridRunning){
   lastS1At=0;
   if(tofReady)tof.startContinuous(35);
 }
 if(!next && hybridRunning && tofReady)tof.stopContinuous();
 hybridRunning=next;
 server.send(200,"application/json",String("{\"running\":")+(hybridRunning?"true":"false")+"}");
}
const char HYBRID_PAGE[] PROGMEM=R"PAGE(
<!doctype html><html lang="es"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SONIC BONGO v19 Hybrid</title><style>
:root{--bg:#080e15;--panel:#101d29;--border:#264052;--fg:#e7f1fa;--muted:#9daebe;--u:#66d7ee;--t:#f2ad88;--ok:#6edba4;--bad:#fd8686}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px system-ui,-apple-system,Segoe UI,sans-serif}
header,main{max-width:1100px;margin:auto;padding:16px}h1{font-size:22px;margin:0}h2{font-size:17px;margin:0}.sub{color:var(--muted);font-size:12px;margin-top:3px}
.panel{background:var(--panel);border:1px solid var(--border);border-radius:14px;padding:14px;margin-top:12px}
.sensorHead{display:flex;align-items:center;gap:16px}.sensorIcon{width:94px;flex:0 0 94px}.sensorIcon svg{width:90px;height:76px}
.distance{font-size:31px;font-weight:800;letter-spacing:-.04em}.meta{color:var(--muted);font-size:11px}
.statRow{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-top:10px}
.stat{border:1px solid #243b4b;border-radius:8px;padding:8px}.stat b{display:block;font-size:14px;margin-top:2px}.stat span{color:var(--muted);font-size:10px}
canvas{display:block;width:100%;height:auto;margin-top:12px;border:1px solid #1d3344;border-radius:8px;background:#08131b}
button,a.cta{display:inline-block;background:#1d3545;color:#fff;border:1px solid #4f7085;border-radius:8px;padding:10px 14px;text-decoration:none;cursor:pointer;font-weight:700}
button:active{transform:translateY(1px)}.pill{display:inline-block;border:1px solid #406c55;border-radius:15px;color:var(--ok);font-size:10px;padding:5px 9px}
.actions{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}.note{font-size:12px;color:var(--muted);line-height:1.5}
@media(min-width:700px){.statRow{grid-template-columns:repeat(6,1fr)}}@media(max-width:500px){.statRow{grid-template-columns:repeat(3,1fr)}}
</style></head><body>
<header><h1>SONIC BONGO · SENSOR LAB</h1><div class="sub">v19.0 · diagnóstico híbrido · dos sensores independientes, sin fusión ni HIT</div></header>
<main>
<div class="actions"><button onclick="run(true)">INICIAR</button><button onclick="run(false)">DETENER</button>
<span class="pill" id="runState">Conectando...</span></div>
<section class="panel">
 <div class="sensorHead"><div class="sensorIcon">
 <svg viewBox="0 0 94 76"><rect x="2" y="15" width="90" height="46" rx="7" fill="#173b53" stroke="#62b9e0" stroke-width="2"/>
 <circle cx="26" cy="38" r="18" fill="#07121b" stroke="#8bcbe5" stroke-width="5"/><circle cx="68" cy="38" r="18" fill="#07121b" stroke="#8bcbe5" stroke-width="5"/>
 </svg></div><div><h2>S1 · HC-SR04</h2><div class="sub">Ultrasónico · TRIG GPIO13 · ECHO GPIO14 vía TXS0108E</div>
 <div id="d1" class="distance" style="color:var(--u)">-- cm</div><div class="meta" id="s1state">Sin lecturas</div></div></div>
 <canvas id="chart1" width="900" height="160"></canvas>
 <div class="statRow"><div class="stat"><span>Hz reales</span><b id="hz1">--</b></div><div class="stat"><span>Lecturas válidas</span><b id="valid1">--</b></div><div class="stat"><span>Timeouts</span><b id="timeout1">--</b></div><div class="stat"><span>Fuera de rango</span><b id="reject1">--</b></div><div class="stat"><span>Edad</span><b id="age1">--</b></div><div class="stat"><span>Suavizada</span><b id="smooth1">--</b></div><div class="stat"><span>ECHO pulso (µs)</span><b id="echoPulse">--</b></div><div class="stat"><span>ECHO calculado</span><b id="echoCm">--</b></div><div class="stat"><span>ECHO antes/después</span><b id="echoLevel">--</b></div></div><p class="note" id="echoNote">Diagnóstico: esperando TRIG y ECHO.</p>
</section>
<section class="panel">
 <div class="sensorHead"><div class="sensorIcon">
 <svg viewBox="0 0 94 76"><rect x="20" y="5" width="54" height="66" rx="8" fill="#892c3b" stroke="#f2a18e" stroke-width="2"/>
 <circle cx="29" cy="14" r="3" fill="#fff4db"/><circle cx="65" cy="14" r="3" fill="#fff4db"/>
 <rect x="34" y="22" width="25" height="30" rx="4" fill="#1e1922" stroke="#ffd5c4"/><rect x="40" y="26" width="13" height="22" rx="2" fill="#718b97"/>
 <circle cx="29" cy="62" r="3" fill="#fff4db"/><circle cx="65" cy="62" r="3" fill="#fff4db"/>
 </svg></div><div><h2>S2 · VL53L1X</h2><div class="sub">Láser ToF · I²C GPIO8/9 · 0x29 · Short / 20 ms</div>
 <div id="d2" class="distance" style="color:var(--t)">-- cm</div><div class="meta" id="s2state">Inicializando I²C...</div></div></div>
 <canvas id="chart2" width="900" height="160"></canvas>
 <div class="statRow"><div class="stat"><span>Hz reales</span><b id="hz2">--</b></div><div class="stat"><span>Lecturas válidas</span><b id="valid2">--</b></div><div class="stat"><span>Estado VL53</span><b id="status2">--</b></div><div class="stat"><span>Rechazadas</span><b id="reject2">--</b></div><div class="stat"><span>Edad</span><b id="age2">--</b></div><div class="stat"><span>Señal/ambiente</span><b id="sig2">--</b></div></div>
</section>
<section class="panel"><h2>Adquisición y siguientes etapas</h2>
 <p class="note">HC-SR04 nominal: <b id="rate1">--</b> ms entre pings. VL53L1X: 35 ms nominal; las tasas reales aparecen arriba. Gráficos con muestras nuevas, no valores repetidos por el navegador.</p>
 <p class="note">Antiguo sensor S2 ultrasónico: deshabilitado (GPIO15/16 no disparan). Sin distancia cero ficticia. El motor v18 queda preservado en el firmware para integración posterior; no controla golpes en esta página.</p>
 <p class="note">15 mm entre bordes físicos de placas. No se infiere la separación entre centros ni se fusionan mediciones hasta validar ambos sensores.</p>
 <p class="note" id="err"></p>
 </section>
</main><script>
const $=x=>document.getElementById(x),hist=[[],[]],lastSeq=[-1,-1];
function draw(n,max){
 const c=$('chart'+(n+1)),g=c.getContext('2d'),w=c.width,h=c.height,p=28;
 g.clearRect(0,0,w,h);
 for(let d=0;d<=max;d+=10){let y=h-p-(d/max)*(h-p-8);g.strokeStyle='#193544';g.beginPath();g.moveTo(p,y);g.lineTo(w-8,y);g.stroke();g.fillStyle='#6c8797';g.font='10px system-ui';g.fillText(d+'',3,y+3);}
 const a=hist[n],cut=performance.now()-8000,lim=a.filter(x=>x[0]>=cut);
 hist[n]=lim;
 g.beginPath();let started=false;
 lim.forEach(pt=>{let x=p+(pt[0]-cut)/8000*(w-p-10),y=h-p-(pt[1]/max)*(h-p-8);
   if(!started){g.moveTo(x,y);started=true}else g.lineTo(x,y);
 });
 g.strokeStyle=n===0?'#66d7ee':'#f2ad88';g.lineWidth=2.5;g.stroke();
}
async function run(on){try{const r=await fetch('/api/hybrid/run?on='+(on?1:0),{cache:'no-store'});if(!r.ok)throw Error(r.status);$('err').textContent='';}catch(e){$('err').textContent='Error al cambiar adquisición: '+e;}}
let busy=false;
async function refresh(){
 if(busy)return;busy=true;
 try{
  const r=await fetch('/api/hybrid/status',{cache:'no-store'});if(!r.ok)throw Error('HTTP '+r.status);
  const j=await r.json();$('runState').textContent=j.running?'DETECCIÓN ACTIVA':'DETENIDA';
  [j.s1,j.s2].forEach((s,i)=>{let n=i+1;
    $('d'+n).textContent=s.valid?s.rawCm.toFixed(2)+' cm':'-- cm';
    $('hz'+n).textContent=s.hz.toFixed(1);$('valid'+n).textContent=s.seq;
    $('reject'+n).textContent=s.rejected;$('age'+n).textContent=s.lastValid?s.ageMs+' ms':'--';
    if(s.seq!==lastSeq[i]&&s.seq>0){lastSeq[i]=s.seq;hist[i].push([performance.now(),s.rawCm]);}
  });
  $('s1state').textContent=j.s1.valid?'LECTURA VÁLIDA':'SIN LECTURA FRESCA';
  $('s2state').textContent=j.tofReady?(j.s2.valid?'LECTURA VÁLIDA':'SENSOR ACTIVO · SIN DATO VÁLIDO'):'SIN INICIALIZACIÓN: '+j.tofInit;
  $('echoPulse').textContent=j.s1Diag.pulseUs;$('echoCm').textContent=j.s1Diag.pulseUs?j.s1Diag.echoCm.toFixed(1)+' cm':'--';$('echoLevel').textContent=j.s1Diag.echoBefore+' / '+j.s1Diag.echoAfter;$('echoNote').textContent=j.s1Diag.pulseUs?'Pulso ECHO recibido. Si está fuera de 50 cm aparecerá en Rechazadas, no en Timeouts.':(j.s1Diag.echoBefore?'GPIO14 ya está ALTO antes de TRIG: revisar ECHO y adaptador.':'GPIO14 sin pulso ECHO de hasta '+j.s1Diag.timeoutUs+' µs: revisar TRIG, alimentación, ECHO y adaptador.');$('timeout1').textContent=j.s1.timeouts;$('smooth1').textContent=j.s1.lastValid?j.s1.smoothCm.toFixed(2)+' cm':'--';
  $('status2').textContent=j.tofStatus;$('sig2').textContent=j.s2.signal.toFixed(1)+' / '+j.s2.ambient.toFixed(1);
  $('rate1').textContent=j.hcPeriodMs;
  draw(0,j.maxRangeCm);draw(1,j.maxRangeCm);$('err').textContent='';
 }catch(e){$('err').textContent='Error de telemetría: '+e}
 finally{busy=false;}
}
setInterval(refresh,80);refresh();
</script></body></html>
)PAGE";


const char HYBRID_LAB_PAGE[] PROGMEM=R"HYBRIDHTML(
<!doctype html><html lang="es"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>SONIC BONGO · Hybrid Function</title>
<style>
:root{color-scheme:dark;--bg:#070e14;--p:#101d29;--p2:#152635;--line:#2b4353;--fg:#ecf4fa;--muted:#98aaba;--u:#67d6e6;--t:#ffa18c;--y:#f9f9fb;--hit:#ffcc6f}*{box-sizing:border-box}body{background:var(--bg);color:var(--fg);font:14px system-ui,Segoe UI,sans-serif;margin:0}main,header{max-width:1100px;margin:auto;padding:14px}h1{font-size:23px;margin:0}.sub,.note{color:var(--muted);font-size:11px}.tabs{display:flex;gap:6px;margin:12px 0}.tabs button{flex:1}.tab{display:none}.tab.active{display:block}.card{background:var(--p);border:1px solid var(--line);border-radius:12px;padding:13px;margin-top:11px}.summary{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.metric{background:var(--p2);border:1px solid var(--line);border-radius:9px;padding:8px}.metric small{display:block;color:var(--muted);font-size:10px}.metric b{font-size:20px}.sensor{display:flex;align-items:center;gap:12px}.mini{width:76px;flex:0 0 76px}.mini svg{width:75px}.headline{font-size:23px;font-weight:800}canvas{display:block;background:#09131d;border:1px solid #1e3140;border-radius:7px;width:100%;height:auto;margin-top:9px}button{border:1px solid #4d6b7a;background:#1d3545;color:#f4f8fa;border-radius:8px;padding:10px 12px;font:700 12px system-ui;cursor:pointer}.activeBtn{border-color:#77b5e2;background:#224962}.actions{display:flex;flex-wrap:wrap;gap:8px;margin-top:10px}h2{font-size:16px;margin:0}.stats{display:grid;grid-template-columns:repeat(3,1fr);gap:7px;margin-top:9px}.stats>div{border:1px solid #2b4455;border-radius:8px;padding:7px}.stats small{display:block;color:var(--muted);font-size:10px}.stats b{font-size:14px}label{display:block;color:#b8cbda;font-size:12px;margin-top:11px}.lab{display:flex;justify-content:space-between}input[type=range]{display:block;width:100%;accent-color:#74b7f1}.status{font-size:12px;margin-top:7px;color:var(--hit)}a{color:#98cef3}.badge{font-size:10px;letter-spacing:.05em;color:#92d9be}@media(min-width:730px){.cols{display:grid;grid-template-columns:1fr 1fr;gap:10px}.summary{grid-template-columns:repeat(6,1fr)}}@media(max-width:450px){.metric b{font-size:17px}}
</style></head><body>
<header><h1>SONIC BONGO <span style="color:#7cadd8">HYBRID</span></h1><div class="sub">v19.1 · VL53L1X prioritario + HC-SR04 complementario · F1 / dataset</div>
<div class="actions"><button onclick="setRun(true)">INICIAR</button><button onclick="setRun(false)">DETENER</button><span id="run" class="badge">...</span></div>
<div class="tabs"><button id="tab0" class="activeBtn" onclick="tab(0)">SENSORES</button><button id="tab1" onclick="tab(1)">FUNCIÓN</button><button id="tab2" onclick="tab(2)">PRUEBAS / DATOS</button></div>
</header><main>
<section class="tab active" id="panel0">
<div class="card">
 <div class="sensor"><div class="mini"><svg viewBox="0 0 84 64"><rect x="2" y="10" width="80" height="43" rx="6" fill="#17435d" stroke="#70c8eb" stroke-width="2"/><circle cx="23" cy="31" r="15" fill="#09151a" stroke="#82b4c3" stroke-width="4"/><circle cx="61" cy="31" r="15" fill="#09151a" stroke="#82b4c3" stroke-width="4"/></svg></div><div><h2>S1 · HC-SR04</h2><div id="d1" class="headline" style="color:var(--u)">-- cm</div><div class="sub">Acústico · GPIO13/14</div></div></div><canvas id="c1" width="800" height="145"></canvas>
<div class="stats"><div><small>Hz reales</small><b id="hz1">--</b></div><div><small>Válidas</small><b id="ok1">--</b></div><div><small>Timeouts</small><b id="to1">--</b></div><div><small>Rechazadas</small><b id="rej1">--</b></div><div><small>RAW / estable</small><b id="raw1">--</b></div><div><small>Edad</small><b id="age1">--</b></div></div>
</div>
<div class="card">
 <div class="sensor"><div class="mini"><svg viewBox="0 0 84 64"><rect x="20" y="3" width="45" height="58" rx="6" fill="#8b3142" stroke="#ffa18c" stroke-width="2"/><rect x="32" y="17" width="22" height="29" rx="4" fill="#1e1820" stroke="#f2c0ac"/><rect x="38" y="21" width="10" height="21" rx="2" fill="#718991"/></svg></div><div><h2>S2 · VL53L1X</h2><div id="d2" class="headline" style="color:var(--t)">-- cm</div><div class="sub">Óptico ToF · I²C GPIO8/9</div></div></div><canvas id="c2" width="800" height="145"></canvas>
<div class="stats"><div><small>Hz reales</small><b id="hz2">--</b></div><div><small>Válidas</small><b id="ok2">--</b></div><div><small>Estado</small><b id="to2">--</b></div><div><small>Rechazadas</small><b id="rej2">--</b></div><div><small>RAW / estable</small><b id="raw2">--</b></div><div><small>Edad</small><b id="age2">--</b></div></div>
</div><div class="card note">La resolución numérica no representa la exactitud física. El HC-SR04 confirma presencia/altura aproximada y el ToF entrega mayor granularidad. Si las lecturas discrepan demasiado, se evita promediarlas.</div>
</section>
<section class="tab" id="panel1">
<div class="summary">
<div class="metric"><small>Y fusionada</small><b id="y">--</b></div><div class="metric"><small>V(t)</small><b id="v">--</b></div><div class="metric"><small>A(t)</small><b id="a">--</b></div><div class="metric"><small>Fase</small><b id="phase">--</b></div><div class="metric"><small>GOLPES F1</small><b id="hits">0</b></div><div class="metric"><small>Fuente / calidad</small><b id="source">--</b></div>
</div>
<div class="card"><h2>Altura Y(t) · datos reales</h2><canvas id="cy" width="900" height="190"></canvas><div class="note">Celeste = HC-SR04; naranja = VL53L1X; blanco = Y fusionada; marcas amarillas = candidatos F1.</div></div>
<div class="cols"><div class="card"><h2>Velocidad V(t) · cm/s</h2><canvas id="cv" width="900" height="185"></canvas></div><div class="card"><h2>Aceleración A(t) · cm/s²</h2><canvas id="ca" width="900" height="185"></canvas></div></div>
<div class="card"><h2>Ajustes de fusión</h2>
<label><span class="lab">Peso del ToF <b id="wval">85%</b></span><input id="w" type="range" min="50" max="100" step="5" value="85"></label>
<label><span class="lab">Coincidencia máxima de distancias <b id="agreeval">7 cm</b></span><input id="agree" type="range" min="1" max="20" step="1" value="7"></label>
<label><span class="lab">Offset óptico <b id="offsetval">0 cm</b></span><input id="offset" type="range" min="-10" max="10" step=".5" value="0"></label>
<label><span class="lab">Velocidad mínima descenso <b id="downval">30 cm/s</b></span><input id="down" type="range" min="10" max="150" step="5" value="30"></label>
<label><span class="lab">Amplitud mínima <b id="ampval">1.3 cm</b></span><input id="amp" type="range" min=".4" max="8" step=".1" value="1.3"></label>
<div class="actions"><button onclick="saveCfg()">GUARDAR Y APLICAR</button><button onclick="loadCfg()">CARGAR ACTUALES</button><button id="sound" onclick="toggleSound()">SONIDO HIT OFF</button><button onclick="testHit()">PROBAR SONIDO</button></div><p class="status" id="cfgStatus"></p>
</div></section>
<section class="tab" id="panel2"><div class="card"><h2>Metrónomo de entrenamiento</h2>
<p class="note">El click es referencia temporal. No se etiqueta automáticamente como “golpe humano real”; esos datos se guardan separados.</p>
<div class="metric" style="margin-top:10px"><small>BPM referencia</small><b id="bpmval">120</b></div>
<input type="range" id="bpm" min="40" max="280" step="5" value="120">
<div class="actions"><button onclick="startMetro()">INICIAR CLICK</button><button onclick="stopMetro()">DETENER CLICK</button><button onclick="testHit()">PROBAR HIT</button></div>
<p id="metroStatus" class="status">Inactivo</p></div>
<div class="card"><h2>Datos para aprendizaje</h2>
<div class="stats"><div><small>Muestras</small><b id="sampleCount">--</b></div><div><small>Gestos F1</small><b id="eventCount">--</b></div><div><small>Clicks referencia</small><b id="beatCount">--</b></div></div>
<div class="actions"><a href="/api/data/samples?n=1024">SAMPLES CSV</a><a href="/api/data/events?n=512">EVENTS CSV</a><a href="/api/data/configs?n=32">CONFIGS CSV</a><a href="/api/hybrid/beats.csv">METRÓNOMO CSV</a></div>
<p class="note">Se conservan los registros v17 con Y(t), V(t), A(t), S1/S2, baseline, calidad y candidatos. La red neuronal todavía NO está entrenando; el objetivo es recoger muestras válidas etiquetables.</p></div>
</section>
<p id="error" class="status"></p></main>
<script>
const $=s=>document.getElementById(s),vals={w:'wval',agree:'agreeval',offset:'offsetval',down:'downval',amp:'ampval',bpm:'bpmval'};
let current=null,activeTab=0,dirty=false,audio=null,soundOn=false,lastEvent=0,afterSeq=0,records=[],metroTimer=null,metroCount=0;
function tab(n){activeTab=n;for(let i=0;i<3;i++){$('panel'+i).classList.toggle('active',i===n);$('tab'+i).classList.toggle('activeBtn',i===n)}}
function fmt(n,dec=1){return Number(n).toFixed(dec)}
function labels(){Object.entries(vals).forEach(([k,v])=>$(v).textContent=$(k).value+(k==='w'?'%':k==='bpm'?'':k==='down'?' cm/s':k==='amp'||k==='offset'||k==='agree'?' cm':''));}
Object.keys(vals).forEach(k=>$(k).addEventListener('input',()=>{dirty=true;labels();$('cfgStatus').textContent='CAMBIOS PENDIENTES'}));
function syncCfg(c){$('w').value=c.tofWeight*100;$('agree').value=c.agreement;$('offset').value=c.offset;$('bpm').value=c.bpm;$('down').value=c.down;$('amp').value=c.amp;dirty=false;labels();}
async function loadCfg(){const j=await (await fetch('/api/hybrid/status',{cache:'no-store'})).json();syncCfg(j.cfgHybrid);$('cfgStatus').textContent='VALORES ACTUALES CARGADOS';}
async function saveCfg(){try{const q=new URLSearchParams({w:($('w').value/100),agree:$('agree').value,offset:$('offset').value,bpm:$('bpm').value,down:$('down').value,amp:$('amp').value});const r=await fetch('/api/hybrid/settings',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:q.toString()});let j=await r.json();if(!r.ok||!j.ok)throw Error(j.detail);dirty=false;$('cfgStatus').textContent='GUARDADO Y VERIFICADO · revisión '+j.revision;}catch(e){$('cfgStatus').textContent='ERROR DE GUARDADO: '+e}}
async function setRun(on){try{await fetch('/api/hybrid/run?on='+(on?'1':'0'),{cache:'no-store'})}catch(e){$('error').textContent=String(e)}}
async function ensureAudio(){if(!audio)audio=new(window.AudioContext||window.webkitAudioContext)();await audio.resume();}
function click(freq=720,dur=.052,gain=.16){if(!audio)return;let now=audio.currentTime,o=audio.createOscillator(),g=audio.createGain();o.type='triangle';o.frequency.value=freq;g.gain.setValueAtTime(.0001,now);g.gain.exponentialRampToValueAtTime(gain,now+.002);g.gain.exponentialRampToValueAtTime(.0001,now+dur);o.connect(g);g.connect(audio.destination);o.start(now);o.stop(now+dur+.002)}
async function toggleSound(){await ensureAudio();soundOn=!soundOn;$('sound').textContent='SONIDO HIT '+(soundOn?'ON':'OFF');if(soundOn)click(820)}
async function testHit(){await ensureAudio();click(1100,.075)}
async function startMetro(){await ensureAudio();stopMetro();metroCount=0;const bpm=Number($('bpm').value);let expected=performance.now();function beat(){click((metroCount%4===0)?1250:800,.035,.15);const index=metroCount++;fetch('/api/hybrid/beat',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({i:index,bpm,client:performance.now()})}).catch(()=>{});$('metroStatus').textContent='CLICK '+bpm+' BPM · beat '+metroCount;expected+=60000/bpm;metroTimer=setTimeout(beat,Math.max(0,expected-performance.now()));}beat();}
function stopMetro(){if(metroTimer!==null)clearTimeout(metroTimer);metroTimer=null;$('metroStatus').textContent='Detenido';}
function axes(g,w,h,min,max){g.clearRect(0,0,w,h);g.strokeStyle='#1d3445';g.fillStyle='#748b9b';g.font='11px system-ui';for(let i=0;i<=4;i++){let y=8+i*(h-22)/4;g.beginPath();g.moveTo(33,y);g.lineTo(w-6,y);g.stroke();g.fillText(fmt(max-(max-min)*i/4,0),2,y+3)}}
function plot(id,series,min,max,events=false){let c=$(id),g=c.getContext('2d'),w=c.width,h=c.height,now=records.length?records[records.length-1][1]:0,begin=now-8500;axes(g,w,h,min,max);for(const [index,color] of series){g.beginPath();let started=false;records.forEach(r=>{if(r[1]<begin)return;let val=r[index];if(val===null||!Number.isFinite(val)){started=false;return}let x=33+(r[1]-begin)/8500*(w-43),y=8+(max-val)/(max-min)*(h-22);if(!started){g.moveTo(x,y);started=true}else g.lineTo(x,y)});g.strokeStyle=color;g.lineWidth=index===2?2.4:1.45;g.stroke();}
if(events){g.fillStyle='#ffcc6f';for(let i=1;i<records.length;i++){let r=records[i];if(r[1]<begin||r[7]===records[i-1][7])continue;let x=33+(r[1]-begin)/8500*(w-43),y=8+(max-r[2])/(max-min)*(h-22);g.beginPath();g.arc(x,y,4,0,Math.PI*2);g.fill();}}}
function redraw(){if(!records.length)return;const range=Math.max(50,Number(current?.maxRangeCm||50));plot('c1',[[5,'#67d6e6']],0,range);plot('c2',[[6,'#ffa18c']],0,range);plot('cy',[[5,'#3b8b9a'],[6,'#c87566'],[2,'#ffffff']],0,range,true);plot('cv',[[3,'#67d6c3']],-220,220);plot('ca',[[4,'#c7a1ff']],-5000,5000);}
async function getTrace(){try{let r=await fetch('/api/hybrid/trace?after='+afterSeq,{cache:'no-store'});if(!r.ok)return;let j=await r.json();j.points.forEach(p=>{if(p[0]<=afterSeq)return;records.push(p);afterSeq=p[0]});if(records.length>550)records=records.slice(-550);if(activeTab===0||activeTab===1)redraw()}catch(e){}}
async function getMeta(){try{const d=await (await fetch('/api/hybrid/data',{cache:'no-store'})).json();$('sampleCount').textContent=d.sampleCount;$('eventCount').textContent=d.eventCount;$('beatCount').textContent=d.beatRefCount}catch(e){}}
let polling=false;
async function refresh(){if(polling)return;polling=true;try{let r=await fetch('/api/hybrid/status',{cache:'no-store'}),j=await r.json();current=j;$('run').textContent=j.running?'ADQUISICIÓN ACTIVA':'DETENIDA';
[j.s1,j.s2].forEach((q,i)=>{const n=i+1;$('d'+n).textContent=q.valid?fmt(q.rawCm,2)+' cm':'-- cm';$('hz'+n).textContent=fmt(q.hz);$('ok'+n).textContent=q.seq;$('rej'+n).textContent=q.rejected;$('age'+n).textContent=q.lastValid?q.ageMs+' ms':'--';$('raw'+n).textContent=(q.lastValid?fmt(q.rawCm,1)+' / '+fmt(q.smoothCm,1):'--');});
$('to1').textContent=j.s1.timeouts;$('to2').textContent=j.tofStatus;
let f=j.fusion;$('y').textContent=f.valid?fmt(f.y,2)+' cm':'--';$('v').textContent=fmt(f.velocity,0)+' cm/s';$('a').textContent=fmt(f.acceleration,0)+' cm/s²';$('phase').textContent=f.phase;$('hits').textContent=f.hits;$('source').textContent=(f.sourceMask===3?'AMBOS':f.sourceMask===2?'ToF':f.sourceMask===1?'US':'--')+' / '+fmt(f.quality,0)+'%';
if(!dirty && !window.cfgLoaded){syncCfg(j.cfgHybrid);window.cfgLoaded=true;}
if(lastEvent===0){lastEvent=f.eventSeq;}else if(f.eventSeq!==lastEvent){lastEvent=f.eventSeq;if(soundOn)click(520+f.midi*3,.055);}
$('error').textContent='';
}catch(e){$('error').textContent='Sin telemetría: '+e}finally{polling=false;}}
setInterval(refresh,120);setInterval(getTrace,130);setInterval(getMeta,2000);
refresh();getTrace();getMeta();labels();
</script></body></html>
)HYBRIDHTML";

void setup() {
 server.on("/",HTTP_GET,[](){server.sendHeader("Location","/hybrid");server.send(302,"text/plain","");});
 server.on("/hybrid",HTTP_GET,[](){server.send_P(200,"text/html",HYBRID_LAB_PAGE);});
 server.on("/api/hybrid/status",HTTP_GET,hybridStatus);
 server.on("/api/hybrid/run",HTTP_GET,hybridRun);
 server.on("/api/hybrid/trace",HTTP_GET,hybridTrace);
 server.on("/api/hybrid/settings",HTTP_POST,handleHybridSettings);
 server.on("/api/hybrid/beat",HTTP_POST,appendBeatReference);
 server.on("/api/hybrid/beats.csv",HTTP_GET,exportBeatRefs);
 server.on("/api/hybrid/data",HTTP_GET,handleHybridData);
 setup_v18();
 // Previous HC-SR04 S2 is disconnected, never issue GPIO15 trigger.
 pinMode(15,INPUT);pinMode(16,INPUT);
 Wire.begin(TOF_SDA,TOF_SCL);
 Wire.setClock(400000);
 initTof();
 loadHybridConfig();
 lastS1At=millis();
 lastHybridEventSeq=motion.eventSeq;
}
void loop() {
 // Deliberately do not call loop_v18, since it would ping the removed sensor S2.
 server.handleClient();
 serviceHybrid();
 delay(1);
}
