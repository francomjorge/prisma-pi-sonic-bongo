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

static const char* HYBRID_VER = "20.0-tof-only-lab";
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


// This lab deliberately ignores HC-SR04: no pulseIn, no GPIO13 trigger.
// ToF data are time-stamped; invalid reads are never 0 cm.
struct TofLab {
 uint32_t attempts=0,valid=0,reject=0,busErrors=0,seq=0,lastAt=0,lastAnyAt=0;
 uint32_t previous=0,rateAt=0;
 float hz=0,intervalMs=0,intervalMin=99999,intervalMax=0,jitterMs=0;
 float raw=0,filtered=0,vel=0,acc=0,oldV=0;
 float minCm=99999,maxCm=-99999,signal=0,ambient=0,noise=0;
 bool have=false,currentlyValid=false,descending=false,hit=false;
 uint8_t lastRangeStatus=255;
 uint32_t lastHitAt=0,hitSeq=0,hits=0,descentAt=0;
 float descentStart=0,descentMin=99999,descentSpeed=0;
 String phase="ESPERA";
};
TofLab lab;
struct TofSample {uint32_t seq,ms,hitSeq;float raw,y,v,a;uint8_t quality;};
static const int HISTORY=500;
TofSample history[HISTORY];
uint16_t histNext=0,histSize=0;
struct HumanLabel {uint32_t at,seq,hitSeq;uint8_t label;};
HumanLabel labels[128];uint16_t labelNext=0,labelCount=0;
float filterAlpha=0.55f,hitSpeedCmS=28.0f,hitAmpCm=1.2f,reversalCmS=3.0f;
uint32_t hitGapMs=170;
bool captureEnabled=true;
void loadLabConfig(){
 prefs.begin("tofLab20",true);
 filterAlpha=clampf(prefs.getFloat("alpha",.55f),.08f,1);
 hitSpeedCmS=clampf(prefs.getFloat("speed",28),3,200);
 hitAmpCm=clampf(prefs.getFloat("amp",1.2f),.2f,15);
 reversalCmS=clampf(prefs.getFloat("rev",3),0,60);
 hitGapMs=clampu(prefs.getUInt("gap",170),60,800);
 prefs.end();
}
void writeHistory(){
 TofSample a={lab.seq,millis(),lab.hitSeq,lab.raw,lab.filtered,lab.vel,lab.acc,lab.lastRangeStatus};
 history[histNext]=a;histNext=(histNext+1)%HISTORY;
 if(histSize<HISTORY)histSize++;
}
void feedTof(float cm,uint8_t quality,float signal,float ambient){
 uint32_t now=millis(),dt=lab.have?now-lab.lastAt:0;
 lab.raw=cm;lab.signal=signal;lab.ambient=ambient;lab.lastRangeStatus=quality;
 if(!lab.have || dt>500){
   lab.filtered=cm;lab.vel=0;lab.acc=0;lab.descending=false;
   lab.phase="ESPERA";
 }else{
   float oldY=lab.filtered;
   lab.filtered=filterAlpha*cm+(1-filterAlpha)*lab.filtered;
   lab.noise=.92f*lab.noise+.08f*fabsf(cm-lab.filtered);
   if(dt>=10&&dt<=350){
     float instV=clampf((lab.filtered-oldY)*1000.0f/dt,-500,500);
     float newV=.55f*instV+.45f*lab.vel;
     float instA=clampf((newV-lab.vel)*1000.0f/dt,-16000,16000);
     lab.acc=.35f*instA+.65f*lab.acc;lab.vel=newV;
   }
   if(!lab.descending){
     lab.phase=lab.vel>reversalCmS?"SUBIENDO":lab.vel<-hitSpeedCmS?"DESCENSO":"ESPERA";
     if(lab.vel<-hitSpeedCmS){
       lab.descending=true;lab.descentStart=oldY;
       lab.descentMin=lab.filtered;lab.descentSpeed=fabsf(lab.vel);lab.descentAt=now;
     }
   }else{
     lab.phase="DESCENSO";
     if(lab.filtered<lab.descentMin)lab.descentMin=lab.filtered;
     if(-lab.vel>lab.descentSpeed)lab.descentSpeed=-lab.vel;
     float amplitude=lab.descentStart-lab.descentMin;
     bool reversing=lab.vel>=reversalCmS || lab.filtered-lab.descentMin>=.35f;
     if(reversing || now-lab.descentAt>850){
       if(reversing && amplitude>=hitAmpCm &&
         lab.descentSpeed>=hitSpeedCmS &&
         (!lab.lastHitAt || now-lab.lastHitAt>=hitGapMs)){
           lab.hitSeq++;lab.hits++;lab.lastHitAt=now;lab.phase="HIT";
       }else lab.phase="REBOTE";
       lab.descending=false;
     }
   }
 }
 if(lab.have&&dt>0){
   lab.intervalMs=dt;
   if(dt<lab.intervalMin)lab.intervalMin=dt;
   if(dt>lab.intervalMax)lab.intervalMax=dt;
   lab.jitterMs=.85f*lab.jitterMs+.15f*fabsf(dt-35.0f);
 }
 lab.have=true;lab.currentlyValid=true;lab.lastAt=now;lab.valid++;lab.seq++;
 if(cm<lab.minCm)lab.minCm=cm;if(cm>lab.maxCm)lab.maxCm=cm;
 writeHistory();
}
void sampleLab(){
 if(!captureEnabled||!tofReady)return;
 if(!tof.dataReady()){
   if(tof.last_status!=0){tofReady=false;tofInitStatus="I2C_ERROR";}
   return;
 }
 uint16_t mm=tof.read(false);
 lab.attempts++;lab.lastAnyAt=millis();
 if(tof.timeoutOccurred() || tof.last_status!=0){
  lab.busErrors++;lab.currentlyValid=false;return;
 }
 uint8_t q=(uint8_t)tof.ranging_data.range_status;
 lab.lastRangeStatus=q;
 float cm=mm*.1f;
 if(q!=VL53L1X::RangeValid || cm<4 || cm>maxRangeCm){
   lab.reject++;lab.currentlyValid=false;return;
 }
 feedTof(cm,q,tof.ranging_data.peak_signal_count_rate_MCPS,
                  tof.ranging_data.ambient_count_rate_MCPS);
}
void tickLab(){
 uint32_t now=millis();
 if(!tofReady && now-lastTofInitAt>5000)initTof();
 sampleLab();
 if(lab.rateAt==0){lab.rateAt=now;lab.previous=lab.seq;}
 else if(now-lab.rateAt>=1000){
   float sec=(now-lab.rateAt)/1000.0f;
   lab.hz=(lab.seq-lab.previous)/sec;
   lab.previous=lab.seq;lab.rateAt=now;
 }
}
void tofJson(){
 uint32_t now=millis();
 uint32_t age=lab.have?now-lab.lastAt:999999;
 bool fresh=lab.have&&tofReady&&age<160;
 String j="{\"fw\":\"20.0-tof-only-lab\",\"running\":"+String(captureEnabled?"true":"false")+
 ",\"tofReady\":"+String(tofReady?"true":"false")+
 ",\"tofInit\":\""+tofInitStatus+"\",\"hasReading\":"+String(lab.have?"true":"false")+
 ",\"fresh\":"+String(fresh?"true":"false")+
 ",\"rawCm\":"+String(lab.raw,2)+",\"yCm\":"+String(lab.filtered,2)+
 ",\"vCmS\":"+String(lab.vel,1)+",\"aCmS2\":"+String(lab.acc,0)+
 ",\"ageMs\":"+String(age)+",\"seq\":"+String(lab.seq)+
 ",\"hz\":"+String(lab.hz,1)+",\"intervalMs\":"+String(lab.intervalMs,1)+
 ",\"minIntervalMs\":"+String(lab.intervalMin==99999?0:lab.intervalMin,1)+
 ",\"maxIntervalMs\":"+String(lab.intervalMax,1)+",\"jitterMs\":"+String(lab.jitterMs,1)+
 ",\"attempts\":"+String(lab.attempts)+",\"valid\":"+String(lab.valid)+
 ",\"rejected\":"+String(lab.reject)+",\"busErrors\":"+String(lab.busErrors)+
 ",\"status\":"+String(lab.lastRangeStatus)+",\"signal\":"+String(lab.signal,2)+
 ",\"ambient\":"+String(lab.ambient,2)+",\"noise\":"+String(lab.noise,2)+
 ",\"minCm\":"+String(lab.minCm==99999?0:lab.minCm,2)+
 ",\"maxCm\":"+String(lab.maxCm==-99999?0:lab.maxCm,2)+
 ",\"hits\":"+String(lab.hits)+",\"hitSeq\":"+String(lab.hitSeq)+
 ",\"phase\":\""+lab.phase+"\",\"labels\":"+String(labelCount)+
 ",\"alpha\":"+String(filterAlpha,2)+",\"speed\":"+String(hitSpeedCmS,1)+
 ",\"amplitude\":"+String(hitAmpCm,2)+",\"gapMs\":"+String(hitGapMs)+"}";
 server.sendHeader("Cache-Control","no-store");server.send(200,"application/json",j);
}
void traceJson(){
 uint32_t after=server.hasArg("after")?strtoul(server.arg("after").c_str(),nullptr,10):0;
 String j="{\"samples\":[";
 bool first=true;int sent=0;
 for(int i=0;i<histSize;i++){
   TofSample &a=history[(histNext+HISTORY-histSize+i)%HISTORY];
   if(a.seq<=after)continue;
   if(sent++>=65)break;
   if(!first)j+=",";first=false;
   j+="["+String(a.seq)+","+String(a.ms)+","+String(a.raw,2)+
     ","+String(a.y,2)+","+String(a.v,1)+","+String(a.a,0)+
     ","+String(a.hitSeq)+","+String(a.quality)+"]";
 }
 j+="]}";server.sendHeader("Cache-Control","no-store");
 server.send(200,"application/json",j);
}
void configureLab(){
 if(server.hasArg("alpha"))filterAlpha=clampf(server.arg("alpha").toFloat(),.08f,1);
 if(server.hasArg("speed"))hitSpeedCmS=clampf(server.arg("speed").toFloat(),3,200);
 if(server.hasArg("amp"))hitAmpCm=clampf(server.arg("amp").toFloat(),.2f,15);
 if(server.hasArg("gap"))hitGapMs=clampu(server.arg("gap").toInt(),60,800);
 prefs.begin("tofLab20",false);
 prefs.putFloat("alpha",filterAlpha);prefs.putFloat("speed",hitSpeedCmS);
 prefs.putFloat("amp",hitAmpCm);prefs.putUInt("gap",hitGapMs);prefs.end();
 prefs.begin("tofLab20",true);
 bool ok=fabsf(prefs.getFloat("alpha",-1)-filterAlpha)<.001f &&
 fabsf(prefs.getFloat("speed",-1)-hitSpeedCmS)<.01f &&
 fabsf(prefs.getFloat("amp",-1)-hitAmpCm)<.01f &&
 prefs.getUInt("gap",0)==hitGapMs;prefs.end();
 server.send(ok?200:500,"application/json",String("{\"ok\":")+(ok?"true":"false")+"}");
}
void humanLabel(){
 if(!server.hasArg("hit")){server.send(400,"text/plain","hit required");return;}
 HumanLabel a={millis(),lab.seq,lab.hitSeq,(uint8_t)(server.arg("hit")=="1"?1:0)};
 labels[labelNext]=a;labelNext=(labelNext+1)%128;if(labelCount<128)labelCount++;
 server.send(200,"application/json","{\"ok\":true}");
}
void exportTrace(){
 server.setContentLength(CONTENT_LENGTH_UNKNOWN);server.send(200,"text/csv","");
 server.sendContent("seq,esp_ms,raw_cm,y_cm,velocity_cm_s,accel_cm_s2,hit_seq,range_status\n");
 for(int i=0;i<histSize;i++){
  TofSample &a=history[(histNext+HISTORY-histSize+i)%HISTORY];
  server.sendContent(String(a.seq)+","+a.ms+","+String(a.raw,2)+","+
   String(a.y,2)+","+String(a.v,1)+","+String(a.a,0)+","+
   String(a.hitSeq)+","+String(a.quality)+"\n");
 }
 server.sendContent("");
}
void exportHuman(){
 server.setContentLength(CONTENT_LENGTH_UNKNOWN);server.send(200,"text/csv","");
 server.sendContent("esp_ms,point_seq,auto_hit_seq,human_label\n");
 for(int i=0;i<labelCount;i++){
  HumanLabel &a=labels[(labelNext+128-labelCount+i)%128];
  server.sendContent(String(a.at)+","+a.seq+","+a.hitSeq+","+a.label+"\n");
 }
 server.sendContent("");
}

const char LAB_PAGE[] PROGMEM=R"TOFHTML(
<!DOCTYPE html><html lang="es"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>SONIC BONGO · ToF LAB</title><style>
:root{color-scheme:dark;--bg:#070d14;--panel:#142331;--border:#31536a;--text:#eaf3fa;--muted:#a8c0cd;--good:#88d8b6;--bad:#f5ae86}
*{box-sizing:border-box}body{background:var(--bg);color:var(--text);font:15px system-ui,Arial;margin:0}
header,main{max-width:1000px;margin:0 auto;padding:16px}.top{display:flex;justify-content:space-between;align-items:center}h1{font-size:24px;margin:0}h2{font-size:17px;margin:0}.sub{font-size:12px;color:var(--muted)}
.card{background:var(--panel);border:1px solid var(--border);border-radius:14px;padding:15px;margin-bottom:12px}
.number{font-size:38px;font-weight:800;color:var(--good)}button{border:1px solid #6589a3;border-radius:9px;padding:11px;background:#213c51;color:white;font-weight:750;cursor:pointer}
button:active{background:#39627f}.row{display:flex;flex-wrap:wrap;gap:9px;margin:12px 0}.grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px}
.cell{border:1px solid #2b4a5d;padding:9px;border-radius:8px}.cell small{display:block;color:var(--muted);font-size:10px}.cell b{font-size:17px}
canvas{display:block;width:100%;height:auto;background:#07131c;border:1px solid #244253;border-radius:8px;margin:10px 0}
label{display:block;margin:15px 0;color:var(--muted)}input[type=range]{width:100%;accent-color:#87c4ed}.alert{color:#ffbf89;font-size:12px}a{color:#b1dcf8;margin-right:16px}
@media(min-width:700px){.grid{grid-template-columns:repeat(6,minmax(0,1fr))}}
</style></head><body><header><div class="top"><div><h1>SONIC BONGO · ToF LAB</h1><div class="sub">v20.0 · Sensor óptico VL53L1X · análisis y sonido celular</div></div><div id="conn" class="sub">CONECTANDO</div></div></header>
<main><div class="card">
<h2>Sensor óptico · medición real</h2><div class="number" id="distance">-- cm</div><div class="sub" id="freshness">SIN MUESTRAS</div>
<canvas id="heightGraph" width="900" height="230"></canvas>
<div class="grid">
<div class="cell"><small>Velocidad</small><b id="speedVal">--</b></div>
<div class="cell"><small>Aceleración</small><b id="accelVal">--</b></div>
<div class="cell"><small>Hz válidos</small><b id="hzVal">--</b></div>
<div class="cell"><small>Intervalo</small><b id="intervalVal">--</b></div>
<div class="cell"><small>Jitter</small><b id="jitterVal">--</b></div>
<div class="cell"><small>Ruido (estimado)</small><b id="noiseVal">--</b></div>
<div class="cell"><small>Rango medido</small><b id="rangeVal">--</b></div>
<div class="cell"><small>Muestras válidas</small><b id="validVal">0</b></div>
<div class="cell"><small>Rechazadas</small><b id="rejectedVal">0</b></div>
<div class="cell"><small>Estado VL53</small><b id="statusVal">--</b></div>
<div class="cell"><small>Señal/ambiente</small><b id="signalVal">--</b></div>
<div class="cell"><small>Fallos I²C</small><b id="busVal">0</b></div>
</div></div>
<div class="card"><div class="top"><h2>Función · golpe</h2><div id="hitCount" class="number">0</div></div>
<div id="phase" class="sub">ESPERA</div>
<canvas id="velocityGraph" width="900" height="145"></canvas>
<canvas id="accelGraph" width="900" height="145"></canvas>
<div class="row"><button id="audioBtn" onclick="unlockAudio()">ACTIVAR AUDIO DEL CELULAR</button><button onclick="manualSound()">PROBAR SONIDO</button></div>
<div class="sub" id="audioState">Audio desactivado. Tocá ACTIVAR AUDIO primero.</div>
<label>Suavizado del ToF <b id="alphaVal">0.55</b><input id="alpha" type="range" min=".08" max="1" step=".01" value=".55"></label>
<label>Velocidad mínima de descenso <b id="speedCfg">28 cm/s</b><input id="speed" type="range" min="3" max="200" step="1" value="28"></label>
<label>Amplitud mínima <b id="ampCfg">1.2 cm</b><input id="amp" type="range" min=".2" max="15" step=".1" value="1.2"></label>
<label>Intervalo mínimo entre golpes <b id="gapCfg">170 ms</b><input id="gap" type="range" min="60" max="800" step="10" value="170"></label>
<div class="row"><button onclick="save()">GUARDAR PARÁMETROS</button><button onclick="labelHit(1)">MARCAR GOLPE REAL</button><button onclick="labelHit(0)">MARCAR FALSO GOLPE</button></div>
<div id="message" class="alert"></div></div>
<div class="card"><h2>Datos y entrenamiento</h2><p class="sub">Las etiquetas manuales se guardan separadas de los eventos automáticos. Todavía no se entrena una red neuronal.</p>
<div class="row"><a href="/api/tof/samples.csv">Exportar muestras CSV</a><a href="/api/tof/labels.csv">Exportar etiquetas CSV</a><a href="/api/tof/status">Ver JSON de diagnóstico</a></div>
<div class="sub">HC-SR04 deshabilitado en este firmware. No es necesario conectarlo para probar el ToF.</div></div>
</main><script>
const $=id=>document.getElementById(id);
let audio=null,audioOn=false,points=[],lastSeq=0,lastEvent=null,configLoaded=false,editing=false,busy=false,traceBusy=false;
const configFields=['alpha','speed','amp','gap'];
function labels(){configFields.forEach(k=>{let val=$(k).value;$(k==='alpha'?'alphaVal':k==='speed'?'speedCfg':k==='amp'?'ampCfg':'gapCfg').textContent=val+(k==='speed'?' cm/s':k==='amp'?' cm':k==='gap'?' ms':'')})}
configFields.forEach(k=>$(k).addEventListener('input',()=>{editing=true;labels();}));
function beep(hz=880,dur=.065){if(!audio||audio.state!=='running')return;
let t=audio.currentTime,osc=audio.createOscillator(),g=audio.createGain();
osc.type='triangle';osc.frequency.setValueAtTime(hz,t);g.gain.setValueAtTime(.001,t);
g.gain.exponentialRampToValueAtTime(.24,t+.004);g.gain.exponentialRampToValueAtTime(.001,t+dur);
osc.connect(g).connect(audio.destination);osc.start(t);osc.stop(t+dur+.008)}
async function unlockAudio(){try{
audio=audio||new(window.AudioContext||window.webkitAudioContext)();
await audio.resume();audioOn=audio.state==='running';if(audioOn)beep(950,.13);
$('audioBtn').textContent=audioOn?'AUDIO ACTIVADO':'REINTENTAR AUDIO';
$('audioState').textContent=audioOn?'Audio habilitado · escuchaste un sonido de prueba':'Audio suspendido por el navegador';
}catch(e){$('audioState').textContent='Error de audio: '+e;}}
async function manualSound(){if(!audioOn)await unlockAudio();else beep(1100,.1);}
async function save(){let b=new URLSearchParams();configFields.forEach(k=>b.set(k,$(k).value));
try{let r=await fetch('/api/tof/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:b});let j=await r.json();
if(!r.ok||!j.ok)throw Error('no verificado');editing=false;$('message').textContent='Configuración guardada';
}catch(e){$('message').textContent='Error al guardar: '+e}}
async function labelHit(hit){try{let r=await fetch('/api/tof/label',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'hit='+hit});if(!r.ok)throw Error(r.status);$('message').textContent=hit?'Golpe real etiquetado':'Falso golpe etiquetado'}catch(e){$('message').textContent=String(e)}}
function graph(id,index,min,max,colors){const c=$(id),g=c.getContext('2d');g.clearRect(0,0,c.width,c.height);
let w=c.width,h=c.height,p=27,end=points.length?points[points.length-1][1]:0,begin=end-8500;
g.strokeStyle='#244052';g.fillStyle='#7794a5';g.font='11px system-ui';
for(let i=0;i<=4;i++){let y=8+i*(h-24)/4;g.beginPath();g.moveTo(p,y);g.lineTo(w-8,y);g.stroke();g.fillText(Math.round(max-i*(max-min)/4),2,y+4);}
for(let line=0;line<index.length;line++){g.beginPath();let started=false;for(const r of points){if(r[1]<begin)continue;let val=r[index[line]],x=p+(r[1]-begin)/8500*(w-p-9),y=8+(max-val)/(max-min)*(h-24);
if(!started){g.moveTo(x,y);started=true}else g.lineTo(x,y);}
g.strokeStyle=colors[line];g.lineWidth=line===1?2.5:1.5;g.stroke();}}
function redraw(){if(!points.length)return;let a=points.slice(-300),values=a.map(x=>x[2]);
let lo=Math.max(0,Math.min(...values)-3),hi=Math.max(lo+15,Math.max(...values)+3);
graph('heightGraph',[2,3],lo,hi,['#618ba4','#f8ead2']);
graph('velocityGraph',[4],-250,250,['#65c7bc']);graph('accelGraph',[5],-6000,6000,['#bba3f3']);}
async function updateTrace(){if(traceBusy)return;traceBusy=true;try{
let r=await fetch('/api/tof/trace?after='+lastSeq,{cache:'no-store'});if(!r.ok)throw Error(r.status);
let j=await r.json();j.samples.forEach(s=>{if(s[0]>lastSeq){points.push(s);lastSeq=s[0];}});
if(points.length>550)points=points.slice(-550);redraw();
}catch(e){}finally{traceBusy=false}}
async function update(){if(busy)return;busy=true;try{
let r=await fetch('/api/tof/status',{cache:'no-store'});if(!r.ok)throw Error('HTTP '+r.status);
let d=await r.json();$('conn').textContent=d.tofReady?'TOF ACTIVO':'TOF '+d.tofInit;
$('distance').textContent=d.hasReading?d.rawCm.toFixed(2)+' cm'+(d.fresh?'':' · última'):'-- cm';
$('freshness').textContent=d.hasReading?'Edad de la medición: '+d.ageMs+' ms'+(d.fresh?' · VÁLIDA':' · NO RECIENTE'):'SIN LECTURAS VÁLIDAS';
$('speedVal').textContent=d.vCmS.toFixed(1)+' cm/s';$('accelVal').textContent=d.aCmS2.toFixed(0)+' cm/s²';
$('hzVal').textContent=d.hz.toFixed(1);$('intervalVal').textContent=d.intervalMs.toFixed(1)+' ms';
$('jitterVal').textContent=d.jitterMs.toFixed(1)+' ms';$('noiseVal').textContent=d.noise.toFixed(2)+' cm';
$('rangeVal').textContent=d.hasReading?d.minCm.toFixed(1)+'–'+d.maxCm.toFixed(1)+' cm':'--';
$('validVal').textContent=d.valid;$('rejectedVal').textContent=d.rejected;
$('statusVal').textContent=d.status;$('signalVal').textContent=d.signal.toFixed(1)+' / '+d.ambient.toFixed(1);
$('busVal').textContent=d.busErrors;$('hitCount').textContent=d.hits;$('phase').textContent=d.phase;
if(lastEvent===null)lastEvent=d.hitSeq;
else if(lastEvent!==d.hitSeq){lastEvent=d.hitSeq;if(audioOn)beep(680,.07);}
if(!configLoaded&&!editing){configFields.forEach(k=>$(k).value=k==='speed'?d.speed:k==='amp'?d.amplitude:k==='gap'?d.gapMs:d.alpha);labels();configLoaded=true;}
$('message').textContent=$('message').textContent==='Sin telemetría'?'':'';
}catch(e){$('conn').textContent='ERROR';$('message').textContent='Sin telemetría: '+e;}finally{busy=false}}
setInterval(update,130);setInterval(updateTrace,150);update();updateTrace();labels();
</script></body></html>
)TOFHTML";

void setup(){
 server.on("/",HTTP_GET,[](){server.sendHeader("Location","/tof");server.send(302,"text/plain","");});
 server.on("/tof",HTTP_GET,[](){server.send_P(200,"text/html",LAB_PAGE);});
 server.on("/api/tof/status",HTTP_GET,tofJson);
 server.on("/api/tof/trace",HTTP_GET,traceJson);
 server.on("/api/tof/config",HTTP_POST,configureLab);
 server.on("/api/tof/label",HTTP_POST,humanLabel);
 server.on("/api/tof/samples.csv",HTTP_GET,exportTrace);
 server.on("/api/tof/labels.csv",HTTP_GET,exportHuman);
 setup_v18();
 // No HC-SR04 activity, no TRIG or ECHO consumption.
 digitalWrite(PINS[0].trig,LOW);
 pinMode(14,INPUT);
 pinMode(15,INPUT);pinMode(16,INPUT);
 Wire.begin(TOF_SDA,TOF_SCL);Wire.setClock(400000);
 loadLabConfig();
 initTof();
}
void loop(){
 server.handleClient();
 tickLab();
 delay(1);
}
