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

static const char* HYBRID_VER = "19.0.2-isolated-gpio14-diagnostic";
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


// GPIO14 isolated-input diagnostic (not a voltage measurement).
// REQUIRES PHYSICAL DISCONNECT TXS A1 -> ESP32 GPIO14.
// Never switches this pin to OUTPUT or sends any voltage.
uint16_t diagReadOnes() {
 uint16_t n=0;
 for(int i=0;i<40;i++){if(digitalRead(14)==HIGH)n++;delayMicroseconds(100);}
 return n;
}
void handleGpio14Test(){
 if(!server.hasArg("isolated")||server.arg("isolated")!="YES"){
  server.send(400,"application/json","{\"ok\":false,\"error\":\"Disconnect A1-GPIO14 with power OFF first, then confirm isolated=YES\"}");
  return;
 }
 bool was=hybridRunning;
 hybridRunning=false;
 digitalWrite(PINS[0].trig,LOW); // GPIO13 safe inactive, no new ping
 pinMode(14,INPUT_PULLDOWN);delay(3);uint16_t down=diagReadOnes();
 pinMode(14,INPUT_PULLUP);delay(3);uint16_t up=diagReadOnes();
 pinMode(14,INPUT);delay(3);uint16_t floating=diagReadOnes();
 hybridRunning=was;
 String j="{\"ok\":true,\"gpio\":14,\"samples\":40,\"pulldownHigh\":"+String(down)+
   ",\"pullupHigh\":"+String(up)+",\"floatingHigh\":"+String(floating)+
   ",\"hint\":\"isolated_input_only_not_voltage_meter\"}";
 server.sendHeader("Cache-Control","no-store");
 server.send(200,"application/json",j);
}
const char GPIO14_DIAG_PAGE[] PROGMEM=R"PAGE14(
<!doctype html><html lang="es"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>SONIC BONGO - GPIO14</title><style>body{background:#0b1018;color:#edf4f8;font:16px system-ui;max-width:700px;padding:24px;margin:auto}
article{border:1px solid #557084;padding:18px;border-radius:14px}button{font-size:17px;padding:12px;border-radius:9px;background:#254964;color:white;border:1px solid #77a8c1}
pre{white-space:pre-wrap;overflow-wrap:anywhere}strong{color:#ffca88}</style></head><body>
<article><h2>GPIO14: prueba aislada</h2>
<p><strong>ANTES de usar:</strong> cortar TODA alimentación, desconectar SOLO A1 del TXS0108E hacia GPIO14.
El ECHO del HC-SR04 puede permanecer desconectado de B1. No tocar SDA/SCL ni el ToF.</p>
<p>Volver a energizar. Esta prueba aplica sólo resistencias internas débiles de pull-down y pull-up a GPIO14.
Nunca configura GPIO14 como salida. No mide voltajes.</p>
<button onclick="run()">PROBAR GPIO14 AISLADO</button><pre id="out">Sin prueba.</pre>
<p>Valor esperable con GPIO14 aislado: pulldownHigh cerca de 0 / 40 y pullupHigh cerca de 40 / 40.</p>
<p><a href="/hybrid" style="color:#a5d8f8">Regresar a S1/S2</a></p></article>
<script>
async function run(){let o=document.getElementById('out');o.textContent='Ejecutando...';
try{let r=await fetch('/api/test_gpio14',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'isolated=YES'});
o.textContent=JSON.stringify(await r.json(),null,2)}
catch(e){o.textContent=String(e)}}
</script></body></html>
)PAGE14";

void setup() {
 // All legacy routes and NVS configuration are retained.
 server.on("/hybrid",HTTP_GET,[](){server.send_P(200,"text/html",HYBRID_PAGE);});
 server.on("/api/hybrid/status",HTTP_GET,hybridStatus);
 server.on("/api/hybrid/run",HTTP_GET,hybridRun);
 server.on("/gpio14",HTTP_GET,[](){server.send_P(200,"text/html",GPIO14_DIAG_PAGE);});
 server.on("/api/test_gpio14",HTTP_POST,handleGpio14Test);
 setup_v18();
 // Disable old ultrasonic sensor #2 entirely. It must not produce any pulses.
 pinMode(15,INPUT);
 pinMode(16,INPUT);
 Wire.begin(TOF_SDA,TOF_SCL);
 Wire.setClock(400000);
 initTof();
 lastS1At=millis();
}
void loop() {
 // Deliberately do not call loop_v18, since it would ping the removed sensor S2.
 server.handleClient();
 serviceHybrid();
 delay(1);
}
