/* SONIC BONGO / PRISMA - ESP32-S3 N16R8 Dual Isolation v9.2
   Exact HC-SR04 acquisition routine recovered from v8 that worked with one sensor.
   S1: GPIO13 -> TRIG direct, ECHO -> TXS B1 -> A1 -> GPIO14
   S2: GPIO15 -> TRIG direct, ECHO -> TXS B2 -> A2 -> GPIO16
   Modes: S1 only (default), S2 only, slow alternate.
*/
#include <WiFi.h>
#include <WebServer.h>
#include <esp_system.h>

WebServer server(80);
static const char* FW="9.2-dual-isolation";
static const char* SSID="PRISMA-S3-DIAG";
static const uint32_t SENSOR_TIMEOUT_US=30000;
static const uint32_t GAP_MS=100;
static const uint32_t HEALTH_MS=10000;

struct Pins{uint8_t trig,echo;};
const Pins P[2]={{13,14},{15,16}};
struct State{bool ok=false;uint32_t ms=0,echoUs=0,samples=0,valid=0,timeouts=0;float d=0;int idle=LOW;};
State st[2];
uint8_t mode=0; // 0 S1 only, 1 S2 only, 2 alternate
uint8_t nextSensor=0;
uint32_t nextMs=0,lastHealth=0;
String logs;

void logLine(const String&s){String l="["+String(millis())+" ms] "+s+"\n";logs+=l;if(logs.length()>16000)logs.remove(0,logs.length()-14000);Serial.print(l);}

void measureSensor(uint8_t i){
  const Pins &p=P[i];
  st[i].samples++;
  st[i].idle=digitalRead(p.echo);
  digitalWrite(p.trig,LOW);
  delayMicroseconds(4);
  digitalWrite(p.trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(p.trig,LOW);
  uint32_t duration=pulseIn(p.echo,HIGH,SENSOR_TIMEOUT_US);
  st[i].ms=millis();st[i].echoUs=duration;
  if(duration>0){
    st[i].d=((float)duration*0.0343f)/2.0f;st[i].ok=true;st[i].valid++;
  }else{st[i].ok=false;st[i].timeouts++;}
}

void service(){
  uint32_t now=millis(); if((int32_t)(now-nextMs)<0)return;
  uint8_t i=mode==0?0:(mode==1?1:nextSensor);
  if(mode==2)nextSensor^=1;
  measureSensor(i);
  nextMs=millis()+GAP_MS;
}

const char PAGE[] PROGMEM=R"HTML(
<!doctype html><html lang="es"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>SONIC BONGO v9.2</title><style>
body{margin:0;background:#091018;color:#eef5fb;font-family:system-ui}header,.wrap{max-width:900px;margin:auto;padding:14px}.sub{color:#91a5b8;font-size:12px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:10px}.card{background:#111923;border:1px solid #2c3b49;border-radius:12px;padding:12px}.dist{font-size:38px;font-weight:900;color:#68f0d1}.row{display:flex;justify-content:space-between;margin:6px 0;font-size:12px}.row b{font-size:14px}.btns{display:flex;gap:8px;flex-wrap:wrap;margin:12px 0}button,a{padding:10px 12px;border-radius:9px;border:1px solid #40566c;background:#182635;color:#fff;font-weight:800;text-decoration:none}.sel{background:#16362c}.wire,pre{font:11px/1.6 monospace;background:#05090d;padding:10px;border-radius:9px}.ok{color:#45ef89}.bad{color:#ff6262}
</style></head><body><header><h2>SONIC BONGO · DUAL ISOLATION v9.2</h2><div class="sub">Rutina de adquisición recuperada de v8 · prueba un canal por vez</div></header><div class="wrap">
<div class="btns"><button onclick="m(0)">S1 SOLO</button><button onclick="m(1)">S2 SOLO</button><button onclick="m(2)">ALTERNADO LENTO</button><a href="/api/export">EXPORTAR TXT</a></div>
<div class="grid"><div class="card"><h3>S1 · GPIO13/14</h3><div class="dist" id="d1">--</div><div class="row"><span>Estado</span><b id="o1">--</b></div><div class="row"><span>ECHO</span><b id="e1">--</b></div><div class="row"><span>Línea ECHO en reposo</span><b id="i1">--</b></div><div class="row"><span>OK / timeout</span><b id="q1">--</b></div></div>
<div class="card"><h3>S2 · GPIO15/16</h3><div class="dist" id="d2">--</div><div class="row"><span>Estado</span><b id="o2">--</b></div><div class="row"><span>ECHO</span><b id="e2">--</b></div><div class="row"><span>Línea ECHO en reposo</span><b id="i2">--</b></div><div class="row"><span>OK / timeout</span><b id="q2">--</b></div></div></div>
<div class="card" style="margin-top:10px"><div class="row"><span>MODO ACTUAL</span><b id="mode">--</b></div><div class="sub">Arranca en S1 SOLO. En ALTERNADO LENTO hay 100 ms entre disparos globales y 200 ms por sensor, para eliminar casi por completo el crosstalk como variable.</div></div>
<div class="wire">S1: GPIO13 → TRIG directo · ECHO → B1 → A1 → GPIO14<br>S2: GPIO15 → TRIG directo · ECHO → B2 → A2 → GPIO16<br>TXS: VCCA=3.3V · VCCB=5V · OE=3.3V · GND común</div><pre id="log">...</pre></div>
<script>function f(x){return Number(x||0).toFixed(1)}async function r(){try{let j=await(await fetch('/api/status',{cache:'no-store'})).json();mode.textContent=j.mode;for(let n=1;n<=2;n++){let s=j['s'+n];document.getElementById('d'+n).textContent=s.ok?f(s.distance)+' cm':'TIMEOUT';document.getElementById('o'+n).textContent=s.ok?'OK':'SIN ECHO';document.getElementById('e'+n).textContent=s.echoUs+' us';document.getElementById('i'+n).textContent=s.idle?'HIGH':'LOW';document.getElementById('q'+n).textContent=s.valid+' / '+s.timeouts}}catch(e){}}async function m(x){await fetch('/api/mode?v='+x);r()}async function l(){try{log.textContent=await(await fetch('/api/log',{cache:'no-store'})).text()}catch(e){}}setInterval(r,250);setInterval(l,1800);r();l();</script></body></html>
)HTML";

String modeName(){return mode==0?"S1 SOLO":mode==1?"S2 SOLO":"ALTERNADO LENTO";}
void status(){String j="{\"fw\":\""+String(FW)+"\",\"mode\":\""+modeName()+"\",";for(uint8_t i=0;i<2;i++){j+="\"s"+String(i+1)+"\":{";j+="\"ok\":"+String(st[i].ok?"true":"false")+",";j+="\"distance\":"+String(st[i].d,2)+",";j+="\"echoUs\":"+String(st[i].echoUs)+",";j+="\"idle\":"+String(st[i].idle)+",";j+="\"valid\":"+String(st[i].valid)+",";j+="\"timeouts\":"+String(st[i].timeouts)+"}";if(i==0)j+=",";}j+="}";server.send(200,"application/json",j);}
void setMode(){if(server.hasArg("v")){int v=server.arg("v").toInt();if(v>=0&&v<=2){mode=v;nextSensor=0;nextMs=millis();logLine("MODE "+modeName());}}server.send(200,"text/plain","OK");}
void exportTxt(){String t="SONIC BONGO DUAL ISOLATION v9.2\nFW="+String(FW)+"\nmode="+modeName()+"\n";for(uint8_t i=0;i<2;i++)t+="S"+String(i+1)+" distance="+String(st[i].d,2)+" cm; echo="+String(st[i].echoUs)+" us; idle="+String(st[i].idle?"HIGH":"LOW")+"; valid="+String(st[i].valid)+"; timeouts="+String(st[i].timeouts)+"\n";t+="\n"+logs;server.sendHeader("Content-Disposition","attachment; filename=sonic_bongo_v9_2.txt");server.send(200,"text/plain",t);}

void setup(){Serial.begin(115200);delay(100);for(uint8_t i=0;i<2;i++){pinMode(P[i].trig,OUTPUT);digitalWrite(P[i].trig,LOW);pinMode(P[i].echo,INPUT);}WiFi.mode(WIFI_AP);WiFi.softAP(SSID);server.on("/",[](){server.send_P(200,"text/html",PAGE);});server.on("/api/status",status);server.on("/api/mode",setMode);server.on("/api/log",[](){server.send(200,"text/plain",logs);});server.on("/api/export",exportTxt);server.begin();nextMs=millis()+200;logLine("BOOT "+String(FW));logLine("Default mode S1 SOLO");logLine("Exact v8 trigger: LOW 4us, HIGH 10us, pulseIn HIGH timeout 30000us");}
void loop(){server.handleClient();service();uint32_t n=millis();if(n-lastHealth>=HEALTH_MS){lastHealth=n;logLine("HEALTH mode="+modeName()+" S1="+String(st[0].valid)+"/"+String(st[0].timeouts)+" idle="+String(st[0].idle)+" S2="+String(st[1].valid)+"/"+String(st[1].timeouts)+" idle="+String(st[1].idle));}delay(1);}
