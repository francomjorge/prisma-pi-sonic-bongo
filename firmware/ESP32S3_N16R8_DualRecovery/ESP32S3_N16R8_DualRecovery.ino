/* SONIC BONGO / PRISMA - ESP32-S3 N16R8 Dual Recovery v9.2
   Goal: recover the exact HC-SR04 acquisition path that already worked,
   then compare S1-only, S2-only and slow dual operation.

   S1: GPIO13 -> TRIG direct; ECHO -> TXS B1 -> A1 -> GPIO14
   S2: GPIO15 -> TRIG direct; ECHO -> TXS B2 -> A2 -> GPIO16
   HC-SR04 VCC=5V, GND=common
   TXS VCCA=3.3V, VCCB=5V, OE=3.3V, GND=common
*/
#include <WiFi.h>
#include <WebServer.h>

WebServer server(80);
static const char* SSID="PRISMA-S3-DIAG";
static const char* FW="9.2-dual-recovery";
static const uint32_t TIMEOUT_US=30000;
static const uint32_t SLOT_MS=80; // diagnostic: generous acoustic separation

struct Pins{uint8_t trig,echo;};
const Pins P[2]={{13,14},{15,16}};
struct State{bool ok=false;uint32_t pulse=0;float cm=0;uint32_t okCount=0;uint32_t timeouts=0;uint32_t lastMs=0;int idle=0;};
State st[2];

enum Mode:uint8_t{STOPPED=0,S1_ONLY=1,S2_ONLY=2,DUAL=3};
Mode mode=S1_ONLY;
uint8_t nextDual=0;uint32_t nextMs=0;
String logBuf;

void logLine(const String&s){String x="["+String(millis())+" ms] "+s+"\n";logBuf+=x;if(logBuf.length()>14000)logBuf.remove(0,2000);Serial.print(x);}

void measure(uint8_t i){
  st[i].idle=digitalRead(P[i].echo);
  digitalWrite(P[i].trig,LOW);delayMicroseconds(4);
  digitalWrite(P[i].trig,HIGH);delayMicroseconds(10);
  digitalWrite(P[i].trig,LOW);
  uint32_t d=pulseIn(P[i].echo,HIGH,TIMEOUT_US);
  st[i].lastMs=millis();st[i].pulse=d;
  if(d){st[i].ok=true;st[i].cm=((float)d*0.0343f)/2.0f;st[i].okCount++;}
  else{st[i].ok=false;st[i].cm=0;st[i].timeouts++;}
}

void service(){
  if(mode==STOPPED)return;
  uint32_t now=millis(); if((int32_t)(now-nextMs)<0)return;
  if(mode==S1_ONLY) measure(0);
  else if(mode==S2_ONLY) measure(1);
  else {measure(nextDual);nextDual^=1;}
  nextMs=millis()+SLOT_MS;
}

String modeName(){if(mode==S1_ONLY)return"S1 SOLO";if(mode==S2_ONLY)return"S2 SOLO";if(mode==DUAL)return"DUAL LENTO";return"DETENIDO";}

const char PAGE[] PROGMEM=R"HTML(
<!doctype html><html lang="es"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>SONIC BONGO v9.2</title><style>
body{margin:0;background:#081019;color:#edf5fb;font-family:system-ui}.w{max-width:900px;margin:auto;padding:14px}.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:10px}.c{background:#111923;border:1px solid #304151;border-radius:12px;padding:12px}.d{font-size:38px;font-weight:900;color:#68f0d1}.row{display:flex;justify-content:space-between;margin:6px 0}.muted{color:#8fa2b5;font-size:12px}.bad{color:#ff6868}.ok{color:#51ef8e}button{padding:10px 12px;margin:4px;border-radius:9px;border:1px solid #496078;background:#172636;color:#fff;font-weight:800}.on{background:#164431}.wire,pre{background:#05090d;border:1px solid #2c3a46;border-radius:9px;padding:10px;font:11px/1.6 monospace;white-space:pre-wrap}</style></head><body><div class="w">
<h2>SONIC BONGO · HC-SR04 RECOVERY v9.2</h2><div class="muted">Primero probamos cada sensor aislado; después DUAL lento.</div>
<div class="c"><div class="row"><span>MODO</span><b id="mode">...</b></div><button onclick="setMode(1)">S1 SOLO</button><button onclick="setMode(2)">S2 SOLO</button><button onclick="setMode(3)">DUAL LENTO</button><button onclick="setMode(0)">DETENER</button><button onclick="once(0)">MEDIR S1 AHORA</button><button onclick="once(1)">MEDIR S2 AHORA</button></div>
<div class="cards">
<div class="c"><h3>S1 · GPIO13/14</h3><div class="d" id="d1">--</div><div class="row"><span>ECHO us</span><b id="p1">--</b></div><div class="row"><span>ECHO idle antes trigger</span><b id="i1">--</b></div><div class="row"><span>OK / TIMEOUT</span><b id="q1">--</b></div></div>
<div class="c"><h3>S2 · GPIO15/16</h3><div class="d" id="d2">--</div><div class="row"><span>ECHO us</span><b id="p2">--</b></div><div class="row"><span>ECHO idle antes trigger</span><b id="i2">--</b></div><div class="row"><span>OK / TIMEOUT</span><b id="q2">--</b></div></div>
</div>
<div class="c"><h3>Interpretación</h3><div class="muted">Si S1 SOLO mide y S2 SOLO mide, pero DUAL falla: interferencia acústica/timing. Si un solo canal falla aislado: revisar ese ECHO/TXS. Si ambos fallan aislados: revisar alimentación/TXS/OE/GND común.</div></div>
<div class="wire">S1: GPIO13 → TRIG directo · ECHO → B1 → A1 → GPIO14\nS2: GPIO15 → TRIG directo · ECHO → B2 → A2 → GPIO16\nTXS: VCCA=3.3V · VCCB=5V · OE=3.3V · GND común</div><pre id="log">...</pre>
</div><script>
async function status(){try{let r=await fetch('/api/status',{cache:'no-store'}),j=await r.json();mode.textContent=j.mode;for(let n=1;n<=2;n++){let s=j['s'+n];document.getElementById('d'+n).textContent=s.ok?s.cm.toFixed(1)+' cm':'TIMEOUT';document.getElementById('p'+n).textContent=s.pulse;document.getElementById('i'+n).textContent=s.idle?'HIGH':'LOW';document.getElementById('q'+n).textContent=s.okCount+' / '+s.timeouts}}catch(e){}}
async function setMode(m){await fetch('/api/mode?m='+m);status()}async function once(i){await fetch('/api/once?i='+i);status()}async function logs(){try{let r=await fetch('/api/log',{cache:'no-store'});log.textContent=await r.text()}catch(e){}}
setInterval(status,250);setInterval(logs,1500);status();logs();
</script></body></html>
)HTML";

void sendStatus(){String j="{\"fw\":\""+String(FW)+"\",\"mode\":\""+modeName()+"\"";for(uint8_t i=0;i<2;i++){j+=",\"s"+String(i+1)+"\":{";j+="\"ok\":"+String(st[i].ok?"true":"false")+",";j+="\"cm\":"+String(st[i].cm,2)+",";j+="\"pulse\":"+String(st[i].pulse)+",";j+="\"idle\":"+String(st[i].idle)+",";j+="\"okCount\":"+String(st[i].okCount)+",";j+="\"timeouts\":"+String(st[i].timeouts)+"}";}j+="}";server.send(200,"application/json",j);}

void setup(){
  Serial.begin(115200);delay(100);
  for(uint8_t i=0;i<2;i++){pinMode(P[i].echo,INPUT);pinMode(P[i].trig,OUTPUT);digitalWrite(P[i].trig,LOW);}
  WiFi.mode(WIFI_AP);WiFi.softAP(SSID);
  server.on("/",[](){server.send_P(200,"text/html",PAGE);});
  server.on("/api/status",sendStatus);
  server.on("/api/log",[](){server.send(200,"text/plain",logBuf);});
  server.on("/api/mode",[](){if(server.hasArg("m")){int m=server.arg("m").toInt();if(m>=0&&m<=3){mode=(Mode)m;nextDual=0;nextMs=millis();logLine("MODE "+modeName());}}server.send(200,"text/plain","OK");});
  server.on("/api/once",[](){if(server.hasArg("i")){int i=server.arg("i").toInt();if(i==0||i==1){Mode old=mode;mode=STOPPED;measure(i);logLine("ONE-SHOT S"+String(i+1)+(st[i].ok?" OK ":" TIMEOUT ")+String(st[i].cm,1)+" cm pulse="+String(st[i].pulse));mode=old;nextMs=millis()+SLOT_MS;}}server.send(200,"text/plain","OK");});
  server.begin();nextMs=millis()+300;logLine("BOOT "+String(FW));logLine("Default mode S1 SOLO");
}
void loop(){server.handleClient();service();delay(1);}
