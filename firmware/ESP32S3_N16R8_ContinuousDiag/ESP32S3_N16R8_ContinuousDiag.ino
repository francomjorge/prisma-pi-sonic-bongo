/* SONIC BONGO / PRISMA - ESP32-S3 N16R8 Continuous Diagnostic v8.2
   S1: GPIO13->TRIG, ECHO->TXS B1/A1->GPIO14
   S2: GPIO15->TRIG, ECHO->TXS B2/A2->GPIO16
   S3: GPIO17->TRIG, ECHO->TXS B3/A3->GPIO18
   HC-SR04: VCC=5V, GND=common. TXS: VCCA=3.3V, VCCB=5V, OE=3.3V, GND=common.
   Continuous stagger: one trigger every 25 ms, each sensor every ~75 ms (~13.3 Hz).
*/
#include <WiFi.h>
#include <WebServer.h>
#include <esp_system.h>
#include <math.h>

WebServer server(80);
static const char* FW="8.2-continuous";
static const char* SSID="PRISMA-S3-DIAG";
static const uint32_t SLOT_US=25000, TIMEOUT_US=24000;
static const uint32_t DEBOUNCE_MS=20, HEALTH_MS=10000;
static const float HIT_NEAR_CM=20.0f, HIT_SPEED_CM_S=80.0f;

struct Pins{uint8_t trig,echo,txs;};
const Pins SP[3]={{13,14,1},{15,16,2},{17,18,3}};
const uint8_t SAFE[]={2,4,5,6,7,8,9,10,11,12,21,39,40,41,42,47};
const size_t SAFE_N=sizeof(SAFE)/sizeof(SAFE[0]);

struct Sample{uint32_t ms,echoUs;float d,v,a;bool ok;};
struct State{Sample last;float prevD,v,a;uint32_t prevMs,samples,valid,timeouts,hits,lastHit;float peakV,peakA;bool have;};
State st[3]={};
static const uint8_t HIST=96;
Sample hist[3][HIST]; uint8_t hHead[3]={0},hCount[3]={0};

volatile uint32_t riseUs[3]={0},pulseUs[3]={0};
volatile bool done[3]={false},armed[3]={false};
bool running=true; int8_t active=-1; uint8_t nextS=0; uint32_t startedUs=0,nextUs=0;

String logBuf; uint32_t lastHealth=0;
int8_t inputPin=-1; int rawState=HIGH,stableState=HIGH; uint32_t rawChanged=0,presses=0,transitions=0;

void logLine(const String&s){String l="["+String(millis())+" ms] "+s+"\n";logBuf+=l;if(logBuf.length()>18000)logBuf.remove(0,logBuf.length()-16000);Serial.print(l);}
String resetText(){switch(esp_reset_reason()){case ESP_RST_POWERON:return"POWERON";case ESP_RST_SW:return"SOFTWARE";case ESP_RST_BROWNOUT:return"BROWNOUT";case ESP_RST_TASK_WDT:return"TASK_WDT";case ESP_RST_WDT:return"WDT";default:return String((int)esp_reset_reason());}}

void echoEdge(uint8_t i){if(!armed[i])return;uint32_t n=micros();int lv=digitalRead(SP[i].echo);if(lv){riseUs[i]=n;}else if(riseUs[i]){pulseUs[i]=n-riseUs[i];done[i]=true;armed[i]=false;}}
void echo0(){echoEdge(0);} void echo1(){echoEdge(1);} void echo2(){echoEdge(2);}

void saveHist(uint8_t i,const Sample&s){hist[i][hHead[i]]=s;hHead[i]=(hHead[i]+1)%HIST;if(hCount[i]<HIST)hCount[i]++;}

void process(uint8_t i,uint32_t p,bool ok){
  uint32_t now=millis(); Sample m={now,p,0,st[i].v,st[i].a,ok}; st[i].samples++;
  if(!ok||!p){st[i].timeouts++;st[i].last=m;saveHist(i,m);return;}
  m.d=((float)p*0.0343f)/2.0f; st[i].valid++;
  if(st[i].have&&now>st[i].prevMs){
    float dt=(now-st[i].prevMs)/1000.0f;
    if(dt>0.02f&&dt<0.30f){
      float rv=(m.d-st[i].prevD)/dt, oldV=st[i].v;
      st[i].v=0.45f*rv+0.55f*st[i].v;
      float ra=(st[i].v-oldV)/dt; st[i].a=0.30f*ra+0.70f*st[i].a;
      if(st[i].v<0&&-st[i].v>st[i].peakV)st[i].peakV=-st[i].v;
      if(fabsf(st[i].a)>st[i].peakA)st[i].peakA=fabsf(st[i].a);
      if(m.d<=HIT_NEAR_CM&&st[i].v<=-HIT_SPEED_CM_S&&(now-st[i].lastHit)>=250){st[i].hits++;st[i].lastHit=now;logLine("S"+String(i+1)+" HIT est d="+String(m.d,1)+" v="+String(st[i].v,1));}
    }
  }else{st[i].have=true;st[i].v=0;st[i].a=0;}
  m.v=st[i].v;m.a=st[i].a;st[i].prevD=m.d;st[i].prevMs=now;st[i].last=m;saveHist(i,m);
}

void trigger(uint8_t i){
  noInterrupts();riseUs[i]=pulseUs[i]=0;done[i]=false;armed[i]=true;interrupts();
  digitalWrite(SP[i].trig,LOW);delayMicroseconds(3);digitalWrite(SP[i].trig,HIGH);delayMicroseconds(10);digitalWrite(SP[i].trig,LOW);
  active=i;startedUs=micros();
}

void serviceSensors(){
  if(!running)return;uint32_t now=micros();
  if(active>=0){uint8_t i=(uint8_t)active;bool d;uint32_t p;noInterrupts();d=done[i];p=pulseUs[i];interrupts();
    if(d){process(i,p,true);active=-1;}else if((uint32_t)(now-startedUs)>=TIMEOUT_US){noInterrupts();armed[i]=false;interrupts();process(i,0,false);active=-1;}}
  if(active<0&&(int32_t)(now-nextUs)>=0){uint8_t i=nextS;nextS=(nextS+1)%3;nextUs=now+SLOT_US;trigger(i);}
}

bool safePin(int p){for(size_t i=0;i<SAFE_N;i++)if((int)SAFE[i]==p)return true;return false;}
void setInput(int p){if(inputPin>=0&&safePin(inputPin))pinMode(inputPin,INPUT);inputPin=-1;presses=transitions=0;if(p<0)return;if(!safePin(p))return;inputPin=p;pinMode(inputPin,INPUT_PULLUP);rawState=stableState=digitalRead(inputPin);rawChanged=millis();}
void serviceInput(){if(inputPin<0)return;int r=digitalRead(inputPin);uint32_t n=millis();if(r!=rawState){rawState=r;rawChanged=n;}if(n-rawChanged>=DEBOUNCE_MS&&r!=stableState){stableState=r;transitions++;if(stableState==LOW)presses++;}}
String motion(uint8_t i){if(!st[i].last.ok)return"TIMEOUT";if(st[i].v<-20)return"ACERCANDO";if(st[i].v>20)return"ALEJANDO";return"ESTABLE";}

const char PAGE[] PROGMEM=R"HTML(
<!doctype html><html lang="es"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>SONIC BONGO Continuous Lab</title><style>
:root{--bg:#080c11;--p:#111923;--l:#2b3948;--t:#edf4fa;--m:#8fa2b5;--ok:#45ef89;--bad:#ff6262;--w:#ffbf47;--tr:#54d7ff;--ec:#ffae5c;--test:#ff8fd8}*{box-sizing:border-box}body{margin:0;background:linear-gradient(#080c11,#0e1620);color:var(--t);font-family:system-ui;min-height:100vh}header,.stats,.panel,.boardWrap,.actions,.legend,.notice{max-width:1100px;margin:auto}header{padding:18px 16px 6px}h1{font-size:22px;margin:0}.sub,.hint{color:var(--m);font-size:11px}.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:8px;padding:8px 16px}.card,.panel{background:var(--p);border:1px solid var(--l);border-radius:11px}.card{padding:9px}.k{font-size:9px;color:var(--m)}.v{font-size:15px;font-weight:850}.ok{color:var(--ok)}.bad{color:var(--bad)}.notice{padding:10px 12px;background:#211b10;border:1px solid #594820;border-radius:10px;color:#ffda8b;font-size:12px}.panel{margin-top:12px;padding:14px 16px}.sgrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(270px,1fr));gap:9px}.sensor{background:#091018;border:1px solid #295044;border-radius:10px;padding:11px}.sensor h3{margin:0 0 8px}.big{font-size:26px;font-weight:900;color:#68f0d1}.row{display:flex;justify-content:space-between;gap:8px;margin:5px 0;font-size:12px}.row b{font-size:14px}.actions{display:flex;gap:8px;flex-wrap:wrap;padding:10px 16px 4px}button,a,select{border:1px solid #40566c;background:#182635;color:var(--t);padding:9px 11px;border-radius:9px;font-weight:800;text-decoration:none}.go{background:#12382f}.stop{background:#351919}.test{background:#3a1731}.wire{background:#060b10;border:1px solid var(--l);border-radius:8px;padding:9px;font:11px/1.6 monospace;margin-top:8px}.boardWrap{padding:14px 12px}.board{display:grid;grid-template-columns:minmax(180px,1fr) minmax(190px,235px) minmax(180px,1fr);gap:10px;background:#202a35;border:2px solid #465767;border-radius:24px;padding:14px}.pincol{display:flex;flex-direction:column;gap:5px}.pin{min-height:31px;display:flex;align-items:center;gap:7px;padding:4px 7px;background:var(--p);border:1px solid var(--l);border-radius:8px}.right .pin{flex-direction:row-reverse;text-align:right}.dot{width:12px;height:12px;border-radius:50%;background:#3b4b5a}.locked .dot{background:var(--w)}.power .dot{background:#58d8ff}.info .dot{background:#b28cff}.ground .dot{background:#000}.trigpin{border-color:#236b83}.trigpin .dot{background:var(--tr)}.echopin{border-color:#8d5d2c}.echopin .dot{background:var(--ec)}.activeTest{border-color:var(--test);box-shadow:0 0 0 1px var(--test)}.activeTest .dot{background:var(--test)}.name{font-size:12px;font-weight:850}.desc{font-size:9px;color:var(--m)}.middle{min-height:775px;display:flex;flex-direction:column;align-items:center;justify-content:space-between;padding:17px 8px;background:#0d141c;border:1px solid #344251;border-radius:18px}.ant{width:87%;height:88px;border:2px solid #6f5a36;border-radius:9px;background:repeating-linear-gradient(90deg,transparent 0 12px,#9d7a3d 12px 20px,transparent 20px 31px)}.mod{width:88%;height:292px;display:flex;align-items:center;justify-content:center;text-align:center;font-weight:900;color:#171717;background:linear-gradient(145deg,#c8b27c,#82734f);border-radius:9px}.usbrow{display:flex;gap:14px}.usb{width:56px;height:26px;background:#aab3bb;border:2px solid #68747e;border-radius:5px}.legend{display:flex;gap:12px;flex-wrap:wrap;padding:8px 16px;color:var(--m);font-size:11px}.mini{display:inline-block;width:9px;height:9px;border-radius:50%}pre{white-space:pre-wrap;max-height:300px;overflow:auto;background:#05090d;padding:10px;border-radius:8px}.hidden{display:none}@media(max-width:720px){.board{grid-template-columns:1fr 110px 1fr;gap:5px;padding:7px}.middle{min-height:742px}.desc{display:none}.name{font-size:10px}}
</style></head><body><header><h1>SONIC BONGO · CONTINUOUS SENSOR LAB v8.2</h1><div class="sub">3 HC-SR04 · distancia · velocidad · aceleración estimada · PIN LAB</div></header><section class="stats"><div class="card"><div class="k">CHIP</div><div class="v" id="chip">...</div></div><div class="card"><div class="k">FLASH</div><div class="v" id="flash">...</div></div><div class="card"><div class="k">PSRAM</div><div class="v" id="psram">...</div></div><div class="card"><div class="k">ADQUISICIÓN</div><div class="v" id="run">...</div></div><div class="card"><div class="k">SLOT</div><div class="v">25 ms</div></div><div class="card"><div class="k">CADA SENSOR</div><div class="v">~13.3 Hz</div></div></section><div class="notice"><b>TRIG directo al ESP32.</b> Sólo ECHO pasa por TXS0108E. S1/S2/S3 se disparan intercalados.</div><section class="panel"><h2>Movimiento continuo</h2><div class="sgrid">
<div class="sensor"><h3>S1 · 13/14</h3><div class="big" id="s1d">--</div><div class="row"><span>Movimiento</span><b id="s1m">--</b></div><div class="row"><span>Velocidad</span><b id="s1v">--</b></div><div class="row"><span>Aceleración est.</span><b id="s1a">--</b></div><div class="row"><span>Peak aproximación</span><b id="s1p">--</b></div><div class="row"><span>Golpes est.</span><b id="s1h">0</b></div><div class="row"><span>OK / timeout</span><b id="s1q">--</b></div></div>
<div class="sensor"><h3>S2 · 15/16</h3><div class="big" id="s2d">--</div><div class="row"><span>Movimiento</span><b id="s2m">--</b></div><div class="row"><span>Velocidad</span><b id="s2v">--</b></div><div class="row"><span>Aceleración est.</span><b id="s2a">--</b></div><div class="row"><span>Peak aproximación</span><b id="s2p">--</b></div><div class="row"><span>Golpes est.</span><b id="s2h">0</b></div><div class="row"><span>OK / timeout</span><b id="s2q">--</b></div></div>
<div class="sensor"><h3>S3 · 17/18</h3><div class="big" id="s3d">--</div><div class="row"><span>Movimiento</span><b id="s3m">--</b></div><div class="row"><span>Velocidad</span><b id="s3v">--</b></div><div class="row"><span>Aceleración est.</span><b id="s3a">--</b></div><div class="row"><span>Peak aproximación</span><b id="s3p">--</b></div><div class="row"><span>Golpes est.</span><b id="s3h">0</b></div><div class="row"><span>OK / timeout</span><b id="s3q">--</b></div></div></div><div class="actions" style="padding-left:0"><button class="go" onclick="setRun(1)">INICIAR</button><button class="stop" onclick="setRun(0)">PAUSAR</button></div><div class="wire">S1: GPIO13 → TRIG | ECHO → B1 → A1 → GPIO14<br>S2: GPIO15 → TRIG | ECHO → B2 → A2 → GPIO16<br>S3: GPIO17 → TRIG | ECHO → B3 → A3 → GPIO18<br>HC-SR04 5V/GND · TXS VCCA=3.3V VCCB=5V OE=3.3V GND común</div><div class="hint">Aceleración derivada de distancia/tiempo; para impacto físico real después conviene piezo o IMU.</div></section>
<section class="panel"><h2>Pulsador · PIN LAB</h2><div class="actions" style="padding-left:0"><select id="pinSel"><option>4</option><option>5</option><option>6</option><option>7</option><option>8</option><option>9</option><option>10</option><option>11</option><option>12</option><option>21</option><option>39</option><option>40</option><option>41</option><option>42</option><option>47</option><option>2</option></select><button class="test" onclick="armPin()">ACTIVAR INPUT_PULLUP</button><button class="stop" onclick="disablePin()">DESACTIVAR</button></div><div class="row"><span>GPIO</span><b id="ip">NINGUNO</b></div><div class="row"><span>Estado</span><b id="is">--</b></div><div class="row"><span>Pulsaciones / cambios</span><b id="ic">0 / 0</b></div><div class="wire">GPIO elegido ─ PULSADOR ─ GND · suelto=HIGH · presionado=LOW</div></section>
<div class="boardWrap"><div class="board"><div class="pincol left" id="leftPins"></div><div class="middle"><div class="ant"></div><div class="mod">ESP32-S3<br>WROOM-1<br><br>N16R8</div><div>44 PIN · CONTINUOUS LAB</div><div>BOOT &nbsp; RESET</div><div class="usbrow"><div class="usb"></div><div class="usb"></div></div></div><div class="pincol right" id="rightPins"></div></div></div><div class="legend"><span><i class="mini" style="background:#54d7ff"></i>TRIG</span><span><i class="mini" style="background:#ffae5c"></i>ECHO/TXS</span><span><i class="mini" style="background:#ff8fd8"></i>PIN TEST</span><span><i class="mini" style="background:#ffbf47"></i>RESERVADO</span></div><div class="actions"><a href="/api/export">EXPORTAR TXT</a><button onclick="logBox.classList.toggle('hidden');loadLog()">LOG</button></div><section class="panel hidden" id="logBox"><pre id="logText"></pre></section>
<script>
const L=[["3V3",null,"power","3.3V"],["3V3",null,"power","3.3V"],["RST",null,"info","RESET"],["GPIO4",4,"gpio","K1/test"],["GPIO5",5,"gpio","K2/test"],["GPIO6",6,"gpio","K3/test"],["GPIO7",7,"gpio","K4/test"],["GPIO15",15,"trigpin","TRIG S2"],["GPIO16",16,"echopin","ECHO S2"],["GPIO17",17,"trigpin","TRIG S3"],["GPIO18",18,"echopin","ECHO S3"],["GPIO8",8,"gpio","OLED SDA"],["GPIO3",3,"locked","strap/JTAG"],["GPIO46",46,"locked","strap"],["GPIO9",9,"gpio","OLED SCL"],["GPIO10",10,"gpio","Touch"],["GPIO11",11,"gpio","Touch1"],["GPIO12",12,"gpio","Touch2"],["GPIO13",13,"trigpin","TRIG S1"],["GPIO14",14,"echopin","ECHO S1"],["5V",null,"power","VIN"],["GND",null,"ground","masa"]];
const R=[["GND",null,"ground","masa"],["TX/GPIO43",43,"locked","UART"],["RX/GPIO44",44,"locked","UART"],["GPIO1",1,"gpio","ADC"],["GPIO2",2,"gpio","test"],["GPIO42",42,"gpio","test"],["GPIO41",41,"gpio","test"],["GPIO40",40,"gpio","test"],["GPIO39",39,"gpio","test"],["GPIO38",38,"locked","RGB?"],["GPIO37",37,"locked","PSRAM"],["GPIO36",36,"locked","PSRAM"],["GPIO35",35,"locked","PSRAM"],["GPIO0",0,"locked","BOOT"],["GPIO45",45,"locked","strap"],["GPIO48",48,"locked","reserved"],["GPIO47",47,"gpio","test"],["GPIO21",21,"gpio","ARM/MUTE"],["GPIO20",20,"locked","USB D+"],["GPIO19",19,"locked","USB D-"],["GND",null,"ground","masa"],["GND",null,"ground","masa"]];
function mk(p){let e=document.createElement('div');e.className='pin '+(p[2]=='gpio'?'':p[2]);e.dataset.gpio=p[1]===null?'':p[1];e.innerHTML='<span class="dot"></span><div><div class="name">'+p[0]+'</div><div class="desc">'+p[3]+'</div></div>';return e}L.forEach(p=>leftPins.appendChild(mk(p)));R.forEach(p=>rightPins.appendChild(mk(p)));
function s(i,x){document.getElementById('s'+i+'d').textContent=x.ok?x.d.toFixed(1)+' cm':'TIMEOUT';document.getElementById('s'+i+'m').textContent=x.m;document.getElementById('s'+i+'v').textContent=x.v.toFixed(1)+' cm/s';document.getElementById('s'+i+'a').textContent=x.a.toFixed(0)+' cm/s²';document.getElementById('s'+i+'p').textContent=x.peakV.toFixed(1)+' cm/s';document.getElementById('s'+i+'h').textContent=x.hits;document.getElementById('s'+i+'q').textContent=x.valid+' / '+x.timeouts}
async function refresh(){try{let d=await(await fetch('/api/status',{cache:'no-store'})).json();chip.textContent=d.chip;flash.textContent=d.flash+' MB';psram.textContent=d.psram+' MB';run.textContent=d.running?'CORRIENDO':'PAUSADO';run.className='v '+(d.running?'ok':'bad');s(1,d.s[0]);s(2,d.s[1]);s(3,d.s[2]);ip.textContent=d.input.active?'GPIO'+d.input.pin:'NINGUNO';is.textContent=d.input.active?(d.input.pressed?'PRESIONADO/LOW':'SUELTO/HIGH'):'--';ic.textContent=d.input.presses+' / '+d.input.transitions;document.querySelectorAll('.pin').forEach(e=>{e.classList.remove('activeTest');if(d.input.active&&Number(e.dataset.gpio)===d.input.pin)e.classList.add('activeTest')})}catch(e){run.textContent='WEB ERROR'}}
async function setRun(v){await fetch('/api/run?v='+v);refresh()}async function armPin(){await fetch('/api/input?pin='+pinSel.value);refresh()}async function disablePin(){await fetch('/api/input?pin=-1');refresh()}async function loadLog(){if(!logBox.classList.contains('hidden'))logText.textContent=await(await fetch('/api/log',{cache:'no-store'})).text()}refresh();setInterval(refresh,150);setInterval(loadLog,2000);
</script></body></html>
)HTML";

void sensorJson(String&j,uint8_t i){Sample&m=st[i].last;j+="{\"ok\":"+String(m.ok?"true":"false")+",\"d\":"+String(m.d,2)+",\"v\":"+String(st[i].v,2)+",\"a\":"+String(st[i].a,2)+",\"peakV\":"+String(st[i].peakV,2)+",\"peakA\":"+String(st[i].peakA,2)+",\"hits\":"+String(st[i].hits)+",\"valid\":"+String(st[i].valid)+",\"timeouts\":"+String(st[i].timeouts)+",\"m\":\""+motion(i)+"\"}";}
void statusApi(){String j;j.reserve(2200);j+="{\"fw\":\""+String(FW)+"\",\"chip\":\""+String(ESP.getChipModel())+"\",\"flash\":"+String(ESP.getFlashChipSize()/(1024*1024))+",\"psram\":"+String(ESP.getPsramSize()/(1024*1024))+",\"running\":"+String(running?"true":"false")+",\"s\":[";sensorJson(j,0);j+=",";sensorJson(j,1);j+=",";sensorJson(j,2);j+="],\"input\":{\"active\":"+String(inputPin>=0?"true":"false")+",\"pin\":"+String(inputPin)+",\"pressed\":"+String(inputPin>=0&&stableState==LOW?"true":"false")+",\"presses\":"+String(presses)+",\"transitions\":"+String(transitions)+"}}";server.send(200,"application/json",j);}
void runApi(){if(server.hasArg("v"))running=server.arg("v").toInt()!=0;if(!running&&active>=0){noInterrupts();armed[(uint8_t)active]=false;interrupts();active=-1;}if(running)nextUs=micros()+1000;logLine(String("RUN ")+(running?"ON":"OFF"));server.send(200,"text/plain","OK");}
void inputApi(){setInput(server.hasArg("pin")?server.arg("pin").toInt():-1);server.send(200,"text/plain","OK");}
void exportApi(){String o;o.reserve(30000);o+="SONIC BONGO CONTINUOUS DIAG v"+String(FW)+"\nreset="+resetText()+"\nschedule=25ms global, ~75ms per sensor, ~13.3Hz\n";o+="wiring: S1 13/TRIG 14/ECHO(B1/A1); S2 15/16(B2/A2); S3 17/18(B3/A3)\n\n";for(uint8_t i=0;i<3;i++){o+="S"+String(i+1)+" samples="+String(st[i].samples)+" valid="+String(st[i].valid)+" timeout="+String(st[i].timeouts)+" hits="+String(st[i].hits)+" peakV="+String(st[i].peakV,2)+" peakA="+String(st[i].peakA,2)+"\nms,ok,echo_us,d_cm,v_cm_s,a_cm_s2\n";uint8_t start=(hHead[i]+HIST-hCount[i])%HIST;for(uint8_t n=0;n<hCount[i];n++){Sample&m=hist[i][(start+n)%HIST];o+=String(m.ms)+","+String(m.ok?1:0)+","+String(m.echoUs)+","+String(m.d,2)+","+String(m.v,2)+","+String(m.a,2)+"\n";}o+="\n";}o+="LOG\n"+logBuf;server.sendHeader("Content-Disposition","attachment; filename=SONIC_BONGO_CONTINUOUS_DIAG_v8_2.txt");server.send(200,"text/plain; charset=utf-8",o);}

void setup(){Serial.begin(115200);delay(300);for(size_t i=0;i<SAFE_N;i++)pinMode(SAFE[i],INPUT);for(uint8_t i=0;i<3;i++){pinMode(SP[i].trig,OUTPUT);digitalWrite(SP[i].trig,LOW);pinMode(SP[i].echo,INPUT);}attachInterrupt(digitalPinToInterrupt(14),echo0,CHANGE);attachInterrupt(digitalPinToInterrupt(16),echo1,CHANGE);attachInterrupt(digitalPinToInterrupt(18),echo2,CHANGE);WiFi.mode(WIFI_AP);WiFi.setSleep(false);WiFi.softAP(SSID);server.on("/",HTTP_GET,[]{server.send_P(200,"text/html; charset=utf-8",PAGE);});server.on("/api/status",HTTP_GET,statusApi);server.on("/api/run",HTTP_GET,runApi);server.on("/api/input",HTTP_GET,inputApi);server.on("/api/log",HTTP_GET,[]{server.send(200,"text/plain",logBuf);});server.on("/api/export",HTTP_GET,exportApi);server.begin();nextUs=micros()+100000;logLine("BOOT v8.2 AP="+String(SSID)+" IP="+WiFi.softAPIP().toString());}
void loop(){server.handleClient();serviceSensors();serviceInput();uint32_t n=millis();if(n-lastHealth>=HEALTH_MS){lastHealth=n;logLine("HEALTH heap="+String(ESP.getFreeHeap())+" S1="+String(st[0].valid)+"/"+String(st[0].timeouts)+" S2="+String(st[1].valid)+"/"+String(st[1].timeouts)+" S3="+String(st[2].valid)+"/"+String(st[2].timeouts));}delay(1);}
