/*
  SONIC BONGO / PRISMA
  ESP32-S3 N16R8 - Dual Sensor Bongo v10

  CONFIRMED WIRING (unchanged from v9.2):
    S1: GPIO13 -> TRIG direct
        HC-SR04 ECHO -> TXS B1 -> A1 -> GPIO14
    S2: GPIO15 -> TRIG direct
        HC-SR04 ECHO -> TXS B2 -> A2 -> GPIO16

    HC-SR04 VCC -> 5V
    HC-SR04 GND -> common GND
    TXS VCCA -> 3.3V
    TXS VCCB -> 5V
    TXS OE   -> 3.3V
    TXS GND  -> common GND

  Design:
    - Two HC-SR04 form ONE virtual bongo patch.
    - Sensors are 1.0 cm apart (configurable baseline).
    - Alternating pings reduce acoustic overlap.
    - Geometry is diagnostic only: 1 cm baseline makes lateral x very noise-sensitive.
    - HIT engine uses fused range + approach velocity + dual-sensor coherence + hysteresis.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

WebServer server(80);

static const char* FW_VERSION = "10.0-dual-bongo";
static const char* AP_SSID = "PRISMA-S3-DIAG";

struct SensorPins { uint8_t trig; uint8_t echo; };
const SensorPins PINS[2] = { {13,14}, {15,16} };

static const uint32_t ECHO_TIMEOUT_US = 30000;
static const uint32_t SLOT_MS = 40;            // alternating ping, each sensor ~80ms
static const uint32_t STALE_MS = 220;
static const float FILTER_ALPHA = 0.55f;
static const float VELOCITY_ALPHA = 0.45f;

float baselineCm = 1.0f;
float hitPlaneCm = 12.0f;
float hitSpeedCmS = 35.0f;
float rearmMarginCm = 6.0f;
float coneHalfAngleDeg = 7.5f; // visualization/model only

struct SensorState {
  bool have = false;
  bool ok = false;
  uint32_t lastMs = 0;
  uint32_t pulseUs = 0;
  int idle = 0;
  float rawCm = 0;
  float filteredCm = 0;
  float prevFilteredCm = 0;
  uint32_t prevMs = 0;
  float velocityCmS = 0;
  uint32_t valid = 0;
  uint32_t timeouts = 0;
};
SensorState s[2];

struct PairState {
  bool valid = false;
  bool geoValid = false;
  bool armed = true;
  float zCm = 0;
  float xCm = 0;
  float deltaCm = 0;
  float velocityCmS = 0;
  float coherencePct = 0;
  float peakApproachCmS = 0;
  uint32_t hits = 0;
  uint32_t lastHitMs = 0;
  float lastHitVelocity = 0;
  int lastHitMidi = 0;
  String lastZone = "--";
};
PairState pair;

uint8_t nextSensor = 0;
uint32_t nextMeasureMs = 0;
String logBuffer;

void addLog(const String& text) {
  String line = "[" + String(millis()) + " ms] " + text + "\n";
  logBuffer += line;
  if (logBuffer.length() > 15000) logBuffer.remove(0, 2500);
  Serial.print(line);
}

float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

int midiFromVelocity(float approachSpeed) {
  float x = clampf((approachSpeed - 25.0f) / (220.0f - 25.0f), 0.0f, 1.0f);
  return 1 + (int)lroundf(x * 126.0f);
}

String zoneFromDelta(float delta) {
  if (fabsf(delta) < 0.35f) return "CENTRO";
  return delta < 0 ? "IZQUIERDA" : "DERECHA";
}

void updatePair() {
  uint32_t now = millis();
  bool fresh0 = s[0].ok && (now - s[0].lastMs <= STALE_MS);
  bool fresh1 = s[1].ok && (now - s[1].lastMs <= STALE_MS);
  pair.valid = fresh0 && fresh1;

  if (!pair.valid) {
    if (now - pair.lastHitMs > 500) pair.armed = true;
    return;
  }

  float d1 = s[0].filteredCm;
  float d2 = s[1].filteredCm;
  pair.zCm = 0.5f * (d1 + d2);
  pair.deltaCm = d1 - d2;
  pair.velocityCmS = 0.5f * (s[0].velocityCmS + s[1].velocityCmS);

  // Soft confidence: excellent when both ranges agree, fades by about 5 cm difference.
  pair.coherencePct = clampf(100.0f - fabsf(pair.deltaCm) * 20.0f, 0.0f, 100.0f);

  // Exact 2D circle-intersection diagnostic. With b=1cm this can be unstable/noisy.
  pair.geoValid = false;
  if (baselineCm > 0.01f && fabsf(d1 - d2) <= baselineCm) {
    float x = (d1*d1 - d2*d2) / (2.0f * baselineCm);
    float z2 = d1*d1 - (x + baselineCm*0.5f) * (x + baselineCm*0.5f);
    if (z2 >= 0.0f) {
      pair.xCm = x;
      pair.geoValid = true;
    }
  }

  if (pair.velocityCmS < 0 && -pair.velocityCmS > pair.peakApproachCmS)
    pair.peakApproachCmS = -pair.velocityCmS;

  // Virtual bongo hit:
  //  - both sensors recent and valid
  //  - hand inside virtual plane
  //  - approaching fast enough
  //  - at least modest agreement between sensors
  if (pair.armed &&
      pair.zCm <= hitPlaneCm &&
      pair.velocityCmS <= -hitSpeedCmS &&
      pair.coherencePct >= 20.0f) {
    pair.hits++;
    pair.lastHitMs = now;
    pair.lastHitVelocity = -pair.velocityCmS;
    pair.lastHitMidi = midiFromVelocity(pair.lastHitVelocity);
    pair.lastZone = zoneFromDelta(pair.deltaCm);
    pair.armed = false;
    addLog("HIT z=" + String(pair.zCm,1) +
           "cm v=" + String(pair.lastHitVelocity,1) +
           "cm/s midi=" + String(pair.lastHitMidi) +
           " zone=" + pair.lastZone +
           " coh=" + String(pair.coherencePct,0) + "%");
  }

  // Rearm only after the hand clearly leaves the virtual head.
  if (!pair.armed && pair.zCm >= hitPlaneCm + rearmMarginCm) {
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
    q.filteredCm = raw;
    q.prevFilteredCm = raw;
    q.prevMs = now;
    q.velocityCmS = 0;
  } else {
    q.filteredCm = FILTER_ALPHA * raw + (1.0f - FILTER_ALPHA) * q.filteredCm;
    if (now > q.prevMs) {
      float dt = (now - q.prevMs) / 1000.0f;
      if (dt > 0.035f && dt < 0.35f) {
        float rawV = (q.filteredCm - q.prevFilteredCm) / dt;
        q.velocityCmS = VELOCITY_ALPHA * rawV + (1.0f - VELOCITY_ALPHA) * q.velocityCmS;
      }
    }
    q.prevFilteredCm = q.filteredCm;
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
  digitalWrite(p.trig, LOW);
  delayMicroseconds(4);
  digitalWrite(p.trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(p.trig, LOW);
  uint32_t pulse = pulseIn(p.echo, HIGH, ECHO_TIMEOUT_US);
  if (pulse) processValid(i, pulse);
  else processTimeout(i);
  updatePair();
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
  if (pair.velocityCmS <= -15.0f) return "ACERCANDO";
  if (pair.velocityCmS >= 15.0f) return "ALEJANDO";
  return "ESTABLE";
}

const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html lang="es"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>SONIC BONGO v10</title>
<style>
:root{--bg:#081019;--p:#111923;--l:#2d3d4c;--t:#edf5fb;--m:#8fa2b5;--a:#68f0d1;--b:#79a7ff;--hit:#ffcc66;--ok:#50e98a;--bad:#ff6969}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--t);font-family:system-ui,-apple-system,Segoe UI,sans-serif}header,.w{max-width:1180px;margin:auto;padding:14px}h1{font-size:22px;margin:0}.sub{color:var(--m);font-size:12px;margin-top:4px}.cards{display:grid;grid-template-columns:repeat(5,minmax(120px,1fr));gap:8px;margin-bottom:10px}.c,.panel{background:var(--p);border:1px solid var(--l);border-radius:12px;padding:12px}.k{font-size:10px;letter-spacing:.05em;color:var(--m)}.v{font-size:21px;font-weight:900;margin-top:4px}.a{color:var(--a)}.b{color:var(--b)}.hit{color:var(--hit)}.grid{display:grid;grid-template-columns:1.15fr .85fr;gap:10px}.row{display:flex;justify-content:space-between;gap:10px;margin:7px 0;font-size:12px}.row b{font-size:13px}canvas{display:block;width:100%;height:auto;background:#070d13;border:1px solid #263441;border-radius:10px}.controls{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:10px}label{font-size:11px;color:var(--m)}input{width:100%;margin-top:4px;background:#0a1118;color:#fff;border:1px solid #3a4c5c;border-radius:7px;padding:7px}button{padding:9px 11px;border-radius:8px;border:1px solid #40566c;background:#182635;color:#fff;font-weight:800}.note{font-size:11px;color:var(--m);line-height:1.45;margin-top:8px}.badge{display:inline-block;padding:4px 7px;border:1px solid #40566c;border-radius:7px;font-size:11px;font-weight:900}.ok{color:var(--ok)}.bad{color:var(--bad)}@media(max-width:900px){.grid{grid-template-columns:1fr}.cards{grid-template-columns:repeat(2,1fr)}}
</style></head><body>
<header><h1>SONIC BONGO · DUAL SENSOR PATCH v10</h1><div class="sub">2 HC-SR04 = 1 bongó virtual · baseline 1 cm · fusión + velocidad + coherencia + geometría diagnóstica</div></header>
<div class="w">
<div class="cards">
<div class="c"><div class="k">S1</div><div class="v a" id="d1">--</div></div>
<div class="c"><div class="k">S2</div><div class="v b" id="d2">--</div></div>
<div class="c"><div class="k">Z FUSIONADA</div><div class="v" id="z">--</div></div>
<div class="c"><div class="k">MOVIMIENTO</div><div class="v" id="motion">--</div></div>
<div class="c"><div class="k">HITS</div><div class="v hit" id="hits">0</div></div>
</div>
<div class="grid">
<section class="panel"><div class="row"><b>PARCHE / GEOMETRÍA</b><span id="coh" class="badge">--</span></div><canvas id="geo" width="720" height="520"></canvas><div class="note">Los círculos son las distancias medidas. El punto X/Z sólo se dibuja cuando las dos circunferencias tienen una intersección física exacta. El HIT no depende de esa triangulación.</div></section>
<section class="panel"><b>MOTOR DE BONGÓ</b>
<div class="row"><span>Baseline</span><b id="baseTxt">1.00 cm</b></div><div class="row"><span>Δ S1−S2</span><b id="delta">--</b></div><div class="row"><span>X estimada</span><b id="x">--</b></div><div class="row"><span>Velocidad fusionada</span><b id="vel">--</b></div><div class="row"><span>Coherencia</span><b id="cohTxt">--</b></div><div class="row"><span>Plano HIT</span><b id="planeTxt">12 cm</b></div><div class="row"><span>Umbral velocidad</span><b id="speedTxt">35 cm/s</b></div><div class="row"><span>ARMADO</span><b id="armed">--</b></div><div class="row"><span>Último HIT</span><b id="last">--</b></div><div class="row"><span>Velocity MIDI</span><b id="midi">--</b></div><div class="row"><span>Zona estimada</span><b id="zone">--</b></div>
<div class="controls"><label>Baseline cm<input id="base" type="number" min="0.2" max="20" step="0.1" value="1.0"></label><label>Plano HIT cm<input id="plane" type="number" min="4" max="40" step="0.5" value="12"></label><label>Velocidad mínima cm/s<input id="speed" type="number" min="5" max="300" step="5" value="35"></label><label>Semiángulo cono °<input id="angle" type="number" min="2" max="30" step="0.5" value="7.5"></label></div><button style="margin-top:10px" onclick="saveCfg()">APLICAR</button> <button onclick="resetHits()">RESET HITS</button>
<div class="note">HIT = ambos sensores recientes + Z fusionada dentro del plano + velocidad de aproximación suficiente + coherencia mínima. Rearm cuando la mano se retira 6 cm por encima del plano.</div></section>
</div>
<section class="panel" style="margin-top:10px"><div class="row"><b>TIMELINE</b><span class="note">S1 · S2 · Z fusionada · HIT</span></div><canvas id="tl" width="1120" height="270"></canvas></section>
</div>
<script>
const G=document.getElementById('geo').getContext('2d'),T=document.getElementById('tl').getContext('2d');const H=[];let lastHits=0;
function line(c,x1,y1,x2,y2,col='#40505d',w=1){c.beginPath();c.moveTo(x1,y1);c.lineTo(x2,y2);c.strokeStyle=col;c.lineWidth=w;c.stroke()}function txt(c,s,x,y,col='#91a4b5',sz=10,a='left'){c.fillStyle=col;c.font=sz+'px system-ui';c.textAlign=a;c.fillText(s,x,y)}
function drawGeo(j){let W=720,Hh=520,by=470,cx=360,max=35,sc=(Hh-80)/max;G.clearRect(0,0,W,Hh);for(let q=5;q<=35;q+=5){let y=by-q*sc;line(G,45,y,W-25,y,'#17232d');txt(G,q+'cm',8,y+3)}let x1=cx-j.cfg.baseline/2*sc,x2=cx+j.cfg.baseline/2*sc,hy=by-j.cfg.hitPlane*sc;line(G,45,hy,W-25,hy,'#ffcc66',2);txt(G,'HIT '+j.cfg.hitPlane.toFixed(1)+'cm',W-30,hy-6,'#ffcc66',11,'right');let len=max*sc,spread=Math.tan(j.cfg.coneHalfAngle* Math.PI/180)*len;[[x1,'#68f0d1'],[x2,'#79a7ff']].forEach(v=>{G.save();G.globalAlpha=.13;G.beginPath();G.moveTo(v[0],by);G.lineTo(v[0]-spread,by-len);G.lineTo(v[0]+spread,by-len);G.closePath();G.fillStyle=v[1];G.fill();G.restore();line(G,v[0],by,v[0]-spread,by-len,v[1]);line(G,v[0],by,v[0]+spread,by-len,v[1])});if(j.s1.ok){G.beginPath();G.arc(x1,by,j.s1.filtered*sc,0,Math.PI*2);G.strokeStyle='#68f0d1';G.globalAlpha=.45;G.stroke();G.globalAlpha=1}if(j.s2.ok){G.beginPath();G.arc(x2,by,j.s2.filtered*sc,0,Math.PI*2);G.strokeStyle='#79a7ff';G.globalAlpha=.45;G.stroke();G.globalAlpha=1}G.fillStyle='#68f0d1';G.fillRect(x1-7,by-6,14,12);G.fillStyle='#79a7ff';G.fillRect(x2-7,by-6,14,12);txt(G,'S1',x1,by+24,'#68f0d1',11,'center');txt(G,'S2',x2,by+24,'#79a7ff',11,'center');if(j.pair.valid){let px=cx+(j.pair.geoValid?j.pair.x:0)*sc,py=by-j.pair.z*sc;G.beginPath();G.arc(px,py,8,0,Math.PI*2);G.fillStyle=j.pair.geoValid?'#fff':'#ffcc66';G.fill();txt(G,j.pair.geoValid?'X/Z':'Z fusionada',px+12,py-10,j.pair.geoValid?'#fff':'#ffcc66',11)}}
function drawTL(j){let W=1120,Hh=270,p=35,max=35;T.clearRect(0,0,W,Hh);let y=v=>Hh-p-v/max*(Hh-2*p);for(let q=5;q<=35;q+=5){line(T,p,y(q),W-10,y(q),'#17232d');txt(T,q,7,y(q)+3)}line(T,p,y(j.cfg.hitPlane),W-10,y(j.cfg.hitPlane),'#ffcc66',1.5);function ser(k,col){T.beginPath();let st=false;H.forEach((r,i)=>{let v=r[k];if(!Number.isFinite(v)){st=false;return}let x=p+i/(Math.max(1,H.length-1))*(W-p-15);if(!st){T.moveTo(x,y(v));st=true}else T.lineTo(x,y(v))});T.strokeStyle=col;T.lineWidth=2;T.stroke()}ser('d1','#68f0d1');ser('d2','#79a7ff');ser('z','#fff');H.forEach((r,i)=>{if(r.hit){let x=p+i/(Math.max(1,H.length-1))*(W-p-15);line(T,x,15,x,Hh-p,'#ffcc66',1.5)}})}
async function refresh(){try{let r=await fetch('/api/status',{cache:'no-store'}),j=await r.json();d1.textContent=j.s1.ok?j.s1.filtered.toFixed(1)+' cm':'TIMEOUT';d2.textContent=j.s2.ok?j.s2.filtered.toFixed(1)+' cm':'TIMEOUT';z.textContent=j.pair.valid?j.pair.z.toFixed(1)+' cm':'--';motion.textContent=j.pair.motion;hits.textContent=j.pair.hits;delta.textContent=j.pair.valid?j.pair.delta.toFixed(2)+' cm':'--';x.textContent=j.pair.geoValid?j.pair.x.toFixed(2)+' cm':'--';vel.textContent=j.pair.valid?j.pair.velocity.toFixed(1)+' cm/s':'--';cohTxt.textContent=j.pair.valid?j.pair.coherence.toFixed(0)+'%':'--';coh.textContent=j.pair.valid?'COH '+j.pair.coherence.toFixed(0)+'%':'SIN PAR';coh.className='badge '+(j.pair.coherence>=50?'ok':'bad');planeTxt.textContent=j.cfg.hitPlane.toFixed(1)+' cm';speedTxt.textContent=j.cfg.hitSpeed.toFixed(0)+' cm/s';baseTxt.textContent=j.cfg.baseline.toFixed(2)+' cm';armed.textContent=j.pair.armed?'SÍ':'NO';last.textContent=j.pair.lastHitMs?j.pair.lastHitVelocity.toFixed(1)+' cm/s':'--';midi.textContent=j.pair.lastMidi||'--';zone.textContent=j.pair.lastZone;base.value=j.cfg.baseline;plane.value=j.cfg.hitPlane;speed.value=j.cfg.hitSpeed;angle.value=j.cfg.coneHalfAngle;let hit=j.pair.hits>lastHits;lastHits=j.pair.hits;H.push({d1:j.s1.ok?j.s1.filtered:NaN,d2:j.s2.ok?j.s2.filtered:NaN,z:j.pair.valid?j.pair.z:NaN,hit});if(H.length>180)H.shift();drawGeo(j);drawTL(j)}catch(e){}}
async function saveCfg(){await fetch('/api/config?b='+encodeURIComponent(base.value)+'&p='+encodeURIComponent(plane.value)+'&s='+encodeURIComponent(speed.value)+'&a='+encodeURIComponent(angle.value));refresh()}async function resetHits(){await fetch('/api/reset');refresh()}setInterval(refresh,120);refresh();
</script></body></html>
)HTML";

void sendStatus() {
  String j = "{";
  j += "\"fw\":\"" + String(FW_VERSION) + "\",";
  for (uint8_t i=0;i<2;i++) {
    SensorState &q=s[i];
    j += "\"s" + String(i+1) + "\":{";
    j += "\"ok\":" + String(q.ok?"true":"false") + ",";
    j += "\"raw\":" + String(q.rawCm,2) + ",";
    j += "\"filtered\":" + String(q.filteredCm,2) + ",";
    j += "\"velocity\":" + String(q.velocityCmS,2) + ",";
    j += "\"pulse\":" + String(q.pulseUs) + ",";
    j += "\"valid\":" + String(q.valid) + ",";
    j += "\"timeouts\":" + String(q.timeouts) + "}";
    if(i==0) j += ",";
  }
  j += ",\"pair\":{";
  j += "\"valid\":" + String(pair.valid?"true":"false") + ",";
  j += "\"geoValid\":" + String(pair.geoValid?"true":"false") + ",";
  j += "\"z\":" + String(pair.zCm,2) + ",";
  j += "\"x\":" + String(pair.xCm,2) + ",";
  j += "\"delta\":" + String(pair.deltaCm,2) + ",";
  j += "\"velocity\":" + String(pair.velocityCmS,2) + ",";
  j += "\"coherence\":" + String(pair.coherencePct,1) + ",";
  j += "\"armed\":" + String(pair.armed?"true":"false") + ",";
  j += "\"hits\":" + String(pair.hits) + ",";
  j += "\"lastHitMs\":" + String(pair.lastHitMs) + ",";
  j += "\"lastHitVelocity\":" + String(pair.lastHitVelocity,1) + ",";
  j += "\"lastMidi\":" + String(pair.lastHitMidi) + ",";
  j += "\"lastZone\":\"" + pair.lastZone + "\",";
  j += "\"motion\":\"" + motionLabel() + "\"},";
  j += "\"cfg\":{";
  j += "\"baseline\":" + String(baselineCm,2) + ",";
  j += "\"hitPlane\":" + String(hitPlaneCm,2) + ",";
  j += "\"hitSpeed\":" + String(hitSpeedCmS,1) + ",";
  j += "\"coneHalfAngle\":" + String(coneHalfAngleDeg,1) + "}";
  j += "}";
  server.send(200,"application/json",j);
}

void handleConfig() {
  if (server.hasArg("b")) baselineCm = clampf(server.arg("b").toFloat(),0.2f,20.0f);
  if (server.hasArg("p")) hitPlaneCm = clampf(server.arg("p").toFloat(),4.0f,40.0f);
  if (server.hasArg("s")) hitSpeedCmS = clampf(server.arg("s").toFloat(),5.0f,300.0f);
  if (server.hasArg("a")) coneHalfAngleDeg = clampf(server.arg("a").toFloat(),2.0f,30.0f);
  addLog("CFG b="+String(baselineCm,1)+" hit="+String(hitPlaneCm,1)+" speed="+String(hitSpeedCmS,0));
  server.send(200,"text/plain","OK");
}

void setup() {
  Serial.begin(115200);
  delay(100);
  for (uint8_t i=0;i<2;i++) {
    pinMode(PINS[i].echo, INPUT);
    pinMode(PINS[i].trig, OUTPUT);
    digitalWrite(PINS[i].trig, LOW);
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID);
  server.on("/", [](){ server.send_P(200,"text/html",PAGE); });
  server.on("/api/status", sendStatus);
  server.on("/api/config", handleConfig);
  server.on("/api/reset", [](){ pair.hits=0;pair.lastHitMs=0;pair.lastHitVelocity=0;pair.lastHitMidi=0;pair.lastZone="--";pair.peakApproachCmS=0;pair.armed=true;server.send(200,"text/plain","OK"); });
  server.on("/api/log", [](){ server.send(200,"text/plain",logBuffer); });
  server.begin();

  nextMeasureMs = millis() + 300;
  addLog("BOOT " + String(FW_VERSION));
  addLog("ONE BONGO / 2 HC-SR04 / baseline 1cm / GPIO13-16");
  addLog("AP " + String(AP_SSID) + " http://192.168.4.1");
}

void loop() {
  server.handleClient();
  serviceSensors();
  delay(1);
}
