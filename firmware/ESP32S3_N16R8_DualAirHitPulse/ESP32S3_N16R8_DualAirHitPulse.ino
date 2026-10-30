/*
  SONIC BONGO / PRISMA
  ESP32-S3 N16R8 Dual Air-Hit Lab v9.1

  Recovery build based on the acquisition method that already worked with one HC-SR04:
  pulseIn() on ECHO, with TRIG direct from ESP32 and ECHO only through TXS0108E.

  WIRING
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

  Acquisition
    S1 and S2 alternate every 35 ms.
    Each sensor is measured about every 70 ms (~14.3 Hz).
    The combined stream receives a new sample about every 35 ms (~28.6 Hz)
    when both sensors are valid.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <esp_system.h>
#include <math.h>

WebServer server(80);

static const char* FW_VERSION = "9.1-dual-pulsein";
static const char* AP_SSID = "PRISMA-S3-DIAG";
static const uint32_t SLOT_MS = 35;
static const uint32_t ECHO_TIMEOUT_US = 25000;
static const uint32_t HEALTH_MS = 10000;
static const float MOTION_THRESHOLD_CM_S = 18.0f;
static const float HIT_NEAR_CM = 18.0f;
static const float HIT_SPEED_CM_S = 65.0f;
static const uint32_t HIT_REARM_MS = 280;

struct SensorPins { uint8_t trig; uint8_t echo; };
const SensorPins SENSOR[2] = {{13,14},{15,16}};

struct SensorState {
  bool have = false;
  bool ok = false;
  uint32_t ms = 0;
  uint32_t echoUs = 0;
  float distanceCm = 0;
  float prevDistanceCm = 0;
  uint32_t prevMs = 0;
  float velocityCmS = 0;
  float accelerationCmS2 = 0;
  float peakApproachCmS = 0;
  uint32_t samples = 0;
  uint32_t valid = 0;
  uint32_t timeouts = 0;
  uint32_t hits = 0;
  uint32_t lastHitMs = 0;
};
SensorState state[2];

struct FusedState {
  bool ok = false;
  uint8_t source = 0;
  uint32_t ms = 0;
  float distanceCm = 0;
  float prevDistanceCm = 0;
  uint32_t prevMs = 0;
  float velocityCmS = 0;
  uint32_t samples = 0;
};
FusedState fused;

bool acquisitionRunning = true;
uint8_t nextSensor = 0;
uint32_t nextSlotMs = 0;
uint32_t lastHealthMs = 0;
String logBuffer;

void addLog(const String& s) {
  String line = "[" + String(millis()) + " ms] " + s + "\n";
  logBuffer += line;
  if (logBuffer.length() > 16000) logBuffer.remove(0, logBuffer.length() - 14000);
  Serial.print(line);
}

String motionLabel(uint8_t i) {
  if (!state[i].ok) return "TIMEOUT";
  if (state[i].velocityCmS <= -MOTION_THRESHOLD_CM_S) return "ACERCANDO";
  if (state[i].velocityCmS >= MOTION_THRESHOLD_CM_S) return "ALEJANDO";
  return "ESTABLE";
}

void updateFused(uint8_t source, float d, uint32_t now) {
  if (fused.ok && now > fused.prevMs) {
    float dt = (now - fused.prevMs) / 1000.0f;
    if (dt > 0.015f && dt < 0.20f) {
      float rawV = (d - fused.prevDistanceCm) / dt;
      fused.velocityCmS = 0.45f * rawV + 0.55f * fused.velocityCmS;
    }
  }
  fused.ok = true;
  fused.source = source;
  fused.ms = now;
  fused.distanceCm = d;
  fused.prevDistanceCm = d;
  fused.prevMs = now;
  fused.samples++;
}

void processValid(uint8_t i, uint32_t pulse) {
  SensorState &s = state[i];
  uint32_t now = millis();
  float d = ((float)pulse * 0.0343f) / 2.0f;

  s.samples++;
  s.valid++;
  s.ok = true;
  s.ms = now;
  s.echoUs = pulse;
  s.distanceCm = d;

  if (s.have && now > s.prevMs) {
    float dt = (now - s.prevMs) / 1000.0f;
    if (dt > 0.035f && dt < 0.35f) {
      float rawV = (d - s.prevDistanceCm) / dt;
      float oldV = s.velocityCmS;
      s.velocityCmS = 0.50f * rawV + 0.50f * s.velocityCmS;
      float rawA = (s.velocityCmS - oldV) / dt;
      s.accelerationCmS2 = 0.30f * rawA + 0.70f * s.accelerationCmS2;
      if (s.velocityCmS < 0 && -s.velocityCmS > s.peakApproachCmS) s.peakApproachCmS = -s.velocityCmS;
      if (d <= HIT_NEAR_CM && s.velocityCmS <= -HIT_SPEED_CM_S && now - s.lastHitMs >= HIT_REARM_MS) {
        s.hits++;
        s.lastHitMs = now;
        addLog("S" + String(i+1) + " HIT est d=" + String(d,1) + " v=" + String(s.velocityCmS,1));
      }
    }
  } else {
    s.have = true;
  }

  s.prevDistanceCm = d;
  s.prevMs = now;
  updateFused(i, d, now);
}

void processTimeout(uint8_t i) {
  SensorState &s = state[i];
  s.samples++;
  s.timeouts++;
  s.ok = false;
  s.ms = millis();
  s.echoUs = 0;
}

void measureOne(uint8_t i) {
  const SensorPins &p = SENSOR[i];
  digitalWrite(p.trig, LOW);
  delayMicroseconds(4);
  digitalWrite(p.trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(p.trig, LOW);

  uint32_t pulse = pulseIn(p.echo, HIGH, ECHO_TIMEOUT_US);
  if (pulse > 0) processValid(i, pulse);
  else processTimeout(i);
}

void serviceSensors() {
  if (!acquisitionRunning) return;
  uint32_t now = millis();
  if ((int32_t)(now - nextSlotMs) < 0) return;

  uint8_t i = nextSensor;
  nextSensor = (nextSensor + 1) & 1;
  measureOne(i);
  nextSlotMs = millis() + SLOT_MS;
}

String pairState() {
  if (!state[0].ok && !state[1].ok) return "SIN LECTURAS";
  if (!state[0].ok) return "SOLO S2";
  if (!state[1].ok) return "SOLO S1";
  bool a = state[0].velocityCmS <= -MOTION_THRESHOLD_CM_S;
  bool b = state[1].velocityCmS <= -MOTION_THRESHOLD_CM_S;
  if (a && b) return "AMBOS ACERCANDO";
  if (a) return "S1 ACERCANDO";
  if (b) return "S2 ACERCANDO";
  return "ESTABLE / RETIRADA";
}

const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html lang="es"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>SONIC BONGO v9.1</title>
<style>body{margin:0;background:#091018;color:#eef5fb;font-family:system-ui}header,.wrap{max-width:980px;margin:auto;padding:14px}.sub{color:#90a2b4;font-size:12px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:10px}.card{background:#111923;border:1px solid #2c3b49;border-radius:12px;padding:12px}.dist{font-size:36px;font-weight:900;color:#68f0d1}.row{display:flex;justify-content:space-between;margin:6px 0;font-size:12px}.row b{font-size:14px}.fused{font-size:32px;font-weight:900;color:#ffcf6e}.ok{color:#45ef89}.bad{color:#ff6262}.btns{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}button,a{padding:10px 12px;border-radius:9px;border:1px solid #40566c;background:#182635;color:#fff;font-weight:800;text-decoration:none}.wire{font:11px/1.6 monospace;background:#05090d;padding:10px;border-radius:9px;margin-top:12px}pre{white-space:pre-wrap;background:#05090d;padding:10px;border-radius:9px;max-height:250px;overflow:auto}</style></head><body>
<header><h2>SONIC BONGO · DUAL AIR-HIT LAB v9.1</h2><div class="sub">Método probado pulseIn · 2 HC-SR04 alternados · stream combinado</div></header><div class="wrap">
<div class="grid"><div class="card"><h3>S1 · GPIO13/14</h3><div class="dist" id="s1d">--</div><div class="row"><span>Estado</span><b id="s1m">--</b></div><div class="row"><span>Velocidad</span><b id="s1v">--</b></div><div class="row"><span>Echo</span><b id="s1e">--</b></div><div class="row"><span>OK / timeout</span><b id="s1q">--</b></div></div>
<div class="card"><h3>S2 · GPIO15/16</h3><div class="dist" id="s2d">--</div><div class="row"><span>Estado</span><b id="s2m">--</b></div><div class="row"><span>Velocidad</span><b id="s2v">--</b></div><div class="row"><span>Echo</span><b id="s2e">--</b></div><div class="row"><span>OK / timeout</span><b id="s2q">--</b></div></div></div>
<div class="card" style="margin-top:10px"><h3>STREAM COMBINADO</h3><div class="fused" id="fd">--</div><div class="row"><span>Fuente última muestra</span><b id="fs">--</b></div><div class="row"><span>Velocidad combinada</span><b id="fv">--</b></div><div class="row"><span>Estado del par</span><b id="pair">--</b></div><div class="row"><span>Δ distancia S1-S2</span><b id="delta">--</b></div><div class="row"><span>Muestras combinadas</span><b id="fc">0</b></div><div class="sub">Con ambos válidos, llega una muestra nueva aproximadamente cada 35 ms (~28,6 Hz efectivos del par).</div></div>
<div class="btns"><button onclick="run(1)">INICIAR</button><button onclick="run(0)">DETENER</button><a href="/api/export">EXPORTAR TXT</a></div>
<div class="wire">S1: GPIO13 → TRIG directo · ECHO → TXS B1 → A1 → GPIO14<br>S2: GPIO15 → TRIG directo · ECHO → TXS B2 → A2 → GPIO16<br>TXS: VCCA=3.3V · VCCB=5V · OE=3.3V · GND común</div><pre id="log">...</pre></div>
<script>function f(v,n=1){return Number(v||0).toFixed(n)}async function refresh(){try{let r=await fetch('/api/status',{cache:'no-store'}),j=await r.json();for(let i=1;i<=2;i++){let s=j['s'+i];document.getElementById('s'+i+'d').textContent=s.ok?f(s.distance)+' cm':'TIMEOUT';document.getElementById('s'+i+'m').textContent=s.motion;document.getElementById('s'+i+'v').textContent=f(s.velocity)+' cm/s';document.getElementById('s'+i+'e').textContent=s.echoUs+' us';document.getElementById('s'+i+'q').textContent=s.valid+' / '+s.timeouts}fd.textContent=j.fused.ok?f(j.fused.distance)+' cm':'--';fs.textContent=j.fused.ok?'S'+j.fused.source:'--';fv.textContent=f(j.fused.velocity)+' cm/s';pair.textContent=j.pair;delta.textContent=f(j.delta)+' cm';fc.textContent=j.fused.samples}catch(e){}}async function run(v){await fetch('/api/run?on='+v);refresh()}async function loadLog(){try{let r=await fetch('/api/log',{cache:'no-store'});log.textContent=await r.text()}catch(e){}}setInterval(refresh,200);setInterval(loadLog,1500);refresh();loadLog();</script></body></html>
)HTML";

void handleStatus() {
  String j="{";
  j += "\"running\":" + String(acquisitionRunning?"true":"false") + ",";
  for(uint8_t i=0;i<2;i++){
    SensorState &s=state[i];
    j += "\"s"+String(i+1)+"\":{";
    j += "\"ok\":"+String(s.ok?"true":"false")+",";
    j += "\"distance\":"+String(s.distanceCm,2)+",";
    j += "\"velocity\":"+String(s.velocityCmS,2)+",";
    j += "\"echoUs\":"+String(s.echoUs)+",";
    j += "\"valid\":"+String(s.valid)+",";
    j += "\"timeouts\":"+String(s.timeouts)+",";
    j += "\"motion\":\""+motionLabel(i)+"\"}";
    if(i==0) j += ",";
  }
  float dd=(state[0].ok&&state[1].ok)?state[0].distanceCm-state[1].distanceCm:0;
  j += ",\"pair\":\""+pairState()+"\",";
  j += "\"delta\":"+String(dd,2)+",";
  j += "\"fused\":{";
  j += "\"ok\":"+String(fused.ok?"true":"false")+",";
  j += "\"source\":"+String((int)fused.source+1)+",";
  j += "\"distance\":"+String(fused.distanceCm,2)+",";
  j += "\"velocity\":"+String(fused.velocityCmS,2)+",";
  j += "\"samples\":"+String(fused.samples)+"}";
  j += "}";
  server.send(200,"application/json",j);
}

void handleRun(){
  if(server.hasArg("on")) acquisitionRunning=server.arg("on")!="0";
  nextSlotMs=millis();
  addLog(acquisitionRunning?"Acquisition START":"Acquisition STOP");
  server.send(200,"text/plain","OK");
}

void handleExport(){
  String t="SONIC BONGO DUAL AIR-HIT v9.1\n";
  t += "FW="+String(FW_VERSION)+"\n";
  for(uint8_t i=0;i<2;i++){
    t += "S"+String(i+1)+" distance="+String(state[i].distanceCm,2)+" cm; velocity="+String(state[i].velocityCmS,2)+" cm/s; valid="+String(state[i].valid)+"; timeouts="+String(state[i].timeouts)+"\n";
  }
  t += "fused distance="+String(fused.distanceCm,2)+" cm; velocity="+String(fused.velocityCmS,2)+" cm/s; samples="+String(fused.samples)+"\n\n"+logBuffer;
  server.sendHeader("Content-Disposition","attachment; filename=sonic_bongo_v9_1.txt");
  server.send(200,"text/plain",t);
}

void setup(){
  Serial.begin(115200);
  delay(100);
  for(uint8_t i=0;i<2;i++){
    pinMode(SENSOR[i].trig,OUTPUT);
    digitalWrite(SENSOR[i].trig,LOW);
    pinMode(SENSOR[i].echo,INPUT);
  }
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID);
  server.on("/",[](){server.send_P(200,"text/html",PAGE);});
  server.on("/api/status",handleStatus);
  server.on("/api/run",handleRun);
  server.on("/api/log",[](){server.send(200,"text/plain",logBuffer);});
  server.on("/api/export",handleExport);
  server.begin();
  nextSlotMs=millis()+100;
  addLog("BOOT "+String(FW_VERSION));
  addLog("S1 13->TRIG direct, ECHO->B1/A1->14");
  addLog("S2 15->TRIG direct, ECHO->B2/A2->16");
}

void loop(){
  server.handleClient();
  serviceSensors();
  uint32_t now=millis();
  if(now-lastHealthMs>=HEALTH_MS){
    lastHealthMs=now;
    addLog("HEALTH S1="+String(state[0].valid)+"/"+String(state[0].timeouts)+" S2="+String(state[1].valid)+"/"+String(state[1].timeouts)+" fused="+String(fused.samples));
  }
  delay(1);
}
