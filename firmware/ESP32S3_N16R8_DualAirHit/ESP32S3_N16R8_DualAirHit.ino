/*
  SONIC BONGO / PRISMA
  ESP32-S3 N16R8 Dual Air-Hit Lab v9

  Two HC-SR04 continuous diagnostic.

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

  Acquisition:
    S1 and S2 are staggered every 35 ms.
    Each individual sensor is triggered every ~70 ms (~14.3 Hz).
*/

#include <WiFi.h>
#include <WebServer.h>
#include <esp_system.h>
#include <math.h>

WebServer server(80);

static const char* FW_VERSION = "9.0-dual-air-hit";
static const char* AP_SSID = "PRISMA-S3-DIAG";

static const uint32_t SLOT_US = 35000;      // alternate S1/S2 every 35ms
static const uint32_t TIMEOUT_US = 28000;
static const uint32_t HEALTH_MS = 10000;
static const float MOTION_THRESHOLD_CM_S = 18.0f;
static const float HIT_NEAR_CM = 18.0f;
static const float HIT_SPEED_CM_S = 65.0f;
static const uint32_t HIT_REARM_MS = 280;

struct SensorPins {
  uint8_t trig;
  uint8_t echo;
  uint8_t txsChannel;
};

const SensorPins SENSOR[2] = {
  {13, 14, 1},
  {15, 16, 2}
};

struct SensorState {
  bool have;
  bool ok;
  uint32_t ms;
  uint32_t echoUs;
  float distanceCm;
  float prevDistanceCm;
  uint32_t prevMs;
  float velocityCmS;
  float accelerationCmS2;
  float peakApproachCmS;
  uint32_t samples;
  uint32_t valid;
  uint32_t timeouts;
  uint32_t hits;
  uint32_t lastHitMs;
  uint32_t lastValidMs;
};

SensorState state[2] = {};

volatile uint32_t echoRiseUs[2] = {0, 0};
volatile uint32_t echoPulseUs[2] = {0, 0};
volatile bool echoDone[2] = {false, false};
volatile bool echoArmed[2] = {false, false};

bool acquisitionRunning = true;
int8_t activeSensor = -1;
uint8_t nextSensor = 0;
uint32_t triggerStartedUs = 0;
uint32_t nextSlotUs = 0;
uint32_t lastHealthMs = 0;
String logBuffer;

void addLog(const String& s) {
  String line = "[" + String(millis()) + " ms] " + s + "\n";
  logBuffer += line;
  if (logBuffer.length() > 16000) {
    int cut = logBuffer.indexOf('\n', logBuffer.length() - 14000);
    if (cut >= 0) logBuffer.remove(0, cut + 1);
  }
  Serial.print(line);
}

String resetReasonText() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_SW: return "SOFTWARE";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    default: return String((int)esp_reset_reason());
  }
}

void echoEdge(uint8_t i) {
  if (!echoArmed[i]) return;
  uint32_t now = micros();
  int level = digitalRead(SENSOR[i].echo);
  if (level) {
    echoRiseUs[i] = now;
  } else if (echoRiseUs[i] != 0) {
    echoPulseUs[i] = now - echoRiseUs[i];
    echoDone[i] = true;
    echoArmed[i] = false;
  }
}

void echoISR0() { echoEdge(0); }
void echoISR1() { echoEdge(1); }

String motionLabel(uint8_t i) {
  if (!state[i].ok) return "TIMEOUT";
  float v = state[i].velocityCmS;
  if (v <= -MOTION_THRESHOLD_CM_S) return "ACERCANDO";
  if (v >= MOTION_THRESHOLD_CM_S) return "ALEJANDO";
  return "ESTABLE";
}

void processMeasurement(uint8_t i, uint32_t pulse, bool ok) {
  SensorState &s = state[i];
  uint32_t now = millis();
  s.samples++;
  s.ms = now;
  s.echoUs = pulse;
  s.ok = ok && pulse > 0;

  if (!s.ok) {
    s.timeouts++;
    return;
  }

  float d = ((float)pulse * 0.0343f) / 2.0f;
  s.distanceCm = d;
  s.valid++;
  s.lastValidMs = now;

  if (s.have && now > s.prevMs) {
    float dt = (now - s.prevMs) / 1000.0f;
    if (dt > 0.035f && dt < 0.35f) {
      float rawV = (d - s.prevDistanceCm) / dt;
      float oldV = s.velocityCmS;
      // Mild low-pass: enough smoothing to make approach trend readable.
      s.velocityCmS = 0.50f * rawV + 0.50f * s.velocityCmS;
      float rawA = (s.velocityCmS - oldV) / dt;
      s.accelerationCmS2 = 0.30f * rawA + 0.70f * s.accelerationCmS2;

      if (s.velocityCmS < 0 && -s.velocityCmS > s.peakApproachCmS) {
        s.peakApproachCmS = -s.velocityCmS;
      }

      if (d <= HIT_NEAR_CM && s.velocityCmS <= -HIT_SPEED_CM_S && (now - s.lastHitMs) >= HIT_REARM_MS) {
        s.hits++;
        s.lastHitMs = now;
        addLog("S" + String(i + 1) + " HIT est d=" + String(d, 1) + "cm v=" + String(s.velocityCmS, 1) + "cm/s");
      }
    }
  } else {
    s.have = true;
    s.velocityCmS = 0.0f;
    s.accelerationCmS2 = 0.0f;
  }

  s.prevDistanceCm = d;
  s.prevMs = now;
}

void triggerSensor(uint8_t i) {
  noInterrupts();
  echoRiseUs[i] = 0;
  echoPulseUs[i] = 0;
  echoDone[i] = false;
  echoArmed[i] = true;
  interrupts();

  digitalWrite(SENSOR[i].trig, LOW);
  delayMicroseconds(3);
  digitalWrite(SENSOR[i].trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(SENSOR[i].trig, LOW);

  activeSensor = (int8_t)i;
  triggerStartedUs = micros();
}

void serviceSensors() {
  if (!acquisitionRunning) return;

  uint32_t now = micros();
  if (activeSensor >= 0) {
    uint8_t i = (uint8_t)activeSensor;
    bool done;
    uint32_t pulse;
    noInterrupts();
    done = echoDone[i];
    pulse = echoPulseUs[i];
    interrupts();

    if (done) {
      processMeasurement(i, pulse, true);
      activeSensor = -1;
    } else if ((uint32_t)(now - triggerStartedUs) >= TIMEOUT_US) {
      noInterrupts();
      echoArmed[i] = false;
      interrupts();
      processMeasurement(i, 0, false);
      activeSensor = -1;
    }
  }

  if (activeSensor < 0 && (int32_t)(now - nextSlotUs) >= 0) {
    uint8_t i = nextSensor;
    nextSensor = (nextSensor + 1) % 2;
    nextSlotUs = now + SLOT_US;
    triggerSensor(i);
  }
}

String pairState() {
  if (!state[0].ok && !state[1].ok) return "SIN LECTURAS";
  if (!state[0].ok) return "SOLO S2";
  if (!state[1].ok) return "SOLO S1";

  bool a1 = state[0].velocityCmS <= -MOTION_THRESHOLD_CM_S;
  bool a2 = state[1].velocityCmS <= -MOTION_THRESHOLD_CM_S;
  if (a1 && a2) return "AMBOS ACERCANDO";
  if (a1) return "S1 ACERCANDO";
  if (a2) return "S2 ACERCANDO";
  return "ESTABLE / RETIRADA";
}

String leaderState() {
  if (!state[0].ok || !state[1].ok) return "--";
  float delta = state[0].distanceCm - state[1].distanceCm;
  if (fabsf(delta) < 2.0f) return "PAREJOS";
  return delta < 0 ? "S1 MÁS CERCA" : "S2 MÁS CERCA";
}

const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html lang="es"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>SONIC BONGO · Dual Air-Hit Lab</title>
<style>
:root{--bg:#080c11;--p:#111923;--l:#2b3948;--t:#edf4fa;--m:#8fa2b5;--ok:#45ef89;--bad:#ff6262;--a:#68f0d1;--b:#75a7ff;--w:#ffbf47}*{box-sizing:border-box}body{margin:0;background:linear-gradient(180deg,#080c11,#0e1620);color:var(--t);font-family:system-ui,-apple-system,"Segoe UI",sans-serif;min-height:100vh}header,.stats,.panel,.notice,.actions{max-width:1000px;margin-left:auto;margin-right:auto}header{padding:18px 16px 7px}h1{font-size:22px;margin:0}.sub{font-size:12px;color:var(--m)}.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(125px,1fr));gap:8px;padding:8px 16px}.card,.panel{background:var(--p);border:1px solid var(--l);border-radius:11px}.card{padding:9px 10px}.k{font-size:10px;color:var(--m);letter-spacing:.06em}.v{font-size:15px;font-weight:850;margin-top:3px}.notice{padding:10px 12px;border:1px solid #594820;background:#211b10;color:#ffda8b;border-radius:10px;font-size:12px}.panel{margin-top:12px;padding:14px 16px}.sgrid{display:grid;grid-template-columns:repeat(2,minmax(260px,1fr));gap:10px}.sensor{background:#091018;border:1px solid var(--l);border-radius:10px;padding:12px}.sensor.s1{border-color:#276858}.sensor.s2{border-color:#385a94}.sensor h2{margin:0 0 8px;font-size:16px}.dist{font-size:34px;font-weight:950}.s1 .dist{color:var(--a)}.s2 .dist{color:var(--b)}.row{display:flex;justify-content:space-between;gap:8px;margin:6px 0;font-size:12px}.row b{font-size:14px}.pair{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:8px}.box{background:#091018;border:1px solid var(--l);border-radius:9px;padding:10px}.big{font-size:20px;font-weight:900;margin-top:4px}.wire{font:11px/1.65 ui-monospace,Consolas,monospace;background:#060b10;border:1px solid var(--l);padding:10px;border-radius:8px}.actions{display:flex;gap:8px;flex-wrap:wrap;padding:12px 16px}.actions button,.actions a{border:1px solid #40566c;background:#182635;color:var(--t);padding:10px 12px;border-radius:9px;font-weight:800;text-decoration:none}.go{background:#12382f!important}.stop{background:#351919!important}.ok{color:var(--ok)}.bad{color:var(--bad)}pre{white-space:pre-wrap;word-break:break-word;max-height:300px;overflow:auto;background:#05090d;border:1px solid var(--l);padding:10px;border-radius:9px;font-size:11px}@media(max-width:650px){.sgrid{grid-template-columns:1fr}.dist{font-size:30px}}
</style></head><body>
<header><h1>SONIC BONGO · DUAL AIR-HIT LAB v9</h1><div class="sub">2 × HC-SR04 · adquisición continua · llegada comparada</div></header>
<section class="stats"><div class="card"><div class="k">CHIP</div><div class="v" id="chip">...</div></div><div class="card"><div class="k">FLASH</div><div class="v" id="flash">...</div></div><div class="card"><div class="k">PSRAM</div><div class="v" id="psram">...</div></div><div class="card"><div class="k">ADQUISICIÓN</div><div class="v" id="run">...</div></div><div class="card"><div class="k">POR SENSOR</div><div class="v">~14,3 Hz</div></div></section>
<div class="notice"><b>TRIG directo al ESP32.</b> Sólo ECHO atraviesa el TXS0108E. Los sensores se disparan alternados para reducir interferencia acústica.</div>
<section class="panel"><div class="sgrid">
<div class="sensor s1"><h2>SENSOR 1 · GPIO13/14</h2><div class="dist" id="s1d">-- cm</div><div class="row"><span>Movimiento</span><b id="s1m">--</b></div><div class="row"><span>Velocidad</span><b id="s1v">--</b></div><div class="row"><span>Aceleración est.</span><b id="s1a">--</b></div><div class="row"><span>Pico acercamiento</span><b id="s1p">--</b></div><div class="row"><span>Golpes estimados</span><b id="s1h">0</b></div><div class="row"><span>OK / timeout</span><b id="s1q">--</b></div></div>
<div class="sensor s2"><h2>SENSOR 2 · GPIO15/16</h2><div class="dist" id="s2d">-- cm</div><div class="row"><span>Movimiento</span><b id="s2m">--</b></div><div class="row"><span>Velocidad</span><b id="s2v">--</b></div><div class="row"><span>Aceleración est.</span><b id="s2a">--</b></div><div class="row"><span>Pico acercamiento</span><b id="s2p">--</b></div><div class="row"><span>Golpes estimados</span><b id="s2h">0</b></div><div class="row"><span>OK / timeout</span><b id="s2q">--</b></div></div>
</div></section>
<section class="panel"><h2>LECTURA COMBINADA</h2><div class="pair"><div class="box"><div class="k">ESTADO DEL PAR</div><div class="big" id="pair">--</div></div><div class="box"><div class="k">QUIÉN LLEGA MÁS CERCA</div><div class="big" id="leader">--</div></div><div class="box"><div class="k">Δ DISTANCIA S1-S2</div><div class="big" id="delta">-- cm</div></div><div class="box"><div class="k">Δ VELOCIDAD S1-S2</div><div class="big" id="dv">-- cm/s</div></div></div><div style="margin-top:10px;font-size:11px;color:var(--m)">Si luego montamos ambos sensores sobre el mismo parche, estas diferencias nos sirven para estudiar entrada central/lateral y confirmar el gesto con ambos canales.</div></section>
<section class="panel"><h2>CONEXIÓN</h2><div class="wire">S1: GPIO13 → TRIG &nbsp;&nbsp; ECHO → TXS B1 → A1 → GPIO14<br>S2: GPIO15 → TRIG &nbsp;&nbsp; ECHO → TXS B2 → A2 → GPIO16<br><br>HC-SR04: VCC=5V · GND=común<br>TXS0108E: VCCA=3.3V · VCCB=5V · OE=3.3V · GND=común</div></section>
<div class="actions"><button class="go" onclick="setRun(1)">INICIAR CONTINUO</button><button class="stop" onclick="setRun(0)">DETENER</button><button onclick="resetPeaks()">RESET PICOS/GOLPES</button><a href="/api/export">EXPORTAR TXT</a></div>
<section class="panel"><h2>LOG</h2><pre id="log">...</pre></section>
<script>
function f(v,n=1){return Number(v||0).toFixed(n)}
async function refresh(){try{let r=await fetch('/api/status',{cache:'no-store'});let j=await r.json();chip.textContent=j.chip;flash.textContent=j.flashMB+' MB';psram.textContent=j.psramMB+' MB';run.textContent=j.running?'CONTINUA':'DETENIDA';run.className='v '+(j.running?'ok':'bad');for(let i=1;i<=2;i++){let s=j['s'+i];document.getElementById('s'+i+'d').textContent=s.ok?f(s.distance,1)+' cm':'TIMEOUT';document.getElementById('s'+i+'m').textContent=s.motion;document.getElementById('s'+i+'v').textContent=f(s.velocity,1)+' cm/s';document.getElementById('s'+i+'a').textContent=f(s.acceleration,0)+' cm/s²';document.getElementById('s'+i+'p').textContent=f(s.peakApproach,1)+' cm/s';document.getElementById('s'+i+'h').textContent=s.hits;document.getElementById('s'+i+'q').textContent=s.valid+' / '+s.timeouts;}pair.textContent=j.pair.state;leader.textContent=j.pair.leader;delta.textContent=f(j.pair.deltaDistance,1)+' cm';dv.textContent=f(j.pair.deltaVelocity,1)+' cm/s';}catch(e){run.textContent='SIN RESPUESTA';run.className='v bad'}}
async function setRun(v){await fetch('/api/run?on='+v);refresh()}
async function resetPeaks(){await fetch('/api/reset');refresh()}
async function loadLog(){try{let r=await fetch('/api/log',{cache:'no-store'});log.textContent=await r.text()}catch(e){}}
setInterval(refresh,180);setInterval(loadLog,1800);refresh();loadLog();
</script></body></html>
)HTML";

void handleStatus() {
  String j = "{";
  j += "\"fw\":\"" + String(FW_VERSION) + "\",";
  j += "\"chip\":\"ESP32-S3\",";
  j += "\"flashMB\":" + String(ESP.getFlashChipSize() / (1024 * 1024)) + ",";
  j += "\"psramMB\":" + String(ESP.getPsramSize() / (1024 * 1024)) + ",";
  j += "\"running\":" + String(acquisitionRunning ? "true" : "false") + ",";

  for (uint8_t i = 0; i < 2; i++) {
    SensorState &s = state[i];
    j += "\"s" + String(i + 1) + "\":{";
    j += "\"ok\":" + String(s.ok ? "true" : "false") + ",";
    j += "\"distance\":" + String(s.distanceCm, 2) + ",";
    j += "\"velocity\":" + String(s.velocityCmS, 2) + ",";
    j += "\"acceleration\":" + String(s.accelerationCmS2, 1) + ",";
    j += "\"peakApproach\":" + String(s.peakApproachCmS, 2) + ",";
    j += "\"hits\":" + String(s.hits) + ",";
    j += "\"valid\":" + String(s.valid) + ",";
    j += "\"timeouts\":" + String(s.timeouts) + ",";
    j += "\"motion\":\"" + motionLabel(i) + "\"}";
    if (i == 0) j += ",";
  }

  float dd = (state[0].ok && state[1].ok) ? state[0].distanceCm - state[1].distanceCm : 0.0f;
  float dv = state[0].velocityCmS - state[1].velocityCmS;
  j += ",\"pair\":{";
  j += "\"state\":\"" + pairState() + "\",";
  j += "\"leader\":\"" + leaderState() + "\",";
  j += "\"deltaDistance\":" + String(dd, 2) + ",";
  j += "\"deltaVelocity\":" + String(dv, 2) + "}";
  j += "}";
  server.send(200, "application/json", j);
}

void handleRun() {
  if (server.hasArg("on")) acquisitionRunning = server.arg("on") != "0";
  if (acquisitionRunning) {
    nextSlotUs = micros();
    addLog("Continuous acquisition START");
  } else {
    noInterrupts();
    echoArmed[0] = echoArmed[1] = false;
    interrupts();
    activeSensor = -1;
    addLog("Continuous acquisition STOP");
  }
  server.send(200, "text/plain", "OK");
}

void handleReset() {
  for (uint8_t i = 0; i < 2; i++) {
    state[i].peakApproachCmS = 0;
    state[i].hits = 0;
    state[i].lastHitMs = 0;
  }
  addLog("Peaks/hits reset");
  server.send(200, "text/plain", "OK");
}

void handleExport() {
  String t;
  t += "SONIC BONGO DUAL AIR-HIT LAB v9\n";
  t += "FW=" + String(FW_VERSION) + "\n";
  t += "S1 GPIO13 TRIG direct / GPIO14 ECHO via TXS B1-A1\n";
  t += "S2 GPIO15 TRIG direct / GPIO16 ECHO via TXS B2-A2\n";
  t += "slot_ms=35; each_sensor_ms=70; each_sensor_hz~=14.3\n\n";
  for (uint8_t i = 0; i < 2; i++) {
    SensorState &s = state[i];
    t += "S" + String(i + 1) + ": distance=" + String(s.distanceCm, 2) + " cm; v=" + String(s.velocityCmS, 2) + " cm/s; a=" + String(s.accelerationCmS2, 1) + " cm/s2; peak=" + String(s.peakApproachCmS, 2) + " cm/s; hits=" + String(s.hits) + "; valid=" + String(s.valid) + "; timeouts=" + String(s.timeouts) + "\n";
  }
  t += "\nLOG\n" + logBuffer;
  server.sendHeader("Content-Disposition", "attachment; filename=sonic_bongo_dual_air_hit_v9.txt");
  server.send(200, "text/plain", t);
}

void setup() {
  Serial.begin(115200);
  delay(100);

  for (uint8_t i = 0; i < 2; i++) {
    pinMode(SENSOR[i].trig, OUTPUT);
    digitalWrite(SENSOR[i].trig, LOW);
    pinMode(SENSOR[i].echo, INPUT);
  }

  attachInterrupt(digitalPinToInterrupt(SENSOR[0].echo), echoISR0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SENSOR[1].echo), echoISR1, CHANGE);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID);

  server.on("/", [](){ server.send_P(200, "text/html", PAGE); });
  server.on("/api/status", handleStatus);
  server.on("/api/run", handleRun);
  server.on("/api/reset", handleReset);
  server.on("/api/log", [](){ server.send(200, "text/plain", logBuffer); });
  server.on("/api/export", handleExport);
  server.begin();

  nextSlotUs = micros() + 100000;
  addLog("BOOT " + String(FW_VERSION));
  addLog("AP " + String(AP_SSID) + " http://192.168.4.1");
  addLog("S1 GPIO13->TRIG; ECHO->TXS B1/A1->GPIO14");
  addLog("S2 GPIO15->TRIG; ECHO->TXS B2/A2->GPIO16");
}

void loop() {
  server.handleClient();
  serviceSensors();

  uint32_t now = millis();
  if (now - lastHealthMs >= HEALTH_MS) {
    lastHealthMs = now;
    addLog("HEALTH S1=" + String(state[0].distanceCm, 1) + "cm/" + motionLabel(0) + " S2=" + String(state[1].distanceCm, 1) + "cm/" + motionLabel(1));
  }
  delay(1);
}
