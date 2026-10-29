/*
  SONIC BONGO / PRISMA
  ESP32-S3 N16R8 Diagnostic v6

  SENSOR 1 through TXS0108E:
    GPIO13 -> A1 -> B1 -> HC-SR04 TRIG
    GPIO14 <- A2 <- B2 <- HC-SR04 ECHO

  TXS0108E LOOPBACK DIAGNOSTIC:
    GPIO15 <-> A3/B3 -- JUMPER B3<->B4 -- B4/A4 <-> GPIO16

  The loopback test drives GPIO15/GPIO16 only while the test is running.
  Both pins return to INPUT afterwards.

  Power:
    TXS VCCA -> 3.3V
    TXS VCCB -> 5V
    TXS OE   -> 3.3V
    TXS GND  -> common GND

  Wi-Fi AP:
    SSID: PRISMA-S3-DIAG
    Password: OPEN
    Web: http://192.168.4.1
*/

#include <WiFi.h>
#include <WebServer.h>
#include <esp_system.h>

WebServer server(80);

static const char* FW_VERSION = "6.0";
static const char* AP_SSID = "PRISMA-S3-DIAG";
static const uint8_t ANALOG_PIN = 1;
static const uint8_t S1_TRIG_PIN = 13;
static const uint8_t S1_ECHO_PIN = 14;
static const uint8_t TXS_A3_GPIO = 15;
static const uint8_t TXS_A4_GPIO = 16;
static const uint32_t S1_TIMEOUT_US = 30000;
static const uint32_t LOG_SAMPLE_MS = 10000;
static const size_t MAX_LOG_CHARS = 11000;

const uint8_t MONITOR_PINS[] = {
  2,
  4, 5, 6, 7,
  8, 9, 10, 11, 12,
  17, 18,
  21,
  39, 40, 41, 42,
  47
};
const size_t MONITOR_PIN_COUNT = sizeof(MONITOR_PINS) / sizeof(MONITOR_PINS[0]);

struct SensorSample {
  uint32_t ms;
  uint32_t durationUs;
  float distanceCm;
  bool ok;
};

struct TxsDiagResult {
  bool hasRun;
  bool forwardOk;
  bool reverseOk;
  uint8_t forwardPass;
  uint8_t reversePass;
  uint8_t total;
  uint32_t ms;
};

static const uint8_t SENSOR_HISTORY_SIZE = 24;
SensorSample sensorHistory[SENSOR_HISTORY_SIZE];
uint8_t sensorHistoryCount = 0;
uint8_t sensorHistoryHead = 0;
SensorSample lastSensor = {0, 0, 0.0f, false};
TxsDiagResult lastTxs = {false, false, false, 0, 0, 0, 0};

String runtimeLog;
uint32_t lastPeriodicLog = 0;

void addLog(const String& message) {
  String line;
  line.reserve(message.length() + 32);
  line += "[";
  line += String(millis());
  line += " ms] ";
  line += message;
  line += "\n";
  runtimeLog += line;

  if (runtimeLog.length() > MAX_LOG_CHARS) {
    size_t cut = runtimeLog.length() - MAX_LOG_CHARS;
    int nl = runtimeLog.indexOf('\n', cut);
    if (nl >= 0) runtimeLog.remove(0, nl + 1);
    else runtimeLog.remove(0, cut);
  }
  Serial.print(line);
}

String resetReasonText(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_UNKNOWN:   return "UNKNOWN";
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXTERNAL";
    case ESP_RST_SW:        return "SOFTWARE";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                 return String((int)reason);
  }
}

void readAdcStats(uint16_t samples, uint16_t &minV, uint16_t &maxV, uint32_t &avgV) {
  minV = 4095;
  maxV = 0;
  uint32_t sum = 0;
  for (uint16_t i = 0; i < samples; i++) {
    uint16_t v = analogRead(ANALOG_PIN);
    if (v < minV) minV = v;
    if (v > maxV) maxV = v;
    sum += v;
    delayMicroseconds(120);
  }
  avgV = samples ? (sum / samples) : 0;
}

void storeSensorSample(const SensorSample& s) {
  sensorHistory[sensorHistoryHead] = s;
  sensorHistoryHead = (sensorHistoryHead + 1) % SENSOR_HISTORY_SIZE;
  if (sensorHistoryCount < SENSOR_HISTORY_SIZE) sensorHistoryCount++;
  lastSensor = s;
}

SensorSample measureSensor1(bool logResult = true) {
  SensorSample s;
  s.ms = millis();
  s.durationUs = 0;
  s.distanceCm = 0.0f;
  s.ok = false;

  digitalWrite(S1_TRIG_PIN, LOW);
  delayMicroseconds(4);
  digitalWrite(S1_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(S1_TRIG_PIN, LOW);

  uint32_t duration = pulseIn(S1_ECHO_PIN, HIGH, S1_TIMEOUT_US);
  s.durationUs = duration;
  if (duration > 0) {
    s.distanceCm = ((float)duration * 0.0343f) / 2.0f;
    s.ok = true;
  }

  storeSensorSample(s);
  if (logResult) {
    if (s.ok) addLog("S1 OK echo=" + String(s.durationUs) + " us distance=" + String(s.distanceCm, 2) + " cm");
    else addLog("S1 TIMEOUT: no ECHO HIGH detected within " + String(S1_TIMEOUT_US) + " us");
  }
  return s;
}

uint8_t runTxsDirection(uint8_t outPin, uint8_t inPin, const char* label) {
  static const uint8_t pattern[] = {0,1,0,1,1,0,1,0,0,1,1,0};
  const uint8_t total = sizeof(pattern) / sizeof(pattern[0]);
  uint8_t pass = 0;

  pinMode(inPin, INPUT);
  pinMode(outPin, OUTPUT);
  digitalWrite(outPin, LOW);
  delay(3);

  for (uint8_t i = 0; i < total; i++) {
    digitalWrite(outPin, pattern[i] ? HIGH : LOW);
    delayMicroseconds(1500);
    int got = digitalRead(inPin) ? 1 : 0;
    if (got == pattern[i]) pass++;
  }

  digitalWrite(outPin, LOW);
  delayMicroseconds(500);
  pinMode(outPin, INPUT);
  pinMode(inPin, INPUT);

  addLog(String("TXS ") + label + " pass=" + String(pass) + "/" + String(total));
  return pass;
}

TxsDiagResult runTxsLoopback() {
  TxsDiagResult r;
  r.hasRun = true;
  r.total = 12;
  r.ms = millis();

  // Forward: GPIO15 -> A3 -> B3 -> jumper -> B4 -> A4 -> GPIO16
  r.forwardPass = runTxsDirection(TXS_A3_GPIO, TXS_A4_GPIO, "FWD GPIO15/A3/B3 -> B4/A4/GPIO16");
  delay(5);

  // Reverse: GPIO16 -> A4 -> B4 -> jumper -> B3 -> A3 -> GPIO15
  r.reversePass = runTxsDirection(TXS_A4_GPIO, TXS_A3_GPIO, "REV GPIO16/A4/B4 -> B3/A3/GPIO15");

  r.forwardOk = (r.forwardPass == r.total);
  r.reverseOk = (r.reversePass == r.total);
  lastTxs = r;

  if (r.forwardOk && r.reverseOk) addLog("TXS LOOPBACK PASS: bidirectional translation path OK");
  else addLog("TXS LOOPBACK FAIL: check B3-B4 jumper, VCCA/VCCB/OE/GND and A3/A4 wiring");

  return r;
}

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="es">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SONIC BONGO · Diagnostic v6</title>
<style>
:root{--bg:#080c11;--panel:#111923;--line:#2b3948;--text:#edf4fa;--muted:#8fa2b5;--ok:#45ef89;--bad:#ff6262;--sensor:#68f0d1;--txs:#75a7ff;--warn:#ffbf47}
*{box-sizing:border-box}body{margin:0;background:linear-gradient(180deg,#080c11,#0e1620);color:var(--text);font-family:system-ui,-apple-system,"Segoe UI",sans-serif;min-height:100vh}
header,.stats,.panel,.notice,.actions{max-width:980px;margin-left:auto;margin-right:auto}header{padding:18px 16px 6px}h1{font-size:22px;margin:0 0 5px}.sub{font-size:12px;color:var(--muted)}
.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(125px,1fr));gap:8px;padding:8px 16px}.card,.panel{background:var(--panel);border:1px solid var(--line);border-radius:11px}.card{padding:9px 10px}.k{font-size:10px;color:var(--muted);letter-spacing:.08em}.v{font-size:15px;font-weight:800;margin-top:3px}.ok{color:var(--ok)}.bad{color:var(--bad)}
.notice{margin-top:8px;padding:10px 12px;border:1px solid #594820;background:#211b10;color:#ffda8b;border-radius:10px;font-size:12px}.panel{margin-top:12px;padding:14px 16px}.panel h2{font-size:17px;margin:0 0 8px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:8px}.value{background:#091018;border:1px solid var(--line);border-radius:9px;padding:10px}.big{font-size:23px;font-weight:900;margin-top:4px}.sensor .big{color:var(--sensor)}.txs .big{color:var(--txs)}
.wiring{margin-top:10px;background:#060b10;border:1px solid var(--line);border-radius:8px;padding:9px;font:11px/1.65 ui-monospace,Consolas,monospace;color:#bdcad6}.actions{display:flex;flex-wrap:wrap;gap:8px;padding:12px 16px 4px}button,a.btn{border:1px solid #40566c;background:#182635;color:#edf4fa;padding:10px 12px;border-radius:9px;font-weight:800;font-size:12px;text-decoration:none;cursor:pointer}.sensorBtn{border-color:#277762;background:#12382f}.txsBtn{border-color:#446db3;background:#152b4e}.stop{border-color:#743c3c;background:#351919}pre{white-space:pre-wrap;word-break:break-word;background:#05090d;border:1px solid var(--line);border-radius:9px;padding:10px;max-height:330px;overflow:auto;color:#b9c8d6;font-size:11px}.hidden{display:none}.result{font-weight:900}.hint{font-size:11px;color:var(--muted);margin-top:8px}
</style>
</head>
<body>
<header><h1>SONIC BONGO · DIAGNÓSTICO v6</h1><div class="sub">ESP32-S3 N16R8 · HC-SR04 Sensor 1 + autodiagnóstico TXS0108E</div></header>
<section class="stats">
<div class="card"><div class="k">CHIP</div><div class="v" id="chip">...</div></div>
<div class="card"><div class="k">FLASH</div><div class="v" id="flash">...</div></div>
<div class="card"><div class="k">PSRAM</div><div class="v" id="psram">...</div></div>
<div class="card"><div class="k">RESET</div><div class="v" id="resetCause">...</div></div>
<div class="card"><div class="k">HEAP</div><div class="v" id="heap">...</div></div>
<div class="card"><div class="k">WEB</div><div class="v" id="link">CONECTANDO</div></div>
</section>
<div class="notice"><b>TXS loopback:</b> para esa prueba conectá únicamente un puente B3 ↔ B4. A3 debe ir a GPIO15 y A4 a GPIO16. No conectes un Sensor 2 a esos canales durante la prueba.</div>
<section class="panel txs">
<h2>TXS0108E · LOOPBACK BIDIRECCIONAL</h2>
<div class="grid">
<div class="value"><div class="k">RESULTADO</div><div class="big" id="txsStatus">SIN PRUEBA</div></div>
<div class="value"><div class="k">IDA</div><div class="big" id="txsFwd">--</div></div>
<div class="value"><div class="k">VUELTA</div><div class="big" id="txsRev">--</div></div>
</div>
<div class="actions" style="padding-left:0"><button class="txsBtn" onclick="testTxs()">PROBAR TXS LOOPBACK</button></div>
<div class="wiring">GPIO15 ↔ A3 / B3 ─────┐<br>                         ├── puente B3 ↔ B4<br>GPIO16 ↔ A4 / B4 ─────┘<br><br>TXS: VCCA=3.3V · VCCB=5V · OE=3.3V · GND=común</div>
<div class="hint">PASS en ambos sentidos confirma el funcionamiento de alimentación/OE y el camino de traducción usando los canales 3 y 4. No garantiza por sí solo que A1/A2 estén cableados correctamente.</div>
</section>
<section class="panel sensor">
<h2>HC-SR04 · SENSOR 1</h2>
<div class="grid">
<div class="value"><div class="k">ESTADO</div><div class="big" id="sensorStatus">SIN PRUEBA</div></div>
<div class="value"><div class="k">DISTANCIA</div><div class="big" id="distance">-- cm</div></div>
<div class="value"><div class="k">ECHO</div><div class="big" id="echoUs">-- µs</div></div>
<div class="value"><div class="k">VÁLIDAS / TOTAL</div><div class="big" id="validCount">--</div></div>
</div>
<div class="actions" style="padding-left:0"><button class="sensorBtn" onclick="testSensor(1)">PROBAR 1 MEDICIÓN</button><button class="sensorBtn" onclick="testSensor(5)">PROBAR 5 MEDICIONES</button><button class="sensorBtn" id="autoBtn" onclick="toggleAuto()">AUTO: INICIAR</button><button class="stop" onclick="stopAuto()">DETENER</button></div>
<div class="wiring">GPIO13 → A1 → B1 → HC-SR04 TRIG<br>GPIO14 ← A2 ← B2 ← HC-SR04 ECHO<br>HC-SR04: VCC=5V · GND=común</div>
</section>
<div class="actions"><a class="btn" href="/api/export">EXPORTAR DIAGNÓSTICO (.TXT)</a><button onclick="toggleLog()">VER / OCULTAR LOG</button><button onclick="loadLog()">ACTUALIZAR LOG</button></div>
<section class="panel hidden" id="logPanel"><h2>LOG</h2><pre id="logText">Cargando...</pre></section>
<script>
let autoTimer=null;
async function refresh(){try{const d=await (await fetch('/api/status',{cache:'no-store'})).json();chip.textContent=d.chip;flash.textContent=d.flashMB+' MB';psram.textContent=d.psramMB+' MB';resetCause.textContent=d.resetCause;heap.textContent=Math.round(d.freeHeap/1024)+' KB';link.textContent='ONLINE';link.className='v ok';flash.className='v '+(d.flashMB===16?'ok':'bad');psram.className='v '+(d.psramMB===8?'ok':'bad');if(d.txs.hasRun){txsFwd.textContent=d.txs.forwardPass+'/'+d.txs.total;txsRev.textContent=d.txs.reversePass+'/'+d.txs.total;if(d.txs.forwardOk&&d.txs.reverseOk){txsStatus.textContent='PASS';txsStatus.className='big ok'}else{txsStatus.textContent='FAIL';txsStatus.className='big bad'}}}catch(e){link.textContent='SIN RESPUESTA';link.className='v bad'}}
async function testTxs(){txsStatus.textContent='PROBANDO...';txsStatus.className='big';try{const d=await (await fetch('/api/txs',{cache:'no-store'})).json();txsFwd.textContent=d.forwardPass+'/'+d.total;txsRev.textContent=d.reversePass+'/'+d.total;if(d.forwardOk&&d.reverseOk){txsStatus.textContent='PASS';txsStatus.className='big ok'}else{txsStatus.textContent='FAIL';txsStatus.className='big bad'}if(!logPanel.classList.contains('hidden'))loadLog()}catch(e){txsStatus.textContent='ERROR WEB';txsStatus.className='big bad'}}
async function testSensor(count){sensorStatus.textContent='MIDIENDO...';sensorStatus.className='big';try{const d=await (await fetch('/api/sensor1?count='+count,{cache:'no-store'})).json();validCount.textContent=d.valid+'/'+d.total;if(d.valid>0){distance.textContent=d.avgCm.toFixed(1)+' cm';echoUs.textContent=Math.round(d.avgEchoUs)+' µs';sensorStatus.textContent='OK';sensorStatus.className='big ok'}else{distance.textContent='-- cm';echoUs.textContent='-- µs';sensorStatus.textContent='SIN ECHO';sensorStatus.className='big bad'}if(!logPanel.classList.contains('hidden'))loadLog()}catch(e){sensorStatus.textContent='ERROR WEB';sensorStatus.className='big bad'}}
function toggleAuto(){if(autoTimer){stopAuto();return}testSensor(1);autoTimer=setInterval(()=>testSensor(1),900);autoBtn.textContent='AUTO: ACTIVO'}
function stopAuto(){if(autoTimer){clearInterval(autoTimer);autoTimer=null}autoBtn.textContent='AUTO: INICIAR'}
function toggleLog(){logPanel.classList.toggle('hidden');if(!logPanel.classList.contains('hidden'))loadLog()}
async function loadLog(){try{logText.textContent=await (await fetch('/api/log',{cache:'no-store'})).text();logText.scrollTop=logText.scrollHeight}catch(e){logText.textContent='Error leyendo log'}}
refresh();setInterval(refresh,5000);
</script>
</body>
</html>
)HTML";

void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleStatus() {
  String json;
  json.reserve(1500);
  json += "{";
  json += "\"fw\":\"" + String(FW_VERSION) + "\",";
  json += "\"chip\":\"" + String(ESP.getChipModel()) + "\",";
  json += "\"revision\":" + String(ESP.getChipRevision()) + ",";
  json += "\"flashMB\":" + String((uint32_t)(ESP.getFlashChipSize() / (1024 * 1024))) + ",";
  json += "\"psramMB\":" + String((uint32_t)(ESP.getPsramSize() / (1024 * 1024))) + ",";
  json += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
  json += "\"minFreeHeap\":" + String(ESP.getMinFreeHeap()) + ",";
  json += "\"resetCause\":\"" + resetReasonText(esp_reset_reason()) + "\",";
  json += "\"stations\":" + String(WiFi.softAPgetStationNum()) + ",";
  json += "\"uptimeMs\":" + String(millis()) + ",";
  json += "\"adc1\":" + String(analogRead(ANALOG_PIN)) + ",";
  json += "\"sensor1\":{";
  json += "\"hasSample\":" + String(lastSensor.ms ? "true" : "false") + ",";
  json += "\"ok\":" + String(lastSensor.ok ? "true" : "false") + ",";
  json += "\"durationUs\":" + String(lastSensor.durationUs) + ",";
  json += "\"distanceCm\":" + String(lastSensor.distanceCm, 2) + "},";
  json += "\"txs\":{";
  json += "\"hasRun\":" + String(lastTxs.hasRun ? "true" : "false") + ",";
  json += "\"forwardOk\":" + String(lastTxs.forwardOk ? "true" : "false") + ",";
  json += "\"reverseOk\":" + String(lastTxs.reverseOk ? "true" : "false") + ",";
  json += "\"forwardPass\":" + String(lastTxs.forwardPass) + ",";
  json += "\"reversePass\":" + String(lastTxs.reversePass) + ",";
  json += "\"total\":" + String(lastTxs.total) + "}";
  json += "}";
  server.send(200, "application/json", json);
}

void handleSensor1() {
  int requested = 1;
  if (server.hasArg("count")) requested = server.arg("count").toInt();
  if (requested < 1) requested = 1;
  if (requested > 10) requested = 10;

  uint8_t valid = 0;
  float sumCm = 0.0f;
  uint64_t sumEcho = 0;
  float minCm = 99999.0f;
  float maxCm = 0.0f;

  for (int i = 0; i < requested; i++) {
    SensorSample s = measureSensor1(true);
    if (s.ok) {
      valid++;
      sumCm += s.distanceCm;
      sumEcho += s.durationUs;
      if (s.distanceCm < minCm) minCm = s.distanceCm;
      if (s.distanceCm > maxCm) maxCm = s.distanceCm;
    }
    if (i + 1 < requested) delay(70);
  }

  float avgCm = valid ? (sumCm / valid) : 0.0f;
  float avgEcho = valid ? ((float)sumEcho / valid) : 0.0f;
  if (!valid) minCm = 0.0f;

  String json;
  json.reserve(320);
  json += "{";
  json += "\"total\":" + String(requested) + ",";
  json += "\"valid\":" + String(valid) + ",";
  json += "\"avgCm\":" + String(avgCm, 2) + ",";
  json += "\"minCm\":" + String(minCm, 2) + ",";
  json += "\"maxCm\":" + String(maxCm, 2) + ",";
  json += "\"avgEchoUs\":" + String(avgEcho, 1) + ",";
  json += "\"timeoutUs\":" + String(S1_TIMEOUT_US);
  json += "}";
  server.send(200, "application/json", json);
}

void handleTxs() {
  TxsDiagResult r = runTxsLoopback();
  String json;
  json.reserve(220);
  json += "{";
  json += "\"forwardOk\":" + String(r.forwardOk ? "true" : "false") + ",";
  json += "\"reverseOk\":" + String(r.reverseOk ? "true" : "false") + ",";
  json += "\"forwardPass\":" + String(r.forwardPass) + ",";
  json += "\"reversePass\":" + String(r.reversePass) + ",";
  json += "\"total\":" + String(r.total);
  json += "}";
  server.send(200, "application/json", json);
}

void handleLog() {
  server.send(200, "text/plain; charset=utf-8", runtimeLog);
}

void handleExport() {
  uint16_t adcMin, adcMax;
  uint32_t adcAvg;
  readAdcStats(64, adcMin, adcMax, adcAvg);

  String out;
  out.reserve(17000);
  out += "SONIC BONGO / PRISMA - ESP32-S3 N16R8 DIAGNOSTIC\n";
  out += "Firmware: v" + String(FW_VERSION) + "\n";
  out += "============================================================\n";
  out += "Chip: " + String(ESP.getChipModel()) + "\n";
  out += "Revision: " + String(ESP.getChipRevision()) + "\n";
  out += "Flash MB: " + String((uint32_t)(ESP.getFlashChipSize() / (1024 * 1024))) + "\n";
  out += "PSRAM MB: " + String((uint32_t)(ESP.getPsramSize() / (1024 * 1024))) + "\n";
  out += "Reset cause: " + resetReasonText(esp_reset_reason()) + "\n";
  out += "Uptime ms: " + String(millis()) + "\n";
  out += "Free heap: " + String(ESP.getFreeHeap()) + "\n";
  out += "Min free heap: " + String(ESP.getMinFreeHeap()) + "\n";
  out += "AP SSID: " + String(AP_SSID) + "\n";
  out += "AP IP: " + WiFi.softAPIP().toString() + "\n";
  out += "Connected stations: " + String(WiFi.softAPgetStationNum()) + "\n\n";

  out += "TXS0108E LOOPBACK WIRING\n";
  out += "GPIO15 <-> A3/B3 -- jumper B3<->B4 -- B4/A4 <-> GPIO16\n";
  out += "VCCA=3.3V, VCCB=5V, OE=3.3V, GND=common\n";
  if (!lastTxs.hasRun) {
    out += "Result: NOT TESTED\n";
  } else {
    out += "Forward GPIO15->GPIO16: " + String(lastTxs.forwardPass) + "/" + String(lastTxs.total) + (lastTxs.forwardOk ? " PASS\n" : " FAIL\n");
    out += "Reverse GPIO16->GPIO15: " + String(lastTxs.reversePass) + "/" + String(lastTxs.total) + (lastTxs.reverseOk ? " PASS\n" : " FAIL\n");
    out += "Overall: " + String((lastTxs.forwardOk && lastTxs.reverseOk) ? "PASS" : "FAIL") + "\n";
  }

  out += "\nSENSOR 1 WIRING\n";
  out += "GPIO13 -> A1/B1 -> HC-SR04 TRIG\n";
  out += "GPIO14 <- A2/B2 <- HC-SR04 ECHO\n";
  out += "Timeout us: " + String(S1_TIMEOUT_US) + "\n";
  out += "Last sample: ";
  if (lastSensor.ms == 0) out += "NO SAMPLE\n";
  else if (lastSensor.ok) out += "OK, echo=" + String(lastSensor.durationUs) + " us, distance=" + String(lastSensor.distanceCm, 2) + " cm\n";
  else out += "TIMEOUT, no echo\n";

  out += "\nSENSOR 1 HISTORY (oldest -> newest)\n";
  if (sensorHistoryCount == 0) out += "No samples yet.\n";
  else {
    uint8_t start = (sensorHistoryHead + SENSOR_HISTORY_SIZE - sensorHistoryCount) % SENSOR_HISTORY_SIZE;
    for (uint8_t i = 0; i < sensorHistoryCount; i++) {
      const SensorSample &s = sensorHistory[(start + i) % SENSOR_HISTORY_SIZE];
      out += String(s.ms) + " ms | ";
      if (s.ok) out += "OK | " + String(s.durationUs) + " us | " + String(s.distanceCm, 2) + " cm\n";
      else out += "TIMEOUT\n";
    }
  }

  out += "\nGPIO STATES\n";
  out += "GPIO13 TRIG = " + String(digitalRead(S1_TRIG_PIN) ? "HIGH" : "LOW") + "\n";
  out += "GPIO14 ECHO = " + String(digitalRead(S1_ECHO_PIN) ? "HIGH" : "LOW") + "\n";
  out += "GPIO15 TXS A3 = " + String(digitalRead(TXS_A3_GPIO) ? "HIGH" : "LOW") + "\n";
  out += "GPIO16 TXS A4 = " + String(digitalRead(TXS_A4_GPIO) ? "HIGH" : "LOW") + "\n";
  out += "GPIO1 ADC current = " + String(analogRead(ANALOG_PIN)) + "\n";
  out += "GPIO1 ADC 64 samples min/avg/max = " + String(adcMin) + "/" + String(adcAvg) + "/" + String(adcMax) + "\n";
  for (size_t i = 0; i < MONITOR_PIN_COUNT; i++) {
    out += "GPIO" + String(MONITOR_PINS[i]) + " = " + String(digitalRead(MONITOR_PINS[i]) ? "HIGH" : "LOW") + "\n";
  }

  out += "\nRUNTIME LOG\n";
  out += runtimeLog;

  server.sendHeader("Content-Disposition", "attachment; filename=SONIC_BONGO_N16R8_TXS_SENSOR1_DIAG_v6.txt");
  server.send(200, "text/plain; charset=utf-8", out);
}

void handleNotFound() {
  server.send(404, "text/plain; charset=utf-8", "404");
}

void setup() {
  Serial.begin(115200);
  delay(350);

  pinMode(ANALOG_PIN, INPUT);
  analogReadResolution(12);

  for (size_t i = 0; i < MONITOR_PIN_COUNT; i++) pinMode(MONITOR_PINS[i], INPUT);

  pinMode(S1_ECHO_PIN, INPUT);
  pinMode(S1_TRIG_PIN, OUTPUT);
  digitalWrite(S1_TRIG_PIN, LOW);

  pinMode(TXS_A3_GPIO, INPUT);
  pinMode(TXS_A4_GPIO, INPUT);

  addLog("SONIC BONGO ESP32-S3 N16R8 DIAG v6 boot");
  addLog("Reset cause: " + resetReasonText(esp_reset_reason()));
  addLog("Sensor1: GPIO13 TRIG output LOW; GPIO14 ECHO input");
  addLog("TXS loopback ready: GPIO15=A3, GPIO16=A4, requires B3-B4 jumper");

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  bool ok = WiFi.softAP(AP_SSID);
  if (ok) addLog("AP started: " + String(AP_SSID) + " IP=" + WiFi.softAPIP().toString());
  else addLog("ERROR: failed to start Access Point");

  addLog("Chip=" + String(ESP.getChipModel()) + " rev=" + String(ESP.getChipRevision()));
  addLog("Flash=" + String(ESP.getFlashChipSize() / (1024 * 1024)) + "MB PSRAM=" + String(ESP.getPsramSize() / (1024 * 1024)) + "MB");

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/sensor1", HTTP_GET, handleSensor1);
  server.on("/api/txs", HTTP_GET, handleTxs);
  server.on("/api/log", HTTP_GET, handleLog);
  server.on("/api/export", HTTP_GET, handleExport);
  server.onNotFound(handleNotFound);
  server.begin();
  addLog("Web server ready");
}

void loop() {
  server.handleClient();

  uint32_t now = millis();
  if (now - lastPeriodicLog >= LOG_SAMPLE_MS) {
    lastPeriodicLog = now;
    uint16_t mn, mx;
    uint32_t av;
    readAdcStats(16, mn, mx, av);
    addLog("HEALTH heap=" + String(ESP.getFreeHeap()) + " stations=" + String(WiFi.softAPgetStationNum()) + " adc1=" + String(mn) + "/" + String(av) + "/" + String(mx));
  }
  delay(2);
}
