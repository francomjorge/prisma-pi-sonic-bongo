/*
  SONIC BONGO / PRISMA
  ESP32-S3 N16R8 Diagnostic v7 - PIN LAB

  Keeps the complete 44-pin board view and adds:
  - Safe single-GPIO pushbutton input test using INPUT_PULLUP.
  - Existing HC-SR04 Sensor 1 diagnostic.
  - Existing TXS0108E bidirectional loopback diagnostic.
  - Diagnostic TXT export and runtime log.

  PUSHBUTTON TEST WIRING:
    Selected safe GPIO ---- pushbutton ---- GND
  No external 3.3 V or 5 V is used for the button test.

  SENSOR 1 through TXS0108E:
    GPIO13 -> A1 -> B1 -> HC-SR04 TRIG
    GPIO14 <- A2 <- B2 <- HC-SR04 ECHO

  TXS0108E LOOPBACK:
    GPIO15 <-> A3/B3 -- jumper B3<->B4 -- B4/A4 <-> GPIO16

  Wi-Fi AP:
    SSID: PRISMA-S3-DIAG
    Password: OPEN
    Web: http://192.168.4.1
*/

#include <WiFi.h>
#include <WebServer.h>
#include <esp_system.h>

WebServer server(80);

static const char* FW_VERSION = "7.0";
static const char* AP_SSID = "PRISMA-S3-DIAG";

static const uint8_t ANALOG_PIN = 1;
static const uint8_t S1_TRIG_PIN = 13;
static const uint8_t S1_ECHO_PIN = 14;
static const uint8_t TXS_A3_GPIO = 15;
static const uint8_t TXS_A4_GPIO = 16;
static const uint32_t S1_TIMEOUT_US = 30000;
static const uint32_t LOG_SAMPLE_MS = 10000;
static const size_t MAX_LOG_CHARS = 13000;

// Pins that can be selected for the direct pushbutton-to-GND test.
// GPIO13/14 are reserved for Sensor 1 and GPIO15/16 for TXS loopback.
const uint8_t SAFE_INPUT_TEST_PINS[] = {
  2, 4, 5, 6, 7, 8, 9, 10, 11, 12,
  17, 18, 21, 39, 40, 41, 42, 47
};
const size_t SAFE_INPUT_TEST_PIN_COUNT = sizeof(SAFE_INPUT_TEST_PINS) / sizeof(SAFE_INPUT_TEST_PINS[0]);

const uint8_t PASSIVE_MONITOR_PINS[] = {
  2, 4, 5, 6, 7, 8, 9, 10, 11, 12,
  17, 18, 21, 39, 40, 41, 42, 47
};
const size_t PASSIVE_MONITOR_PIN_COUNT = sizeof(PASSIVE_MONITOR_PINS) / sizeof(PASSIVE_MONITOR_PINS[0]);

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

// Pushbutton test state.
int8_t activeInputPin = -1;
int lastRawInputState = HIGH;
int stableInputState = HIGH;
uint32_t inputLastRawChangeMs = 0;
uint32_t inputPressCount = 0;
uint32_t inputTransitionCount = 0;
static const uint32_t INPUT_DEBOUNCE_MS = 20;

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

bool isSafeInputTestPin(int pin) {
  for (size_t i = 0; i < SAFE_INPUT_TEST_PIN_COUNT; i++) {
    if ((int)SAFE_INPUT_TEST_PINS[i] == pin) return true;
  }
  return false;
}

void restorePassiveInput(int pin) {
  if (pin >= 0 && isSafeInputTestPin(pin)) pinMode(pin, INPUT);
}

void setInputTestPin(int pin) {
  if (activeInputPin >= 0) restorePassiveInput(activeInputPin);

  activeInputPin = -1;
  inputPressCount = 0;
  inputTransitionCount = 0;
  lastRawInputState = HIGH;
  stableInputState = HIGH;
  inputLastRawChangeMs = millis();

  if (pin < 0) {
    addLog("INPUT TEST disabled");
    return;
  }

  if (!isSafeInputTestPin(pin)) {
    addLog("INPUT TEST rejected unsafe/reserved GPIO" + String(pin));
    return;
  }

  activeInputPin = pin;
  pinMode(activeInputPin, INPUT_PULLUP);
  lastRawInputState = digitalRead(activeInputPin);
  stableInputState = lastRawInputState;
  inputLastRawChangeMs = millis();
  addLog("INPUT TEST GPIO" + String(activeInputPin) + " armed as INPUT_PULLUP; button must connect GPIO to GND");
}

void serviceInputTest() {
  if (activeInputPin < 0) return;

  int raw = digitalRead(activeInputPin);
  uint32_t now = millis();

  if (raw != lastRawInputState) {
    lastRawInputState = raw;
    inputLastRawChangeMs = now;
  }

  if ((now - inputLastRawChangeMs) >= INPUT_DEBOUNCE_MS && raw != stableInputState) {
    stableInputState = raw;
    inputTransitionCount++;
    if (stableInputState == LOW) {
      inputPressCount++;
      addLog("INPUT GPIO" + String(activeInputPin) + " PRESSED count=" + String(inputPressCount));
    } else {
      addLog("INPUT GPIO" + String(activeInputPin) + " RELEASED");
    }
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

  r.forwardPass = runTxsDirection(TXS_A3_GPIO, TXS_A4_GPIO, "FWD GPIO15/A3/B3 -> B4/A4/GPIO16");
  delay(5);
  r.reversePass = runTxsDirection(TXS_A4_GPIO, TXS_A3_GPIO, "REV GPIO16/A4/B4 -> B3/A3/GPIO15");

  r.forwardOk = (r.forwardPass == r.total);
  r.reverseOk = (r.reversePass == r.total);
  lastTxs = r;

  if (r.forwardOk && r.reverseOk) addLog("TXS LOOPBACK PASS");
  else addLog("TXS LOOPBACK FAIL");
  return r;
}

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="es">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SONIC BONGO · PIN LAB v7</title>
<style>
:root{--bg:#080c11;--panel:#111923;--line:#2b3948;--text:#edf4fa;--muted:#8fa2b5;--ok:#45ef89;--low:#3b4b5a;--bad:#ff6262;--warn:#ffbf47;--power:#58d8ff;--info:#b28cff;--sensor:#68f0d1;--txs:#75a7ff;--test:#ff8fd8}
*{box-sizing:border-box}body{margin:0;min-height:100vh;background:linear-gradient(180deg,#080c11,#0e1620 65%,#091019);color:var(--text);font-family:system-ui,-apple-system,"Segoe UI",sans-serif}
header,.stats,.notice,.panel,.boardWrap,.legend,.actions,.logPanel{max-width:1080px;margin-left:auto;margin-right:auto}
header{padding:18px 16px 8px}h1{font-size:23px;margin:0 0 5px}.sub{font-size:12px;color:var(--muted)}
.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(125px,1fr));gap:8px;padding:8px 16px}.card,.panel,.logPanel{background:var(--panel);border:1px solid var(--line);border-radius:11px}.card{padding:9px 10px}.k{font-size:10px;color:var(--muted);letter-spacing:.08em}.v{font-size:15px;font-weight:800;margin-top:3px}.ok{color:var(--ok)}.bad{color:var(--bad)}
.notice{margin-top:7px;padding:10px 12px;border:1px solid #594820;background:#211b10;color:#ffda8b;border-radius:10px;font-size:12px}.panel{margin-top:12px;padding:14px 16px}.panel h2{font-size:17px;margin:0 0 8px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(145px,1fr));gap:8px}.value{background:#091018;border:1px solid var(--line);border-radius:9px;padding:10px}.big{font-size:22px;font-weight:900;margin-top:4px}
.actions{display:flex;flex-wrap:wrap;gap:8px;padding:12px 16px 4px}button,a.btn,select{border:1px solid #40566c;background:#182635;color:#edf4fa;padding:10px 12px;border-radius:9px;font-weight:800;font-size:12px;text-decoration:none}button{cursor:pointer}.testBtn{border-color:#8a4f78;background:#3a1731}.sensorBtn{border-color:#277762;background:#12382f}.txsBtn{border-color:#446db3;background:#152b4e}.stop{border-color:#743c3c;background:#351919}.hint{font-size:11px;color:var(--muted);margin-top:8px}.wiring{margin-top:10px;background:#060b10;border:1px solid var(--line);border-radius:8px;padding:9px;font:11px/1.65 ui-monospace,Consolas,monospace;color:#bdcad6}
.boardWrap{padding:14px 12px 4px}.board{display:grid;grid-template-columns:minmax(180px,1fr) minmax(190px,235px) minmax(180px,1fr);gap:10px;background:#202a35;border:2px solid #465767;border-radius:24px;padding:14px;box-shadow:0 18px 45px rgba(0,0,0,.38)}.pincol{display:flex;flex-direction:column;gap:5px}.pin{min-height:31px;display:flex;align-items:center;gap:7px;padding:4px 7px;background:var(--panel);border:1px solid var(--line);border-radius:8px}.right .pin{flex-direction:row-reverse;text-align:right}.dot{width:12px;height:12px;flex:0 0 12px;border-radius:50%;background:var(--low)}.pin.high .dot{background:var(--ok);box-shadow:0 0 13px var(--ok)}.pin.locked .dot{background:var(--warn)}.pin.power .dot{background:var(--power)}.pin.ground .dot{background:#080808;border:1px solid #59616a}.pin.info .dot{background:var(--info)}.pin.sensorpin{border-color:#277762}.pin.sensorpin .dot{background:var(--sensor)}.pin.txspin{border-color:#446db3}.pin.txspin .dot{background:var(--txs)}.pin.testselected{border:2px solid var(--test);box-shadow:0 0 15px rgba(255,143,216,.25)}.pin.testselected .dot{background:var(--test);box-shadow:0 0 13px var(--test)}.name{font-size:12px;font-weight:850;white-space:nowrap}.desc{font-size:9px;color:var(--muted);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.middle{min-height:775px;display:flex;flex-direction:column;align-items:center;justify-content:space-between;padding:17px 8px 13px;background:#0d141c;border:1px solid #344251;border-radius:18px}.antenna{width:87%;height:88px;border:2px solid #6f5a36;border-radius:9px;background:repeating-linear-gradient(90deg,transparent 0 12px,#9d7a3d 12px 20px,transparent 20px 31px)}.module{width:88%;height:292px;display:flex;align-items:center;justify-content:center;text-align:center;font-weight:900;color:#171717;background:linear-gradient(145deg,#c8b27c,#82734f);border-radius:9px}.boardname{text-align:center;font-size:12px;color:#aebdca;line-height:1.5}.buttons{font-size:10px;color:#8ea1b2}.usbrow{display:flex;gap:14px}.usb{width:56px;height:26px;border-radius:5px 5px 9px 9px;background:#aab3bb;border:2px solid #68747e}
.legend{display:flex;flex-wrap:wrap;gap:13px;padding:10px 16px 4px;color:var(--muted);font-size:11px}.legend span{display:flex;align-items:center;gap:5px}.mini{width:9px;height:9px;border-radius:50%;display:inline-block}.logPanel{margin-top:10px;padding:12px 16px 18px}.logPanel h2{font-size:14px;margin:0 0 8px}pre{white-space:pre-wrap;word-break:break-word;background:#05090d;border:1px solid var(--line);border-radius:9px;padding:10px;max-height:320px;overflow:auto;color:#b9c8d6;font-size:11px}.hidden{display:none}
@media(max-width:720px){.board{grid-template-columns:minmax(100px,1fr) 110px minmax(100px,1fr);gap:5px;padding:7px}.middle{min-height:742px;padding:8px 3px}.module{height:235px;font-size:11px}.antenna{height:65px}.pin{padding:3px;gap:4px;min-height:29px}.name{font-size:10px}.desc{display:none}.dot{width:10px;height:10px;flex-basis:10px}.usb{width:40px;height:20px}}
</style>
</head>
<body>
<header><h1>SONIC BONGO · N16R8 PIN LAB v7</h1><div class="sub">ESP32-S3 N16R8 · mapa completo 44 pines · test de entradas · Sensor 1 · TXS0108E</div></header>
<section class="stats">
<div class="card"><div class="k">CHIP</div><div class="v" id="chip">...</div></div><div class="card"><div class="k">FLASH</div><div class="v" id="flash">...</div></div><div class="card"><div class="k">PSRAM</div><div class="v" id="psram">...</div></div><div class="card"><div class="k">RESET</div><div class="v" id="resetCause">...</div></div><div class="card"><div class="k">HEAP</div><div class="v" id="heap">...</div></div><div class="card"><div class="k">WEB</div><div class="v" id="link">CONECTANDO</div></div>
</section>
<div class="notice"><b>TEST INPUT:</b> elegí un GPIO seguro y conectá únicamente <b>GPIO → pulsador → GND</b>. El firmware activa INPUT_PULLUP sólo en ese pin. Suelto=HIGH; presionado=LOW.</div>
<section class="panel">
<h2>PULSADOR · TEST DE ENTRADA UNO POR UNO</h2>
<div class="grid"><div class="value"><div class="k">GPIO ACTIVO</div><div class="big" id="inputPin">NINGUNO</div></div><div class="value"><div class="k">ESTADO</div><div class="big" id="inputState">--</div></div><div class="value"><div class="k">PULSACIONES</div><div class="big" id="inputPresses">0</div></div><div class="value"><div class="k">CAMBIOS</div><div class="big" id="inputTransitions">0</div></div></div>
<div class="actions" style="padding-left:0"><select id="pinSelect"><option value="4">GPIO4</option><option value="5">GPIO5</option><option value="6">GPIO6</option><option value="7">GPIO7</option><option value="8">GPIO8</option><option value="9">GPIO9</option><option value="10">GPIO10</option><option value="11">GPIO11</option><option value="12">GPIO12</option><option value="17">GPIO17</option><option value="18">GPIO18</option><option value="21">GPIO21</option><option value="2">GPIO2</option><option value="39">GPIO39</option><option value="40">GPIO40</option><option value="41">GPIO41</option><option value="42">GPIO42</option><option value="47">GPIO47</option></select><button class="testBtn" onclick="armInput()">ACTIVAR TEST</button><button class="stop" onclick="stopInput()">DESACTIVAR</button></div>
<div class="wiring">GPIO seleccionado ─── PULSADOR ─── GND<br><br>Sin 5V. Sin 3.3V externo. El pull-up es interno del ESP32.</div>
<div class="hint">GPIO13/14 quedan reservados para Sensor 1. GPIO15/16 quedan reservados para el loopback TXS.</div>
</section>
<div class="boardWrap"><div class="board"><div class="pincol left" id="leftPins"></div><div class="middle"><div class="antenna"></div><div class="module">ESP32-S3<br>WROOM-1<br><br>N16R8</div><div class="boardname">16 MB FLASH<br>8 MB OCTAL PSRAM<br><br>PIN LAB v7</div><div class="buttons">BOOT &nbsp;&nbsp;&nbsp; RESET</div><div class="usbrow"><div class="usb"></div><div class="usb"></div></div></div><div class="pincol right" id="rightPins"></div></div></div>
<div class="legend"><span><i class="mini" style="background:#ff8fd8"></i>GPIO EN TEST</span><span><i class="mini" style="background:#45ef89"></i>HIGH</span><span><i class="mini" style="background:#3b4b5a"></i>LOW</span><span><i class="mini" style="background:#68f0d1"></i>SENSOR 1</span><span><i class="mini" style="background:#75a7ff"></i>TXS TEST</span><span><i class="mini" style="background:#ffbf47"></i>RESERVADO</span><span><i class="mini" style="background:#58d8ff"></i>ALIMENTACIÓN</span></div>
<section class="panel">
<h2>HC-SR04 · SENSOR 1</h2><div class="grid"><div class="value"><div class="k">ESTADO</div><div class="big" id="sensorStatus">SIN PRUEBA</div></div><div class="value"><div class="k">DISTANCIA</div><div class="big" id="distance">-- cm</div></div><div class="value"><div class="k">ECHO</div><div class="big" id="echoUs">-- µs</div></div><div class="value"><div class="k">VÁLIDAS/TOTAL</div><div class="big" id="validCount">--</div></div></div><div class="actions" style="padding-left:0"><button class="sensorBtn" onclick="testSensor(1)">PROBAR 1</button><button class="sensorBtn" onclick="testSensor(5)">PROBAR 5</button></div><div class="wiring">GPIO13 → A1 → B1 → TRIG<br>GPIO14 ← A2 ← B2 ← ECHO</div>
</section>
<section class="panel">
<h2>TXS0108E · LOOPBACK</h2><div class="grid"><div class="value"><div class="k">RESULTADO</div><div class="big" id="txsStatus">SIN PRUEBA</div></div><div class="value"><div class="k">IDA</div><div class="big" id="txsFwd">--</div></div><div class="value"><div class="k">VUELTA</div><div class="big" id="txsRev">--</div></div></div><div class="actions" style="padding-left:0"><button class="txsBtn" onclick="testTxs()">PROBAR TXS LOOPBACK</button></div><div class="wiring">GPIO15 ↔ A3/B3 ── puente B3↔B4 ── B4/A4 ↔ GPIO16</div>
</section>
<div class="actions"><a class="btn" href="/api/export">EXPORTAR DIAGNÓSTICO (.TXT)</a><button onclick="toggleLog()">VER / OCULTAR LOG</button><button onclick="loadLog()">ACTUALIZAR LOG</button></div>
<div class="logPanel hidden" id="logPanel"><h2>Log circular</h2><pre id="logText">Cargando...</pre></div>
<script>
const LEFT=[["3V3",null,"power","3,3 V"],["3V3",null,"power","3,3 V"],["RST",null,"info","EN / RESET"],["GPIO4",4,"gpio","K1 / test input"],["GPIO5",5,"gpio","K2 / test input"],["GPIO6",6,"gpio","K3 / test input"],["GPIO7",7,"gpio","K4 / test input"],["GPIO15",15,"txspin","TXS A3 loopback"],["GPIO16",16,"txspin","TXS A4 loopback"],["GPIO17",17,"gpio","TRIG S3 futuro / test"],["GPIO18",18,"gpio","ECHO S3 futuro / test"],["GPIO8",8,"gpio","OLED SDA / test"],["GPIO3",3,"locked","strapping / JTAG"],["GPIO46",46,"locked","strapping"],["GPIO9",9,"gpio","OLED SCL / test"],["GPIO10",10,"gpio","Touch grande / test"],["GPIO11",11,"gpio","Touch 1 / test"],["GPIO12",12,"gpio","Touch 2 / test"],["GPIO13",13,"sensorpin","TRIG S1 · OUTPUT"],["GPIO14",14,"sensorpin","ECHO S1 · INPUT"],["5V",null,"power","5 V / VIN"],["GND",null,"ground","masa"]];
const RIGHT=[["GND",null,"ground","masa"],["TX / GPIO43",43,"locked","UART0"],["RX / GPIO44",44,"locked","UART0"],["GPIO1",1,"gpio","ADC flotante si libre"],["GPIO2",2,"gpio","test input"],["GPIO42",42,"gpio","test input"],["GPIO41",41,"gpio","test input"],["GPIO40",40,"gpio","test input"],["GPIO39",39,"gpio","test input"],["GPIO38",38,"locked","RGB posible"],["GPIO37",37,"locked","Octal PSRAM"],["GPIO36",36,"locked","Octal PSRAM"],["GPIO35",35,"locked","Octal PSRAM"],["GPIO0",0,"locked","BOOT / strapping"],["GPIO45",45,"locked","strapping"],["GPIO48",48,"locked","RGB posible / reservado"],["GPIO47",47,"gpio","test input"],["GPIO21",21,"gpio","ARM/MUTE / test"],["GPIO20",20,"locked","USB D+"],["GPIO19",19,"locked","USB D-"],["GND",null,"ground","masa"],["GND",null,"ground","masa"]];
function makePin(s){const [label,gpio,type,desc]=s;const e=document.createElement('div');e.className='pin '+(type==='gpio'?'':type);e.dataset.gpio=gpio===null?'':String(gpio);e.innerHTML='<span class="dot"></span><div style="min-width:0"><div class="name">'+label+'</div><div class="desc">'+desc+'</div></div>';return e}
LEFT.forEach(p=>leftPins.appendChild(makePin(p)));RIGHT.forEach(p=>rightPins.appendChild(makePin(p)));
function applyStates(states,active){document.querySelectorAll('.pin[data-gpio]').forEach(el=>{const p=el.dataset.gpio;el.classList.toggle('testselected',active>=0&&Number(p)===active);if(p!==''&&Object.prototype.hasOwnProperty.call(states,p)&&!el.classList.contains('sensorpin')&&!el.classList.contains('txspin')&&!el.classList.contains('testselected'))el.classList.toggle('high',states[p]===1)})}
async function refresh(){try{const r=await fetch('/api/status',{cache:'no-store'});const d=await r.json();chip.textContent=d.chip;flash.textContent=d.flashMB+' MB';psram.textContent=d.psramMB+' MB';resetCause.textContent=d.resetCause;heap.textContent=Math.round(d.freeHeap/1024)+' KB';link.textContent='ONLINE';link.className='v ok';flash.className='v '+(d.flashMB===16?'ok':'bad');psram.className='v '+(d.psramMB===8?'ok':'bad');inputPin.textContent=d.inputTest.active?'GPIO'+d.inputTest.pin:'NINGUNO';inputPresses.textContent=d.inputTest.presses;inputTransitions.textContent=d.inputTest.transitions;if(d.inputTest.active){inputState.textContent=d.inputTest.pressed?'PRESIONADO / LOW':'SUELTO / HIGH';inputState.style.color=d.inputTest.pressed?'#ff8fd8':'#45ef89'}else{inputState.textContent='--';inputState.style.color=''}applyStates(d.pins,d.inputTest.active?d.inputTest.pin:-1)}catch(e){link.textContent='SIN RESPUESTA';link.className='v bad'}}
async function armInput(){await fetch('/api/input?pin='+pinSelect.value,{cache:'no-store'});refresh()}
async function stopInput(){await fetch('/api/input?pin=off',{cache:'no-store'});refresh()}
async function testSensor(count){sensorStatus.textContent='MIDIENDO...';try{const d=await (await fetch('/api/sensor1?count='+count,{cache:'no-store'})).json();validCount.textContent=d.valid+'/'+d.total;if(d.valid){distance.textContent=d.avgCm.toFixed(1)+' cm';echoUs.textContent=Math.round(d.avgEchoUs)+' µs';sensorStatus.textContent='OK';sensorStatus.className='big ok'}else{distance.textContent='-- cm';echoUs.textContent='-- µs';sensorStatus.textContent='SIN ECHO';sensorStatus.className='big bad'}}catch(e){sensorStatus.textContent='ERROR'}}
async function testTxs(){txsStatus.textContent='PROBANDO...';try{const d=await (await fetch('/api/txs',{cache:'no-store'})).json();txsFwd.textContent=d.forwardPass+'/'+d.total;txsRev.textContent=d.reversePass+'/'+d.total;const ok=d.forwardOk&&d.reverseOk;txsStatus.textContent=ok?'PASS':'FAIL';txsStatus.className='big '+(ok?'ok':'bad')}catch(e){txsStatus.textContent='ERROR'}}
function toggleLog(){logPanel.classList.toggle('hidden');if(!logPanel.classList.contains('hidden'))loadLog()}
async function loadLog(){try{logText.textContent=await (await fetch('/api/log',{cache:'no-store'})).text();logText.scrollTop=logText.scrollHeight}catch(e){logText.textContent='Error leyendo log'}}
refresh();setInterval(refresh,180);
</script>
</body>
</html>
)HTML";

void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleStatus() {
  String json;
  json.reserve(1800);
  json += "{";
  json += "\"fw\":\"" + String(FW_VERSION) + "\",";
  json += "\"chip\":\"" + String(ESP.getChipModel()) + "\",";
  json += "\"revision\":" + String(ESP.getChipRevision()) + ",";
  json += "\"flashMB\":" + String((uint32_t)(ESP.getFlashChipSize() / (1024 * 1024))) + ",";
  json += "\"psramMB\":" + String((uint32_t)(ESP.getPsramSize() / (1024 * 1024))) + ",";
  json += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
  json += "\"resetCause\":\"" + resetReasonText(esp_reset_reason()) + "\",";
  json += "\"uptimeMs\":" + String(millis()) + ",";
  json += "\"pins\":{";
  json += "\"1\":" + String(digitalRead(ANALOG_PIN) ? 1 : 0);
  for (size_t i = 0; i < PASSIVE_MONITOR_PIN_COUNT; i++) {
    int p = PASSIVE_MONITOR_PINS[i];
    json += ",\"" + String(p) + "\":" + String(digitalRead(p) ? 1 : 0);
  }
  json += ",\"13\":" + String(digitalRead(S1_TRIG_PIN) ? 1 : 0);
  json += ",\"14\":" + String(digitalRead(S1_ECHO_PIN) ? 1 : 0);
  json += ",\"15\":" + String(digitalRead(TXS_A3_GPIO) ? 1 : 0);
  json += ",\"16\":" + String(digitalRead(TXS_A4_GPIO) ? 1 : 0);
  json += "},";
  json += "\"inputTest\":{";
  json += "\"active\":" + String(activeInputPin >= 0 ? "true" : "false") + ",";
  json += "\"pin\":" + String(activeInputPin) + ",";
  json += "\"pressed\":" + String((activeInputPin >= 0 && stableInputState == LOW) ? "true" : "false") + ",";
  json += "\"raw\":" + String(activeInputPin >= 0 ? digitalRead(activeInputPin) : -1) + ",";
  json += "\"presses\":" + String(inputPressCount) + ",";
  json += "\"transitions\":" + String(inputTransitionCount);
  json += "},";
  json += "\"sensor1\":{";
  json += "\"ok\":" + String(lastSensor.ok ? "true" : "false") + ",";
  json += "\"durationUs\":" + String(lastSensor.durationUs) + ",";
  json += "\"distanceCm\":" + String(lastSensor.distanceCm, 2);
  json += "},";
  json += "\"txs\":{";
  json += "\"hasRun\":" + String(lastTxs.hasRun ? "true" : "false") + ",";
  json += "\"forwardPass\":" + String(lastTxs.forwardPass) + ",";
  json += "\"reversePass\":" + String(lastTxs.reversePass) + ",";
  json += "\"total\":" + String(lastTxs.total);
  json += "}}";
  server.send(200, "application/json", json);
}

void handleInputTest() {
  if (!server.hasArg("pin")) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"missing pin\"}");
    return;
  }

  String arg = server.arg("pin");
  if (arg == "off") {
    setInputTestPin(-1);
    server.send(200, "application/json", "{\"ok\":true,\"active\":false}");
    return;
  }

  int pin = arg.toInt();
  if (!isSafeInputTestPin(pin)) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"unsafe or reserved pin\"}");
    return;
  }

  setInputTestPin(pin);
  String out = "{\"ok\":true,\"active\":true,\"pin\":" + String(pin) + "}";
  server.send(200, "application/json", out);
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
  json += "\"avgEchoUs\":" + String(avgEcho, 1);
  json += "}";
  server.send(200, "application/json", json);
}

void handleTxs() {
  TxsDiagResult r = runTxsLoopback();
  String json;
  json.reserve(220);
  json += "{";
  json += "\"forwardPass\":" + String(r.forwardPass) + ",";
  json += "\"reversePass\":" + String(r.reversePass) + ",";
  json += "\"total\":" + String(r.total) + ",";
  json += "\"forwardOk\":" + String(r.forwardOk ? "true" : "false") + ",";
  json += "\"reverseOk\":" + String(r.reverseOk ? "true" : "false");
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
  out.reserve(16000);
  out += "SONIC BONGO / PRISMA - ESP32-S3 N16R8 PIN LAB\n";
  out += "Firmware: v" + String(FW_VERSION) + "\n";
  out += "============================================================\n";
  out += "Chip: " + String(ESP.getChipModel()) + "\n";
  out += "Revision: " + String(ESP.getChipRevision()) + "\n";
  out += "Flash MB: " + String((uint32_t)(ESP.getFlashChipSize() / (1024 * 1024))) + "\n";
  out += "PSRAM MB: " + String((uint32_t)(ESP.getPsramSize() / (1024 * 1024))) + "\n";
  out += "Reset cause: " + resetReasonText(esp_reset_reason()) + "\n";
  out += "Uptime ms: " + String(millis()) + "\n";
  out += "Free heap: " + String(ESP.getFreeHeap()) + "\n";
  out += "AP SSID: " + String(AP_SSID) + "\n";
  out += "AP IP: " + WiFi.softAPIP().toString() + "\n";
  out += "Connected stations: " + String(WiFi.softAPgetStationNum()) + "\n\n";

  out += "INPUT TEST\n";
  if (activeInputPin < 0) out += "Active pin: NONE\n";
  else {
    out += "Active pin: GPIO" + String(activeInputPin) + "\n";
    out += "Stable state: " + String(stableInputState == LOW ? "PRESSED/LOW" : "RELEASED/HIGH") + "\n";
    out += "Press count: " + String(inputPressCount) + "\n";
    out += "Transition count: " + String(inputTransitionCount) + "\n";
  }

  out += "\nSENSOR 1\n";
  if (lastSensor.ms == 0) out += "No sample yet.\n";
  else if (lastSensor.ok) out += "OK echo=" + String(lastSensor.durationUs) + " us distance=" + String(lastSensor.distanceCm, 2) + " cm\n";
  else out += "TIMEOUT no echo\n";

  out += "\nTXS LOOPBACK\n";
  if (!lastTxs.hasRun) out += "Not run.\n";
  else out += "Forward=" + String(lastTxs.forwardPass) + "/" + String(lastTxs.total) + " Reverse=" + String(lastTxs.reversePass) + "/" + String(lastTxs.total) + "\n";

  out += "\nGPIO1 ADC 64 samples min/avg/max = " + String(adcMin) + "/" + String(adcAvg) + "/" + String(adcMax) + "\n";
  out += "\nRUNTIME LOG\n";
  out += runtimeLog;

  server.sendHeader("Content-Disposition", "attachment; filename=SONIC_BONGO_N16R8_PIN_LAB_v7.txt");
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

  for (size_t i = 0; i < PASSIVE_MONITOR_PIN_COUNT; i++) {
    pinMode(PASSIVE_MONITOR_PINS[i], INPUT);
  }

  pinMode(S1_ECHO_PIN, INPUT);
  pinMode(S1_TRIG_PIN, OUTPUT);
  digitalWrite(S1_TRIG_PIN, LOW);
  pinMode(TXS_A3_GPIO, INPUT);
  pinMode(TXS_A4_GPIO, INPUT);

  addLog("SONIC BONGO ESP32-S3 N16R8 PIN LAB v7 boot");
  addLog("Reset cause: " + resetReasonText(esp_reset_reason()));
  addLog("Sensor 1 GPIO13/TRIG + GPIO14/ECHO ready");
  addLog("TXS loopback GPIO15/GPIO16 ready");
  addLog("Input test ready; safe GPIO selected from web uses INPUT_PULLUP");

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  bool ok = WiFi.softAP(AP_SSID);
  if (ok) addLog("AP started: " + String(AP_SSID) + " IP=" + WiFi.softAPIP().toString());
  else addLog("ERROR: failed to start Access Point");

  addLog("Chip=" + String(ESP.getChipModel()) + " rev=" + String(ESP.getChipRevision()));
  addLog("Flash=" + String(ESP.getFlashChipSize() / (1024 * 1024)) + "MB PSRAM=" + String(ESP.getPsramSize() / (1024 * 1024)) + "MB");

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/input", HTTP_GET, handleInputTest);
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
  serviceInputTest();

  uint32_t now = millis();
  if (now - lastPeriodicLog >= LOG_SAMPLE_MS) {
    lastPeriodicLog = now;
    uint16_t mn, mx;
    uint32_t av;
    readAdcStats(16, mn, mx, av);
    String msg = "HEALTH heap=" + String(ESP.getFreeHeap()) + " stations=" + String(WiFi.softAPgetStationNum()) + " adc1=" + String(mn) + "/" + String(av) + "/" + String(mx);
    if (activeInputPin >= 0) msg += " inputGPIO" + String(activeInputPin) + "=" + String(stableInputState == LOW ? "PRESSED" : "RELEASED");
    addLog(msg);
  }
  delay(2);
}
