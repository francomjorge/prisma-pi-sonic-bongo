/*
  SONIC BONGO / PRISMA
  ESP32-S3 N16R8 Diagnostic v8 - 3 SENSOR PIN LAB

  Confirmed wiring strategy:
  - HC-SR04 TRIG goes DIRECTLY from ESP32 GPIO (3.3 V logic is sufficient).
  - HC-SR04 ECHO is 5 V and MUST go through TXS0108E before the ESP32.

  SENSOR WIRING
    S1: GPIO13 -> TRIG                  ECHO -> TXS B1 -> A1 -> GPIO14
    S2: GPIO15 -> TRIG                  ECHO -> TXS B2 -> A2 -> GPIO16
    S3: GPIO17 -> TRIG                  ECHO -> TXS B3 -> A3 -> GPIO18

  POWER
    HC-SR04 VCC -> 5 V
    HC-SR04 GND -> common GND
    TXS VCCA -> 3.3 V
    TXS VCCB -> 5 V
    TXS OE   -> 3.3 V
    TXS GND  -> common GND

  PUSHBUTTON INPUT TEST
    Selected safe GPIO ---- pushbutton ---- GND
    Uses INPUT_PULLUP. No external voltage is used for the button.

  Wi-Fi AP
    SSID: PRISMA-S3-DIAG
    Password: OPEN
    Web: http://192.168.4.1
*/

#include <WiFi.h>
#include <WebServer.h>
#include <esp_system.h>

WebServer server(80);

static const char* FW_VERSION = "8.0";
static const char* AP_SSID = "PRISMA-S3-DIAG";
static const uint8_t ANALOG_PIN = 1;
static const uint32_t SENSOR_TIMEOUT_US = 30000;
static const uint32_t SENSOR_GAP_MS = 70;
static const uint32_t LOG_SAMPLE_MS = 10000;
static const uint32_t INPUT_DEBOUNCE_MS = 20;
static const size_t MAX_LOG_CHARS = 15000;

struct SensorPins {
  uint8_t trig;
  uint8_t echo;
  uint8_t txsChannel;
};

const SensorPins SENSOR_PINS[3] = {
  {13, 14, 1},
  {15, 16, 2},
  {17, 18, 3}
};

const uint8_t SAFE_INPUT_TEST_PINS[] = {
  2, 4, 5, 6, 7, 8, 9, 10, 11, 12,
  21, 39, 40, 41, 42, 47
};
const size_t SAFE_INPUT_TEST_PIN_COUNT = sizeof(SAFE_INPUT_TEST_PINS) / sizeof(SAFE_INPUT_TEST_PINS[0]);

const uint8_t PASSIVE_MONITOR_PINS[] = {
  2, 4, 5, 6, 7, 8, 9, 10, 11, 12,
  21, 39, 40, 41, 42, 47
};
const size_t PASSIVE_MONITOR_PIN_COUNT = sizeof(PASSIVE_MONITOR_PINS) / sizeof(PASSIVE_MONITOR_PINS[0]);

struct SensorSample {
  uint32_t ms;
  uint32_t durationUs;
  float distanceCm;
  bool ok;
};

static const uint8_t HISTORY_SIZE = 20;
SensorSample historyData[3][HISTORY_SIZE];
uint8_t historyHead[3] = {0, 0, 0};
uint8_t historyCount[3] = {0, 0, 0};
SensorSample lastSensor[3] = {
  {0,0,0.0f,false}, {0,0,0.0f,false}, {0,0,0.0f,false}
};

String runtimeLog;
uint32_t lastPeriodicLog = 0;

int8_t activeInputPin = -1;
int lastRawInputState = HIGH;
int stableInputState = HIGH;
uint32_t inputLastRawChangeMs = 0;
uint32_t inputPressCount = 0;
uint32_t inputTransitionCount = 0;

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
  addLog("INPUT TEST GPIO" + String(activeInputPin) + " armed INPUT_PULLUP; connect button to GND");
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

void storeSensorSample(uint8_t index, const SensorSample& s) {
  historyData[index][historyHead[index]] = s;
  historyHead[index] = (historyHead[index] + 1) % HISTORY_SIZE;
  if (historyCount[index] < HISTORY_SIZE) historyCount[index]++;
  lastSensor[index] = s;
}

SensorSample measureSensor(uint8_t index, bool logResult = true) {
  SensorSample s = {millis(), 0, 0.0f, false};
  if (index > 2) return s;

  const SensorPins &p = SENSOR_PINS[index];

  digitalWrite(p.trig, LOW);
  delayMicroseconds(4);
  digitalWrite(p.trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(p.trig, LOW);

  uint32_t duration = pulseIn(p.echo, HIGH, SENSOR_TIMEOUT_US);
  s.durationUs = duration;
  if (duration > 0) {
    s.distanceCm = ((float)duration * 0.0343f) / 2.0f;
    s.ok = true;
  }

  storeSensorSample(index, s);
  if (logResult) {
    String prefix = "S" + String(index + 1) + " ";
    if (s.ok) addLog(prefix + "OK echo=" + String(s.durationUs) + " us distance=" + String(s.distanceCm, 2) + " cm");
    else addLog(prefix + "TIMEOUT no ECHO HIGH within " + String(SENSOR_TIMEOUT_US) + " us");
  }
  return s;
}

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="es">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SONIC BONGO · 3 SENSOR PIN LAB v8</title>
<style>
:root{--bg:#080c11;--panel:#111923;--line:#2b3948;--text:#edf4fa;--muted:#8fa2b5;--ok:#45ef89;--low:#3b4b5a;--bad:#ff6262;--warn:#ffbf47;--power:#58d8ff;--info:#b28cff;--sensor:#68f0d1;--test:#ff8fd8;--trig:#54d7ff;--echo:#ffae5c}
*{box-sizing:border-box}body{margin:0;min-height:100vh;background:linear-gradient(180deg,#080c11,#0e1620 65%,#091019);color:var(--text);font-family:system-ui,-apple-system,"Segoe UI",sans-serif}
header,.stats,.notice,.panel,.boardWrap,.legend,.actions,.logPanel{max-width:1080px;margin-left:auto;margin-right:auto}
header{padding:18px 16px 8px}h1{font-size:23px;margin:0 0 5px}.sub{font-size:12px;color:var(--muted)}
.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(125px,1fr));gap:8px;padding:8px 16px}.card,.panel,.logPanel{background:var(--panel);border:1px solid var(--line);border-radius:11px}.card{padding:9px 10px}.k{font-size:10px;color:var(--muted);letter-spacing:.08em}.v{font-size:15px;font-weight:800;margin-top:3px}.ok{color:var(--ok)}.bad{color:var(--bad)}
.notice{margin-top:7px;padding:10px 12px;border:1px solid #594820;background:#211b10;color:#ffda8b;border-radius:10px;font-size:12px}.panel{margin-top:12px;padding:14px 16px}.panel h2{font-size:17px;margin:0 0 8px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(145px,1fr));gap:8px}.value{background:#091018;border:1px solid var(--line);border-radius:9px;padding:10px}.big{font-size:21px;font-weight:900;margin-top:4px}
.sensorGrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(245px,1fr));gap:9px}.sensorCard{background:#091018;border:1px solid #28483f;border-radius:10px;padding:11px}.sensorCard h3{margin:0 0 8px;font-size:15px}.sensorLine{display:flex;justify-content:space-between;gap:8px;font-size:12px;margin:5px 0}.sensorLine b{font-size:14px}.wiring{margin-top:9px;background:#060b10;border:1px solid var(--line);border-radius:8px;padding:9px;font:11px/1.65 ui-monospace,Consolas,monospace;color:#bdcad6}
.actions{display:flex;flex-wrap:wrap;gap:8px;padding:12px 16px 4px}button,a.btn,select{border:1px solid #40566c;background:#182635;color:#edf4fa;padding:9px 11px;border-radius:9px;font-weight:800;font-size:12px;text-decoration:none}button{cursor:pointer}.testBtn{border-color:#8a4f78;background:#3a1731}.sensorBtn{border-color:#277762;background:#12382f}.stop{border-color:#743c3c;background:#351919}.hint{font-size:11px;color:var(--muted);margin-top:8px}
.boardWrap{padding:14px 12px 4px}.board{display:grid;grid-template-columns:minmax(180px,1fr) minmax(190px,235px) minmax(180px,1fr);gap:10px;background:#202a35;border:2px solid #465767;border-radius:24px;padding:14px;box-shadow:0 18px 45px rgba(0,0,0,.38)}.pincol{display:flex;flex-direction:column;gap:5px}.pin{min-height:31px;display:flex;align-items:center;gap:7px;padding:4px 7px;background:var(--panel);border:1px solid var(--line);border-radius:8px}.right .pin{flex-direction:row-reverse;text-align:right}.dot{width:12px;height:12px;flex:0 0 12px;border-radius:50%;background:var(--low)}.pin.high .dot{background:var(--ok);box-shadow:0 0 13px var(--ok)}.pin.locked .dot{background:var(--warn)}.pin.power .dot{background:var(--power)}.pin.ground .dot{background:#080808;border:1px solid #59616a}.pin.info .dot{background:var(--info)}.pin.trigpin{border-color:#236b83}.pin.trigpin .dot{background:var(--trig)}.pin.echopin{border-color:#8d5d2c}.pin.echopin .dot{background:var(--echo)}.pin.activeTest{border-color:var(--test);box-shadow:0 0 0 1px var(--test),0 0 15px rgba(255,143,216,.25)}.pin.activeTest .dot{background:var(--test)}.name{font-size:12px;font-weight:850;white-space:nowrap}.desc{font-size:9px;color:var(--muted);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.middle{min-height:775px;display:flex;flex-direction:column;align-items:center;justify-content:space-between;padding:17px 8px 13px;background:#0d141c;border:1px solid #344251;border-radius:18px}.antenna{width:87%;height:88px;border:2px solid #6f5a36;border-radius:9px;background:repeating-linear-gradient(90deg,transparent 0 12px,#9d7a3d 12px 20px,transparent 20px 31px)}.module{width:88%;height:292px;display:flex;align-items:center;justify-content:center;text-align:center;font-weight:900;color:#171717;background:linear-gradient(145deg,#c8b27c,#82734f);border-radius:9px}.boardname{text-align:center;font-size:12px;color:#aebdca;line-height:1.5}.buttons{font-size:10px;color:#8ea1b2}.usbrow{display:flex;gap:14px}.usb{width:56px;height:26px;border-radius:5px 5px 9px 9px;background:#aab3bb;border:2px solid #68747e}
.legend{display:flex;flex-wrap:wrap;gap:13px;padding:10px 16px 4px;color:var(--muted);font-size:11px}.legend span{display:flex;align-items:center;gap:5px}.mini{width:9px;height:9px;border-radius:50%;display:inline-block}pre{white-space:pre-wrap;word-break:break-word;background:#05090d;border:1px solid var(--line);border-radius:9px;padding:10px;max-height:330px;overflow:auto;color:#b9c8d6;font-size:11px}.hidden{display:none}
@media(max-width:720px){.board{grid-template-columns:minmax(100px,1fr) 110px minmax(100px,1fr);gap:5px;padding:7px}.middle{min-height:742px;padding:8px 3px}.module{height:235px;font-size:11px}.antenna{height:65px}.pin{padding:3px;gap:4px;min-height:29px}.name{font-size:10px}.desc{display:none}.dot{width:10px;height:10px;flex-basis:10px}.usb{width:40px;height:20px}}
</style>
</head>
<body>
<header><h1>SONIC BONGO · 3 SENSOR PIN LAB v8</h1><div class="sub">ESP32-S3 N16R8 · mapa completo · 3 HC-SR04 · test de entradas</div></header>
<section class="stats">
<div class="card"><div class="k">CHIP</div><div class="v" id="chip">...</div></div>
<div class="card"><div class="k">FLASH</div><div class="v" id="flash">...</div></div>
<div class="card"><div class="k">PSRAM</div><div class="v" id="psram">...</div></div>
<div class="card"><div class="k">RESET</div><div class="v" id="resetCause">...</div></div>
<div class="card"><div class="k">HEAP</div><div class="v" id="heap">...</div></div>
<div class="card"><div class="k">WEB</div><div class="v" id="link">CONECTANDO</div></div>
</section>
<div class="notice"><b>IMPORTANTE:</b> TRIG va directo al ESP32. Sólo ECHO pasa por el TXS0108E. Nunca conectes ECHO de 5 V directamente a un GPIO del ESP32.</div>

<section class="panel">
<h2>3 × HC-SR04</h2>
<div class="sensorGrid">
  <div class="sensorCard"><h3>SENSOR 1 · GPIO13 / GPIO14</h3><div class="sensorLine"><span>Estado</span><b id="s1status">SIN PRUEBA</b></div><div class="sensorLine"><span>Distancia</span><b id="s1dist">-- cm</b></div><div class="sensorLine"><span>Echo</span><b id="s1echo">-- µs</b></div><button class="sensorBtn" onclick="testOne(1)">PROBAR S1</button></div>
  <div class="sensorCard"><h3>SENSOR 2 · GPIO15 / GPIO16</h3><div class="sensorLine"><span>Estado</span><b id="s2status">SIN PRUEBA</b></div><div class="sensorLine"><span>Distancia</span><b id="s2dist">-- cm</b></div><div class="sensorLine"><span>Echo</span><b id="s2echo">-- µs</b></div><button class="sensorBtn" onclick="testOne(2)">PROBAR S2</button></div>
  <div class="sensorCard"><h3>SENSOR 3 · GPIO17 / GPIO18</h3><div class="sensorLine"><span>Estado</span><b id="s3status">SIN PRUEBA</b></div><div class="sensorLine"><span>Distancia</span><b id="s3dist">-- cm</b></div><div class="sensorLine"><span>Echo</span><b id="s3echo">-- µs</b></div><button class="sensorBtn" onclick="testOne(3)">PROBAR S3</button></div>
</div>
<div class="actions" style="padding-left:0"><button class="sensorBtn" onclick="testAll()">PROBAR LOS 3</button><button class="sensorBtn" id="autoBtn" onclick="toggleAuto()">AUTO: INICIAR</button><button class="stop" onclick="stopAuto()">DETENER</button></div>
<div class="wiring">S1: GPIO13 → TRIG &nbsp;&nbsp; ECHO → B1 → A1 → GPIO14<br>S2: GPIO15 → TRIG &nbsp;&nbsp; ECHO → B2 → A2 → GPIO16<br>S3: GPIO17 → TRIG &nbsp;&nbsp; ECHO → B3 → A3 → GPIO18<br><br>HC-SR04: VCC=5V · GND=común<br>TXS0108E: VCCA=3.3V · VCCB=5V · OE=3.3V · GND=común</div>
<div class="hint">La prueba de los 3 dispara uno por vez con una pausa entre sensores para reducir interferencia acústica.</div>
</section>

<section class="panel">
<h2>PULSADOR · TEST DE ENTRADA</h2>
<div class="grid"><div class="value"><div class="k">GPIO ACTIVO</div><div class="big" id="inputPin">NINGUNO</div></div><div class="value"><div class="k">ESTADO</div><div class="big" id="inputState">--</div></div><div class="value"><div class="k">PULSACIONES</div><div class="big" id="pressCount">0</div></div><div class="value"><div class="k">CAMBIOS</div><div class="big" id="transitionCount">0</div></div></div>
<div class="actions" style="padding-left:0"><select id="pinSelect"><option value="4">GPIO4</option><option value="5">GPIO5</option><option value="6">GPIO6</option><option value="7">GPIO7</option><option value="8">GPIO8</option><option value="9">GPIO9</option><option value="10">GPIO10</option><option value="11">GPIO11</option><option value="12">GPIO12</option><option value="21">GPIO21</option><option value="39">GPIO39</option><option value="40">GPIO40</option><option value="41">GPIO41</option><option value="42">GPIO42</option><option value="47">GPIO47</option><option value="2">GPIO2</option></select><button class="testBtn" onclick="armInput()">ACTIVAR TEST</button><button class="stop" onclick="disableInput()">DESACTIVAR</button></div>
<div class="wiring">GPIO elegido ─── PULSADOR ─── GND<br>Modo INPUT_PULLUP: suelto=HIGH · presionado=LOW</div>
</section>

<div class="boardWrap"><div class="board"><div class="pincol left" id="leftPins"></div><div class="middle"><div class="antenna"></div><div class="module">ESP32-S3<br>WROOM-1<br><br>N16R8</div><div class="boardname">3 SENSOR PIN LAB<br>16 MB FLASH<br>8 MB PSRAM</div><div class="buttons">BOOT &nbsp;&nbsp;&nbsp; RESET</div><div class="usbrow"><div class="usb"></div><div class="usb"></div></div></div><div class="pincol right" id="rightPins"></div></div></div>
<div class="legend"><span><i class="mini" style="background:#54d7ff"></i>TRIG directo</span><span><i class="mini" style="background:#ffae5c"></i>ECHO vía TXS</span><span><i class="mini" style="background:#ff8fd8"></i>GPIO bajo test</span><span><i class="mini" style="background:#45ef89"></i>HIGH</span><span><i class="mini" style="background:#3b4b5a"></i>LOW</span><span><i class="mini" style="background:#ffbf47"></i>RESERVADO</span></div>
<div class="actions"><a class="btn" href="/api/export">EXPORTAR DIAGNÓSTICO (.TXT)</a><button onclick="toggleLog()">VER / OCULTAR LOG</button><button onclick="loadLog()">ACTUALIZAR LOG</button></div>
<div class="logPanel hidden" id="logPanel"><pre id="logText">Cargando...</pre></div>

<script>
const LEFT=[["3V3",null,"power","3,3 V"],["3V3",null,"power","3,3 V"],["RST",null,"info","EN / RESET"],["GPIO4",4,"gpio","K1 / test"],["GPIO5",5,"gpio","K2 / test"],["GPIO6",6,"gpio","K3 / test"],["GPIO7",7,"gpio","K4 / test"],["GPIO15",15,"trigpin","TRIG S2 directo"],["GPIO16",16,"echopin","ECHO S2 vía TXS B2/A2"],["GPIO17",17,"trigpin","TRIG S3 directo"],["GPIO18",18,"echopin","ECHO S3 vía TXS B3/A3"],["GPIO8",8,"gpio","OLED SDA"],["GPIO3",3,"locked","strapping / JTAG"],["GPIO46",46,"locked","strapping"],["GPIO9",9,"gpio","OLED SCL"],["GPIO10",10,"gpio","Touch grande"],["GPIO11",11,"gpio","Touch 1"],["GPIO12",12,"gpio","Touch 2"],["GPIO13",13,"trigpin","TRIG S1 directo"],["GPIO14",14,"echopin","ECHO S1 vía TXS B1/A1"],["5V",null,"power","5 V / VIN"],["GND",null,"ground","masa"]];
const RIGHT=[["GND",null,"ground","masa"],["TX / GPIO43",43,"locked","UART0"],["RX / GPIO44",44,"locked","UART0"],["GPIO1",1,"gpio","ADC pedal / puede flotar"],["GPIO2",2,"gpio","test seguro"],["GPIO42",42,"gpio","test seguro"],["GPIO41",41,"gpio","test seguro"],["GPIO40",40,"gpio","test seguro"],["GPIO39",39,"gpio","test seguro"],["GPIO38",38,"locked","RGB posible"],["GPIO37",37,"locked","Octal PSRAM"],["GPIO36",36,"locked","Octal PSRAM"],["GPIO35",35,"locked","Octal PSRAM"],["GPIO0",0,"locked","BOOT"],["GPIO45",45,"locked","strapping"],["GPIO48",48,"locked","reservado"],["GPIO47",47,"gpio","test seguro"],["GPIO21",21,"gpio","ARM / MUTE"],["GPIO20",20,"locked","USB D+"],["GPIO19",19,"locked","USB D-"],["GND",null,"ground","masa"],["GND",null,"ground","masa"]];
function makePin(s){const [label,gpio,type,desc]=s;const e=document.createElement('div');e.className='pin '+(type==='gpio'?'':type);e.dataset.gpio=gpio===null?'':String(gpio);e.innerHTML='<span class="dot"></span><div style="min-width:0"><div class="name">'+label+'</div><div class="desc">'+desc+'</div></div>';return e}
LEFT.forEach(p=>leftPins.appendChild(makePin(p)));RIGHT.forEach(p=>rightPins.appendChild(makePin(p)));
function applyStates(states,activePin){document.querySelectorAll('.pin[data-gpio]').forEach(el=>{const p=el.dataset.gpio;el.classList.remove('activeTest');if(p!==''&&Number(p)===activePin)el.classList.add('activeTest');if(p!==''&&Object.prototype.hasOwnProperty.call(states,p)&&!el.classList.contains('trigpin')&&!el.classList.contains('echopin'))el.classList.toggle('high',states[p]===1)})}
async function refresh(){try{const r=await fetch('/api/status',{cache:'no-store'});const d=await r.json();chip.textContent=d.chip;flash.textContent=d.flashMB+' MB';psram.textContent=d.psramMB+' MB';resetCause.textContent=d.resetCause;heap.textContent=Math.round(d.freeHeap/1024)+' KB';link.textContent='ONLINE';link.className='v ok';flash.className='v '+(d.flashMB===16?'ok':'bad');psram.className='v '+(d.psramMB===8?'ok':'bad');applyStates(d.pins,d.input.activePin);inputPin.textContent=d.input.activePin<0?'NINGUNO':'GPIO'+d.input.activePin;inputState.textContent=d.input.activePin<0?'--':(d.input.state===0?'PRESIONADO / LOW':'SUELTO / HIGH');pressCount.textContent=d.input.presses;transitionCount.textContent=d.input.transitions;for(let i=1;i<=3;i++){const s=d.sensors[i-1];if(s.hasSample){updateSensorUI(i,s.ok,s.distanceCm,s.durationUs)}}}catch(e){link.textContent='SIN RESPUESTA';link.className='v bad'}}
function updateSensorUI(id,ok,cm,us){const st=document.getElementById('s'+id+'status'),di=document.getElementById('s'+id+'dist'),ec=document.getElementById('s'+id+'echo');if(ok){st.textContent='OK';st.className='ok';di.textContent=Number(cm).toFixed(1)+' cm';ec.textContent=us+' µs'}else{st.textContent='TIMEOUT';st.className='bad';di.textContent='-- cm';ec.textContent='-- µs'}}
async function testOne(id){const st=document.getElementById('s'+id+'status');st.textContent='MIDIENDO...';st.className='';try{const d=await (await fetch('/api/sensor?id='+id,{cache:'no-store'})).json();updateSensorUI(id,d.ok,d.distanceCm,d.durationUs);if(!logPanel.classList.contains('hidden'))loadLog()}catch(e){st.textContent='ERROR WEB';st.className='bad'}}
async function testAll(){try{for(let i=1;i<=3;i++)document.getElementById('s'+i+'status').textContent='MIDIENDO...';const d=await (await fetch('/api/sensors',{cache:'no-store'})).json();for(let i=1;i<=3;i++){const s=d.sensors[i-1];updateSensorUI(i,s.ok,s.distanceCm,s.durationUs)}if(!logPanel.classList.contains('hidden'))loadLog()}catch(e){for(let i=1;i<=3;i++){document.getElementById('s'+i+'status').textContent='ERROR WEB';document.getElementById('s'+i+'status').className='bad'}}}
let autoTimer=null;function toggleAuto(){if(autoTimer){stopAuto();return}testAll();autoTimer=setInterval(testAll,900);autoBtn.textContent='AUTO: ACTIVO'}function stopAuto(){if(autoTimer){clearInterval(autoTimer);autoTimer=null}autoBtn.textContent='AUTO: INICIAR'}
async function armInput(){await fetch('/api/input/set?pin='+pinSelect.value,{cache:'no-store'});refresh()}async function disableInput(){await fetch('/api/input/set?pin=-1',{cache:'no-store'});refresh()}
function toggleLog(){logPanel.classList.toggle('hidden');if(!logPanel.classList.contains('hidden'))loadLog()}async function loadLog(){try{logText.textContent=await (await fetch('/api/log',{cache:'no-store'})).text();logText.scrollTop=logText.scrollHeight}catch(e){logText.textContent='Error leyendo log'}}
refresh();setInterval(refresh,700);
</script>
</body></html>
)HTML";

void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void appendSensorJson(String &json, uint8_t i) {
  const SensorSample &s = lastSensor[i];
  json += "{";
  json += "\"id\":" + String(i + 1) + ",";
  json += "\"trig\":" + String(SENSOR_PINS[i].trig) + ",";
  json += "\"echo\":" + String(SENSOR_PINS[i].echo) + ",";
  json += "\"txsChannel\":" + String(SENSOR_PINS[i].txsChannel) + ",";
  json += "\"hasSample\":" + String(s.ms ? "true" : "false") + ",";
  json += "\"ok\":" + String(s.ok ? "true" : "false") + ",";
  json += "\"durationUs\":" + String(s.durationUs) + ",";
  json += "\"distanceCm\":" + String(s.distanceCm, 2) + ",";
  json += "\"sampleMs\":" + String(s.ms);
  json += "}";
}

void handleStatus() {
  String json;
  json.reserve(2400);
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
  json += "\"pins\":{";
  json += "\"1\":" + String(digitalRead(ANALOG_PIN) ? 1 : 0);
  for (size_t i = 0; i < PASSIVE_MONITOR_PIN_COUNT; i++) {
    uint8_t p = PASSIVE_MONITOR_PINS[i];
    json += ",\"" + String(p) + "\":" + String(digitalRead(p) ? 1 : 0);
  }
  for (uint8_t i = 0; i < 3; i++) {
    json += ",\"" + String(SENSOR_PINS[i].trig) + "\":" + String(digitalRead(SENSOR_PINS[i].trig) ? 1 : 0);
    json += ",\"" + String(SENSOR_PINS[i].echo) + "\":" + String(digitalRead(SENSOR_PINS[i].echo) ? 1 : 0);
  }
  json += "},";
  json += "\"input\":{";
  json += "\"activePin\":" + String((int)activeInputPin) + ",";
  json += "\"state\":" + String(activeInputPin >= 0 ? stableInputState : -1) + ",";
  json += "\"presses\":" + String(inputPressCount) + ",";
  json += "\"transitions\":" + String(inputTransitionCount) + "},";
  json += "\"sensors\":[";
  for (uint8_t i = 0; i < 3; i++) {
    if (i) json += ",";
    appendSensorJson(json, i);
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleSensor() {
  int id = server.hasArg("id") ? server.arg("id").toInt() : 1;
  if (id < 1 || id > 3) {
    server.send(400, "application/json", "{\"error\":\"sensor id must be 1..3\"}");
    return;
  }
  SensorSample s = measureSensor((uint8_t)(id - 1), true);
  String json;
  json.reserve(220);
  json += "{";
  json += "\"id\":" + String(id) + ",";
  json += "\"ok\":" + String(s.ok ? "true" : "false") + ",";
  json += "\"durationUs\":" + String(s.durationUs) + ",";
  json += "\"distanceCm\":" + String(s.distanceCm, 2) + ",";
  json += "\"timeoutUs\":" + String(SENSOR_TIMEOUT_US) + "}";
  server.send(200, "application/json", json);
}

void handleSensors() {
  SensorSample samples[3];
  for (uint8_t i = 0; i < 3; i++) {
    samples[i] = measureSensor(i, true);
    if (i < 2) delay(SENSOR_GAP_MS);
  }
  String json;
  json.reserve(700);
  json += "{\"gapMs\":" + String(SENSOR_GAP_MS) + ",\"sensors\":[";
  for (uint8_t i = 0; i < 3; i++) {
    if (i) json += ",";
    json += "{";
    json += "\"id\":" + String(i + 1) + ",";
    json += "\"ok\":" + String(samples[i].ok ? "true" : "false") + ",";
    json += "\"durationUs\":" + String(samples[i].durationUs) + ",";
    json += "\"distanceCm\":" + String(samples[i].distanceCm, 2) + "}";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleInputSet() {
  int pin = server.hasArg("pin") ? server.arg("pin").toInt() : -1;
  setInputTestPin(pin);
  String json = "{\"activePin\":" + String((int)activeInputPin) + "}";
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
  out.reserve(22000);
  out += "SONIC BONGO / PRISMA - ESP32-S3 N16R8 3 SENSOR PIN LAB\n";
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

  out += "WIRING - TRIG DIRECT / ECHO VIA TXS0108E\n";
  for (uint8_t i = 0; i < 3; i++) {
    out += "S" + String(i + 1) + ": GPIO" + String(SENSOR_PINS[i].trig) + " -> TRIG | ECHO -> B" + String(SENSOR_PINS[i].txsChannel) + " -> A" + String(SENSOR_PINS[i].txsChannel) + " -> GPIO" + String(SENSOR_PINS[i].echo) + "\n";
  }
  out += "HC-SR04 VCC=5V GND=common\n";
  out += "TXS VCCA=3.3V VCCB=5V OE=3.3V GND=common\n";
  out += "Sensor timeout us: " + String(SENSOR_TIMEOUT_US) + "\n";
  out += "Inter-sensor gap ms: " + String(SENSOR_GAP_MS) + "\n\n";

  for (uint8_t i = 0; i < 3; i++) {
    out += "SENSOR " + String(i + 1) + " HISTORY (oldest -> newest)\n";
    if (historyCount[i] == 0) {
      out += "No samples yet.\n";
    } else {
      uint8_t start = (historyHead[i] + HISTORY_SIZE - historyCount[i]) % HISTORY_SIZE;
      for (uint8_t n = 0; n < historyCount[i]; n++) {
        const SensorSample &s = historyData[i][(start + n) % HISTORY_SIZE];
        out += String(s.ms) + " ms | ";
        if (s.ok) out += "OK | " + String(s.durationUs) + " us | " + String(s.distanceCm, 2) + " cm\n";
        else out += "TIMEOUT\n";
      }
    }
    out += "\n";
  }

  out += "INPUT TEST\n";
  out += "Active GPIO: " + String((int)activeInputPin) + "\n";
  out += "State: " + String(activeInputPin >= 0 ? (stableInputState == LOW ? "PRESSED/LOW" : "RELEASED/HIGH") : "DISABLED") + "\n";
  out += "Presses: " + String(inputPressCount) + "\n";
  out += "Transitions: " + String(inputTransitionCount) + "\n\n";

  out += "GPIO STATES\n";
  for (uint8_t i = 0; i < 3; i++) {
    out += "GPIO" + String(SENSOR_PINS[i].trig) + " TRIG S" + String(i + 1) + " = " + String(digitalRead(SENSOR_PINS[i].trig) ? "HIGH" : "LOW") + "\n";
    out += "GPIO" + String(SENSOR_PINS[i].echo) + " ECHO S" + String(i + 1) + " = " + String(digitalRead(SENSOR_PINS[i].echo) ? "HIGH" : "LOW") + "\n";
  }
  out += "GPIO1 ADC current = " + String(analogRead(ANALOG_PIN)) + "\n";
  out += "GPIO1 ADC 64 samples min/avg/max = " + String(adcMin) + "/" + String(adcAvg) + "/" + String(adcMax) + "\n";
  for (size_t i = 0; i < PASSIVE_MONITOR_PIN_COUNT; i++) {
    uint8_t p = PASSIVE_MONITOR_PINS[i];
    out += "GPIO" + String(p) + " = " + String(digitalRead(p) ? "HIGH" : "LOW") + "\n";
  }

  out += "\nRUNTIME LOG\n";
  out += runtimeLog;

  server.sendHeader("Content-Disposition", "attachment; filename=SONIC_BONGO_N16R8_3SENSOR_PIN_LAB_v8.txt");
  server.send(200, "text/plain; charset=utf-8", out);
}

void setup() {
  Serial.begin(115200);
  delay(350);

  pinMode(ANALOG_PIN, INPUT);
  analogReadResolution(12);

  for (size_t i = 0; i < PASSIVE_MONITOR_PIN_COUNT; i++) {
    pinMode(PASSIVE_MONITOR_PINS[i], INPUT);
  }

  for (uint8_t i = 0; i < 3; i++) {
    pinMode(SENSOR_PINS[i].echo, INPUT);
    pinMode(SENSOR_PINS[i].trig, OUTPUT);
    digitalWrite(SENSOR_PINS[i].trig, LOW);
  }

  addLog("SONIC BONGO ESP32-S3 N16R8 3 SENSOR PIN LAB v8 boot");
  addLog("Reset cause: " + resetReasonText(esp_reset_reason()));
  addLog("S1 GPIO13 TRIG direct / GPIO14 ECHO via TXS B1-A1");
  addLog("S2 GPIO15 TRIG direct / GPIO16 ECHO via TXS B2-A2");
  addLog("S3 GPIO17 TRIG direct / GPIO18 ECHO via TXS B3-A3");

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  bool ok = WiFi.softAP(AP_SSID);
  if (ok) addLog("AP started: " + String(AP_SSID) + " IP=" + WiFi.softAPIP().toString());
  else addLog("ERROR: failed to start Access Point");

  addLog("Chip=" + String(ESP.getChipModel()) + " rev=" + String(ESP.getChipRevision()));
  addLog("Flash=" + String(ESP.getFlashChipSize() / (1024 * 1024)) + "MB PSRAM=" + String(ESP.getPsramSize() / (1024 * 1024)) + "MB");

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/sensor", HTTP_GET, handleSensor);
  server.on("/api/sensors", HTTP_GET, handleSensors);
  server.on("/api/input/set", HTTP_GET, handleInputSet);
  server.on("/api/log", HTTP_GET, handleLog);
  server.on("/api/export", HTTP_GET, handleExport);
  server.onNotFound([](){ server.send(404, "text/plain; charset=utf-8", "404"); });
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
    addLog("HEALTH heap=" + String(ESP.getFreeHeap()) + " stations=" + String(WiFi.softAPgetStationNum()) + " adc1=" + String(mn) + "/" + String(av) + "/" + String(mx));
  }
  delay(2);
}
