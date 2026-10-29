/*
  SONIC BONGO / PRISMA
  ESP32-S3 N16R8 Continuous Diagnostic v8.1

  3 x HC-SR04 continuous staggered acquisition.
  TRIG goes DIRECTLY from ESP32. ECHO ONLY goes through TXS0108E.

  S1: GPIO13 -> TRIG      ECHO -> TXS B1 -> A1 -> GPIO14
  S2: GPIO15 -> TRIG      ECHO -> TXS B2 -> A2 -> GPIO16
  S3: GPIO17 -> TRIG      ECHO -> TXS B3 -> A3 -> GPIO18

  Power:
    HC-SR04 VCC -> 5V, GND -> common GND
    TXS VCCA -> 3.3V, VCCB -> 5V, OE -> 3.3V, GND -> common GND

  Continuous schedule:
    one trigger every 25 ms: S1, S2, S3, S1...
    each sensor is triggered every ~75 ms (~13.3 Hz).

  Wi-Fi AP:
    SSID: PRISMA-S3-DIAG
    Password: OPEN
    Web: http://192.168.4.1
*/

#include <WiFi.h>
#include <WebServer.h>
#include <esp_system.h>
#include <driver/gpio.h>
#include <math.h>

WebServer server(80);

static const char* FW_VERSION = "8.1-continuous";
static const char* AP_SSID = "PRISMA-S3-DIAG";
static const uint8_t ANALOG_PIN = 1;
static const uint32_t SENSOR_SLOT_US = 25000;      // global trigger spacing
static const uint32_t ECHO_TIMEOUT_US = 24000;    // enough for ~4 m
static const uint32_t LOG_SAMPLE_MS = 10000;
static const uint32_t INPUT_DEBOUNCE_MS = 20;
static const size_t MAX_LOG_CHARS = 18000;
static const float HIT_NEAR_CM = 20.0f;
static const float HIT_SPEED_CM_S = 80.0f;
static const uint32_t HIT_REFRACTORY_MS = 250;

struct SensorPins { uint8_t trig; uint8_t echo; uint8_t txs; };
const SensorPins SENSOR_PINS[3] = {
  {13,14,1}, {15,16,2}, {17,18,3}
};

const uint8_t SAFE_INPUT_TEST_PINS[] = {
  2,4,5,6,7,8,9,10,11,12,21,39,40,41,42,47
};
const size_t SAFE_INPUT_TEST_PIN_COUNT = sizeof(SAFE_INPUT_TEST_PINS)/sizeof(SAFE_INPUT_TEST_PINS[0]);

const uint8_t PASSIVE_MONITOR_PINS[] = {
  2,4,5,6,7,8,9,10,11,12,21,39,40,41,42,47
};
const size_t PASSIVE_MONITOR_PIN_COUNT = sizeof(PASSIVE_MONITOR_PINS)/sizeof(PASSIVE_MONITOR_PINS[0]);

struct MotionSample {
  uint32_t ms;
  uint32_t echoUs;
  float distanceCm;
  float velocityCms;
  float accelCms2;
  bool ok;
};

struct SensorState {
  MotionSample last;
  float prevDistance;
  uint32_t prevGoodMs;
  float filteredVelocity;
  float prevFilteredVelocity;
  float filteredAccel;
  bool hasPrevious;
  uint32_t samples;
  uint32_t valid;
  uint32_t timeouts;
  float peakApproachCms;
  float peakAccelCms2;
  uint32_t hits;
  uint32_t lastHitMs;
};

static const uint8_t HISTORY_SIZE = 96;
MotionSample sensorHistory[3][HISTORY_SIZE];
uint8_t historyHead[3] = {0,0,0};
uint8_t historyCount[3] = {0,0,0};
SensorState S[3] = {};

volatile uint32_t echoRiseUs[3] = {0,0,0};
volatile uint32_t echoPulseUs[3] = {0,0,0};
volatile bool echoDone[3] = {false,false,false};
volatile bool echoArmed[3] = {false,false,false};

bool continuousEnabled = true;
int8_t activeSensor = -1;
uint8_t nextSensor = 0;
uint32_t activeStartedUs = 0;
uint32_t nextTriggerUs = 0;

String runtimeLog;
uint32_t lastPeriodicLog = 0;

int8_t activeInputPin = -1;
int lastRawInputState = HIGH;
int stableInputState = HIGH;
uint32_t inputLastRawChangeMs = 0;
uint32_t inputPressCount = 0;
uint32_t inputTransitionCount = 0;

void addLog(const String& msg) {
  String line = "[" + String(millis()) + " ms] " + msg + "\n";
  runtimeLog += line;
  if (runtimeLog.length() > MAX_LOG_CHARS) {
    size_t cut = runtimeLog.length() - MAX_LOG_CHARS;
    int nl = runtimeLog.indexOf('\n', cut);
    if (nl >= 0) runtimeLog.remove(0, nl + 1); else runtimeLog.remove(0, cut);
  }
  Serial.print(line);
}

String resetReasonText(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXTERNAL";
    case ESP_RST_SW: return "SOFTWARE";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    default: return String((int)reason);
  }
}

inline void IRAM_ATTR handleEchoISR(uint8_t i) {
  if (!echoArmed[i]) return;
  int level = gpio_get_level((gpio_num_t)SENSOR_PINS[i].echo);
  uint32_t now = micros();
  if (level) {
    echoRiseUs[i] = now;
  } else if (echoRiseUs[i] != 0) {
    echoPulseUs[i] = now - echoRiseUs[i];
    echoDone[i] = true;
    echoArmed[i] = false;
  }
}
void IRAM_ATTR echoISR0(){ handleEchoISR(0); }
void IRAM_ATTR echoISR1(){ handleEchoISR(1); }
void IRAM_ATTR echoISR2(){ handleEchoISR(2); }

void storeHistory(uint8_t i, const MotionSample& m) {
  sensorHistory[i][historyHead[i]] = m;
  historyHead[i] = (historyHead[i] + 1) % HISTORY_SIZE;
  if (historyCount[i] < HISTORY_SIZE) historyCount[i]++;
}

void processMeasurement(uint8_t i, uint32_t pulseUs, bool ok) {
  uint32_t nowMs = millis();
  MotionSample m = {nowMs, pulseUs, 0, S[i].filteredVelocity, S[i].filteredAccel, ok};
  S[i].samples++;

  if (!ok || pulseUs == 0) {
    S[i].timeouts++;
    S[i].last = m;
    storeHistory(i, m);
    return;
  }

  m.distanceCm = ((float)pulseUs * 0.0343f) / 2.0f;
  S[i].valid++;

  if (S[i].hasPrevious && nowMs > S[i].prevGoodMs) {
    float dt = (nowMs - S[i].prevGoodMs) / 1000.0f;
    if (dt > 0.02f && dt < 0.30f) {
      float rawV = (m.distanceCm - S[i].prevDistance) / dt;
      float oldV = S[i].filteredVelocity;
      S[i].filteredVelocity = 0.45f * rawV + 0.55f * S[i].filteredVelocity;
      float rawA = (S[i].filteredVelocity - oldV) / dt;
      S[i].filteredAccel = 0.30f * rawA + 0.70f * S[i].filteredAccel;
      m.velocityCms = S[i].filteredVelocity;
      m.accelCms2 = S[i].filteredAccel;

      if (S[i].filteredVelocity < 0) {
        float approach = -S[i].filteredVelocity;
        if (approach > S[i].peakApproachCms) S[i].peakApproachCms = approach;
      }
      float absA = fabsf(S[i].filteredAccel);
      if (absA > S[i].peakAccelCms2) S[i].peakAccelCms2 = absA;

      if (m.distanceCm <= HIT_NEAR_CM && S[i].filteredVelocity <= -HIT_SPEED_CM_S &&
          (nowMs - S[i].lastHitMs) >= HIT_REFRACTORY_MS) {
        S[i].hits++;
        S[i].lastHitMs = nowMs;
        addLog("S" + String(i+1) + " HIT estimated distance=" + String(m.distanceCm,1) +
               "cm approach=" + String(-S[i].filteredVelocity,1) + "cm/s accel=" +
               String(S[i].filteredAccel,1) + "cm/s2");
      }
    }
  } else {
    S[i].filteredVelocity = 0;
    S[i].filteredAccel = 0;
    m.velocityCms = 0;
    m.accelCms2 = 0;
    S[i].hasPrevious = true;
  }

  S[i].prevDistance = m.distanceCm;
  S[i].prevGoodMs = nowMs;
  S[i].prevFilteredVelocity = S[i].filteredVelocity;
  S[i].last = m;
  storeHistory(i, m);
}

void triggerSensor(uint8_t i) {
  noInterrupts();
  echoRiseUs[i] = 0;
  echoPulseUs[i] = 0;
  echoDone[i] = false;
  echoArmed[i] = true;
  interrupts();

  digitalWrite(SENSOR_PINS[i].trig, LOW);
  delayMicroseconds(3);
  digitalWrite(SENSOR_PINS[i].trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(SENSOR_PINS[i].trig, LOW);

  activeSensor = i;
  activeStartedUs = micros();
}

void serviceSensors() {
  if (!continuousEnabled) return;
  uint32_t nowUs = micros();

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
    } else if ((uint32_t)(nowUs - activeStartedUs) >= ECHO_TIMEOUT_US) {
      noInterrupts(); echoArmed[i] = false; interrupts();
      processMeasurement(i, 0, false);
      activeSensor = -1;
    }
  }

  if (activeSensor < 0 && (int32_t)(nowUs - nextTriggerUs) >= 0) {
    uint8_t i = nextSensor;
    nextSensor = (nextSensor + 1) % 3;
    nextTriggerUs = nowUs + SENSOR_SLOT_US;
    triggerSensor(i);
  }
}

bool isSafeInputTestPin(int pin) {
  for (size_t i=0;i<SAFE_INPUT_TEST_PIN_COUNT;i++) if ((int)SAFE_INPUT_TEST_PINS[i]==pin) return true;
  return false;
}
void setInputTestPin(int pin) {
  if (activeInputPin >= 0 && isSafeInputTestPin(activeInputPin)) pinMode(activeInputPin, INPUT);
  activeInputPin = -1; inputPressCount=0; inputTransitionCount=0;
  if (pin < 0) { addLog("INPUT TEST disabled"); return; }
  if (!isSafeInputTestPin(pin)) { addLog("INPUT TEST rejected GPIO" + String(pin)); return; }
  activeInputPin = pin;
  pinMode(activeInputPin, INPUT_PULLUP);
  lastRawInputState = stableInputState = digitalRead(activeInputPin);
  inputLastRawChangeMs = millis();
  addLog("INPUT TEST GPIO" + String(pin) + " INPUT_PULLUP; button to GND");
}
void serviceInputTest() {
  if (activeInputPin < 0) return;
  int raw = digitalRead(activeInputPin);
  uint32_t now = millis();
  if (raw != lastRawInputState) { lastRawInputState = raw; inputLastRawChangeMs = now; }
  if ((now - inputLastRawChangeMs) >= INPUT_DEBOUNCE_MS && raw != stableInputState) {
    stableInputState = raw; inputTransitionCount++;
    if (stableInputState == LOW) inputPressCount++;
  }
}

String movementLabel(uint8_t i) {
  if (!S[i].last.ok) return "TIMEOUT";
  float v = S[i].filteredVelocity;
  if (v < -20) return "ACERCANDO";
  if (v > 20) return "ALEJANDO";
  return "ESTABLE";
}

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="es"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SONIC BONGO · Continuous Sensor Lab v8.1</title>
<style>
:root{--bg:#080c11;--panel:#111923;--line:#2b3948;--text:#edf4fa;--muted:#8fa2b5;--ok:#45ef89;--low:#3b4b5a;--bad:#ff6262;--warn:#ffbf47;--power:#58d8ff;--info:#b28cff;--trig:#54d7ff;--echo:#ffae5c;--test:#ff8fd8}
*{box-sizing:border-box}body{margin:0;background:linear-gradient(180deg,#080c11,#0e1620);color:var(--text);font-family:system-ui,-apple-system,"Segoe UI",sans-serif;min-height:100vh}header,.stats,.notice,.panel,.boardWrap,.legend,.actions,.logPanel{max-width:1100px;margin-left:auto;margin-right:auto}header{padding:18px 16px 6px}h1{font-size:22px;margin:0 0 4px}.sub,.hint{font-size:11px;color:var(--muted)}.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(125px,1fr));gap:8px;padding:8px 16px}.card,.panel,.logPanel{background:var(--panel);border:1px solid var(--line);border-radius:11px}.card{padding:9px}.k{font-size:9px;color:var(--muted);letter-spacing:.08em}.v{font-size:15px;font-weight:850;margin-top:3px}.ok{color:var(--ok)}.bad{color:var(--bad)}.notice{margin-top:8px;padding:10px 12px;border:1px solid #594820;background:#211b10;color:#ffda8b;border-radius:10px;font-size:12px}.panel{margin-top:12px;padding:14px 16px}.sensorGrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:9px}.sensor{background:#091018;border:1px solid #295044;border-radius:10px;padding:11px}.sensor h3{margin:0 0 8px}.row{display:flex;justify-content:space-between;gap:10px;margin:5px 0;font-size:12px}.row b{font-size:14px}.big{font-size:26px;font-weight:900;color:#68f0d1}.actions{display:flex;flex-wrap:wrap;gap:8px;padding:10px 16px 4px}button,a.btn,select{border:1px solid #40566c;background:#182635;color:#edf4fa;padding:9px 11px;border-radius:9px;font-weight:800;font-size:12px;text-decoration:none}button{cursor:pointer}.go{border-color:#277762;background:#12382f}.stop{border-color:#743c3c;background:#351919}.test{border-color:#8a4f78;background:#3a1731}.wiring{margin-top:10px;background:#060b10;border:1px solid var(--line);border-radius:8px;padding:9px;font:11px/1.65 ui-monospace,Consolas,monospace;color:#bdcad6}
.boardWrap{padding:14px 12px 4px}.board{display:grid;grid-template-columns:minmax(180px,1fr) minmax(190px,235px) minmax(180px,1fr);gap:10px;background:#202a35;border:2px solid #465767;border-radius:24px;padding:14px}.pincol{display:flex;flex-direction:column;gap:5px}.pin{min-height:31px;display:flex;align-items:center;gap:7px;padding:4px 7px;background:var(--panel);border:1px solid var(--line);border-radius:8px}.right .pin{flex-direction:row-reverse;text-align:right}.dot{width:12px;height:12px;flex:0 0 12px;border-radius:50%;background:var(--low)}.pin.high .dot{background:var(--ok);box-shadow:0 0 12px var(--ok)}.pin.locked .dot{background:var(--warn)}.pin.power .dot{background:var(--power)}.pin.ground .dot{background:#080808;border:1px solid #59616a}.pin.info .dot{background:var(--info)}.pin.trigpin{border-color:#236b83}.pin.trigpin .dot{background:var(--trig)}.pin.echopin{border-color:#8d5d2c}.pin.echopin .dot{background:var(--echo)}.pin.activeTest{border-color:var(--test);box-shadow:0 0 0 1px var(--test)}.pin.activeTest .dot{background:var(--test)}.name{font-size:12px;font-weight:850}.desc{font-size:9px;color:var(--muted)}.middle{min-height:775px;display:flex;flex-direction:column;align-items:center;justify-content:space-between;padding:17px 8px 13px;background:#0d141c;border:1px solid #344251;border-radius:18px}.antenna{width:87%;height:88px;border:2px solid #6f5a36;border-radius:9px;background:repeating-linear-gradient(90deg,transparent 0 12px,#9d7a3d 12px 20px,transparent 20px 31px)}.module{width:88%;height:292px;display:flex;align-items:center;justify-content:center;text-align:center;font-weight:900;color:#171717;background:linear-gradient(145deg,#c8b27c,#82734f);border-radius:9px}.boardname{text-align:center;font-size:12px;color:#aebdca;line-height:1.5}.usbrow{display:flex;gap:14px}.usb{width:56px;height:26px;border-radius:5px;background:#aab3bb;border:2px solid #68747e}.legend{display:flex;flex-wrap:wrap;gap:12px;padding:10px 16px;color:var(--muted);font-size:11px}.mini{width:9px;height:9px;border-radius:50%;display:inline-block}pre{white-space:pre-wrap;background:#05090d;border:1px solid var(--line);border-radius:9px;padding:10px;max-height:320px;overflow:auto;font-size:11px}.hidden{display:none}@media(max-width:720px){.board{grid-template-columns:minmax(100px,1fr) 110px minmax(100px,1fr);gap:5px;padding:7px}.middle{min-height:742px;padding:8px 3px}.module{height:235px;font-size:11px}.desc{display:none}.name{font-size:10px}}
</style></head><body>
<header><h1>SONIC BONGO · CONTINUOUS SENSOR LAB v8.1</h1><div class="sub">ESP32-S3 N16R8 · 3 HC-SR04 · distancia + velocidad + aceleración estimada</div></header>
<section class="stats"><div class="card"><div class="k">CHIP</div><div class="v" id="chip">...</div></div><div class="card"><div class="k">FLASH</div><div class="v" id="flash">...</div></div><div class="card"><div class="k">PSRAM</div><div class="v" id="psram">...</div></div><div class="card"><div class="k">ADQUISICIÓN</div><div class="v" id="run">...</div></div><div class="card"><div class="k">SLOT GLOBAL</div><div class="v">25 ms</div></div><div class="card"><div class="k">POR SENSOR</div><div class="v">~13.3 Hz</div></div></section>
<div class="notice"><b>TRIG directo al ESP32. Sólo ECHO pasa por TXS0108E.</b> Los tres sensores se disparan intercalados; nunca simultáneamente.</div>
<section class="panel"><h2>Movimiento continuo</h2><div class="sensorGrid">
<div class="sensor"><h3>S1 · GPIO13 / GPIO14</h3><div class="big" id="s1d">-- cm</div><div class="row"><span>Movimiento</span><b id="s1m">--</b></div><div class="row"><span>Velocidad</span><b id="s1v">--</b></div><div class="row"><span>Acel. estimada</span><b id="s1a">--</b></div><div class="row"><span>Peak aproximación</span><b id="s1p">--</b></div><div class="row"><span>Golpes estimados</span><b id="s1h">0</b></div><div class="row"><span>Válidas / timeout</span><b id="s1q">--</b></div></div>
<div class="sensor"><h3>S2 · GPIO15 / GPIO16</h3><div class="big" id="s2d">-- cm</div><div class="row"><span>Movimiento</span><b id="s2m">--</b></div><div class="row"><span>Velocidad</span><b id="s2v">--</b></div><div class="row"><span>Acel. estimada</span><b id="s2a">--</b></div><div class="row"><span>Peak aproximación</span><b id="s2p">--</b></div><div class="row"><span>Golpes estimados</span><b id="s2h">0</b></div><div class="row"><span>Válidas / timeout</span><b id="s2q">--</b></div></div>
<div class="sensor"><h3>S3 · GPIO17 / GPIO18</h3><div class="big" id="s3d">-- cm</div><div class="row"><span>Movimiento</span><b id="s3m">--</b></div><div class="row"><span>Velocidad</span><b id="s3v">--</b></div><div class="row"><span>Acel. estimada</span><b id="s3a">--</b></div><div class="row"><span>Peak aproximación</span><b id="s3p">--</b></div><div class="row"><span>Golpes estimados</span><b id="s3h">0</b></div><div class="row"><span>Válidas / timeout</span><b id="s3q">--</b></div></div>
</div><div class="actions" style="padding-left:0"><button class="go" onclick="setRun(1)">INICIAR CONTINUO</button><button class="stop" onclick="setRun(0)">PAUSAR</button></div><div class="wiring">S1: GPIO13 → TRIG &nbsp;&nbsp; ECHO → B1 → A1 → GPIO14<br>S2: GPIO15 → TRIG &nbsp;&nbsp; ECHO → B2 → A2 → GPIO16<br>S3: GPIO17 → TRIG &nbsp;&nbsp; ECHO → B3 → A3 → GPIO18<br><br>HC-SR04 VCC=5V · GND=común &nbsp;&nbsp; TXS VCCA=3.3V · VCCB=5V · OE=3.3V · GND=común</div><div class="hint">La aceleración es una estimación derivada de distancia/tiempo y filtrada. Para impacto físico real conviene luego sumar piezo o IMU.</div></section>
<section class="panel"><h2>Pulsador · PIN LAB</h2><div class="actions" style="padding-left:0"><select id="pinSel"><option>4</option><option>5</option><option>6</option><option>7</option><option>8</option><option>9</option><option>10</option><option>11</option><option>12</option><option>21</option><option>39</option><option>40</option><option>41</option><option>42</option><option>47</option><option>2</option></select><button class="test" onclick="armPin()">ACTIVAR INPUT_PULLUP</button><button class="stop" onclick="disablePin()">DESACTIVAR</button></div><div class="row"><span>GPIO</span><b id="ip">NINGUNO</b></div><div class="row"><span>Estado</span><b id="is">--</b></div><div class="row"><span>Pulsaciones / cambios</span><b id="ic">0 / 0</b></div><div class="wiring">GPIO elegido ─── PULSADOR ─── GND &nbsp;&nbsp; suelto=HIGH · presionado=LOW</div></section>
<div class="boardWrap"><div class="board"><div class="pincol left" id="leftPins"></div><div class="middle"><div class="antenna"></div><div class="module">ESP32-S3<br>WROOM-1<br><br>N16R8</div><div class="boardname">CONTINUOUS SENSOR LAB<br>16 MB FLASH<br>8 MB PSRAM</div><div>BOOT &nbsp;&nbsp; RESET</div><div class="usbrow"><div class="usb"></div><div class="usb"></div></div></div><div class="pincol right" id="rightPins"></div></div></div>
<div class="legend"><span><i class="mini" style="background:#54d7ff"></i>TRIG directo</span><span><i class="mini" style="background:#ffae5c"></i>ECHO vía TXS</span><span><i class="mini" style="background:#ff8fd8"></i>GPIO test</span><span><i class="mini" style="background:#ffbf47"></i>reservado</span></div>
<div class="actions"><a class="btn" href="/api/export">EXPORTAR TXT</a><button onclick="toggleLog()">LOG</button></div><div class="logPanel hidden" id="logPanel"><pre id="logText"></pre></div>
<script>
const LEFT=[["3V3",null,"power","3.3 V"],["3V3",null,"power","3.3 V"],["RST",null,"info","EN / RESET"],["GPIO4",4,"gpio","K1 / test"],["GPIO5",5,"gpio","K2 / test"],["GPIO6",6,"gpio","K3 / test"],["GPIO7",7,"gpio","K4 / test"],["GPIO15",15,"trigpin","TRIG S2"],["GPIO16",16,"echopin","ECHO S2 / TXS"],["GPIO17",17,"trigpin","TRIG S3"],["GPIO18",18,"echopin","ECHO S3 / TXS"],["GPIO8",8,"gpio","OLED SDA"],["GPIO3",3,"locked","strapping / JTAG"],["GPIO46",46,"locked","strapping"],["GPIO9",9,"gpio","OLED SCL"],["GPIO10",10,"gpio","Touch grande"],["GPIO11",11,"gpio","Touch 1"],["GPIO12",12,"gpio","Touch 2"],["GPIO13",13,"trigpin","TRIG S1"],["GPIO14",14,"echopin","ECHO S1 / TXS"],["5V",null,"power","5V / VIN"],["GND",null,"ground","masa"]];
const RIGHT=[["GND",null,"ground","masa"],["TX / GPIO43",43,"locked","UART0"],["RX / GPIO44",44,"locked","UART0"],["GPIO1",1,"gpio","ADC"],["GPIO2",2,"gpio","test"],["GPIO42",42,"gpio","test"],["GPIO41",41,"gpio","test"],["GPIO40",40,"gpio","test"],["GPIO39",39,"gpio","test"],["GPIO38",38,"locked","RGB posible"],["GPIO37",37,"locked","Octal PSRAM"],["GPIO36",36,"locked","Octal PSRAM"],["GPIO35",35,"locked","Octal PSRAM"],["GPIO0",0,"locked","BOOT"],["GPIO45",45,"locked","strapping"],["GPIO48",48,"locked","reservado"],["GPIO47",47,"gpio","test"],["GPIO21",21,"gpio","ARM / MUTE"],["GPIO20",20,"locked","USB D+"],["GPIO19",19,"locked","USB D-"],["GND",null,"ground","masa"],["GND",null,"ground","masa"]];
function mk(p){const e=document.createElement('div');e.className='pin '+(p[2]==='gpio'?'':p[2]);e.dataset.gpio=p[1]===null?'':p[1];e.innerHTML='<span class="dot"></span><div><div class="name">'+p[0]+'</div><div class="desc">'+p[3]+'</div></div>';return e}LEFT.forEach(p=>leftPins.appendChild(mk(p)));RIGHT.forEach(p=>rightPins.appendChild(mk(p)));
function sensor(i,s){document.getElementById('s'+i+'d').textContent=s.ok?s.distanceCm.toFixed(1)+' cm':'TIMEOUT';document.getElementById('s'+i+'m').textContent=s.movement;document.getElementById('s'+i+'v').textContent=s.velocityCms.toFixed(1)+' cm/s';document.getElementById('s'+i+'a').textContent=s.accelCms2.toFixed(0)+' cm/s²';document.getElementById('s'+i+'p').textContent=s.peakApproachCms.toFixed(1)+' cm/s';document.getElementById('s'+i+'h').textContent=s.hits;document.getElementById('s'+i+'q').textContent=s.valid+' / '+s.timeouts}
function applyPins(d){document.querySelectorAll('.pin').forEach(e=>{e.classList.remove('activeTest');const p=e.dataset.gpio;if(p!==''&&d.input.active&&Number(p)===d.input.pin)e.classList.add('activeTest')})}
async function refresh(){try{const d=await (await fetch('/api/status',{cache:'no-store'})).json();chip.textContent=d.chip;flash.textContent=d.flashMB+' MB';psram.textContent=d.psramMB+' MB';run.textContent=d.continuous?'CORRIENDO':'PAUSADO';run.className='v '+(d.continuous?'ok':'bad');sensor(1,d.sensors[0]);sensor(2,d.sensors[1]);sensor(3,d.sensors[2]);ip.textContent=d.input.active?'GPIO'+d.input.pin:'NINGUNO';is.textContent=d.input.active?(d.input.pressed?'PRESIONADO / LOW':'SUELTO / HIGH'):'--';ic.textContent=d.input.presses+' / '+d.input.transitions;applyPins(d)}catch(e){run.textContent='WEB ERROR';run.className='v bad'}}
async function setRun(v){await fetch('/api/continuous?enable='+v);refresh()}async function armPin(){await fetch('/api/input/arm?pin='+pinSel.value);refresh()}async function disablePin(){await fetch('/api/input/disable');refresh()}function toggleLog(){logPanel.classList.toggle('hidden');if(!logPanel.classList.contains('hidden'))loadLog()}async function loadLog(){logText.textContent=await (await fetch('/api/log',{cache:'no-store'})).text();logText.scrollTop=logText.scrollHeight}refresh();setInterval(refresh,150);setInterval(()=>{if(!logPanel.classList.contains('hidden'))loadLog()},2000);
</script></body></html>
)HTML";

void handleRoot(){ server.send_P(200,"text/html; charset=utf-8",INDEX_HTML); }

void appendSensorJson(String& j, uint8_t i) {
  MotionSample &m = S[i].last;
  j += "{";
  j += "\"ok\":" + String(m.ok?"true":"false") + ",";
  j += "\"distanceCm\":" + String(m.distanceCm,2) + ",";
  j += "\"echoUs\":" + String(m.echoUs) + ",";
  j += "\"velocityCms\":" + String(S[i].filteredVelocity,2) + ",";
  j += "\"accelCms2\":" + String(S[i].filteredAccel,2) + ",";
  j += "\"peakApproachCms\":" + String(S[i].peakApproachCms,2) + ",";
  j += "\"peakAccelCms2\":" + String(S[i].peakAccelCms2,2) + ",";
  j += "\"hits\":" + String(S[i].hits) + ",";
  j += "\"samples\":" + String(S[i].samples) + ",";
  j += "\"valid\":" + String(S[i].valid) + ",";
  j += "\"timeouts\":" + String(S[i].timeouts) + ",";
  j += "\"movement\":\"" + movementLabel(i) + "\"}";
}

void handleStatus(){
  String j; j.reserve(2600);
  j += "{\"fw\":\""+String(FW_VERSION)+"\",\"chip\":\""+String(ESP.getChipModel())+"\",";
  j += "\"flashMB\":"+String(ESP.getFlashChipSize()/(1024*1024))+",\"psramMB\":"+String(ESP.getPsramSize()/(1024*1024))+",";
  j += "\"continuous\":"+String(continuousEnabled?"true":"false")+",\"slotMs\":25,\"perSensorHz\":13.33,";
  j += "\"sensors\":["; appendSensorJson(j,0); j+=","; appendSensorJson(j,1); j+=","; appendSensorJson(j,2); j += "],";
  j += "\"input\":{\"active\":"+String(activeInputPin>=0?"true":"false")+",\"pin\":"+String(activeInputPin)+",\"pressed\":"+String(activeInputPin>=0&&stableInputState==LOW?"true":"false")+",\"presses\":"+String(inputPressCount)+",\"transitions\":"+String(inputTransitionCount)+"}}";
  server.send(200,"application/json",j);
}

void handleContinuous(){
  if (server.hasArg("enable")) continuousEnabled = server.arg("enable").toInt()!=0;
  if (!continuousEnabled && activeSensor>=0) { noInterrupts(); echoArmed[(uint8_t)activeSensor]=false; interrupts(); activeSensor=-1; }
  if (continuousEnabled) nextTriggerUs = micros()+1000;
  addLog(String("CONTINUOUS ") + (continuousEnabled?"ON":"OFF"));
  server.send(200,"application/json",String("{\"continuous\":")+(continuousEnabled?"true":"false")+"}");
}
void handleInputArm(){ int pin=server.hasArg("pin")?server.arg("pin").toInt():-1; setInputTestPin(pin); server.send(200,"text/plain","OK"); }
void handleInputDisable(){ setInputTestPin(-1); server.send(200,"text/plain","OK"); }
void handleLog(){ server.send(200,"text/plain; charset=utf-8",runtimeLog); }

void handleExport(){
  String out; out.reserve(30000);
  out += "SONIC BONGO / PRISMA - CONTINUOUS SENSOR LAB\nFirmware: "+String(FW_VERSION)+"\n";
  out += "Chip: "+String(ESP.getChipModel())+" rev="+String(ESP.getChipRevision())+"\n";
  out += "Flash MB: "+String(ESP.getFlashChipSize()/(1024*1024))+" PSRAM MB: "+String(ESP.getPsramSize()/(1024*1024))+"\n";
  out += "Reset: "+resetReasonText(esp_reset_reason())+" uptime_ms="+String(millis())+"\n";
  out += "Schedule: 25ms global slot; ~75ms/sensor; ~13.3Hz/sensor\n";
  out += "Wiring: S1 GPIO13 TRIG direct, ECHO B1/A1 GPIO14; S2 GPIO15/B2/A2/GPIO16; S3 GPIO17/B3/A3/GPIO18\n\n";
  for (uint8_t i=0;i<3;i++) {
    out += "SENSOR "+String(i+1)+" summary\n";
    out += "samples="+String(S[i].samples)+" valid="+String(S[i].valid)+" timeouts="+String(S[i].timeouts)+" hits="+String(S[i].hits)+"\n";
    out += "last_distance_cm="+String(S[i].last.distanceCm,2)+" velocity_cm_s="+String(S[i].filteredVelocity,2)+" accel_cm_s2="+String(S[i].filteredAccel,2)+"\n";
    out += "peak_approach_cm_s="+String(S[i].peakApproachCms,2)+" peak_accel_cm_s2="+String(S[i].peakAccelCms2,2)+"\n";
    out += "history oldest->newest: ms,ok,echo_us,distance_cm,velocity_cm_s,accel_cm_s2\n";
    uint8_t start=(historyHead[i]+HISTORY_SIZE-historyCount[i])%HISTORY_SIZE;
    for(uint8_t n=0;n<historyCount[i];n++){
      MotionSample &m=sensorHistory[i][(start+n)%HISTORY_SIZE];
      out += String(m.ms)+","+String(m.ok?1:0)+","+String(m.echoUs)+","+String(m.distanceCm,2)+","+String(m.velocityCms,2)+","+String(m.accelCms2,2)+"\n";
    }
    out += "\n";
  }
  out += "RUNTIME LOG\n"+runtimeLog;
  server.sendHeader("Content-Disposition","attachment; filename=SONIC_BONGO_CONTINUOUS_DIAG_v8_1.txt");
  server.send(200,"text/plain; charset=utf-8",out);
}

void setup(){
  Serial.begin(115200); delay(300);
  pinMode(ANALOG_PIN,INPUT); analogReadResolution(12);
  for(size_t i=0;i<PASSIVE_MONITOR_PIN_COUNT;i++) pinMode(PASSIVE_MONITOR_PINS[i],INPUT);
  for(uint8_t i=0;i<3;i++){
    pinMode(SENSOR_PINS[i].trig,OUTPUT); digitalWrite(SENSOR_PINS[i].trig,LOW);
    pinMode(SENSOR_PINS[i].echo,INPUT);
  }
  attachInterrupt(digitalPinToInterrupt(SENSOR_PINS[0].echo),echoISR0,CHANGE);
  attachInterrupt(digitalPinToInterrupt(SENSOR_PINS[1].echo),echoISR1,CHANGE);
  attachInterrupt(digitalPinToInterrupt(SENSOR_PINS[2].echo),echoISR2,CHANGE);

  WiFi.mode(WIFI_AP); WiFi.setSleep(false); WiFi.softAP(AP_SSID);
  addLog("BOOT continuous diagnostic v8.1");
  addLog("AP "+String(AP_SSID)+" IP="+WiFi.softAPIP().toString());
  addLog("TRIG direct: GPIO13/15/17; ECHO via TXS to GPIO14/16/18");
  addLog("Continuous scheduler: 25ms global, ~13.3Hz per sensor");

  server.on("/",HTTP_GET,handleRoot);
  server.on("/api/status",HTTP_GET,handleStatus);
  server.on("/api/continuous",HTTP_GET,handleContinuous);
  server.on("/api/input/arm",HTTP_GET,handleInputArm);
  server.on("/api/input/disable",HTTP_GET,handleInputDisable);
  server.on("/api/log",HTTP_GET,handleLog);
  server.on("/api/export",HTTP_GET,handleExport);
  server.onNotFound([](){server.send(404,"text/plain","404");});
  server.begin();
  nextTriggerUs = micros()+100000;
}

void loop(){
  server.handleClient();
  serviceSensors();
  serviceInputTest();
  uint32_t now=millis();
  if(now-lastPeriodicLog>=LOG_SAMPLE_MS){
    lastPeriodicLog=now;
    addLog("HEALTH heap="+String(ESP.getFreeHeap())+" stations="+String(WiFi.softAPgetStationNum())+
      " S1="+String(S[0].valid)+"/"+String(S[0].timeouts)+
      " S2="+String(S[1].valid)+"/"+String(S[1].timeouts)+
      " S3="+String(S[2].valid)+"/"+String(S[2].timeouts));
  }
  delay(1);
}
