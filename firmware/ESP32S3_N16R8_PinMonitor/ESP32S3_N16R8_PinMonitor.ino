/*
  SONIC BONGO / PRISMA
  ESP32-S3 N16R8 Diagnostic v5

  Sensor 1 diagnostic wiring through TXS0108E:
    ESP32 GPIO13 -> TXS A1 -> B1 -> HC-SR04 TRIG
    ESP32 GPIO14 <- TXS A2 <- B2 <- HC-SR04 ECHO

  Power:
    ESP32 5V <- breadboard 5V
    ESP32 GND <- common GND
    TXS VCCA <- 3.3V
    TXS VCCB <- 5V
    TXS OE   <- 3.3V
    TXS GND  <- common GND
    HC-SR04 VCC <- 5V
    HC-SR04 GND <- common GND

  IMPORTANT:
  - GPIO13 is the ONLY diagnostic pin actively driven by this firmware.
  - It is kept LOW except for the standard ~10 us HC-SR04 trigger pulse.
  - GPIO14 is input-only.
  - Other monitored GPIO are configured INPUT without internal pull resistors.

  Wi-Fi AP:
    SSID: PRISMA-S3-DIAG
    Password: OPEN
    Web: http://192.168.4.1
*/

#include <WiFi.h>
#include <WebServer.h>
#include <esp_system.h>

WebServer server(80);

static const char* FW_VERSION = "5.0";
static const char* AP_SSID = "PRISMA-S3-DIAG";
static const uint8_t ANALOG_PIN = 1;
static const uint8_t S1_TRIG_PIN = 13;
static const uint8_t S1_ECHO_PIN = 14;
static const uint32_t S1_TIMEOUT_US = 30000;
static const uint32_t LOG_SAMPLE_MS = 10000;
static const size_t MAX_LOG_CHARS = 9000;

const uint8_t MONITOR_PINS[] = {
  2,
  4, 5, 6, 7,
  8, 9, 10, 11, 12,
  15, 16, 17, 18,
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

static const uint8_t SENSOR_HISTORY_SIZE = 24;
SensorSample sensorHistory[SENSOR_HISTORY_SIZE];
uint8_t sensorHistoryCount = 0;
uint8_t sensorHistoryHead = 0;
SensorSample lastSensor = {0, 0, 0.0f, false};

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

  // Ensure a clean LOW before the standard HC-SR04 trigger pulse.
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
    if (s.ok) {
      addLog("S1 OK echo=" + String(s.durationUs) + " us distance=" + String(s.distanceCm, 2) + " cm");
    } else {
      addLog("S1 TIMEOUT: no ECHO HIGH detected within " + String(S1_TIMEOUT_US) + " us");
    }
  }
  return s;
}

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="es">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SONIC BONGO · Sensor 1 Diagnostic v5</title>
<style>
:root{--bg:#080c11;--panel:#111923;--panel2:#182330;--line:#2b3948;--text:#edf4fa;--muted:#8fa2b5;--high:#45ef89;--low:#3b4b5a;--locked:#ffbf47;--power:#58d8ff;--info:#b28cff;--danger:#ff6262;--sensor:#68f0d1}
*{box-sizing:border-box}body{margin:0;min-height:100vh;background:linear-gradient(180deg,#080c11,#0e1620 65%,#091019);color:var(--text);font-family:system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif}
header,.stats,.notice,.sensorPanel,.actions,.boardWrap,.legend,.map,.logPanel{max-width:1080px;margin-left:auto;margin-right:auto}
header{padding:18px 16px 8px}h1{font-size:23px;margin:0 0 5px}.sub{font-size:13px;color:var(--muted)}
.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:8px;padding:8px 16px}.card,.sensorPanel,.logPanel{background:var(--panel);border:1px solid var(--line);border-radius:11px}.card{padding:9px 10px}.k{font-size:10px;color:var(--muted);letter-spacing:.08em}.v{font-size:15px;font-weight:800;margin-top:3px}.ok{color:var(--high)}.bad{color:var(--danger)}
.notice{margin-top:7px;padding:10px 12px;border:1px solid #594820;background:#211b10;color:#ffda8b;border-radius:10px;font-size:12px}
.sensorPanel{margin-top:12px;padding:14px 16px}.sensorTitle{display:flex;align-items:center;justify-content:space-between;gap:10px;flex-wrap:wrap}.sensorTitle h2{margin:0;font-size:17px}.sensorGrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:8px;margin-top:12px}.sensorValue{background:#0b141b;border:1px solid #254638;border-radius:10px;padding:10px}.sensorValue .big{font-size:24px;font-weight:900;color:var(--sensor);margin-top:4px}.wiring{margin-top:11px;font-family:ui-monospace,Consolas,monospace;font-size:11px;line-height:1.6;color:#b8c7d4;background:#080e13;border-radius:8px;padding:9px;border:1px solid var(--line)}
.actions{display:flex;flex-wrap:wrap;gap:8px;padding:12px 16px 2px}button,a.action{appearance:none;border:1px solid #40566c;background:#182635;color:#edf4fa;padding:9px 12px;border-radius:9px;font-weight:800;font-size:12px;text-decoration:none;cursor:pointer}button:hover,a.action:hover{background:#21364a}button.sensor{border-color:#277762;background:#12382f}button.sensor:hover{background:#185344}button.stop{border-color:#743c3c;background:#351919}
.boardWrap{padding:14px 12px 4px}.board{display:grid;grid-template-columns:minmax(180px,1fr) minmax(190px,235px) minmax(180px,1fr);gap:10px;background:#202a35;border:2px solid #465767;border-radius:24px;padding:14px}.pincol{display:flex;flex-direction:column;gap:5px}.pin{min-height:31px;display:flex;align-items:center;gap:7px;padding:4px 7px;background:var(--panel);border:1px solid var(--line);border-radius:8px}.right .pin{flex-direction:row-reverse;text-align:right}.dot{width:12px;height:12px;flex:0 0 12px;border-radius:50%;background:var(--low)}.pin.high .dot{background:var(--high);box-shadow:0 0 13px var(--high)}.pin.locked .dot{background:var(--locked)}.pin.power .dot{background:var(--power)}.pin.ground .dot{background:#080808;border:1px solid #59616a}.pin.info .dot{background:var(--info)}.pin.sensorpin{border-color:#277762}.pin.sensorpin .dot{background:var(--sensor)}.name{font-size:12px;font-weight:850;white-space:nowrap}.desc{font-size:9px;color:var(--muted);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.middle{min-height:775px;display:flex;flex-direction:column;align-items:center;justify-content:space-between;padding:17px 8px 13px;background:#0d141c;border:1px solid #344251;border-radius:18px}.antenna{width:87%;height:88px;border:2px solid #6f5a36;border-radius:9px;background:repeating-linear-gradient(90deg,transparent 0 12px,#9d7a3d 12px 20px,transparent 20px 31px)}.module{width:88%;height:292px;display:flex;align-items:center;justify-content:center;text-align:center;font-weight:900;color:#171717;background:linear-gradient(145deg,#c8b27c,#82734f);border-radius:9px}.boardname{text-align:center;font-size:12px;color:#aebdca;line-height:1.5}.buttons{font-size:10px;color:#8ea1b2}.usbrow{display:flex;gap:14px}.usb{width:56px;height:26px;border-radius:5px 5px 9px 9px;background:#aab3bb;border:2px solid #68747e}
.legend{display:flex;flex-wrap:wrap;gap:13px;padding:10px 16px 4px;color:var(--muted);font-size:11px}.legend span{display:flex;align-items:center;gap:5px}.mini{width:9px;height:9px;border-radius:50%;display:inline-block}.map{padding:12px 16px 18px}.mapgrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:6px}.mapitem{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:7px 8px;font-size:11px}.logPanel{margin-top:10px;padding:12px 16px 18px}.logPanel h2{font-size:14px;margin:0 0 8px}pre{white-space:pre-wrap;word-break:break-word;background:#05090d;border:1px solid var(--line);border-radius:9px;padding:10px;max-height:320px;overflow:auto;color:#b9c8d6;font-size:11px}.hidden{display:none}
@media(max-width:720px){.board{grid-template-columns:minmax(100px,1fr) 110px minmax(100px,1fr);gap:5px;padding:7px}.middle{min-height:742px;padding:8px 3px}.module{height:235px;font-size:11px}.antenna{height:65px}.pin{padding:3px;gap:4px;min-height:29px}.name{font-size:10px}.desc{display:none}.dot{width:10px;height:10px;flex-basis:10px}.usb{width:40px;height:20px}}
</style>
</head>
<body>
<header><h1>SONIC BONGO · SENSOR 1 DIAGNOSTIC v5</h1><div class="sub">ESP32-S3 N16R8 + TXS0108E + HC-SR04 · GPIO13 TRIG / GPIO14 ECHO</div></header>
<section class="stats">
<div class="card"><div class="k">CHIP</div><div class="v" id="chip">...</div></div><div class="card"><div class="k">FLASH</div><div class="v" id="flash">...</div></div><div class="card"><div class="k">PSRAM</div><div class="v" id="psram">...</div></div><div class="card"><div class="k">RESET</div><div class="v" id="resetCause">...</div></div><div class="card"><div class="k">HEAP</div><div class="v" id="heap">...</div></div><div class="card"><div class="k">WEB</div><div class="v" id="link">CONECTANDO</div></div>
</section>
<div class="notice">En esta versión <b>GPIO13 sí es una salida</b>, exclusivamente para el pulso TRIG del Sensor 1. GPIO14 permanece como entrada ECHO. No conectes 5 V directamente a GPIO14: el ECHO debe seguir pasando por el TXS0108E.</div>
<section class="sensorPanel">
<div class="sensorTitle"><h2>HC-SR04 · Sensor 1</h2><div id="sensorStatus" class="v">SIN PRUEBA</div></div>
<div class="sensorGrid">
<div class="sensorValue"><div class="k">DISTANCIA</div><div class="big" id="distance">-- cm</div></div>
<div class="sensorValue"><div class="k">ECHO</div><div class="big" id="echoUs">-- µs</div></div>
<div class="sensorValue"><div class="k">VÁLIDAS / TOTAL</div><div class="big" id="validCount">--</div></div>
<div class="sensorValue"><div class="k">RANGO ÚLTIMO LOTE</div><div class="big" id="range">--</div></div>
</div>
<div class="actions" style="padding-left:0;padding-right:0"><button class="sensor" onclick="testSensor(1)">PROBAR 1 MEDICIÓN</button><button class="sensor" onclick="testSensor(5)">PROBAR 5 MEDICIONES</button><button class="sensor" id="autoBtn" onclick="toggleAuto()">AUTO: INICIAR</button><button class="stop" onclick="stopAuto()">DETENER</button></div>
<div class="wiring">GPIO13 → A1 → B1 → TRIG<br>GPIO14 ← A2 ← B2 ← ECHO<br>HC-SR04 VCC → 5V · GND → GND común<br>TXS: VCCA 3.3V · VCCB 5V · OE 3.3V · GND común</div>
</section>
<div class="actions"><a class="action" href="/api/export">EXPORTAR DIAGNÓSTICO (.TXT)</a><button onclick="toggleLog()">VER / OCULTAR LOG</button><button onclick="loadLog()">ACTUALIZAR LOG</button></div>
<div class="logPanel hidden" id="logPanel"><h2>Log circular</h2><pre id="logText">Cargando...</pre></div>
<div class="boardWrap"><div class="board"><div class="pincol left" id="leftPins"></div><div class="middle"><div class="antenna"></div><div class="module">ESP32-S3<br>WROOM-1<br><br>N16R8</div><div class="boardname">SENSOR 1<br>GPIO13 / GPIO14</div><div class="buttons">BOOT &nbsp;&nbsp;&nbsp; RESET</div><div class="usbrow"><div class="usb"></div><div class="usb"></div></div></div><div class="pincol right" id="rightPins"></div></div></div>
<div class="legend"><span><i class="mini" style="background:#68f0d1"></i>SENSOR 1</span><span><i class="mini" style="background:#45ef89"></i>HIGH</span><span><i class="mini" style="background:#3b4b5a"></i>LOW</span><span><i class="mini" style="background:#ffbf47"></i>RESERVADO</span><span><i class="mini" style="background:#58d8ff"></i>ALIMENTACIÓN</span></div>
<section class="map"><div class="mapgrid"><div class="mapitem"><b>GPIO13</b> TRIG S1 · OUTPUT</div><div class="mapitem"><b>GPIO14</b> ECHO S1 · INPUT</div><div class="mapitem"><b>A1/B1</b> TRIG por TXS</div><div class="mapitem"><b>A2/B2</b> ECHO por TXS</div></div></section>
<script>
const LEFT=[["3V3",null,"power","3,3 V"],["3V3",null,"power","3,3 V"],["RST",null,"info","EN / RESET"],["GPIO4",4,"gpio","monitor"],["GPIO5",5,"gpio","monitor"],["GPIO6",6,"gpio","monitor"],["GPIO7",7,"gpio","monitor"],["GPIO15",15,"gpio","TRIG S2 futuro"],["GPIO16",16,"gpio","ECHO S2 futuro"],["GPIO17",17,"gpio","TRIG S3 futuro"],["GPIO18",18,"gpio","ECHO S3 futuro"],["GPIO8",8,"gpio","OLED SDA"],["GPIO3",3,"locked","strapping / JTAG"],["GPIO46",46,"locked","strapping"],["GPIO9",9,"gpio","OLED SCL"],["GPIO10",10,"gpio","Touch grande"],["GPIO11",11,"gpio","Touch 1"],["GPIO12",12,"gpio","Touch 2"],["GPIO13",13,"sensorpin","TRIG S1 · OUTPUT"],["GPIO14",14,"sensorpin","ECHO S1 · INPUT"],["5V",null,"power","5 V / VIN"],["GND",null,"ground","masa"]];
const RIGHT=[["GND",null,"ground","masa"],["TX / GPIO43",43,"locked","UART0"],["RX / GPIO44",44,"locked","UART0"],["GPIO1",1,"gpio","ADC flotante si libre"],["GPIO2",2,"gpio","monitor"],["GPIO42",42,"gpio","monitor"],["GPIO41",41,"gpio","monitor"],["GPIO40",40,"gpio","monitor"],["GPIO39",39,"gpio","monitor"],["GPIO38",38,"locked","RGB posible"],["GPIO37",37,"locked","Octal PSRAM"],["GPIO36",36,"locked","Octal PSRAM"],["GPIO35",35,"locked","Octal PSRAM"],["GPIO0",0,"locked","BOOT"],["GPIO45",45,"locked","strapping"],["GPIO48",48,"locked","reservado"],["GPIO47",47,"gpio","monitor"],["GPIO21",21,"gpio","ARM / MUTE"],["GPIO20",20,"locked","USB D+"],["GPIO19",19,"locked","USB D-"],["GND",null,"ground","masa"],["GND",null,"ground","masa"]];
function makePin(s,side,i){const [label,gpio,type,desc]=s;const e=document.createElement('div');e.className='pin '+(type==='gpio'?'':type);e.dataset.gpio=gpio===null?'':String(gpio);e.innerHTML='<span class="dot"></span><div style="min-width:0"><div class="name">'+label+'</div><div class="desc">'+desc+'</div></div>';return e}
LEFT.forEach((p,i)=>leftPins.appendChild(makePin(p,'L',i)));RIGHT.forEach((p,i)=>rightPins.appendChild(makePin(p,'R',i)));
function applyStates(states){document.querySelectorAll('.pin[data-gpio]').forEach(el=>{const p=el.dataset.gpio;if(p!==''&&Object.prototype.hasOwnProperty.call(states,p)&&!el.classList.contains('sensorpin'))el.classList.toggle('high',states[p]===1)})}
async function refresh(){try{const r=await fetch('/api/status',{cache:'no-store'});const d=await r.json();chip.textContent=d.chip;flash.textContent=d.flashMB+' MB';psram.textContent=d.psramMB+' MB';resetCause.textContent=d.resetCause;heap.textContent=Math.round(d.freeHeap/1024)+' KB';link.textContent='ONLINE';link.className='v ok';flash.className='v '+(d.flashMB===16?'ok':'bad');psram.className='v '+(d.psramMB===8?'ok':'bad');applyStates(d.pins)}catch(e){link.textContent='SIN RESPUESTA';link.className='v bad'}}
let autoTimer=null;
async function testSensor(count){sensorStatus.textContent='MIDIENDO...';sensorStatus.className='v';try{const r=await fetch('/api/sensor1?count='+count,{cache:'no-store'});const d=await r.json();validCount.textContent=d.valid+'/'+d.total;if(d.valid>0){distance.textContent=d.avgCm.toFixed(1)+' cm';echoUs.textContent=Math.round(d.avgEchoUs)+' µs';range.textContent=d.minCm.toFixed(1)+'–'+d.maxCm.toFixed(1)+' cm';sensorStatus.textContent='OK';sensorStatus.className='v ok'}else{distance.textContent='-- cm';echoUs.textContent='-- µs';range.textContent='TIMEOUT';sensorStatus.textContent='SIN ECHO';sensorStatus.className='v bad'}if(!logPanel.classList.contains('hidden'))loadLog()}catch(e){sensorStatus.textContent='ERROR WEB';sensorStatus.className='v bad'}}
function toggleAuto(){if(autoTimer){stopAuto();return}testSensor(1);autoTimer=setInterval(()=>testSensor(1),800);autoBtn.textContent='AUTO: ACTIVO'}
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
  json.reserve(1300);
  json += "{";
  json += "\"fw\":\"" + String(FW_VERSION) + "\",";
  json += "\"profile\":\"ESP32-S3-WROOM-1-N16R8\",";
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
  json += "\"pins\":{";
  json += "\"1\":" + String(digitalRead(ANALOG_PIN) ? 1 : 0);
  for (size_t i = 0; i < MONITOR_PIN_COUNT; i++) {
    json += ",\"" + String(MONITOR_PINS[i]) + "\":" + String(digitalRead(MONITOR_PINS[i]) ? 1 : 0);
  }
  json += ",\"13\":" + String(digitalRead(S1_TRIG_PIN) ? 1 : 0);
  json += ",\"14\":" + String(digitalRead(S1_ECHO_PIN) ? 1 : 0);
  json += "},";
  json += "\"sensor1\":{";
  json += "\"hasSample\":" + String(lastSensor.ms ? "true" : "false") + ",";
  json += "\"ok\":" + String(lastSensor.ok ? "true" : "false") + ",";
  json += "\"durationUs\":" + String(lastSensor.durationUs) + ",";
  json += "\"distanceCm\":" + String(lastSensor.distanceCm, 2) + ",";
  json += "\"sampleMs\":" + String(lastSensor.ms);
  json += "}}";
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
  json.reserve(300);
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

void handleLog() {
  server.send(200, "text/plain; charset=utf-8", runtimeLog);
}

void handleExport() {
  uint16_t adcMin, adcMax;
  uint32_t adcAvg;
  readAdcStats(64, adcMin, adcMax, adcAvg);

  String out;
  out.reserve(14000);
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
  out += "AP MAC: " + WiFi.softAPmacAddress() + "\n";
  out += "Connected stations: " + String(WiFi.softAPgetStationNum()) + "\n\n";

  out += "SENSOR 1 WIRING\n";
  out += "GPIO13 -> TXS A1/B1 -> HC-SR04 TRIG\n";
  out += "GPIO14 <- TXS A2/B2 <- HC-SR04 ECHO\n";
  out += "Timeout us: " + String(S1_TIMEOUT_US) + "\n";
  out += "Last sample: ";
  if (lastSensor.ms == 0) out += "NO SAMPLE\n";
  else if (lastSensor.ok) out += "OK, echo=" + String(lastSensor.durationUs) + " us, distance=" + String(lastSensor.distanceCm, 2) + " cm\n";
  else out += "TIMEOUT, no echo\n";

  out += "\nSENSOR 1 HISTORY (oldest -> newest)\n";
  if (sensorHistoryCount == 0) {
    out += "No samples yet.\n";
  } else {
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
  out += "GPIO1 ADC current = " + String(analogRead(ANALOG_PIN)) + "\n";
  out += "GPIO1 ADC 64 samples min/avg/max = " + String(adcMin) + "/" + String(adcAvg) + "/" + String(adcMax) + "\n";
  for (size_t i = 0; i < MONITOR_PIN_COUNT; i++) {
    out += "GPIO" + String(MONITOR_PINS[i]) + " = " + String(digitalRead(MONITOR_PINS[i]) ? "HIGH" : "LOW") + "\n";
  }

  out += "\nRUNTIME LOG\n";
  out += runtimeLog;

  server.sendHeader("Content-Disposition", "attachment; filename=SONIC_BONGO_N16R8_SENSOR1_DIAG_v5.txt");
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

  for (size_t i = 0; i < MONITOR_PIN_COUNT; i++) {
    pinMode(MONITOR_PINS[i], INPUT);
  }

  pinMode(S1_ECHO_PIN, INPUT);
  pinMode(S1_TRIG_PIN, OUTPUT);
  digitalWrite(S1_TRIG_PIN, LOW);

  addLog("SONIC BONGO ESP32-S3 N16R8 DIAG v5 boot");
  addLog("Reset cause: " + resetReasonText(esp_reset_reason()));
  addLog("Sensor 1: GPIO13 TRIG output LOW; GPIO14 ECHO input");

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  bool ok = WiFi.softAP(AP_SSID);
  if (ok) {
    addLog("AP started: " + String(AP_SSID) + " IP=" + WiFi.softAPIP().toString());
  } else {
    addLog("ERROR: failed to start Access Point");
  }

  addLog("Chip=" + String(ESP.getChipModel()) + " rev=" + String(ESP.getChipRevision()));
  addLog("Flash=" + String(ESP.getFlashChipSize() / (1024 * 1024)) + "MB PSRAM=" + String(ESP.getPsramSize() / (1024 * 1024)) + "MB");

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/sensor1", HTTP_GET, handleSensor1);
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
