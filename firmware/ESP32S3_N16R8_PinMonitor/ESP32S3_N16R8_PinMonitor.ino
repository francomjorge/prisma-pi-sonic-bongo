/*
  SONIC BONGO / PRISMA
  ESP32-S3 N16R8 Pin Monitor v4

  Target:
  - ESP32-S3-WROOM-1 N16R8
  - ESP32-S3-DevKitC-1 compatible 44-pin layout
  - 16 MB Flash + 8 MB Octal PSRAM

  SAFE START:
  - First test: power ONLY from USB.
  - Do not connect external 5 V / 3.3 V supplies yet.
  - Monitored GPIOs are never driven HIGH/LOW by this firmware.
  - GPIO1 ADC is INPUT without pull resistor; if unconnected it WILL float.

  Wi-Fi AP:
    SSID: PRISMA-S3-DIAG
    Password: OPEN
    Web: http://192.168.4.1

  Diagnostic endpoints:
    /api/status
    /api/log
    /api/export   -> downloadable TXT report
*/

#include <WiFi.h>
#include <WebServer.h>
#include <esp_system.h>

WebServer server(80);

static const char* FW_VERSION = "4.0";
static const char* AP_SSID = "PRISMA-S3-DIAG";
static const uint8_t ANALOG_PIN = 1;
static const uint32_t LOG_SAMPLE_MS = 10000;
static const size_t MAX_LOG_CHARS = 7000;

const uint8_t SAFE_DIGITAL_PINS[] = {
  2,
  4, 5, 6, 7,
  8, 9, 10, 11, 12, 13, 14,
  15, 16, 17, 18,
  21,
  39, 40, 41, 42,
  47
};

const size_t SAFE_DIGITAL_PIN_COUNT =
  sizeof(SAFE_DIGITAL_PINS) / sizeof(SAFE_DIGITAL_PINS[0]);

String runtimeLog;
uint32_t lastPeriodicLog = 0;

void addLog(const String& message) {
  String line;
  line.reserve(message.length() + 24);
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

void readAdcStats(uint16_t samples, uint16_t &minV, uint16_t &maxV, uint32_t &avgV) {
  minV = 4095;
  maxV = 0;
  uint32_t sum = 0;

  for (uint16_t i = 0; i < samples; i++) {
    uint16_t v = analogRead(ANALOG_PIN);
    if (v < minV) minV = v;
    if (v > maxV) maxV = v;
    sum += v;
    delayMicroseconds(150);
  }

  avgV = samples ? (sum / samples) : 0;
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

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="es">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SONIC BONGO · N16R8 Diagnostic v4</title>
<style>
:root{
  --bg:#080c11;--panel:#111923;--line:#2b3948;--text:#edf4fa;--muted:#8fa2b5;
  --high:#45ef89;--low:#3b4b5a;--locked:#ffbf47;--power:#58d8ff;--info:#b28cff;--danger:#ff6262;
}
*{box-sizing:border-box}
body{margin:0;min-height:100vh;background:linear-gradient(180deg,#080c11,#0e1620 65%,#091019);color:var(--text);font-family:system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif}
header,.stats,.notice,.actions,.boardWrap,.legend,.projectMap,.logPanel{max-width:1080px;margin-left:auto;margin-right:auto}
header{padding:18px 16px 8px} h1{font-size:23px;margin:0 0 5px}.sub{font-size:13px;color:var(--muted)}
.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(135px,1fr));gap:8px;padding:8px 16px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:11px;padding:9px 10px}.card .k{font-size:10px;color:var(--muted);letter-spacing:.08em}.card .v{font-size:15px;font-weight:800;margin-top:3px}
.ok{color:var(--high)}.bad{color:var(--danger)}.muted{color:var(--muted)}
.notice{margin-top:7px;padding:10px 12px;border:1px solid #594820;background:#211b10;color:#ffda8b;border-radius:10px;font-size:12px}
.actions{display:flex;flex-wrap:wrap;gap:8px;padding:12px 16px 2px}
button,a.action{appearance:none;border:1px solid #40566c;background:#182635;color:#edf4fa;padding:9px 12px;border-radius:9px;font-weight:800;font-size:12px;text-decoration:none;cursor:pointer}button:hover,a.action:hover{background:#21364a}
.boardWrap{padding:14px 12px 4px}.board{display:grid;grid-template-columns:minmax(180px,1fr) minmax(190px,235px) minmax(180px,1fr);gap:10px;background:#202a35;border:2px solid #465767;border-radius:24px;padding:14px;box-shadow:0 18px 45px rgba(0,0,0,.38)}
.pincol{display:flex;flex-direction:column;gap:5px}.pin{min-height:31px;display:flex;align-items:center;gap:7px;padding:4px 7px;background:var(--panel);border:1px solid var(--line);border-radius:8px}.right .pin{flex-direction:row-reverse;text-align:right}
.dot{width:12px;height:12px;flex:0 0 12px;border-radius:50%;background:var(--low);box-shadow:0 0 0 2px rgba(255,255,255,.04)}.pin.high .dot{background:var(--high);box-shadow:0 0 13px var(--high)}.pin.locked .dot{background:var(--locked)}.pin.power .dot{background:var(--power)}.pin.ground .dot{background:#080808;border:1px solid #59616a}.pin.info .dot{background:var(--info)}
.name{font-size:12px;font-weight:850;white-space:nowrap}.desc{font-size:9px;color:var(--muted);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.middle{min-height:775px;display:flex;flex-direction:column;align-items:center;justify-content:space-between;padding:17px 8px 13px;background:#0d141c;border:1px solid #344251;border-radius:18px}.antenna{width:87%;height:88px;border:2px solid #6f5a36;border-radius:9px;background:repeating-linear-gradient(90deg,transparent 0 12px,#9d7a3d 12px 20px,transparent 20px 31px)}.module{width:88%;height:292px;display:flex;align-items:center;justify-content:center;text-align:center;font-weight:900;color:#171717;background:linear-gradient(145deg,#c8b27c,#82734f);border-radius:9px;box-shadow:inset 0 0 0 2px #665a40}.boardname{text-align:center;font-size:12px;color:#aebdca;line-height:1.5}.buttons{font-size:10px;color:#8ea1b2;letter-spacing:.08em}.usbrow{display:flex;gap:14px}.usb{width:56px;height:26px;border-radius:5px 5px 9px 9px;background:#aab3bb;border:2px solid #68747e}
.legend{display:flex;flex-wrap:wrap;gap:13px;padding:10px 16px 4px;color:var(--muted);font-size:11px}.legend span{display:flex;align-items:center;gap:5px}.mini{width:9px;height:9px;border-radius:50%;display:inline-block}
.projectMap,.logPanel{padding:12px 16px 25px}.projectMap h2,.logPanel h2{font-size:14px;margin:0 0 8px}.mapgrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(175px,1fr));gap:6px}.mapitem{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:7px 8px;font-size:11px}.mapitem b{display:inline-block;min-width:57px}
pre{white-space:pre-wrap;word-break:break-word;background:#05090d;border:1px solid var(--line);border-radius:9px;padding:10px;max-height:310px;overflow:auto;color:#b9c8d6;font-size:11px}.hidden{display:none}
@media(max-width:720px){.board{grid-template-columns:minmax(100px,1fr) 110px minmax(100px,1fr);gap:5px;padding:7px}.middle{min-height:742px;padding:8px 3px}.module{height:235px;font-size:11px}.antenna{height:65px}.pin{padding:3px;gap:4px;min-height:29px}.name{font-size:10px}.desc{display:none}.dot{width:10px;height:10px;flex-basis:10px}.usb{width:40px;height:20px}}
</style>
</head>
<body>
<header><h1>SONIC BONGO · N16R8 PIN DIAGNOSTIC v4</h1><div class="sub">ESP32-S3-WROOM-1 N16R8 · 44 pines · lectura segura + exportación de diagnóstico</div></header>
<section class="stats">
  <div class="card"><div class="k">CHIP</div><div class="v" id="chip">...</div></div>
  <div class="card"><div class="k">REVISIÓN</div><div class="v" id="rev">...</div></div>
  <div class="card"><div class="k">FLASH</div><div class="v" id="flash">...</div></div>
  <div class="card"><div class="k">PSRAM</div><div class="v" id="psram">...</div></div>
  <div class="card"><div class="k">GPIO1 ADC</div><div class="v" id="adc1">...</div></div>
  <div class="card"><div class="k">RESET CAUSE</div><div class="v" id="resetCause">...</div></div>
  <div class="card"><div class="k">HEAP LIBRE</div><div class="v" id="heap">...</div></div>
  <div class="card"><div class="k">WEB</div><div class="v" id="link">CONECTANDO</div></div>
</section>
<div class="notice">RST/EN en violeta es sólo un pin especial, <b>no una alarma</b>. GPIO1 ADC sin nada conectado queda flotante y puede variar continuamente; eso por sí solo no indica una falla.</div>
<div class="actions"><a class="action" href="/api/export">EXPORTAR DIAGNÓSTICO (.TXT)</a><button onclick="toggleLog()">VER / OCULTAR LOG</button><button onclick="loadLog()">ACTUALIZAR LOG</button></div>
<div class="logPanel hidden" id="logPanel"><h2>Log circular en RAM</h2><pre id="logText">Cargando...</pre></div>
<div class="boardWrap"><div class="board"><div class="pincol left" id="leftPins"></div><div class="middle"><div class="antenna"></div><div class="module">ESP32-S3<br>WROOM-1<br><br>N16R8</div><div class="boardname">16 MB FLASH<br>8 MB OCTAL PSRAM</div><div class="buttons">BOOT &nbsp;&nbsp;&nbsp; RESET</div><div class="usbrow"><div class="usb"></div><div class="usb"></div></div></div><div class="pincol right" id="rightPins"></div></div></div>
<div class="legend"><span><i class="mini" style="background:#45ef89"></i>HIGH</span><span><i class="mini" style="background:#3b4b5a"></i>LOW</span><span><i class="mini" style="background:#ffbf47"></i>RESERVADO</span><span><i class="mini" style="background:#b28cff"></i>PIN ESPECIAL</span><span><i class="mini" style="background:#58d8ff"></i>ALIMENTACIÓN</span></div>
<section class="projectMap"><h2>Mapa funcional previsto</h2><div class="mapgrid">
  <div class="mapitem"><b>GPIO4</b>K1</div><div class="mapitem"><b>GPIO5</b>K2</div><div class="mapitem"><b>GPIO6</b>K3</div><div class="mapitem"><b>GPIO7</b>K4</div>
  <div class="mapitem"><b>GPIO8</b>OLED SDA</div><div class="mapitem"><b>GPIO9</b>OLED SCL</div><div class="mapitem"><b>GPIO10</b>Touch grande</div><div class="mapitem"><b>GPIO11</b>Touch 1</div><div class="mapitem"><b>GPIO12</b>Touch 2</div>
  <div class="mapitem"><b>GPIO13</b>TRIG S1</div><div class="mapitem"><b>GPIO14</b>ECHO S1 / shifter</div><div class="mapitem"><b>GPIO15</b>TRIG S2</div><div class="mapitem"><b>GPIO16</b>ECHO S2 / shifter</div><div class="mapitem"><b>GPIO17</b>TRIG S3</div><div class="mapitem"><b>GPIO18</b>ECHO S3 / shifter</div>
  <div class="mapitem"><b>GPIO21</b>ARM / MUTE</div><div class="mapitem"><b>GPIO1</b>Pedal EXP / ADC</div>
</div></section>
<script>
const LEFT=[["3V3",null,"power","3,3 V"],["3V3",null,"power","3,3 V"],["RST",null,"info","EN / RESET · no es alarma"],["GPIO4",4,"gpio","K1"],["GPIO5",5,"gpio","K2"],["GPIO6",6,"gpio","K3"],["GPIO7",7,"gpio","K4"],["GPIO15",15,"gpio","TRIG S2"],["GPIO16",16,"gpio","ECHO S2 / shifter"],["GPIO17",17,"gpio","TRIG S3"],["GPIO18",18,"gpio","ECHO S3 / shifter"],["GPIO8",8,"gpio","OLED SDA"],["GPIO3",3,"locked","strapping / JTAG"],["GPIO46",46,"locked","strapping"],["GPIO9",9,"gpio","OLED SCL"],["GPIO10",10,"gpio","Touch grande"],["GPIO11",11,"gpio","Touch 1"],["GPIO12",12,"gpio","Touch 2"],["GPIO13",13,"gpio","TRIG S1"],["GPIO14",14,"gpio","ECHO S1 / shifter"],["5V",null,"power","5 V / VIN"],["GND",null,"ground","masa"]];
const RIGHT=[["GND",null,"ground","masa"],["TX / GPIO43",43,"locked","UART0 / bridge"],["RX / GPIO44",44,"locked","UART0 / bridge"],["GPIO1",1,"gpio","Pedal EXP / ADC · flotante si libre"],["GPIO2",2,"gpio","monitor"],["GPIO42",42,"gpio","monitor"],["GPIO41",41,"gpio","monitor"],["GPIO40",40,"gpio","monitor"],["GPIO39",39,"gpio","monitor"],["GPIO38",38,"locked","RGB LED posible"],["GPIO37",37,"locked","N16R8: Octal PSRAM"],["GPIO36",36,"locked","N16R8: Octal PSRAM"],["GPIO35",35,"locked","N16R8: Octal PSRAM"],["GPIO0",0,"locked","BOOT / strapping"],["GPIO45",45,"locked","strapping"],["GPIO48",48,"locked","RGB posible / reservado"],["GPIO47",47,"gpio","monitor"],["GPIO21",21,"gpio","ARM / MUTE"],["GPIO20",20,"locked","USB D+"],["GPIO19",19,"locked","USB D-"],["GND",null,"ground","masa"],["GND",null,"ground","masa"]];
function makePin(spec,side,index){const [label,gpio,type,desc]=spec;const el=document.createElement("div");el.className="pin "+(type==="gpio"?"":type);el.dataset.gpio=gpio===null?"":String(gpio);el.id=`${side}-${index}`;const dot=document.createElement("span");dot.className="dot";const text=document.createElement("div");text.style.minWidth="0";text.innerHTML=`<div class="name">${label}</div><div class="desc">${desc}</div>`;el.appendChild(dot);el.appendChild(text);return el}
LEFT.forEach((p,i)=>document.getElementById("leftPins").appendChild(makePin(p,"L",i)));RIGHT.forEach((p,i)=>document.getElementById("rightPins").appendChild(makePin(p,"R",i)));
function applyStates(states){document.querySelectorAll(".pin[data-gpio]").forEach(el=>{const p=el.dataset.gpio;if(p==="")return;if(Object.prototype.hasOwnProperty.call(states,p))el.classList.toggle("high",states[p]===1)})}
async function refresh(){const link=document.getElementById("link");try{const r=await fetch("/api/status",{cache:"no-store"});if(!r.ok)throw new Error("HTTP "+r.status);const d=await r.json();chip.textContent=d.chip;rev.textContent=d.revision;flash.textContent=d.flashMB+" MB";psram.textContent=d.psramMB+" MB";adc1.textContent=d.adc1;resetCause.textContent=d.resetReason;heap.textContent=Math.round(d.freeHeap/1024)+" KB";flash.className="v "+(d.flashMB===16?"ok":"bad");psram.className="v "+(d.psramMB===8?"ok":"bad");link.textContent="ONLINE";link.className="v ok";applyStates(d.pins)}catch(e){link.textContent="SIN RESPUESTA";link.className="v bad"}}
async function loadLog(){try{const r=await fetch("/api/log",{cache:"no-store"});document.getElementById("logText").textContent=await r.text()}catch(e){document.getElementById("logText").textContent=String(e)}}
function toggleLog(){const p=document.getElementById("logPanel");p.classList.toggle("hidden");if(!p.classList.contains("hidden"))loadLog()}
refresh();setInterval(refresh,250);
</script>
</body></html>
)HTML";

String buildStatusJson() {
  String json;
  json.reserve(1200);

  uint16_t adcNow = analogRead(ANALOG_PIN);
  esp_reset_reason_t rr = esp_reset_reason();

  json += "{";
  json += "\"firmware\":\"" + String(FW_VERSION) + "\",";
  json += "\"profile\":\"ESP32-S3-WROOM-1-N16R8\",";
  json += "\"chip\":\"" + String(ESP.getChipModel()) + "\",";
  json += "\"revision\":" + String(ESP.getChipRevision()) + ",";
  json += "\"flashMB\":" + String((uint32_t)(ESP.getFlashChipSize() / (1024 * 1024))) + ",";
  json += "\"psramMB\":" + String((uint32_t)(ESP.getPsramSize() / (1024 * 1024))) + ",";
  json += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
  json += "\"minFreeHeap\":" + String(ESP.getMinFreeHeap()) + ",";
  json += "\"resetReason\":\"" + resetReasonText(rr) + "\",";
  json += "\"adc1\":" + String(adcNow) + ",";
  json += "\"uptimeMs\":" + String(millis()) + ",";
  json += "\"stations\":" + String(WiFi.softAPgetStationNum()) + ",";
  json += "\"pins\":{";

  json += "\"1\":" + String(digitalRead(ANALOG_PIN) ? 1 : 0);
  for (size_t i = 0; i < SAFE_DIGITAL_PIN_COUNT; i++) {
    int pin = SAFE_DIGITAL_PINS[i];
    json += ",\"" + String(pin) + "\":" + String(digitalRead(pin) ? 1 : 0);
  }
  json += "}}";
  return json;
}

String buildDiagnosticReport() {
  uint16_t adcMin, adcMax;
  uint32_t adcAvg;
  readAdcStats(64, adcMin, adcMax, adcAvg);

  String report;
  report.reserve(10000);

  report += "SONIC BONGO - ESP32-S3 N16R8 DIAGNOSTIC REPORT\n";
  report += "================================================\n";
  report += "Firmware: v" + String(FW_VERSION) + "\n";
  report += "Profile: ESP32-S3-WROOM-1-N16R8\n";
  report += "Chip: " + String(ESP.getChipModel()) + "\n";
  report += "Revision: " + String(ESP.getChipRevision()) + "\n";
  report += "CPU MHz: " + String(ESP.getCpuFreqMHz()) + "\n";
  report += "Flash MB: " + String((uint32_t)(ESP.getFlashChipSize() / (1024 * 1024))) + "\n";
  report += "PSRAM MB: " + String((uint32_t)(ESP.getPsramSize() / (1024 * 1024))) + "\n";
  report += "Free heap: " + String(ESP.getFreeHeap()) + " bytes\n";
  report += "Min free heap: " + String(ESP.getMinFreeHeap()) + " bytes\n";
  report += "Reset reason: " + resetReasonText(esp_reset_reason()) + "\n";
  report += "Uptime: " + String(millis()) + " ms\n\n";

  report += "WIFI AP\n-------\n";
  report += "SSID: " + String(AP_SSID) + "\n";
  report += "IP: " + WiFi.softAPIP().toString() + "\n";
  report += "MAC: " + WiFi.softAPmacAddress() + "\n";
  report += "Connected stations: " + String(WiFi.softAPgetStationNum()) + "\n\n";

  report += "GPIO1 ADC (64 samples)\n----------------------\n";
  report += "MIN: " + String(adcMin) + "\n";
  report += "MAX: " + String(adcMax) + "\n";
  report += "AVG: " + String(adcAvg) + "\n";
  report += "SPAN: " + String((uint16_t)(adcMax - adcMin)) + "\n";
  report += "NOTE: if GPIO1 is physically unconnected, changing ADC values are expected because the input is floating.\n\n";

  report += "PIN STATES\n----------\n";
  report += "GPIO1=" + String(digitalRead(ANALOG_PIN) ? 1 : 0) + " (ADC / floating if unconnected)\n";
  for (size_t i = 0; i < SAFE_DIGITAL_PIN_COUNT; i++) {
    int pin = SAFE_DIGITAL_PINS[i];
    report += "GPIO" + String(pin) + "=" + String(digitalRead(pin) ? 1 : 0) + "\n";
  }

  report += "\nRESERVED / NOT TOUCHED\n----------------------\n";
  report += "GPIO0 BOOT/strap\nGPIO3 strap/JTAG\nGPIO19 USB D-\nGPIO20 USB D+\nGPIO35/36/37 N16R8 Octal PSRAM\nGPIO38 possible RGB LED\nGPIO43/44 UART0\nGPIO45/46 strapping\nGPIO48 reserved/RGB possible\nRST/EN special pin; UI color is informational, not an error state.\n\n";

  report += "RUNTIME LOG\n-----------\n";
  report += runtimeLog;
  return report;
}

void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleStatus() {
  server.send(200, "application/json", buildStatusJson());
}

void handleLog() {
  server.send(200, "text/plain; charset=utf-8", runtimeLog);
}

void handleExport() {
  addLog("Diagnostic report exported");
  String report = buildDiagnosticReport();
  server.sendHeader("Content-Disposition", "attachment; filename=SONIC_BONGO_N16R8_DIAG.txt");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain; charset=utf-8", report);
}

void handleNotFound() {
  server.send(404, "text/plain; charset=utf-8", "404");
}

void setup() {
  Serial.begin(115200);
  delay(400);
  runtimeLog.reserve(MAX_LOG_CHARS + 256);

  addLog("========================================");
  addLog("SONIC BONGO ESP32-S3 N16R8 DIAG v4");
  addLog("========================================");
  addLog("Reset reason: " + resetReasonText(esp_reset_reason()));

  pinMode(ANALOG_PIN, INPUT);
  analogReadResolution(12);

  for (size_t i = 0; i < SAFE_DIGITAL_PIN_COUNT; i++) {
    pinMode(SAFE_DIGITAL_PINS[i], INPUT_PULLDOWN);
  }
  addLog("Safe GPIO monitor configured as INPUT_PULLDOWN; GPIO1 as INPUT ADC");

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  bool ok = WiFi.softAP(AP_SSID);
  if (!ok) {
    addLog("ERROR: WiFi AP could not start");
  } else {
    addLog("WiFi AP started: " + String(AP_SSID));
    addLog("AP IP: " + WiFi.softAPIP().toString());
  }

  addLog("Chip: " + String(ESP.getChipModel()));
  addLog("Revision: " + String(ESP.getChipRevision()));
  addLog("Flash MB: " + String(ESP.getFlashChipSize() / (1024 * 1024)));
  addLog("PSRAM MB: " + String(ESP.getPsramSize() / (1024 * 1024)));
  addLog("Free heap: " + String(ESP.getFreeHeap()));

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/log", HTTP_GET, handleLog);
  server.on("/api/export", HTTP_GET, handleExport);
  server.onNotFound(handleNotFound);
  server.begin();

  addLog("Web server started");
}

void loop() {
  server.handleClient();

  uint32_t now = millis();
  if (now - lastPeriodicLog >= LOG_SAMPLE_MS) {
    lastPeriodicLog = now;
    uint16_t adcMin, adcMax;
    uint32_t adcAvg;
    readAdcStats(16, adcMin, adcMax, adcAvg);
    addLog("Health: heap=" + String(ESP.getFreeHeap()) +
           " minHeap=" + String(ESP.getMinFreeHeap()) +
           " stations=" + String(WiFi.softAPgetStationNum()) +
           " adc1[min/avg/max]=" + String(adcMin) + "/" + String(adcAvg) + "/" + String(adcMax));
  }

  delay(2);
}
