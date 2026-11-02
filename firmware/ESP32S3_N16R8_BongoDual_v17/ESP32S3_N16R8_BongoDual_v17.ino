/*
  SONIC BONGO / PRISMA
  ESP32-S3 N16R8 - v17 DATASET MATRIX

  This sketch inherits the proven v16 acquisition/function engine at build time.
  v17 does NOT change the musical detector. It adds a structured recording layer
  for future adaptive/ML work:

    SAMPLES: continuous per-point rows for sequence learning.
    EVENTS:  one feature row per automatic gesture candidate.
    CONFIGS: immutable snapshots keyed by configRevision.

  Automatic candidates and future human labels are deliberately separate.
*/

#include <esp_heap_caps.h>
#include <esp_system.h>

#define setup setup_v16
#define loop loop_v16
#include "base_v16.h"
#undef setup
#undef loop

static const char* DATA_SCHEMA_VERSION = "sonic-bongo-dataset-v1";

// ---------- Dataset schema ----------
// Designed to map cleanly later to X[time,features] for sequence models.
struct __attribute__((packed)) TrainingSample {
  uint32_t tMs;
  uint32_t pointSeq;
  uint32_t configRevision;

  float s1Raw;
  float s1Fast;
  float s1Stable;
  float s1Velocity;
  float s2Raw;
  float s2Fast;
  float s2Stable;
  float s2Velocity;

  float yCandidate;
  float yPoint;
  float yStable;
  float velocity;
  float acceleration;
  float adaptiveBaseline;
  float displacement;
  float coherence;
  float jitter;
  float candidateScore;

  uint16_t ageS1Ms;
  uint16_t ageS2Ms;
  uint8_t sourceMask;
  uint8_t phaseCode;
  uint8_t flags;
  int8_t humanLabel;
};

struct __attribute__((packed)) GestureEventRow {
  uint32_t eventSeq;
  uint32_t tMs;
  uint32_t configRevision;
  uint32_t pointSeq;
  uint32_t descentDurationMs;

  float yAtEvent;
  float adaptiveBaseline;
  float displacement;
  float amplitude;
  float peakDownSpeed;
  float velocityAtEvent;
  float accelerationAtEvent;
  float periodMs;
  float rollHz;
  float energy;
  float coherence;
  float jitter;

  uint8_t sourceMask;
  uint8_t rollCount;
  uint8_t midi;
  uint8_t autoClass;      // 1=candidate, 2=roll candidate
  int8_t humanLabel;      // -1=unlabelled, 0=reject, 1=confirmed hit, 2=roll
  uint8_t reserved[3];
};

struct __attribute__((packed)) ConfigSnapshot {
  uint32_t revision;
  uint32_t tMs;
  float baselineCm;
  float maxRangeCm;
  float hitPlaneCm;
  float coneHalfAngleDeg;
  float targetHzPerSensor;
  float smoothingPct;
  float intensityRefCmS;
  float funcMinDownSpeed;
  float funcMinAmplitude;
  float funcReversalSpeed;
  uint32_t funcMinIntervalMs;
  uint32_t funcRollMaxGapMs;
};

// Sample flags.
static const uint8_t SF_S1_FRESH = 1 << 0;
static const uint8_t SF_S2_FRESH = 1 << 1;
static const uint8_t SF_BOTH = 1 << 2;
static const uint8_t SF_DESCENDING = 1 << 3;
static const uint8_t SF_ROLL = 1 << 4;
static const uint8_t SF_EVENT = 1 << 5;
static const uint8_t SF_BASELINE_FROZEN = 1 << 6;

TrainingSample* sampleRows = nullptr;
GestureEventRow* eventRows = nullptr;
size_t sampleCapacity = 0, eventCapacity = 0;
size_t sampleWrite = 0, sampleCount = 0;
size_t eventWrite = 0, eventCount = 0;
bool datasetUsesPsram = false;

static const size_t CONFIG_CAPACITY = 32;
ConfigSnapshot configRows[CONFIG_CAPACITY];
size_t configWrite = 0, configCount = 0;
uint32_t configRevision = 0;
ConfigSnapshot lastConfig = {};
bool haveConfigSnapshot = false;

uint32_t datasetSessionId = 0;
uint32_t lastRecordedPointSeq = 0;
uint32_t lastRecordedEventSeq = 0;

// Research-only adaptive baseline. It is recorded as a feature but does not alter v16 detection.
float learnedBaseline = 0;
bool learnedBaselineReady = false;

uint8_t phaseToCode(const String& p) {
  if (p == "DESCENSO") return 1;
  if (p == "SUBIENDO") return 2;
  if (p == "REBOTE") return 3;
  if (p == "CANDIDATO") return 4;
  if (p == "ROLL") return 5;
  return 0; // ESPERA / unknown
}

uint16_t age16(uint32_t age) {
  return (uint16_t)(age > 65535 ? 65535 : age);
}

void* allocDatasetBytes(size_t bytes, bool preferPsram) {
  void* p = nullptr;
  if (preferPsram && psramFound()) {
    p = heap_caps_calloc(1, bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }
  if (!p) p = calloc(1, bytes);
  return p;
}

void initDatasetMemory() {
  datasetUsesPsram = psramFound();

  // ~0.4 MB for continuous rows in PSRAM. Graceful fallback on smaller targets.
  const size_t sampleChoices[] = {4096, 2048, 1024, 512};
  for (size_t cap : sampleChoices) {
    sampleRows = (TrainingSample*)allocDatasetBytes(sizeof(TrainingSample) * cap, true);
    if (sampleRows) { sampleCapacity = cap; break; }
  }

  const size_t eventChoices[] = {512, 256, 128, 64};
  for (size_t cap : eventChoices) {
    eventRows = (GestureEventRow*)allocDatasetBytes(sizeof(GestureEventRow) * cap, true);
    if (eventRows) { eventCapacity = cap; break; }
  }

  datasetSessionId = esp_random();
}

ConfigSnapshot currentConfigSnapshot(uint32_t revision) {
  ConfigSnapshot c = {};
  c.revision = revision;
  c.tMs = millis();
  c.baselineCm = baselineCm;
  c.maxRangeCm = maxRangeCm;
  c.hitPlaneCm = hitPlaneCm;
  c.coneHalfAngleDeg = coneHalfAngleDeg;
  c.targetHzPerSensor = targetHzPerSensor;
  c.smoothingPct = smoothingPct;
  c.intensityRefCmS = intensityRefCmS;
  c.funcMinDownSpeed = funcMinDownSpeed;
  c.funcMinAmplitude = funcMinAmplitude;
  c.funcReversalSpeed = funcReversalSpeed;
  c.funcMinIntervalMs = funcMinIntervalMs;
  c.funcRollMaxGapMs = funcRollMaxGapMs;
  return c;
}

bool sameConfig(const ConfigSnapshot& a, const ConfigSnapshot& b) {
  return fabsf(a.baselineCm-b.baselineCm)<0.001f &&
         fabsf(a.maxRangeCm-b.maxRangeCm)<0.001f &&
         fabsf(a.hitPlaneCm-b.hitPlaneCm)<0.001f &&
         fabsf(a.coneHalfAngleDeg-b.coneHalfAngleDeg)<0.001f &&
         fabsf(a.targetHzPerSensor-b.targetHzPerSensor)<0.001f &&
         fabsf(a.smoothingPct-b.smoothingPct)<0.001f &&
         fabsf(a.intensityRefCmS-b.intensityRefCmS)<0.001f &&
         fabsf(a.funcMinDownSpeed-b.funcMinDownSpeed)<0.001f &&
         fabsf(a.funcMinAmplitude-b.funcMinAmplitude)<0.001f &&
         fabsf(a.funcReversalSpeed-b.funcReversalSpeed)<0.001f &&
         a.funcMinIntervalMs==b.funcMinIntervalMs &&
         a.funcRollMaxGapMs==b.funcRollMaxGapMs;
}

void captureConfigIfChanged(bool force=false) {
  ConfigSnapshot probe = currentConfigSnapshot(configRevision);
  if (!force && haveConfigSnapshot && sameConfig(probe, lastConfig)) return;

  configRevision++;
  ConfigSnapshot c = currentConfigSnapshot(configRevision);
  configRows[configWrite] = c;
  configWrite = (configWrite + 1) % CONFIG_CAPACITY;
  if (configCount < CONFIG_CAPACITY) configCount++;
  lastConfig = c;
  haveConfigSnapshot = true;
}

void updateAdaptiveBaseline() {
  if (!pair.valid || !pair.havePoint) return;
  if (!learnedBaselineReady) {
    learnedBaseline = pair.yPoint;
    learnedBaselineReady = true;
    return;
  }

  // Baseline follows only calm hand positions. During a gesture it freezes.
  bool calm = !motion.descending && fabsf(pair.velocity) < 14.0f && fabsf(pair.acceleration) < 700.0f;
  if (calm) {
    const float alpha = 0.018f;
    learnedBaseline += alpha * (pair.yPoint - learnedBaseline);
  }
}

void appendSample(bool eventNow) {
  if (!sampleRows || !sampleCapacity || !pair.valid || !pair.havePoint) return;
  uint32_t now = millis();
  bool f1 = sensorFresh(0, now), f2 = sensorFresh(1, now);

  updateAdaptiveBaseline();
  bool baselineFrozen = motion.descending || fabsf(pair.velocity) >= 14.0f || fabsf(pair.acceleration) >= 700.0f;

  TrainingSample r = {};
  r.tMs = now;
  r.pointSeq = pair.pointSeq;
  r.configRevision = configRevision;
  r.s1Raw=s[0].rawCm; r.s1Fast=s[0].fastCm; r.s1Stable=s[0].stableCm; r.s1Velocity=s[0].velocityCmS;
  r.s2Raw=s[1].rawCm; r.s2Fast=s[1].fastCm; r.s2Stable=s[1].stableCm; r.s2Velocity=s[1].velocityCmS;
  r.yCandidate=pair.yCandidate; r.yPoint=pair.yPoint; r.yStable=pair.yStable;
  r.velocity=pair.velocity; r.acceleration=pair.acceleration;
  r.adaptiveBaseline=learnedBaselineReady?learnedBaseline:pair.yPoint;
  r.displacement=r.yPoint-r.adaptiveBaseline;
  r.coherence=pair.coherence; r.jitter=pair.jitter; r.candidateScore=motion.candidateScore;
  r.ageS1Ms=age16(sensorAge(0,now)); r.ageS2Ms=age16(sensorAge(1,now));
  r.sourceMask=pair.sourceMask; r.phaseCode=phaseToCode(motion.phase); r.humanLabel=-1;
  if(f1)r.flags|=SF_S1_FRESH; if(f2)r.flags|=SF_S2_FRESH; if(pair.both)r.flags|=SF_BOTH;
  if(motion.descending)r.flags|=SF_DESCENDING; if(motion.roll)r.flags|=SF_ROLL;
  if(eventNow)r.flags|=SF_EVENT; if(baselineFrozen)r.flags|=SF_BASELINE_FROZEN;

  sampleRows[sampleWrite]=r;
  sampleWrite=(sampleWrite+1)%sampleCapacity;
  if(sampleCount<sampleCapacity)sampleCount++;
}

void appendEvent() {
  if (!eventRows || !eventCapacity) return;
  uint32_t now=millis();
  GestureEventRow e = {};
  e.eventSeq=motion.eventSeq; e.tMs=now; e.configRevision=configRevision; e.pointSeq=pair.pointSeq;
  e.descentDurationMs=motion.descentStartMs && now>=motion.descentStartMs ? now-motion.descentStartMs : 0;
  e.yAtEvent=pair.yPoint; e.adaptiveBaseline=learnedBaselineReady?learnedBaseline:pair.yPoint;
  e.displacement=pair.yPoint-e.adaptiveBaseline; e.amplitude=motion.lastAmplitude;
  e.peakDownSpeed=motion.lastPeakSpeed; e.velocityAtEvent=pair.velocity; e.accelerationAtEvent=pair.acceleration;
  e.periodMs=motion.lastPeriodMs; e.rollHz=motion.rollHz; e.energy=motion.energy;
  e.coherence=pair.coherence; e.jitter=pair.jitter; e.sourceMask=pair.sourceMask;
  e.rollCount=motion.rollCount; e.midi=(uint8_t)clampu(motion.midi,0,127);
  e.autoClass=motion.roll?2:1; e.humanLabel=-1;

  eventRows[eventWrite]=e;
  eventWrite=(eventWrite+1)%eventCapacity;
  if(eventCount<eventCapacity)eventCount++;
}

void datasetTick() {
  captureConfigIfChanged(false);

  bool newPoint = pair.pointSeq != lastRecordedPointSeq;
  bool newEvent = motion.eventSeq != lastRecordedEventSeq;

  if (newPoint) {
    appendSample(newEvent);
    lastRecordedPointSeq=pair.pointSeq;
  }
  if (newEvent) {
    appendEvent();
    lastRecordedEventSeq=motion.eventSeq;
  }
}

// ---------- Export/API ----------
void beginCsv(const String& header) {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200,"text/csv","");
  server.sendContent(header + "\n");
}

size_t requestedRows(size_t available, size_t maxRows) {
  size_t n = server.hasArg("n") ? (size_t)server.arg("n").toInt() : available;
  if (!n || n > available) n=available;
  if (n > maxRows) n=maxRows;
  return n;
}

void handleDataSamples() {
  size_t n=requestedRows(sampleCount,1024);
  beginCsv("session_id,t_ms,point_seq,config_revision,s1_raw,s1_fast,s1_stable,s1_velocity,s2_raw,s2_fast,s2_stable,s2_velocity,y_candidate,y_point,y_stable,velocity,acceleration,adaptive_baseline,displacement,coherence,jitter,candidate_score,age_s1_ms,age_s2_ms,source_mask,phase_code,flags,human_label");
  if(!sampleRows||!n){server.sendContent("");return;}
  size_t start=(sampleWrite+sampleCapacity-n)%sampleCapacity;
  for(size_t i=0;i<n;i++){
    const TrainingSample&r=sampleRows[(start+i)%sampleCapacity];
    String x=String(datasetSessionId)+","+String(r.tMs)+","+String(r.pointSeq)+","+String(r.configRevision)+",";
    x+=String(r.s1Raw,3)+","+String(r.s1Fast,3)+","+String(r.s1Stable,3)+","+String(r.s1Velocity,3)+",";
    x+=String(r.s2Raw,3)+","+String(r.s2Fast,3)+","+String(r.s2Stable,3)+","+String(r.s2Velocity,3)+",";
    x+=String(r.yCandidate,3)+","+String(r.yPoint,3)+","+String(r.yStable,3)+","+String(r.velocity,3)+","+String(r.acceleration,3)+",";
    x+=String(r.adaptiveBaseline,3)+","+String(r.displacement,3)+","+String(r.coherence,2)+","+String(r.jitter,3)+","+String(r.candidateScore,2)+",";
    x+=String(r.ageS1Ms)+","+String(r.ageS2Ms)+","+String(r.sourceMask)+","+String(r.phaseCode)+","+String(r.flags)+","+String((int)r.humanLabel)+"\n";
    server.sendContent(x);
    if((i&31)==31)delay(0);
  }
  server.sendContent("");
}

void handleDataEvents() {
  size_t n=requestedRows(eventCount,512);
  beginCsv("session_id,event_seq,t_ms,config_revision,point_seq,descent_duration_ms,y_at_event,adaptive_baseline,displacement,amplitude,peak_down_speed,velocity_at_event,acceleration_at_event,period_ms,roll_hz,energy,coherence,jitter,source_mask,roll_count,midi,auto_class,human_label");
  if(!eventRows||!n){server.sendContent("");return;}
  size_t start=(eventWrite+eventCapacity-n)%eventCapacity;
  for(size_t i=0;i<n;i++){
    const GestureEventRow&e=eventRows[(start+i)%eventCapacity];
    String x=String(datasetSessionId)+","+String(e.eventSeq)+","+String(e.tMs)+","+String(e.configRevision)+","+String(e.pointSeq)+","+String(e.descentDurationMs)+",";
    x+=String(e.yAtEvent,3)+","+String(e.adaptiveBaseline,3)+","+String(e.displacement,3)+","+String(e.amplitude,3)+","+String(e.peakDownSpeed,3)+",";
    x+=String(e.velocityAtEvent,3)+","+String(e.accelerationAtEvent,3)+","+String(e.periodMs,2)+","+String(e.rollHz,3)+","+String(e.energy,4)+",";
    x+=String(e.coherence,2)+","+String(e.jitter,3)+","+String(e.sourceMask)+","+String(e.rollCount)+","+String(e.midi)+","+String(e.autoClass)+","+String((int)e.humanLabel)+"\n";
    server.sendContent(x);
  }
  server.sendContent("");
}

void handleDataConfigs() {
  size_t n=requestedRows(configCount,CONFIG_CAPACITY);
  beginCsv("session_id,revision,t_ms,baseline_cm,max_range_cm,hit_plane_cm,cone_half_angle_deg,target_hz_per_sensor,smoothing_pct,intensity_ref_cm_s,func_min_down_speed,func_min_amplitude,func_reversal_speed,func_min_interval_ms,func_roll_max_gap_ms");
  size_t start=(configWrite+CONFIG_CAPACITY-n)%CONFIG_CAPACITY;
  for(size_t i=0;i<n;i++){
    const ConfigSnapshot&c=configRows[(start+i)%CONFIG_CAPACITY];
    String x=String(datasetSessionId)+","+String(c.revision)+","+String(c.tMs)+","+String(c.baselineCm,3)+","+String(c.maxRangeCm,2)+","+String(c.hitPlaneCm,2)+",";
    x+=String(c.coneHalfAngleDeg,2)+","+String(c.targetHzPerSensor,2)+","+String(c.smoothingPct,1)+","+String(c.intensityRefCmS,2)+","+String(c.funcMinDownSpeed,2)+",";
    x+=String(c.funcMinAmplitude,3)+","+String(c.funcReversalSpeed,2)+","+String(c.funcMinIntervalMs)+","+String(c.funcRollMaxGapMs)+"\n";
    server.sendContent(x);
  }
  server.sendContent("");
}

void handleDataMeta() {
  String j="{";
  j+="\"fw\":\"17.0-dataset-matrix\",";
  j+="\"schema\":\""+String(DATA_SCHEMA_VERSION)+"\",";
  j+="\"sessionId\":"+String(datasetSessionId)+",";
  j+="\"psram\":"+String(datasetUsesPsram?"true":"false")+",";
  j+="\"sampleBytes\":"+String(sizeof(TrainingSample))+",\"eventBytes\":"+String(sizeof(GestureEventRow))+",";
  j+="\"sampleCapacity\":"+String(sampleCapacity)+",\"sampleCount\":"+String(sampleCount)+",";
  j+="\"eventCapacity\":"+String(eventCapacity)+",\"eventCount\":"+String(eventCount)+",";
  j+="\"configCapacity\":"+String(CONFIG_CAPACITY)+",\"configCount\":"+String(configCount)+",\"configRevision\":"+String(configRevision)+",";
  j+="\"adaptiveBaseline\":"+String(learnedBaseline,3)+",";
  j+="\"humanLabelSemantics\":\"-1 unlabelled; 0 reject; 1 confirmed_hit; 2 confirmed_roll\",";
  j+="\"sampleShape\":\"N x 28 columns; future sequence tensor = [batch,time,features]\"";
  j+="}";
  server.send(200,"application/json",j);
}

void handleDataClear() {
  sampleWrite=sampleCount=0;
  eventWrite=eventCount=0;
  learnedBaselineReady=false; learnedBaseline=0;
  datasetSessionId=esp_random();
  lastRecordedPointSeq=pair.pointSeq;
  lastRecordedEventSeq=motion.eventSeq;
  captureConfigIfChanged(true);
  server.send(200,"application/json","{\"ok\":true,\"sessionId\":"+String(datasetSessionId)+"}");
}

void registerDatasetRoutes() {
  server.on("/api/data/meta",HTTP_GET,handleDataMeta);
  server.on("/api/data/samples",HTTP_GET,handleDataSamples);
  server.on("/api/data/events",HTTP_GET,handleDataEvents);
  server.on("/api/data/configs",HTTP_GET,handleDataConfigs);
  server.on("/api/data/clear",HTTP_POST,handleDataClear);
}

void setup() {
  initDatasetMemory();
  registerDatasetRoutes();
  setup_v16();
  captureConfigIfChanged(true);
  lastRecordedPointSeq=pair.pointSeq;
  lastRecordedEventSeq=motion.eventSeq;
}

void loop() {
  loop_v16();
  datasetTick();
}
