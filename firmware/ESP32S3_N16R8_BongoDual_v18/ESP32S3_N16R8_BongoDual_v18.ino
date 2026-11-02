/*
  SONIC BONGO / PRISMA
  ESP32-S3 N16R8 - v18 120 BPM REFERENCE

  Inherits v17 dataset matrix + v16 continuous function.

  v18 goals:
    - Keep calibration/function/dataset architecture intact.
    - Apply a one-time 120 BPM reference profile on first v18 boot.
    - Separate pulse tracking from fast ROLL classification.
    - Add a conservative rhythm assistant for missed weak strokes:
      a weak gesture is only accepted when it still has descent + minimum/rebound
      AND lands near the predicted next beat.
    - Existing controls remain editable/persistent after the initial profile load.
*/

#include "base_v17.h"

static const char* FW18 = "18.0-120bpm-reference";
static const uint32_t PROFILE_MARKER = 180120;

// 120 BPM reference = 500 ms between beats.
float rhythmTargetBpm = 120.0f;
uint32_t rhythmToleranceMs = 170;
float rhythmWeakMinSpeed = 18.0f;
float rhythmWeakMinAmplitude = 0.70f;
bool rhythmAssistEnabled = true;

struct RhythmState {
  bool descending=false;
  float startY=0;
  float minY=999;
  float peakDownSpeed=0;
  uint32_t startMs=0;

  uint32_t lastPointSeq=0;
  uint32_t lastObservedEventSeq=0;
  uint32_t lastBeatMs=0;
  uint32_t previousBeatMs=0;

  float learnedPeriodMs=500.0f;
  float bpmEstimate=120.0f;
  float phaseErrorMs=0;
  uint8_t lockCount=0;
  bool locked=false;

  uint32_t nativeBeats=0;
  uint32_t assistedBeats=0;
  uint32_t rejectedWeak=0;
};
RhythmState rhythm;

float targetPeriodMs() {
  return 60000.0f / clampf(rhythmTargetBpm, 40.0f, 300.0f);
}

void loadRhythmConfig() {
  prefs.begin(NVS_NS,true);
  rhythmTargetBpm=clampf(prefs.getFloat("rBpm",120.0f),40.0f,300.0f);
  rhythmToleranceMs=clampu(prefs.getUInt("rTol",170),60,300);
  rhythmWeakMinSpeed=clampf(prefs.getFloat("rWSp",18.0f),5.0f,120.0f);
  rhythmWeakMinAmplitude=clampf(prefs.getFloat("rWAmp",0.70f),0.2f,5.0f);
  rhythmAssistEnabled=prefs.getBool("rAssist",true);
  prefs.end();
  rhythm.learnedPeriodMs=targetPeriodMs();
  rhythm.bpmEstimate=rhythmTargetBpm;
}

void saveRhythmConfig() {
  prefs.begin(NVS_NS,false);
  prefs.putFloat("rBpm",rhythmTargetBpm);
  prefs.putUInt("rTol",rhythmToleranceMs);
  prefs.putFloat("rWSp",rhythmWeakMinSpeed);
  prefs.putFloat("rWAmp",rhythmWeakMinAmplitude);
  prefs.putBool("rAssist",rhythmAssistEnabled);
  prefs.end();
}

bool apply120Profile(bool force=false) {
  prefs.begin(NVS_NS,true);
  uint32_t marker=prefs.getUInt("profile18",0);
  prefs.end();
  if(!force && marker==PROFILE_MARKER) return false;

  // Physical geometry defaults remain the confirmed reference geometry.
  baselineCm=1.0f;
  maxRangeCm=50.0f;
  hitPlaneCm=12.0f;
  coneHalfAngleDeg=7.5f;

  // Acquisition: maximum safe target currently exposed by the proven engine,
  // with less smoothing than v16 default so a 500 ms cycle keeps its shape.
  targetHzPerSensor=16.0f;
  smoothingPct=35.0f;

  // Gesture profile for deliberate quarter-note strokes at 120 BPM.
  intensityRefCmS=60.0f;
  rearmMarginCm=3.0f;
  funcMinDownSpeed=30.0f;
  funcMinAmplitude=1.30f;
  funcReversalSpeed=4.0f;
  funcMinIntervalMs=240;

  // ROLL stays a different phenomenon; 120 BPM pulse is NOT classified as roll.
  funcRollMaxGapMs=340;

  rhythmTargetBpm=120.0f;
  rhythmToleranceMs=170;
  rhythmWeakMinSpeed=18.0f;
  rhythmWeakMinAmplitude=0.70f;
  rhythmAssistEnabled=true;

  String detail;
  bool ok=saveAndVerifyConfig(detail);
  saveRhythmConfig();
  if(ok){
    prefs.begin(NVS_NS,false);
    prefs.putUInt("profile18",PROFILE_MARKER);
    prefs.end();
  }

  resetTracking(true);
  rhythm=RhythmState();
  rhythm.learnedPeriodMs=500.0f;
  rhythm.bpmEstimate=120.0f;
  captureConfigIfChanged(true);
  return ok;
}

void registerBeat(uint32_t beatMs,bool nativeBeat) {
  if(!beatMs) beatMs=millis();
  rhythm.previousBeatMs=rhythm.lastBeatMs;
  if(rhythm.lastBeatMs && beatMs>rhythm.lastBeatMs){
    float p=(float)(beatMs-rhythm.lastBeatMs);
    float target=targetPeriodMs();
    // For the 120 reference, only learn from plausible pulse intervals.
    float lo=target-(float)rhythmToleranceMs*1.35f;
    float hi=target+(float)rhythmToleranceMs*1.35f;
    if(p>=lo && p<=hi){
      if(rhythm.lockCount==0) rhythm.learnedPeriodMs=p;
      else rhythm.learnedPeriodMs=.28f*p+.72f*rhythm.learnedPeriodMs;
      rhythm.lockCount=(uint8_t)clampu(rhythm.lockCount+1,0,20);
      rhythm.locked=rhythm.lockCount>=2;
      rhythm.bpmEstimate=60000.0f/rhythm.learnedPeriodMs;
    } else {
      if(rhythm.lockCount>0) rhythm.lockCount--;
      if(rhythm.lockCount<2) rhythm.locked=false;
      if(!rhythm.locked){rhythm.learnedPeriodMs=target;rhythm.bpmEstimate=rhythmTargetBpm;}
    }
  }
  rhythm.lastBeatMs=beatMs;
  if(nativeBeat) rhythm.nativeBeats++; else rhythm.assistedBeats++;
}

bool nearExpectedBeat(uint32_t now) {
  if(!rhythm.locked || !rhythm.lastBeatMs) return false;
  float expected=(float)rhythm.lastBeatMs+rhythm.learnedPeriodMs;
  rhythm.phaseErrorMs=(float)now-expected;
  return fabsf(rhythm.phaseErrorMs)<=(float)rhythmToleranceMs;
}

void resetAssistGesture() {
  rhythm.descending=false;
  rhythm.startY=pair.yPoint;
  rhythm.minY=999;
  rhythm.peakDownSpeed=0;
  rhythm.startMs=0;
}

void rhythmAssistTick() {
  if(!pair.valid || !pair.havePoint) return;
  if(pair.pointSeq==rhythm.lastPointSeq) return;
  rhythm.lastPointSeq=pair.pointSeq;
  uint32_t now=millis();

  // If the original continuous-function detector already produced an event,
  // use it as the authoritative beat and only update the rhythm model.
  if(motion.eventSeq!=rhythm.lastObservedEventSeq){
    rhythm.lastObservedEventSeq=motion.eventSeq;
    registerBeat(motion.lastCandidateMs?motion.lastCandidateMs:now,true);
    resetAssistGesture();
    return;
  }

  float down=fmaxf(0.0f,-pair.velocity);
  float startThreshold=rhythm.locked?rhythmWeakMinSpeed:funcMinDownSpeed;

  if(!rhythm.descending){
    if(pair.velocity<=-startThreshold){
      rhythm.descending=true;
      rhythm.startY=fmaxf(pair.yPoint, learnedBaselineReady?learnedBaseline:pair.yPoint);
      rhythm.minY=pair.yPoint;
      rhythm.peakDownSpeed=down;
      rhythm.startMs=now;
    }
    return;
  }

  if(pair.yPoint<rhythm.minY) rhythm.minY=pair.yPoint;
  if(down>rhythm.peakDownSpeed) rhythm.peakDownSpeed=down;

  float amp=rhythm.startY-rhythm.minY;
  float rebound=pair.yPoint-rhythm.minY;
  bool reversal=(pair.velocity>=2.0f)||(rebound>=0.25f);
  bool timeout=(now-rhythm.startMs)>700;
  if(!reversal && !timeout) return;

  bool strong=(amp>=funcMinAmplitude && rhythm.peakDownSpeed>=funcMinDownSpeed);
  bool expected=nearExpectedBeat(now);
  bool weakButRhythmic=rhythmAssistEnabled && expected &&
      amp>=rhythmWeakMinAmplitude && rhythm.peakDownSpeed>=rhythmWeakMinSpeed;

  uint32_t sinceBase=motion.lastCandidateMs?now-motion.lastCandidateMs:999999;
  if((strong||weakButRhythmic) && sinceBase>=funcMinIntervalMs){
    // The v16 detector missed this rebound. Inject the same event type used by
    // the existing UI/dataset, so sound, counters and CSV remain coherent.
    emitCandidate(now,amp,rhythm.peakDownSpeed);
    motion.phase=motion.roll?"ROLL":"CANDIDATO";
    motion.descending=false;
    motion.peakY=pair.yPoint;
    motion.minimumY=999;
    motion.peakDownSpeed=0;
    motion.candidateScore=0;
    rhythm.lastObservedEventSeq=motion.eventSeq;
    registerBeat(now,false);
  } else if(expected && amp>=0.35f) {
    rhythm.rejectedWeak++;
  }
  resetAssistGesture();
}

String rhythmJson() {
  String j="{";
  j+="\"fw\":\""+String(FW18)+"\",";
  j+="\"targetBpm\":"+String(rhythmTargetBpm,1)+",";
  j+="\"targetPeriodMs\":"+String(targetPeriodMs(),1)+",";
  j+="\"toleranceMs\":"+String(rhythmToleranceMs)+",";
  j+="\"weakMinSpeed\":"+String(rhythmWeakMinSpeed,1)+",";
  j+="\"weakMinAmplitude\":"+String(rhythmWeakMinAmplitude,2)+",";
  j+="\"assistEnabled\":"+String(rhythmAssistEnabled?"true":"false")+",";
  j+="\"locked\":"+String(rhythm.locked?"true":"false")+",";
  j+="\"lockCount\":"+String(rhythm.lockCount)+",";
  j+="\"learnedPeriodMs\":"+String(rhythm.learnedPeriodMs,1)+",";
  j+="\"bpmEstimate\":"+String(rhythm.bpmEstimate,2)+",";
  j+="\"phaseErrorMs\":"+String(rhythm.phaseErrorMs,1)+",";
  j+="\"nativeBeats\":"+String(rhythm.nativeBeats)+",";
  j+="\"assistedBeats\":"+String(rhythm.assistedBeats)+",";
  j+="\"rejectedWeak\":"+String(rhythm.rejectedWeak);
  j+="}";
  return j;
}

void handleRhythmStatus(){server.send(200,"application/json",rhythmJson());}

void handleRhythmConfig(){
  if(server.hasArg("bpm")) rhythmTargetBpm=clampf(server.arg("bpm").toFloat(),40,300);
  if(server.hasArg("tol")) rhythmToleranceMs=clampu(server.arg("tol").toInt(),60,300);
  if(server.hasArg("weakSpeed")) rhythmWeakMinSpeed=clampf(server.arg("weakSpeed").toFloat(),5,120);
  if(server.hasArg("weakAmp")) rhythmWeakMinAmplitude=clampf(server.arg("weakAmp").toFloat(),.2f,5);
  if(server.hasArg("assist")) rhythmAssistEnabled=server.arg("assist")!="0";
  saveRhythmConfig();
  rhythm.lockCount=0;rhythm.locked=false;rhythm.lastBeatMs=0;
  rhythm.learnedPeriodMs=targetPeriodMs();rhythm.bpmEstimate=rhythmTargetBpm;
  server.send(200,"application/json",rhythmJson());
}

void handleProfile120(){
  bool ok=apply120Profile(true);
  server.send(ok?200:500,"application/json",String("{\"ok\":")+(ok?"true":"false")+",\"profile\":\"120 BPM\"}");
}

void setup() {
  // Register before the inherited WebServer begins listening.
  server.on("/api/rhythm/status",HTTP_GET,handleRhythmStatus);
  server.on("/api/rhythm/config",HTTP_POST,handleRhythmConfig);
  server.on("/api/profile120",HTTP_POST,handleProfile120);

  setup_v17();
  loadRhythmConfig();
  apply120Profile(false);
  rhythm.lastPointSeq=pair.pointSeq;
  rhythm.lastObservedEventSeq=motion.eventSeq;
}

void loop() {
  loop_v17();
  rhythmAssistTick();
}
