// ==========================================================
//  FREQUENCY PROJECT — ESP32-S3 PAN/TILT CONTROLLER
//  Versione compatibile GUI Manuale_v5 con STOP (2025-11)
// ----------------------------------------------------------
//  ✅ COMANDI SERIALI ACCETTATI:
//  PING                → risponde "PONG"
//  STATUS              → stampa stato completo
//  HOME                → esegue homing automatico (PAN→TILT)
//  ZERO                → ritorna a PAN=180°, TILT=0°
//  MOVE PAN ±X         → muove di ±X gradi antenna (fluido)
//  MOVE TILT ±X        → muove di ±X gradi antenna (fluido)
//  STOP PAN/TILT       → interrompe il movimento in corso
//  MOTOR_PAN_ON/OFF    → abilita/disabilita PAN
//  MOTOR_TILT_ON/OFF   → abilita/disabilita TILT
// ==========================================================

#include <Arduino.h>

// === Pin motori / driver MKS SERVO42C ======================
#define PAN_STEP   27
#define PAN_DIR    26
#define PAN_EN     14
#define TILT_STEP  33
#define TILT_DIR   32
#define TILT_EN    25

// === Sensori GL-8H ========================================
#define PAN_HOME_PIN  4
#define TILT_HOME_PIN 5

// === Parametri meccanici misurati ==========================
#define PAN_STEPS_360  25060.0f
#define TILT_STEPS_180 6119.0f

#define PAN_STEPS_PER_DEG  (PAN_STEPS_360 / 360.0f)   // ≈ 69.6
#define TILT_STEPS_PER_DEG (TILT_STEPS_180 / 180.0f)  // ≈ 34.0

// === Parametri dinamici ===================================
const int stepDelayMin = 150;
const int stepDelayMax = 1000;

// === Stato =================================================
float panAngle = 180.0f, tiltAngle = 0.0f;
bool motorPanEnabled = true, motorTiltEnabled = true;
bool panHomeDone = false, tiltHomeDone = false;
long panStepsBetween = 0, tiltStepsBetween = 0;

const float PAN_HOME_DEG  = 180.0f;
const float TILT_HOME_DEG =   0.0f;

// === STOP management ======================================
volatile bool stopPan  = false;
volatile bool stopTilt = false;

// ===========================================================
//                      FUNZIONI BASE
// ===========================================================
void setPanEnable(bool on){ digitalWrite(PAN_EN, on ? LOW : HIGH); motorPanEnabled = on; }
void setTiltEnable(bool on){ digitalWrite(TILT_EN, on ? LOW : HIGH); motorTiltEnabled = on; }

long degToSteps(bool panAxis, float deg){
  return lround(deg * (panAxis ? PAN_STEPS_PER_DEG : TILT_STEPS_PER_DEG));
}
float stepsToDeg(bool panAxis, long steps){
  return (float)steps / (panAxis ? PAN_STEPS_PER_DEG : TILT_STEPS_PER_DEG);
}

// ===========================================================
//                 MOVIMENTO FLUIDO CON RAMPA
// ===========================================================
void moveMotorSmooth(bool panAxis, float deltaDeg){
  if (fabs(deltaDeg) < 0.01f) return;
  long steps = degToSteps(panAxis, fabs(deltaDeg));
  if (steps < 1) return;

  int stepPin = panAxis ? PAN_STEP : TILT_STEP;
  int dirPin  = panAxis ? PAN_DIR  : TILT_DIR;
  bool dirHigh = (deltaDeg > 0);
  digitalWrite(dirPin, dirHigh);

  volatile bool *stopFlag = panAxis ? &stopPan : &stopTilt;
  *stopFlag = false;

  int accelSteps = min<long>(steps / 4, 300);
  int decelStart = steps - accelSteps;

  for (long i=0; i<steps; i++){
    if (*stopFlag) {
      Serial.printf("[STOP] %s interrotto\n", panAxis ? "PAN" : "TILT");
      break;
    }

    int d = stepDelayMax;
    if (i < accelSteps)
      d = map(i, 0, accelSteps, stepDelayMax, stepDelayMin);
    else if (i > decelStart)
      d = map(i, decelStart, steps, stepDelayMin, stepDelayMax);

    digitalWrite(stepPin, HIGH); delayMicroseconds(d);
    digitalWrite(stepPin, LOW);  delayMicroseconds(d);
  }

  if (!*stopFlag) {
    if (panAxis) panAngle += deltaDeg;
    else tiltAngle += deltaDeg;
  }
  *stopFlag = false;
}

// ===========================================================
//                         HOMING
// ===========================================================
inline void pulseStep(bool panAxis, int dly){
  int pin = panAxis ? PAN_STEP : TILT_STEP;
  digitalWrite(pin, HIGH); delayMicroseconds(dly);
  digitalWrite(pin, LOW);  delayMicroseconds(dly);
}
inline void setDir(bool panAxis, bool dirHigh){
  digitalWrite(panAxis ? PAN_DIR : TILT_DIR, dirHigh);
}

bool stepUntilMagnet(bool panAxis, int pin, bool dirHigh, long maxSteps, int dly, long &stepsOut){
  setDir(panAxis, dirHigh);
  stepsOut=0; uint8_t st=0;
  while(stepsOut<maxSteps){
    pulseStep(panAxis,dly); stepsOut++;
    int v=digitalRead(pin);
    if(v==LOW){ if(++st>=3) return true; } else st=0;
  }
  return false;
}
void exitWindow(bool panAxis,int pin,bool dirHigh,long maxSteps,int dly){
  setDir(panAxis,dirHigh);
  long k=0; while(digitalRead(pin)==LOW && k<maxSteps){ pulseStep(panAxis,dly); k++; }
  delay(10);
}

void doHoming(){
  Serial.println("[HOME] Avvio homing…");
  const int dPan=400, dTilt=400;

  // ---- PAN ----
  exitWindow(true, PAN_HOME_PIN, true, 2000, dPan);
  long tmp=0;
  if(!stepUntilMagnet(true,PAN_HOME_PIN,true,40000,dPan,tmp)){ Serial.println("[PAN] Magnete 360 non trovato"); return; }
  exitWindow(true, PAN_HOME_PIN,false,2000,dPan);
  long stepsCCW=0;
  if(!stepUntilMagnet(true,PAN_HOME_PIN,false,80000,dPan,stepsCCW)){ Serial.println("[PAN] Magnete 0 non trovato"); return; }
  panStepsBetween=stepsCCW;
  long halfPan=panStepsBetween/2;
  setDir(true,true); for(long i=0;i<halfPan;i++) pulseStep(true,dPan);
  panAngle=180.0f; panHomeDone=true;
  Serial.printf("[PAN] OK (%ld passi, 360°)\n",panStepsBetween);

  // ---- TILT ----
  exitWindow(false, TILT_HOME_PIN, true, 2000, dTilt);
  long steps1=0; if(!stepUntilMagnet(false,TILT_HOME_PIN,true,20000,dTilt,steps1)){ Serial.println("[TILT] +90 non trovato"); return; }
  exitWindow(false, TILT_HOME_PIN,false,1500,dTilt);
  long steps2=0; if(!stepUntilMagnet(false,TILT_HOME_PIN,false,20000,dTilt,steps2)){ Serial.println("[TILT] -90 non trovato"); return; }
  tiltStepsBetween=steps2;
  long halfTilt=tiltStepsBetween/2;
  setDir(false,true); for(long i=0;i<halfTilt;i++) pulseStep(false,dTilt);
  tiltAngle=0.0f; tiltHomeDone=true;
  Serial.printf("[TILT] OK (%ld passi, 180°)\n",tiltStepsBetween);
  Serial.println("[HOME] Homing completato (PAN=180, TILT=0)");
}

// ===========================================================
//                        ZERO
// ===========================================================
void goHome(){
  float dPan=PAN_HOME_DEG-panAngle;
  float dTilt=TILT_HOME_DEG-tiltAngle;
  if(motorPanEnabled) moveMotorSmooth(true,dPan);
  if(motorTiltEnabled) moveMotorSmooth(false,dTilt);
  panAngle=PAN_HOME_DEG; tiltAngle=TILT_HOME_DEG;
  Serial.println("[ZERO] PAN=180, TILT=0");
}

// ===========================================================
//                        STATUS
// ===========================================================
void statusReport(){
  Serial.printf("STATUS -> PAN: %.2f° | TILT: %.2f°\n", panAngle, tiltAngle);
  Serial.printf("MOTORS -> PAN:%s | TILT:%s\n",
                motorPanEnabled?"ON":"OFF",motorTiltEnabled?"ON":"OFF");
  Serial.flush();
}

// ===========================================================
//                         SETUP
// ===========================================================
void setup(){
  Serial.begin(115200);
  pinMode(PAN_STEP,OUTPUT); pinMode(PAN_DIR,OUTPUT); pinMode(PAN_EN,OUTPUT);
  pinMode(TILT_STEP,OUTPUT); pinMode(TILT_DIR,OUTPUT); pinMode(TILT_EN,OUTPUT);
  pinMode(PAN_HOME_PIN,INPUT_PULLUP); pinMode(TILT_HOME_PIN,INPUT_PULLUP);
  setPanEnable(true); setTiltEnable(true);
  Serial.println("ESP32-S3 Ready (MOVE+STOP support)");
}

// ===========================================================
//                         LOOP
// ===========================================================
void loop(){
  if(!Serial.available()) return;
  String cmd=Serial.readStringUntil('\n'); cmd.trim(); if(cmd.isEmpty()) return;

  if(cmd=="PING"){ Serial.println("PONG"); return; }
  if(cmd=="STATUS"){ statusReport(); return; }
  if(cmd=="HOME"){ doHoming(); return; }
  if(cmd=="ZERO"){ goHome(); return; }

  if(cmd=="MOTOR_PAN_ON"){ setPanEnable(true); Serial.println("PAN ON"); return; }
  if(cmd=="MOTOR_PAN_OFF"){ setPanEnable(false); Serial.println("PAN OFF"); return; }
  if(cmd=="MOTOR_TILT_ON"){ setTiltEnable(true); Serial.println("TILT ON"); return; }
  if(cmd=="MOTOR_TILT_OFF"){ setTiltEnable(false); Serial.println("TILT OFF"); return; }

  // ---- STOP PAN/TILT ----
  if(cmd=="STOP PAN"){ stopPan=true; Serial.println("[STOP] PAN richiesto"); return; }
  if(cmd=="STOP TILT"){ stopTilt=true; Serial.println("[STOP] TILT richiesto"); return; }

  // ---- MOVE PAN/TILT ±X ----
  if(cmd.startsWith("MOVE ")){
    int axisEnd=cmd.indexOf(' ',5);
    if(axisEnd>0){
      String axis=cmd.substring(5,axisEnd);
      float val=cmd.substring(axisEnd+1).toFloat();
      if(axis=="PAN" && motorPanEnabled) moveMotorSmooth(true,val);
      else if(axis=="TILT" && motorTiltEnabled) moveMotorSmooth(false,val);
      Serial.printf("OK MOVE %s %.2f°\n",axis.c_str(),val);
      return;
    }
  }

  Serial.println("[WARN] Comando non riconosciuto");
}
