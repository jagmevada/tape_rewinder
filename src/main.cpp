#include <Arduino.h>
#include <PinChangeInterrupt.h>

#include <moving_average.hpp>

#define DBG
// --- Serial Print Macros for Quick Debugging ---
#ifdef DBG
#define p(x) Serial.print(x)
#define pln(x) Serial.println(x)
#else
#define p(x) ((void)0)
#define pln(x) ((void)0)
#endif

// --- Debounce delay (ms) ---
#define D_DELAY 500

// --- Forward Declarations for Interrupt Handlers ---
void startAll();
void stopAll();
void startLed1();
void startLed2();
void startLed3();
void startLed4();
void stopLed1();
void stopLed2();
void stopLed3();
void stopLed4();

// === Pin Definitions ===
const byte ledPin1 = 3;     // OC2B, Channel 1 PWM & LED indicator####
const byte ledPin2 = 9;     // OC1A, Channel 2 PWM & LED indicator####
const byte ledPin3 = 10;    // OC1B, Channel 3 PWM & LED indicator####
const byte ledPin4 = 11;    // OC2A, Channel 4 PWM & LED indicator####
const byte masterLed = 13;  // Master ON/OFF indicator

const byte startAllButton = 2;  // External Interrupt 0
const byte stopAllButton = 4;   // PCINT for STOP ALL

const byte startButton1 = 5;  // PCINT for START channel 1
const byte startButton2 = 6;  // PCINT for START channel 2
const byte startButton3 = 7;  // PCINT for START channel 3
const byte startButton4 = 8;  // PCINT for START channel 4

const byte stopButton1 = 12;  // PCINT for STOP channel 1
const byte stopButton2 = A0;  // PCINT for STOP channel 2
const byte stopButton3 = A1;  // PCINT for STOP channel 3
const byte stopButton4 = A2;  // PCINT for STOP channel 4

const byte sensePin1 = A4;  // Analog current sense channel 1####
const byte sensePin2 = A5;  // Analog current sense channel 2####
const byte sensePin3 = A6;  // Analog current sense channel 3####
const byte sensePin4 = A7;  // Analog current sense channel 4####

// === Timer & PWM Aliases ===
#define PWM_1 OCR2B  // drives pwmPin1
#define PWM_2 OCR1A  // drives pwmPin2
#define PWM_3 OCR1B  // drives pwmPin3
#define PWM_4 OCR2A  // drives pwmPin4

// === Filter & Control Parameters ===
#define WINDOW_SIZE 4
#define RN (WINDOW_SIZE * 1024.0)
#define DUTY1 180
#define DUTY2 180
#define DUTY3 180
#define DUTY4 180
#define th1 0.35
#define th2 0.35
#define th3 0.35
#define th4 0.35

// === Globals for Moving-Sum Filter ===
// float x1[N], x2[N], x3[N], x4[N];

float y1n, y1n_1 = 0;
float y2n, y2n_1 = 0;
float y3n, y3n_1 = 0;
float y4n, y4n_1 = 0;
float x1 = 0, x2 = 0, x3 = 0, x4 = 0;
unsigned long on1 = 0, on2 = 0, on3 = 0, on4 = 0;

// --- For deferred startAll logic ---
volatile bool startAllFlag = false;

struct PWMState {
  bool to_start = false;
  bool to_stop = false;

  bool ramp_up = false;
  bool ramp_down = false;

  uint8_t target_duty = 0;
  uint8_t current_duty = 0;

  unsigned long last_update = 0;
};

const unsigned long kRampInterval = 50;  // ms between duty updates
const uint8_t kRampStep = 5;             // duty increment/decrement step

MovingAverage<WINDOW_SIZE> motorFilter[4];

// PWM states for the 4 motors
PWMState pwm_state[4];

void setup() {
  randomSeed(analogRead(A3));
  PWM_1 = 0x00;  // pwm1
  PWM_2 = 0x00;  // pwm2
  PWM_3 = 0x00;  // pwm3
  PWM_4 = 0x00;  // pwm4

  //  --- Timer1: 16-bit Fast PWM, Mode 14 (TOP = ICR1), prescaler 8 ---
  TCCR1A = _BV(WGM11) | _BV(COM1A1) | _BV(COM1B1);
  TCCR1B = _BV(WGM12) | _BV(WGM13) | _BV(CS11);
  ICR1 = 0xFF;  // 8-bit range via 16-bit timer

  // --- Timer2: 8-bit Fast PWM, Mode 3 (TOP = 0xFF), prescaler 8 ---
  TCCR2A = _BV(WGM21) | _BV(WGM20) | _BV(COM2A1) | _BV(COM2B1);
  TCCR2B = _BV(CS21);

  TCCR2A &= ~_BV(COM2B1);  // ledPin1 (D3)
  TCCR1A &= ~_BV(COM1A1);  // ledPin2 (D9)
  TCCR1A &= ~_BV(COM1B1);  // ledPin3 (D10)
  TCCR2A &= ~_BV(COM2A1);  // ledPin4 (D11)

  // PWM / LED pins
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  pinMode(ledPin4, OUTPUT);
  pinMode(masterLed, OUTPUT);

  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);
  digitalWrite(ledPin3, LOW);
  digitalWrite(ledPin4, LOW);
  digitalWrite(masterLed, LOW);

  // Start/Stop buttons
  pinMode(startAllButton, INPUT_PULLUP);
  pinMode(stopAllButton, INPUT_PULLUP);
  pinMode(startButton1, INPUT_PULLUP);
  pinMode(startButton2, INPUT_PULLUP);
  pinMode(startButton3, INPUT_PULLUP);
  pinMode(startButton4, INPUT_PULLUP);
  pinMode(stopButton1, INPUT_PULLUP);
  pinMode(stopButton2, INPUT_PULLUP);
  pinMode(stopButton3, INPUT_PULLUP);
  pinMode(stopButton4, INPUT_PULLUP);

  // PWM_1 = 0; digitalWrite(ledPin1, LOW);
  // PWM_2 = 0; digitalWrite(ledPin2, LOW);
  // PWM_3 = 0; digitalWrite(ledPin3, LOW);
  // PWM_4 = 0; digitalWrite(ledPin4, LOW);
  stopAll();

  // --- Interrupts for Start/Stop ---
  attachInterrupt(digitalPinToInterrupt(startAllButton), startAll, FALLING);
  attachPCINT(digitalPinToPCINT(stopAllButton), stopAll, RISING);

  attachPCINT(digitalPinToPCINT(startButton1), startLed1, RISING);
  attachPCINT(digitalPinToPCINT(startButton2), startLed2, RISING);
  attachPCINT(digitalPinToPCINT(startButton3), startLed3, RISING);
  attachPCINT(digitalPinToPCINT(startButton4), startLed4, RISING);

  attachPCINT(digitalPinToPCINT(stopButton1), stopLed1, RISING);
  attachPCINT(digitalPinToPCINT(stopButton2), stopLed2, RISING);
  attachPCINT(digitalPinToPCINT(stopButton3), stopLed3, RISING);
  attachPCINT(digitalPinToPCINT(stopButton4), stopLed4, RISING);

  Serial.begin(9600);
}

void loop() {
  // Handle deferred startAll logic (non-blocking, not in ISR)
  if (startAllFlag) {
    digitalWrite(masterLed, HIGH);
    pln("startall");
    TCCR2A |= _BV(COM2B1);
    PWM_1 = DUTY1;
    on1 = millis();
    pln("start1");
    delay(random(10, 21));
    TCCR1A |= _BV(COM1A1);
    PWM_2 = DUTY2;
    on2 = millis();
    pln("start2");
    delay(random(10, 21));
    TCCR1A |= _BV(COM1B1);
    PWM_3 = DUTY3;
    on3 = millis();
    pln("start3");
    delay(random(10, 21));
    TCCR2A |= _BV(COM2A1);
    PWM_4 = DUTY4;
    on4 = millis();
    pln("start4");
    startAllFlag = false;
  }
  // Read & normalize currents
  x1 = analogRead(sensePin1) / RN;
  x2 = analogRead(sensePin2) / RN;
  x3 = analogRead(sensePin3) / RN;
  x4 = analogRead(sensePin4) / RN;
  y1n = motorFilter[0].update(x1);
  y2n = motorFilter[1].update(x2);
  y3n = motorFilter[2].update(x3);
  y4n = motorFilter[3].update(x4);

  // Debug print: print all sensePin1-4 values separated by tabs
  // p(y1n); p("\t");
  // p(y2n); p("\t");
  // p(y3n); p("\t");
  // pln(y4n);

  // // Stall detection: shut off if over threshold after 1 second
  // if (y1n >= th1 && millis() - on1 > 1000) {
  //     on1 = millis();
  //     stopLed1();
  //     pln("stop1");
  // }
  // if (y2n >= th2 && millis() - on2 > 1000) {
  //     on2 = millis();
  //     stopLed2();
  //     pln("stop2");
  // }
  // if (y3n >= th3 && millis() - on3 > 1000) {
  //     on3 = millis();
  //     stopLed3();
  //     pln("stop3");
  // }
  // if (y4n >= th4 && millis() - on4 > 1000) {
  //     on4 = millis();
  //     stopLed4();
  //     pln("stop4");
  // }
  // Print them in one line

  Serial.print("y1n: ");
  Serial.print(y1n);
  Serial.print("\t y2n: ");
  Serial.print(y2n);
  Serial.print("\t y3n: ");
  Serial.print(y3n);
  Serial.print("\t y4n: ");
  Serial.println(y4n);

  delay(10);
}

// --- Global START/STOP Handlers ---
void startAll() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < D_DELAY) return;
  last = now;
  startAllFlag = true;
}

void stopAll() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < D_DELAY) return;
  last = now;
  PWM_1 = PWM_2 = PWM_3 = PWM_4 = 0;
  // Force PWM pins low by toggling COMxy1 bits
  TCCR2A &= ~_BV(COM2B1);  // ledPin1 (D3)
  TCCR1A &= ~_BV(COM1A1);  // ledPin2 (D9)
  TCCR1A &= ~_BV(COM1B1);  // ledPin3 (D10)
  TCCR2A &= ~_BV(COM2A1);  // ledPin4 (D11)
  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);
  digitalWrite(ledPin3, LOW);
  digitalWrite(ledPin4, LOW);

  digitalWrite(masterLed, LOW);
  pln("stopall");
}

// --- Per-Channel START Handlers ---
void startLed1() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < D_DELAY) return;
  last = now;
  TCCR2A |= _BV(COM2B1);  // Reconnect timer to pin
  on1 = millis();
  PWM_1 = DUTY1;
  pln("start1");
}
void startLed2() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < D_DELAY) return;
  last = now;
  TCCR1A |= _BV(COM1A1);  // Reconnect timer to pin
  on2 = millis();
  PWM_2 = DUTY2;
  pln("start2");
}
void startLed3() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < D_DELAY) return;
  last = now;
  TCCR1A |= _BV(COM1B1);  // Reconnect timer to pin
  on3 = millis();
  PWM_3 = DUTY3;
  pln("start3");
}
void startLed4() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < D_DELAY) return;
  last = now;
  TCCR2A |= _BV(COM2A1);  // Reconnect timer to pin
  on4 = millis();
  PWM_4 = DUTY4;
  pln("start4");
}

// --- Per-Channel STOP Handlers ---
void stopLed1() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < D_DELAY) return;
  last = now;
  PWM_1 = 0;
  TCCR2A &= ~_BV(COM2B1);  // Disconnect timer from pin
  digitalWrite(ledPin1, LOW);
  pln("stop1");
}
void stopLed2() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < D_DELAY) return;
  last = now;
  PWM_2 = 0;
  TCCR1A &= ~_BV(COM1A1);  // Disconnect timer from pin
  digitalWrite(ledPin2, LOW);
  pln("stop2");
}
void stopLed3() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < D_DELAY) return;
  last = now;
  PWM_3 = 0;
  TCCR1A &= ~_BV(COM1B1);  // Disconnect timer from pin
  digitalWrite(ledPin3, LOW);
  pln("stop3");
}
void stopLed4() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < D_DELAY) return;
  last = now;
  PWM_4 = 0;
  TCCR2A &= ~_BV(COM2A1);  // Disconnect timer from pin
  digitalWrite(ledPin4, LOW);
  pln("stop4");
}
