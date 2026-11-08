#include <avr/io.h>

// ---------- Settings that MATLAB can change ----------
float KP = 0.1;
float KI = 40.0;
float remoteTargetV = 12.0;
bool  remoteMode = false;

const float MIN_VREF = 5.0, MAX_VREF = 18.0;
const float TS = 0.001;                 // 1 ms control loop (now accurate)
const uint8_t MAX_DUTY = 200;

// ---------- Pins ----------
const int FEEDBACK_PIN = A5;
const int POT_PIN      = A4;
// PWM output is now D9 (OC1A)  <-- move your wire from D5 to D9

const float R_TOP = 40.0, R_BOT = 10.0;
const float VOLTAGE_MULTIPLIER = (R_TOP + R_BOT) / R_BOT;

// ---------- Controller state ----------
float error = 0, prev_error = 0, control_y = 0, prev_y = 0;
float lastTargetV = 0;
int   lastAdc = 0;

char serialBuffer[40];
byte serialIndex = 0;

void handleCommand(char *cmd) {
  if (strcmp(cmd, "RESET") == 0) {          // duty back to 0, clear PI memory
    control_y = 0; prev_y = 0; prev_error = 0;
    OCR1A = 0;
    Serial.println(F("ACK,RESET"));
    return;
  }
  char *eq = strchr(cmd, '=');
  if (eq == NULL) { Serial.println(F("ERROR,INVALID_COMMAND")); return; }
  *eq = '\0';
  float val = atof(eq + 1);

  if (strcmp(cmd, "VREF") == 0) {
    if (val >= MIN_VREF && val <= MAX_VREF) {
      remoteTargetV = val; remoteMode = true;
      Serial.print(F("ACK,VREF,")); Serial.println(val, 2);
    } else Serial.println(F("ERROR,VREF_OUT_OF_RANGE"));
  }
  else if (strcmp(cmd, "KP") == 0) {
    if (val >= 0 && val <= 5) { KP = val; Serial.print(F("ACK,KP,")); Serial.println(val, 4); }
    else Serial.println(F("ERROR,KP_OUT_OF_RANGE"));
  }
  else if (strcmp(cmd, "KI") == 0) {
    if (val >= 0 && val <= 500) { KI = val; Serial.print(F("ACK,KI,")); Serial.println(val, 4); }
    else Serial.println(F("ERROR,KI_OUT_OF_RANGE"));
  }
  else Serial.println(F("ERROR,INVALID_COMMAND"));
}

void processSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialIndex == 0) continue;
      serialBuffer[serialIndex] = '\0';
      handleCommand(serialBuffer);
      serialIndex = 0;
    } else if (serialIndex < sizeof(serialBuffer) - 1) {
      serialBuffer[serialIndex++] = c;
    } else {
      serialIndex = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Timer1, 8-bit fast PWM on D9, ~62.5 kHz (does NOT affect millis/micros)
  DDRB |= (1 << PB1);
  TCCR1A = (1 << COM1A1) | (1 << WGM10);
  TCCR1B = (1 << WGM12)  | (1 << CS10);
  OCR1A = 0;

  Serial.println(F("READY"));
}

void loop() {
  processSerialCommands();

  // ---- Control loop: every 1 ms ----
  static unsigned long lastCtl = 0;
  if (micros() - lastCtl >= 1000) {
    lastCtl += 1000;

    float target_vout;
    if (remoteMode) target_vout = remoteTargetV;
    else target_vout = map(analogRead(POT_PIN), 0, 1023, 50, 180) / 10.0;
    lastTargetV = target_vout;

    float target_adc = (target_vout / VOLTAGE_MULTIPLIER / 5.0) * 1023.0;
    lastAdc = analogRead(FEEDBACK_PIN);

    error = target_adc - (float)lastAdc;
    control_y = prev_y + KP * (error - prev_error) + KI * TS * error;

    if (control_y > MAX_DUTY) control_y = MAX_DUTY;
    else if (control_y < 0)   control_y = 0;

    OCR1A = (uint8_t)control_y;
    prev_error = error;
    prev_y = control_y;
  }

  // ---- Telemetry: every 50 ms, easy-to-parse CSV ----
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 10) {
    lastPrint = millis();
    float vout = (lastAdc / 1023.0) * 5.0 * VOLTAGE_MULTIPLIER;
    float duty = ((float)OCR1A / 255.0) * 100.0;
    Serial.print(F("DATA,"));
    Serial.print(millis());      Serial.print(',');
    Serial.print(lastTargetV, 2); Serial.print(',');
    Serial.print(vout, 3);       Serial.print(',');
    Serial.print(duty, 1);       Serial.print(',');
    Serial.println(OCR1A);
  }
}