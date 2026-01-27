#include <Arduino.h>
#include "io_pins.h"

void mirrorRelayToLed(int relayIndex, bool on) {
  int ledPin = -1;
  switch (relayIndex) {
    case 1: ledPin = LED_REL_1; break;
    case 2: ledPin = LED_REL_2; break;
    case 3: ledPin = LED_REL_3; break;
    case 4: ledPin = LED_REL_4; break;
    default: ledPin = -1; break;
  }

  if (ledPin >= 0) {
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, on ? HIGH : LOW);
  }
}

void initPins() {
  // Inputs (alarms)
  pinMode(PIN_A_CON_ALA, INPUT);
  pinMode(PIN_FLO_ALA, INPUT);
  pinMode(PIN_R_C_P_ALA, INPUT);
  pinMode(PIN_L_C_P_ALA, INPUT);
  pinMode(PIN_F_O_ALA, INPUT);
  pinMode(PIN_A_P_ALA, INPUT);
  pinMode(PIN_TRA_ALA, INPUT);

  // Outputs (24V)
  pinMode(PIN_FAN_1, OUTPUT);
  pinMode(PIN_FAN_2, OUTPUT);
  pinMode(PIN_RM_V_1, OUTPUT);
  pinMode(PIN_N2_RM_V_1, OUTPUT);

  digitalWrite(PIN_FAN_1, LOW);
  digitalWrite(PIN_FAN_2, LOW);
  digitalWrite(PIN_RM_V_1, LOW);
  digitalWrite(PIN_N2_RM_V_1, LOW);

  // LED mirror pins
  pinMode(LED_REL_1, OUTPUT);
  pinMode(LED_REL_2, OUTPUT);
  pinMode(LED_REL_3, OUTPUT);
  pinMode(LED_REL_4, OUTPUT);

  digitalWrite(LED_REL_1, LOW);
  digitalWrite(LED_REL_2, LOW);
  digitalWrite(LED_REL_3, LOW);
  digitalWrite(LED_REL_4, LOW);
}
