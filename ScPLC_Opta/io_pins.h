#pragma once

// Digital input alarms (24V via Opta input interface)
constexpr int PIN_A_CON_ALA = A0;  // Analyzer connected alarm
constexpr int PIN_FLO_ALA   = A1;  // Flow alarm
constexpr int PIN_R_C_P_ALA = A2;  // Right cylinder pressure alarm
constexpr int PIN_L_C_P_ALA = A3;  // Left cylinder pressure alarm
constexpr int PIN_F_O_ALA   = A4;  // Fans overload alarm
constexpr int PIN_A_P_ALA   = A5;  // Air pressure alarm
constexpr int PIN_TRA_ALA   = A6;  // Transformer alarm

// Digital outputs (24V)
constexpr int PIN_FAN_1     = D0;
constexpr int PIN_FAN_2     = D1;
constexpr int PIN_RM_V_1    = D2;  // Room valve 1
constexpr int PIN_N2_RM_V_1 = D3;  // N2 valve room 1

// A0602 analog inputs (4-20 mA) - abstracted as channels
constexpr int CH_O2_SIG  = 1;  // I1+ / I1-
constexpr int CH_CO2_SIG = 2;  // I2+ / I2-

// D1608E relay outputs - abstract as indices 1..8
constexpr int REL_MEAS_P      = 1; // Measuring pump
constexpr int REL_V_4_4       = 2; // Valve 4 & 4
constexpr int REL_V_2_3       = 3; // Valve 2 & 3
constexpr int REL_V_1_1       = 4; // Valve 1 & 1
constexpr int REL_M_V_N2_GEN  = 5; // Measuring valve N2 generator
constexpr int REL_MEAS_V_CAL  = 6; // Measuring valve Calibration
constexpr int REL_MEAS_V_A_I  = 7; // Measuring valve Ads In
constexpr int REL_N2_REC_V    = 8; // N2 recirculation valve

// Built-in Opta LEDs that correspond to onboard outputs D0..D3.
// (Function name kept for compatibility with existing calls.)
constexpr int LED_REL_1 = LED_D0; // aligns with PIN_FAN_1 (D0)
constexpr int LED_REL_2 = LED_D1; // aligns with PIN_FAN_2 (D1)
constexpr int LED_REL_3 = LED_D2; // aligns with PIN_RM_V_1 (D2)
constexpr int LED_REL_4 = LED_D3; // aligns with PIN_N2_RM_V_1 (D3)

// Helper to drive built-in LEDs by index 1..4.
void mirrorRelayToLed(int relayIndex, bool on);

void initPins();
