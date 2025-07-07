/* This code is for control CubeMars AK70-10 Motor with Python via Arduino.
   Using MCP2515 based CAN Bus Shield for CAN Communication between Motor & Arduino. */

/* Reference Code & Library
      1. https://github.com/NikodemBartnik/Overpowered-Robotic-Chassis
      2. https://github.com/lezgin-alimoglu/CubeMars-AK60-6
      3. https://github.com/coryjfowler/MCP_CAN_lib */

/*********************************************************************************************************
  START OF THE CODE
*********************************************************************************************************/

/* IMPORT LIBARAY*/
#include <mcp_can.h>
#include <SPI.h>

/* SETUP FOR CAN COMMUNICATION */
const int CS_PIN = 10; 
byte EXT   = 0;
byte LEN   = 8;
unsigned long int CAN_ID = 1;
MCP_CAN CAN(CS_PIN);

/* LIMIT VALUE OF AK70-10 MIT MODE */
#define P_MIN   -12.5f
#define P_MAX   12.5f
#define V_MIN   -50.0f
#define V_MAX   50.0f
#define KP_MIN  0.0f
#define KP_MAX  500.0f
#define KD_MIN  0.0f
#define KD_MAX  5.0f
#define T_MIN   -25.0f
#define T_MAX   25.0f
// IF YOU WANT TO CHANGE THE VALUE FOR OTHER MOTOR, PLEASE CHECK "CubeMars Manual" 44 PAGE.

/* VALUE FOR INPUT COMMAND FROM PYTHON */
String cmd;

/* VALUE FOR MOTOR INPUT & OUTPUT */
// MOTOR INPUT
float pos = 0.0f; 
float vel = 0.0f; 
float kp  = 0.0f; 
float kd  = 0.0f; 
float tor = 0.0f;

// MOTOR OUTPUT
float pos_out;
float vel_out;
float tor_out;

float torInit = 0.0f;
float zeroOffset = 0.0f;

/*********************************************************************************************************
  MAIN SECTION
*********************************************************************************************************/

void setup() {
  Serial.begin(115200);
  delay(1000);
  while (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) != CAN_OK) {
    delay(100);
  }
  CAN.setMode(MCP_NORMAL);
  delay(1000); 
}

void loop() {
  char buf[100];
  if (Serial.available() > 0) {
    cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "ON") {
      EnterMotorMode(CAN_ID);
      Serial.println("Motor Mode :: ON");
      return;
    }

    if (cmd == "OFF") {
      ExitMotorMode(CAN_ID);
      Serial.println("Motor Mode :: OFF");
      return;
    }

    if (cmd == "ORIGIN") {
      ZeroMode(CAN_ID);
      Serial.println("SET ORIGIN");
      return;
    }

    if (cmd.startsWith("ID:")) {
      int new_id = cmd.substring(3).toInt();
      CAN_ID = new_id;
      Serial.print("MOTOR ID :: "); Serial.println(CAN_ID);
      return;
    }

    if (cmd.startsWith("SEND POS:")) {
      pos = parseValue(cmd, "POS:");
      vel = parseValue(cmd, "VEL:");
      kp  = parseValue(cmd, "KP:");
      kd  = parseValue(cmd, "KD:");
      tor = parseValue(cmd, "TOR:");

    }
  }
  SendToMotor(CAN_ID, pos, vel, kp, kd, tor);
  delay(10);
  SendToArduino();   // TO GET OUTPUT VALUE FROM MOTOR IN REAL-TIME, MUST PUT THIS FUNCTION ON LOOP NOT SERIAL COMMUNICATION PART.
  delay(100);
}

/*********************************************************************************************************
  MOTOR MODE SECTION
*********************************************************************************************************/

// MOTOR TURN ON
void EnterMotorMode(int mot_id) {
  byte data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
  CAN.sendMsgBuf(mot_id, EXT, LEN, data);
}

// MOTOR TURN OFF
void ExitMotorMode(int mot_id) {
  byte data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
  CAN.sendMsgBuf(mot_id, EXT, LEN, data);
}

// MOTOR SET ORIGIN
void ZeroMode(int mot_id) {
  byte data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};
  CAN.sendMsgBuf(mot_id, EXT, LEN, data);
}

/*********************************************************************************************************
  CAN COMMUNICATION SECTION
*********************************************************************************************************/

// SEND POS, VEL, KP, KD, TOR TO MOTOR
void SendToMotor(int mot_id, float pos, float vel, float kp, float kd, float torq) {
  byte data[8];
  unsigned int con_pos = float_to_uint(constrain(pos, P_MIN, P_MAX), P_MIN, P_MAX, 16);
  unsigned int con_vel = float_to_uint(constrain(vel, V_MIN, V_MAX), V_MIN, V_MAX, 12);
  unsigned int con_kp  = float_to_uint(constrain(kp,  KP_MIN, KP_MAX), KP_MIN, KP_MAX, 12);
  unsigned int con_kd  = float_to_uint(constrain(kd,  KD_MIN, KD_MAX), KD_MIN, KD_MAX, 12);
  unsigned int con_torq= float_to_uint(constrain(torq,T_MIN, T_MAX), T_MIN, T_MAX, 12);
  data[0] = con_pos >> 8;
  data[1] = con_pos & 0xFF;
  data[2] = con_vel >> 4;
  data[3] = ((con_vel & 0xF) << 4) | (con_kp >> 8);
  data[4] = con_kp & 0xFF;
  data[5] = con_kd >> 4;
  data[6] = ((con_kd & 0xF) << 4) | (con_torq >> 8);
  data[7] = con_torq & 0xFF;
  CAN.sendMsgBuf(mot_id, EXT, LEN, data);
}

// RECEIVE POS_OUT, VEL_OUT, TOR_OUT FROM MOTOR
void SendToArduino() {
  byte dat[8];
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    CAN.readMsgBuf(&CAN_ID, &EXT, &LEN, dat); 
    pos_out = uint_to_float((dat[1] << 8) | dat[2], P_MIN, P_MAX, 16);
    vel_out = uint_to_float((dat[3] << 4) | (dat[4] >> 4), V_MIN, V_MAX, 12);
    tor_out = uint_to_float(((dat[4] & 0xF) << 8) | dat[5], T_MIN, T_MAX, 12);
    Serial.print("Position: ");
    Serial.print(pos_out);
    Serial.print(", Velocity: ");
    Serial.print(vel_out);
    Serial.print(", Torque: ");
    Serial.println(tor_out);
  } else {
    Serial.println("Cannot Receive from Motor...");
  } 
}

/*********************************************************************************************************
  RECEIVE COMMAND FROM PYTHON VIA SERIAL COMMUNICATION
*********************************************************************************************************/

float parseValue(String input, String key) {
  int startIndex = input.indexOf(key);
  if (startIndex == -1) return 0;
  int colonIndex = startIndex + key.length();
  int commaIndex = input.indexOf(",", colonIndex);
  if (commaIndex == -1) commaIndex = input.length();
  return input.substring(colonIndex, commaIndex).toFloat();
}

/*********************************************************************************************************
  CONVERSION SECTION
*********************************************************************************************************/

// CONVERT FLOAT TO UNSIGNED INT
unsigned int float_to_uint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  if(bits == 12) return (unsigned int)((x - offset) * 4095.0 / span);
  if(bits == 16) return (unsigned int)((x - offset) * 65535.0 / span);
  return 0;
}

// CONVERT UNSIGNED INT TO FLOAT
float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  if(bits == 12) return ((float)x_int) * span / 4095.0 + offset;
  if(bits == 16) return ((float)x_int) * span / 65535.0 + offset;
  return 0;
}

/*********************************************************************************************************
  END OF THE CODE & FILE
*********************************************************************************************************/