/*********************************************************************************************************
  START OF THE CODE
*********************************************************************************************************/

#include <SPI.h>
#include <mcp_can.h>

/* ===== CAN setup ===== */
const int CS_PIN = 10;
MCP_CAN CAN(CS_PIN);
const byte EXT = 0;   // standard 11-bit
const byte LEN = 8;   // MIT frame 8 bytes

/* ===== MIT LIMITS (AK70-10) ===== */
#define P_MIN   -12.5f
#define P_MAX    12.5f
#define V_MIN   -50.0f
#define V_MAX    50.0f
#define KP_MIN    0.0f
#define KP_MAX  500.0f
#define KD_MIN    0.0f
#define KD_MAX    5.0f
#define T_MIN   -25.0f
#define T_MAX    25.0f

/* ===== ID list & per-ID command/state ===== */
#define MAX_IDS 8
int ids[MAX_IDS];             // Motor ID Index
int num_ids = 0;
int currentID = -1;           // Current Motor ID for Command

// per-ID 원하는 setpoint (PC에서 SEND POS:...로 갱신)
float posCmd[MAX_IDS] = {0};
float velCmd[MAX_IDS] = {0};
float kpCmd [MAX_IDS] = {0};
float kdCmd [MAX_IDS] = {0};
float torCmd[MAX_IDS] = {0};

// per-ID 최근 상태(수신) 저장
float pLast[MAX_IDS] = {0};
float vLast[MAX_IDS] = {0};
float tLast[MAX_IDS] = {0};

/* ===== Timers (us) =====
   CTRL_US: 라운드로빈으로 한 번에 "한 모터"에만 제어 프레임 송신.
            따라서 모터당 제어주파수 ≈ (1e6 / CTRL_US) / num_ids
   LOG_US : 한 줄/샘플 출력 주기(모든 모터 상태)
*/
const uint32_t CTRL_US = 2000;   // 총 500 Hz → 3모터면 모터당 ≈ 166 Hz
const uint32_t LOG_US  = 10000;  // 100 Hz 한 줄 출력 (필요시 20ms=50Hz로 낮추세요)
uint32_t tCtrl = 0, tLog = 0;
int rr = 0; // round-robin index

/* ===== UART command buffer ===== */
String cmd;

/*********************************************************************************************************
  MAIN SECTION
*********************************************************************************************************/

void setup() {
  /* ===== Begin Serial Communication ===== */
  Serial.begin(115200);
  delay(200);
  /* ===== Begin CAN Communication ===== */
  while (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) != CAN_OK) {
    delay(100);
  }
  CAN.setMode(MCP_NORMAL);
  delay(200);
  /* ===== Motor ID Initialization ===== */
  num_ids = 0;
  currentID = -1;
}

void loop() {
  // 1) PC 명령 비차단 처리
  if (Serial.available() > 0) {
    cmd = Serial.readStringUntil('\n');  // line-based
    cmd.trim();
    // 대소문자 혼합 명령을 위해 원본 유지, 키워드 비교는 대문자 복사로 처리
    parseCommand(cmd);
  }

  uint32_t now = micros();

  // 2) 라운드로빈 제어 송신
  if (num_ids > 0 && (now - tCtrl >= CTRL_US)) {
    tCtrl = now;
    int mid = ids[rr];
    int k = indexOfId(mid);
    if (k >= 0) {
      // setpoint는 per-ID 배열에서 꺼냄
      float p = posCmd[k], v = velCmd[k], kp = kpCmd[k], kd = kdCmd[k], tq = torCmd[k];
      SendToMotor(mid, p, v, kp, kd, tq);
    }
    rr = (rr + 1) % num_ids;
  }

  // 3) CAN 수신 프레임 싹 비우고 최신값 반영
  drainCanAndUpdate();

  // 4) 한 줄/샘플 출력
  if (now - tLog >= LOG_US) {
    tLog = now;
    if (num_ids > 0) {
      Serial.print("Time:"); Serial.print(millis()); // ms 기준
      for (int i = 0; i < num_ids; i++) {
        int mid = ids[i];
        int k = indexOfId(mid);
        Serial.print(", "); Serial.print(mid); Serial.print(":");
        if (k >= 0) {
          Serial.print(pLast[k], 3); Serial.print(",");
          Serial.print(vLast[k], 3); Serial.print(",");
          Serial.print(tLast[k], 3);
        } else {
          Serial.print("nan,nan,nan");
        }
      }
      Serial.println();
    }
  }
}

/*********************************************************************************************************
  COMMAND PARASE SECTION
*********************************************************************************************************/

void parseCommand(const String& sOriginal) {
  String s = sOriginal;
  String up = sOriginal; up.toUpperCase();

  // IDS:1,2,3
  if (up.startsWith("IDS:")) {
    setIds(s);                  // 원본 문자열로 파싱(숫자)
    return;
  }

  // ID: <n> (현재 선택 ID)
  if (up.startsWith("ID:")) {
    int sel = s.substring(3).toInt();
    // 리스트에 없으면 자동 추가(최대 MAX_IDS)
    int idx = indexOfId(sel);
    if (idx < 0 && num_ids < MAX_IDS) {
      ids[num_ids++] = sel;
      idx = num_ids - 1;
      // 기본 setpoint 0으로 초기화
      posCmd[idx] = velCmd[idx] = kpCmd[idx] = kdCmd[idx] = torCmd[idx] = 0.0f;
    }
    currentID = sel;
    Serial.print("MOTOR ID :: "); Serial.println(currentID);
    return;
  }

  // ON / OFF / ORIGIN / ON ALL / OFF ALL / ORIGIN ALL
  if (up == "ON ALL") {
    for (int i = 0; i < num_ids; i++) EnterMotorMode(ids[i]);
    Serial.println("Motor Mode :: ON ALL");
    return;
  }
  if (up == "OFF ALL") {
    for (int i = 0; i < num_ids; i++) ExitMotorMode(ids[i]);
    Serial.println("Motor Mode :: OFF ALL");
    return;
  }
  if (up == "ORIGIN ALL") {
    for (int i = 0; i < num_ids; i++) ZeroMode(ids[i]);
    Serial.println("SET ORIGIN ALL");
    return;
  }

  if (up == "ON") {
    if (currentID >= 0) { EnterMotorMode(currentID); Serial.println("Motor Mode :: ON"); }
    return;
  }
  if (up == "OFF") {
    if (currentID >= 0) { ExitMotorMode(currentID); Serial.println("Motor Mode :: OFF"); }
    return;
  }
  if (up == "ORIGIN") {
    if (currentID >= 0) { ZeroMode(currentID); Serial.println("SET ORIGIN"); }
    return;
  }

  // SEND POS:...,VEL:...,KP:...,KD:...,TOR:...
  if (up.startsWith("SEND POS:")) {
    if (currentID < 0) return;
    int k = indexOfId(currentID);
    if (k < 0) return;

    float pos = parseValue(up, "POS:");
    float vel = parseValue(up, "VEL:");
    float kp  = parseValue(up, "KP:");
    float kd  = parseValue(up, "KD:");
    float tor = parseValue(up, "TOR:");

    // NaN 방지: parseValue 실패시 0 반환 → 그대로 사용(원하시면 유지정책으로 바꿔도 됨)
  if (hasKey(up, "POS:")) posCmd[k] = pos;
  if (hasKey(up, "VEL:")) velCmd[k] = vel;
  if (hasKey(up, "KP:"))  kpCmd[k]  = kp;
  if (hasKey(up, "KD:"))  kdCmd[k]  = kd;
  if (hasKey(up, "TOR:")) torCmd[k] = tor;
    // 즉시 송신을 원하면 여기서 SendToMotor(currentID, ...)를 한 번 더 호출해도 됨.
    return;
  }

  if (up == "GET") {
    if (currentID >= 0) {
      int k = indexOfId(currentID);
      if (k >= 0) {
        Serial.print("Position:");  Serial.print(pLast[k], 3);
        Serial.print(",Velocity:"); Serial.print(vLast[k], 3);
        Serial.print(",Torque:");   Serial.println(tLast[k], 3);
      }
    }
    return;
  }
}

/* ====== IDS list parsing ====== */
void setIds(String s) {
  // s: "IDS:1,2,3"
  s.trim();
  // 대문자/소문자 혼용 허용
  int colon = s.indexOf(':');
  if (colon < 0) return;
  String list = s.substring(colon + 1);
  list.trim();

  // reset
  num_ids = 0;
  currentID = -1;

  while (list.length() && num_ids < MAX_IDS) {
    int p = list.indexOf(',');
    String tok = (p == -1) ? list : list.substring(0, p);
    tok.trim();
    int val = tok.toInt();
    if (val > 0) {
      ids[num_ids] = val;
      // 기본 setpoint 0
      posCmd[num_ids] = velCmd[num_ids] = kpCmd[num_ids] = kdCmd[num_ids] = torCmd[num_ids] = 0.0f;
      // 최근 상태도 0
      pLast[num_ids]  = vLast[num_ids]  = tLast[num_ids]  = 0.0f;
      num_ids++;
    }
    if (p == -1) break;
    list = list.substring(p + 1);
  }

  if (num_ids > 0) currentID = ids[0];

  Serial.print("IDS SET :: ");
  for (int i = 0; i < num_ids; i++) {
    Serial.print(ids[i]);
    if (i != num_ids - 1) Serial.print(",");
  }
  Serial.println();
}

int indexOfId(int mot_id) {
  for (int i = 0; i < num_ids; i++) {
    if (ids[i] == mot_id) return i;
  }
  return -1;
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

// Read CAN Receive Frame to Update Table
void drainCanAndUpdate() {
  byte dat[8];
  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    unsigned long rxId; byte rxExt; byte rxLen;
    CAN.readMsgBuf(&rxId, &rxExt, &rxLen, dat);  // 지역 변수 사용! 전역 덮어쓰기 금지

    int k = indexOfId((int)rxId);
    if (k < 0) continue;

    float pos_out = uint_to_float((dat[1] << 8) | dat[2], P_MIN, P_MAX, 16);
    float vel_out = uint_to_float((dat[3] << 4) | (dat[4] >> 4), V_MIN, V_MAX, 12);
    float tor_out = uint_to_float(((dat[4] & 0xF) << 8) | dat[5], T_MIN, T_MAX, 12);

    pLast[k] = pos_out;
    vLast[k] = vel_out;
    tLast[k] = tor_out;
  }
}

/* ===================== Helpers ===================== */
float parseValue(const String& input, const String& key) {
  int startIndex = input.indexOf(key);
  if (startIndex == -1) return 0.0f;
  int begin = startIndex + key.length();
  int comma = input.indexOf(",", begin);
  if (comma == -1) comma = input.length();
  String ss = input.substring(begin, comma);
  ss.trim();
  return (float) ss.toFloat();
}

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

bool hasKey(const String& up, const String& key) { 
  return up.indexOf(key) >= 0; 
  }

/*********************************************************************************************************
  END OF THE CODE & FILE
*********************************************************************************************************/
