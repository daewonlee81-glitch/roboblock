/*
 * ESP32 (NodeMCU-32S) + Bluepad32 + Xbox Controller + 로보블록(웹 블루투스)
 * 4WD 축구로봇 — DRV8871 단채널 드라이버 x 4 (모터당 1개)
 *
 * ── 두 가지 모드 (동시에 쓰지 않음) ──
 *   조종 모드 : 지금처럼 Xbox 컨트롤러로 조종. 웹 에디터가 연결되지 않은 기본 상태.
 *   블록 모드 : 로보블록 웹 에디터가 Web Bluetooth로 연결되면 들어간다.
 *               연결 순간 Xbox 컨트롤러를 끊고 새 연결도 막는다. 로봇은 블록 명령으로만 움직인다.
 *               에디터에서 연결을 끊으면 다시 조종 모드로 돌아가 Xbox 컨트롤러가 재연결된다.
 *               블록 모드에서 BOOT 버튼을 짧게 누르면 블록의 "로봇 버튼을 누르면"이 실행된다.
 *   안전: 블록 모드에서 0.5초 동안 명령이 없으면(브라우저 닫힘·전파 끊김) 즉시 능동 제동.
 *
 * 파일: soccer_roboblock.ino (이 파일), RoboLink.h/.cpp (BLE GATT 서버), roboblock_gatt.h (GATT 표)
 *   소프트웨어에서 좌/우 2채널로 묶어 제어 (스키드 스티어)
 * 모터: JGA25-370 12V 1200rpm x 4
 * 전원: DXF 3S 리포 11.1V 650mAh 100C (XT60) → 보드 DC 잭 + Y분기로 4개 드라이버 VM
 *
 * ── DRV8871을 쓰는 이유 (축구로봇 = 반복 스톨) ──
 *   - 모터당 독립 드라이버, 피크 3.6A, 낮은 RDS(on) → TB6612 대비 발열 적음
 *   - 과전류/과열/저전압 보호 내장, 전원 6.5~45V (12V 여유 충분)
 *   - (HW-686형 보드는 ILIM·47µF 캡 내장 고정. 헤더는 IN2/IN1/VM/GND 4핀)
 *
 * ── 제어 방식 (TB6612와 다름!) ──
 *   DRV8871은 STBY/별도 PWM핀이 없고, 입력 2핀(IN1/IN2)에 PWM을 직접 싣는다.
 *     정방향 : IN1=PWM, IN2=0
 *     역방향 : IN1=0,   IN2=PWM
 *     제동   : IN1=1,   IN2=1   (양 출력 GND 단락 = 능동 제동)
 *     코스트 : IN1=0,   IN2=0   (Hi-Z, 자유 회전)
 *   → 모터 1개당 PWM 가능 핀 2개가 필요 (LEDC 채널 8개 사용).
 *
 * 안전: 리포는 셀당 3.0V (총 9.0V) 미만 방전 금지. 저전압 알람 부저 필수.
 *        DRV8871 IN핀은 부팅 중 플로팅 방지를 위해 각 핀에 10k 풀다운(→GND) 권장.
 *
 * 보드 패키지: ESP32 Bluepad32 → NodeMCU-32S
 *   File → Preferences → Additional Board URLs:
 *   https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json
 */

#include <Bluepad32.h>
#include <Preferences.h>        // 페어링 MAC 영구 저장 (NVS)
#include <string.h>             // memcmp
#include "RoboLink.h"           // 로보블록 웹 에디터 연결 (BLE)
#include "soc/soc.h"            // 브라운아웃 디텍터 비활성화용
#include "soc/rtc_cntl_reg.h"

// ===== 자가 진단 모드 =====
// 1로 바꿔서 업로드하면 부팅 직후 4개 모터를 하나씩 정·역 회전시킵니다.
// 시리얼 모니터(115200)를 보면 어느 모터가 도는지 확인할 수 있어요.
// 진단이 끝나면 다시 0으로 되돌려 업로드.
#define SELFTEST 0

// ===== 페어링 초기화 =====
// 1로 바꿔서 업로드하면 이 로봇에 저장된 컨트롤러 MAC을 지웁니다.
// 그 다음 다시 0으로 되돌려 업로드하면 다음 연결 첫 컨트롤러가 자동 등록됩니다.
// (다른 로봇과 조종기를 다시 짝지을 때만 사용)
#define FORGET_PAIRING 0

// ===== BOOT 버튼으로 현장 페어링 초기화 =====
// 운행 중 ESP32의 BOOT 버튼(GPIO0)을 3초 이상 누르면
//   - 저장된 페어링이 지워지고
//   - Bluepad32 BT 키 캐시도 비우고
//   - 자동으로 재부팅하여 새 컨트롤러를 받을 준비 상태가 됨.
// FORGET_PAIRING 매크로처럼 코드 수정·재업로드 없이 현장에서 즉시 가능.
#define BOOT_BTN_PIN      0
#define FORGET_HOLD_MS    3000

// ===== OLED 0.96" SSD1306 (I2C) 디스플레이 =====
// Library Manager에서 "Adafruit SSD1306" 설치 (Adafruit GFX도 같이 깔립니다)
// 배선: VCC→3.3V, GND→GND, SDA→P16, SCL→P17
#define USE_TFT 1
#if USE_TFT
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  #define OLED_SDA   16
  #define OLED_SCL   17
  #define OLED_W    128
  #define OLED_H     64
  #define OLED_ADDR 0x3C   // 화면이 안 뜨면 0x3D로 바꿔 시도
  Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1);
  int lastSpdPct = 0;
  const char* lastDirLabel = "STOP";
#endif

// ===== 핀 배치 (DRV8871 x4, 모터당 IN1/IN2 2핀, STBY 없음) =====
// 모든 핀은 LEDC PWM 채널에 연결한다 (방향 전환 시 PWM을 싣는 핀이 바뀌므로).
//
// 부팅 중 모터 오동작 방지: 아래 8개 핀에 각각 10kΩ 풀다운(→GND)을 달아두면
//   ESP32 리셋 동안 IN1=IN2=0(코스트)으로 안전하게 떨어진다.
const int LF_IN1 = 32;       // Left Front  (DRV8871 #1)
const int LF_IN2 = 33;
const int LR_IN1 = 25;       // Left Rear   (DRV8871 #2)
const int LR_IN2 = 26;
const int RF_IN1 = 27;       // Right Front (DRV8871 #3)
const int RF_IN2 = 4;        // ← GPIO14는 JTAG(MTMS) 핀이라 부팅 시 잠깐 HIGH로 떠
                             //   모터가 역방향으로 휘청거리는 문제가 있어 GPIO4로 이동.
                             //   #3 보드의 IN2 점퍼를 ESP32 P14 → P4로 옮길 것.
const int RR_IN1 = 22;       // Right Rear  (DRV8871 #4)
const int RR_IN2 = 21;

// ===== PWM (LEDC) 채널 — 모터당 2채널, 총 8채널 =====
const int PWM_FREQ = 20000;  // 20kHz: 가청 소음 없음, DRV8871이 충분히 지원
const int PWM_RES  = 8;      // 0~255
const int CH_LF1 = 0, CH_LF2 = 1;
const int CH_LR1 = 2, CH_LR2 = 3;
const int CH_RF1 = 4, CH_RF2 = 5;
const int CH_RR1 = 6, CH_RR2 = 7;

// 11.1V (3S 리포) → 12V 모터. 스톨 전류는 DRV8871 ILIM 저항이 하드웨어로 제한하므로
// 소프트웨어 PWM은 풀(255)까지 써도 안전하다. 너무 빠르면 MAX_PWM을 낮춘다.
const int MAX_PWM   = 255;       // RT 풀로 당길 때: 풀 파워 (ILIM이 전류 상한 보호)
const int BASE_PCT  = 70;        // D-pad 기본 속도 = MAX_PWM의 70% (RT 안 당겼을 때)
                                 //   값을 키우면 평소 빠름, 줄이면 평소 느림
const int TRIG_DEAD = 30;        // 트리거 데드존 (0~1023)
const int TURN_RATIO_PCT = 90;   // 제자리 회전 속도 = 직진의 90%
const int INNER_RATIO_PCT = 50;  // 대각선 완만한 곡선: 안쪽 바퀴 = 바깥쪽의 50%

// ===== 자동 페어링 (NVS 기반) =====
// 첫 부팅에서 처음으로 연결되는 컨트롤러 MAC을 ESP32 내부 저장소(NVS)에 기록하고,
// 이후 부팅부터는 그 컨트롤러만 받아들이고 다른 컨트롤러는 즉시 차단한다.
// → 여러 로봇·조종기를 동시에 켜도 서로 간섭 없이 1:1 매칭이 유지됨.
// 다시 짝지으려면 FORGET_PAIRING을 1로 잠깐 업로드 후 0으로 되돌려 업로드.

Preferences prefs;
uint8_t pairedMAC[6] = {0,0,0,0,0,0};
bool hasPaired = false;

static bool macIsZero(const uint8_t* m) {
  uint8_t z[6] = {0,0,0,0,0,0};
  return memcmp(m, z, 6) == 0;
}

void loadPairedMAC() {
  prefs.begin("x4wd", true);   // 읽기 전용
  if (prefs.getBytesLength("mac") == 6) {
    prefs.getBytes("mac", pairedMAC, 6);
    hasPaired = !macIsZero(pairedMAC);
  }
  prefs.end();
  if (hasPaired) {
    Serial.printf("이 로봇 전용 컨트롤러 MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
      pairedMAC[0],pairedMAC[1],pairedMAC[2],pairedMAC[3],pairedMAC[4],pairedMAC[5]);
  } else {
    Serial.println("페어링 없음. 첫 연결 컨트롤러를 이 로봇 전용으로 자동 등록합니다.");
  }
}

void savePairedMAC(const uint8_t* mac) {
  prefs.begin("x4wd", false);  // 쓰기
  prefs.putBytes("mac", mac, 6);
  prefs.end();
  memcpy(pairedMAC, mac, 6);
  hasPaired = true;
}

void clearPairedMAC() {
  prefs.begin("x4wd", false);
  prefs.remove("mac");
  prefs.end();
  memset(pairedMAC, 0, 6);
  hasPaired = false;
}

ControllerPtr myCtl = nullptr;

// ===== 로보블록 상태 =====
// 바퀴 순서: 0=FL(앞-좌) 1=FR(앞-우) 2=BL(뒤-좌) 3=BR(뒤-우) — 에디터와 같은 순서
int8_t  motorPct[4] = {0, 0, 0, 0};     // 지금 실제로 내보내는 출력 (−100~100 %)
uint8_t brakeMask   = 0x0F;              // 비트가 1이면 그 바퀴는 능동 제동
const int CH_OF[4][2] = { {0, 1}, {4, 5}, {2, 3}, {6, 7} };   // 바퀴별 (IN1 채널, IN2 채널)
const int PIN_OF[4][2] = { {32, 33}, {27, 4}, {25, 26}, {22, 21} };
bool webMode = false;                    // 블록 모드(웹 에디터 연결)인가
unsigned long lastWebCmdMs = 0;          // 마지막 웹 명령 시각 (안전 정지용)
const unsigned long WEB_TIMEOUT_MS = 500;
bool webTimedOut = false;
char webText[19] = "";                   // 블록의 "화면에 표시" 내용 (ASCII)
uint16_t statusSeq = 0;
bool bootShortPress = false;             // BOOT 짧게 누름 (블록의 "로봇 버튼을 누르면")

int chIndex(int chF) {                   // IN1 채널 번호 → 바퀴 번호
  for (int i = 0; i < 4; i++) if (CH_OF[i][0] == chF) return i;
  return -1;
}

// ---------------------------------------------------------------
// 단일 모터 구동 (s: -255 ~ +255, 부호로 방향)
//   chF = IN1 채널, chR = IN2 채널
//   s == 0 일 때는 IN1=IN2=HIGH → 능동 제동(코스트 없이 즉시 정지)
void driveMotor(int chF, int chR, int s) {
  s = constrain(s, -255, 255);
  int idx = chIndex(chF);
  if (idx >= 0) {
    motorPct[idx] = (int8_t)((s * 100) / 255);
    if (s == 0) brakeMask |= (1 << idx); else brakeMask &= ~(1 << idx);
  }
  if (s > 0) {                          // 정방향: IN1=PWM, IN2=0
    ledcWrite(chF, s);
    ledcWrite(chR, 0);
  } else if (s < 0) {                   // 역방향: IN1=0, IN2=PWM
    ledcWrite(chF, 0);
    ledcWrite(chR, -s);
  } else {                              // 제동: 양 입력 HIGH (양 출력 GND 단락)
    ledcWrite(chF, 255);
    ledcWrite(chR, 255);
  }
}

// 한 모터를 능동 제동으로 즉시 정지
void brakeMotor(int chF, int chR) {
  int idx = chIndex(chF);
  if (idx >= 0) { motorPct[idx] = 0; brakeMask |= (1 << idx); }
  ledcWrite(chF, 255);
  ledcWrite(chR, 255);
}

// 좌측 두 모터를 동일하게 구동
void setLeft(int s) {
  driveMotor(CH_LF1, CH_LF2, s);
  driveMotor(CH_LR1, CH_LR2, s);
}

// 우측 두 모터를 동일하게 구동
void setRight(int s) {
  driveMotor(CH_RF1, CH_RF2, s);
  driveMotor(CH_RR1, CH_RR2, s);
}

// 네 모터 모두 즉시 능동 제동
void brakeAll() {
  brakeMotor(CH_LF1, CH_LF2);
  brakeMotor(CH_LR1, CH_LR2);
  brakeMotor(CH_RF1, CH_RF2);
  brakeMotor(CH_RR1, CH_RR2);
}

// 호환용 — 정지 = 능동 제동
void stopAll() { brakeAll(); }

// ---------------------------------------------------------------
// BOOT 버튼 길게 누르면 페어링 초기화 + 재부팅.
// loop()에서 매 사이클 호출. 상태는 정적 변수로 추적.
void checkBootButton() {
  static unsigned long pressStart = 0;
  static bool wasPressed = false;
  static bool triggered = false;

  bool pressed = (digitalRead(BOOT_BTN_PIN) == LOW);

  if (pressed && !wasPressed) {
    pressStart = millis();
    wasPressed = true;
    triggered = false;
    Serial.println("BOOT 버튼 눌림. 3초 이상 유지하면 페어링 초기화.");
  } else if (!pressed && wasPressed) {
    if (!triggered) {
      if (millis() - pressStart < 1000) bootShortPress = true;   // 짧게 누름
      else Serial.println(" → 일찍 떼서 취소.");
    }
    wasPressed = false;
  } else if (pressed && wasPressed && !triggered &&
             (millis() - pressStart >= FORGET_HOLD_MS)) {
    triggered = true;
    Serial.println("\n*** 페어링 초기화 → 재부팅 ***");

    brakeAll();
#if USE_TFT
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(2);
    oled.setCursor(8, 14);
    oled.print("PAIRING");
    oled.setCursor(22, 36);
    oled.print("RESET");
    oled.display();
#endif

    clearPairedMAC();
    BP32.forgetBluetoothKeys();
    delay(1500);
    ESP.restart();
  }
}

// 현재 BOOT 버튼 보유 시간(ms) — OLED 카운트다운 표시용
unsigned long getBootHoldMs() {
  static unsigned long pressStart = 0;
  static bool wasPressed = false;
  bool pressed = (digitalRead(BOOT_BTN_PIN) == LOW);
  if (pressed && !wasPressed) { pressStart = millis(); wasPressed = true; }
  else if (!pressed) { wasPressed = false; return 0; }
  return wasPressed ? (millis() - pressStart) : 0;
}

// ---------------------------------------------------------------
// OLED — 로봇 상태 표시
#if USE_TFT
void setupTFT() {
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 초기화 실패. I2C 주소(0x3C/0x3D)·배선 확인.");
    return;
  }
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(2);
  oled.setCursor(4, 4);
  oled.print("4WD ROBOT");
  oled.drawLine(0, 22, OLED_W - 1, 22, SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 28);
  oled.print("Booting...");
  oled.display();
  Serial.println("OLED 초기화 완료.");
}

// 부분 갱신 없이 매번 통째 redraw (OLED는 display() 호출 전엔 깜빡임 없음)
void updateTFT() {
  static unsigned long last = 0;
  if (millis() - last < 200) return;
  last = millis();

  oled.clearDisplay();

  // 타이틀
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(2);
  oled.setCursor(4, 0);
  oled.print("4WD ROBOT");
  oled.drawLine(0, 18, OLED_W - 1, 18, SSD1306_WHITE);

  // BOOT 버튼이 눌리고 있으면 카운트다운만 큼지막하게 표시하고 끝
  unsigned long btnHold = getBootHoldMs();
  if (btnHold > 200) {
    long remain = (long)FORGET_HOLD_MS - (long)btnHold;
    if (remain < 0) remain = 0;
    int secLeft = (remain + 999) / 1000;
    oled.setTextSize(1);
    oled.setCursor(0, 24);
    oled.print("FORGET PAIRING");
    oled.setTextSize(2);
    oled.setCursor(28, 36);
    oled.printf("%d sec", secLeft);
    oled.display();
    return;
  }

  // 정보 영역
  oled.setTextSize(1);

  if (webMode) {                       // 블록 모드 화면
    oled.setCursor(0, 22);
    oled.print("BLOCK MODE  WEB: OK");
    oled.setCursor(0, 32);
    oled.printf("FL%+4d  FR%+4d", motorPct[0], motorPct[1]);
    oled.setCursor(0, 42);
    oled.printf("BL%+4d  BR%+4d", motorPct[2], motorPct[3]);
    oled.setCursor(0, 52);
    if (webTimedOut) oled.print("SIGNAL LOST - BRAKE");
    else oled.printf("> %s", webText);
    oled.display();
    return;
  }

  // BT 연결 상태
  oled.setCursor(0, 22);
  oled.print("BT : ");
  oled.println(myCtl && myCtl->isConnected() ? "OK" : "--");

  // 페어링 MAC (끝 4바이트)
  oled.setCursor(0, 32);
  if (hasPaired) {
    oled.printf("MAC: %02X:%02X:%02X:%02X\n",
      pairedMAC[2], pairedMAC[3], pairedMAC[4], pairedMAC[5]);
  } else {
    oled.println("MAC: --");
  }

  // 속도
  oled.setCursor(0, 42);
  oled.printf("Spd: %3d%%\n", lastSpdPct);

  // 방향
  oled.setCursor(0, 52);
  oled.printf("Dir: %s\n", lastDirLabel);

  oled.display();
}
#endif

// ---------------------------------------------------------------
// 자가 진단: 4개 모터를 하나씩 정·역 회전시키며 시리얼에 이름 출력.
// SELFTEST=1 일 때 setup()에서 1회 호출.
void runSelfTest() {
  const int TEST_PWM = 150;
  struct M { const char* name; int chF; int chR; };
  M tests[] = {
    {"LF (앞-좌)", CH_LF1, CH_LF2},
    {"LR (뒤-좌)", CH_LR1, CH_LR2},
    {"RF (앞-우)", CH_RF1, CH_RF2},
    {"RR (뒤-우)", CH_RR1, CH_RR2},
  };
  Serial.println("\n===== 자가 진단 시작 =====");
  for (auto& t : tests) {
    Serial.printf("[%s] 정방향 1초\n", t.name);
    driveMotor(t.chF, t.chR,  TEST_PWM); delay(1000);
    brakeMotor(t.chF, t.chR);            delay(300);
    Serial.printf("[%s] 역방향 1초\n", t.name);
    driveMotor(t.chF, t.chR, -TEST_PWM); delay(1000);
    brakeMotor(t.chF, t.chR);            delay(600);
  }
  Serial.println("===== 자가 진단 완료 =====\n");
}

// ---------------------------------------------------------------
void onConnect(ControllerPtr ctl) {
  if (webMode) {                       // 블록 모드에서는 컨트롤러를 받지 않는다
    Serial.println("블록 모드라 컨트롤러 연결 거부.");
    ctl->disconnect();
    return;
  }
  ControllerProperties p = ctl->getProperties();
  Serial.printf("Controller MAC: %02X:%02X:%02X:%02X:%02X:%02X  (%s)\n",
                p.btaddr[0], p.btaddr[1], p.btaddr[2],
                p.btaddr[3], p.btaddr[4], p.btaddr[5],
                ctl->getModelName().c_str());

  if (hasPaired) {
    // 이미 이 로봇 전용 컨트롤러가 등록되어 있음 — 다른 MAC은 즉시 차단
    if (memcmp(p.btaddr, pairedMAC, 6) != 0) {
      Serial.println(" → 다른 컨트롤러. 이 로봇 전용이 아님. 연결 거부.");
      ctl->disconnect();
      brakeAll();
      return;
    }
  } else {
    // 첫 페어링 — 이 컨트롤러를 이 로봇 전용으로 저장
    savePairedMAC(p.btaddr);
    Serial.println(" → 첫 페어링. 이 컨트롤러를 이 로봇 전용으로 저장했습니다.");
  }

  // 이미 다른 컨트롤러가 잡혀 있으면 추가 연결은 거부 (중복 방지)
  if (myCtl != nullptr && myCtl != ctl) {
    Serial.println(" → 이미 사용 중. 추가 연결 거부.");
    ctl->disconnect();
    return;
  }

  Serial.println(" → 연결 승인");
  myCtl = ctl;
  ctl->setColorLED(0, 128, 255);   // 지원 모델만
}

void onDisconnect(ControllerPtr ctl) {
  Serial.println("Controller disconnected");
  if (myCtl == ctl) myCtl = nullptr;
  brakeAll();                    // 연결 끊기면 즉시 능동 제동
}

// ---------------------------------------------------------------
// RT(우 트리거, 0~1023) = 부스터.
//   D-pad만 누름        → BASE_PCT(예: 70%) 속도
//   D-pad + RT 풀 당김  → 100% (MAX_PWM)
//   중간은 선형 보간.
int throttleToPwm(ControllerPtr ctl) {
  int base = (MAX_PWM * BASE_PCT) / 100;
  int t = ctl->throttle();                            // 0~1023
  if (t < TRIG_DEAD) return base;                     // RT 안 당기면 기본 속도만
  int boost = map(t, TRIG_DEAD, 1023, 0, MAX_PWM - base);
  return constrain(base + boost, base, MAX_PWM);
}

// ---------------------------------------------------------------
void handleInput(ControllerPtr ctl) {
  // === 왼쪽 트리거(LT) = 급정거 ===
  // LT가 데드존 이상이면 RT·D-pad 무시하고 즉시 능동 제동.
  if (ctl->brake() >= TRIG_DEAD) {
    brakeAll();
#if USE_TFT
    lastDirLabel = "BRAKE";
    lastSpdPct = 0;
#endif
    return;
  }

  uint8_t d = ctl->dpad();
  bool up    = d & DPAD_UP;
  bool down  = d & DPAD_DOWN;
  bool left  = d & DPAD_LEFT;
  bool right = d & DPAD_RIGHT;

  int spd  = throttleToPwm(ctl);                 // BASE_PCT 기본 + RT 부스터
  int turn = (spd * TURN_RATIO_PCT)  / 100;       // 제자리 회전 속도
  int inner= (spd * INNER_RATIO_PCT) / 100;       // 대각선 안쪽 바퀴 속도 (완만한 곡선)

  // D-pad를 안 누르면 즉시 제동 (RT만 당겨도 D-pad가 없으면 정지)
  if (!up && !down && !left && !right) {
    brakeAll();
#if USE_TFT
    lastDirLabel = "STOP";
    lastSpdPct = 0;
#endif
    return;
  }

#if USE_TFT
  lastSpdPct = (spd * 100) / 255;
  if      (up && left)    lastDirLabel = "FL";
  else if (up && right)   lastDirLabel = "FR";
  else if (down && left)  lastDirLabel = "BL";
  else if (down && right) lastDirLabel = "BR";
  else if (up)            lastDirLabel = "FWD";
  else if (down)          lastDirLabel = "BWD";
  else if (left)          lastDirLabel = "LEFT";
  else if (right)         lastDirLabel = "RIGHT";
#endif

  if      (up && left)   { setLeft(inner);     setRight(spd);    }   // ↖ 완만한 좌-전진
  else if (up && right)  { setLeft(spd);       setRight(inner);  }   // ↗ 완만한 우-전진
  else if (down && left) { setLeft(-inner);    setRight(-spd);   }   // ↙ 완만한 좌-후진
  else if (down && right){ setLeft(-spd);      setRight(-inner); }   // ↘ 완만한 우-후진
  else if (up)           { setLeft(spd);       setRight(spd);    }   // ↑ 직진
  else if (down)         { setLeft(-spd);      setRight(-spd);   }   // ↓ 후진
  else if (left)         { setLeft(-turn);     setRight(turn);   }   // ← 제자리 좌회전
  else if (right)        { setLeft(turn);      setRight(-turn);  }   // → 제자리 우회전
}

// ---------------------------------------------------------------
// 로보블록 — 모드 전환, 명령 처리, 상태 전송

void enterWebMode() {
  webMode = true;
  brakeAll();
  BP32.enableNewBluetoothConnections(false);   // 새 컨트롤러 연결 막기
  if (myCtl) myCtl->disconnect();              // 연결된 컨트롤러 끊기
  lastWebCmdMs = millis();
  webTimedOut = false;
  webText[0] = 0;
  Serial.println("웹 에디터 연결 → 블록 모드 (Xbox 컨트롤러 끊음)");
}

void leaveWebMode() {
  webMode = false;
  brakeAll();
  BP32.enableNewBluetoothConnections(true);    // 컨트롤러 다시 받기
  Serial.println("웹 에디터 연결 해제 → 조종 모드 (Xbox 컨트롤러를 켜세요)");
}

// 명령 형식 (첫 바이트 = 종류)
//   0x01 FL FR BL BR : 바퀴 4개 출력 (int8, −100~100 %). 0이면 그 바퀴는 능동 제동
//   0x02             : 모두 능동 제동
//   0x03             : 살아 있음 (안전 정지 타이머만 갱신)
//   0x04 글자...      : OLED 마지막 줄에 표시 (ASCII, 최대 18자)
//   0x05 핀 값        : 디지털 출력 (모터·OLED·BOOT·플래시·입력 전용 핀은 거부)
//   0x06 바퀴 값      : 바퀴 하나만 (0~3, int8 %)
bool pinAllowed(int p) {
  if (p < 0 || p > 39) return false;
  if (p >= 6 && p <= 11) return false;           // 플래시
  if (p >= 34) return false;                     // 입력 전용
  if (p == 0 || p == 1 || p == 3) return false;  // BOOT, 시리얼
  if (p == OLED_SDA || p == OLED_SCL) return false;
  for (int i = 0; i < 4; i++) if (PIN_OF[i][0] == p || PIN_OF[i][1] == p) return false;
  return true;
}

void setWheel(int idx, int pct) {
  pct = constrain(pct, -100, 100);
  int s = (pct * MAX_PWM) / 100;
  driveMotor(CH_OF[idx][0], CH_OF[idx][1], s);
}

void handleWebCommand(const uint8_t* d, uint8_t n) {
  lastWebCmdMs = millis();
  webTimedOut = false;
  switch (d[0]) {
    case 0x01:
      if (n >= 5) for (int i = 0; i < 4; i++) setWheel(i, (int8_t)d[1 + i]);
      break;
    case 0x02:
      brakeAll();
      break;
    case 0x03:
      break;
    case 0x04: {
      int len = min((int)n - 1, 18), k = 0;
      for (int i = 0; i < len; i++) { char c = d[1 + i]; webText[k++] = (c >= 32 && c < 127) ? c : '?'; }
      webText[k] = 0;
      break;
    }
    case 0x05:
      if (n >= 3 && pinAllowed(d[1])) { pinMode(d[1], OUTPUT); digitalWrite(d[1], d[2] ? HIGH : LOW); }
      break;
    case 0x06:
      if (n >= 3 && d[1] < 4) setWheel(d[1], (int8_t)d[2]);
      break;
    default:
      break;
  }
}

// 상태 (20바이트, 10 Hz)
//   0: 0xA1   1: 모드(0 조종, 1 블록)   2: 플래그(bit0 컨트롤러 연결, bit1 안전 정지, bit2 BOOT 눌림, bit3 BOOT 짧게 눌림)
//   3~6: 바퀴 출력 FL FR BL BR (int8 %)   7: 제동 비트(FL=bit0 … BR=bit3)
//   8~9: 순번   10~13: millis()
void sendWebStatus() {
  static unsigned long last = 0;
  if (millis() - last < 100) return;
  last = millis();
  uint8_t s[20] = {0};
  s[0] = 0xA1;
  s[1] = webMode ? 1 : 0;
  s[2] = (myCtl && myCtl->isConnected() ? 1 : 0) | (webTimedOut ? 2 : 0) |
         (digitalRead(BOOT_BTN_PIN) == LOW ? 4 : 0) | (bootShortPress ? 8 : 0);
  bootShortPress = false;
  for (int i = 0; i < 4; i++) s[3 + i] = (uint8_t)motorPct[i];
  s[7] = brakeMask;
  s[8] = statusSeq & 0xFF;  s[9] = statusSeq >> 8;  statusSeq++;
  uint32_t ms = millis();
  memcpy(s + 10, &ms, 4);
  RoboLink::sendStatus(s, 14);
}

void startRoboLink() {
  uint64_t mac = ESP.getEfuseMac();
  char name[24];
  snprintf(name, sizeof(name), "RoboBlock-%02X%02X", (uint8_t)(mac >> 32), (uint8_t)(mac >> 40));
  char info[200];
  snprintf(info, sizeof(info),
           "robot=soccer;fw=roboblock-soccer-0.1;name=%s;"
           "FL=32,33;FR=27,4;BL=25,26;BR=22,21;oled=16,17;boot=0", name);
  RoboLink::begin(name, info);
  Serial.printf("로보블록 웹 블루투스 이름: %s\n", name);
}

// ---------------------------------------------------------------
void setup() {
  // === 브라운아웃 디텍터 비활성화 ===
  // 모터 시동/방향 전환 순간의 전압 새깅(브라운아웃) 때문에
  // ESP32가 리셋되면서 BT 연결이 끊기는 현상을 방지.
  // 반드시 setup() 맨 첫 줄에 위치시켜야 효과 있음.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // === 부팅 즉시 모터 IN 핀을 LOW로 강제 (튐 방지) ===
  // DRV8871은 IN1=IN2=0이면 코스트(정지) 상태.
  // 내부 풀다운만으론 부팅 첫 수십 ms 동안 모터선 결합으로 튈 수 있어,
  // 가능한 한 빨리 핀을 출력 LOW로 잡아 강제 코스트 시킨다.
  const int inPins[8] = { LF_IN1, LF_IN2, LR_IN1, LR_IN2,
                          RF_IN1, RF_IN2, RR_IN1, RR_IN2 };
  for (int p : inPins) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }

  Serial.begin(115200);
  delay(300);
  Serial.println("\nESP32 4WD Xbox 축구로봇 (DRV8871 x4 · 브라운아웃 OFF · 능동 제동) Booting...");

  // PWM 채널 8개 설정 후 각 IN핀에 연결.
  // (STBY가 없으므로 외부 10k 풀다운으로 부팅 중 코스트 보장 — 배선 가이드 참조)
  ledcSetup(CH_LF1, PWM_FREQ, PWM_RES); ledcAttachPin(LF_IN1, CH_LF1);
  ledcSetup(CH_LF2, PWM_FREQ, PWM_RES); ledcAttachPin(LF_IN2, CH_LF2);
  ledcSetup(CH_LR1, PWM_FREQ, PWM_RES); ledcAttachPin(LR_IN1, CH_LR1);
  ledcSetup(CH_LR2, PWM_FREQ, PWM_RES); ledcAttachPin(LR_IN2, CH_LR2);
  ledcSetup(CH_RF1, PWM_FREQ, PWM_RES); ledcAttachPin(RF_IN1, CH_RF1);
  ledcSetup(CH_RF2, PWM_FREQ, PWM_RES); ledcAttachPin(RF_IN2, CH_RF2);
  ledcSetup(CH_RR1, PWM_FREQ, PWM_RES); ledcAttachPin(RR_IN1, CH_RR1);
  ledcSetup(CH_RR2, PWM_FREQ, PWM_RES); ledcAttachPin(RR_IN2, CH_RR2);

  // 부팅 직후엔 능동 제동 상태로 시작
  brakeAll();

#if SELFTEST
  delay(800);
  runSelfTest();
#endif

  // BOOT 버튼을 입력 풀업으로 설정 (눌리지 않으면 HIGH, 눌리면 LOW)
  pinMode(BOOT_BTN_PIN, INPUT_PULLUP);

#if FORGET_PAIRING
  clearPairedMAC();
  BP32.forgetBluetoothKeys();
  Serial.println("\n*** 저장된 페어링을 모두 삭제했습니다. ***");
  Serial.println("    FORGET_PAIRING을 0으로 되돌리고 다시 업로드하세요.\n");
#endif
  loadPairedMAC();

  BP32.setup(&onConnect, &onDisconnect);
  BP32.enableVirtualDevice(false);
  BP32.enableBLEService(false);      // Bluepad32 자체 BLE 서비스는 끈다 (RoboLink가 GATT 서버를 쓴다)
  startRoboLink();

#if USE_TFT
  setupTFT();
#endif

  Serial.println("Ready. Xbox 컨트롤러를 페어링 모드로 두세요.");
}

// ---------------------------------------------------------------
void loop() {
  bool updated = BP32.update();

  // 모드 전환: 웹 에디터가 붙으면 블록 모드, 떨어지면 조종 모드
  bool web = RoboLink::connected();
  if (web && !webMode) enterWebMode();
  else if (!web && webMode) leaveWebMode();

  if (webMode) {
    uint8_t buf[20], n;
    while (RoboLink::popCommand(buf, &n)) handleWebCommand(buf, n);
    if (!webTimedOut && millis() - lastWebCmdMs > WEB_TIMEOUT_MS) {
      brakeAll();                // 명령이 끊기면 즉시 능동 제동
      webTimedOut = true;
    }
  } else {
    if (updated && myCtl && myCtl->isConnected() && myCtl->hasData()) {
      handleInput(myCtl);
    }
    if (!myCtl || !myCtl->isConnected()) {
      brakeAll();                // 컨트롤러 없으면 즉시 제동 유지
    }
  }

  checkBootButton();             // BOOT 버튼 3초 유지 → 페어링 초기화, 짧게 → 블록의 로봇 버튼
  sendWebStatus();

#if USE_TFT
  updateTFT();
#endif

  delay(10);                     // 응답성을 위해 약간 더 자주 폴링
}
