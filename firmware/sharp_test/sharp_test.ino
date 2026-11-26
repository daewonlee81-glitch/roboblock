// ===============================================================
// Sharp GP2Y0A21 거리 센서 3개 작동 확인 (스모로봇)
//   보드: NodeMCU-32S (esp32_bluepad32 보드 패키지 또는 일반 ESP32 보드 패키지 모두 됨)
//   배선: 앞 Vo → SVP(GPIO36), 왼쪽 Vo → SVN(GPIO39), 오른쪽 Vo → GPIO34
//         Vcc 3개 → 확장 보드 5V 핀, GND → G (Vcc·GND 사이 10~100 µF 권장)
//   결과: 시리얼 모니터(115200)에 0.2초마다 표로, OLED(SDA 16, SCL 17)에 세 거리 막대로
//   시리얼 플로터로 보려면 PLOTTER를 1로 바꾸세요.
// ===============================================================
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define PLOTTER 0                 // 1: 시리얼 플로터용 (F:30 L:45 R:80 형식)

const int   PINS[3]  = {36, 39, 34};
const char* NAMES[3] = {"F", "L", "R"};
const char* LABELS[3] = {"앞   (SVP 36)", "왼쪽 (SVN 39)", "오른쪽 (34)  "};

#define OLED_SDA 16
#define OLED_SCL 17
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool hasOled = false;

#define SH_N 9                    // 중앙값 샘플 수 (8 ms × 9 ≈ 72 ms) — 로봇 펌웨어 0.7.1과 같은 필터
#define SH_JUMP_MV 250            // 이보다 크게 바뀌면 3번 연속일 때만 따라감 (한 번짜리 튐은 버림)
uint16_t buf[3][SH_N];
uint8_t  jumpCnt[3] = {0, 0, 0};
uint8_t  idx = 0;
float    ema[3] = {0, 0, 0};
uint16_t mv[3];
int      cm[3];                   // -1 = 80 cm 넘음(앞에 없음), 0 = 신호 없음
unsigned long lowSince[3] = {0, 0, 0};

// GP2Y0A21 근사식 (출력 전압 → 거리). 0.42 V 아래는 80 cm 넘음
int toCm(float v) {
  if (v < 0.42f) return -1;
  float d = 29.988f * powf(v, -1.173f);
  if (d < 10) d = 10;             // 10 cm 안쪽은 전압이 다시 내려가 잘못 읽히므로 10으로
  if (d > 80) d = 80;
  return (int)(d + 0.5f);
}

uint16_t medianN(uint16_t* b) {
  uint16_t t[SH_N]; memcpy(t, b, sizeof(t));
  for (int i = 1; i < SH_N; i++) for (int j = i; j > 0 && t[j - 1] > t[j]; j--) { uint16_t x = t[j]; t[j] = t[j - 1]; t[j - 1] = x; }
  return t[SH_N / 2];
}

const char* state(int i) {
  if (lowSince[i] && millis() - lowSince[i] > 1000) return "신호 없음 (Vo·5V·GND 확인)";
  if (mv[i] > 3000) return "너무 가깝거나 배선 확인";
  if (cm[i] < 0) return "앞에 없음 (80 cm 넘음)";
  if (cm[i] < 70) return "감지 (70 cm 안)";
  return "멀리 있음";
}

void setup() {
  Serial.begin(115200);
  delay(300);
  for (int i = 0; i < 3; i++) {
    pinMode(PINS[i], INPUT);
    analogSetPinAttenuation(PINS[i], ADC_11db);   // 0 ~ 3.1 V 측정
  }
  Wire.begin(OLED_SDA, OLED_SCL);
  hasOled = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (hasOled) { oled.clearDisplay(); oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1); oled.setCursor(0, 0); oled.print("Sharp test"); oled.display(); }
#if !PLOTTER
  Serial.println();
  Serial.println("Sharp GP2Y0A21 x3 test");
  Serial.println("빈 공간: 0.3~0.4 V(>80 cm) / 손 20 cm: 약 1.3 V / 10 cm: 약 2.3 V 가 정상입니다.");
  Serial.println(hasOled ? "OLED 찾음" : "OLED 없음 (시리얼만 사용)");
#endif
}

void loop() {
  static unsigned long tRead = 0, tPrint = 0, tOled = 0;
  unsigned long now = millis();

  // 8 ms마다 세 센서를 읽고 최근 5개의 중앙값 + 지수 평균
  if (now - tRead >= 8) {
    tRead = now;
    for (int i = 0; i < 3; i++) {
      buf[i][idx] = analogReadMilliVolts(PINS[i]);
      float m = medianN(buf[i]), diff = m - ema[i];
      if (ema[i] <= 0) ema[i] = m;
      else if (fabsf(diff) > SH_JUMP_MV) { if (++jumpCnt[i] >= 3) { ema[i] += 0.5f * diff; jumpCnt[i] = 0; } }
      else { jumpCnt[i] = 0; ema[i] += 0.25f * diff; }
      mv[i] = (uint16_t)(ema[i] + 0.5f);
      cm[i] = toCm(mv[i] / 1000.0f);
      if (mv[i] < 150) { if (!lowSince[i]) lowSince[i] = now; } else lowSince[i] = 0;
    }
    idx = (idx + 1) % SH_N;
  }

  // 시리얼
  if (now - tPrint >= (PLOTTER ? 50 : 200)) {
    tPrint = now;
#if PLOTTER
    for (int i = 0; i < 3; i++) { Serial.printf("%s:%d", NAMES[i], cm[i] < 0 ? 85 : cm[i]); Serial.print(i < 2 ? " " : "\n"); }
#else
    Serial.println("----------------------------------------------");
    for (int i = 0; i < 3; i++) {
      if (cm[i] < 0) Serial.printf("%s  %.2f V   >80 cm   %s\n", LABELS[i], mv[i] / 1000.0f, state(i));
      else           Serial.printf("%s  %.2f V   %3d cm   %s\n", LABELS[i], mv[i] / 1000.0f, cm[i], state(i));
    }
#endif
  }

  // OLED: 센서마다 한 줄 (이름, 전압, 거리, 막대)
  if (hasOled && now - tOled >= 100) {
    tOled = now;
    oled.clearDisplay();
    oled.setCursor(0, 0); oled.print("Sharp  V     cm");
    for (int i = 0; i < 3; i++) {
      int y = 14 + i * 17;
      oled.setCursor(0, y);
      oled.printf("%s %4.2f ", NAMES[i], mv[i] / 1000.0f);
      if (lowSince[i] && now - lowSince[i] > 1000) oled.print(" --");
      else if (cm[i] < 0) oled.print(">80");
      else oled.printf("%3d", cm[i]);
      // 막대: 가까울수록 길게 (10 cm = 가득, 80 cm 넘음 = 없음)
      int len = cm[i] < 0 ? 0 : map(80 - cm[i], 0, 70, 2, 46);
      oled.drawRect(80, y, 48, 8, SSD1306_WHITE);
      oled.fillRect(81, y + 1, len, 6, SSD1306_WHITE);
    }
    oled.display();
  }
}
