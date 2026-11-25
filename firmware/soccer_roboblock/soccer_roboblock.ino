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
 *   OLED: 켜지면 PC 연결 상태(PC : waiting / PC:OK)와 웹 블루투스 이름(ID : RoboBlock-XXXX)을 보여 주고,
 *         연결·해제되는 순간 2초 동안 PC LINKED / PC UNLINK를 크게 띄운다.
 *
 * 파일: 이 .ino 하나로 완결됩니다 (BLE GATT 서버와 GATT 표가 안에 들어 있음).
 *       필요한 라이브러리: Adafruit SSD1306 (+ Adafruit GFX)
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
#include "btstack.h"            // Bluepad32 안의 BTstack (로보블록 웹 연결용)
#include "soc/soc.h"            // 브라운아웃 디텍터 비활성화용
#include "soc/rtc_cntl_reg.h"
#include <Wire.h>
#define COLOR_R_SDA 13            // 오른쪽 바닥 색 센서용 두 번째 I²C (Wire1)
#define COLOR_R_SCL 14

// 바닥 색 센서 TCS34725 한 개의 상태 (아래 "바닥 색 센서" 부분에서 사용)
struct ColorSensor {
  TwoWire* bus;
  bool ok;
  uint16_t c, r, g, b;          // 원래 값
  uint8_t bright;               // 밝기 0~100 (보정 반영)
  uint8_t color;                // 0 없음 1 검정 2 흰색 3 빨강 4 노랑 5 초록 6 파랑
  uint8_t r8, g8, b8;           // 화면 표시용 색 (흰색 보정 반영, 0~255)
  uint16_t wc, wr, wg, wb;      // 흰색 보정 (0 = 안 함)
  uint16_t kc;                  // 검정 보정
};


// =====================================================================
// RoboLink — 로보블록 웹 에디터와 Web Bluetooth(BLE GATT)로 통신
//   Bluepad32가 쓰는 BTstack에 GATT 서버(peripheral)를 함께 올린다.
//   Bluepad32 자체 BLE 서비스는 꺼 둔다(BP32.enableBLEService(false)).
//   BTstack 콜백은 BTstack 태스크에서, loop()는 다른 태스크에서 돈다:
//   받은 명령은 큐에 넣어 loop()에서 꺼내고, 상태 알림은 BTstack 태스크로 넘겨 보낸다.
// =====================================================================

// ---- GATT 표 (roboblock.gatt를 BTstack tool/compile_gatt.py로 만든 것) ----

// Reference: https://en.cppreference.com/w/cpp/feature_test
#if __cplusplus >= 200704L
constexpr
#endif
static const uint8_t profile_data[] =
{
    // ATT DB Version
    1,

    // 0x0001 PRIMARY_SERVICE-GAP_SERVICE
    0x0a, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00, 0x28, 0x00, 0x18, 
    // 0x0002 CHARACTERISTIC-GAP_DEVICE_NAME - READ
    0x0d, 0x00, 0x02, 0x00, 0x02, 0x00, 0x03, 0x28, 0x02, 0x03, 0x00, 0x00, 0x2a, 
    // 0x0003 VALUE CHARACTERISTIC-GAP_DEVICE_NAME - READ -'RoboBlock'
    // READ_ANYBODY
    0x11, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00, 0x2a, 0x52, 0x6f, 0x62, 0x6f, 0x42, 0x6c, 0x6f, 0x63, 0x6b, 
    // 0x0004 PRIMARY_SERVICE-GATT_SERVICE
    0x0a, 0x00, 0x02, 0x00, 0x04, 0x00, 0x00, 0x28, 0x01, 0x18, 
    // 0x0005 CHARACTERISTIC-GATT_DATABASE_HASH - READ
    0x0d, 0x00, 0x02, 0x00, 0x05, 0x00, 0x03, 0x28, 0x02, 0x06, 0x00, 0x2a, 0x2b, 
    // 0x0006 VALUE CHARACTERISTIC-GATT_DATABASE_HASH - READ -''
    // READ_ANYBODY
    0x18, 0x00, 0x02, 0x00, 0x06, 0x00, 0x2a, 0x2b, 0xee, 0x68, 0x51, 0x67, 0x13, 0xc6, 0xf6, 0x4d, 0x7e, 0x78, 0xa0, 0x0b, 0x72, 0x3c, 0xcd, 0x40, 
    // RoboBlock 서비스
    // 0x0007 PRIMARY_SERVICE-7B1C0000-5A3E-4C2B-9F6D-2B7E0D1A0C00
    0x18, 0x00, 0x02, 0x00, 0x07, 0x00, 0x00, 0x28, 0x00, 0x0c, 0x1a, 0x0d, 0x7e, 0x2b, 0x6d, 0x9f, 0x2b, 0x4c, 0x3e, 0x5a, 0x00, 0x00, 0x1c, 0x7b, 
    // 명령 (에디터 → 로봇)
    // 0x0008 CHARACTERISTIC-7B1C0001-5A3E-4C2B-9F6D-2B7E0D1A0C00 - WRITE | WRITE_WITHOUT_RESPONSE | DYNAMIC
    0x1b, 0x00, 0x02, 0x00, 0x08, 0x00, 0x03, 0x28, 0x0c, 0x09, 0x00, 0x00, 0x0c, 0x1a, 0x0d, 0x7e, 0x2b, 0x6d, 0x9f, 0x2b, 0x4c, 0x3e, 0x5a, 0x01, 0x00, 0x1c, 0x7b, 
    // 0x0009 VALUE CHARACTERISTIC-7B1C0001-5A3E-4C2B-9F6D-2B7E0D1A0C00 - WRITE | WRITE_WITHOUT_RESPONSE | DYNAMIC
    // WRITE_ANYBODY
    0x16, 0x00, 0x0c, 0x03, 0x09, 0x00, 0x00, 0x0c, 0x1a, 0x0d, 0x7e, 0x2b, 0x6d, 0x9f, 0x2b, 0x4c, 0x3e, 0x5a, 0x01, 0x00, 0x1c, 0x7b, 
    // 상태 (로봇 → 에디터, 10 Hz 알림)
    // 0x000a CHARACTERISTIC-7B1C0002-5A3E-4C2B-9F6D-2B7E0D1A0C00 - READ | NOTIFY | DYNAMIC
    0x1b, 0x00, 0x02, 0x00, 0x0a, 0x00, 0x03, 0x28, 0x12, 0x0b, 0x00, 0x00, 0x0c, 0x1a, 0x0d, 0x7e, 0x2b, 0x6d, 0x9f, 0x2b, 0x4c, 0x3e, 0x5a, 0x02, 0x00, 0x1c, 0x7b, 
    // 0x000b VALUE CHARACTERISTIC-7B1C0002-5A3E-4C2B-9F6D-2B7E0D1A0C00 - READ | NOTIFY | DYNAMIC
    // READ_ANYBODY
    0x16, 0x00, 0x02, 0x03, 0x0b, 0x00, 0x00, 0x0c, 0x1a, 0x0d, 0x7e, 0x2b, 0x6d, 0x9f, 0x2b, 0x4c, 0x3e, 0x5a, 0x02, 0x00, 0x1c, 0x7b, 
    // 0x000c CLIENT_CHARACTERISTIC_CONFIGURATION
    // READ_ANYBODY, WRITE_ANYBODY
    0x0a, 0x00, 0x0e, 0x01, 0x0c, 0x00, 0x02, 0x29, 0x00, 0x00, 
    // 로봇 정보 (종류, 펌웨어, 핀)
    // 0x000d CHARACTERISTIC-7B1C0003-5A3E-4C2B-9F6D-2B7E0D1A0C00 - READ | DYNAMIC
    0x1b, 0x00, 0x02, 0x00, 0x0d, 0x00, 0x03, 0x28, 0x02, 0x0e, 0x00, 0x00, 0x0c, 0x1a, 0x0d, 0x7e, 0x2b, 0x6d, 0x9f, 0x2b, 0x4c, 0x3e, 0x5a, 0x03, 0x00, 0x1c, 0x7b, 
    // 0x000e VALUE CHARACTERISTIC-7B1C0003-5A3E-4C2B-9F6D-2B7E0D1A0C00 - READ | DYNAMIC
    // READ_ANYBODY
    0x16, 0x00, 0x02, 0x03, 0x0e, 0x00, 0x00, 0x0c, 0x1a, 0x0d, 0x7e, 0x2b, 0x6d, 0x9f, 0x2b, 0x4c, 0x3e, 0x5a, 0x03, 0x00, 0x1c, 0x7b, 
    // END
    0x00, 0x00, 
}; // total size 153 bytes 


//
// list service handle ranges
//
#define ATT_SERVICE_GAP_SERVICE_START_HANDLE 0x0001
#define ATT_SERVICE_GAP_SERVICE_END_HANDLE 0x0003
#define ATT_SERVICE_GAP_SERVICE_01_START_HANDLE 0x0001
#define ATT_SERVICE_GAP_SERVICE_01_END_HANDLE 0x0003
#define ATT_SERVICE_GATT_SERVICE_START_HANDLE 0x0004
#define ATT_SERVICE_GATT_SERVICE_END_HANDLE 0x0006
#define ATT_SERVICE_GATT_SERVICE_01_START_HANDLE 0x0004
#define ATT_SERVICE_GATT_SERVICE_01_END_HANDLE 0x0006
#define ATT_SERVICE_7B1C0000_5A3E_4C2B_9F6D_2B7E0D1A0C00_START_HANDLE 0x0007
#define ATT_SERVICE_7B1C0000_5A3E_4C2B_9F6D_2B7E0D1A0C00_END_HANDLE 0x000e
#define ATT_SERVICE_7B1C0000_5A3E_4C2B_9F6D_2B7E0D1A0C00_01_START_HANDLE 0x0007
#define ATT_SERVICE_7B1C0000_5A3E_4C2B_9F6D_2B7E0D1A0C00_01_END_HANDLE 0x000e

//
// list mapping between characteristics and handles
//
#define ATT_CHARACTERISTIC_GAP_DEVICE_NAME_01_VALUE_HANDLE 0x0003
#define ATT_CHARACTERISTIC_GATT_DATABASE_HASH_01_VALUE_HANDLE 0x0006
#define ATT_CHARACTERISTIC_7B1C0001_5A3E_4C2B_9F6D_2B7E0D1A0C00_01_VALUE_HANDLE 0x0009
#define ATT_CHARACTERISTIC_7B1C0002_5A3E_4C2B_9F6D_2B7E0D1A0C00_01_VALUE_HANDLE 0x000b
#define ATT_CHARACTERISTIC_7B1C0002_5A3E_4C2B_9F6D_2B7E0D1A0C00_01_CLIENT_CONFIGURATION_HANDLE 0x000c
#define ATT_CHARACTERISTIC_7B1C0003_5A3E_4C2B_9F6D_2B7E0D1A0C00_01_VALUE_HANDLE 0x000e

// ---- RoboLink 본체 ----
#define CMD_HANDLE     ATT_CHARACTERISTIC_7B1C0001_5A3E_4C2B_9F6D_2B7E0D1A0C00_01_VALUE_HANDLE
#define STATUS_HANDLE  ATT_CHARACTERISTIC_7B1C0002_5A3E_4C2B_9F6D_2B7E0D1A0C00_01_VALUE_HANDLE
#define STATUS_CCC     ATT_CHARACTERISTIC_7B1C0002_5A3E_4C2B_9F6D_2B7E0D1A0C00_01_CLIENT_CONFIGURATION_HANDLE
#define INFO_HANDLE    ATT_CHARACTERISTIC_7B1C0003_5A3E_4C2B_9F6D_2B7E0D1A0C00_01_VALUE_HANDLE


portMUX_TYPE rl_mux = portMUX_INITIALIZER_UNLOCKED;

volatile hci_con_handle_t rl_conHandle = HCI_CON_HANDLE_INVALID;
volatile bool rl_isConnected = false;
volatile bool rl_notifyOn = false;
volatile bool rl_notifyPending = false;
volatile bool rl_notifyQueued = false;   // notifyReg가 BTstack 큐에 들어가 있는 동안 true
volatile bool rl_discQueued = false;

// 명령 큐 (BTstack 태스크 → loop)
struct RlCmd { uint8_t len; uint8_t d[20]; };
const int rl_QN = 16;
RlCmd rl_queue[rl_QN];
volatile uint8_t rl_qHead = 0, rl_qTail = 0;

uint8_t rl_statusBuf[20];
uint8_t rl_statusLen = 0;
uint8_t rl_extraBuf[20];                 // 두 번째 알림 (0xA2 바닥 색 센서)
uint8_t rl_extraLen = 0;
volatile bool rl_statusDirty = false, rl_extraDirty = false;

char rl_infoStr[400];
uint8_t rl_advData[31];
uint8_t rl_advLen = 0;
uint8_t rl_scanResp[31];
uint8_t rl_scanLen = 0;

btstack_context_callback_registration_t rl_initReg;
btstack_context_callback_registration_t rl_notifyReg;
btstack_context_callback_registration_t rl_discReg;

uint16_t rl_onRead(hci_con_handle_t, uint16_t handle, uint16_t offset, uint8_t* buffer, uint16_t size) {
  if (handle == STATUS_HANDLE) {
    uint8_t tmp[20];
    uint16_t n;
    portENTER_CRITICAL(&rl_mux);
    n = rl_statusLen;
    memcpy(tmp, rl_statusBuf, n);
    portEXIT_CRITICAL(&rl_mux);
    return att_read_callback_handle_blob(tmp, n, offset, buffer, size);
  }
  if (handle == INFO_HANDLE) {
    char tmp[sizeof(rl_infoStr)];
    portENTER_CRITICAL(&rl_mux);
    memcpy(tmp, rl_infoStr, sizeof(tmp));
    portEXIT_CRITICAL(&rl_mux);
    return att_read_callback_handle_blob((const uint8_t*)tmp, strlen(tmp), offset, buffer, size);
  }
  if (handle == STATUS_CCC) {
    uint8_t v[2] = {(uint8_t)(rl_notifyOn ? 1 : 0), 0};
    return att_read_callback_handle_blob(v, 2, offset, buffer, size);
  }
  return 0;
}

int rl_onWrite(hci_con_handle_t, uint16_t handle, uint16_t mode, uint16_t offset, uint8_t* buffer, uint16_t size) {
  (void)mode;
  if (handle == STATUS_CCC) {
    rl_notifyOn = size >= 2 && little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
    return 0;
  }
  if (handle == CMD_HANDLE) {
    if (offset != 0 || size == 0 || size > 20) return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
    portENTER_CRITICAL(&rl_mux);
    uint8_t next = (rl_qHead + 1) % rl_QN;
    if (next != rl_qTail) {           // 가득 차면 가장 새 명령을 버린다
      rl_queue[rl_qHead].len = size;
      memcpy(rl_queue[rl_qHead].d, buffer, size);
      rl_qHead = next;
    }
    portEXIT_CRITICAL(&rl_mux);
    return 0;
  }
  return 0;
}

void rl_onPacket(uint8_t type, uint16_t, uint8_t* packet, uint16_t) {
  if (type != HCI_EVENT_PACKET) return;
  switch (hci_event_packet_get_type(packet)) {
    case ATT_EVENT_CONNECTED:
      rl_conHandle = att_event_connected_get_handle(packet);
      rl_notifyOn = false;
      rl_notifyPending = false;
      rl_isConnected = true;
      break;
    case ATT_EVENT_DISCONNECTED:
      if (att_event_disconnected_get_handle(packet) == rl_conHandle) {
        rl_conHandle = HCI_CON_HANDLE_INVALID;
        rl_isConnected = false;
        rl_notifyOn = false;
        rl_notifyPending = false;
      }
      break;
    case ATT_EVENT_CAN_SEND_NOW: {
      uint8_t tmp[20];
      uint16_t n = 0;
      bool more;
      portENTER_CRITICAL(&rl_mux);
      if (rl_statusDirty) { n = rl_statusLen; memcpy(tmp, rl_statusBuf, n); rl_statusDirty = false; }
      else if (rl_extraDirty) { n = rl_extraLen; memcpy(tmp, rl_extraBuf, n); rl_extraDirty = false; }
      more = rl_statusDirty || rl_extraDirty;
      portEXIT_CRITICAL(&rl_mux);
      bool ok = rl_conHandle != HCI_CON_HANDLE_INVALID && rl_notifyOn;
      if (ok && n) att_server_notify(rl_conHandle, STATUS_HANDLE, tmp, n);
      if (ok && more) att_server_request_can_send_now_event(rl_conHandle);   // 남은 알림을 이어서 보낸다
      else rl_notifyPending = false;
      break;
    }
    default:
      break;
  }
}

// BTstack 태스크에서 실행
void rl_doInit(void*) {
  att_server_init(profile_data, rl_onRead, rl_onWrite);
  att_server_register_packet_handler(rl_onPacket);
  bd_addr_t nullAddr = {0};
  gap_advertisements_set_params(0x0030, 0x0060, 0 /* ADV_IND 연결 가능 */, 0, nullAddr, 0x07, 0x00);
  gap_advertisements_set_data(rl_advLen, rl_advData);
  gap_scan_response_set_data(rl_scanLen, rl_scanResp);
  gap_advertisements_enable(1);
}

void rl_doNotify(void*) {
  rl_notifyQueued = false;
  if (rl_conHandle != HCI_CON_HANDLE_INVALID && rl_notifyOn) att_server_request_can_send_now_event(rl_conHandle);
  else rl_notifyPending = false;
}

void rl_doDisconnect(void*) {
  rl_discQueued = false;
  if (rl_conHandle != HCI_CON_HANDLE_INVALID) gap_disconnect(rl_conHandle);
}


// 정보 문자열 바꾸기 (핀 설정이 바뀌었을 때)
void rlSetInfo(const char* info) {
  portENTER_CRITICAL(&rl_mux);
  strncpy(rl_infoStr, info, sizeof(rl_infoStr) - 1);
  rl_infoStr[sizeof(rl_infoStr) - 1] = 0;
  portEXIT_CRITICAL(&rl_mux);
}

void rlBegin(const char* advName, const char* info) {
  strncpy(rl_infoStr, info, sizeof(rl_infoStr) - 1);

  // 광고: 플래그 + 이름 (Web Bluetooth에서 "RoboBlock"으로 시작하는 이름으로 찾는다)
  size_t nameLen = strlen(advName);
  if (nameLen > 26) nameLen = 26;
  rl_advLen = 0;
  rl_advData[rl_advLen++] = 2;  rl_advData[rl_advLen++] = BLUETOOTH_DATA_TYPE_FLAGS;  rl_advData[rl_advLen++] = 0x06;
  rl_advData[rl_advLen++] = nameLen + 1;  rl_advData[rl_advLen++] = BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME;
  memcpy(rl_advData + rl_advLen, advName, nameLen);  rl_advLen += nameLen;

  // 스캔 응답: 128비트 서비스 UUID (리틀 엔디언)
  static const uint8_t uuid[16] = {0x00, 0x0C, 0x1A, 0x0D, 0x7E, 0x2B, 0x6D, 0x9F,
                                   0x2B, 0x4C, 0x3E, 0x5A, 0x00, 0x00, 0x1C, 0x7B};
  rl_scanLen = 0;
  rl_scanResp[rl_scanLen++] = 17;  rl_scanResp[rl_scanLen++] = BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS;
  memcpy(rl_scanResp + rl_scanLen, uuid, 16);  rl_scanLen += 16;

  rl_initReg.callback = rl_doInit;
  btstack_run_loop_execute_on_main_thread(&rl_initReg);
}

bool rlConnected() { return rl_isConnected; }

bool rlPopCommand(uint8_t* buf, uint8_t* len) {
  bool ok = false;
  portENTER_CRITICAL(&rl_mux);
  if (rl_qTail != rl_qHead) {
    *len = rl_queue[rl_qTail].len;
    memcpy(buf, rl_queue[rl_qTail].d, *len);
    rl_qTail = (rl_qTail + 1) % rl_QN;
    ok = true;
  }
  portEXIT_CRITICAL(&rl_mux);
  return ok;
}

void rlKick() {
  if (!rl_isConnected || !rl_notifyOn || rl_notifyPending || rl_notifyQueued) return;
  rl_notifyPending = true;
  rl_notifyQueued = true;
  rl_notifyReg.callback = rl_doNotify;
  btstack_run_loop_execute_on_main_thread(&rl_notifyReg);
}

// 두 번째 알림 (상태와 같은 특성으로 보내고, 첫 바이트로 구분: 0xA2)
void rlSendExtra(const uint8_t* data, uint8_t len) {
  if (len > 20) len = 20;
  portENTER_CRITICAL(&rl_mux);
  memcpy(rl_extraBuf, data, len);
  rl_extraLen = len;
  rl_extraDirty = true;
  portEXIT_CRITICAL(&rl_mux);
  rlKick();
}

void rlSendStatus(const uint8_t* data, uint8_t len) {
  if (len > 20) len = 20;
  portENTER_CRITICAL(&rl_mux);
  memcpy(rl_statusBuf, data, len);
  rl_statusLen = len;
  rl_statusDirty = true;
  portEXIT_CRITICAL(&rl_mux);
  if (!rl_isConnected || !rl_notifyOn || rl_notifyPending || rl_notifyQueued) return;
  rl_notifyPending = true;
  rl_notifyQueued = true;
  rl_notifyReg.callback = rl_doNotify;
  btstack_run_loop_execute_on_main_thread(&rl_notifyReg);
}

void rlDisconnect() {
  if (rl_discQueued) return;
  rl_discQueued = true;
  rl_discReg.callback = rl_doDisconnect;
  btstack_run_loop_execute_on_main_thread(&rl_discReg);
}


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
const int PIN_DEFAULT[4][2] = { {32, 33}, {27, 4}, {25, 26}, {22, 21} };
int PIN_OF[4][2] = { {32, 33}, {27, 4}, {25, 26}, {22, 21} };   // 바퀴별 (IN1, IN2) 핀 — 웹 에디터에서 바꾸면 NVS에 저장
char robotInfo[400] = "";
bool webMode = false;                    // 블록 모드(웹 에디터 연결)인가
unsigned long lastWebCmdMs = 0;          // 마지막 웹 명령 시각 (안전 정지용)
const unsigned long WEB_TIMEOUT_MS = 500;
bool webTimedOut = false;
char webText[19] = "";                   // 블록의 "화면에 표시" 내용 (ASCII)
char robotName[24] = "RoboBlock";        // 웹 블루투스 이름 (OLED에도 표시)
unsigned long pcBannerUntil = 0;         // PC 연결·해제 순간 큰 글씨 안내를 보여 줄 시각
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
// RoboMakers 로고 44x44 (1비트, 가로 한 줄씩, MSB 먼저) — oled.drawBitmap()용
const int LOGO_W = 44, LOGO_H = 44;
const uint8_t LOGO_BMP[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x3F, 0xC0, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xF0, 0x00, 0x00,
  0x00, 0x03, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFE, 0x00, 0x00,
  0x00, 0x1F, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xC0, 0x00,
  0x00, 0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x03, 0xFF, 0xFF, 0xCF, 0xFC, 0x00,
  0x07, 0xFF, 0xFF, 0xC7, 0xFE, 0x00, 0x07, 0xFF, 0xFF, 0xCF, 0x8E, 0x00,
  0x07, 0xFF, 0xFF, 0xEE, 0x0E, 0x00, 0x07, 0xFF, 0xFF, 0xE8, 0x0E, 0x00,
  0x07, 0xFF, 0xFF, 0xE0, 0x0E, 0x00, 0x07, 0xFF, 0xFF, 0xC0, 0x0E, 0x00,
  0x07, 0xFF, 0xFF, 0x00, 0x0E, 0x00, 0x07, 0xFF, 0xFC, 0x00, 0x0E, 0x00,
  0x07, 0xFF, 0xF8, 0x00, 0x0E, 0x00, 0x07, 0xFF, 0xF8, 0x01, 0x8E, 0x00,
  0x07, 0xFF, 0xF8, 0x01, 0x8E, 0x00, 0x07, 0xFF, 0xF8, 0x21, 0x0E, 0x00,
  0x07, 0xFF, 0xF8, 0x70, 0x0E, 0x00, 0x07, 0xFF, 0xF8, 0x30, 0x0E, 0x00,
  0x07, 0xFF, 0xE0, 0x00, 0x0E, 0x00, 0x07, 0xFF, 0xC0, 0x00, 0x0E, 0x00,
  0x07, 0xFF, 0xC0, 0x00, 0x0E, 0x00, 0x07, 0xFF, 0xC0, 0x00, 0x0E, 0x00,
  0x07, 0xFF, 0xC0, 0x00, 0x0E, 0x00, 0x07, 0xFF, 0xE0, 0x00, 0x3E, 0x00,
  0x03, 0xFF, 0xF8, 0x00, 0xFC, 0x00, 0x00, 0xFF, 0xF8, 0x03, 0xF0, 0x00,
  0x00, 0x7F, 0xF8, 0x07, 0xC0, 0x00, 0x00, 0x1F, 0xF8, 0x1F, 0x80, 0x00,
  0x00, 0x07, 0xF8, 0x7E, 0x00, 0x00, 0x00, 0x01, 0xFD, 0xF8, 0x00, 0x00,
  0x00, 0x00, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xC0, 0x00, 0x00,
  0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

void setupTFT() {
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 초기화 실패. I2C 주소(0x3C/0x3D)·배선 확인.");
    return;
  }
  // 부팅 화면: 왼쪽 RoboMakers 로고, 오른쪽 이름·웹 블루투스 ID (1.5초)
  oled.clearDisplay();
  oled.drawBitmap(0, 10, LOGO_BMP, LOGO_W, LOGO_H, SSD1306_WHITE);
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(48, 8);
  oled.print("RoboMakers");
  oled.drawLine(48, 18, OLED_W - 1, 18, SSD1306_WHITE);
  oled.setCursor(48, 24);
  oled.print("Booting...");
  oled.setCursor(48, 38);
  oled.print("PC : wait");
  oled.setCursor(44, 52);
  oled.print(robotName);            // RoboBlock-XXXX (14자 = 84px)
  oled.display();
  Serial.println("OLED 초기화 완료.");
  delay(1500);
}

// 부분 갱신 없이 매번 통째 redraw (OLED는 display() 호출 전엔 깜빡임 없음)
void updateTFT() {
  static unsigned long last = 0;
  if (millis() - last < 200) return;
  last = millis();

  oled.clearDisplay();

  // 타이틀 (블록 모드면 BLOCK MODE)
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(2);
  oled.setCursor(4, 0);
  oled.print(webMode ? "BLOCK MODE" : "RoboMakers");
  oled.drawLine(0, 18, OLED_W - 1, 18, SSD1306_WHITE);

  // PC(웹 에디터)와 연결·해제되는 순간 2초 동안 크게 안내
  if (millis() < pcBannerUntil) {
    oled.setTextSize(2);
    oled.setCursor(webMode ? 16 : 10, 26);
    oled.print(webMode ? "PC LINKED" : "PC UNLINK");
    oled.setTextSize(1);
    oled.setCursor(0, 50);
    oled.print(webMode ? "Blocks only, pad off" : "Xbox pad mode");
    oled.display();
    return;
  }

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
    oled.printf("PC:OK  %s", robotName);
    oled.setCursor(0, 32);
    oled.printf("FL%+4d  FR%+4d", motorPct[0], motorPct[1]);
    drawOpp(32);
    oled.setCursor(0, 42);
    char dtxt[8]; sharpText(dtxt, sizeof(dtxt));
    oled.printf("BL%+4d  BR%+4d %s", motorPct[2], motorPct[3], dtxt);
    oled.setCursor(0, 52);
    if (webTimedOut) oled.print("SIGNAL LOST - BRAKE");
    else oled.printf("> %.16s", webText);
    drawColor(52);
    oled.display();
    return;
  }

  // PC(웹 에디터) 연결 상태 — 기다리는 중이면 점이 깜빡인다
  oled.setCursor(0, 22);
  oled.print("PC : waiting");
  drawOpp(22);
  if ((millis() / 500) % 2) oled.print("...");

  // 웹 블루투스 이름 (에디터 목록에서 이 이름을 고른다)
  oled.setCursor(0, 32);
  oled.printf("ID : %s", robotName);

  // Xbox 컨트롤러 연결 상태 + 페어링 MAC 끝 2바이트
  oled.setCursor(0, 42);
  oled.printf("PAD: %s", myCtl && myCtl->isConnected() ? "OK" : "--");
  if (hasPaired) oled.printf("  MAC %02X:%02X", pairedMAC[4], pairedMAC[5]);
  else oled.print("  MAC --");
  drawColor(42);

  // 속도·방향
  oled.setCursor(0, 52);
  char dtxt[8]; sharpText(dtxt, sizeof(dtxt));
  oled.printf("Spd:%3d%% %-5s D%s", lastSpdPct, lastDirLabel, dtxt);

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
  pcBannerUntil = millis() + 2000;
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
  pcBannerUntil = millis() + 2000;
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
//   0x08 핀 [핀 핀]   : Sharp 거리 센서 핀 앞 [왼쪽 오른쪽] (32~39 ADC1, 255 = 없음) → NVS에 저장
//   0x0B 단계         : 바닥 색 보정 (1 흰색 저장, 2 검정 저장, 0 지우기) → NVS에 저장
//   0x0A 핀×3 극성    : 상대 감지 센서 핀 왼·가운데·오른 (255 = 안 씀), 극성(1 = 감지 때 HIGH) → NVS에 저장
//   0x09 단계 [센서]  : Sharp 보정, 센서 0 앞 1 왼쪽 2 오른쪽 (0 = 지우기, 1 = 지금 값을 '빈 공간', 2 = 지금 값을 '흰 종이 20 cm') → NVS에 저장
//   0x07 핀×8         : 바퀴 핀 설정 FL_IN1 FL_IN2 FR_IN1 FR_IN2 BL_IN1 BL_IN2 BR_IN1 BR_IN2 → 적용하고 NVS에 저장
bool pinAllowed(int p) {
  if (p < 0 || p > 39) return false;
  if (p >= 6 && p <= 11) return false;           // 플래시
  if (p >= 34) return false;                     // 입력 전용
  if (p == 0 || p == 1 || p == 3) return false;  // BOOT, 시리얼
  if (p == OLED_SDA || p == OLED_SCL) return false;
  for (int i = 0; i < 4; i++) if (PIN_OF[i][0] == p || PIN_OF[i][1] == p) return false;
  return true;
}

// ---- 모터 핀 설정 (바퀴별 IN1/IN2) ----
bool motorPinOk(int p) {
  if (p < 0 || p > 33) return false;              // 34~39는 입력 전용
  if (p >= 6 && p <= 11) return false;            // 플래시
  if (p == 0 || p == 1 || p == 3 || p == 12) return false;   // BOOT, 시리얼, 부팅 전압 핀
  if (p == OLED_SDA || p == OLED_SCL) return false;
  return true;
}

// ---- Sharp GP2Y0A21 적외선 거리 센서 3개 (아날로그, 10~80 cm): 앞 · 왼쪽 · 오른쪽 ----
//   배선: Vcc → 5V, GND → 공통 GND, Vo → ADC1 핀. 센서마다 Vcc·GND 사이에 10~100 µF 콘덴서 권장.
//   기본 핀: 앞 GPIO36(SVP) · 왼쪽 GPIO39(SVN) · 오른쪽 GPIO34
//   Bluepad32가 블루투스를 계속 쓰므로 ADC2 핀(0,2,4,12~15,25~27)은 못 쓴다 → 32~39 중에서.
#define SHARP_SERIAL 1                   // 1이면 시리얼 모니터에 0.5초마다 거리 출력 (센서 확인용)
const char* SHARP_NAME[3] = {"F", "L", "R"};
const char* SHARP_KEY[3] = {"", "1", "2"};  // NVS 키 끝 (앞 센서는 예전 키 그대로)
struct SharpS {
  int pin;                               // 255 = 센서 없음. 웹 에디터(명령 0x08)에서 바꾸면 NVS에 저장
  uint16_t mv;                           // 출력 전압 (mV, 보정 전)
  uint8_t cm;                            // 0 = 센서 없음, 255 = 80 cm보다 멀다, 그 밖 = cm
  uint16_t buf[5];
  uint8_t idx;
  float ema;                             // 중앙값을 한 번 더 부드럽게 (지수 평균)
  // 보정: 실제전압 = a × 측정전압 + b. 센서마다 바닥 전압이 달라(배선·복제품) 두 점으로 맞춘다.
  //   빈 공간(80 cm 넘음) = 0.40 V, 흰 종이 20 cm = 1.30 V (GP2Y0A21 대표값)
  float a, b;
  uint16_t farMv, c20Mv;                 // 보정 때 잰 원래 전압 (0 = 아직 안 함)
};
SharpS sharpS[3] = {{36}, {39}, {34}};

bool sharpPinOk(int p, int self) {
  if (p == 255) return true;
  if (p < 32 || p > 39 || p == 37 || p == 38) return false;   // ADC1만 (37·38은 보드에 없음)
  for (int i = 0; i < 4; i++) if (PIN_OF[i][0] == p || PIN_OF[i][1] == p) return false;
  for (int i = 0; i < 3; i++) if (i != self && sharpS[i].pin == p) return false;
  return true;
}

void setupSharp() {
  for (int i = 0; i < 3; i++) {
    SharpS& sh = sharpS[i];
    sh.ema = 0; sh.idx = 0; memset(sh.buf, 0, sizeof(sh.buf)); sh.cm = 0; sh.mv = 0;
    if (sh.pin == 255) continue;
    pinMode(sh.pin, INPUT);
    analogSetPinAttenuation(sh.pin, ADC_11db);   // 0~3.1 V 측정
  }
}

void loadSharpPin() {
  char k[8];
  prefs.begin("x4wd", true);
  for (int i = 0; i < 3; i++) {
    SharpS& sh = sharpS[i];
    snprintf(k, sizeof(k), "sharp%s", SHARP_KEY[i]); int v = prefs.getUChar(k, sh.pin);
    snprintf(k, sizeof(k), "sca%s", SHARP_KEY[i]); sh.a = prefs.getFloat(k, 1.0f);
    snprintf(k, sizeof(k), "scb%s", SHARP_KEY[i]); sh.b = prefs.getFloat(k, 0.0f);
    snprintf(k, sizeof(k), "scf%s", SHARP_KEY[i]); sh.farMv = prefs.getUShort(k, 0);
    snprintf(k, sizeof(k), "sc2%s", SHARP_KEY[i]); sh.c20Mv = prefs.getUShort(k, 0);
    if (sharpPinOk(v, i)) sh.pin = v;
    if (!(sh.a > 0.3f && sh.a < 3.0f) || !(sh.b > -2.0f && sh.b < 1.0f)) { sh.a = 1.0f; sh.b = 0.0f; }
  }
  prefs.end();
}

void saveSharpPins() {
  char k[8];
  prefs.begin("x4wd", false);
  for (int i = 0; i < 3; i++) { snprintf(k, sizeof(k), "sharp%s", SHARP_KEY[i]); prefs.putUChar(k, sharpS[i].pin); }
  prefs.end();
}

// 보정 계산: 두 점이 있으면 기울기·절편, 빈 공간만 있으면 바닥 전압만 0.40 V로 내림
void computeSharpCal(int i) {
  SharpS& sh = sharpS[i];
  char k[8];
  sh.a = 1.0f; sh.b = 0.0f;
  float vf = sh.farMv / 1000.0f, v20 = sh.c20Mv / 1000.0f;
  if (sh.farMv && sh.c20Mv && v20 > vf + 0.2f) {
    sh.a = (1.30f - 0.40f) / (v20 - vf);
    sh.b = 0.40f - sh.a * vf;
    if (!(sh.a > 0.3f && sh.a < 3.0f)) { sh.a = 1.0f; sh.b = 0.40f - vf; }
  } else if (sh.farMv && vf > 0.40f) {
    sh.b = 0.40f - vf;
  }
  prefs.begin("x4wd", false);
  snprintf(k, sizeof(k), "sca%s", SHARP_KEY[i]); prefs.putFloat(k, sh.a);
  snprintf(k, sizeof(k), "scb%s", SHARP_KEY[i]); prefs.putFloat(k, sh.b);
  snprintf(k, sizeof(k), "scf%s", SHARP_KEY[i]); prefs.putUShort(k, sh.farMv);
  snprintf(k, sizeof(k), "sc2%s", SHARP_KEY[i]); prefs.putUShort(k, sh.c20Mv);
  prefs.end();
  Serial.printf("Sharp %s 보정: 빈 공간 %u mV, 20 cm %u mV → 실제 = %.3f × 측정 %+.3f V\n", SHARP_NAME[i], sh.farMv, sh.c20Mv, sh.a, sh.b);
}

// 8 ms마다 세 센서를 한 번씩 읽고, 센서마다 최근 5개의 중앙값으로 튀는 값을 거른다
void readSharp() {
  static unsigned long last = 0, lastPrint = 0;
  if (millis() - last < 8) return;
  last = millis();
  for (int i = 0; i < 3; i++) {
    SharpS& sh = sharpS[i];
    if (sh.pin == 255) { sh.cm = 0; sh.mv = 0; continue; }
    sh.buf[sh.idx] = analogReadMilliVolts(sh.pin);
    sh.idx = (sh.idx + 1) % 5;
    uint16_t t[5];
    memcpy(t, sh.buf, sizeof(t));
    for (int x = 1; x < 5; x++) for (int j = x; j > 0 && t[j - 1] > t[j]; j--) { uint16_t y = t[j]; t[j] = t[j - 1]; t[j - 1] = y; }
    sh.ema = sh.ema <= 0 ? t[2] : sh.ema + 0.35f * (t[2] - sh.ema);
    sh.mv = (uint16_t)(sh.ema + 0.5f);
    float v = sh.a * (sh.mv / 1000.0f) + sh.b;            // 보정한 전압
    if (v < 0.42f) sh.cm = 255;                          // 80 cm보다 멀거나 앞에 아무것도 없음
    else {
      float cm = 29.988f * powf(v, -1.173f);             // GP2Y0A21YK0F 근사식
      sh.cm = (uint8_t)constrain((int)(cm + 0.5f), 10, 80);   // 10 cm 안쪽은 값이 다시 작아지므로 10으로 고정
    }
  }
#if SHARP_SERIAL
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    Serial.print("Sharp");
    for (int i = 0; i < 3; i++) {
      SharpS& sh = sharpS[i];
      if (sh.pin == 255) { Serial.printf("  %s: 없음", SHARP_NAME[i]); continue; }
      if (sh.cm == 255) Serial.printf("  %s(GPIO%d) %.2fV >80cm", SHARP_NAME[i], sh.pin, sh.mv / 1000.0f);
      else Serial.printf("  %s(GPIO%d) %.2fV %dcm", SHARP_NAME[i], sh.pin, sh.mv / 1000.0f, sh.cm);
    }
    Serial.println();
  }
#endif
}

// ---- 상대 감지 센서: Sharp GP2Y0D 디지털 거리 센서 3개 (앞 왼쪽·가운데·오른쪽) ----
//   GP2Y0D02YK0F(80 cm): 물체가 정해진 거리 안에 있으면 출력 HIGH. 전원 4.5~5.5 V.
//   출력 HIGH가 5 V 가까이라 ESP32에 바로 꽂지 말고 레벨 변환기(BSS138 모듈) 등을 거친다.
//   GP2Y0D8xx 계열(5~15 cm)처럼 감지 때 LOW인 센서는 oppActiveHigh = 0.
const char* OPP_NAME[3] = {"L", "C", "R"};
int oppPin[3] = {255, 255, 255};         // 255 = 안 씀 (기본: 안 씀, 18·19·23 권장). 웹 에디터(명령 0x0A)에서 바꾸면 NVS에 저장
bool oppActiveHigh = true;
uint8_t oppMask = 0;                     // bit0 왼쪽, bit1 가운데, bit2 오른쪽 (감지 = 1)
uint8_t oppPrev = 0;

bool oppPinOk(int p) {
  if (p == 255) return true;
  if (p < 0 || p > 39 || (p >= 6 && p <= 11) || p == 37 || p == 38) return false;
  if (p == 0 || p == 1 || p == 3 || p == OLED_SDA || p == OLED_SCL || p == sharpS[0].pin || p == sharpS[1].pin || p == sharpS[2].pin || p == COLOR_R_SDA || p == COLOR_R_SCL) return false;
  for (int i = 0; i < 4; i++) if (PIN_OF[i][0] == p || PIN_OF[i][1] == p) return false;
  return true;
}

void setupOpp() {
  for (int i = 0; i < 3; i++) if (oppPin[i] != 255) pinMode(oppPin[i], INPUT);   // 레벨 변환기·분압 저항이 레벨을 잡아 준다
}

void loadOppPins() {
  uint8_t b[3];
  prefs.begin("x4wd", true);
  bool ok = prefs.getBytesLength("opp") == 3 && prefs.getBytes("opp", b, 3) == 3;
  oppActiveHigh = prefs.getUChar("opph", 1) != 0;
  prefs.end();
  if (!ok) return;
  for (int i = 0; i < 3; i++) if (!oppPinOk(b[i])) return;
  for (int i = 0; i < 3; i++) oppPin[i] = b[i];
}

// 매 loop마다 읽고, 두 번 연속 같은 값일 때만 바꾼다 (튀는 값 거르기)
void readOpp() {
  static unsigned long last = 0;
  if (millis() - last < 5) return;
  last = millis();
  uint8_t now = 0;
  for (int i = 0; i < 3; i++) {
    if (oppPin[i] == 255) continue;
    bool hi = digitalRead(oppPin[i]) == HIGH;
    if (hi == oppActiveHigh) now |= (1 << i);
  }
  uint8_t stable = (now & oppPrev) | (oppMask & (now | oppPrev));   // 두 번 연속 같으면 반영
  oppPrev = now;
  if (stable != oppMask) {
    oppMask = stable;
#if SHARP_SERIAL
    Serial.printf("상대 감지: 왼쪽 %s  가운데 %s  오른쪽 %s\n", oppMask & 1 ? "O" : "-", oppMask & 2 ? "O" : "-", oppMask & 4 ? "O" : "-");
#endif
  }
}

// ---- 바닥 색 센서: TCS34725 2개 (왼쪽 = OLED와 같은 I²C 16·17, 오른쪽 = 두 번째 I²C 13·14) ----
//   TCS34725 주소는 0x29 하나로 고정이라 한 I²C 선에 2개를 달 수 없다 → 선을 둘로 나눈다.
//   VIN → 3V3(또는 5V, 모듈에 레귤레이터가 있으면), GND → GND, LED 핀은 비워 두면 흰 LED가 켜진다.
//   센서 아래 면과 바닥 사이 3~10 mm.
#define TCS_ADDR    0x29
const char* COLOR_NAME[7] = {"-", "K", "W", "R", "Y", "G", "B"};   // 없음·검정·흰색·빨강·노랑·초록·파랑
ColorSensor colorS[2] = {{&Wire, false}, {&Wire1, false}};
uint8_t colorPrev[2] = {255, 255};

bool tcsWrite(TwoWire* w, uint8_t reg, uint8_t v) {
  w->beginTransmission(TCS_ADDR); w->write(0x80 | reg); w->write(v);
  return w->endTransmission() == 0;
}
bool tcsInit(ColorSensor& cs) {
  cs.bus->beginTransmission(TCS_ADDR); cs.bus->write(0x80 | 0x12);
  if (cs.bus->endTransmission() != 0) return false;
  if (cs.bus->requestFrom((uint8_t)TCS_ADDR, (uint8_t)1) != 1) return false;
  uint8_t id = cs.bus->read();
  if (id != 0x44 && id != 0x4D && id != 0x10) return false;      // TCS34725 / 34727 / 3472x
  tcsWrite(cs.bus, 0x01, 0xF6);                                   // 적분 24 ms (최대 10240)
  tcsWrite(cs.bus, 0x0F, 0x01);                                   // 이득 4배
  tcsWrite(cs.bus, 0x00, 0x01); delay(3);                         // 전원
  tcsWrite(cs.bus, 0x00, 0x03);                                   // 전원 + 측정
  return true;
}
void loadColorCal() {
  uint16_t v[10];
  prefs.begin("x4wd", true);
  bool ok = prefs.getBytesLength("colcal") == sizeof(v) && prefs.getBytes("colcal", v, sizeof(v)) == sizeof(v);
  prefs.end();
  if (!ok) return;
  for (int i = 0; i < 2; i++) { ColorSensor& cs = colorS[i]; cs.wc = v[i*5]; cs.wr = v[i*5+1]; cs.wg = v[i*5+2]; cs.wb = v[i*5+3]; cs.kc = v[i*5+4]; }
}
void saveColorCal() {
  uint16_t v[10];
  for (int i = 0; i < 2; i++) { ColorSensor& cs = colorS[i]; v[i*5] = cs.wc; v[i*5+1] = cs.wr; v[i*5+2] = cs.wg; v[i*5+3] = cs.wb; v[i*5+4] = cs.kc; }
  prefs.begin("x4wd", false); prefs.putBytes("colcal", v, sizeof(v)); prefs.end();
}
void setupColor() {
  Wire1.begin(COLOR_R_SDA, COLOR_R_SCL, 400000);
  Wire1.setTimeOut(10);          // 센서가 없을 때 오래 기다리지 않게
  loadColorCal();
  for (int i = 0; i < 2; i++) {
    colorS[i].ok = tcsInit(colorS[i]);
    Serial.printf("바닥 색 센서 %s: %s\n", i ? "오른쪽(13·14)" : "왼쪽(16·17)", colorS[i].ok ? "찾음" : "없음");
  }
}
// 원래 값 → 밝기·색 이름
void classifyColor(ColorSensor& cs) {
  float white = cs.wc ? cs.wc : 6000.0f, black = cs.kc ? cs.kc : 150.0f;
  if (white < black + 50) white = black + 50;
  float br = (cs.c - black) / (white - black) * 100.0f;
  cs.bright = br < 0 ? 0 : br > 100 ? 100 : (uint8_t)br;
  // 흰색 보정으로 센서마다 다른 빨강·초록·파랑 감도를 맞춘다
  float r = cs.r / (cs.wr ? cs.wr / 1000.0f : 1.0f), g = cs.g / (cs.wg ? cs.wg / 1000.0f : 1.0f), b = cs.b / (cs.wb ? cs.wb / 1000.0f : 1.0f);
  float mx = max(r, max(g, b)), mn = min(r, min(g, b));
  if (mx < 1) { cs.r8 = cs.g8 = cs.b8 = 0; cs.color = 1; return; }
  cs.r8 = r / mx * 255; cs.g8 = g / mx * 255; cs.b8 = b / mx * 255;
  float sat = (mx - mn) / mx;
  if (sat < 0.25f || cs.bright < 12) { cs.color = cs.bright < 35 ? 1 : 2; return; }      // 무채색: 검정 / 흰색
  float h;                                                                              // 색상각 0~360
  if (mx == r) h = 60 * fmodf((g - b) / (mx - mn), 6);
  else if (mx == g) h = 60 * ((b - r) / (mx - mn) + 2);
  else h = 60 * ((r - g) / (mx - mn) + 4);
  if (h < 0) h += 360;
  cs.color = (h < 25 || h >= 320) ? 3 : h < 75 ? 4 : h < 165 ? 5 : h < 260 ? 6 : 3;
}
void readColor() {
  static unsigned long last = 0, retry = 0;
  if (millis() - last < 25) return;                // 적분 24 ms마다 새 값
  last = millis();
  bool anyMissing = false;
  for (int i = 0; i < 2; i++) {
    ColorSensor& cs = colorS[i];
    if (!cs.ok) { anyMissing = true; continue; }
    cs.bus->beginTransmission(TCS_ADDR); cs.bus->write(0xA0 | 0x14);   // 자동 증가, CDATAL부터 8바이트
    if (cs.bus->endTransmission() != 0 || cs.bus->requestFrom((uint8_t)TCS_ADDR, (uint8_t)8) != 8) { cs.ok = false; cs.color = 0; continue; }
    uint8_t d[8]; for (int k = 0; k < 8; k++) d[k] = cs.bus->read();
    cs.c = d[0] | d[1] << 8; cs.r = d[2] | d[3] << 8; cs.g = d[4] | d[5] << 8; cs.b = d[6] | d[7] << 8;
    classifyColor(cs);
#if SHARP_SERIAL
    if (cs.color != colorPrev[i]) Serial.printf("바닥 %s: %s 밝기 %u%% (C%u R%u G%u B%u)\n", i ? "오른쪽" : "왼쪽", COLOR_NAME[cs.color], cs.bright, cs.c, cs.r, cs.g, cs.b);
#endif
    colorPrev[i] = cs.color;
  }
  // 뽑혀 있던 센서를 나중에 꽂아도 3초마다 다시 찾는다
  if (anyMissing && millis() - retry > 3000) {
    retry = millis();
    for (int i = 0; i < 2; i++) if (!colorS[i].ok) colorS[i].ok = tcsInit(colorS[i]);
  }
}
// 보정: 1 = 흰색 위에서 저장, 2 = 검정 위에서 저장, 0 = 지우기
void colorCalStep(uint8_t step) {
  for (int i = 0; i < 2; i++) {
    ColorSensor& cs = colorS[i];
    if (step == 0) { cs.wc = cs.wr = cs.wg = cs.wb = cs.kc = 0; continue; }
    if (!cs.ok) continue;
    if (step == 1) { cs.wc = cs.c; cs.wr = cs.r; cs.wg = cs.g; cs.wb = cs.b; }
    if (step == 2) cs.kc = cs.c;
  }
  saveColorCal();
  Serial.printf("바닥 색 보정 %s\n", step == 0 ? "지움" : step == 1 ? "흰색 저장" : "검정 저장");
}
void sendColorStatus() {
  static unsigned long last = 0;
  if (millis() - last < 100) return;
  last = millis();
  // 0: 0xA2   1: 있음(bit0 왼 bit1 오른)   2~7: 왼쪽 밝기 색 R G B 보정됨(bit0 흰 bit1 검)   8~13: 오른쪽 같은 순서
  //   14~16: 왼쪽 Sharp 전압(mV 2) 거리(cm)   17~19: 오른쪽 Sharp 같은 순서
  uint8_t s[20] = {0xA2, (uint8_t)((colorS[0].ok ? 1 : 0) | (colorS[1].ok ? 2 : 0))};
  for (int i = 0; i < 2; i++) {
    ColorSensor& cs = colorS[i]; uint8_t* o = s + 2 + i * 6;
    o[0] = cs.bright; o[1] = cs.ok ? cs.color : 0; o[2] = cs.r8; o[3] = cs.g8; o[4] = cs.b8;
    o[5] = (cs.wc ? 1 : 0) | (cs.kc ? 2 : 0);
  }
  for (int i = 1; i < 3; i++) { uint8_t* o = s + 14 + (i - 1) * 3; o[0] = sharpS[i].mv & 0xFF; o[1] = sharpS[i].mv >> 8; o[2] = sharpS[i].cm; }
  rlSendExtra(s, 20);
}

// OLED: 바닥 색 두 글자 (K 검정 W 흰색 R 빨강 Y 노랑 G 초록 B 파랑, - 센서 없음)
void drawColor(int y) {
  if (!colorS[0].ok && !colorS[1].ok) return;
  oled.setCursor(116, y);
  oled.print(COLOR_NAME[colorS[0].ok ? colorS[0].color : 0]);
  oled.print(COLOR_NAME[colorS[1].ok ? colorS[1].color : 0]);
}

// OLED: 상대 감지 3칸 (왼·가운데·오른, 감지되면 채움)
void drawOpp(int y) {
  if (oppPin[0] == 255 && oppPin[1] == 255 && oppPin[2] == 255) return;   // 상대 감지 센서를 안 쓰면 그리지 않는다
  for (int i = 0; i < 3; i++) {
    int x = 106 + i * 8;
    if (oppPin[i] == 255) { oled.drawLine(x, y + 3, x + 5, y + 3, SSD1306_WHITE); continue; }
    if (oppMask & (1 << i)) oled.fillRect(x, y, 6, 7, SSD1306_WHITE);
    else oled.drawRect(x, y, 6, 7, SSD1306_WHITE);
  }
}

void sharpText(char* out, size_t n) {           // OLED용 짧은 거리 글자 (앞 센서)
  const int i = 0;
  if (sharpS[i].pin == 255) snprintf(out, n, "--");
  else if (sharpS[i].cm == 255) snprintf(out, n, ">80");
  else snprintf(out, n, "%dcm", sharpS[i].cm);
}

void buildInfo() {
  snprintf(robotInfo, sizeof(robotInfo),
           "robot=soccer;fw=roboblock-soccer-0.6;name=%s;FL=%d,%d;FR=%d,%d;BL=%d,%d;BR=%d,%d;oled=%d,%d;boot=0;sharp=%d;scal=%.3f,%.3f;scap=%u,%u;sharpL=%d;scalL=%.3f,%.3f;scapL=%u,%u;sharpR=%d;scalR=%.3f,%.3f;scapR=%u,%u;opp=%d,%d,%d;opph=%d;col=%d,%d,%d,%d;colok=%d,%d",
           robotName, PIN_OF[0][0], PIN_OF[0][1], PIN_OF[1][0], PIN_OF[1][1],
           PIN_OF[2][0], PIN_OF[2][1], PIN_OF[3][0], PIN_OF[3][1], OLED_SDA, OLED_SCL,
           sharpS[0].pin, sharpS[0].a, sharpS[0].b, sharpS[0].farMv, sharpS[0].c20Mv,
           sharpS[1].pin, sharpS[1].a, sharpS[1].b, sharpS[1].farMv, sharpS[1].c20Mv,
           sharpS[2].pin, sharpS[2].a, sharpS[2].b, sharpS[2].farMv, sharpS[2].c20Mv, oppPin[0], oppPin[1], oppPin[2], oppActiveHigh ? 1 : 0,
           OLED_SDA, OLED_SCL, COLOR_R_SDA, COLOR_R_SCL, colorS[0].ok ? 1 : 0, colorS[1].ok ? 1 : 0);
}

void loadMotorPins() {
  uint8_t b[8];
  prefs.begin("x4wd", true);
  bool ok = prefs.getBytesLength("pins") == 8 && prefs.getBytes("pins", b, 8) == 8;
  prefs.end();
  if (!ok) return;
  for (int i = 0; i < 8; i++) if (!motorPinOk(b[i])) return;           // 이상한 값이면 기본 핀 유지
  for (int i = 0; i < 8; i++) for (int j = i + 1; j < 8; j++) if (b[i] == b[j]) return;
  for (int i = 0; i < 4; i++) { PIN_OF[i][0] = b[2 * i]; PIN_OF[i][1] = b[2 * i + 1]; }
}

// 새 핀을 검사해 LEDC 채널에 다시 붙인다. 잘못된 값이면 아무것도 바꾸지 않는다.
bool applyMotorPins(const uint8_t* b, bool save) {
  for (int i = 0; i < 8; i++) if (!motorPinOk(b[i])) { Serial.printf("핀 설정 거부: GPIO%d는 모터에 쓸 수 없음\n", b[i]); return false; }
  for (int i = 0; i < 8; i++) for (int j = i + 1; j < 8; j++) if (b[i] == b[j]) { Serial.println("핀 설정 거부: 같은 핀이 두 번 있음"); return false; }
  brakeAll();
  for (int i = 0; i < 4; i++) for (int k = 0; k < 2; k++) {      // 옛 핀 떼기 → LOW
    ledcDetachPin(PIN_OF[i][k]);
    pinMode(PIN_OF[i][k], OUTPUT);
    digitalWrite(PIN_OF[i][k], LOW);
  }
  for (int i = 0; i < 4; i++) for (int k = 0; k < 2; k++) {      // 새 핀 붙이기
    PIN_OF[i][k] = b[2 * i + k];
    ledcAttachPin(PIN_OF[i][k], CH_OF[i][k]);
  }
  brakeAll();
  if (save) {
    prefs.begin("x4wd", false);
    prefs.putBytes("pins", b, 8);
    prefs.end();
  }
  buildInfo();
  rlSetInfo(robotInfo);
  Serial.printf("모터 핀 변경: FL %d,%d  FR %d,%d  BL %d,%d  BR %d,%d\n", b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);
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
    case 0x0B:
      if (n >= 2) { colorCalStep(d[1]); buildInfo(); rlSetInfo(robotInfo); }
      break;
    case 0x0A:
      if (n >= 5 && oppPinOk(d[1]) && oppPinOk(d[2]) && oppPinOk(d[3])) {
        for (int i = 0; i < 3; i++) oppPin[i] = d[1 + i];
        oppActiveHigh = d[4] != 0;
        uint8_t b[3] = {(uint8_t)oppPin[0], (uint8_t)oppPin[1], (uint8_t)oppPin[2]};
        prefs.begin("x4wd", false); prefs.putBytes("opp", b, 3); prefs.putUChar("opph", oppActiveHigh ? 1 : 0); prefs.end();
        setupOpp(); oppMask = 0; oppPrev = 0;
        buildInfo(); rlSetInfo(robotInfo);
        Serial.printf("상대 감지 센서 핀: %d %d %d (감지 %s)\n", oppPin[0], oppPin[1], oppPin[2], oppActiveHigh ? "HIGH" : "LOW");
      }
      break;
    case 0x09:
      if (n >= 2) {
        int i = n >= 3 && d[2] < 3 ? d[2] : 0;          // 센서 (0 앞, 1 왼쪽, 2 오른쪽)
        SharpS& sh = sharpS[i];
        if (d[1] == 0) { sh.farMv = 0; sh.c20Mv = 0; }
        else if (d[1] == 1) sh.farMv = sh.mv;
        else if (d[1] == 2) sh.c20Mv = sh.mv;
        computeSharpCal(i);
        buildInfo(); rlSetInfo(robotInfo);
      }
      break;
    case 0x08:
      if (n >= 2) {
        int cnt = n >= 4 ? 3 : 1, old[3];
        for (int i = 0; i < 3; i++) { old[i] = sharpS[i].pin; if (i < cnt) sharpS[i].pin = 255; }
        bool ok = true;
        for (int i = 0; i < cnt; i++) { if (sharpPinOk(d[1 + i], i)) sharpS[i].pin = d[1 + i]; else ok = false; }
        if (!ok) for (int i = 0; i < 3; i++) sharpS[i].pin = old[i];   // 하나라도 안 되면 그대로
        saveSharpPins();
        setupSharp();
        for (int i = 0; i < 3; i++) if (!oppPinOk(oppPin[i])) { oppPin[i] = 255; setupOpp(); }
        buildInfo(); rlSetInfo(robotInfo);
        Serial.printf("Sharp 센서 핀: 앞 %d  왼쪽 %d  오른쪽 %d\n", sharpS[0].pin, sharpS[1].pin, sharpS[2].pin);
      }
      break;
    case 0x07:
      if (n >= 9) {
        applyMotorPins(d + 1, true);
        bool clash = false;                 // 모터와 겹친 상대 감지 핀은 끈다
        for (int i = 0; i < 3; i++) if (!oppPinOk(oppPin[i])) { oppPin[i] = 255; clash = true; }
        if (clash) { setupOpp(); buildInfo(); rlSetInfo(robotInfo); }
      }
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
//   8~9: 순번   10~13: millis()   14~15: Sharp 전압(mV)   16: Sharp 거리(cm, 0=없음, 255=80 cm 넘음)   17: 상대 감지(bit0 왼 bit1 가운데 bit2 오른)
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
  s[14] = sharpS[0].mv & 0xFF;  s[15] = sharpS[0].mv >> 8;  s[16] = sharpS[0].cm;
  s[17] = oppMask;
  rlSendStatus(s, 18);
}

void startRoboLink() {
  uint64_t mac = ESP.getEfuseMac();
  char* name = robotName;
  snprintf(robotName, sizeof(robotName), "RoboBlock-%02X%02X", (uint8_t)(mac >> 32), (uint8_t)(mac >> 40));
  buildInfo();
  rlBegin(name, robotInfo);
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
  //   핀은 웹 에디터에서 바꿔 저장한 값(NVS)을 먼저 읽는다. 없으면 기본 핀.
  loadMotorPins();
  loadSharpPin();
  setupSharp();
  loadOppPins();
  for (int i = 0; i < 3; i++) if (!oppPinOk(oppPin[i])) oppPin[i] = 255;
  setupOpp();
  for (int i = 0; i < 4; i++) for (int k = 0; k < 2; k++) { pinMode(PIN_OF[i][k], OUTPUT); digitalWrite(PIN_OF[i][k], LOW); }

  Serial.begin(115200);
  delay(300);
  Serial.println("\nESP32 4WD Xbox 축구로봇 (DRV8871 x4 · 브라운아웃 OFF · 능동 제동) Booting...");

  // PWM 채널 8개 설정 후 각 IN핀에 연결.
  // (STBY가 없으므로 외부 10k 풀다운으로 부팅 중 코스트 보장 — 배선 가이드 참조)
  //   바퀴 i의 IN1/IN2 핀을 채널 CH_OF[i]에 붙인다 (FL=0/1, FR=4/5, BL=2/3, BR=6/7)
  for (int i = 0; i < 4; i++) for (int k = 0; k < 2; k++) {
    ledcSetup(CH_OF[i][k], PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_OF[i][k], CH_OF[i][k]);
  }
  Serial.printf("모터 핀: FL %d,%d  FR %d,%d  BL %d,%d  BR %d,%d\n", PIN_OF[0][0], PIN_OF[0][1], PIN_OF[1][0], PIN_OF[1][1],
                PIN_OF[2][0], PIN_OF[2][1], PIN_OF[3][0], PIN_OF[3][1]);

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
  setupColor();                  // 바닥 색 센서 (OLED I²C를 쓰므로 OLED 다음에)

  Serial.println("Ready. Xbox 컨트롤러를 페어링 모드로 두세요.");
}

// ---------------------------------------------------------------
void loop() {
  bool updated = BP32.update();

  // 모드 전환: 웹 에디터가 붙으면 블록 모드, 떨어지면 조종 모드
  bool web = rlConnected();
  if (web && !webMode) enterWebMode();
  else if (!web && webMode) leaveWebMode();

  if (webMode) {
    uint8_t buf[20], n;
    while (rlPopCommand(buf, &n)) handleWebCommand(buf, n);
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
  readSharp();
  readOpp();
  readColor();
  sendWebStatus();
  sendColorStatus();             // 0xA2: 바닥 색 + 왼쪽·오른쪽 Sharp

#if USE_TFT
  updateTFT();
#endif

  delay(10);                     // 응답성을 위해 약간 더 자주 폴링
}
