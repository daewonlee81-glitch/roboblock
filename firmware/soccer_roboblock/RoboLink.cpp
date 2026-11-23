// RoboLink — 로보블록 웹 에디터용 BLE GATT 서버 (BTstack)
#include "RoboLink.h"

#include <Arduino.h>
#include <string.h>

#include "btstack.h"
#include "roboblock_gatt.h"

#define CMD_HANDLE     ATT_CHARACTERISTIC_7B1C0001_5A3E_4C2B_9F6D_2B7E0D1A0C00_01_VALUE_HANDLE
#define STATUS_HANDLE  ATT_CHARACTERISTIC_7B1C0002_5A3E_4C2B_9F6D_2B7E0D1A0C00_01_VALUE_HANDLE
#define STATUS_CCC     ATT_CHARACTERISTIC_7B1C0002_5A3E_4C2B_9F6D_2B7E0D1A0C00_01_CLIENT_CONFIGURATION_HANDLE
#define INFO_HANDLE    ATT_CHARACTERISTIC_7B1C0003_5A3E_4C2B_9F6D_2B7E0D1A0C00_01_VALUE_HANDLE

namespace {

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

volatile hci_con_handle_t conHandle = HCI_CON_HANDLE_INVALID;
volatile bool isConnected = false;
volatile bool notifyOn = false;
volatile bool notifyPending = false;
volatile bool notifyQueued = false;   // notifyReg가 BTstack 큐에 들어가 있는 동안 true
volatile bool discQueued = false;

// 명령 큐 (BTstack 태스크 → loop)
struct Cmd { uint8_t len; uint8_t d[20]; };
const int QN = 16;
Cmd queue[QN];
volatile uint8_t qHead = 0, qTail = 0;

uint8_t statusBuf[20];
uint8_t statusLen = 0;

char infoStr[200];
uint8_t advData[31];
uint8_t advLen = 0;
uint8_t scanResp[31];
uint8_t scanLen = 0;

btstack_context_callback_registration_t initReg;
btstack_context_callback_registration_t notifyReg;
btstack_context_callback_registration_t discReg;

uint16_t onRead(hci_con_handle_t, uint16_t handle, uint16_t offset, uint8_t* buffer, uint16_t size) {
  if (handle == STATUS_HANDLE) {
    uint8_t tmp[20];
    uint16_t n;
    portENTER_CRITICAL(&mux);
    n = statusLen;
    memcpy(tmp, statusBuf, n);
    portEXIT_CRITICAL(&mux);
    return att_read_callback_handle_blob(tmp, n, offset, buffer, size);
  }
  if (handle == INFO_HANDLE) {
    return att_read_callback_handle_blob((const uint8_t*)infoStr, strlen(infoStr), offset, buffer, size);
  }
  if (handle == STATUS_CCC) {
    uint8_t v[2] = {(uint8_t)(notifyOn ? 1 : 0), 0};
    return att_read_callback_handle_blob(v, 2, offset, buffer, size);
  }
  return 0;
}

int onWrite(hci_con_handle_t, uint16_t handle, uint16_t mode, uint16_t offset, uint8_t* buffer, uint16_t size) {
  (void)mode;
  if (handle == STATUS_CCC) {
    notifyOn = size >= 2 && little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
    return 0;
  }
  if (handle == CMD_HANDLE) {
    if (offset != 0 || size == 0 || size > 20) return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
    portENTER_CRITICAL(&mux);
    uint8_t next = (qHead + 1) % QN;
    if (next != qTail) {           // 가득 차면 가장 새 명령을 버린다
      queue[qHead].len = size;
      memcpy(queue[qHead].d, buffer, size);
      qHead = next;
    }
    portEXIT_CRITICAL(&mux);
    return 0;
  }
  return 0;
}

void onPacket(uint8_t type, uint16_t, uint8_t* packet, uint16_t) {
  if (type != HCI_EVENT_PACKET) return;
  switch (hci_event_packet_get_type(packet)) {
    case ATT_EVENT_CONNECTED:
      conHandle = att_event_connected_get_handle(packet);
      notifyOn = false;
      notifyPending = false;
      isConnected = true;
      break;
    case ATT_EVENT_DISCONNECTED:
      if (att_event_disconnected_get_handle(packet) == conHandle) {
        conHandle = HCI_CON_HANDLE_INVALID;
        isConnected = false;
        notifyOn = false;
        notifyPending = false;
      }
      break;
    case ATT_EVENT_CAN_SEND_NOW: {
      uint8_t tmp[20];
      uint16_t n;
      portENTER_CRITICAL(&mux);
      n = statusLen;
      memcpy(tmp, statusBuf, n);
      portEXIT_CRITICAL(&mux);
      if (conHandle != HCI_CON_HANDLE_INVALID && notifyOn && n) att_server_notify(conHandle, STATUS_HANDLE, tmp, n);
      notifyPending = false;
      break;
    }
    default:
      break;
  }
}

// BTstack 태스크에서 실행
void doInit(void*) {
  att_server_init(profile_data, onRead, onWrite);
  att_server_register_packet_handler(onPacket);
  bd_addr_t nullAddr = {0};
  gap_advertisements_set_params(0x0030, 0x0060, 0 /* ADV_IND 연결 가능 */, 0, nullAddr, 0x07, 0x00);
  gap_advertisements_set_data(advLen, advData);
  gap_scan_response_set_data(scanLen, scanResp);
  gap_advertisements_enable(1);
}

void doNotify(void*) {
  notifyQueued = false;
  if (conHandle != HCI_CON_HANDLE_INVALID && notifyOn) att_server_request_can_send_now_event(conHandle);
  else notifyPending = false;
}

void doDisconnect(void*) {
  discQueued = false;
  if (conHandle != HCI_CON_HANDLE_INVALID) gap_disconnect(conHandle);
}

}  // namespace

namespace RoboLink {

void begin(const char* advName, const char* info) {
  strncpy(infoStr, info, sizeof(infoStr) - 1);

  // 광고: 플래그 + 이름 (Web Bluetooth에서 "RoboBlock"으로 시작하는 이름으로 찾는다)
  size_t nameLen = strlen(advName);
  if (nameLen > 26) nameLen = 26;
  advLen = 0;
  advData[advLen++] = 2;  advData[advLen++] = BLUETOOTH_DATA_TYPE_FLAGS;  advData[advLen++] = 0x06;
  advData[advLen++] = nameLen + 1;  advData[advLen++] = BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME;
  memcpy(advData + advLen, advName, nameLen);  advLen += nameLen;

  // 스캔 응답: 128비트 서비스 UUID (리틀 엔디언)
  static const uint8_t uuid[16] = {0x00, 0x0C, 0x1A, 0x0D, 0x7E, 0x2B, 0x6D, 0x9F,
                                   0x2B, 0x4C, 0x3E, 0x5A, 0x00, 0x00, 0x1C, 0x7B};
  scanLen = 0;
  scanResp[scanLen++] = 17;  scanResp[scanLen++] = BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS;
  memcpy(scanResp + scanLen, uuid, 16);  scanLen += 16;

  initReg.callback = doInit;
  btstack_run_loop_execute_on_main_thread(&initReg);
}

bool connected() { return isConnected; }

bool popCommand(uint8_t* buf, uint8_t* len) {
  bool ok = false;
  portENTER_CRITICAL(&mux);
  if (qTail != qHead) {
    *len = queue[qTail].len;
    memcpy(buf, queue[qTail].d, *len);
    qTail = (qTail + 1) % QN;
    ok = true;
  }
  portEXIT_CRITICAL(&mux);
  return ok;
}

void sendStatus(const uint8_t* data, uint8_t len) {
  if (len > 20) len = 20;
  portENTER_CRITICAL(&mux);
  memcpy(statusBuf, data, len);
  statusLen = len;
  portEXIT_CRITICAL(&mux);
  if (!isConnected || !notifyOn || notifyPending || notifyQueued) return;
  notifyPending = true;
  notifyQueued = true;
  notifyReg.callback = doNotify;
  btstack_run_loop_execute_on_main_thread(&notifyReg);
}

void disconnect() {
  if (discQueued) return;
  discQueued = true;
  discReg.callback = doDisconnect;
  btstack_run_loop_execute_on_main_thread(&discReg);
}

}  // namespace RoboLink
