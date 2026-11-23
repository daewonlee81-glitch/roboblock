// RoboLink — 로보블록 웹 에디터와 Web Bluetooth(BLE GATT)로 통신
//
// Bluepad32는 BTstack 위에서 돈다. RoboLink는 같은 BTstack에 GATT 서버(peripheral)를
// 올린다. Bluepad32의 자체 BLE 설정 서비스는 꺼 둔 상태(기본값)여야 한다.
//
// 스레드: BTstack 콜백은 BTstack 태스크에서, Arduino loop()는 다른 태스크에서 돈다.
//   - 받은 명령은 큐에 넣고 loop()에서 popCommand()로 꺼낸다.
//   - 상태 알림은 sendStatus()가 BTstack 태스크로 넘겨서 보낸다.
#pragma once
#include <stdint.h>

namespace RoboLink {

// GATT 서버와 광고를 시작한다. BP32.setup() 뒤에 한 번 부른다.
//   advName: 광고 이름 (예: "RoboBlock-A1B2", 최대 20자)
//   info:    정보 특성에서 읽히는 문자열 (로봇 종류·펌웨어·핀)
void begin(const char* advName, const char* info);

// 웹 에디터가 연결되어 있는가
bool connected();

// 받은 명령 하나를 꺼낸다. 없으면 false. buf는 최소 20바이트.
bool popCommand(uint8_t* buf, uint8_t* len);

// 상태 알림 (최대 20바이트). 연결되어 알림이 켜져 있을 때만 실제로 보낸다.
void sendStatus(const uint8_t* data, uint8_t len);

// 연결을 끊는다 (예: BOOT 버튼으로 조종 모드 복귀)
void disconnect();

}  // namespace RoboLink
