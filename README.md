# 로보블록 (RoboBlock)

ESP32 로봇을 블록으로 코딩하는 교육용 웹 에디터입니다. 블록을 실행하면 ESP32의 어떤 선(PWM·HIGH/LOW·펄스·I²C·아날로그)에 신호가 나가는지 배선도에서 바로 보입니다.

- **지원 로봇:** 축구로봇(4WD 스키드), 미들봇(메카넘 + EEZYbotARM 흡착), 스모
- **보드:** NodeMCU-32S (ESP32-WROOM-32, 38핀)
- **조종기:** Xbox 무선 컨트롤러 (Bluepad32)

## 기능 (프로토타입 v5)

- 레벨 1~3: 아이콘 블록 → 일반 블록 → C++ 코드 보기 (Blockly 11.2.1)
- ESP32 배선도: ESP32 → 드라이버 모듈(DRV8871, A4988, PCA9685, XY-MOS) → 모터·서보·펌프·밸브, 그리고 전원 배선
- 신호 보기: 실행 중 활성화된 선이 빛나고, PWM은 흐르는 점선, 모터 아이콘은 회전 방향대로 돈다
- 장치 추가와 핀 지정: DC·엔코더·스테핑·서보 모터, 센서 9종, PCA9685 채널 서보. 플래시 핀·입력 전용·ADC2·스트래핑 핀, I²C 주소 충돌, PWM 채널 수를 바로 검사한다
- 실시간 모니터: 장치마다 현재 값과 최근 6초 그래프. 센서는 가상 입력 슬라이더로 시험한다
- 가상 Xbox 컨트롤러: 각 로봇 펌웨어와 같은 조작 배치. 보기 + 메뉴를 함께 누르면 비상 정지

아직 실제 로봇 연결(BLE·Wi-Fi)은 없습니다. 로봇 펌웨어(바이트코드 인터프리터)가 준비되면 같은 화면에 실제 신호가 들어옵니다.

## 실행

빌드가 필요 없는 단일 HTML 파일입니다.

```bash
# 로컬에서
python3 -m http.server 8000
# 브라우저에서 http://localhost:8000 열기
```

Blockly와 글꼴은 CDN(jsdelivr, Google Fonts)에서 불러오므로 인터넷 연결이 필요합니다.

## GitHub Pages로 서비스하기

1. 저장소 **Settings → Pages**
2. **Source: Deploy from a branch**, **Branch: main / (root)** 선택 후 저장
3. 1~2분 뒤 `https://<아이디>.github.io/roboblock/` 에서 열린다

Pages는 https로 서비스되므로, 나중에 크롬·엣지·안드로이드에서 Web Bluetooth로 로봇에 바로 연결할 수 있습니다.

## 파일

| 파일 | 내용 |
|---|---|
| `index.html` | 에디터 전체 (HTML·CSS·JS 한 파일) |
| `.nojekyll` | GitHub Pages가 파일을 그대로 서비스하게 함 |

학생이 만든 블록과 장치 구성은 각자의 브라우저(localStorage)에만 저장됩니다.
