# 스모 레이더 모니터 패널 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 웹 에디터 "센서 확인" 창 맨 위에, 로봇을 위에서 본 레이더 그림(센서 3개 시야 + 감지된 물체 + 바닥 IR)을 넣고, 연결 전에는 끌어 놓는 가상 상대 로봇으로 센서 값을 만들어 블록 프로그램을 시험할 수 있게 한다.

**Architecture:** 단일 파일 에디터(index.html)에 캔버스 패널 하나를 추가한다. 로봇 기준 좌표(cm, 앞 = +y, 오른쪽 = +x)로 센서 위치·방향을 상수로 두고, 가상 상대(15×15 cm 상자)에 대해 센서마다 광선 교차로 거리를 계산해 Sharp 장치의 가상 값(R(d).value)에 넣는다. 로봇과 연결되어 실제 값이 오는 센서는 실제 거리를 점으로 표시한다.

**Tech Stack:** 바닐라 JS + Canvas 2D (기존 에디터와 같음), Playwright로 가짜 로봇 시험.

**Spec:** 대화에서 승인한 설계 (2026-11-26): 연결 시 실시간 감지 표시, 미연결 시 가상 상대 끌어 놓기, 스모 "센서 확인" 창 맨 위, 폰 폭 지원, 로봇 움직임 시뮬레이션은 하지 않음.

## Global Constraints
- 센서 기하(cm, 하판 가운데 원점): 앞 (0, 8.0) 0°, 왼쪽 (−7.3, 8.1) −45°, 오른쪽 (+7.3, 8.1) +45°; 바닥 IR (±7.95, 9.5); 날 끝 y = 12.9, 날 폭 18.8.
- Sharp 거리: 10 cm 미만은 10, 80 cm 넘으면 "앞에 없음"(far). 감지 기준 70 cm (기존 블록과 같음).
- 가상 상대 크기 15 × 15 cm, 기본 위치 (0, 40). 위치·켜짐은 localStorage 'rb.radar'에 (try/catch).

## Review Focus
- 폰(390 px)에서 캔버스를 끌 때 페이지가 같이 스크롤되면 안 됨 → touch-action: none, 가로 넘침 0.
- 로봇 연결 중에는 가상 상대가 실제 값을 덮어쓰면 안 됨 → 실제 센서는 가상 값 적용 제외.
- 가상 상대를 끄면 슬라이더 값이 다시 쓰여야 함.
- 상대가 센서 10 cm 안에 있을 때 0 cm가 아닌 10 cm.
- 다크 모드에서 색이 CSS 변수를 따라야 함.

---

### Task 1: 레이더 패널 + 가상 상대
**Files:** Modify `index.html` (원본: scratchpad roboblock_editor.html → build_index.py)
- [x] HTML: `#sensorCard` 안 맨 위 `<div class="sec" id="radarBox">` (canvas#radar, button#radarToggle, .meta#radarHint)
- [x] JS: `RADAR_GEOM`, `radarRayBox(ox, oy, a, box) -> number|null`, `virtualDistances() -> {C,L,R}`, `applyRadarVirtual()`, `drawRadar(now)`; 포인터로 상대 이동
- [x] `updateSensorCard`에서 showSharp일 때 호출
- [x] Playwright: 미리보기 (0,40) → 앞 약 24.5 cm, 좌우 >80; 왼쪽 (−30,30)으로 옮기면 왼쪽 감지 → 스모 예제 바퀴 −45/+45; 연결 시 실제 점 표시; 폰 넘침 0; 오류 0
- [x] 빌드, 아티팩트·맥 저장소 반영, 커밋
