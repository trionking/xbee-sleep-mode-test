# Zigbee Sleep Mode 구현 문서

## 개요

리모콘 장치의 저전력 동작을 위해 MCU Stop 모드와 XBee Sleep 모드를 연동하여 구현.

## 하드웨어 구성

### XBee 연결

| MCU 핀 | XBee 핀 | 기능 |
|--------|---------|------|
| PB1 (OT_ZIG_SLEEP) | Pin 9 (SLEEP_RQ) | Sleep 제어 |
| USART2 TX | Pin 3 (DIN) | 데이터 송신 |
| USART2 RX | Pin 2 (DOUT) | 데이터 수신 |

### XBee 역할 구성

| 장치 | XBee 역할 | Sleep |
|------|-----------|-------|
| 리모콘 | End Device | SM=1 (Pin Hibernate) |
| emplate (상대편) | Coordinator | Sleep 불가 |

> 참고: Zigbee에서 Coordinator/Router는 Sleep을 지원하지 않음. End Device만 Sleep 가능.

## XBee 설정 (XCTU)

### 리모콘 (End Device)

```
CE = 0          (End Device)
ID = 0x1234     (PAN ID - 양쪽 동일)
JV = Enabled    (Channel Verification)
SM = 1          (Pin Hibernate)
DH = 0          (Destination High)
DL = FFFF       (Destination Low - Broadcast)
AP = 0          (AT/Transparent mode)
```

### emplate (Coordinator)

```
CE = 1          (Coordinator)
ID = 0x1234     (PAN ID - 양쪽 동일)
NJ = FF         (Node Join Time - 항상 허용)
DH = 0          (Destination High)
DL = FFFF       (Destination Low - Broadcast)
AP = 0          (AT/Transparent mode)
```

## 소프트웨어 구조

### 주요 파일

- `Core/Src/user_def.c` - Sleep 모드, XBee 제어, 스위치 처리
- `Core/Inc/user_def.h` - 함수 선언, 매크로 정의
- `Core/Inc/main.h` - GPIO 핀 정의

### XBee Sleep 제어 함수

```c
// user_def.c

void xbee_sleep_ctl(uint8_t ctrl)
{
    if (ctrl)
    {
        // Wake up: SLEEP_RQ = LOW
        HAL_GPIO_WritePin(OT_ZIG_SLEEP_GPIO_Port, OT_ZIG_SLEEP_Pin, GPIO_PIN_RESET);
    }
    else
    {
        // Sleep: SLEEP_RQ = HIGH
        HAL_GPIO_WritePin(OT_ZIG_SLEEP_GPIO_Port, OT_ZIG_SLEEP_Pin, GPIO_PIN_SET);
    }
}
```

### Sleep/Wake 핀 로직 (SM=1 Pin Hibernate)

| 함수 호출 | GPIO 상태 | XBee 상태 |
|-----------|-----------|-----------|
| `xbee_sleep_ctl(1)` | LOW | Wake (동작) |
| `xbee_sleep_ctl(0)` | HIGH | Sleep (절전) |

### 메인 루프 동작 흐름

```
시작
  │
  ▼
init_proc()
  ├─ HAL_TIM_Base_Start_IT(&htim7)
  ├─ init_UART_COM()
  ├─ bz_ctrl_init()
  └─ xbee_init() → xbee_sleep_ctl(1) → XBee Wake
  │
  ▼
┌─────────────────────────────────────┐
│         enter_sleep_mode()          │
│  ├─ xbee_sleep_ctl(0) → XBee Sleep  │
│  ├─ TIM7, Buzzer 중지               │
│  ├─ HAL_SuspendTick()               │
│  ├─ setup_wakeup_gpio() → EXTI 설정 │
│  └─ HAL_PWR_EnterSTOPMode()         │
│         ↓                           │
│    [MCU Stop 모드 - 저전력 대기]    │
│         ↓                           │
│    [스위치 EXTI → Wake]             │
│         ↓                           │
│  ├─ SystemClock_Config()            │
│  ├─ HAL_ResumeTick()                │
│  ├─ restore_gpio_normal()           │
│  ├─ read_switch() → 스위치 값       │
│  ├─ HAL_TIM_Base_Start_IT(&htim7)   │
│  └─ xbee_sleep_ctl(1) → XBee Wake   │
│  └─ HAL_Delay(500) → 동기화 대기    │
└─────────────────────────────────────┘
  │
  ▼
process_switch_action(sw_val)
  ├─ SW_ON_OFF → xbee_send_on() / xbee_send_off()
  ├─ SW_OPEN   → xbee_send_open()
  └─ SW_CLOSE  → xbee_send_close()
  │
  ▼
wait_switch_release()
  │
  ▼
[루프 반복 → enter_sleep_mode()]
```

### Wake-up 딜레이

```c
// user_def.c:417
xbee_sleep_ctl(1);
HAL_Delay(500);  // XBee wake-up 및 Coordinator 동기화 대기
```

XBee End Device가 Pin Hibernate에서 깨어난 후 Coordinator와 동기화하는 데 시간이 필요함.
- 하드웨어 wake-up: ~13ms
- Coordinator 동기화: 가변 (네트워크 상황에 따라 다름)
- 안정적 동작을 위해 **500ms**로 설정

## 동작 모드 전환

```c
// user_def.c:520
#if 1  // Sleep mode enabled (0=Polling for test, 1=Sleep)
    // Sleep 모드: Stop → Wake → Process → Stop 반복
    while (1)
    {
        sw_val = enter_sleep_mode();
        if (sw_val != 0)
        {
            process_switch_action(sw_val);
            wait_switch_release();
        }
    }
#else
    // Polling 모드: 디버깅/테스트용 (Sleep 없음)
    ...
#endif
```

- `#if 1`: Sleep 모드 활성화 (정상 운용)
- `#if 0`: Polling 모드 (디버깅용, Sleep 없이 동작)

## 전력 소비

| 상태 | 전류 (대략) |
|------|-------------|
| XBee 동작 중 | ~30mA |
| MCU Stop + XBee Sleep | 수십 uA 이하 |

## 주의사항

1. **XBee SLEEP_RQ 핀**: Pin 9에 연결해야 함 (Pin 12 CTS, Pin 13 ON/SLEEP은 출력 핀)

2. **네트워크 연결 순서**: Coordinator 먼저 전원 ON → End Device 리셋

3. **End Device Join 확인**: AI (Association Indication) = 0x00 이어야 정상

4. **SM=1 설정 후 XCTU 연결**: SLEEP_RQ 핀을 GND에 연결해야 XBee가 wake 상태가 되어 XCTU 연결 가능

## 변경 이력

| 날짜 | 내용 |
|------|------|
| 2026-01-28 | XBee Sleep 모드 구현 (SM=1), Wake-up 딜레이 500ms 설정 |
