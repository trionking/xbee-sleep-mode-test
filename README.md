# XBee Sleep Mode Remote Control

STM32G4 기반 Zigbee 리모콘 프로젝트. MCU Stop 모드와 XBee Sleep 모드를 연동하여 저전력 동작 구현.

## 하드웨어

- MCU: STM32G431CBT6
- Zigbee: XBee S2C (End Device, SM=1 Pin Hibernate)
- 통신: UART2 (9600 baud)

## 기능

- 버튼 입력으로 Zigbee 명령 전송 (on/off/open/close)
- MCU Stop 모드 + XBee Sleep 모드로 저전력 대기
- 버튼 EXTI로 wake-up 후 명령 전송

## XBee 설정

| 파라미터 | 값 | 설명 |
|----------|-----|------|
| CE | 0 | End Device |
| SM | 1 | Pin Hibernate |
| JV | Enabled | Channel Verification |

## 핀 연결

| MCU | XBee | 기능 |
|-----|------|------|
| PB1 | Pin 9 (SLEEP_RQ) | Sleep 제어 |
| USART2 TX | Pin 3 (DIN) | 데이터 송신 |
| USART2 RX | Pin 2 (DOUT) | 데이터 수신 |

## 문서

상세 문서는 [doc/DOC_ZIGBEE_SLEEP.md](doc/DOC_ZIGBEE_SLEEP.md) 참고.
