<div align="center">

# 🖱️ Ring Mouse V2.0

### 차세대 웨어러블 마우스, 당신의 손가락에 혁신을

[![Version](https://img.shields.io/badge/version-2.0-blue.svg)](https://www.futuristec.co.kr)
[![Bluetooth](https://img.shields.io/badge/Bluetooth-5.1-green.svg)](https://www.futuristec.co.kr)
[![Battery](https://img.shields.io/badge/Battery-60mAh-orange.svg)](https://www.futuristec.co.kr)
[![Weight](https://img.shields.io/badge/Weight-9g-lightgrey.svg)](https://www.futuristec.co.kr)

[한국어](#한국어) | [English](#english)

---

</div>

## 한국어

### 📱 제품 소개

**Ring Mouse V2.0**는 퓨처리스텍이 개발한 혁신적인 웨어러블 마우스입니다. 손가락에 착용하는 링 형태로 설계되어, 프레젠테이션, 업무, 그리고 일상 생활에서 자유롭고 편리한 PC/모바일 제어를 가능하게 합니다.

### ✨ 주요 특징

- 🎯 **직관적인 조작**: 터치 & 클릭 방식으로 마우스 커서 이동, 클릭, 스크롤 기능 제공
- 🪶 **초경량 디자인**: 단 9g의 초경량으로 장시간 착용해도 피로감 없음
- 🔋 **긴 배터리 수명**: 완충 시 최대 12시간 연속 사용 가능
- 📡 **Bluetooth 5.1**: 안정적인 무선 연결로 최대 10m 거리 지원
- 🖐️ **왼손/오른손 모드**: 양손 모두 사용 가능한 설정 제공
- 💡 **스마트 LED 표시**: 배터리 상태 및 연결 상태를 한눈에 확인
- 🔄 **숙련자 모드**: 터치 없이도 커서 이동 가능한 고급 모드

### 🎮 작동 방식

#### 기본 동작
| 동작 | 방법 |
|------|------|
| **마우스 커서 이동** | Move 버튼 터치 + 링마우스 움직임 |
| **왼쪽 클릭** | L 버튼 클릭 |
| **오른쪽 클릭** | R 버튼 클릭 |
| **스크롤** | Scroll 버튼 터치 + 상하 기울이기 |
| **뒤로가기/앞으로가기** | Scroll 버튼 터치 + 좌우 움직임 |

### 📊 기술 사양

| 항목 | 사양 |
|------|------|
| **모델명** | RMWTB200 (Black) / RMWTW200 (White) |
| **크기** | 39mm × 40mm × 13mm |
| **무게** | 9g |
| **배터리** | 3.7V / 60mAh 리튬이온 폴리머 |
| **연속 사용** | 완충 시 12시간 |
| **블루투스** | Bluetooth 5.1 |
| **주파수** | 2402MHz ~ 2480MHz |
| **동작 온도** | 10°C ~ 32°C |
| **충전** | USB Type-C |

### 💻 호환성

#### 지원 운영체제
- ✅ **Windows 10** 이상
- ✅ **iPadOS 16** 이상
- ✅ **iOS 16** 이상
- ✅ **Android** (Bluetooth 4.2 이상)

### 🚀 빠른 시작 가이드

#### 1️⃣ 전원 켜기
```
전원 버튼을 짧게 1회 누르면 LED가 깜박이며 연결 대기 상태가 됩니다.
```

#### 2️⃣ 블루투스 연결
```
1. 기기의 블루투스 설정에서 "RingMouse [V2.xx]" 검색
2. 선택하여 페어링 완료
3. LED가 4초마다 짧게 깜박이면 연결 성공
```

#### 3️⃣ 착용 방법
```
- 검지손가락에 링마우스를 착용
- 전원버튼이 위를 향하도록 배치
- 오목한 면이 중지와 맞닿도록 조정
```

#### 4️⃣ 사용 시작
```
Move 버튼을 엄지로 터치 → 손가락을 부채꼴 모양으로 움직이면 커서가 따라 이동합니다!
```

### 🎯 고급 기능

#### 숙련자 모드
터치 없이도 자동으로 커서가 움직이는 모드
```
설정 방법: Move 버튼을 1.2초 이내에 4번 연속 터치
```

#### 왼손 모드
왼손 착용 시 사용하는 모드
```
설정 방법: Move 터치 상태 + R버튼 2초 이상 누르기
```

#### 절전 모드
3분 미사용 시 자동 절전
```
해제 방법: 아무 버튼이나 누르거나 링마우스 움직이기
```

### 🔧 하드웨어 아키텍처

#### MCU 및 센서
- **주 컨트롤러**: nRF52832 (ARM Cortex-M4F)
  - Bluetooth 5.1 통신
  - 64MHz 동작 주파수
  - 512KB Flash, 64KB RAM
- **센서**: 6축 관성 센서 (가속도계 + 자이로스코프)
- **터치 센서**: 정전식 터치 패드 (Move, Scroll 영역)

#### ADC 설정
```c
// ADC Reference 설정
// RingMouse의 최대 ADC 전압: 약 2.5V (Internal Reference + Gain 1/5)
// nRF52832 Spec 문서 P.363 참조

// Internal reference, single ended input, gain 1/6
Input range = (0.6V / (1/6)) = 3.6V

// 실제 사용 설정
ADC_REFERENCE = INTERNAL  // Internal 0.6V reference
ADC_GAIN = 1/6            // Gain 1/6
Input_Range = 3.6V        // 최대 입력 범위
```

#### 전력 관리
```
Battery: 3.7V / 60mAh LiPo
Operating: ~5mA (활성 모드)
Sleep: <100uA (절전 모드)
Charging: USB Type-C (5V / 0.07A)
```

### 🔄 펌웨어 아키텍처

#### 현재 버전
- **Hardware Version**: V2.0
- **Firmware Version**: V2.217
- **빌드 날짜**: 2025.11.28

#### 펌웨어 구조
```
Ring Mouse V2.0 Firmware
├── BLE Stack (Bluetooth 5.1)
│   ├── GAP (Generic Access Profile)
│   ├── GATT (Generic Attribute Profile)
│   └── HID over GATT (마우스 프로필)
├── Sensor Driver
│   ├── IMU (6-axis sensor)
│   ├── Touch Sensor
│   └── Battery Monitor
├── Application Layer
│   ├── Cursor Control
│   ├── Click Handler
│   ├── Scroll Handler
│   └── Mode Manager
└── Power Management
    ├── Sleep Mode
    ├── Battery Monitor
    └── LED Control
```

#### 펌웨어 업데이트 프로토콜

**DFU (Device Firmware Update) 방식**
- Nordic nRF52 DFU 프로토콜 사용
- OTA (Over-The-Air) 업데이트 지원
- Bootloader를 통한 안전한 업데이트

**업데이트 파일 구조**
```
app_ringmouse_v2.xx.zip
├── manifest.json          # 펌웨어 메타데이터
├── ringmouse_v2.xx.dat   # 펌웨어 초기화 패킷
└── ringmouse_v2.xx.bin   # 실제 펌웨어 바이너리
```

**manifest.json 구조**
```json
{
  "manifest": {
    "application": {
      "bin_file": "ringmouse_v2.xx.bin",
      "dat_file": "ringmouse_v2.xx.dat"
    }
  }
}
```

#### Android 업데이트 절차
1. 기존 연결 해제
   ```
   설정 > 연결 > Bluetooth > RingMouse [V2.xx] 삭제
   ```

2. 펌웨어 다운로드
   - 공식 사이트에서 `app_ringmouse_v2.xx.zip` 다운로드

3. nRF DFU 앱 사용
   ```
   - Google Play: "nRF Device Firmware Update" 설치
   - File > Select: 다운로드한 ZIP 파일 선택
   - Device > Select: RingMouse [V2.xx] 선택
   - Progress > Start: 업데이트 시작
   ```

4. 업데이트 프로세스
   ```
   Bootloader enabled → DFU initialized → Uploading (0-100%) → Completed
   ```

#### iOS 업데이트 절차
1. 기존 페어링 해제
   ```
   설정 > Bluetooth > 나의 기기 > RingMouse [V2.xx] 삭제
   ```

2. nRF DFU 앱 사용
   ```
   - App Store: "nRF Device Firmware Update" 설치
   - ZIP 파일 선택 및 장치 페어링
   - 업데이트 진행 및 완료 후 재연결
   ```

### 💻 개발자 정보

#### BLE HID 프로토콜

**서비스 UUID**
```
HID Service: 0x1812
- HID Information: 0x2A4A
- Report Map: 0x2A4B
- HID Control Point: 0x2A4C
- Report: 0x2A4D (Input, Output, Feature)
```

**마우스 리포트 디스크립터**
```c
// HID Report Descriptor (Mouse)
static const uint8_t hid_report_map[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (Button 1)
    0x29, 0x03,        //     Usage Maximum (Button 3)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x75, 0x01,        //     Report Size (1)
    0x95, 0x03,        //     Report Count (3)
    0x81, 0x02,        //     Input (Data, Variable, Absolute)
    0x75, 0x05,        //     Report Size (5)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x01,        //     Input (Constant)
    0x05, 0x01,        //     Usage Page (Generic Desktop)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x09, 0x38,        //     Usage (Wheel)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x03,        //     Report Count (3)
    0x81, 0x06,        //     Input (Data, Variable, Relative)
    0xC0,              //   End Collection
    0xC0               // End Collection
};
```

**마우스 리포트 구조**
```c
typedef struct {
    uint8_t buttons;     // Bit 0: Left, Bit 1: Right, Bit 2: Middle
    int8_t  x;           // X 축 이동 (-127 ~ 127)
    int8_t  y;           // Y 축 이동 (-127 ~ 127)
    int8_t  wheel;       // 스크롤 휠 (-127 ~ 127)
} __attribute__((packed)) mouse_report_t;
```

#### 센서 데이터 처리

**IMU 데이터 읽기**
```c
// 6축 센서에서 각속도 및 가속도 읽기
typedef struct {
    int16_t accel_x, accel_y, accel_z;  // 가속도 (mg)
    int16_t gyro_x, gyro_y, gyro_z;     // 각속도 (dps)
} imu_data_t;

// 센서 데이터를 마우스 이동으로 변환
void process_imu_data(imu_data_t *data, mouse_report_t *report) {
    // 자이로 데이터를 커서 이동으로 매핑
    report->x = (int8_t)(data->gyro_y * SENSITIVITY_X);
    report->y = (int8_t)(data->gyro_x * SENSITIVITY_Y);
}
```

**터치 감지**
```c
// 정전식 터치 센서 상태
typedef enum {
    TOUCH_NONE = 0,
    TOUCH_MOVE = 1,
    TOUCH_SCROLL = 2
} touch_state_t;

touch_state_t read_touch_sensor(void);
```

#### 모드 전환 로직

**기본 모드 → 숙련자 모드**
```c
// Move 버튼을 1.2초 이내에 4번 터치하면 모드 변경
#define MODE_CHANGE_TIMEOUT_MS  1200
#define MODE_CHANGE_TAP_COUNT   4

typedef enum {
    MODE_BASIC = 0,      // Move 터치 시에만 커서 이동
    MODE_EXPERT_1 = 1,   // 1회 터치 후 멈출 때까지 이동
    MODE_EXPERT_2 = 2    // 항상 이동 (터치 시 정지)
} operation_mode_t;

// 모드 변경 시 LED 0.5초 점등으로 확인
void change_mode(void) {
    current_mode = (current_mode + 1) % 3;
    led_blink(500);  // 0.5초 점등
}
```

**왼손 모드 전환**
```c
// Move 터치 + R버튼 2초 누름
#define LEFT_HAND_MODE_HOLD_MS  2000

void toggle_hand_mode(void) {
    if (is_move_touched() && is_r_button_held(LEFT_HAND_MODE_HOLD_MS)) {
        left_hand_mode = !left_hand_mode;
        led_blink(1000);  // 1초 점등
        system_reboot();  // 설정 적용을 위해 재부팅
    }
}
```

#### 전력 관리

**절전 모드 진입**
```c
#define SLEEP_TIMEOUT_MS  180000  // 3분

void power_management_task(void) {
    if (idle_time > SLEEP_TIMEOUT_MS) {
        enter_sleep_mode();
        // LED OFF
        // BLE advertising 중지
        // 센서 polling 중지
    }
}

// Wake-up 조건
// - 버튼 클릭
// - 터치 감지
// - 센서 움직임 감지
```

**배터리 모니터링**
```c
// ADC를 통한 배터리 전압 측정
#define BATTERY_LOW_THRESHOLD_MV   3300  // 3.3V

void battery_monitor_task(void) {
    uint16_t battery_mv = adc_read_battery();
    
    if (battery_mv < BATTERY_LOW_THRESHOLD_MV) {
        // LED 2.6초마다 2번 깜박임
        led_set_pattern(LED_PATTERN_LOW_BATTERY);
    }
    
    if (battery_mv < BATTERY_CRITICAL_MV) {
        // 자동 전원 차단
        power_off();
    }
}
```

### 🔍 디버깅 및 로깅

#### UART 디버그 출력
```c
// nRF52832 UART 설정 (개발용)
#define UART_TX_PIN  6
#define UART_RX_PIN  8
#define UART_BAUD    115200

// 디버그 로그 예시
DEBUG_LOG("IMU: ax=%d, ay=%d, az=%d", accel_x, accel_y, accel_z);
DEBUG_LOG("Touch: state=%d", touch_state);
DEBUG_LOG("Battery: %dmV", battery_voltage);
```

#### LED 디버그 패턴
```c
typedef enum {
    LED_PATTERN_BT_STANDBY,      // 0.8초 주기
    LED_PATTERN_BT_CONNECTED,    // 4초 주기
    LED_PATTERN_LOW_BATTERY,     // 2.6초마다 2번
    LED_PATTERN_MODE_CHANGE,     // 0.5초 점등
    LED_PATTERN_CHARGING,        // 상시 점등
    LED_PATTERN_CHARGED          // OFF
} led_pattern_t;
```

### 💡 LED 상태 표시

| LED 동작 | 의미 |
|---------|------|
| 💚 0.8초 주기로 깜박임 | 블루투스 연결 대기 중 |
| 💚 4초마다 짧게 깜박임 | 블루투스 연결됨 |
| 🔴 계속 켜짐 | 충전 중 |
| 💚 LED 꺼짐 | 충전 완료 |
| 💚 2.6초마다 2번 깜박임 | 배터리 부족 (충전 필요) |

### ⚠️ 주의사항

- 🚫 **절대 분해하지 마세요** - 전파인증 무선기기로 분해 시 인증 효력 상실
- 💧 **물에 닿지 않도록 주의** - 방수 기능 없음
- 🌡️ **적정 온도에서 사용** - 동작 온도: 10°C ~ 32°C
- 🔋 **배터리 관리** - 장기간 미사용 시 주기적으로 충전

### 📞 고객 지원

궁금하신 점이나 문제가 있으신가요?

- 🌐 **공식 웹사이트**: [www.futuristec.co.kr](https://www.futuristec.co.kr)
- 📧 **고객센터**: 홈페이지 고객센터 메뉴 이용
- 📚 **자료실**: 최신 펌웨어 및 상세 매뉴얼 다운로드

### 📜 인증 정보

- **KC 인증**: R-R-fTR-RMWTB200
- **FCC ID**: 2BQZ4-RMWTB200
- **J-MIC**: 210-253328

### 🏆 제조사 정보

**퓨처리스텍 (Futuristec)**
- 📍 주소: 인천 연수구 갯벌로 169, 202-1호
- 📄 버전: 25.1128

---

<div align="center">

### 🎉 Ring Mouse와 함께 새로운 차원의 편리함을 경험하세요!

[![Website](https://img.shields.io/badge/Website-Visit-blue?style=for-the-badge&logo=google-chrome)](https://www.futuristec.co.kr)
[![Support](https://img.shields.io/badge/Support-Contact-green?style=for-the-badge&logo=help-circle)](https://www.futuristec.co.kr)

</div>

---

## English

### 📱 Product Overview

**Ring Mouse V2.0** is an innovative wearable mouse developed by Futuristec. Designed as a ring worn on your finger, it enables free and convenient PC/mobile control for presentations, work, and daily life.

### ✨ Key Features

- 🎯 **Intuitive Control**: Touch & click interface for cursor movement, clicking, and scrolling
- 🪶 **Ultra-Lightweight**: Only 9g - comfortable for extended wear
- 🔋 **Long Battery Life**: Up to 12 hours of continuous use on full charge
- 📡 **Bluetooth 5.1**: Stable wireless connection up to 10m range
- 🖐️ **Left/Right Hand Mode**: Configurable for both hands
- 💡 **Smart LED Indicator**: Easy battery and connection status monitoring
- 🔄 **Expert Mode**: Advanced mode with touchless cursor movement

### 🎮 How to Use

#### Basic Operations
| Action | Method |
|--------|--------|
| **Cursor Movement** | Touch Move button + move ring mouse |
| **Left Click** | Click L button |
| **Right Click** | Click R button |
| **Scroll** | Touch Scroll button + tilt up/down |
| **Back/Forward** | Touch Scroll button + move left/right |

### 📊 Technical Specifications

| Item | Specification |
|------|---------------|
| **Model** | RMWTB200 (Black) / RMWTW200 (White) |
| **Dimensions** | 39mm × 40mm × 13mm |
| **Weight** | 9g |
| **Battery** | 3.7V / 60mAh Lithium-ion Polymer |
| **Runtime** | 12 hours (full charge) |
| **Bluetooth** | Bluetooth 5.1 |
| **Frequency** | 2402MHz ~ 2480MHz |
| **Operating Temp** | 10°C ~ 32°C |
| **Charging** | USB Type-C |

### 💻 Compatibility

#### Supported OS
- ✅ **Windows 10** or higher
- ✅ **iPadOS 16** or higher
- ✅ **iOS 16** or higher
- ✅ **Android** (Bluetooth 4.2 or higher)

### 🚀 Quick Start Guide

#### 1️⃣ Power On
```
Press the power button once briefly. LED will blink indicating connection standby.
```

#### 2️⃣ Bluetooth Connection
```
1. Search for "RingMouse [V2.xx]" in device Bluetooth settings
2. Select to pair
3. LED blinks briefly every 4 seconds when connected
```

#### 3️⃣ How to Wear
```
- Wear ring mouse on index finger
- Position power button facing up
- Adjust concave side to touch middle finger
```

#### 4️⃣ Start Using
```
Touch Move button with thumb → Move finger in fan shape and cursor follows!
```

### 🎯 Advanced Features

#### Expert Mode
Automatic cursor movement without touch
```
Setup: Touch Move button 4 times within 1.2 seconds
```

#### Left-Hand Mode
For left-hand users
```
Setup: Touch Move + Hold R button for 2 seconds
```

#### Power Saving Mode
Auto sleep after 3 minutes of inactivity
```
Wake: Press any button or move ring mouse
```

### 💡 LED Status Indicators

| LED Pattern | Meaning |
|------------|---------|
| 💚 Blinks every 0.8s | Bluetooth standby |
| 💚 Brief blink every 4s | Bluetooth connected |
| 🔴 Stays on | Charging |
| 💚 Off | Fully charged |
| 💚 2 blinks every 2.6s | Low battery (charge needed) |

### ⚠️ Safety Precautions

- 🚫 **Do not disassemble** - FCC certified wireless device
- 💧 **Keep away from water** - Not waterproof
- 🌡️ **Use in proper temperature** - Operating temp: 10°C ~ 32°C
- 🔋 **Battery management** - Charge periodically if unused for extended periods

### 📞 Customer Support

Questions or issues?

- 🌐 **Official Website**: [www.futuristec.co.kr](https://www.futuristec.co.kr)
- 📧 **Customer Service**: Use contact menu on website
- 📚 **Downloads**: Latest firmware and detailed manuals

### 📜 Certification

- **KC**: R-R-fTR-RMWTB200
- **FCC ID**: 2BQZ4-RMWTB200
- **J-MIC**: 210-253328

### 🏆 Manufacturer

**Futuristec**
- 📍 Address: 169 Gaetbeol-ro, Yeonsu-gu, Incheon, 202-1ho
- 📄 Version: 25.1128

---

<div align="center">

### 🎉 Experience a New Dimension of Convenience with Ring Mouse!

[![Website](https://img.shields.io/badge/Website-Visit-blue?style=for-the-badge&logo=google-chrome)](https://www.futuristec.co.kr)
[![Support](https://img.shields.io/badge/Support-Contact-green?style=for-the-badge&logo=help-circle)](https://www.futuristec.co.kr)

**© 2025 Futuristec. All rights reserved.**

</div>
