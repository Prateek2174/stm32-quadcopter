# STM32F446RE Quadcopter Flight Controller

A bare-metal quadcopter flight controller written in C for the STM32F446RE microcontroller. All peripheral drivers are implemented directly against hardware registers — no STM32 HAL, no third-party sensor libraries.

> **Status:** Active development — drivers complete, flight loop integration in progress.

---

## Hardware

| Component | Part | Interface |
|---|---|---|
| Microcontroller | STM32F446RE (Cortex-M4, 180 MHz) | — |
| IMU | MPU-6050 (6-axis accel + gyro) | I2C1 (PB6/PB7) |
| Magnetometer | LIS3MDL (3-axis) | I2C1 (PB6/PB7) |
| Motor PWM | 4x ESC via TIM1 CH1–CH4 | PA8/PA9/PA10/PA11 |
| Debug UART | USART2 | PA2 (TX) / PA3 (RX) |

### Motor Layout (X-configuration)

```
    Front
  M1(CCW)  M2(CW)
     \      /
      [CTRL]
     /      \
  M4(CW)  M3(CCW)
```

| Motor | Position | Spin | TIM1 Channel | Pin |
|---|---|---|---|---|
| M1 | Front-Left | CCW | CH1 | PA8 |
| M2 | Front-Right | CW | CH2 | PA9 |
| M3 | Rear-Right | CCW | CH3 | PA10 |
| M4 | Rear-Left | CW | CH4 | PA11 |

---

## Firmware Architecture

```
main.c
├── gpio.c          — GPIO alternate function configuration (I2C, TIM1, USART2)
├── i2c.c           — Bare-metal I2C master driver (standard mode, 100 kHz)
│   ├── mpu6050.c   — MPU-6050 driver (accel, gyro, temp, FIFO, orientation)
│   └── lis3mdl.c   — LIS3MDL magnetometer driver
├── pwm.c           — TIM1 4-channel PWM driver for ESC control
├── pid.c           — PID controller (trapezoidal integrator, filtered derivative, anti-windup)
└── usart.c         — USART2 debug output driver
```

---

## Module Breakdown

### `i2c.c` — I2C Master Driver
Implements the full I2C master protocol at the register level against the STM32F446 I2C peripheral (RM0390 §27).

- Software reset on init
- Standard mode (100 kHz), peripheral clock = 16 MHz, CCR = 0x50, TRISE = 17
- Single-byte and multi-byte read with proper repeated start sequence
- Helper functions: `i2c_set_bits`, `i2c_clear_bits`, `i2c_update_bitfield` for clean register-level sensor configuration
- Generic `i2c_general_*` API for any I2C peripheral (I2C1/2/3)

### `mpu6050.c` — IMU Driver
Full-featured driver for the InvenSense MPU-6050 6-axis IMU over I2C.

- Initialization: device reset, clock source selection (X-gyro PLL), DLPF configuration, full-scale range setup, FIFO enable/reset
- Configurable sample rate via SMPLRT_DIV and DLPF mode (default: 200 Hz)
- FIFO-based burst reads for accel, gyro, and temperature
- Gyroscope full-scale: ±2000°/s (FS_SEL = 3)
- Accelerometer full-scale: ±8g (AFS_SEL = 2)
- Pitch and roll computation from accelerometer using `asinf` / `atan2f`
- Low-power accel-only mode with configurable wake-up frequency (1.25 / 5 / 20 / 40 Hz)
- Per-axis standby control for gyro and accelerometer

### `lis3mdl.c` — Magnetometer Driver
Driver for the ST LIS3MDL 3-axis magnetometer over I2C.

- WHO_AM_I verification on init
- Configurable XY/Z performance modes (low / medium / high / ultra-high)
- Output data rate: 80 Hz (DO = 7)
- Full-scale: ±4 gauss (FS = 0)
- Continuous conversion mode
- Status register polling for data-ready before read

### `pid.c` — PID Controller
Three independent PID controllers for pitch, roll, and yaw axes.

- **Proportional:** standard `kp * error`
- **Integral:** trapezoidal integration (`0.5 * ki * Ts * (e[k] + e[k-1])`)
- **Derivative:** band-limited differentiator on measurement (not error) to avoid derivative kick on setpoint changes:
  ```
  D[k] = (-2*kd*(y[k] - y[k-1]) + (2*tau - Ts)*D[k-1]) / (2*tau + Ts)
  ```
- **Anti-windup:** dynamic integrator clamping — integrator limits shrink as proportional term approaches output saturation
- Configurable output limits, sample time, and derivative filter time constant

### `pwm.c` — Motor PWM Driver
4-channel PWM output via TIM1 (advanced-control timer) for ESC control.

- All 4 channels initialized simultaneously (CH1–CH4)
- PWM Mode 1, edge-aligned, active-high
- Preload enabled on ARR and CCRx for glitch-free duty cycle updates
- MOE (main output enable) set for TIM1
- Runtime duty cycle update via `pwm_update_duty_cycle(TIMx, channel, duty)`
- Generic `pwm_general_out_init` for TIM2–TIM5

### `gpio.c` — GPIO Configuration
Single `gpio_init()` call configures all peripheral pins:

| Pin | Function | Config |
|---|---|---|
| PA8–PA11 | TIM1 CH1–CH4 (motor PWM) | AF1, push-pull |
| PB6 | I2C1 SCL | AF4, open-drain |
| PB7 | I2C1 SDA | AF4, open-drain |
| PA2 | USART2 TX | AF7 |
| PA3 | USART2 RX | AF7 |

### `usart.c` — Debug UART
USART2 at 115200 baud for debug output.

- 8-bit data, 1 stop bit, no parity
- Blocking transmit and receive
- Generic `usart_general_*` API supporting USART1/2/3/6

---

## Motor Mixing

Pitch, roll, and yaw PID outputs are mixed into per-motor throttle commands:

```c
motor1 = throttle + pitch + roll - yaw;  // Front-Left  (CCW)
motor2 = throttle + pitch - roll + yaw;  // Front-Right (CW)
motor3 = throttle - pitch - roll - yaw;  // Rear-Right  (CCW)
motor4 = throttle - pitch + roll + yaw;  // Rear-Left   (CW)
```

---

## Build

The project is configured for **STM32CubeIDE** (Eclipse-based, ARM GCC toolchain).

1. Clone the repo and open STM32CubeIDE
2. `File → Import → Existing Projects into Workspace`
3. Select `stm32-quadcopter/stm32-quadcopter`
4. Build with `Ctrl+B` (Debug configuration)
5. Flash via ST-LINK with `Run → Debug`

**Toolchain:** `arm-none-eabi-gcc`
**Linker script:** `STM32F446RETX_FLASH.ld`

---

## Roadmap

- [x] I2C master driver
- [x] MPU-6050 driver (accel, gyro, FIFO, orientation)
- [x] LIS3MDL magnetometer driver
- [x] 4-channel TIM1 PWM driver
- [x] PID controller with anti-windup
- [x] GPIO alternate function configuration
- [x] USART2 debug driver
- [ ] I2C timeout/error recovery
- [ ] Complementary filter / Madgwick sensor fusion (yaw)
- [ ] RC receiver input (PWM capture or SBUS)
- [ ] ESC arming sequence
- [ ] Failsafe logic
- [ ] Flight loop integration
- [ ] PID tuning

---

## Author

Prateek Singh — [GitHub](https://github.com/Prateek2174)
