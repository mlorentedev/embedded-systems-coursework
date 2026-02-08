# Embedded Systems Engineering

Coursework projects from the **Master en Sistemas Electronicos e Inteligencia Artificial (MSEEI)** program.

## Projects

| Directory | Project | Language | Description |
|-----------|---------|----------|-------------|
| `mobile-apps/android-mortgage-calc` | Mortgage Calculator | Java | Android app for monthly mortgage payment calculations |
| `computer-vision/opencv-projects` | OpenCV Projects | C++ | Image processing: filtering, segmentation, object detection, face recognition |
| `iot-embedded/raspberry-pi` | Raspberry Pi | C++ | GPIO, PWM, threading, MQTT IoT, and sensor interfacing exercises |
| `rtos/msp430f5529` | MSP430 LaunchPad | C | FreeRTOS, I2C, low-power modes, timers, UART on MSP430F5529 |
| `rtos/ti-cc3200` | CC3200 LaunchPad | C | FreeRTOS command server, accelerometer/temperature sensors, UART |
| `fpga/artix7-designs` | FPGA Design Library | Verilog/VHDL | 14 Vivado projects: FSM, UART, OFDM, PicoBlaze, video, FIFO |

## Structure

```
embedded-systems-engineering/
├── mobile-apps/
│   └── android-mortgage-calc/      # Android (Java)
├── computer-vision/
│   └── opencv-projects/            # Image processing (C++)
├── iot-embedded/
│   └── raspberry-pi/               # GPIO, IoT (C++)
├── rtos/
│   ├── msp430f5529/                # TI MSP430 + FreeRTOS (C)
│   └── ti-cc3200/                  # TI CC3200 + FreeRTOS (C)
└── fpga/
    └── artix7-designs/             # Xilinx Vivado (Verilog/VHDL)
```

## Related

- [vicon-fpga-face-detection](https://github.com/manuelguillengrima/vicon-fpga-face-detection) — Master's thesis: VICON system with FPGA-based face detection (Verilog + C#)
