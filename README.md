# PID Water Level Control System

This project implements a **PID-controlled water level management system** using an ultrasonic sensor for real-time water level measurements and automatic pump control. It was designed and tested on AVR-based microcontrollers.

---

## Project Overview

- **Ultrasonic sensor** measures the water level inside a tank.
- **PID controller** adjusts water input/output based on the set target water level.
- **LED indicators** show whether the system is adding or removing water.
- **Serial Communication (USART)** transmits real-time data (water level, error, PID output) for external monitoring.

The system ensures the water level remains close to the setpoint, dynamically adjusting pump behavior based on sensor feedback.

### Explanation and Demo Video
Check out the video [here](https://www.youtube.com/watch?v=zpB85U1-LSw).

### Project Poster
View the full project poster [here](https://github.com/jarkin0513/PID-Embedded-Systems/blob/main/docs/Poster.pdf).


---

## System Details

- **Microcontroller**: AVR (ATmega328P, 16 MHz clock)
- **Sensor**: Ultrasonic distance sensor (HC-SR04)
- **Target Water Level**: Adjustable from 10 mm to 330 mm
- **PID Parameters**:
  - `Kp`: 1.0
  - `Ki`: 0.1
  - `Kd`: 0.05
  - **Sampling Time**: 0.48 s
- **Communication**: 9600 baud rate serial interface (USART)

---

## How It Works

1. **Initialization**:
   - Timers and interrupts for sensor operation and timing.
   - USART is initialized for serial data communication.
   - The PID controller is initialized with the specified tuning parameters.

2. **Operation**:
   - The ultrasonic sensor measures the distance to the water surface.
   - Compute the PID control output based on the difference between measured level and target level.
   - Water level is calculated and averaged over several samples for stability.
   - User adjusts the target water level in real-time using the Rotary Pulse Generator (RPG).
   - The PID controller calculates the necessary control output:
       - If the output is positive → Add water (red LED ON).
       - If the output is negative → Remove water (green LED ON).
       - If close to setpoint → No pump operation (both LEDs OFF).
   - Real-time water level, error, PID output, and target level are transmitted over serial.

---

## Build and Flash Instructions

1. **Requirements**:
   - AVR Toolchain (`avr-gcc`, `avrdude`)
   - Microcontroller programmer (USBasp, Arduino as ISP, etc.)

2. **Compiling**:
   ```bash
   avr-gcc -mmcu=atmega328p -DF_CPU=16000000UL -o pid_water_level.elf src/*.c
   avr-objcopy -O ihex pid_water_level.elf pid_water_level.hex

3. **Flashing**:
```bash
avrdude -c usbasp -p m328p -U flash:w:pid_water_level.hex
```

4. **Serial Monitoring**:
Connect a USB-to-Serial adapter and use any terminal program (e.g., PuTTY, screen, Arduino Serial Monitor) at 9600 baud.

---

## Potential Improvements
- Add LCD Display support for real-time water level visualization.
- Implement low power modes when idle to conserve energy.
- Enhance noise filtering for the ultrasonic measurements.
- Debounce RPG input more robustly.

---

## License
This project is licensed under the [MIT License](https://opensource.org/license/mit).

