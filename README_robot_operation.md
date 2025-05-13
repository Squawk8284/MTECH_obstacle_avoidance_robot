# Robot Operation Guide

This README provides an overview of the hardware and software setup, usage instructions, and key specifications for operating the 0X Delta 4 Wheel Drive Robot.

---

## 🛠️ Hardware Overview

The **0X Delta Robot** is a rugged, all-terrain, differentially steered robot designed for indoor and outdoor robotics applications.

### Key Specifications

- **Drive**: 4-wheel differential drive
- **Size**: 934mm × 670mm × 339mm (without payload)
- **Ground Clearance**: 90mm
- **Max Speed**: 2 m/s (with 3 kg payload)
- **Battery Life**: ≥ 2 hours at full operation
- **Payload Capacity**: 30 kg
- **Sensors** (optional): Ultrasonic (8x), IR Range (8x), IMU, GPS, Camera, LRF
- **Communication**: USB, 2.4GHz Wireless, WiFi (optional), Manual Remote Control (optional)

---

## ⚙️ Software Features

- **Communication**: USB Serial (57600 baud), 2.4 GHz wireless
- **Programming Interfaces**: C/C++, Python, Scilab
- **Command Protocol**: Custom serial command-response protocol (prefixed by `NEX`)
- **GUI**: A Windows GUI tool is available for real-time control and diagnostics.

---

## 🔌 Getting Started

1. **Powering Up**:
   - Use illuminated switches on the back panel (`ROBOT POWER`, `COMPUTE POWER`).
   - Ensure correct Li-Po or Li-ion battery connection via XT-90 or XT-60 plugs.

2. **Communication Setup**:
   - **USB**: Connect the USB COMMUNICATION cable to the onboard PC or external laptop.
   - **Wireless**: Insert 2.4GHz dongle (ensure ID matches robot).
   - **Manual Remote**: Toggle `REMOTE CONTROL OVERRIDE` for RC mode.

3. **Motion Control**:
   - Set linear/angular velocities using API commands or GUI.
   - Real-time encoder-based motion tracking.

---

## 🔋 Charging Instructions

- Use B6AC charger (included).
- Charge in **balanced mode** only.
- Max current: **1.5A**.
- For Li-Po (6-cell): Do **not** allow below 19.8V.
- For Li-ion (3/4-cell): Do **not** allow below 9.9V / 13.2V.

---

## 📡 Sensor Operation

### Ultrasonic & IR Sensors

- Enable/Disable via software commands.
- Retrieve readings from 8 sensor banks.
- Ultrasonic blind spot (0–6 inch) covered using IR sensors.

### Inertial Measurement Unit

- Access raw 3-axis accelerometer, gyroscope, and magnetometer data via serial commands.

---

## 💻 Command Format

Commands use the format:

```
'N' 'E' 'X' <Command> <Sub-command/Data> <Checksum>
```

Example:
```bash
HEX: 4E 45 58 70 01 62 42
```
Where `42` is checksum (1's complement + 1).

---

## 🧯 Safety Guidelines

- Use in static-free environments.
- Avoid moisture/water.
- Do not wear loose clothing while operating.
- Always monitor battery voltage warnings.
- Never leave powered robot unattended.

---

## 🗑️ Recycling Notice

Recycle all components responsibly after end of use.
