# CiA 402 Homing Mode Interactive Simulator

<p align="center">
  <b>English</b> &nbsp;|&nbsp; <a href="README.zh-CN.md">简体中文</a>
</p>

<p align="center">
  <a href="https://zhengyong1023.github.io/cia402-homing-simulator/" target="_blank">
    <img src="https://img.shields.io/badge/Live%20Demo-Online%20Simulator-brightgreen?style=flat-square&logo=google-chrome&logoColor=white" alt="Live Demo">
  </a>
  <a href="LICENSE">
    <img src="https://img.shields.io/badge/License-MIT-blue.svg?style=flat-square" alt="License: MIT">
  </a>
</p>

An interactive, web-based simulation tool designed for motion control engineers, firmware developers, and automation specialists to visualize, study, and verify **CiA 402 (CANopen / EtherCAT)** homing methods and state-machine transitions.

---

## 📌 Overview

The **CiA 402 Homing Mode Simulator** provides a visual physics-and-logic testbed for standard homing routines defined in the **CiA 402 (CANopen device profile for drives and motion control)** and **ETG.6010** specifications.

Implementing and debugging homing methods on physical servo drives can be time-consuming, difficult to visualize, and potentially hazardous to mechanical fixtures. This tool simulates realistic motion dynamics (acceleration, deceleration, velocity changes, edge-detection latches, and mechanical stall) entirely in modern web browsers with **zero external software dependencies**.

---

## ✨ Key Features

- **Comprehensive Homing Method Coverage**:
  - **Torque / Stall Homing (Methods -6 to -1)**: Low-speed / high-speed hard stop detection, torque release back-off, and stall-followed-by-index positioning.
  - **Limit Switch + Index Pulse (Methods 1, 2)**: Reversing upon hitting limits, detecting limit switch edges, and latching encoder Z-phase (Index) pulses.
  - **Home Switch + Index Pulse (Methods 3 to 6)**: Searching for home switch edges both inside and outside the switch area with index latching.
  - **Combined Limit + Home Switch + Index (Methods 7 to 14)**: Full directional reversals upon hitting limit switches while searching for the home switch and Z-phase.
  - **Switch Edge Detection Without Index (Methods 17 to 30)**: Limit and home switch edge-only routines without requiring an encoder Z pulse.
  - **Direct Positioning & Index Only (Methods 33, 34, 35/37)**: Pure index pulse searches and direct coordinate zeroing without physical motion.

- **Real-Time Physics & Sensor Simulation**:
  - Smooth trapezoidal acceleration and deceleration profiles.
  - Dynamic simulation of **Negative Limit (`NOT / L-`)**, **Positive Limit (`POT / L+`)**, **Home Switch (`HOME`)**, **Encoder Z-phase (`INDEX`)**, and **Mechanical Stall (`STALL`)**.
  - Visual status LEDs and position-calibrated ruler overlay.

- **Diagnostic Panel & Event Logging**:
  - High-frequency telemetry: actual position, velocity, direction, signal levels, and state-machine status (`IDLE`, `SEARCH_LIMIT`, `SEARCH_HOME`, `SEARCH_INDEX`, `BACK_OFF`, `HOMED`, `ERROR`).
  - Interactive event log with precision timestamps recording every state transition and latch event.

- **Clean Decoupled Architecture**:
  - Completely decoupled pure JavaScript logic layer (`homing-logic.js`) with zero DOM dependencies, suitable for porting or embedding into other testing frameworks.
  - Modern, dark-themed responsive UI styled with Tailwind CSS.

- **Zero-Install, Pure Client-Side**:
  - Runs directly in Google Chrome, Microsoft Edge, Mozilla Firefox, or Safari.
  - No backend server, Node.js runtime, or compilation pipeline required.

---

## 🚀 Quick Start

### Option 1: Direct File Opening
1. Clone or download this repository.
2. Locate [`src/cia402-homing-simulator.html`](src/cia402-homing-simulator.html).
3. Double-click to open it in any modern web browser.

### Option 2: Run via Local HTTP Server (Recommended)
If your browser restricts local script execution or CDN resources under `file://`, serve the directory using a lightweight HTTP server:

```bash
# Using Python 3
python -m http.server 8080

# Or using Node.js / npx
npx serve .
```
Then navigate to `http://localhost:8080/src/cia402-homing-simulator.html`.

---

## 🕹️ User Interface & Operation Guide

### 1. Motion Axis Stage
- **Motor Carriage (Blue Block)**: Visualizes real-time position and travel across the mechanical range ($0 \sim 1000\text{ u}$).
- **Sensor Indicators (Top LEDs)**:
  - 🔴 **POT (L+)**: Positive hardware limit active.
  - 🔴 **NOT (L-)**: Negative hardware limit active.
  - 🟢 **HOME**: Home switch signal active.
  - 🟡 **Z (INDEX)**: Periodic encoder zero-pulse (Index) latch active.
  - 🟣 **STALL**: Mechanical hard stop or torque threshold reached.

### 2. Control Panel
- **Homing Method**: Dropdown menu categorizing methods by signal type and standard method ID.
- **Initial Position**: Preset slider starting positions (`Left of Home`, `On Home Switch`, `Right of Home`, `Near Limits`, `On Limits`).
- **High Speed (`HomingVelHigh`)**: Velocity used when traversing large distances toward switches ($10 \sim 200\text{ u/s}$).
- **Low Speed (`HomingVelLow`)**: Creep velocity used for precision edge-finding and index latching ($5 \sim 100\text{ u/s}$).
- **Buttons**:
  - `▶ Start`: Initiates the homing sequence according to the selected method.
  - `↺ Reset`: Restores the initial conditions and stops motion.

### 3. Diagnostics & Telemetry
- **Diagnostics Panel**: Real-time display of position coordinate ($u$), velocity ($u/s$), moving direction, raw sensor levels, CiA 402 master state, and sub-state.
- **Event Log**: Chronological list of events (accelerating, signal edge detected, reversing direction, target latched, homing completed/failed).

---

## 📋 Supported CiA 402 Homing Methods Summary

> 📘 **Diagrams & Detailed Timing**: For visual timing charts and step-by-step physical breakdown of each homing routine, refer to the [CiA 402 Homing Methods Illustrated Guide (Chinese)](docs/homing-methods.zh-CN.md).

| Category | Method ID | Description |
| :--- | :---: | :--- |
| **Mechanical Stall / Hard Stop** | `-6` | Low-speed reverse motion until stall $\rightarrow$ Stop and set zero. |
| | `-5` | Low-speed forward motion until stall $\rightarrow$ Stop and set zero. |
| | `-4` | High-speed reverse until stall $\rightarrow$ Reverse direction until torque dissipates $\rightarrow$ Stop. |
| | `-3` | High-speed forward until stall $\rightarrow$ Reverse direction until torque dissipates $\rightarrow$ Stop. |
| | `-2` | Reverse motion until stall $\rightarrow$ Exit torque zone $\rightarrow$ First Index (Z) pulse. |
| | `-1` | Forward motion until stall $\rightarrow$ Exit torque zone $\rightarrow$ First Index (Z) pulse. |
| **Limit Switch + Index Pulse** | `1` | Negative limit switch + Index pulse (searches for Z upon leaving limit). |
| | `2` | Positive limit switch + Index pulse (searches for Z upon leaving limit). |
| **Home Switch + Index Pulse** | `3` | Forward motion, Home switch + Index pulse (Z outside home switch). |
| | `4` | Forward motion, Home switch + Index pulse (Z inside home switch). |
| | `5` | Reverse motion, Home switch + Index pulse (Z outside home switch). |
| | `6` | Reverse motion, Home switch + Index pulse (Z inside home switch). |
| **Limit + Home Switch + Index** | `7` ~ `10` | Forward initial search with limit switch turnaround + Home switch + Index pulse. |
| | `11` ~ `14` | Reverse initial search with limit switch turnaround + Home switch + Index pulse. |
| **Limit Switch Edge Detection** | `17` | Negative limit switch edge detection (stops outside limit, no Index). |
| | `18` | Positive limit switch edge detection (stops outside limit, no Index). |
| **Home Switch Edge Detection** | `19`, `20` | Forward motion to home switch active/inactive edges (no Index). |
| | `21`, `22` | Reverse motion to home switch active/inactive edges (no Index). |
| **Limit + Home Edge Detection** | `23` ~ `26` | Forward initial search with limit turnaround to home switch edges. |
| | `27` ~ `30` | Reverse initial search with limit turnaround to home switch edges. |
| **Index Pulse Only** | `33` | Reverse motion until the first Index (Z) pulse. |
| | `34` | Forward motion until the first Index (Z) pulse. |
| **Immediate Position Setting** | `35` / `37` | Directly set current position as home (motor does not rotate). |

---

## 📂 Repository File Structure

```text
├── index.html                        # GitHub Pages entry point (redirects to simulator)
├── docs/                             # Technical documentation and visual assets
│   ├── homing-methods.zh-CN.md       # Step-by-step illustrated guide for homing routines
│   └── images/                       # Waveform & timing diagram assets
├── src/
│   ├── cia402-homing-simulator.html   # Main web interface (UI, canvas/DOM stage, styling)
│   └── homing-logic.js               # Core homing state machine and physics engine (Pure JS)
├── LICENSE                           # MIT License
├── README.md                         # English project documentation
└── README.zh-CN.md                   # Chinese project documentation
```

### Architecture Highlights
- **Separation of Concerns**:
  - `homing-logic.js` encapsulates all physical simulation equations (trapezoidal acceleration/deceleration, boundary clamping) and CiA 402 state-machine handling. It communicates with the UI strictly via callbacks (`log()`, `setState()`, `completeHoming()`, `triggerError()`).
  - `cia402-homing-simulator.html` handles DOM interactions, user inputs, Tailwind CSS styling, responsive layout, and visual feedback rendering loop via `requestAnimationFrame`.

---

## 📚 Standards & References

- **CAN in Automation (CiA) 402**: CANopen device profile for drives and motion control.
- **ETG.6010**: EtherCAT Implementation Directive for CiA 402 Drive Profile.
- **IEC 61800-7-201**: Adjustable speed electrical power drive systems - Part 7-201: Generic interface and use of profiles for power drive systems - Profile type 1 (CiA 402).

---

## 📄 License

This simulator is distributed under the MIT License. Feel free to use, modify, and integrate it into educational materials, training programs, or simulation environments.
