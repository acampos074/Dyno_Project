# FOC_GUI_V3 — Code Overview

A real-time motor control and dynamometer testing application for brushless motors using Field-Oriented Control (FOC). It combines a live GUI dashboard with a hardware-interfacing real-time sequencer.

---

## System Architecture

The application is split into two separate processes that communicate over **LCM (Lightweight Communications Middleware)**:

```
┌─────────────────────────────────┐
│         GUI Process             │
│  ImGui + ImPlot (60 FPS)        │
│  src/main.cpp                   │
└────────────┬────────────────────┘
             │  LCM "MOTOR" (commands)
             │  LCM "GUI"    (state feedback)
┌────────────┴────────────────────┐
│     Real-Time Sequencer         │
│  100 Hz timer / 50 Hz service   │
│  src/seqgen3.c                  │
└────────────┬────────────────────┘
             │  CAN Bus (1 Mbps, SocketCAN)
             │  USB (MCC 1608FS ADC)
┌────────────┴────────────────────┐
│     Hardware                    │
│  Brushless Motor Controller     │
│  Torque Sensor (analog)         │
└─────────────────────────────────┘
```

---

## File Structure

```
FOC_GUI_V3/
├── src/
│   ├── main.cpp          # GUI application (ImGui/ImPlot + LCM subscriber/publisher)
│   ├── seqgen3.c         # Real-time sequencer (CAN, MCC DAQ, state machine, logging)
│   └── can_rxtx.cpp      # Legacy CAN + Redis integration (not actively used)
├── include/
│   └── dyno.h            # Shared structs, constants, and defines
├── lib/
│   ├── lcm/              # LCM message definitions and generated C/C++ bindings
│   │   ├── motor.lcm     # Message schema (FOC::motor_t)
│   │   ├── FOC_motor_t.h # C binding (used by seqgen3.c)
│   │   └── motor_t.hpp   # C++ binding (used by main.cpp)
│   ├── mcc/              # MCC USB-1608FS driver library
│   ├── imgui/            # Dear ImGui framework
│   └── implot/           # ImPlot real-time plotting extension
└── Makefile
```

---

## Key Components

### `src/main.cpp` — GUI Application

Renders the motor control dashboard using Dear ImGui and ImPlot.

**Responsibilities:**
- Renders 6 real-time scrolling plots: position, velocity, VQ voltage, IQ current, torque, power
- Provides control buttons: Controller ON/OFF, Animate (pause plots), Debug (LED toggle), Calibrate, Dyno Test
- Radio buttons to select control mode
- Sliders for commands: position, velocity, voltage, current, torque, KP, KD
- Publishes user commands to LCM channel `"MOTOR"`
- Subscribes to LCM channel `"GUI"` on a background thread to receive motor state

**Key structures:**
- `ScrollingBuffer` — fixed-size circular buffer for time-series plot data
- `RollingBuffer` — rolling window buffer (wraps at a configurable time span)
- `Handler` — LCM message handler; updates global state variables on receive

**LCM threading model:**
```
main thread       → renders ImGui, publishes "MOTOR" commands
updaterThread     → blocks on lcm.handle(), updates globals on "GUI" messages
```

> **Note:** The globals updated by `updaterThread` and read by the main thread are currently unprotected — see [IMPROVEMENTS.md](IMPROVEMENTS.md).

---

### `src/seqgen3.c` — Real-Time Sequencer

The control backbone. Runs as a separate process with real-time scheduling.

**Threading model:**
- `main()` — initializes hardware (CAN socket, MCC USB, LCM), creates threads, arms timer
- `Sequencer()` — SIGALRM handler at 100 Hz; posts semaphore to release `Service_1`
- `Service_1()` — POSIX thread at 50 Hz (SCHED_FIFO, CPU-pinned); runs the state machine
- `listener()` — background thread receiving LCM "MOTOR" messages from the GUI

**State machine (`dyno.state`):**

| `cmd_id` | State              | Description                                  |
|----------|--------------------|----------------------------------------------|
| 1        | Calibration        | Send home/calibrate command via CAN          |
| 2        | Controller OFF     | Send idle/off command via CAN                |
| 3        | Voltage FOC        | Open-loop Q-axis voltage command             |
| 4        | Position Control   | PID position control to target angle         |
| 5        | Velocity Control   | PID velocity control to target speed         |
| 6        | Dyno Test          | Automated voltage ramp + efficiency sweep    |
| 7        | Toggle LED         | Toggle hardware indicator LED                |
| 8        | Current Control    | Q-axis current (iq) command                  |
| 9        | Torque Control     | Torque PID command                           |

**CAN frame format (8 bytes, ID `0x123`):**

| Byte(s) | Content                         |
|---------|---------------------------------|
| 0       | State / mode flag               |
| 1       | LED flag                        |
| 2–3     | 16-bit encoded command value    |
| 4       | 8-bit encoded KP gain           |
| 5       | 8-bit encoded KD gain           |
| 6–7     | Reserved / additional data      |

Floats are encoded to integers using range-mapped functions:
- `float_to_uint16(x, x_min, x_max)` — for voltages, currents, position, velocity, torque
- `float_to_uint8(x, x_min, x_max)` — for gains (KP, KD)

**Dyno Test sequence:**
1. Ramp voltage up incrementally over `ramp_time` seconds
2. Hold voltage, collect torque/speed/power measurements
3. Repeat for `num_cycles` cycles
4. Ramp voltage back to 0
5. Log all data to `dyno_test.csv`

**CSV output (`foc_open_loop.csv`):**
```
Time (s), Vq_cmd (V), Vq_msr (V), Iq (A), Torque (Nm), Speed (rad/s),
Position (rad), Elec Power (W), Mech Power (W), Efficiency
```

---

### `include/dyno.h` — Shared Definitions

Defines constants and the `dyno_t` struct shared across the sequencer.

**Motor constants:**

| Constant     | Value       | Description              |
|--------------|-------------|--------------------------|
| `V_MAX`      | 24.0 V      | Max Q-axis voltage        |
| `I_MAX`      | 40.0 A      | Max Q-axis current        |
| `TAU_MAX`    | 2.0 Nm      | Max torque                |
| `SPEED_MAX`  | 300.0 rad/s | Max angular velocity      |
| `GR`         | 12.0        | Gear ratio                |
| `KT`         | 0.0217 Nm/A | Motor torque constant     |

**`dyno_t` struct** holds the full sequencer state: commanded values, hardware-encoded integers, test timing parameters, and computed metrics (power, efficiency).

---

### `lib/lcm/motor.lcm` — LCM Message Schema

```
package FOC;
struct motor_t {
    double position, velocity, torque, iq, vq, id, vd;  // measured state
    double position_cmd, velocity_cmd, torque_cmd, iq_cmd, vq_cmd;  // commands
    double kp, kd;       // control gains
    int8_t led;          // LED flag
    int8_t cmd_id;       // state machine command (1–9)
    int8_t flag;
    int8_t control_flag;
}
```

**LCM channels:**

| Channel   | Direction        | Content                        |
|-----------|------------------|--------------------------------|
| `"MOTOR"` | GUI → Sequencer  | User commands (cmd_id + params)|
| `"GUI"`   | Sequencer → GUI  | Motor state feedback           |

---

### `lib/mcc/` — MCC USB-1608FS Driver

Reads the analog torque sensor connected to channel 0 of the USB-1608FS DAQ.

- Range: ±5V (`BP_5_00V`)
- Conversion: `torque (Nm) = ADC_voltage × 0.57`
- Calibration: per-channel EEPROM slope/offset table loaded at startup
- Sampling: on-demand single reads at 50 Hz from `Service_1`

---

## Data Flow Summary

```
1. User interacts with GUI (slider/button)
2. main.cpp sets motor_data fields, publishes on "MOTOR"
3. seqgen3.c listener thread receives message, updates dyno struct
4. Service_1 (50 Hz) reads dyno.state, executes state machine branch
5. Encodes float commands to integers
6. Sends 8-byte CAN frame to motor controller
7. Reads CAN response (motor state: position, velocity, currents, voltages)
8. Reads MCC ADC channel 0 (torque sensor)
9. Computes efficiency = mech_power / elec_power
10. Logs row to CSV file
11. Publishes motor state on "GUI" LCM channel
12. main.cpp updaterThread receives, updates plot buffers
13. Main thread renders updated plots at ~60 FPS
```

---

## Dependencies

### System Libraries (install via package manager)

| Library | Purpose | Install |
|---------|---------|---------|
| **GLFW 3** | Window and OpenGL context management | `sudo apt install libglfw3-dev` |
| **OpenGL** | GPU rendering backend | `sudo apt install libgl1-mesa-dev` |
| **LCM** | Inter-process communication (pub/sub) | `sudo apt install lcm` or build from source |
| **libusb-1.0** | USB device access for MCC DAQ | `sudo apt install libusb-1.0-0-dev` |
| **hidapi-libusb** | HID device interface for MCC DAQ | `sudo apt install libhidapi-dev` |
| **pthreads** | POSIX real-time threading | Included in glibc |
| **librt** | POSIX real-time extensions (timers) | Included in glibc |

> LCM must expose a `pkg-config` entry (`lcm.pc`). Verify with `pkg-config --libs lcm`.

### Vendored Libraries (included in `lib/`)

| Library | Version | Purpose |
|---------|---------|---------|
| **Dear ImGui** | ~1.89 | Immediate-mode GUI framework |
| **ImPlot** | master | Real-time plotting extension for ImGui |
| **LCM C/C++ bindings** | Generated | `FOC::motor_t` message type bindings |
| **MCC USB-1608FS driver** | Custom | `pmd.c`, `nist.c`, `usb-1608FS.c` — analog DAQ support |

### Compiler Requirements

| Tool | Minimum Version |
|------|----------------|
| `g++` | C++11 (`-std=c++11`) |
| `gcc` | C99 compatible |
| `make` | GNU Make |

---

## Build & Run

```bash
# Build both the GUI (dyno_gui) and sequencer (seqgen3)
make

# Run sequencer first (real-time scheduling requires elevated privileges)
sudo ./seqgen3

# Run GUI in a separate terminal
./dyno_gui
```

The sequencer must be running before the GUI to establish the LCM channels.
