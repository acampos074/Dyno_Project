# FOC_GUI_V4

Real-time motor control and dynamometer testing application for a brushless motor driven through a CAN bus interface. Consists of two separate processes that communicate over LCM (Lightweight Communications Middleware).

---

## Architecture

```
┌─────────────────────────────────┐        ┌──────────────────────────────────┐
│          dyno_gui               │        │            seqgen                │
│  (ImGui + ImPlot GUI process)   │        │  (Real-time sequencer process)   │
│                                 │  LCM   │                                  │
│  FOC_ControlPanel()  ──────────────────► │  lcm_interface (my_handler)      │
│    buttons, sliders             │ MOTOR  │    writes dyno struct            │
│    lcm2.publish("MOTOR")        │        │                                  │
│                                 │        │  Service_1  @ 50 Hz              │
│  FOC_Plots()                    │        │    CAN send/receive              │
│    6-panel ImPlot grid          │ ◄──────────  lcm.publish("GUI")          │
│    scrolling buffers            │  GUI   │    MCC DAQ torque read           │
│                                 │        │    ring buffer log push          │
└─────────────────────────────────┘        │                                  │
                                           │  logger_thread  (non-RT)         │
                                           │    drains ring → CSV files       │
                                           └──────────────────────────────────┘
```

### LCM Channels

| Channel  | Direction       | Content                              |
|----------|-----------------|--------------------------------------|
| `MOTOR`  | GUI → seqgen    | Command ID, setpoints, gains         |
| `GUI`    | seqgen → GUI    | Position, velocity, vq, iq, torque   |

---

## Hardware Requirements

| Device | Purpose |
|--------|---------|
| Brushless motor controller | Connected via CAN bus (`can0`, 1 Mbps) |
| MCC USB-1608FS DAQ | Torque sensor input (0.57 Nm/V, channel 0, ±5 V) |
| Linux machine with CAN adapter | Runs both processes |

---

## Software Dependencies

```bash
# LCM
sudo apt install lcm

# GLFW + OpenGL
sudo apt install libglfw3-dev libgl-dev

# libusb (for MCC DAQ)
sudo apt install libusb-1.0-0-dev libhidapi-dev
```

---

## Build

```bash
make          # builds both dyno_gui and seqgen
make dyno_gui # GUI only
make seqgen   # sequencer only
make clean    # remove build artifacts
```

Object files are placed in `build/`.

---

## Running

Start both processes, `seqgen` first:

```bash
# Terminal 1 — sequencer (sudo needed for CAN setup and SCHED_FIFO)
sudo ./seqgen

# Terminal 2 — GUI
./dyno_gui
```

---

## Control Modes

| Mode | Description |
|------|-------------|
| Position Control | PD position tracking |
| Velocity Control | PD velocity tracking |
| Open Loop Voltage | Direct q-axis voltage command |
| Current Control | Direct q-axis current command |
| Torque Control | Torque setpoint with PD gains |
| Dyno Test | Automated voltage sweep with torque, power, and efficiency logging |

---

## Source Layout

```
FOC_GUI_V4/
├── Makefile
├── include/
│   └── dyno.h              # Shared constants and MotorCmd enum (GUI + seqgen)
├── src/
│   ├── gui/
│   │   └── main.cpp        # ImGui/ImPlot GUI — FOC_ControlPanel, FOC_Plots
│   └── sequencer/
│       ├── seqgen_main.c   # main() — init, RT scheduling, thread creation
│       ├── sequencer.c/h   # Sequencer (SIGALRM 100 Hz), Service_1 (50 Hz)
│       ├── lcm_interface.c/h  # LCM subscribe/publish, dyno state, mutex
│       ├── can_driver.c/h  # SocketCAN open/send/receive with select() timeout
│       ├── mcc_driver.c/h  # MCC USB-1608FS init and torque read
│       ├── logger.c/h      # SPSC ring buffer logger (RT-safe, non-blocking)
│       └── encoding.c/h    # Float ↔ uint8/12/16 quantization helpers
└── lib/
    ├── imgui/              # Dear ImGui
    ├── implot/             # ImPlot
    ├── lcm/                # LCM generated types (FOC_motor_t)
    └── mcc/                # MCC USB-1608FS driver
```

---

## Real-Time Design

| Concern | Solution |
|---------|----------|
| Sequencer timer | `timer_create(CLOCK_MONOTONIC)` + SIGALRM at 100 Hz |
| Service_1 rate | Semaphore released every 2nd tick → 50 Hz |
| CPU affinity | Service_1 pinned to core 2, SCHED_FIFO RT_MAX-1 |
| Dyno struct race | Snapshot pattern: lock → copy → unlock → work |
| File I/O in RT thread | SPSC ring buffer; logger_thread does all disk writes |
| CAN blocking read | `select()` with `CAN_RECV_TIMEOUT_MS` (5 ms) timeout |

---

## Output Files

| File | Contents |
|------|----------|
| `foc_open_loop.csv` | Time, Vq cmd/msr, Iq, Torque, Velocity, Position, Elec/Mech Power, Efficiency |
| `dyno_test.csv` | Same columns, written during Dyno Test mode |
