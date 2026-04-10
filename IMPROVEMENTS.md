# Code Improvement Suggestions

This document outlines architectural and code quality improvements for FOC_GUI_V3, ordered by severity.

---

## 1. Race Condition on Shared Globals (High Severity)

**Location:** `src/main.cpp:80–88` (Handler), read throughout `FOC_RealtimePlots()`

**Problem:**
The `Handler::handleMessage` callback runs on `updaterThread` and writes to global variables (`position`, `velocity`, `torque`, `iq`, `vq`, `elec_power`, `mech_power`). The main render thread reads these same variables every frame with no synchronization. This is an undefined behavior data race.

```cpp
// updaterThread writes:
position = msg->position;
velocity = msg->velocity;

// main thread reads simultaneously:
rdata1.AddPoint(t, position);
rdata2.AddPoint(t, velocity);
```

**Fix — option A (mutex):**
```cpp
std::mutex state_mutex;

// In Handler::handleMessage:
{
    std::lock_guard<std::mutex> lock(state_mutex);
    position = msg->position;
    velocity = msg->velocity;
    // ...
}

// In FOC_RealtimePlots(), snapshot before use:
double pos, vel, tau, iq_val, vq_val, ep, mp;
{
    std::lock_guard<std::mutex> lock(state_mutex);
    pos = position; vel = velocity; // ... etc
}
rdata1.AddPoint(t, pos);
```

**Fix — option B (atomics, simpler for scalar doubles):**
```cpp
std::atomic<double> position{0}, velocity{0}, torque{0};
// atomic<double> requires C++11; reads/writes are automatically safe
```

---

## 2. Magic State Numbers (Medium Severity)

**Location:** `src/main.cpp:235–285`, `src/seqgen3.c` (switch statement)

**Problem:**
Control states are referenced as bare integers in both files. If the mapping ever shifts, both files must be updated manually and in sync.

```cpp
state = 4;  // what does 4 mean?
state = 8;
```

**Fix:** Add a shared enum to `include/dyno.h`:

```c
typedef enum {
    CMD_CALIBRATE    = 1,
    CMD_OFF          = 2,
    CMD_VOLTAGE_FOC  = 3,
    CMD_POSITION     = 4,
    CMD_VELOCITY     = 5,
    CMD_DYNO_TEST    = 6,
    CMD_TOGGLE_LED   = 7,
    CMD_CURRENT      = 8,
    CMD_TORQUE       = 9
} MotorCmd;
```

Both `main.cpp` and `seqgen3.c` include `dyno.h` already, so the enum is immediately available in both. Replace all integer literals:

```cpp
// Before:
state = 4;
motor_data.cmd_id = state;

// After:
motor_data.cmd_id = CMD_POSITION;
```

---

## 3. Duplicated Constants (Medium Severity)

**Location:** `src/main.cpp:41–57` and `src/seqgen3.c:65`

**Problem:**
`GAIN`, `kp_max`, `V_MAX`, `I_MAX`, `GR`, `SPEED_MAX`, `ONE_REV` are defined independently in both files. They must be kept in sync manually.

```cpp
// main.cpp
static float GAIN = 1.0f;
static float kp_max = 0.2f;

// seqgen3.c
static float GAIN = 1.0;
static float kp_max = 0.2f;
```

**Fix:** Move all shared constants into `include/dyno.h` (most are already there as `#define`s — add the missing ones):

```c
// In dyno.h
#define GAIN      1.0f
#define KP_MAX    0.2f
```

Remove the local `static` definitions from both source files.

---

## 4. `FOC_RealtimePlots()` is a God Function (Medium Severity)

**Location:** `src/main.cpp:157–515` (~360 lines)

**Problem:**
A single function handles button state management, LCM publishing, slider input, plot configuration, and power calculations. This makes it hard to test, read, or extend any one part.

**Fix:** Split into focused functions:

```cpp
// Render control panel: buttons, radio buttons, sliders
// Returns a MotorCommand if user changed anything, else nullopt
std::optional<MotorCommand> renderControlPanel();

// Publish a MotorCommand over LCM
void publishCommand(const MotorCommand& cmd);

// Render the 3x2 subplot grid
void renderPlots(const MotorState& state, float t, float history, bool paused);
```

`FOC_RealtimePlots()` then becomes a thin coordinator:
```cpp
void FOC_RealtimePlots() {
    auto cmd = renderControlPanel();
    if (cmd) publishCommand(*cmd);
    renderPlots(currentState, t, history, paused);
}
```

---

## 5. Two LCM Instances (Low Severity)

**Location:** `src/main.cpp:59` (`lcm2`, global), `src/main.cpp:547` (`lcm`, local in `main()`)

**Problem:**
`lcm` subscribes to `"GUI"` and `lcm2` publishes to `"MOTOR"`. A single LCM instance handles both pub and sub — the split is unnecessary and opens two separate UDP sockets.

```cpp
lcm::LCM lcm2;         // global, used for publishing
lcm::LCM lcm;          // local in main(), used for subscribing
```

**Fix:** Use one instance for both:
```cpp
lcm::LCM lcm;
lcm.subscribe("GUI", &Handler::handleMessage, &handlerObject);
// publish with the same instance:
lcm.publish("MOTOR", &motor_data);
```

---

## 6. Dead Code (Low Severity)

**Location:** `src/main.cpp:62–106`, `src/main.cpp:347–362`, throughout `seqgen3.c`

**Problem:**
Large commented-out blocks (`MotorHandler`, old `Position` handler, old LED logic, multiple `std::cout` debug lines) clutter the file and create confusion about what is actually active.

**Fix:** Delete commented-out code. If historical versions are needed, git history preserves them. Specifically:
- Remove the commented `class MotorHandler` block (lines 91–106)
- Remove the commented `class Handler` using `mypackage::Position` (lines 62–72)
- Remove the commented LED else-block (lines 347–362)
- Remove commented-out `lcm2` motor subscriber thread (lines 569–581)

---

## 7. `seqgen3.c` is a Monolith (Low Severity — Large Refactor)

**Location:** `src/seqgen3.c` (994 lines)

**Problem:**
A single `.c` file handles: signal/timer setup, real-time thread management, CAN socket I/O, MCC USB initialization and reads, the state machine, LCM message handling, float-to-int encoding, CSV logging, and timing utilities. Each is an independent concern.

**Proposed split:**

```
src/
  sequencer/
    sequencer.c      — timer, semaphore, thread creation (Sequencer + Service_1 shell)
    state_machine.c  — switch(dyno.state) logic only
    can_driver.c     — openCANSocket(), sendCANFrame(), receiveCANFrame()
    mcc_driver.c     — initMCC(), readTorque()
    logger.c         — openLogFile(), logRow(), closeLogFile()
    encoding.c       — float_to_uint16/8/12, uint_to_float variants
```

Each module exposes a minimal header. `Service_1` calls into them:
```c
float torque = readTorque();
sendCANFrame(sock, CAN_ID, encodeCommand(&dyno));
logRow(fp, &dyno);
```

This makes each piece independently testable and readable.

---

## 8. `dyno_t` Struct Conflates Multiple Roles (Low Severity)

**Location:** `include/dyno.h:45–81`

**Problem:**
The struct mixes commanded values, hardware-encoded integers, test configuration, and computed metrics in one flat struct. It's hard to know what is "input", "derived", or "hardware artifact" just by reading it.

```c
double vq_cmd;        // input command
int vq_int;           // hardware encoding of vq_cmd (derived)
float efficiency;     // computed metric (derived)
double ramp_time;     // test configuration
```

**Fix:** Split into purpose-specific structs:

```c
typedef struct {
    double pos_cmd, vel_cmd, vq_cmd, iq_cmd, torque_cmd;
    double kp, kd;
    int state, led_flag;
} MotorCommand;

typedef struct {
    double Vmax, ramp_time, meas_time, ramp_down_time;
    int num_cycles, ramp_step;
} DynoConfig;

typedef struct {
    float torque, elec_power, mech_power, efficiency;
    double Fs, h;
    int total_test_counter, cycle_counter, N, N_ramp_down;
} DynoState;
```

---

## Summary Table

| # | Issue                              | Severity | Effort  | File(s)                          |
|---|------------------------------------|----------|---------|----------------------------------|
| 1 | Race condition on shared globals   | High     | Low     | `src/main.cpp`                   |
| 2 | Magic state numbers                | Medium   | Low     | `src/main.cpp`, `src/seqgen3.c`  |
| 3 | Duplicated constants               | Medium   | Low     | `src/main.cpp`, `src/seqgen3.c`  |
| 4 | God function `FOC_RealtimePlots`   | Medium   | Medium  | `src/main.cpp`                   |
| 5 | Two LCM instances                  | Low      | Low     | `src/main.cpp`                   |
| 6 | Dead code                          | Low      | Low     | `src/main.cpp`, `src/seqgen3.c`  |
| 7 | Monolithic `seqgen3.c`             | Low      | High    | `src/seqgen3.c`                  |
| 8 | Overloaded `dyno_t` struct         | Low      | Medium  | `include/dyno.h`                 |

**Start with items 1–3** — they have the highest impact for the least effort and can be done without restructuring the project.
