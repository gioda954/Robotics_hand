# Robotics Hand Portfolio

## Project Overview

This project is a three-finger robotic hand controlled by an STM32F411CEUx microcontroller. The hand is intended to close around objects, detect contact through pressure sensors, limit travel through encoder feedback, and release by replaying the measured closing distance in the opposite direction. The firmware supports several grasping modes so the same hardware can be tested with light contact, high-force contact, uneven objects, direct manual adjustment, and pressure feedback control.

The repository keeps the firmware in an STM32CubeIDE-compatible folder structure. The main application lives in `Codebase/Core/Src/main.c`, while the USB command interface lives in `Codebase/USB_DEVICE/App/usbd_cdc_if.c`. Doxygen is configured at the repository root so the code, this portfolio, and the firmware API can be rendered together as hosted documentation.

## Major Hardware

The controller target is an STM32F411CEUx configured from `Codebase/lab2.ioc`. The firmware uses TIM1 channels 1, 2, and 3 as PWM outputs for the three motor channels; TIM2, TIM3, and TIM4 as quadrature encoder interfaces; ADC1 channels 0, 1, and 2 as analog pressure sensor inputs; and USB full speed as a CDC serial device for user commands and telemetry.

The mechanical system is modeled in firmware as three independently actuated fingers. Each finger has a motor, a direction or phase output, an encoder, and one pressure sensor. This makes each finger an independent closed-loop axis while still allowing group behaviors such as a synchronized grab or an uneven-object grasp.

## Motors and Actuators

The hand uses three motor channels, one per finger. Motor speed is commanded by PWM duty cycle from TIM1, while direction is set by GPIO phase pins on PB0, PB1, and PB2. The firmware stores each motor channel in a `MotorRuntime` structure containing its PWM timer, encoder timer, phase pin, pressure sensor index, current state, duty cycle, and encoder-derived travel measurements.

The code ramps duty cycle rather than stepping immediately to full command. `Motor_RampDuty()` limits the change by `MOTOR_DUTY_RAMP_STEP_PERCENT` every `MOTOR_DUTY_RAMP_STEP_MS`, which reduces abrupt startup behavior and gives the pressure controller a more predictable actuator response.

## Sensors

Each finger uses a pressure sensor read through ADC1. The firmware maps raw 12-bit ADC values to percent of full scale with `PressurePercentFromRaw()`, using `ADC_MAX_COUNTS = 4095`. Pressure readings are stored in both raw counts and percent so telemetry can show the unprocessed measurement and the control-friendly normalized value.

Each motor also has encoder feedback. Encoder counts are accumulated in signed software counters, which lets the firmware handle timer wraparound and convert travel into logical forward or release motion. The defined encoder scale is `ENCODER_COUNTS_PER_MOTOR_ROTATION = 4096`, with an output rotation limit of `FORWARD_OUTPUT_ROTATION_LIMIT = 0.8`.

## Custom PCB

The CubeMX project is configured for a custom board. The custom PCB's firmware-facing responsibilities are to route the STM32F411CEUx to the motor driver PWM inputs, direction inputs, encoder channels, analog pressure outputs, USB connector, and power system. The pin map reflected in firmware is:

| Function | Pins |
| --- | --- |
| Pressure sensors | PA0, PA1, PA2 |
| Motor PWM | PA8, PA9, PA10 |
| Motor phase or direction | PB0, PB1, PB2 |
| Encoders | TIM2 on PA15/PB3, TIM3 on PA6/PA7, TIM4 on PB6/PB7 |
| USB CDC | PA11, PA12 |

Board photos, schematic exports, and PCB renders should be placed in `docs/media/` and linked from this portfolio when available. The current folder did not include those media files, so the repository includes a media checklist rather than fabricated images.

## Robot Design

The design uses a simple separation between the mechanical fingers and the controller behavior. Mechanically, each finger is treated as one motorized axis that can move forward to close or backward to release. Electrically, each axis exposes motor direction, motor PWM, encoder feedback, and contact pressure to the STM32.

That separation makes the firmware scalable: almost all behavior is implemented by iterating over the three `MotorRuntime` entries. The same state machine logic can stop one finger when it reaches pressure, release one finger based on its previous travel, or synchronize all fingers for an object-level behavior.

## Software Implementation

The firmware is written in C using STM32 HAL and generated CubeIDE support files. The application-specific code is concentrated in the user-code sections of `main.c` and `usbd_cdc_if.c`, which keeps the project compatible with CubeMX regeneration while preserving custom control logic.

The main loop calls `App_Update()` continuously. That update function polls USB commands, runs the 10 ms control loop, ramps motor duty cycle every 25 ms, and prints status telemetry every 100 ms. The control loop reads pressure sensors, updates encoders, evaluates the state of each motor, and then applies stopping, holding, release, setup, adjustment, or PID behavior.

## Drivers and Interfaces

The USB CDC command parser is one of the cleaner driver-level pieces. `CDC_Receive_FS()` parses incoming bytes into complete line commands, accepts backspace, handles command buffer overflow by resetting the buffer, and queues commands through volatile flags. The main loop later polls those flags with interrupt protection, so the USB receive path remains short and avoids running motor-control logic from the USB callback.

The parser accepts fixed commands such as `grab`, `hard`, and `STOP`, and also parses a parameterized adjustment command of the form `adj <motor> <+|-> <rotations>`. Rotation values can include fractional turns, which are stored internally as micro-rotations before conversion to encoder counts.

## Main Loop and Intelligence

The intelligence is implemented as a per-motor finite state machine. The `MotorState` enum includes idle, grabbing, holding, releasing, adjusting, PID grabbing, light grasp, hard grasp, uneven grasp, and setup motion states. Each motor can therefore stop independently when it reaches its own pressure target or travel limit, while commands such as `STOP` and `release` operate across all motors.

The grasp modes are threshold and feedback based. `grab`, `light`, and `hard` close the fingers until either pressure or travel limit is reached. `uneven` is designed for irregular objects: it waits until the primary finger and at least one secondary finger have contact before stopping the group. `grabpid` computes pressure error for each finger and commands a duty cycle proportional to that error, allowing the motor to move forward or backward to regulate contact force.

## Coding Style

The code uses procedural C with explicit state machines and small helper functions. Runtime state is stored in structs rather than global variables per channel, which keeps the three motor channels uniform and reduces duplicated control code. The naming convention separates application functions with `App_`, motor helpers with `Motor_`, serial helpers with `Serial_`, and USB CDC command helpers with `USB_CDC_` or `CDC_`.

The code also uses fixed-width integer types and integer scaling for embedded reliability. For example, fractional rotations are parsed as micro-rotations, pressure is represented as percent, PID gains are scaled by `GRABPID_GAIN_SCALE`, and rotation telemetry is reported in milli-rotations to avoid floating-point formatting in the runtime path.

## Modeling and Calculations

The main physical model is the conversion between encoder counts and finger travel. With `ENCODER_COUNTS_PER_MOTOR_ROTATION = 4096` and `FORWARD_OUTPUT_ROTATION_LIMIT = 0.8`, the firmware computes a forward travel limit in encoder counts. During a grab, the code records the starting encoder count and stops the motor when logical forward travel exceeds that limit or the pressure threshold is reached.

Release motion uses the measured closing travel. When `release` is commanded, each finger moves in the opposite direction until its release travel matches the previously recorded grab travel. This makes release behavior adaptive to the actual object rather than relying on a fixed release duration.

Pressure control is implemented by normalizing ADC counts to percent and comparing that value against per-finger thresholds or PID targets. The PID mode currently uses proportional gain with integral and derivative gains configured to zero, but the data structure includes integral accumulation, derivative calculation, output limiting, deadband handling, and direction reversal.

## Dynamic Behavior

Dynamic behaviors are calculated from encoder deltas, pressure thresholds, and duty ramps. A motor is not simply run for a fixed time. Instead, the firmware measures how far it has moved, how much pressure it is applying, and which state it is in. The duty target is then ramped toward the mode-specific command, such as normal grasp duty, setup duty, or PID-selected duty.

The setup mode demonstrates sequenced behavior. It moves each finger forward and backward through the configured travel range, then advances to the next finger. During backward setup travel, the code can start the next finger early when the current finger is close enough to completing its return travel. This overlap reduces idle time during the setup sequence.

## Performance, Challenges, and Workarounds

The project is structured around reliability concerns that usually appear in small robotic hands: avoiding overtravel, preventing excessive grip pressure, keeping serial command handling responsive, and preserving enough telemetry to diagnose failures. The firmware addresses these by combining encoder travel limits, pressure thresholds, immediate `STOP`, periodic CSV telemetry, and command-specific state transitions.

A key challenge is that pressure sensors and mechanical fingers rarely behave identically. The code therefore uses per-finger pressure thresholds, per-finger PID targets, and per-motor direction signs. Another challenge is recovering from partial grasps or irregular objects, which is why the firmware records per-finger travel before release and includes a separate `uneven` behavior.

## Testing and Demonstration

The firmware reports a startup banner, supported command list, and telemetry header over USB CDC. During operation it prints time, motor states, encoder readings, rotation estimates, duty cycles, raw pressure readings, and normalized pressure readings. These logs can be captured during a demonstration to show repeatability and to diagnose root causes when behavior is inconsistent.

The local folder did not include demonstration video, completed-board photos, CAD renders, or rework photos. Those should be added under `docs/media/` before final submission if they are available, then linked from this section.

## Media and Attachments

Recommended final attachments:

- Demonstration video showing `grab`, `release`, and at least one object-specific mode such as `light`, `hard`, `uneven`, or `grabpid`.
- Photo of the assembled hand.
- Photo of the custom PCB.
- CAD render or image of the finger and hand assembly.
- Any rework photos, especially if they explain a reliability problem and its fix.

See `docs/media/README.md` for the media checklist.
