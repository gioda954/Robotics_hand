# Robotics Hand

This repository contains the ME 507 term project firmware and portfolio documentation for a three-finger robotic hand. The firmware is an STM32CubeIDE project for an STM32F411CEUx controller. It drives three DC motor channels, reads three quadrature encoders, samples three analog pressure sensors, and accepts USB CDC serial commands for grasping, releasing, setup, and manual motor adjustment.

## Documentation

- Portfolio and project memo: [docs/portfolio.md](docs/portfolio.md)
- Doxygen configuration: [Doxyfile](Doxyfile)
- Hosted documentation URL after GitHub Pages deployment: <https://gioda954.github.io/Robotics_hand/>

The GitHub Actions workflow in `.github/workflows/doxygen-pages.yml` builds Doxygen HTML from `Doxyfile` and publishes it to GitHub Pages. If Pages is not already enabled for this repository, enable Pages with GitHub Actions as the source in the repository settings.

## Repository Layout

- `Codebase/` - STM32CubeIDE project, including `.ioc`, startup files, linker scripts, application source, STM32 HAL drivers, CMSIS files, and USB device middleware.
- `Codebase/Core/Src/main.c` - Main robotic hand control loop, motor state machine, pressure control, encoder tracking, and serial status reporting.
- `Codebase/USB_DEVICE/App/usbd_cdc_if.c` - USB CDC command parser and transmit helper.
- `docs/portfolio.md` - Concise project portfolio covering hardware, software, controls, challenges, and media.
- `docs/media/README.md` - Media checklist for photos, CAD renders, and demonstration video links.

## Firmware Summary

The application initializes TIM1 PWM outputs for three motors, TIM2/TIM3/TIM4 encoder interfaces for position feedback, ADC1 channels for pressure sensor feedback, and USB CDC for command and telemetry. Runtime behavior is implemented as a C state machine with per-motor runtime data in `MotorRuntime`.

Supported serial commands include:

| Command | Behavior |
| --- | --- |
| `grab` | Close each finger until its pressure threshold or travel limit is reached. |
| `grabpid` | Use proportional pressure control to regulate each finger around its pressure target. |
| `light` | Close with low pressure thresholds. |
| `hard` | Close with high pressure thresholds. |
| `uneven` | Stop all fingers once the primary finger and one secondary finger reach pressure threshold. |
| `release` or `relase` | Open by the last recorded grab travel distance. |
| `STOP` | Immediately stop all motors. |
| `setup` | Run the forward and backward setup sequence across the three fingers. |
| `adj <motor> <+|-> <rotations>` | Move one motor by a signed rotation count, such as `adj 2 + 0.25`. |
| `yeah` | Run the configured motor by the configured rotation amount. |

Telemetry is reported over USB CDC as CSV-style rows containing timestamp, state, encoder count, estimated rotations, duty cycle, and pressure readings for all three fingers.

## Building

1. Open STM32CubeIDE.
2. Import `Codebase/` as an existing STM32CubeIDE project.
3. Build the project using the included CubeIDE project files.
4. Flash the STM32F411 target using the normal CubeIDE debug or run flow.

Generated build outputs are intentionally ignored by Git. Rebuild locally in CubeIDE when needed.

## Generating Documentation Locally

Install Doxygen, then run:

```sh
doxygen Doxyfile
```

The generated HTML output will be placed under `docs/doxygen/html/`.
