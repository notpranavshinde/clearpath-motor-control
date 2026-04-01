# ClearPath Motor Control

This repository contains multiple Step/Dir control paths for a ClearPath SDSK linear drive setup:

1. Arduino sketches for standalone motion control with AccelStepper.
2. A Python + NI-DAQmx controller for hardware pulse generation on NI USB-6341.

## Repository Layout

- `arduino motor control/clearpath_motor_velocity_ramp_up_and_down.ino`
    - Baseline Arduino sketch with metric input constants.
    - Runs one trapezoidal move per Enter keypress in Serial Monitor.
- `arduino motor control/clearpath_motor_velocity_ramp_up_and_down_debug.ino`
    - Debug-oriented Arduino sketch.
    - Prints requested vs achievable motion (planner limits and actual measured move timing).
- `arduino motor control/clearpath_motor_velocity_ramp_up_and_down_final_with_ui.ino`
    - Interactive runtime UI over Serial.
    - Lets you enter distance/speed/acceleration and optional round-trip loops at runtime.
- `daq_motor_control/clearpath_6341_stepdir.py`
    - NI-DAQ based controller using counter pulse output (step) + digital lines (dir/enable).
    - Supports both constant-velocity moves and segmented trapezoid/triangle moves.

## Hardware Wiring (Step/Dir/Enable)

Common signal mapping used across the code:

- `STEP+` (black) -> pulse output (`D2` on Arduino variants, counter output on NI-DAQ script)
- `DIR+` (white) -> direction line (`D3` on Arduino, DO line on NI-DAQ)
- `EN+` (blue) -> enable line (`D4` on Arduino, DO line on NI-DAQ)
- `STEP-`, `DIR-`, `EN-` -> controller ground

Important: keep the physical enable switch open/disabled while flashing firmware.

## Arduino Workflow

1. Open one of the `.ino` files in Arduino IDE.
2. Install `AccelStepper` from Library Manager.
3. Upload to the board with motor enable open.
4. Open Serial Monitor at `115200` baud.
5. Close enable switch and follow each sketch's prompts.

### Arduino Dependencies

- [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/)

## NI-DAQ Python Workflow

The Python script is intended for NI USB-6341 style hardware and uses `nidaqmx` tasks for:

- Finite pulse trains on counter output for step pulses.
- Digital output lines for direction and enable.

### Python Setup

1. Install NI-DAQmx driver from National Instruments.
2. Install Python dependencies:

```bash
pip install nidaqmx
```

3. Run:

```bash
python daq_motor_control/clearpath_6341_stepdir.py
```

4. Enter distance, speed, profile mode, and optional loop settings when prompted.

## Safety and Operating Notes

The current code includes software guardrails to reduce mechanical risk:

- Distance clamp around `2.5 m` maximum travel.
- Acceleration guard near `10 m/s^2` upper bound.
- Speed limits based on controller/mechanics constraints.

Tune values conservatively and increase gradually during commissioning.

