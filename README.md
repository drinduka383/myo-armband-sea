# Myo-controlled SEA rehabilitation demo

This demo uses Myo muscle activity to control a real embedded actuator chain
and the same signal to drive a virtual 1-DOF series-elastic finger model.

Intended chain: Myo BLE -> Fedora/Python -> ST-LINK USB serial ->
NUCLEO-F446RE -> Maxon ESCON 50/5 -> Harmonic Drive PMA actuator. The Myo
classifier and physical STM32/ESCON actuator chain were validated separately;
a powered end-to-end Myo-to-motor test has not been performed.

Virtual chain: activation -> spool displacement -> spring compression -> tendon
force -> finger angle. The physical actuator-control chain is validated; the
unfinished tendon/Bowden/spring/finger mechanics are represented by this model.

## Wiring and safety

| Nucleo pin | ESCON connection |
|---|---|
| PA4 / A2 (DAC1) | Analog Input 1+ |
| PC0 / A5 | Digital Input 2, high-active enable |
| GND | Signal GND |

Boot and every software error command STOP: PA4 = 0 V and PC0 = low. Test with
the motor unloaded, ESCON limits/ramp low, and motor power disabled until the
serial ACK test passes. `--motor disable` is the default. Enabling the motor is
an explicit action.

## Setup and discovery

```bash
./tools/check_env.sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python tools/list_serial.py
python myo_scan.py --seconds 20
python myo_services.py MYO_ADDRESS
```

If Python 3.14 cannot install a package, use Python 3.13/3.12 if installed.
Matplotlib is only needed for plotting; BLE and serial need only Bleak and
pyserial.

## Acceptance sequence

1. With ESCON off/motor disabled, import/flash the STM32CubeIDE project under
   `stm32/new-stm-code` and verify `/dev/ttyACM*`.
2. Run `python serial_test.py --port /dev/ttyACM0`. Check `0`, `1`, then `0`
   and the returned ACKs before powering the motor. The automated pattern is
   `python serial_test.py --port /dev/ttyACM0 --pattern`.
3. Test real EMG with no motor command:

   ```bash
   python run_myo_sea_demo.py --myo-address MYO_ADDRESS --motor disable --mode diagnostic --live-plot
   ```

   Default mode explicitly trains `pinch_attempt` as the actuator target with
   three independent 2-second attempts in handshake, palm-down, and palm-up
   orientations. Use `--target-override auto` only when comparing
   `claw_hook`, `pinch_attempt`, `partial_hand_close`, and `fist_lite`. Rest is an activity baseline, not a
   cosine-similarity rejection template. Open hand and full fist are static
   rejections; wrist flexion/extension and forearm pronation/supination are
   controlled movement rejections. If no candidate passes every orientation,
   calibration stops without entering live motor control.

   Each prompt waits for Enter, runs a 4-second countdown, and asks whether to
   accept or repeat the attempt. `HOLD` means pose first and remain still;
   `MOVE` means perform one slow movement during the 2-second recording.
   Emergency fallback remains `--detector threshold`.

   Classification combines the normalized eight-channel MAV pattern with the
   calibrated total activity magnitude. After calibration, one independent
   labeled handshake trial each for rest, pinch, open hand, and full fist must
   reach at least 60% correct before the result is considered non-weak.
   When the target is weak, the review prompt can redo every target trial with
   `a`, or only handshake (`h`), palm-down (`d`), or palm-up (`u`) without
   repeating rest and rejection calibration.

   Calibration effort guide:

   Sit with the shoulder relaxed and elbow near 90 degrees. Support the forearm
   without pressing the Myo against the desk, and leave the hand beyond the
   desk edge. Keep the wrist straight unless the prompt explicitly requests a
   wrist movement.

   - `rest`: 0/10 effort. Forearm and fingers loose, wrist neutral.
   - `pinch_attempt`: 2-3/10 effort. Gentle thumb-index pinch intent.
   - `claw_hook`: 3/10 effort. Keep base knuckles straight; bend the middle and fingertip joints—the “rawr” pose.
   - `partial_hand_close`: 2-3/10 effort. Small hand close, not a fist.
   - `fist_lite`: 3-4/10 effort. Light, repeatable fist.
   - `wrist_flex`: bend the palm toward the inner forearm/torso; `wrist_extend`: bend the back of the hand toward the outer forearm.
   - `forearm_pronate`: move handshake to palm-down; `forearm_supinate`: move handshake to palm-up.
   - `open_hand`: 2-3/10 effort. Open/spread hand without strong wrist extension.
   - `full_fist`: 5/10 effort. Firm squeeze, never maximum; a rejection class.

   General rule: use the weakest effort that is still repeatable. Bigger effort
   is not better; it often makes all gestures look like `full_fist`.

   The optional dashboard shows eight vertically offset raw EMG traces, eight
   200 ms MAV traces, and a color-coded REST/TARGET/rejection strip. `orientation`
   is the forearm IMU orientation, while `EMG effort` is relative muscle activity.
   `virtual_force` and `virtual_angle` are model outputs, not measurements. Ctrl+C
   sends STOP before closing it. After a successful new calibration, pressing
   Enter at the save prompt writes a processed preset under `calibrations/`.
   On later runs, choose that preset from the startup menu only if the wearer,
   arm, and Myo placement are unchanged.
   After selecting a saved preset, choose to use it unchanged, edit activation
   and deactivation similarity/margins/gyro/hold times, or reconnect and retake
   a specific target, rest, open-hand, or full-fist orientation. A retake
   replaces that three-attempt block only. On shutdown, changes can be saved as
   a new timestamped preset, overwrite the loaded preset, or be discarded.

   The live `force_N` number is the virtual SEA model output, not a direct force
   sensor measurement. With `--motor disable`, it is only an internal model
   estimate derived from the detected intent and the configured spring model.
4. Repeat with `--mode assistive`; diagnostic mode blocks the virtual finger,
   while assistive mode maps activation to finger angle.
5. Only after tests 1-4 pass, power the safely configured unloaded ESCON and run:

   ```bash
   python run_myo_sea_demo.py --myo-address MYO_ADDRESS --motor enable --serial-port /dev/ttyACM0
   ```

   Relax/rejection sends `0`; detected target curl sends `1`. Ctrl+C, BLE loss,
   serial failure, or stale EMG forces STOP.
6. Generate the report figure:

   ```bash
   MPLBACKEND=Agg python plot_log.py logs/session_YYYYMMDD_HHMMSS.csv \
     --output report/figures/myo-sea-replay.png
   ```

   The replay recomputes an explicitly simulated bounded 0--180 degree finger
   trajectory and virtual spring response from the measured EMG detector state.
   Build the report with `cd report && tectonic main.tex`.

If direct BLE cannot be made reliable after a documented attempt, retain the
same serial/simulation demo with `--source keyboard` (type `1`, `0`, or `q`) or
`--source synthetic --duration 10`. Do not describe those fallbacks as live Myo
control.

## Evidence checklist

- Myo/USB wiring photos and Bluetooth/serial terminal screenshots
- Serial STOP/RUN/STOP ACKs and PA4/PC0 measurements
- ESCON configuration screenshot and unloaded actuator video
- Myo REST/ACTIVE/REST terminal screenshot with motor disabled
- Diagnostic and assistive virtual-model screenshots
- Final Myo-to-motor video (not completed; do not imply otherwise)
- CSV session logs and generated PNG plots

Known limitation: the physical SEA/tendon/finger mechanism is not complete, so
spring compression, tendon force, finger angle, and EMD are design-model outputs,
not direct mechanical measurements.
