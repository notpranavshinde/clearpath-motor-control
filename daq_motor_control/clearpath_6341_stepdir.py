import math
import time
from dataclasses import dataclass

import nidaqmx
from nidaqmx.constants import AcquisitionType, FrequencyUnits, Level
from nidaqmx.system import System


# ----------------------------
# Mechanics / conversion
# ----------------------------
PPR = 200
M_PER_REV = 0.1016  # 4.000 in per rev = 0.1016 m/rev (XL, 20T pulley)
STEPS_PER_M = PPR / M_PER_REV  # ~1968.5039 steps/m


# ----------------------------
# Safety / shop constraints (edit if needed)
# ----------------------------
D_SAFE_MAX_M = 2.50     # m (avoid wave generator clash)
A_SAFE_MAX = 10.0       # m/s^2 (belt slip risk above this; ~5 is comfy)
V_SAFE_MAX = 3.0        # m/s (software guard; DAQ can do more, mechanics may not)
V_MIN = 0.01            # m/s (avoid silly low)


@dataclass
class Channels:
    # Your wiring (signals)
    step_ctr: str         # e.g. "Dev1/ctr0"
    step_out_term: str    # e.g. "/Dev1/PFI12" (CTR0 OUT default)
    dir_line: str         # e.g. "Dev1/port0/line0"  (P0.0)
    en_line: str          # e.g. "Dev1/port0/line1"  (P0.1)


def pick_device_name() -> str:
    devs = list(System.local().devices)
    if not devs:
        raise RuntimeError("No NI-DAQ device found. Check NI-DAQmx install + USB connection.")

    if len(devs) == 1:
        return devs[0].name

    default_idx = 0
    for i, d in enumerate(devs):
        if d.name.lower() == "dev1":
            default_idx = i
            break

    print("\nMultiple NI-DAQ devices detected:")
    for i, d in enumerate(devs, start=1):
        default_tag = " (default)" if (i - 1) == default_idx else ""
        print(f"  {i}. {d.name}{default_tag}")

    while True:
        choice = input(f"Select device [1-{len(devs)}] (Enter for default): ").strip()
        if choice == "":
            return devs[default_idx].name
        if choice.isdigit():
            idx = int(choice) - 1
            if 0 <= idx < len(devs):
                return devs[idx].name
        print("Invalid selection. Please enter a valid device number.")


def set_dir_en(ch: Channels, direction: int, enable: int) -> None:
    """Set DIR and EN with a single DO task."""
    with nidaqmx.Task() as t:
        t.do_channels.add_do_chan(ch.dir_line)
        t.do_channels.add_do_chan(ch.en_line)
        # write two lines in the order they were added: [DIR, EN]
        t.write([bool(direction), bool(enable)], auto_start=True)


def run_pulse_train(ch: Channels, freq_hz: float, pulses: int, duty: float = 0.5) -> None:
    """Generate a finite pulse train on CTR0OUT at freq_hz for 'pulses' pulses."""
    if pulses <= 0:
        return
    if freq_hz <= 0:
        raise ValueError("Frequency must be > 0")

    with nidaqmx.Task() as t:
        co = t.co_channels.add_co_pulse_chan_freq(
            counter=ch.step_ctr,
            units=FrequencyUnits.HZ,
            idle_state=Level.LOW,
            initial_delay=0.0,
            freq=freq_hz,
            duty_cycle=duty,
        )
        # Force the output terminal (important; avoids routing surprises)
        co.co_pulse_term = ch.step_out_term

        # Finite pulse train: implicit timing, samps_per_chan = number of pulses
        t.timing.cfg_implicit_timing(
            sample_mode=AcquisitionType.FINITE,
            samps_per_chan=int(pulses),
        )

        t.start()
        # Wait long enough: pulses/freq + small cushion
        timeout = max(2.0, (pulses / freq_hz) + 2.0)
        t.wait_until_done(timeout=timeout)
        t.stop()


def plan_trapezoid(distance_m: float, v_mps: float, a_mps2: float):
    """
    Returns (triangular?, t_total, v_peak, t_acc, t_cruise)
    Standard trapezoid/triangle with symmetric accel/decel.
    """
    D = abs(distance_m)
    V = abs(v_mps)
    A = abs(a_mps2)

    if D <= 0:
        return True, 0.0, 0.0, 0.0, 0.0
    if V <= 0 or A <= 0:
        raise ValueError("V and A must be > 0")

    # distance needed to accel to V: d_acc = V^2/(2A)
    d_acc = (V * V) / (2.0 * A)

    if 2.0 * d_acc >= D:
        # triangle: peak speed smaller
        v_peak = math.sqrt(A * D)
        t_acc = v_peak / A
        t_cruise = 0.0
        t_total = 2.0 * t_acc
        return True, t_total, v_peak, t_acc, t_cruise
    else:
        # trapezoid
        v_peak = V
        t_acc = V / A
        d_cruise = D - 2.0 * d_acc
        t_cruise = d_cruise / V
        t_total = 2.0 * t_acc + t_cruise
        return False, t_total, v_peak, t_acc, t_cruise


def constant_velocity_move(
    ch: Channels,
    distance_m: float,
    v_mps: float,
    dwell_s: float = 0.0,
):
    """
    Move at a single constant velocity (no accel/decel ramp).
    Good for low speeds where the motor can start/stop without ramping.
    """
    if abs(distance_m) > D_SAFE_MAX_M:
        print(f"[CLAMP] distance {distance_m:.3f} m -> {math.copysign(D_SAFE_MAX_M, distance_m):.3f} m")
        distance_m = math.copysign(D_SAFE_MAX_M, distance_m)
    if v_mps > V_SAFE_MAX:
        print(f"[CLAMP] speed {v_mps:.3f} -> {V_SAFE_MAX:.3f} m/s")
        v_mps = V_SAFE_MAX
    if v_mps < V_MIN:
        print(f"[CLAMP] speed {v_mps:.3f} -> {V_MIN:.3f} m/s")
        v_mps = V_MIN

    direction = 0 if distance_m >= 0 else 1
    total_pulses = int(round(abs(distance_m) * STEPS_PER_M))
    freq_hz = max(1.0, v_mps * STEPS_PER_M)

    if total_pulses <= 0:
        print("Distance too small -> 0 pulses.")
        return

    print("\n--- PLAN (constant velocity) ---")
    print(f"distance (m): {distance_m:.4f}   pulses: {total_pulses}")
    print(f"speed (m/s): {v_mps:.4f}   freq (Hz): {freq_hz:.1f}")

    set_dir_en(ch, direction=direction, enable=0)
    set_dir_en(ch, direction=direction, enable=1)
    time.sleep(0.005)

    t0 = time.time()
    run_pulse_train(ch, freq_hz=freq_hz, pulses=total_pulses, duty=0.5)
    set_dir_en(ch, direction=direction, enable=0)
    t1 = time.time()

    actual_t = t1 - t0
    avg_speed = abs(distance_m) / actual_t if actual_t > 0 else 0.0
    print("\n--- RESULT ---")
    print(f"elapsed(s): {actual_t:.3f}  avg_speed(m/s): {avg_speed:.3f}")

    if dwell_s > 0:
        time.sleep(dwell_s)


def segmented_move(
    ch: Channels,
    distance_m: float,
    v_mps: float,
    a_mps2: float,
    n_segments_acc: int = 60,
    dwell_s: float = 0.0,
):
    """
    Runs a trapezoid/triangle by emitting multiple finite pulse trains.
    DIR is set once. EN is held high for the whole move.
    """
    # ---- clamp to your stated constraints ----
    if abs(distance_m) > D_SAFE_MAX_M:
        print(f"[CLAMP] distance {distance_m:.3f} m -> {math.copysign(D_SAFE_MAX_M, distance_m):.3f} m")
        distance_m = math.copysign(D_SAFE_MAX_M, distance_m)
    if a_mps2 > A_SAFE_MAX:
        print(f"[CLAMP] accel {a_mps2:.3f} -> {A_SAFE_MAX:.3f} m/s^2")
        a_mps2 = A_SAFE_MAX
    if v_mps > V_SAFE_MAX:
        print(f"[CLAMP] speed {v_mps:.3f} -> {V_SAFE_MAX:.3f} m/s")
        v_mps = V_SAFE_MAX
    if v_mps < V_MIN:
        print(f"[CLAMP] speed {v_mps:.3f} -> {V_MIN:.3f} m/s")
        v_mps = V_MIN
    if a_mps2 <= 0:
        raise ValueError("Acceleration must be > 0")

    direction = 0 if distance_m >= 0 else 1  # choose convention; swap later if reversed

    # Planner (physics space)
    tri, t_total, v_peak, t_acc, t_cruise = plan_trapezoid(distance_m, v_mps, a_mps2)

    # Convert to steps domain for total pulse count
    total_pulses = int(round(abs(distance_m) * STEPS_PER_M))
    if total_pulses <= 0:
        print("Distance too small -> 0 pulses.")
        return

    print("\n--- PLAN ---")
    print(f"distance (m): {distance_m:.4f}   pulses: {total_pulses}")
    print(f"speed req (m/s): {v_mps:.4f}   accel (m/s^2): {a_mps2:.4f}")
    print(f"profile: {'TRIANGLE' if tri else 'TRAPEZOID'}  t_total(s)≈{t_total:.3f}  v_peak(m/s)={v_peak:.3f}")
    print(f"steps_per_m: {STEPS_PER_M:.3f}")

    # Build segment pulses
    # We’ll allocate pulses by segment duration fractions (approx); keep total exact by fixing last segment.
    segments = []

    # Accel segments: ramp from 0 -> v_peak
    n_segments_acc = max(10, int(n_segments_acc))
    for i in range(1, n_segments_acc + 1):
        frac = i / n_segments_acc
        v_i = v_peak * frac
        f_i = max(1.0, v_i * STEPS_PER_M)  # Hz
        segments.append(("acc", f_i))

    # Cruise segments (one chunk if any)
    if t_cruise > 1e-6:
        f_cruise = max(1.0, v_peak * STEPS_PER_M)
        segments.append(("cruise", f_cruise))

    # Decel segments: v_peak -> ~0
    for i in range(n_segments_acc - 1, -1, -1):
        frac = i / n_segments_acc
        v_i = max(0.01, v_peak * frac)
        f_i = max(1.0, v_i * STEPS_PER_M)
        segments.append(("dec", f_i))

    # Allocate pulses proportional to time spent.
    # Use physics to estimate time of each segment:
    # - accel/decel segments: equal time slices of t_acc
    # - cruise segment: t_cruise
    dt_acc = t_acc / n_segments_acc if t_acc > 0 else 0.0

    times = []
    for tag, f in segments:
        if tag == "acc" or tag == "dec":
            times.append(dt_acc)
        else:
            times.append(t_cruise)

    t_sum = sum(times) if sum(times) > 0 else 1.0
    pulses_alloc = []
    pulses_used = 0
    for idx, (seg, dt) in enumerate(zip(segments, times)):
        if idx == len(segments) - 1:
            p = total_pulses - pulses_used  # force exact total
        else:
            p = int(round(total_pulses * (dt / t_sum)))
            # ensure at least 1 pulse if there’s time and we still have pulses left
            if p <= 0 and (total_pulses - pulses_used) > 0:
                p = 1
        pulses_used += p
        pulses_alloc.append(p)

    # Fix any rounding overshoot
    if pulses_used != total_pulses:
        pulses_alloc[-1] += (total_pulses - pulses_used)

    # ---- execute ----
    # Set DIR, keep EN low first
    set_dir_en(ch, direction=direction, enable=0)

    # Enable (your physical switch must be closed)
    set_dir_en(ch, direction=direction, enable=1)
    time.sleep(0.005)

    t0 = time.time()
    for (tag, f_hz), p in zip(segments, pulses_alloc):
        if p <= 0:
            continue
        run_pulse_train(ch, freq_hz=f_hz, pulses=p, duty=0.5)

    # disable
    set_dir_en(ch, direction=direction, enable=0)
    t1 = time.time()

    actual_t = t1 - t0
    avg_speed = abs(distance_m) / actual_t if actual_t > 0 else 0.0
    print("\n--- RESULT ---")
    print(f"elapsed(s): {actual_t:.3f}  avg_speed(m/s): {avg_speed:.3f}")

    if dwell_s > 0:
        time.sleep(dwell_s)


def main():
    dev = pick_device_name()
    print(f"Using device: {dev}")

    ch = Channels(
        step_ctr=f"{dev}/ctr0",
        step_out_term=f"/{dev}/PFI12",          # CTR0 OUT default from your table
        dir_line=f"{dev}/port0/line0",          # P0.0
        en_line=f"{dev}/port0/line1",           # P0.1
    )

    print("\nEnter parameters. Suggested:")
    print(f"  distance <= {D_SAFE_MAX_M} m  (avoid wave generator)")
    print("  accel ~5 m/s^2 (safe), <=10 m/s^2 (slip risk)")
    print("  speed choose what you need; start <=1.0 m/s and work up")
    print("\nType values when prompted.\n")

    while True:
        try:
            dist = float(input("Distance (m), sign sets direction (e.g. 0.8 or -0.8): ").strip())
            spd  = float(input("Speed (m/s): ").strip())

            mode = input("Profile: (c)onstant velocity or (t)rapezoid? [c/T]: ").strip().lower()
            use_constant = mode == "c"

            acc = 0.0
            if not use_constant:
                acc = float(input("Accel (m/s^2): ").strip())

            do_loop = input("Loop 5 round-trips? (y/N): ").strip().lower() == "y"
            dwell = 0.0
            if do_loop:
                dwell = float(input("Dwell between legs (s), e.g. 0.5: ").strip() or "0.0")

            if not do_loop:
                if use_constant:
                    constant_velocity_move(ch, dist, spd, dwell_s=0.0)
                else:
                    segmented_move(ch, dist, spd, acc, n_segments_acc=60, dwell_s=0.0)
            else:
                d = abs(dist)
                for k in range(5):
                    print(f"\n=== Round-trip {k+1}/5 forward ===")
                    if use_constant:
                        constant_velocity_move(ch, +d, spd, dwell_s=dwell)
                    else:
                        segmented_move(ch, +d, spd, acc, n_segments_acc=60, dwell_s=dwell)
                    print(f"\n=== Round-trip {k+1}/5 back ===")
                    if use_constant:
                        constant_velocity_move(ch, -d, spd, dwell_s=dwell)
                    else:
                        segmented_move(ch, -d, spd, acc, n_segments_acc=60, dwell_s=dwell)

            again = input("\nRun another? (Y/n): ").strip().lower()
            if again == "n":
                break

        except KeyboardInterrupt:
            print("\nStopping.")
            break
        except Exception as e:
            print(f"\nERROR: {e}")
            print("Fix the input / wiring / device name and retry.\n")


if __name__ == "__main__":
    main()
