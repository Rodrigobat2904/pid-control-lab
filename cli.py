"""Command Line Interface for PID Simulation and Benchmark.

Run:
  python cli.py --plant motor --sp 1200 --duration 1.5
  python cli.py --plant thermal --sp 80.0 --duration 60.0
"""

import argparse
import sys
from metrics import analyze_step_response
from pid import PIDController
from plants import DCMotorPlant, ThermalOvenPlant
from visualizer import ascii_plot, export_html_chart


def run_motor_sim(sp: float = 1200.0, duration: float = 1.2):
    dt = 0.001
    steps = int(duration / dt)

    # Tuned PID for DC motor velocity
    pid = PIDController(
        kp=0.08,
        ki=0.85,
        kd=0.002,
        dt=dt,
        out_min=0.0,
        out_max=24.0,  # 24V supply limit
        derivative_filter_alpha=0.2,
    )
    motor = DCMotorPlant(dt=dt)

    times = []
    rpms = []
    efforts = []

    for step in range(steps):
        t = step * dt
        current_rpm = motor.rpm

        # Apply disturbance torque at t = 0.6s
        load = 0.005 if t >= 0.6 else 0.0

        v_in = pid.compute(sp, current_rpm)
        motor.step(v_in, load_torque=load)

        times.append(t)
        rpms.append(current_rpm)
        efforts.append(v_in)

    return times, rpms, efforts, sp, "DC Motor Velocity Control (RPM)"


def run_thermal_sim(sp: float = 75.0, duration: float = 80.0):
    dt = 0.1
    steps = int(duration / dt)

    # Tuned PID for thermal oven
    pid = PIDController(
        kp=6.0,
        ki=0.12,
        kd=25.0,
        dt=dt,
        out_min=0.0,
        out_max=100.0,  # 0 to 100% PWM
        derivative_filter_alpha=0.15,
    )
    oven = ThermalOvenPlant(dt=dt, ambient_temp=25.0)

    times = []
    temps = []
    efforts = []

    for step in range(steps):
        t = step * dt
        current_temp = oven.temp

        duty = pid.compute(sp, current_temp)
        oven.step(duty)

        times.append(t)
        temps.append(current_temp)
        efforts.append(duty)

    return times, temps, efforts, sp, "Thermal Chamber Temperature Control (C)"


def main():
    parser = argparse.ArgumentParser(description="PID Control Simulation Workbench")
    parser.add_argument(
        "--plant",
        choices=["motor", "thermal"],
        default="motor",
        help="Physical plant to simulate",
    )
    parser.add_argument("--sp", type=float, default=None, help="Target setpoint")
    parser.add_argument("--duration", type=float, default=None, help="Simulation duration (s)")
    parser.add_argument("--html", type=str, default=None, help="Export HTML report to path")

    args = parser.parse_args()

    if args.plant == "motor":
        sp = args.sp if args.sp is not None else 1200.0
        dur = args.duration if args.duration is not None else 1.2
        times, values, efforts, target, title = run_motor_sim(sp, dur)
        init_val = 0.0
    else:
        sp = args.sp if args.sp is not None else 75.0
        dur = args.duration if args.duration is not None else 80.0
        times, values, efforts, target, title = run_thermal_sim(sp, dur)
        init_val = 25.0

    print("=" * 65)
    print(f"  PID CONTROL SIMULATION: {title}")
    print("=" * 65)
    print(ascii_plot(times, values, target))
    print("-" * 65)

    metrics = analyze_step_response(times, values, target, initial_value=init_val)
    print(metrics.summary())
    print("=" * 65)

    if args.html:
        export_html_chart(times, values, efforts, target, title, args.html)
        print(f"Interactive HTML report exported to: {args.html}")


if __name__ == "__main__":
    main()
