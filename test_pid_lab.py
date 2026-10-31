"""Automated Test Suite for PID Controller, Plants, and Control Metrics."""

import unittest
from metrics import analyze_step_response
from pid import PIDController
from plants import DCMotorPlant, ThermalOvenPlant


class TestPIDController(unittest.TestCase):
    def test_proportional_action(self):
        pid = PIDController(kp=2.0, ki=0.0, kd=0.0, dt=0.01, out_min=-100, out_max=100)
        # SP = 10, PV = 4 -> Error = 6 -> Output = 2.0 * 6 = 12.0
        out = pid.compute(setpoint=10.0, measurement=4.0)
        self.assertAlmostEqual(out, 12.0)

    def test_integral_accumulation(self):
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, dt=0.1, out_min=-100, out_max=100)
        # Constant error of 5.0 for 2 steps
        out1 = pid.compute(setpoint=5.0, measurement=0.0)  # dt * error = 0.5
        out2 = pid.compute(setpoint=5.0, measurement=0.0)  # dt * error = 1.0
        self.assertAlmostEqual(out1, 0.5)
        self.assertAlmostEqual(out2, 1.0)

    def test_anti_windup_clamping(self):
        # Limit output to [0, 50]
        pid = PIDController(kp=10.0, ki=10.0, kd=0.0, dt=0.1, out_min=0.0, out_max=50.0)
        # Massive error of 100
        for _ in range(20):
            out = pid.compute(setpoint=100.0, measurement=0.0)
            self.assertLessEqual(out, 50.0)

        # Integral must not have blown up to infinity
        self.assertLessEqual(pid.integral, 5.0)

    def test_derivative_kick_prevention(self):
        # Derivative on measurement: sudden setpoint step should NOT spike derivative
        pid = PIDController(kp=0.0, ki=0.0, kd=5.0, dt=0.1, out_min=-100, out_max=100)
        # Step from SP=0 to SP=100 while PV remains 0
        out = pid.compute(setpoint=100.0, measurement=0.0)
        # Derivative on measurement is 0 because measurement did not change
        self.assertAlmostEqual(out, 0.0)


class TestPlants(unittest.TestCase):
    def test_dc_motor_response(self):
        motor = DCMotorPlant(dt=0.001)
        # Apply constant 12V
        for _ in range(500):
            rpm = motor.step(voltage=12.0)
        self.assertGreater(rpm, 500.0)

    def test_thermal_oven_response(self):
        oven = ThermalOvenPlant(dt=0.1, ambient_temp=25.0)
        # Zero power -> remains at ambient
        temp_idle = oven.step(duty_cycle_percent=0.0)
        self.assertAlmostEqual(temp_idle, 25.0, places=1)

        # 50% power -> temperature rises
        for _ in range(200):
            temp_heated = oven.step(duty_cycle_percent=50.0)
        self.assertGreater(temp_heated, 30.0)


class TestClosedLoopConvergence(unittest.TestCase):
    def test_motor_speed_regulation(self):
        dt = 0.001
        pid = PIDController(
            kp=0.08, ki=0.85, kd=0.002, dt=dt, out_min=0.0, out_max=24.0
        )
        motor = DCMotorPlant(dt=dt)
        sp = 1000.0  # RPM target

        times, rpms = [], []
        for step in range(1500):  # 1.5 seconds
            t = step * dt
            rpm = motor.rpm
            v = pid.compute(sp, rpm)
            motor.step(v)
            times.append(t)
            rpms.append(rpm)

        metrics = analyze_step_response(times, rpms, sp)
        # Steady state error must be less than 2% of setpoint
        self.assertLess(metrics.steady_state_error, 20.0)
        # Must have settled
        self.assertIsNotNone(metrics.settling_time)
        self.assertLess(metrics.settling_time, 1.2)


class TestMetricsAnalysis(unittest.TestCase):
    def test_synthetic_metrics(self):
        times = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
        # Step response reaching 100 with small overshoot to 105
        values = [0.0, 20.0, 80.0, 105.0, 101.0, 100.0]
        metrics = analyze_step_response(times, values, setpoint=100.0)

        self.assertAlmostEqual(metrics.peak_value, 105.0)
        self.assertAlmostEqual(metrics.overshoot_percent, 5.0)
        self.assertAlmostEqual(metrics.steady_state_error, 0.0)


if __name__ == "__main__":
    unittest.main()
