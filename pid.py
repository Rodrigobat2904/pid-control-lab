"""Discrete-Time PID Controller with Anti-Windup and Derivative Filtering.

Designed for embedded control applications (DC motor velocity/position,
thermal plants, and robotics actuators). Portable architecture ready to
translate into microcontroller C/C++ firmware (STM32, ESP32, AVR).
"""

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class PIDGains:
    kp: float
    ki: float
    kd: float


class PIDController:
    """Industrial discrete-time PID controller.

    Features:
      - Anti-windup clamping (conditional integration / saturated clamp)
      - Derivative on measurement (prevents setpoint step derivative kick)
      - First-order derivative low-pass filter (attenuates high-frequency ADC noise)
      - Configurable output saturation bounds (e.g., 0% to 100% PWM)
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        dt: float,
        out_min: float = -100.0,
        out_max: float = 100.0,
        derivative_filter_alpha: float = 0.1,
    ):
        if dt <= 0:
            raise ValueError("Time step dt must be positive.")
        if out_min >= out_max:
            raise ValueError("out_min must be strictly less than out_max.")

        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.dt = float(dt)
        self.out_min = float(out_min)
        self.out_max = float(out_max)
        self.alpha = float(derivative_filter_alpha)

        self.reset()

    def reset(self) -> None:
        """Reset internal accumulator and historical states."""
        self.integral = 0.0
        self.prev_measurement: Optional[float] = None
        self.filtered_derivative = 0.0
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0
        self.last_output = 0.0

    def compute(self, setpoint: float, measurement: float) -> float:
        """Calculate next control effort u[k] given setpoint and process variable.

        Args:
            setpoint: Target reference value r[k].
            measurement: Process variable / feedback measurement y[k].

        Returns:
            Bounded control effort u[k] clamped to [out_min, out_max].
        """
        error = setpoint - measurement

        # 1. Proportional term
        self.last_p = self.kp * error

        # 2. Derivative term (derivative-on-measurement to avoid setpoint kick)
        if self.prev_measurement is None:
            raw_derivative = 0.0
        else:
            # -d(measurement)/dt == d(error)/dt when setpoint is constant
            raw_derivative = -(measurement - self.prev_measurement) / self.dt

        # Exponential moving filter on derivative: y_f[k] = alpha*y[k] + (1-alpha)*y_f[k-1]
        self.filtered_derivative = (
            self.alpha * raw_derivative + (1.0 - self.alpha) * self.filtered_derivative
        )
        self.last_d = self.kd * self.filtered_derivative
        self.prev_measurement = measurement

        # 3. Integral term with anti-windup clamping
        # Prospective control output without new integral addition
        tentative_u = self.last_p + (self.ki * (self.integral + error * self.dt)) + self.last_d

        # Clamping anti-windup: only accumulate integral if output is not saturated,
        # or if error sign tends to pull the output back inside linear range.
        if self.out_min < tentative_u < self.out_max:
            self.integral += error * self.dt
        elif tentative_u >= self.out_max and error < 0:
            self.integral += error * self.dt
        elif tentative_u <= self.out_min and error > 0:
            self.integral += error * self.dt

        self.last_i = self.ki * self.integral

        # Total control effort & clamp
        raw_output = self.last_p + self.last_i + self.last_d
        clamped_output = max(self.out_min, min(self.out_max, raw_output))
        self.last_output = clamped_output

        return clamped_output

    def get_components(self) -> Tuple[float, float, float]:
        """Return (P, I, D) component contributions of the last computation."""
        return (self.last_p, self.last_i, self.last_d)
