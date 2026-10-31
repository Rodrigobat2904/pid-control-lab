"""Physical Plant Simulation Models for Closed-Loop Control Verification.

Includes:
1. DCMotorPlant: 2nd-order electro-mechanical model (armature current + angular velocity).
2. ThermalOvenPlant: 1st-order thermal capacity + ambient heat loss + transport delay.
"""

from collections import deque
import math
from typing import Optional


class DCMotorPlant:
    """Electro-mechanical model of a permanent magnet DC motor.

    Equations:
      di/dt = (V - R*i - Ke*omega) / L
      domega/dt = (Kt*i - B*omega - tau_load) / J

    Where:
      R: Armature resistance (Ohms)
      L: Armature inductance (Henries)
      Kt: Torque constant (N.m / A)
      Ke: Back-EMF constant (V / (rad/s))
      J: Rotor moment of inertia (kg.m^2)
      B: Viscous friction coefficient (N.m.s)
    """

    def __init__(
        self,
        dt: float,
        resistance: float = 2.0,
        inductance: float = 0.05,
        kt: float = 0.05,
        ke: float = 0.05,
        inertia: float = 0.001,
        friction: float = 0.002,
    ):
        self.dt = dt
        self.r = resistance
        self.l = inductance
        self.kt = kt
        self.ke = ke
        self.j = inertia
        self.b = friction

        self.reset()

    def reset(self) -> None:
        self.current = 0.0  # Amperes
        self.omega = 0.0  # rad/s
        self.theta = 0.0  # radians (position)

    @property
    def rpm(self) -> float:
        """Angular velocity converted to RPM."""
        return self.omega * 60.0 / (2.0 * math.pi)

    def step(self, voltage: float, load_torque: float = 0.0) -> float:
        """Euler integration step under applied terminal voltage (Volts).

        Returns:
            Current velocity in RPM.
        """
        # Electrical derivative
        di_dt = (voltage - (self.r * self.current) - (self.ke * self.omega)) / self.l
        # Mechanical derivative
        domega_dt = (
            (self.kt * self.current) - (self.b * self.omega) - load_torque
        ) / self.j

        # State updates
        self.current += di_dt * self.dt
        self.omega = max(0.0, self.omega + domega_dt * self.dt)
        self.theta += self.omega * self.dt

        return self.rpm


class ThermalOvenPlant:
    """1st-order Thermal model with ambient cooling and transport delay.

    Equations:
      dT/dt = (P_heater - k_loss * (T - T_ambient)) / C_thermal
    """

    def __init__(
        self,
        dt: float,
        thermal_capacity: float = 120.0,  # Joules / degree Celsius
        heat_loss_coeff: float = 1.8,  # Watts / degree Celsius
        ambient_temp: float = 25.0,  # degree Celsius
        max_heater_power: float = 250.0,  # Watts at 100% duty cycle
        dead_time_seconds: float = 0.4,
    ):
        self.dt = dt
        self.c_th = thermal_capacity
        self.k_loss = heat_loss_coeff
        self.t_ambient = ambient_temp
        self.p_max = max_heater_power

        # Delay queue for actuator transport delay
        self.delay_steps = max(1, int(round(dead_time_seconds / dt)))
        self.reset()

    def reset(self) -> None:
        self.temp = self.t_ambient
        self.delay_buffer = deque([0.0] * self.delay_steps, maxlen=self.delay_steps)

    def step(self, duty_cycle_percent: float, ambient_temp: Optional[float] = None) -> float:
        """Step simulation. duty_cycle_percent: 0.0 to 100.0%."""
        if ambient_temp is not None:
            self.t_ambient = ambient_temp

        duty = max(0.0, min(100.0, duty_cycle_percent))
        self.delay_buffer.append(duty)
        effective_duty = self.delay_buffer[0]

        p_in = (effective_duty / 100.0) * self.p_max
        heat_lost = self.k_loss * (self.temp - self.t_ambient)
        dt_temp = (p_in - heat_lost) / self.c_th

        self.temp += dt_temp * self.dt
        return self.temp
