"""Step Response Analysis and Transient Performance Metrics.

Calculates key industrial control criteria:
  - Percent Overshoot (Mp)
  - Rise Time (tr, 10% -> 90% threshold)
  - Settling Time (ts, 2% tolerance band)
  - Steady-State Error (ess)
"""

from dataclasses import dataclass
from typing import List, Optional


@dataclass
class StepResponseMetrics:
    setpoint: float
    peak_value: float
    overshoot_percent: float
    rise_time: Optional[float]
    settling_time: Optional[float]
    steady_state_error: float
    final_value: float

    def summary(self) -> str:
        rt_str = f"{self.rise_time:.3f} s" if self.rise_time is not None else "N/A"
        st_str = f"{self.settling_time:.3f} s" if self.settling_time is not None else "N/A"
        return (
            f"Setpoint:            {self.setpoint:.2f}\n"
            f"Final Value:         {self.final_value:.2f}\n"
            f"Peak Value:          {self.peak_value:.2f}\n"
            f"Percent Overshoot:   {self.overshoot_percent:.2f}%\n"
            f"Rise Time (10-90%):  {rt_str}\n"
            f"Settling Time (2%):  {st_str}\n"
            f"Steady-State Error:  {self.steady_state_error:.4f}"
        )


def analyze_step_response(
    time_series: List[float],
    values: List[float],
    setpoint: float,
    initial_value: float = 0.0,
    tolerance_band: float = 0.02,
) -> StepResponseMetrics:
    """Analyze step response trajectory against target setpoint."""
    if not time_series or len(time_series) != len(values):
        raise ValueError("time_series and values must be non-empty and equal length.")

    step_amplitude = setpoint - initial_value
    if abs(step_amplitude) < 1e-9:
        raise ValueError("Step amplitude cannot be zero.")

    peak_value = max(values)
    overshoot = 0.0
    if peak_value > setpoint:
        overshoot = ((peak_value - setpoint) / abs(step_amplitude)) * 100.0

    # Rise time (10% to 90%)
    val_10 = initial_value + 0.10 * step_amplitude
    val_90 = initial_value + 0.90 * step_amplitude

    t_10: Optional[float] = None
    t_90: Optional[float] = None

    for t, v in zip(time_series, values):
        if t_10 is None and v >= val_10:
            t_10 = t
        if t_90 is None and v >= val_90:
            t_90 = t
            break

    rise_time: Optional[float] = None
    if t_10 is not None and t_90 is not None:
        rise_time = t_90 - t_10

    # Settling time (within +/- tolerance_band of setpoint permanently)
    band_lower = setpoint - abs(step_amplitude) * tolerance_band
    band_upper = setpoint + abs(step_amplitude) * tolerance_band

    settling_time: Optional[float] = None
    # Scan backward from the end
    last_violating_index = None
    for idx in range(len(values) - 1, -1, -1):
        if values[idx] < band_lower or values[idx] > band_upper:
            last_violating_index = idx
            break

    if last_violating_index is None:
        # Never left the band
        settling_time = time_series[0]
    elif last_violating_index < len(values) - 1:
        settling_time = time_series[last_violating_index + 1]

    final_val = values[-1]
    ess = abs(setpoint - final_val)

    return StepResponseMetrics(
        setpoint=setpoint,
        peak_value=peak_value,
        overshoot_percent=overshoot,
        rise_time=rise_time,
        settling_time=settling_time,
        steady_state_error=ess,
        final_value=final_val,
    )
