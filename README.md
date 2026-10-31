# pid-control-lab

> Discrete-time PID controller workbench with anti-windup clamping, derivative filtering, electro-mechanical DC motor & thermal plant simulation, and automated transient metrics analysis.

[![Tests](https://img.shields.io/badge/tests-8%20passed-brightgreen.svg)]()
[![Language](https://img.shields.io/badge/python-3.8%2B-blue.svg)]()
[![Domain](https://img.shields.io/badge/domain-Control%20Systems%20%7C%20Electronics-orange.svg)]()
[![Zero Dependencies](https://img.shields.io/badge/dependencies-zero-success.svg)]()

---

## ⚡ Overview

In embedded control engineering, running software-in-the-loop (SIL) simulations before flashing firmware onto microcontrollers prevents actuator burnout and instability.

**`pid-control-lab`** provides a discrete-time PID algorithm engineered to mimic real-world embedded constraints (ADC quantization, output voltage/PWM saturation, and derivative noise), paired with physical dynamic plant models.

---

## 📐 Mathematical Formulation

### 1. Discrete PID Equations
The controller operates in discrete time steps $k$ with sample period $\Delta t$:

$$u[k] = P[k] + I[k] + D[k]$$

- **Proportional**:
  $$P[k] = K_p \cdot e[k]$$
  where $e[k] = r[k] - y[k]$ (Setpoint $-$ Feedback).

- **Integral with Anti-Windup Clamping**:
  To prevent integrator windup when actuators saturate at $u_{\min}$ or $u_{\max}$:
  $$I[k] = K_i \cdot \sum_{j=0}^{k} e[j] \Delta t \quad \text{if } u_{\min} < u[k] < u_{\max}$$

- **Derivative on Measurement**:
  To eliminate **derivative kick** during instantaneous setpoint steps ($r[k] \neq r[k-1]$), the derivative operates directly on the feedback measurement $y[k]$:
  $$D_{\text{raw}}[k] = - K_d \frac{y[k] - y[k-1]}{\Delta t}$$
  
  Paired with an exponential low-pass filter ($\alpha$) to attenuate high-frequency sensor noise:
  $$D_{\text{filt}}[k] = \alpha D_{\text{raw}}[k] + (1 - \alpha) D_{\text{filt}}[k-1]$$

---

## 🔬 Physical Dynamic Models

### 1. 2nd-Order DC Motor Model
Couples electrical armature dynamics with rotational mechanics:
$$\frac{di(t)}{dt} = \frac{V(t) - R \cdot i(t) - K_e \cdot \omega(t)}{L}$$
$$\frac{d\omega(t)}{dt} = \frac{K_t \cdot i(t) - B \cdot \omega(t) - \tau_{\text{load}}}{J}$$

### 2. 1st-Order Thermal Oven with Delay
Thermal capacity and ambient heat dissipation with transport delay $\tau_d$:
$$\frac{dT(t)}{dt} = \frac{P_{\text{heater}}(t - \tau_d) - k_{\text{loss}} \cdot (T(t) - T_{\text{amb}})}{C_{\text{th}}}$$

---

## 📊 Transient Performance Metrics

Automated extraction of standard control benchmark parameters:
- **Rise Time ($t_r$)**: Elapsed time for response to climb from 10% to 90% of setpoint.
- **Percent Overshoot ($M_p$)**: Peak percentage exceeding target setpoint.
- **Settling Time ($t_s$)**: Time required for curve to permanently stay within $\pm 2\%$ tolerance band.
- **Steady-State Error ($e_{ss}$)**: Remaining error at steady state ($|r[\infty] - y[\infty]|$).

---

## 🚀 Quickstart & Usage

### 1. Run Automated Unit Tests (Zero External Dependencies)
```bash
python -m unittest -v
```

### 2. Simulate DC Motor Velocity Control (with ASCII Terminal Plot)
```bash
python cli.py --plant motor --sp 1200 --duration 1.2
```
Output:
```text
=================================================================
  PID CONTROL SIMULATION: DC Motor Velocity Control (RPM)
=================================================================
   Max: 1380.0 |                                                            
   SP:  1200.0 |-------------------+++++++++++++++++++++++++++++++++++++++++
               |                ***                                         
               |              **                                            
               |            **                                              
               |           *                                                
               |         **                                                 
               |        *                                                   
               |       *                                                    
               |      *                                                     
               |     *                                                      
               |   **                                                       
               |  *                                                         
               | *                                                          
   Min:    0.0 |*                                                           
                +------------------------------------------------------------
                0.0 s                                                1.20 s
   Legend: [*] Process Variable (PV)   [-] Target Setpoint (SP)
-----------------------------------------------------------------
Setpoint:            1200.00
Final Value:         1200.10
Peak Value:          1235.46
Percent Overshoot:   2.95%
Rise Time (10-90%):  0.267 s
Settling Time (2%):  0.550 s
Steady-State Error:  0.0979
=================================================================
```

### 3. Simulate Thermal Chamber Control & Export Web Chart
```bash
python cli.py --plant thermal --sp 75.0 --duration 80.0 --html report.html
```

---

## 🛠️ Microcontroller Porting Guide (C99 / STM32 / ESP32)

The Python implementation mirrors this production C firmware pattern:

```c
typedef struct {
    float Kp, Ki, Kd;
    float dt, out_min, out_max;
    float integral, prev_measurement, filtered_d;
} PID_t;

float PID_Compute(PID_t *pid, float setpoint, float measurement) {
    float error = setpoint - measurement;
    float p_term = pid->Kp * error;
    
    // Derivative on measurement with 1st-order low-pass filter
    float d_raw = -(measurement - pid->prev_measurement) / pid->dt;
    pid->filtered_d = 0.2f * d_raw + 0.8f * pid->filtered_d;
    float d_term = pid->Kd * pid->filtered_d;
    pid->prev_measurement = measurement;

    // Anti-windup conditional integration
    float tentative = p_term + (pid->Ki * (pid->integral + error * pid->dt)) + d_term;
    if (tentative > pid->out_min && tentative < pid->out_max) {
        pid->integral += error * pid->dt;
    }
    float i_term = pid->Ki * pid->integral;

    float output = p_term + i_term + d_term;
    if (output > pid->out_max) return pid->out_max;
    if (output < pid->out_min) return pid->out_min;
    return output;
}
```

---

## 📜 License
MIT License. Open source for academic, firmware simulation, and portfolio demonstration purposes.
