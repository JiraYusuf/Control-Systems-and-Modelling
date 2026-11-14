# PID-Based Automatic Water Tank Level Control (C++)

## 1. Project overview

This project models and simulates an automatic water tank level control
system. A PID controller adjusts pump inflow to track a desired tank level.
The simulation includes actuator saturation and a downstream demand
disturbance, making it a useful example of feedback control under practical
constraints.

The tank dynamics are integrated with the classical fourth-order
Runge-Kutta (RK4) method. The controller runs at a sampled interval and
holds its pump command between controller updates.

## 2. Engineering problem

Maintain a target water level when:
- the target level changes;
- the pump has a finite minimum and maximum flow rate; and
- the outlet demand increases during operation.

This represents a simplified industrial level-control loop.

## 3. Mathematical process model

The tank mass balance is:

`A * dh/dt = q_in - q_out`

The outlet flow is modelled as:

`q_out = C_out * sqrt(h)`

Therefore:

`dh/dt = (q_in - C_out * sqrt(h)) / A`

Where:
- `h` is liquid level (m);
- `A` is tank cross-sectional area (m^2);
- `q_in` is pump inflow (m^3/s);
- `q_out` is outlet flow (m^3/s);
- `C_out` is the outlet coefficient (m^2.5/s).

The square-root relationship approximates gravity-driven discharge through
an outlet. It is a simplified relationship and assumes the outlet geometry
and discharge characteristics are represented by a constant coefficient.

## 4. Control strategy

The controller uses a discrete PID algorithm:

`u = u_bias + Kp*e + Ki*integral(e dt) - Kd*(dh/dt)`

The derivative term is calculated from the measured level rather than the
setpoint. This avoids a derivative kick when the setpoint changes.

The pump command is constrained between configured minimum and maximum
flow rates. Conditional-integration anti-windup prevents the integral term
from accumulating when the actuator is saturated and the error would drive
it further into saturation.

## 5. Simulation scenario

- Initial tank level: 0.40 m
- Initial setpoint: 0.40 m
- Setpoint step: 0.80 m at 5 s
- Outlet-demand disturbance: outlet coefficient increases by 25% at 60 s
- Simulation duration: 120 s
- Numerical integration step: 0.01 s
- Controller sample period: 0.10 s
- Tank area: 1.0 m^2
- Pump flow limits: 0 to 0.025 m^3/s

The PID gains and model coefficients are illustrative starting values, not
a tuning recommendation for a real plant. Their suitability should be
assessed using response metrics and further tuning.

## 6. Build and run

Requires a C++17-compatible compiler (GCC or Clang).

```bash
g++ -std=c++17 -O2 -Wall -Wextra -pedantic tank_level_pid.cpp -o tank_level_pid
./tank_level_pid
```

On Windows with MinGW:

```powershell
g++ -std=c++17 -O2 -Wall -Wextra -pedantic tank_level_pid.cpp -o tank_level_pid.exe
.\tank_level_pid.exe
```

## 7. Output data

The program creates `tank_level_response.csv`, containing:
- time;
- setpoint and measured tank level;
- tracking error;
- pump inflow;
- outlet flow; and
- disturbance multiplier.

Import the CSV into Python, MATLAB, Excel, or another plotting tool to
visualise setpoint tracking, actuator effort and disturbance rejection.

## 8. How to evaluate the controller

Do not judge a controller only by whether the final value reaches the
setpoint. Analyse:
- rise time after the setpoint step;
- overshoot and undershoot;
- settling time within a stated tolerance band;
- steady-state error;
- recovery time after the outlet disturbance;
- time spent at pump-flow saturation; and
- sensitivity to changes in PID gains and model parameters.

For reproducible comparisons, state the tolerance band and measurement
window used for each metric.

## 9. Verification and further testing

Suggested next validation steps:
1. Check the mass balance numerically at selected time points.
2. Run with the outlet disturbance disabled and compare responses.
3. Repeat with different integration steps to check numerical convergence.
4. Test proportional-only, PI and PID configurations.
5. Vary pump limits and disturbance magnitude.
6. Compare against an independently implemented model or measured process
   data before making claims about physical accuracy.

## 10. Limitations

This is a lumped-parameter educational model. It does not include sensor
noise, actuator dynamics, transport delay, changing tank cross-section,
pump curves, cavitation, leakage, or detailed hydraulic network effects.
The PID gains have not been optimised for a particular physical installation.

## 11. Potential extensions

- Add low-pass filtering to the derivative term.
- Compare conditional integration with back-calculation anti-windup.
- Add sensor noise and actuator response lag.
- Implement gain tuning and parameter sweeps.
- Calculate performance metrics automatically.
- Produce plots and automated regression tests.
- Compare the model with a trusted process simulator or experimental data.

## 12. Reproducibility and integrity

The source code, equations, parameters, sample interval and output schema are
documented in this repository. The results are generated by the included
program. This is a simulated model, not evidence of commissioning or
validation on a physical water-treatment or industrial plant.
