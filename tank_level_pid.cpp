/*
 * PID-Based Automatic Water Tank Level Control
 *
 * Nonlinear tank model:
 *   A * dh/dt = q_in - C_out * sqrt(h)
 *
 * The outlet coefficient increases at t = 60 s to represent a demand
 * disturbance (for example, a downstream valve opening).
 *
 * Controller:
 *   Discrete PID with derivative on measurement, output saturation,
 *   and conditional-integration anti-windup.
 *
 * Numerical integration:
 *   Classical fourth-order Runge-Kutta (RK4) for the tank dynamics.
 *
 * Build:
 *   g++ -std=c++17 -O2 -Wall -Wextra -pedantic tank_level_pid.cpp -o tank_level_pid
 *
 * Run:
 *   ./tank_level_pid
 *
 * Output:
 *   tank_level_response.csv
 *
 * Educational model; validate parameters against a real process before use.
 */

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

struct Parameters {
    double tank_area_m2 = 1.0;
    double outlet_coefficient_m2_5_per_s = 0.0158113883;
    double pump_min_flow_m3_s = 0.0;
    double pump_max_flow_m3_s = 0.025;

    double kp_m2_per_s = 0.05;
    double ki_m2_per_s2 = 0.002;
    double kd_m2 = 0.01;

    double controller_sample_s = 0.1;
    double integration_step_s = 0.01;
    double simulation_duration_s = 120.0;
};

struct State {
    double level_m;
};

struct ControllerState {
    double integral_error_m_s = 0.0;
    double previous_level_m = 0.0;
    double output_flow_m3_s = 0.0;
};

double setpoint(double time_s) {
    return time_s < 5.0 ? 0.40 : 0.80;
}

double disturbanceMultiplier(double time_s) {
    return time_s < 60.0 ? 1.0 : 1.25;
}

double outletFlow(double level_m, double time_s, const Parameters& p) {
    const double safe_level = std::max(0.0, level_m);
    return p.outlet_coefficient_m2_5_per_s
         * disturbanceMultiplier(time_s) * std::sqrt(safe_level);
}

double levelDerivative(
    double level_m,
    double inflow_m3_s,
    double time_s,
    const Parameters& p
) {
    return (inflow_m3_s - outletFlow(level_m, time_s, p))
         / p.tank_area_m2;
}

double rk4Step(
    double level_m,
    double inflow_m3_s,
    double time_s,
    double dt,
    const Parameters& p
) {
    const double k1 = levelDerivative(level_m, inflow_m3_s, time_s, p);
    const double k2 = levelDerivative(
        level_m + 0.5 * dt * k1, inflow_m3_s, time_s + 0.5 * dt, p
    );
    const double k3 = levelDerivative(
        level_m + 0.5 * dt * k2, inflow_m3_s, time_s + 0.5 * dt, p
    );
    const double k4 = levelDerivative(
        level_m + dt * k3, inflow_m3_s, time_s + dt, p
    );

    const double next_level = level_m
        + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

    if (!std::isfinite(next_level)) {
        throw std::runtime_error("Non-finite tank level; simulation unstable.");
    }

    return std::max(0.0, next_level);
}

double updateController(
    double time_s,
    double measured_level_m,
    const Parameters& p,
    ControllerState& controller
) {
    const double error_m = setpoint(time_s) - measured_level_m;
    const double derivative_measurement =
        (measured_level_m - controller.previous_level_m)
        / p.controller_sample_s;

    // Derivative is applied to the measured process variable to avoid
    // derivative kick when the setpoint changes.
    const double unsaturated =
        0.014
        + p.kp_m2_per_s * error_m
        + p.ki_m2_per_s2 * controller.integral_error_m_s
        - p.kd_m2 * derivative_measurement;

    const double saturated = std::clamp(
        unsaturated, p.pump_min_flow_m3_s, p.pump_max_flow_m3_s
    );

    // Conditional integration: stop integrating when saturation is active
    // and the error would push the actuator further into saturation.
    const bool drives_further_high =
        unsaturated > p.pump_max_flow_m3_s && error_m > 0.0;
    const bool drives_further_low =
        unsaturated < p.pump_min_flow_m3_s && error_m < 0.0;

    if (!drives_further_high && !drives_further_low) {
        controller.integral_error_m_s +=
            error_m * p.controller_sample_s;
    }

    controller.previous_level_m = measured_level_m;
    controller.output_flow_m3_s = saturated;
    return saturated;
}

int main() {
    try {
        const Parameters p{};
        const int steps = static_cast<int>(
            p.simulation_duration_s / p.integration_step_s
        );
        const int controller_stride = static_cast<int>(
            std::round(p.controller_sample_s / p.integration_step_s)
        );

        if (controller_stride <= 0 ||
            std::abs(controller_stride * p.integration_step_s
                     - p.controller_sample_s) > 1e-9) {
            throw std::runtime_error(
                "Controller sample time must be a multiple of integration step."
            );
        }

        State tank{0.40};
        ControllerState controller{};
        controller.previous_level_m = tank.level_m;

        double inflow_m3_s = 0.014;
        std::ofstream csv("tank_level_response.csv");
        if (!csv) {
            std::cerr << "Could not create tank_level_response.csv\n";
            return 1;
        }

        csv << "time_s,setpoint_m,level_m,error_m,inflow_m3_s,"
               "outflow_m3_s,disturbance_multiplier\n";
        csv << std::fixed << std::setprecision(8);

        for (int step = 0; step <= steps; ++step) {
            const double time_s = step * p.integration_step_s;

            if (step % controller_stride == 0) {
                inflow_m3_s = updateController(
                    time_s, tank.level_m, p, controller
                );
            }

            const double target = setpoint(time_s);
            const double outflow = outletFlow(tank.level_m, time_s, p);

            csv << time_s << ','
                << target << ','
                << tank.level_m << ','
                << target - tank.level_m << ','
                << inflow_m3_s << ','
                << outflow << ','
                << disturbanceMultiplier(time_s) << '\n';

            if (step < steps) {
                tank.level_m = rk4Step(
                    tank.level_m,
                    inflow_m3_s,
                    time_s,
                    p.integration_step_s,
                    p
                );
            }
        }
        csv.close();

        const double final_error = setpoint(p.simulation_duration_s)
                                 - tank.level_m;

        std::cout << std::fixed << std::setprecision(5);
        std::cout << "Tank level PID simulation completed.\n";
        std::cout << "Output: tank_level_response.csv\n";
        std::cout << "Final level: " << tank.level_m << " m\n";
        std::cout << "Final setpoint: "
                  << setpoint(p.simulation_duration_s) << " m\n";
        std::cout << "Final tracking error: " << final_error << " m\n";
        std::cout << "Final pump flow: " << inflow_m3_s << " m^3/s\n";
        std::cout << "Final outlet flow: "
                  << outletFlow(tank.level_m, p.simulation_duration_s, p)
                  << " m^3/s\n";
        std::cout << "Disturbance multiplier: "
                  << disturbanceMultiplier(p.simulation_duration_s) << '\n';

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Simulation error: " << error.what() << '\n';
        return 1;
    }
}
