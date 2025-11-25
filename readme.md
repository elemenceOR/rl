# Overhead Crane Control using Reinforcement Learning

This project implements a Reinforcement Learning (RL) solution to control an underactuated overhead crane system. Using Proximal Policy Optimization (PPO) and a custom Curriculum Learning strategy, the agent learns to transport a suspended load to a target position while actively damping sway and minimizing overshoot.

## Table of Contents

- [Project Overview](#project-overview)
- [System Dynamics and Environment](#system-dynamics-and-environment)
- [Development History and Iterative Process](#development-history-and-iterative-process)
- [Final Methodology](#final-methodology)
- [Project Structure](#project-structure)
- [Installation and Usage](#installation-and-usage)
- [Results](#results)

## Project Overview

The overhead crane is a classic non-linear control problem. The system consists of a cart moving along a 1-dimensional track with a load suspended by a cable. The control objective is to apply horizontal force to the cart to reach a specific coordinate.

The challenge lies in the system being underactuated: there are two degrees of freedom (cart position x and sway angle θ) but only one control input (force F). The agent must learn to exploit the system's coupling effects to dampen the load's oscillation while simultaneously traversing to the target.

## System Dynamics and Environment

### Custom Gymnasium Environment: OverheadCrane-v0

The physics simulation relies on the coupled non-linear equations of motion derived via Lagrangian mechanics:

```
ẍ = (F + m_L * L * θ̇² * sin(θ) + m_L * g * sin(θ) * cos(θ)) / (M + m_L * sin²(θ))

θ̈ = (-F * cos(θ) - m_L * L * θ̇² * sin(θ) * cos(θ) - (M + m_L) * g * sin(θ)) / (L * (M + m_L * sin²(θ)))
```

### State Space (Continuous)

- x: Cart position (m)
- ẋ: Cart velocity (m/s)
- θ: Sway angle (rad)
- θ̇: Angular velocity (rad/s)
- x_target: Target position (m)

### Action Space (Continuous)

- Force applied to the cart: F ∈ [-50.0, 50.0] N

### Physical Parameters

- Gravity (g): 9.81 m/s²
- Cart mass (m_c): 10.0 kg
- Load mass (m_l): 2.0 kg
- Cable length (l): 1.5 m
- Simulation frequency: 50 Hz (Δt = 0.02s)

## Development History and Iterative Process

This project evolved through several distinct versions, identifying and resolving specific control behaviors.

### Version 1.0: The "Suicide" Agent

- **Approach:** Standard negative reward for distance error and sway angle.
- **Result:** The agent immediately terminated the episode (drove off-track) at step 1.
- **Analysis:** The cumulative penalty of existing for the full duration (-3000) was mathematically worse than the fixed penalty for failure (-100). The optimal policy was to terminate immediately to maximize the score.
- **Fix:** Normalized rewards to positive values (0.0 to 1.0). "Survival" became profitable.

### Version 2.0: The "Drifting" Agent

- **Approach:** High penalty for sway, low reward for distance. Reduced max force to 50N for realism.
- **Result:** The agent accelerated slowly and refused to brake near the target, coasting past it.
- **Analysis:** The agent was "risk-averse." Braking induces sway (penalty). The agent determined that the penalty for braking/swaying outweighed the marginal reward gain of stopping precisely at the target.

### Version 3.0: The "Aggressive" Agent

- **Approach:** Drastically increased the weight of the Distance Reward to motivate movement.
- **Result:** High-speed travel (1.6 m/s) with massive overshoot (3.0m) and significant ringing/oscillation.
- **Analysis:** The agent learned to "sprint" to maximize the distance score early in the episode, ignoring the velocity costs. It lacked the predictive capacity to brake in time given the force constraints.

### Version 4.0: Automated Hyperparameter Tuning (Failure)

- **Approach:** Used Optuna (RL Zoo) to find optimal reward weights automatically.
- **Result:** The optimizer converged on a policy that refused to move (x=0).
- **Analysis:** Reward Hacking. The optimizer found that maximizing the Stability Reward (weight 50.0) while ignoring the Distance Reward yielded a higher total score than risking movement.

### Version 5.0: The "Precision" Agent (Final Solution)

- **Approach:** Inverted priority logic and Curriculum Learning.
- **Logic Change:** Instead of rewarding speed, we heavily penalized velocity and angle.
  - Stability Priority: High
  - Velocity Cost: High
  - Distance Priority: Medium
- **Fix:** We removed the "Effort Penalty" (F²) entirely, allowing the agent to use full braking power when necessary.
- **Result:** The agent learned to accelerate gently, utilize "back-kick" control to pre-sway the load, and brake aggressively to achieve zero steady-state error with < 0.25m overshoot.

## Final Methodology

### Curriculum Learning Strategy

To overcome the sparse reward problem (learning to brake before learning to travel), training is divided into three phases:

1. **Phase 1: Stabilization (500k Steps):** Target is fixed at 0.0m. Agent learns to balance the inverted pendulum dynamics.
2. **Phase 2: Short Transit (1M Steps):** Target is fixed at 2.0m. Agent learns acceleration and braking over short distances without reaching high velocities.
3. **Phase 3: Full Transit (2M Steps):** Target is fixed at 5.0m. Agent applies learned braking skills to high-momentum scenarios.

### Reward Function Formulation

The final reward function prioritizes stability over location, acting as a "speed governor" that prevents the agent from entering unrecoverable states.

```
R_total = (15.0 * R_dist) + (20.0 * R_angle) + (4.0 * R_vel) + (2.0 * R_ang_vel)
```

Where components R are normalized [0, 1] values.

- **Angle Weight (20.0):** Highest priority. Any sway is penalized heavily.
- **Distance Weight (15.0):** Strong magnetic pull to the target to ensure zero Steady-State Error (SSE).
- **Velocity Weight (4.0):** Caps the maximum speed, preventing the "coasting" issue seen in V2.0.

## Installation and Usage

### Prerequisites

- Python 3.8+
- Gymnasium
- Stable-Baselines3
- Pygame (for rendering)
- Numpy

## Results

The final agent (Version 5.0) demonstrates behavior consistent with expert human operators, including non-minimum phase control input (counter-steering to initiate movement).

### Performance Metrics (Step Response to 5.0m)

- **Rise Time:** ~7 seconds
- **Settling Time:** ~25 seconds
- **Steady State Error:** 0.00 meters
- **Max Overshoot:** 1.2 meters (acceptable trade-off for rapid transit)
- **Residual Sway:** < 0.1 degrees

The controller successfully generalizes to dynamic target tracking, capable of reacting to setpoint changes during operation without inducing unstable oscillation.
