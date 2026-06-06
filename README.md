# Boeing 747 Lateral Flight Control Analysis

A MATLAB control-systems project analyzing the lateral-directional dynamics of a Boeing 747 and designing automatic flight-control strategies for improved stability.

This project models the aircraft in state-space form, converts the system to transfer-function and zero-pole-gain representations, studies the major lateral flight modes, and evaluates classical control approaches such as proportional feedback, washout filtering, actuator dynamics, root-locus design, and pole-placement methods.

## Project Overview

The objective is to study the lateral control behavior of a Boeing 747 using a linearized dynamic model. The system is represented with the state vector:

```text
x = [ beta, r, p, phi ]'
```

where:

- `beta` is the sideslip angle
- `r` is the yaw rate
- `p` is the roll rate
- `phi` is the roll angle

The project focuses on understanding how the aircraft responds to lateral disturbances and how feedback control can improve the dynamic behavior of the system.

## What This Project Includes

- State-space modeling of Boeing 747 lateral dynamics
- Transfer-function generation from state-space matrices
- Zero, pole, and gain analysis
- Identification of key flight modes:
  - Dutch roll mode
  - Roll mode
  - Spiral mode
- Damping ratio and natural frequency analysis
- Bode diagram analysis
- Static gain estimation
- Initial-condition response simulation
- Root-locus study for classical controller design
- Closed-loop feedback analysis
- Washout filter and actuator modeling
- Pole-placement control exploration

## System Model

The aircraft is modeled using the following state-space representation:

```matlab
A = [ -.0558    -.9968   .0802   .0415
       .598     -.115   -.0318      0
      -3.05      .388   -0.4650     0
        0        .0805    1         0 ];

B = [ .00729
     -.475
       .153
        0 ];

C = [ 0  1  0  0 ];

D = 0;
```

The MATLAB command `ss(A,B,C,D)` is used to build the state-space model, followed by transfer-function and zero-pole-gain conversion.

## Key Results

The open-loop system analysis identifies the following poles:

```text
-0.0329 +/- 0.9467i   Dutch roll mode
-0.5627               Roll mode
-0.0073               Spiral mode
```

The transfer-function gain is approximately:

```text
Gain = -0.4750
```

The Dutch roll mode has a low damping ratio, approximately:

```text
Damping ratio = 0.0348
```

This indicates that the Dutch roll mode is lightly damped and requires control action to improve stability and handling quality.

## Control Design Approach

The project explores several control-design steps:

1. Classical proportional feedback using root-locus analysis
2. Closed-loop transfer-function evaluation
3. Frequency-response verification using Bode plots
4. Washout filter design:

```matlab
tau = 3;
washout_s = tau*s/(1 + tau*s);
```

5. Actuator/gouverne dynamics:

```matlab
gouverne_s = 10/(s + 10);
```

6. Pole-placement formulation using an extended system representation

These methods are used to study how feedback gain, filtering, and actuator dynamics influence aircraft stability.

## MATLAB Tools Used

- `ss`
- `tf`
- `zpk`
- `ss2tf`
- `tf2zp`
- `damp`
- `bode`
- `initial`
- `rlocus`
- `sgrid`
- `feedback`
- Symbolic variables with `syms`

## Skills Demonstrated

- Flight dynamics modeling
- Linear systems analysis
- MATLAB programming
- Control-system design
- State-space representation
- Transfer-function analysis
- Frequency-domain analysis
- Root-locus controller tuning
- Stability and damping evaluation
- Aerospace control engineering fundamentals

## Academic Context

This project was completed as part of coursework in complex systems and aerospace control engineering, with a focus on automatic flight control for large aircraft lateral dynamics.

## Why It Matters

Aircraft lateral-directional stability is essential for safe and comfortable flight. Modes such as Dutch roll can create oscillatory motion if insufficiently damped. By modeling the aircraft and designing feedback control laws, engineers can improve stability, reduce oscillations, and support reliable automatic flight-control behavior.

## Repository Notes

The MATLAB script performs the full analysis from model declaration through controller exploration. To run the project, open the script in MATLAB with the Control System Toolbox installed and execute the sections in order.
