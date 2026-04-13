estimationControlBlimp

This repository contains MATLAB scripts and functions for control and analysis related to a blimp vehicle. The code is organized as standalone .m files that operate on a vehicle state struct (b1) and a time struct (time, using time.dt).

Project context

As part of an advanced robotics project, the developer reports a fully autonomous control and estimation system for a blimp operating in a constrained indoor environment. The system used motion capture data fused through a Kalman filter to estimate the blimp's state in real-time, enabling closed-loop control. The reported capabilities include obstacle avoidance, trajectory tracking, target chasing with predictive modeling, and chaser-evader protocols. The control architecture is reported to include modular PID loops for altitude, speed, and yaw regulation, integrated with a high-level mission planner for waypoint navigation. The implementation used MATLAB and Simulink and integrated motion capture measurements.

Files

- vhc_param.m
  - Initializes the vehicle parameter struct b1 with fields used across other scripts (IDs, PWM channels, PID gain placeholders, heading and waypoint parameters, pos_old, forward_spd, etc.).

- blimp_control_heading.m
  - Wrapper for heading control logic. Computes current speed and calls dynamic_heading; triggers dynamic_vertical when altitude error is larger than a threshold.

- blimp_control_spd.m
  - Wrapper for speed control logic. Normalizes heading vectors, computes heading error and calls dynamic_spd. Also calls dynamic_vertical when altitude error is larger than a threshold. Contains commented-out PWM limit code.

- blimp_control_vertical.m
  - Wrapper for vertical (altitude) control. Calls dynamic_vertical when altitude error is larger than a threshold. Computes heading error but heading-control calls are commented out.

- blimp_control_wp.m
  - Waypoint-following logic. Computes desired heading toward the current waypoint, increments the waypoint index when within the configured distance margin (b1.dist_mar), switches between heading and speed controllers, and calls dynamic_vertical when altitude is off target. Sets a reduced forward speed when close to a waypoint.

- dynamic_heading.m
  - PID controller implementation for heading. Computes angle error between current heading and desired heading, uses a signed-angle difference to decide turning direction, sets PWML/PWMR, enforces a global PWM sum limit (<= 450) and per-channel limits, and prints PWMR/PWML for debugging.

- dynamic_spd.m
  - PID controller implementation for forward speed. Computes speed along the heading vector using position differences and time.dt, computes PID output to set PWML/PWMR and enforces PWM limits. Updates pos_old and PID state variables and prints debug values.

- dynamic_vertical.m
  - PID controller implementation for vertical (altitude) control. Uses b1.pos(3) and an internal desired altitude (alt_des = 1200) to compute PWMV, enforces PWM limits, updates PID state and pos_old, and prints PWMV.

- velocityplot.m
  - Utility script that reads a CSV file named 'data_20241105_1350.csv' (expected to have a header row and columns: time, x, y in that order), removes rows with NaN positions, computes x/y velocities and speed magnitude, applies a 2nd-order Butterworth filter, and plots velocity. The filename is hard-coded in the script.

Other

- .vs/
  - Visual Studio workspace files and indexes (IDE artifacts).

Notes

- The code operates on a struct named b1 and expects certain fields to exist (examples: pos, pos_old, heading_vec, heading_dir, PWMV, PWML, PWMR, forward_spd, WP, curWP, dist_mar, fwd_deg). Several scripts rely on a time struct with a dt field.
- The scripts use MATLAB built-in functions (readcell, filtfilt, butter, acosd, atan2, etc.), so they are intended to be run in a MATLAB environment.

No other files or documentation are present in the repository.