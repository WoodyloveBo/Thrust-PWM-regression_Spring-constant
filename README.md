### Spring constant

#### Overview
This section describes how the spring constant was identified using a single Crazyflie hover experiment with a spring-suspended payload (10–20 g). The spring displacement is computed from recorded position data, and the spring constant is estimated using Hooke’s law.

#### Procedure
1. Suspend a payload (10–20 g) from one Crazyflie using a spring, and perform hovering.
2. Record ROS topics (Crazyflie position, payload position) during flight:
   - `rosbag record -a`
3. Measure spring displacement using the post-processing pipeline:
   - `point_index.py -> timestamp.py -> distance.py`
4. Compute the spring displacement:
   - `x = l - l0`
   - where `l0` is the natural spring length and `l` is the measured distance.
5. Save the spring length/displacement data for 10–20 g payloads to:
   - `spring_distance.csv`
6. Run `spring_constant.py` to fit Hooke’s law in the form:
   - `F = kx + b`
   - and obtain the spring constant `k` and offset term `b`.

#### Scripts
- `point_index.py`
  - Identifies the payload as a point mass (used to separate the payload tracking point).

- `timestamp.py`
  - Converts ROS timestamps so that the initial time starts from 0 seconds.

- `distance.py`
  - Computes the actual distance `l` between the Crazyflie position and the payload position.
  - Exports the computed distance (spring length) to a CSV file.

- `spring_constant.py`
  - Estimates `k` and `b` using the model `F = kx + b`.
  - Plots the fitted results.
  - Computes the correlation coefficient `R`.



### Thrust-PWM Regression

##
