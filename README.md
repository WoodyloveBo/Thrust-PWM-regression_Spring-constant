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

### Thrust–PWM Regression

#### Overview
This section describes the procedure used to identify the PWM–thrust relationship of a Crazyflie by estimating thrust from onboard signals and fitting a quadratic regression model. The thrust is indirectly computed using the spring–payload model and body-frame inertial forces, without using a force sensor.

#### Procedure
1. Suspend a payload (10–20 g) from one Crazyflie using a spring and perform hovering flight.
2. Record all ROS topics during flight, including Crazyflie position, payload position, PWM, and body-frame inertial terms:
   - `rosbag record -a`
3. Process the recorded data to compute PWM and thrust using the following pipeline:
   - `point_index.py -> timestamp.py -> thrust.py -> average.py`
4. Perform quadratic regression between PWM and thrust using:
   - `quadratic_regression.py`

#### Processing Pipeline

##### `point_index.py`
- Identifies and separates the payload as a point mass from the motion-capture data.

##### `timestamp.py`
- Resets ROS timestamps so that time starts from 0 seconds.

##### `thrust.py`
- Automatically processes all `10g_*_timestamp.bag` files in the current directory.
- Uses the following topics:
  - Crazyflie log topic (`/cf2/log1`):  
    - `values[0]`: PWM  
    - `values[1–3]`: body-frame inertial force term  
      \[
      \mathbf{v}_b = m\mathbf{a}^B + R_W^B m g \mathbf{e}_3
      \]
  - Crazyflie pose (`/natnet_ros/cf2/pose`)
  - Payload position (`/point_index4`)
- Computes:
  - Spring force \( \mathbf{F}_s^B \) using the spring model  
    \[
    F_s = k (L - L_0) + b
    \]
  - Thrust in the body frame:
    \[
    \mathbf{F}_t^B = \mathbf{v}_b + \mathbf{F}_s^B
    \]
- Converts forces to gf units and saves the results to a new bag file:
  - `<original>_ftfs.bag`

- Generated topics:
  - `/ft/vec_gf` (`geometry_msgs/Vector3Stamped`): thrust vector \( \mathbf{F}_t^B \) [gf]
  - `/fs/vec_gf` (`geometry_msgs/Vector3Stamped`): spring force vector \( \mathbf{F}_s^B \) [gf]
  - `/ft/z_gf` (`std_msgs/Float32`): vertical thrust component [gf]
  - `/pwm` (`std_msgs/Float32`): PWM command

##### `average.py`
- Reads `<original>_ftfs.bag` files.
- Computes the average values of:
  - vertical thrust (`/ft/z_gf`)
  - PWM (`/pwm`)
- Averages are computed over a specified steady-state time window (e.g., 10–15 s).
- Outputs mean thrust and PWM values for each bag file, which are later stored in a CSV file.

#### Thrust–PWM Regression

##### `quadratic_regression.py`
- Loads a CSV file (`PWM-Thrust-alpha.csv`) containing averaged PWM and thrust data.
- Sorts data by PWM and performs quadratic regression:
  \[
  \text{Thrust} = a \cdot \text{PWM}^2 + b \cdot \text{PWM} + c
  \]
- Outputs:
  - regression coefficients \(a, b, c\)
  - coefficient of determination \(R^2\)
- Visualizes:
  - measured PWM–thrust data points
  - fitted quadratic regression curve
