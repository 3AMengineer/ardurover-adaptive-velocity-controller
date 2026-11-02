# ArduRover Adaptive Velocity Controller

**A ROS 2 control layer between Nav2 and ArduPilot that learns how the real rover responds and corrects its commands online.**

Nav2 can publish `0.5 m/s` all day, but on a real skid-steer rover with brushed motors that doesn't mean the rover moves at 0.5 m/s. Motor deadband, drivetrain friction, battery voltage and terrain all change the response, and the rover often turns harder one way than the other. This node closes that gap: it compares what Nav2 asked for with what the rover actually did, and adapts its gains while driving.

Built and field-tested on the UAS-DTU autonomous ground rover for the **DARPA Triage Challenge** (ArduRover on a Cube Orange+, Blickfeld 3D LiDAR, Nav2, MAVROS).

---

## Where it sits

```
Nav2 controller ──/cmd_vel──►  adaptive_velocity_controller  ──/mavros/setpoint_velocity/cmd_vel_unstamped──►  MAVROS ──► ArduRover
                                        ▲
                                        └── /mavros/local_position/odom  (measured velocity)
```

## How it works

Every 10 ms (100 Hz):

1. **Smooth the measurement.** Linear and angular velocity from odometry pass through an exponential moving average (α = 0.2) so wheel slip and sensor noise don't drive the adaptation.
2. **Apply the learned gains.** The commanded speed is `gain_v × v_desired`. Yaw rate uses **separate gains for left and right turns**, because a real drivetrain is rarely symmetric.
3. **Slew-rate limit** the output (0.05 per tick) so the rover never gets a step command, then clamp to the speed and yaw-rate limits.
4. **Adapt.** When both the desired and the measured velocity are outside the deadband, the gain is updated multiplicatively from the tracking ratio:

   ```
   r    = clip(|desired| / |measured|, 0.8, 1.2)
   gain = clip(gain × (1 + lr · (r − 1)), 0.2, 3.0)        lr = 0.02
   ```

   If the rover is slower than asked (r > 1), the gain grows; if it overshoots (r < 1), it shrinks. Clipping the ratio keeps any single noisy sample from yanking the gain.
5. **Reset on big setpoint changes.** If Nav2's desired speed jumps by more than 0.4 m/s, or the yaw rate by more than 0.2 rad/s, the learned gain, slew memory and filter state reset. A gain learned for slow cornering shouldn't be applied to a fast straight.

On the rover, the left- and right-turn gains settled at clearly different values during field runs, which is exactly the drivetrain asymmetry this design was built to absorb.

## Parameters

| Parameter | Value | Meaning |
|---|---|---|
| `initial_gain_v`, `initial_gain_left`, `initial_gain_right` | 0.75 | Starting gains (and the values restored on reset) |
| `lr` | 0.02 | Adaptation rate |
| `min_gain`, `max_gain` | 0.2, 3.0 | Gain limits |
| `dead_v`, `dead_w` | 0.04 m/s, 0.2 rad/s | No adaptation below these speeds |
| `dv_lim`, `dw_lim` | 0.05 per tick | Slew-rate limits |
| `max_v`, `max_w` | 2.0 m/s, 2.5 rad/s | Output limits |
| `reset_threshold_v`, `reset_threshold_w` | 0.4 m/s, 0.2 rad/s | Setpoint jump that triggers a gain reset |
| `alpha` | 0.2 | Odometry EMA smoothing factor |

## Topics

| Direction | Topic | Type |
|---|---|---|
| in | `/cmd_vel` | `geometry_msgs/Twist` from Nav2 |
| in | `/mavros/local_position/odom` | `nav_msgs/Odometry` (best-effort QoS) |
| out | `/mavros/setpoint_velocity/cmd_vel_unstamped` | `geometry_msgs/Twist`: forward speed in `linear.y`, yaw rate in `angular.z` (sign flipped to the rover's convention) |

The node logs `Vd / Vc / Va` (desired, commanded, actual) and the live gains `Gv / GL / GR` at 5 Hz, which makes tuning on the rover straightforward.

## Run it

Requires ROS 2 (tested on Humble), MAVROS connected to ArduRover in GUIDED mode, and NumPy.

```bash
source /opt/ros/humble/setup.bash
python3 adaptive_velocity_controller.py
```

Then remap Nav2's controller output to `/cmd_vel` (the default) and let the node drive the MAVROS velocity setpoint.

## Roadmap

- A `/cmd_vel` watchdog that zeroes the output if Nav2 stops publishing
- Sign-aware adaptation, so a rover rolling backwards is never read as "too slow"
- Consolidating the slew-rate stage and exposing the parameters as ROS 2 parameters
- Desired-vs-measured velocity plots from recorded rover bags

## Author

**Shourya Sharma**, Mathematics & Computing @ DTU, UAS-DTU
[LinkedIn](https://www.linkedin.com/in/shourya-sharma-6a365a341/) · [GitHub](https://github.com/3AMengineer)
