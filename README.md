# Controller Package

Advanced MAP (Model-based Adaptive Pursuit) controller for F1TENTH autonomous racing with dynamic parameter tuning and sophisticated trajectory following.

## Quick Start

```bash
# Build
colcon build --packages-select crazy_controller
source install/setup.bash

# Real car (default)
ros2 launch crazy_controller controller_launch.py

# Simulation mode
ros2 launch crazy_controller controller_launch.py sim_mode:=true
```

## Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sim_mode` | `false` | Use simulation topics if true |
| `mode` | `MAP` | Control mode (MAP only) |
| `l1_params_path` | `config/l1_params.yaml` | L1 parameters file |
| `lookup_table_path` | `config/RBC1_pacejka_lookup_table.csv` | Steering lookup table |

## Topics

### Input
- `/planned_path` - Path from lattice planner (`ae_hyu_msgs/WpntArray`)
  - Real car: remapped to `/planned_waypoints`
  - Simulation: remapped to `/planned_waypoints`
- `/odom` - Vehicle pose and velocity
  - Real car: remapped to `/pf/pose/odom`
  - Simulation: remapped to `/ego_racecar/odom`

### Output
- `/drive` - Ackermann drive commands (`ackermann_msgs/AckermannDriveStamped`)

## Key Configuration

Edit `config/l1_params.yaml`:

```yaml
crazy_controller:
  ros__parameters:
    # L1 Controller Parameters
    t_clip_min: 1.0               # Minimum time horizon
    t_clip_max: 5.0               # Maximum time horizon
    m_l1: 0.5                     # L1 gain multiplier
    q_l1: -0.03                   # L1 damping coefficient

    # Lookahead Control
    speed_lookahead: 0.25         # Speed-based lookahead
    lat_err_coeff: 1.0            # Lateral error coefficient

    # Steering Scaling
    acc_scaler_for_steer: 1.2     # Acceleration steering scaler
    dec_scaler_for_steer: 0.9     # Deceleration steering scaler
    start_scale_speed: 7.0        # Speed scaling start threshold
    end_scale_speed: 8.0          # Speed scaling end threshold
    downscale_factor: 0.2         # Speed downscaling factor

    # Advanced Parameters
    speed_lookahead_for_steer: 0.0 # Speed-dependent steering lookahead
```

## Control Algorithm

The MAP controller implements a sophisticated trajectory following algorithm:

1. **Waypoint Processing**: Convert waypoint array to internal matrix format
2. **L1 Guidance**: Calculate lateral guidance using L1 adaptive control law
3. **Speed Planning**: Dynamic speed adaptation based on curvature and acceleration
4. **Steering Control**:
   - Speed-dependent steering scaling
   - Pacejka tire model lookup table
   - Rate limiting and safety constraints
5. **Safety Monitoring**:
   - Waypoint timeout detection (10 second threshold)
   - Emergency stop on signal (Ctrl+C)
   - Maximum steering angle limiting (±0.4 rad / ±23°)

## Advanced Features

- **Dynamic Parameters**: Real-time parameter tuning via ROS2 parameters
- **Graceful Shutdown**: Safe vehicle stop on termination signals
- **Frenet Coordinates**: Advanced trajectory representation
- **Speed Optimization**: Curvature-aware velocity planning
- **Safety Systems**: Multiple layers of safety monitoring

## Prerequisites

- Lattice planner running (`/planned_waypoints` topic)
- Localization running (odometry with pose and velocity)
- Vehicle hardware or F1TENTH simulator ready
- Proper calibration of steering lookup table

## Troubleshooting

**Vehicle not moving:**
- Check `/planned_waypoints` topic is publishing
- Verify odometry topic is available
- Confirm steering lookup table file exists

**Erratic steering:**
- Adjust L1 parameters (`m_l1`, `q_l1`)
- Check steering rate limits
- Verify lookup table calibration

**Performance tuning:**
- Increase `speed_lookahead` for smoother paths
- Adjust `acc_scaler_for_steer` for cornering aggressiveness
- Fine-tune `lat_err_coeff` for tracking accuracy
