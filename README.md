# Crazy Controller Package

Production-ready MAP (Model-based Adaptive Pursuit) controller for F1TENTH autonomous racing with advanced safety features, dynamic parameter tuning, and sophisticated trajectory following.

## Quick Start

```bash
# Build the package
cd ~/Programming/ae_hyu_bundle
colcon build --packages-select crazy_controller
source install/setup.bash

# Run in simulation mode
ros2 launch crazy_controller controller_launch.py sim_mode:=true

# Run on real car
ros2 launch crazy_controller controller_launch.py sim_mode:=false
```

## Features

- Advanced safety systems (graceful shutdown, emergency stop)
- Environment-aware configuration (simulation/real car auto-detection)
- Dynamic parameter tuning (real-time ROS2 parameter updates)
- Frenet coordinate processing (external frenet odometry support)
- Flexible topic remapping (launch-file managed)
- Production-ready architecture (thread-safe, robust error handling)

## Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sim_mode` | `false` | Automatically configures topics for simulation vs real car |
| `mode` | `MAP` | Control mode (MAP controller only) |
| `l1_params_path` | Auto-selected | `l1_params_sim.yaml` (sim) or `l1_params.yaml` (real) |
| `lookup_table_path` | Auto-selected | `SIM_linear_lookup_table.csv` (sim) or `RBC1_pacejka_lookup_table.csv` (real) |

## Topic Configuration

### Subscribed Topics

| Generic Topic | Simulation Topic | Real Car Topic | Purpose |
|---------------|------------------|----------------|---------|
| `/global_waypoints` | `/global_waypoints` | `/global_waypoints` | Track length information |
| `/local_waypoints` | `/local_waypoints` | `/local_waypoints` | Local trajectory waypoints |
| `/odom` | `/ego_racecar/odom` | `/pf/pose/odom` | Vehicle pose and velocity |
| `/frenet/odom` | `/car_state/frenet/odom` | `/car_state/frenet/odom` | Frenet coordinate state |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | Vehicle control commands |

## Configuration

### L1 Controller Parameters

**Real Car (`config/l1_params.yaml`):**
```yaml
crazy_controller:
  ros__parameters:
    # L1 Pure Pursuit Parameters
    t_clip_min: 1.5               # Minimum time horizon
    t_clip_max: 5.0               # Maximum time horizon
    m_l1: 0.3                     # L1 gain multiplier
    q_l1: 0.5                     # L1 damping coefficient
    
    # Speed Control
    speed_lookahead: 0.25         # Speed-based lookahead distance
    lat_err_coeff: 1.0            # Lateral error influence
    
    # Steering Adaptation
    acc_scaler_for_steer: 1.2     # Acceleration-based steering scaling
    dec_scaler_for_steer: 0.9     # Deceleration-based steering scaling
    start_scale_speed: 7.0        # Speed scaling start threshold
    end_scale_speed: 8.0          # Speed scaling end threshold
    downscale_factor: 0.2         # Speed reduction factor
    
    # Advanced Tuning
    speed_lookahead_for_steer: 0.0 # Speed-dependent steering lookahead
```

**Simulation (`config/l1_params_sim.yaml`):**
```yaml
crazy_controller:
  ros__parameters:
    t_clip_min: 1                 # Faster response for simulation
    m_l1: 0.3                     # Optimized for sim dynamics
    q_l1: 0.15                    # Different damping for sim
    # ... other parameters
```

## Control Algorithm

The MAP controller implements a multi-stage control pipeline:

1. **Message Synchronization**: Waits for track length, waypoints, and vehicle state with 5-second timeout protection
2. **Frenet Coordinate Processing**: Receives pre-calculated frenet coordinates with lateral error validation
3. **L1 Adaptive Guidance**: Dynamic lookahead distance calculation based on speed and curvature
4. **Speed Planning**: Curvature-aware velocity adaptation with acceleration/deceleration scaling
5. **Steering Control**: Pacejka tire model lookup with environment-specific calibration

## Safety Features

- **Signal Handling**: Graceful shutdown on SIGINT/SIGTERM
- **Emergency Stop**: Multiple stop commands on termination
- **Thread Safety**: Atomic operations for shutdown coordination
- **Waypoint Timeout**: Automatic stop if no fresh waypoints (5 seconds)
- **Input Validation**: Comprehensive message and parameter validation
- **Error Recovery**: Robust error handling with detailed logging

## Runtime Operation

### Message Flow
```
/global_waypoints → track_length_cb() → track_length_
/local_waypoints  → local_waypoint_cb() → waypoint_array_in_map_
/odom            → car_state_cb() → position_in_map_, speed_now_
/frenet/odom     → car_state_frenet_cb() → position_in_map_frenet_
                                    ↓
                              control_loop() (40Hz)
                                    ↓
                              map_cycle() → MAP_Controller
                                    ↓
                              /drive commands
```

### Dynamic Parameter Updates
```bash
# Real-time parameter tuning
ros2 param set /controller_manager m_l1 0.4
ros2 param set /controller_manager speed_lookahead 0.3
ros2 param set /controller_manager acc_scaler_for_steer 1.1
```

## Troubleshooting

### Common Issues

**"Invalid l1_params YAML: missing crazy_controller.ros__parameters"**
- Check YAML namespace in config files
- Should be: `crazy_controller.ros__parameters` (not `controller.ros__parameters`)

**"Controller waiting for messages..."**
```bash
# Check required topics are publishing:
ros2 topic list | grep -E "(global_waypoints|local_waypoints|odom|frenet)"
ros2 topic echo /global_waypoints
ros2 topic echo /local_waypoints
```

**"No fresh local waypoints. STOPPING!!"**
```bash
# Check planner frequency:
ros2 topic hz /local_waypoints     # Should be >1Hz
ros2 topic info /local_waypoints   # Check publishers
```

**Vehicle not moving**
1. Check odometry: `ros2 topic echo /ego_racecar/odom` (sim) or `/pf/pose/odom` (real)
2. Verify frenet odometry: `ros2 topic echo /car_state/frenet/odom`
3. Confirm drive commands: `ros2 topic echo /drive`
4. Check steering lookup table file exists

### Performance Tuning

**More Aggressive Racing:**
```yaml
speed_lookahead_for_steer: 0.175  # Increase up to 0.20 MAX
acc_scaler_for_steer: 1.3         # Higher cornering aggression
m_l1: 0.4                         # Faster L1 response
```

**More Conservative/Stable:**
```yaml
speed_lookahead_for_steer: 0.0    # Most conservative
t_clip_min: 2.0                   # Longer planning horizon
lat_err_coeff: 0.8                # Reduce lateral error sensitivity
```

## Dependencies

- ROS2 Humble (or compatible)
- ae_hyu_msgs (custom waypoint messages)
- ackermann_msgs (drive commands)
- nav_msgs (odometry)
- Eigen3 (linear algebra)
- yaml-cpp (configuration parsing)

## Architecture

- Thread-safe design with atomic operations
- Environment agnostic (single codebase for sim and real car)
- Parameter validation with runtime range checking
- Comprehensive logging and error handling
- Memory efficient with smart pointers and move semantics