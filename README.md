# Crazy Controller

MAP (Model-based Adaptive Pursuit) controller for F1TENTH with safety features and dynamic parameter tuning.

## Quick Start

```bash
# Build
colcon build --packages-select crazy_controller
source install/setup.bash

# Real hardware
ros2 launch crazy_controller controller_launch.py

# Simulation
ros2 launch crazy_controller controller_launch.py sim_mode:=true
```

## Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sim_mode` | `false` | Use simulation topics if true |
| `mode` | `MAP` | Control algorithm (MAP only) |
| `l1_params_path` | Auto-selected | L1 parameter file by mode |
| `lookup_table_path` | Auto-selected | Steering lookup table by mode |

## Topics

### Real Hardware Mode
- **Input**: `/pf/pose/odom`, `/car_state/frenet/odom` - Pose and Frenet state
- **Input**: `/global_waypoints`, `/local_waypoints` - Track and path data
- **Output**: `/drive` - Ackermann drive commands

### Simulation Mode
- **Input**: `/ego_racecar/odom`, `/car_state/frenet/odom` - Simulation pose data
- **Input**: `/global_waypoints`, `/local_waypoints` - Track and path data
- **Output**: `/drive` - Ackermann drive commands

## Key Configuration

Edit `config/l1_params.yaml` (real) or `config/l1_params_sim.yaml` (simulation):

```yaml
crazy_controller:
  ros__parameters:
    # Core L1 Parameters
    t_clip_min: 1.5               # Minimum time horizon
    m_l1: 0.3                     # L1 gain multiplier
    q_l1: 0.5                     # L1 damping coefficient
    
    # Speed and Steering
    speed_lookahead: 0.25         # Speed-based lookahead
    acc_scaler_for_steer: 1.2     # Steering scaling factors
    
    # Vehicle parameters (auto-set by sim_mode)
    # Real: Pacejka tire model, wheelbase=0.325
    # Sim: Linear tire model, wheelbase=0.324
```

## Control Algorithm

MAP controller with multi-stage pipeline:
- **Message synchronization**: Waits for all required topics with timeout
- **L1 adaptive guidance**: Dynamic lookahead based on speed and curvature
- **Speed planning**: Curvature-aware velocity with acceleration scaling
- **Steering control**: Pacejka/linear tire model lookup with calibration

## Safety Features

- Signal handling with graceful shutdown
- Emergency stop on termination
- Waypoint timeout protection (5 seconds)
- Input validation and error recovery
- Thread-safe atomic operations

## Dynamic Tuning

```bash
# Real-time parameter updates
ros2 param set /controller_manager m_l1 0.4
ros2 param set /controller_manager speed_lookahead 0.3
ros2 param set /controller_manager acc_scaler_for_steer 1.1
```

## Prerequisites

- Lattice planner running
- Monte Carlo Localization (real hardware)
- Steering lookup table files loaded