# Agent Guide: unh_marine_simulation

> ROS 2 packages for simulating marine autonomous surface vehicles (ASVs), including dynamics, sensors, helm controllers, and Gazebo world generation.

## Package Inventory

| Package | Language | Description |
|---------|----------|-------------|
| `asv_sim` | Python | ASV dynamics simulator with configurable platform models and environment |
| `asv_sim_msgs` | C++ (msg gen) | Service definitions for the ASV simulator (`SetPose`) |
| `asv_helm` | C++ | Converts Twist/Helm commands to throttle and rudder outputs |
| `vrx_project11` | C++ | WAM-V differential-thrust helm with nav2 integration |
| `mbes_sim` | Python | Multibeam echosounder simulator using bathymetry grids |
| `marine_charts_to_gazebo_world` | Python | Generates Gazebo Harmonic SDF worlds from S57 ENC chart data |
| `portsmouth_nh_gazebo` | Python | Pre-configured Gazebo world for Portsmouth NH harbor |
| `marine_simulation` | CMake (config only) | Aggregation package with scenario launch files and configs |

## Repository Layout

```
unh_marine_simulation/
├── asv_sim/                           # ASV dynamics simulator
│   ├── asv_sim/                       #   Python module (dynamics, environment, model, platform)
│   ├── config/                        #   Platform configs (ben.yaml, cw4.yaml, drix.yaml, ...)
│   └── test/                          #   Linting tests
├── asv_sim_msgs/                      # Service definitions
│   └── srv/SetPose.srv
├── asv_helm/                          # Helm controller (Twist → throttle/rudder)
│   └── src/asv_helm.cpp
├── vrx_project11/                     # WAM-V platform integration
│   ├── src/wamv_helm.cpp
│   ├── launch/                        #   7 launch files (sim, nav, operator)
│   ├── config/                        #   nav2_params.yaml, wamv.yaml
│   └── urdf/                          #   WAM-V URDF model
├── mbes_sim/                          # Multibeam sonar simulator
│   ├── mbes_sim/mbes_sim.py
│   └── data/US5NH02M.tiff            #   Bathymetry grid (69 MB)
├── marine_charts_to_gazebo_world/     # Chart-to-Gazebo converter
│   └── marine_charts_to_gazebo_world/ #   generate_world, s57_reader, terrain, heightmap, ...
├── portsmouth_nh_gazebo/              # Portsmouth NH Gazebo world
│   ├── config/portsmouth.yaml
│   └── launch/gazebo_launch.py
└── marine_simulation/                 # Scenario launch files
    ├── launch/                        #   sim_test, sim_demo, sim_drix, ...
    └── config/                        #   bob_platform.yaml, udp_bridge.yaml
```

## Architecture Overview

The repo provides a two-level simulation architecture:

1. **Physics simulation** (`asv_sim`) — Dynamics engine that models thrust, drag,
   rudder response, and environmental forces (current, wind). Each platform is
   configured via YAML files that define propulsion type (jet/prop), mass, drag
   coefficients, and maximum RPM.

2. **Sensor simulation** (`mbes_sim`) — Multibeam echosounder that generates sonar
   detections from a bathymetry grid, publishing `SonarDetections` and `PointCloud2`.

3. **3D visualization** (`marine_charts_to_gazebo_world`, `portsmouth_nh_gazebo`) —
   Generates Gazebo Harmonic worlds from S57 ENC chart data and ETOPO bathymetry.

**Helm controllers** (`asv_helm`, `vrx_project11/wamv_helm`) convert high-level
velocity commands (`cmd_vel`) into platform-specific actuator outputs (throttle/rudder
or differential thrust).

**Scenario launch files** in `marine_simulation` compose these components into
complete simulation setups with platform-specific configs from external repos
(`ben_project11`, `drix_project11`).

Data flow: `cmd_vel` → helm → `throttle`/`rudder` → `asv_sim` → position/orientation → `mbes_sim` → sonar data

## Key Files to Read First

1. `asv_sim/asv_sim/asv_sim_node.py` — Main simulator node; creates platforms from config
2. `asv_sim/config/ben.yaml` — Example platform configuration showing all parameters
3. `asv_helm/src/asv_helm.cpp` — Helm controller showing Twist-to-actuator conversion
4. `asv_sim_msgs/srv/SetPose.srv` — Service interface for resetting platform position
5. `marine_simulation/launch/sim_demo.launch` — Example scenario launch combining all components
6. `mbes_sim/mbes_sim/mbes_sim.py` — Sonar simulator showing lifecycle node pattern

## Build & Test

```bash
# From the layer workspace directory (layers/main/simulation_ws/)
colcon build --packages-select asv_sim asv_sim_msgs asv_helm vrx_project11 mbes_sim marine_charts_to_gazebo_world portsmouth_nh_gazebo marine_simulation

# Testing requires setup.bash in the same shell
source ../../../.agent/scripts/setup.bash && colcon test --packages-select asv_sim && colcon test-result --verbose
```

Known build issues:
- `mbes_sim` includes a 69 MB bathymetry TIFF (`data/US5NH02M.tiff`); initial clone is large
- `marine_charts_to_gazebo_world` requires GDAL (`python3-gdal`) and may fail to build if GDAL headers are missing
- `portsmouth_nh_gazebo` downloads ETOPO data on first launch and caches it in `~/.cache/portsmouth_nh_gazebo/`
- `vrx_project11` depends on `marine_interfaces` and `marine_autonomy` from the core layer

## Cross-Layer Dependencies

| Package | Depends On | Layer | What It Imports |
|---------|-----------|-------|-----------------|
| `asv_helm` | `marine_interfaces` | core | `marine_interfaces/msg/Helm`, `marine_interfaces/msg/Heartbeat` |
| `asv_helm` | `marine_autonomy` | core | Runtime dependency |
| `vrx_project11` | `marine_interfaces` | core | `marine_interfaces/msg/Heartbeat` |
| `vrx_project11` | `marine_autonomy` | core | Runtime dependency |
| `mbes_sim` | `marine_acoustic_msgs` | underlay | `marine_acoustic_msgs/msg/SonarDetections` |
| `mbes_sim` | `marine_autonomy` | core | `marine_autonomy.robot_navigation` |
| `marine_simulation` | `cube_bathymetry` | sensors | Runtime (rviz plugin) |

## Common Pitfalls

- **Default branch is `jazzy`**, not `main`. Always verify with `gh repo view --json defaultBranchRef`.
- **Platform configs are name-sensitive**: `asv_sim` reads a `platforms` parameter (list of names) and looks for matching YAML files. A typo in the platform name causes a silent failure.
- **Lifecycle nodes**: `mbes_sim` uses ROS 2 lifecycle — it must be configured and activated before it publishes data. Launch files handle this, but manual testing requires explicit state transitions.
- **Coordinate frame**: `asv_sim` publishes in geographic coordinates (lat/lon via `NavSatFix`), not in a local frame. Consumers must handle the geographic-to-local transform.
- **S57 ENC data**: `marine_charts_to_gazebo_world` and `portsmouth_nh_gazebo` require S57 ENC chart data. Set `ROS_S57_ENC_ROOT` environment variable to the ENC data directory, or the world generation will produce a flat terrain.
