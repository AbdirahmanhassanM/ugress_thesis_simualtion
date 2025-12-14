# UGRESS Robot Simulation - Thesis Project

## Overview

This repository contains the Gazebo simulation implementation for validating the differential drive mobile robot with 2-DOF manipulator architecture proposed in the thesis "Design and Implementation of an Autonomous Ground-based Weed Elimination System."

## System Requirements

- **OS**: Ubuntu 20.04 LTS
- **ROS**: ROS Noetic
- **Gazebo**: Gazebo 11
- **Python**: Python 3.8+

## Dependencies
```bash
sudo apt-get update
sudo apt-get install ros-noetic-desktop-full
sudo apt-get install ros-noetic-gazebo-ros-pkgs
sudo apt-get install ros-noetic-gazebo-ros-control
sudo apt-get install python3-pip
```

## Installation

### 1. Clone Repository
```bash
cd ~/catkin_ws/src
git clone https://github.com/YOUR_USERNAME/ugress_thesis_simulation.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. Verify Installation
```bash
check_urdf ~/catkin_ws/src/ugress_thesis_simulation/ugress_sim/urdf/ugress.urdf
```
## Multiple Route Demonstrations 🎯

This simulation showcases **5 different autonomous route patterns**, demonstrating system versatility:

| Route | Pattern | Segments | Distance | Video |
|-------|---------|----------|----------|-------|
| **Agricultural** | 9-cycle rows | 9 | ~25m | ✅ simulation_run_5_working.mp4 |
| **Square** | 4-sided perimeter | 4 | 12m | ✅ route_square_demo.mp4 |
| **Zigzag** | Alternating diagonal | 6 | ~12m | ✅ route_zigzag_demo.mp4 |
| **Spiral** | Expanding outward | 8 | ~18m | ✅ route_spiral_demo.mp4 |
| **Circle** | 12-sided polygon | 12 | ~9.6m | ✅ route_circle_demo.mp4 |

### Quick Route Demo
```bash
# Launch Gazebo
roslaunch ugress_sim spawn_robot.launch

# Run any route (new terminal)
rosrun ugress_sim route_square.py   # Square pattern
rosrun ugress_sim route_zigzag.py   # Zigzag pattern
rosrun ugress_sim route_spiral.py   # Spiral pattern
rosrun ugress_sim route_circle.py   # Circle pattern
rosrun ugress_sim follow_path_final.py  # Original agricultural
```

All routes feature:
- ✅ 100% completion rate
- ✅ Automatic CSV data logging
- ✅ Real-time progress display
- ✅ Video demonstration available

See `ugress_sim/scripts/ROUTES_README.md` for detailed documentation.

## Usage

### Launch Simulation

**Terminal 1: Start Gazebo with Robot**
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch ugress_sim spawn_robot.launch
```

**Terminal 2: Run Path Following**
```bash
source ~/catkin_ws/devel/setup.bash
rosrun ugress_sim follow_path_simple.py
```

### Manual Control (Testing)
```bash
# Move forward
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5" -r 10

# Rotate
rostopic pub /cmd_vel geometry_msgs/Twist "angular:
  z: 0.5" -r 10
```

## Repository Structure
```
ugress_thesis_simulation/
├── README.md                          # Main documentation
├── LICENSE                            # MIT License
├── CITATION.cff                       # Citation information
├── CHANGELOG.md                       # Version history
├── FINAL_SUMMARY.md                   # Complete project summary
├── MEDIA_INVENTORY.md                 # Media catalog
│
├── ugress_sim/                        # ROS package
│   ├── CMakeLists.txt                 # Build configuration
│   ├── package.xml                    # Package metadata
│   │
│   ├── launch/
│   │   └── spawn_robot.launch         # Main launch file
│   │
│   ├── urdf/
│   │   └── ugress.urdf                # 4-wheel robot model
│   │
│   ├── scripts/
│   │   ├── follow_path_final.py       # Agricultural pattern (9-cycle)
│   │   ├── route_square.py            # Square pattern
│   │   ├── route_zigzag.py            # Zigzag pattern
│   │   ├── route_spiral.py            # Spiral pattern
│   │   ├── route_circle.py            # Circle pattern
│   │   ├── select_route.py            # Route selector utility
│   │   └── ROUTES_README.md           # Route documentation
│   │
│   └── worlds/
│       └── (empty - uses default world)
│
├── docs/
│   ├── SETUP_GUIDE.md                 # Installation instructions
│   ├
│   ├
│   └── API_REFERENCE.md               # Code documentation
│
├── data/
│   ├── simulation_run_*.csv           # Agricultural pattern data
│   ├── route_square_*.csv             # Square pattern data
│   ├── route_zigzag_*.csv             # Zigzag pattern data
│   ├── route_spiral_*.csv             # Spiral pattern data
│   ├── route_circle_*.csv             # Circle pattern data
│   └── README.md                      # Data documentation
│
├── scripts/
│   ├── analyze_results.py             # Data analysis script
│   └── run_trials.sh                  # Automated testing script
│
├── results/
│   ├── TABLE_8.1.txt                  # Performance metrics table
│   └── performance_summary.txt        # Analysis summary
│
├── videos/
│   ├── simulation_run_1.mp4           # Development videos
│   ├── simulation_run_2.mp4
│   ├── simulation_run_3.mp4
│   ├── simulation_run_4_working.mp4   # Working 4-wheel demo
│   ├── simulation_run_5_working.mp4   # 9-cycle agricultural
│   ├── simulation_run_6_final.mp4     # Final validation
│   ├── route_square_demo.mp4          # Square pattern demo
│   ├── route_zigzag_demo.mp4          # Zigzag pattern demo
│   ├── route_spiral_demo.mp4          # Spiral pattern demo
│   ├── route_circle_demo.mp4          # Circle pattern demo
│   └── README.md                      # Video descriptions
│
└── figures/
    ├── fig_8_1_simulation_environment.png
    ├── fig_8_2_simulation_view.png
    └── README.md                      # Figure descriptions
```
```

## Robot Specifications

### Physical Parameters
- **Base Dimensions**: 0.8m × 0.6m × 0.3m
- **Wheel Diameter**: 0.3m
- **Wheel Separation**: 0.7m
- **Total Mass**: ~17kg (base: 15kg, wheels: 1kg each)

### Performance Metrics
- **Success Rate**: 100%
- **Average Time per Waypoint**: ~XX.XX seconds
- **Path Completion**: Full agricultural row pattern
- **Total Distance**: ~XX.X meters

## Simulation Results

The simulation validates the differential drive architecture through:
- ✅ Successful completion of predefined agricultural path pattern
- ✅ 100% waypoint achievement rate
- ✅ Stable control throughout operation
- ✅ Repeatable performance across multiple trials

See `docs/SIMULATION_RESULTS.md` for detailed analysis.

## Thesis Context

This simulation forms Chapter 8 of the thesis, providing:
1. **Validation** of the architectural design methodology
2. **Quantitative metrics** for Table 8.1
3. **Comparative data** for design trade-offs
4. **Feasibility demonstration** of the integrated system

## Key Features

- **ROS Integration**: Full ROS Noetic compatibility
- **Gazebo Physics**: Realistic differential drive simulation
- **Data Logging**: Automatic CSV generation for analysis
- **Modular Design**: Easy to modify paths and parameters
- **Well Documented**: Extensive comments and documentation

## Testing

Run automated tests:
```bash
cd ~/catkin_ws/src/ugress_thesis_simulation
./scripts/run_trials.sh
```

## Data Analysis

Analyze collected data:
```bash
python3 scripts/analyze_results.py
```

## Contributing

This is a thesis project repository. For questions or suggestions, please open an issue.

## Citation

If you use this work in your research, please cite:
```bibtex
@mastersthesis{hassan2024ugress,
  author  = {Mahammed, Abdirahman Hassan},
  title   = {Design and Implementation of an Autonomous Ground-based Weed Elimination System},
  school  = {NMBU},
  year    = {2025},
  type    = {Master's Thesis}
}
```

## License

MIT License - see LICENSE file for details

## Author

**Abdirahman Hassan**
- Thesis Project: Autonomous Weed Elimination System
- Institution: Norwegian University of Life Sciences (NMBU)
- Year: 2025

## Acknowledgments

- ROS Community
- Gazebo Simulation Team
- Thesis Advisors and Committee

## Contact

For questions regarding this simulation:
- Open an issue in this repository
- Email: abdirahman.hassan.mahammed@nmbu.no

---

**Status**: 🤖 Simulation Validated | 📊 Data Collected
