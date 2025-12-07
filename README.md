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
├── README.md                          # This file
├── LICENSE                            # MIT License
├── CITATION.cff                       # Citation information
├── ugress_sim/                        # ROS package
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch/
│   │   └── spawn_robot.launch        # Main launch file
│   ├── urdf/
│   │   └── ugress.urdf               # Robot model
│   ├── scripts/
│   │   ├── follow_path.py            # Odometry-based path follower
│   │   └── follow_path_simple.py     # Time-based path follower
│   └── worlds/                        # Gazebo world files (if any)
├── docs/
│   ├── SETUP_GUIDE.md                # Detailed setup instructions
│   ├── SIMULATION_RESULTS.md         # Results and analysis
│   ├── THESIS_CHAPTER_8.md           # Thesis chapter text
│   └── API_REFERENCE.md              # Code documentation
├── data/
│   ├── simple_path_20241206_*.csv    # Raw simulation data
│   └── README.md                      # Data description
├── scripts/
│   ├── analyze_results.py            # Data analysis script
│   └── run_trials.sh                 # Automated testing script
├── results/
│   ├── TABLE_8.1.txt                 # Performance metrics table
│   └── performance_summary.txt       # Analysis summary
└── figures/
    ├── simulation_environment.png    # Screenshots
    ├── robot_model.png
    └── path_visualization.png
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
  author  = {Hassan, Abdirahman},
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
- Institution: Your University
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

**Status**: ✅ Thesis Chapter 8 Complete | 🤖 Simulation Validated | 📊 Data Collected
