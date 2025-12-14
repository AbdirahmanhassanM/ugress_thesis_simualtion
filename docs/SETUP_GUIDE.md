# Detailed Setup Guide

## Prerequisites

### System Requirements
- Ubuntu 20.04 LTS (64-bit)
- Minimum 8GB RAM
- 20GB free disk space
- Dedicated GPU recommended (but not required)

### Software Requirements
- ROS Noetic (full desktop installation)
- Gazebo 11
- Python 3.8 or higher

## Step-by-Step Installation

### 1. Install ROS Noetic
```bash
# Setup sources
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup keys
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Install ROS
sudo apt update
sudo apt install ros-noetic-desktop-full

# Environment setup
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install Dependencies
```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
sudo apt install python3-pip
sudo rosdep init
rosdep update
```

### 3. Create Catkin Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

### 4. Clone and Build Repository
```bash
cd ~/catkin_ws/src
git clone https://github.com/AbdirahmanhassanM/ugress_thesis_simualtion.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 5. Verify Installation
```bash
# Check ROS installation
rosversion -d
# Should output: noetic

# Check Gazebo
gazebo --version
# Should output: Gazebo multi-robot simulator, version 11.x

# Check package
rospack find ugress_sim
# Should output path to package

# Verify URDF
check_urdf ~/catkin_ws/src/ugress_thesis_simulation/ugress_sim/urdf/ugress.urdf
# Should output: robot name is: ugress
```

## Running the Simulation

### Basic Launch
```bash
# Terminal 1
roslaunch ugress_sim spawn_robot.launch

# Terminal 2 (after Gazebo fully loads)
rosrun ugress_sim follow_path_simple.py
```

### Expected Behavior

1. Gazebo window opens (may take 30-60 seconds first time)
2. Blue robot with black wheels appears in empty world
3. Robot begins moving forward after ~3 seconds
4. Robot follows rectangular path pattern
5. Console shows progress messages
6. CSV file generated in home directory

## Troubleshooting

### Issue: Gazebo doesn't start
```bash
killall -9 gzserver gzclient
roslaunch ugress_sim spawn_robot.launch
```

### Issue: Robot doesn't appear
```bash
# Check if model spawned
rosnode list
# Should show /spawn_robot

# Check robot description
rosparam get /robot_description
```

### Issue: Robot doesn't move
```bash
# Check if cmd_vel topic exists
rostopic list | grep cmd_vel

# Test manual control
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5" -r 10
```

### Issue: Permission denied on scripts
```bash
chmod +x ~/catkin_ws/src/ugress_thesis_simulation/ugress_sim/scripts/*.py
```

## Configuration

### Modify Path Pattern

Edit `ugress_sim/scripts/follow_path_simple.py`:
```python
# Line ~20: Modify movements list
movements = [
    (0.3, 0.0, 10.0, "Forward 3m"),    # (linear, angular, duration, description)
    (0.0, 0.5, 3.14, "Turn 90°"),
    # Add more segments...
]
```

### Adjust Robot Speed

Edit the launch file or modify speed in script:
```python
linear_speed = 0.3  # m/s (increase for faster)
angular_speed = 0.5  # rad/s (increase for faster turns)
```

### Change Robot Appearance

Edit `ugress_sim/urdf/ugress.urdf`:
```xml
<material name="blue">
  <color rgba="0.2 0.2 0.8 1"/>  <!-- Change RGBA values -->
</material>
```

## Data Collection

Output files are saved to home directory:
- `~/simple_path_YYYYMMDD_HHMMSS.csv` - Path execution data
- `~/.ros/log/` - ROS log files

## Next Steps

After successful installation:
1. Run baseline simulation (see README.md)
2. Analyze results with `scripts/analyze_results.py`
3. Modify parameters for your use case
4. Run multiple trials for statistical analysis

## Support

For issues:
1. Check this troubleshooting guide
2. Review ROS/Gazebo logs in `~/.ros/log/`
3. Open issue on GitHub repository
