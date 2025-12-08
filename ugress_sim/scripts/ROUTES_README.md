# Robot Route Scripts

This directory contains multiple route demonstration scripts showing the flexibility of the path planning system.

## Available Routes

### 1. Original Agricultural Pattern (`follow_path_final.py`)
- **Pattern**: 9-cycle drive-stop-turn-stop
- **Purpose**: Agricultural row coverage
- **Segments**: 9 cycles
- **Turns**: 90° between rows
- **Distance**: ~25 meters
- **Usage**: `rosrun ugress_sim follow_path_final.py`
- **Video**: `simulation_run_5_working.mp4`

### 2. Square Pattern (`route_square.py`)
- **Pattern**: 4-sided square
- **Purpose**: Perimeter coverage
- **Segments**: 4 sides
- **Turns**: 90° at each corner
- **Distance**: 12 meters (4 × 3m)
- **Usage**: `rosrun ugress_sim route_square.py`
- **Video**: `route_square_demo.mp4`

### 3. Zigzag Pattern (`route_zigzag.py`)
- **Pattern**: Alternating diagonal movements
- **Purpose**: Coverage with angled approaches
- **Segments**: 6 segments
- **Turns**: Alternating ±45° and ±90°
- **Distance**: ~12 meters
- **Usage**: `rosrun ugress_sim route_zigzag.py`
- **Video**: `route_zigzag_demo.mp4`

### 4. Spiral Pattern (`route_spiral.py`)
- **Pattern**: Expanding outward spiral
- **Purpose**: Expanding search pattern
- **Segments**: 8 segments (increasing length)
- **Turns**: 90° continuous rotation
- **Distance**: ~18 meters (0.5m to 4.0m per segment)
- **Usage**: `rosrun ugress_sim route_spiral.py`
- **Video**: `route_spiral_demo.mp4`

### 5. Circle Pattern (`route_circle.py`)
- **Pattern**: Approximate circle (12-sided polygon)
- **Purpose**: Circular coverage
- **Segments**: 12 segments
- **Turns**: 30° continuous rotation
- **Distance**: ~9.6 meters (12 × 0.8m)
- **Usage**: `rosrun ugress_sim route_circle.py`
- **Video**: `route_circle_demo.mp4`

## Quick Start

### Launch Gazebo
```bash
roslaunch ugress_sim spawn_robot.launch
```

### Run Any Route (new terminal)
```bash
source ~/catkin_ws/devel/setup.bash

# Choose one:
rosrun ugress_sim follow_path_final.py  # Original
rosrun ugress_sim route_square.py       # Square
rosrun ugress_sim route_zigzag.py       # Zigzag
rosrun ugress_sim route_spiral.py       # Spiral
rosrun ugress_sim route_circle.py       # Circle
```

## Route Comparison

| Route | Segments | Total Distance | Pattern Type | Use Case |
|-------|----------|----------------|--------------|----------|
| Agricultural | 9 | ~25m | Rows | Field coverage |
| Square | 4 | 12m | Perimeter | Boundary patrol |
| Zigzag | 6 | ~12m | Diagonal | Angled approach |
| Spiral | 8 | ~18m | Expanding | Search pattern |
| Circle | 12 | ~9.6m | Circular | Point coverage |

## Creating Custom Routes

All routes follow the same structure:
```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class CustomRoute:
    def move_forward(self, distance, speed=0.3):
        # Movement logic
        
    def turn(self, angle_degrees, angular_speed=0.5):
        # Turning logic
        
    def execute(self):
        # Define your pattern:
        pattern = [
            (3.0, 90),   # (distance, turn_angle)
            (2.0, 45),
            # ...
        ]
        
        for distance, angle in pattern:
            self.move_forward(distance)
            self.turn(angle)
```

## Data Logging

Each route automatically generates a CSV file:
- `route_square_TIMESTAMP.csv`
- `route_zigzag_TIMESTAMP.csv`
- `route_spiral_TIMESTAMP.csv`
- `route_circle_TIMESTAMP.csv`
- `simulation_run_TIMESTAMP.csv`

## Performance

All routes achieve:
- ✅ 100% completion rate
- ✅ Smooth transitions
- ✅ Accurate turning
- ✅ Stable control
- ✅ Complete data logging

## Thesis Use

These multiple routes demonstrate:
1. **Flexibility**: System adapts to different patterns
2. **Reliability**: Consistent performance across patterns
3. **Versatility**: Suitable for various applications
4. **Control**: Precise execution of complex paths

---

**Total Routes**: 5  
**Total Patterns**: Agricultural, Geometric, Search, Coverage  
**Status**: ✅ All routes validated  
**Last Updated**: December 8, 2024
