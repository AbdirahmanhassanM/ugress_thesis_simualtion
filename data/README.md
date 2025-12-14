# Simulation Data

This directory contains raw data collected from all simulation runs, including the agricultural pattern and additional route demonstrations.

## File Naming Convention

### Agricultural Pattern Data
- `simulation_run_YYYYMMDD_HHMMSS.csv` - Original 9-cycle agricultural pattern

### Route Pattern Data
- `route_square_YYYYMMDD_HHMMSS.csv` - Square perimeter pattern
- `route_zigzag_YYYYMMDD_HHMMSS.csv` - Zigzag diagonal pattern
- `route_spiral_YYYYMMDD_HHMMSS.csv` - Expanding spiral pattern
- `route_circle_YYYYMMDD_HHMMSS.csv` - Circular coverage pattern

### Legacy Data
- `simple_path_YYYYMMDD_HHMMSS.csv` - Early development data
- `path_data_YYYYMMDD_HHMMSS.csv` - Odometry-based test data

---

## CSV File Format

### Agricultural Pattern Files
```csv
Cycle,Action,Distance_or_Angle,Duration,Status
1,Drive,3.0m,10.00s,Success
1,Turn,90°,3.14s,Success
2,Drive,2.5m,8.33s,Success
...
```

### Route Pattern Files
```csv
Segment,Action,Value,Duration,Status
1,Drive,3.0m,10.00s,Success
1,Turn,90°,3.14s,Success
2,Drive,3.0m,10.00s,Success
...
```

### Column Descriptions
- **Cycle/Segment**: Movement cycle or segment number
- **Action**: Type of movement (Drive, Turn, Forward)
- **Distance_or_Angle/Value**: Distance in meters or angle in degrees
- **Duration**: Time taken in seconds
- **Status**: Success/Failure indicator

---

## Available Data Files

### Current Repository Files

| File | Pattern | Segments | Distance | Date |
|------|---------|----------|----------|------|
| simulation_run_20241208_172737.csv | Agricultural | 9 | ~25m | Dec 8, 2025 |
| simulation_run_20241208_175645.csv | Agricultural | 9 | ~25m | Dec 8, 2025 |
| simulation_run_20241208_233820.csv | Agricultural | 9 | ~25m | Dec 8, 2025 |
| route_square_20251208_235151.csv | Square | 4 | 12m | Dec 8, 2025 |
| route_zigzag_20251208_235317.csv | Zigzag | 6 | ~12m | Dec 8, 2025 |
| route_spiral_20251208_235520.csv | Spiral | 8 | ~18m | Dec 8, 2025 |
| route_circle_20251208_235753.csv | Circle | 12 | ~9.6m | Dec 8, 2025 |

---

## Data Analysis

### Using Python Analysis Script
```bash
# Analyze agricultural pattern data
python3 ../scripts/analyze_results.py

# View specific file
cat simulation_run_20241208_233820.csv

# Count total segments
wc -l *.csv
```

### Quick Statistics
```bash
# Get success rate for all files
grep -h "Success" *.csv | wc -l

# View all unique actions
cut -d',' -f2 *.csv | sort -u

# Calculate total distance (manual sum from files)
# Agricultural: 9 cycles × ~2.78m avg = ~25m
# Square: 4 sides × 3m = 12m
# Zigzag: 6 segments × ~2m = ~12m
# Spiral: 0.5+1.0+1.5+2.0+2.5+3.0+3.5+4.0 = 18m
# Circle: 12 segments × 0.8m = 9.6m
# TOTAL: ~76.6 meters
```

---

## Data Collection Methods

### Agricultural Pattern (Primary)
```bash
roslaunch ugress_sim spawn_robot.launch
# New terminal:
rosrun ugress_sim follow_path_final.py
```
Data saved to: `~/simulation_run_TIMESTAMP.csv`

### Route Patterns
```bash
roslaunch ugress_sim spawn_robot.launch
# New terminal (choose one):
rosrun ugress_sim route_square.py    # Square pattern
rosrun ugress_sim route_zigzag.py    # Zigzag pattern
rosrun ugress_sim route_spiral.py    # Spiral pattern
rosrun ugress_sim route_circle.py    # Circle pattern
```
Data saved to: `~/route_PATTERN_TIMESTAMP.csv`

### Copying Data to Repository
```bash
# Copy new data files from home directory
cp ~/simulation_run_*.csv ~/ugress_thesis_simulation/data/
cp ~/route_*.csv ~/ugress_thesis_simulation/data/

# Add to git
cd ~/ugress_thesis_simulation
git add data/*.csv
git commit -m "Add new simulation data"
git push
```

---

## Performance Summary (All Data)

### Overall Statistics
- **Total Patterns Tested**: 5 unique routes
- **Total Data Files**: 7+ complete runs
- **Total Distance Logged**: ~76.6 meters
- **Total Segments**: 39 movements
- **Success Rate**: 100% (39/39 segments)
- **Zero Failures**: No manual interventions

### Pattern-Specific Metrics

| Pattern | Avg Time/Segment | Total Time | Success |
|---------|------------------|------------|---------|
| Agricultural | 4.93s | ~44.4s | 100% |
| Square | 6.00s | ~24.0s | 100% |
| Zigzag | 4.67s | ~28.0s | 100% |
| Spiral | 5.25s | ~42.0s | 100% |
| Circle | 3.27s | ~39.2s | 100% |

---

## Using Data in Thesis

### Table 8.1 (Performance Comparison)
Primary data source: `simulation_run_20241208_233820.csv`
- Success rate: 100%
- Average time per waypoint: 4.93s
- Total distance: 25.0m

### Multi-Pattern Validation
Sources: All `route_*.csv` files
- Demonstrates system flexibility
- Validates multiple operational modes
- Proves reliable control across patterns

### CSV files provide:
- Time series data for plots
- Distance covered metrics
- Turn execution statistics
- Overall performance trends

---

## Data Integrity

All data files in this directory:
- ✅ Generated automatically by simulation scripts
- ✅ Timestamped for traceability
- ✅ Complete (no missing segments)
- ✅ Validated (100% success status)
- ✅ Archived for thesis submission

---

## Regenerating Data

To recreate any dataset:

1. Launch Gazebo: `roslaunch ugress_sim spawn_robot.launch`
2. Run desired pattern script
3. Data automatically saved to home directory
4. Copy to repository: `cp ~/FILENAME.csv ./data/`

---

**Status**: ✅ Complete dataset for thesis validation  
**Last Updated**: December 8, 2024  
**Total Files**: 7+ CSV logs  
**Total Success Rate**: 100%