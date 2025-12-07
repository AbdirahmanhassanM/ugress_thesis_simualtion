# Simulation Data

This directory contains raw data collected from simulation runs.

## File Format

CSV files with naming convention: `simple_path_YYYYMMDD_HHMMSS.csv`

### Columns

| Column | Description | Unit |
|--------|-------------|------|
| Segment | Segment number in path | - |
| Action | Description of movement | - |
| Duration | Time taken for segment | seconds |
| Status | Success/Failure | - |

## Sample Data
```csv
Segment,Action,Duration,Status
1,Forward 3m,10.00,Success
2,Turn 90°,3.14,Success
...
```

## Usage

Analyze data using:
```bash
python3 ../scripts/analyze_results.py
```

## Data Collection

Data is automatically generated when running:
```bash
rosrun ugress_sim follow_path_simple.py
```

Files are saved to home directory and should be copied here for archival.
