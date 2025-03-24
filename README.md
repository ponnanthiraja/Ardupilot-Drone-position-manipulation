# Drone Trajectory Formation & Prediction with Tracking Algorithm

## Overview
This project consists of two main components:
1. **Trajectory Formation & Tracking Algorithm**: Generates waypoints between a start and end position, updates an Intermediate Target Point (ITP) dynamically, and visualizes the drone's movement along the trajectory.
2. **Trajectory Prediction**: Uses real-time MAVLink data to estimate the drone’s future trajectory based on velocity and acceleration.

## Features
### Trajectory Formation & Tracking
- Converts GPS coordinates to local meters and vice versa
- Generates waypoints for smooth trajectory formation
- Updates ITP dynamically based on drone position
- Retrieves real-time drone position using MAVLink
- Visualizes the drone path, ITP, and trajectory in real-time

### Trajectory Prediction
- Retrieves real-time GPS and IMU acceleration data
- Computes velocity and acceleration to estimate future drone positions
- Predicts trajectory over a given time horizon
- Visualizes current position, predicted trajectory, and traveled path

## Dependencies
Ensure you have the following Python packages installed:
```bash
pip install numpy matplotlib pymavlink geographiclib
```

## How It Works
### Trajectory Formation & Tracking
1. The user inputs a destination (latitude, longitude).
2. The drone's initial position is obtained using MAVLink.
3. Waypoints are generated between the start and end positions.
4. The drone's position is tracked in real-time, updating the ITP dynamically.
5. A real-time plot visualizes the drone’s movement, ITP, and traveled path.

### Trajectory Prediction
1. The script connects to a MAVLink SITL simulation.
2. It retrieves GPS position and IMU acceleration data.
3. The script estimates the drone’s velocity and acceleration.
4. Using this data, it predicts the future trajectory.
5. The real-time graph updates with the current position, predicted trajectory, and traveled path.
6. 

## Tracking Algorithm
The algorithm divides the drone's movement into different zones:
- **Zone 0**: If the drone is close to the ITP (within 13 meters), the ITP moves to the next waypoint.
- **Zone 1**: If the perpendicular foot of the drone’s position on the track is farther from the destination than the ITP, the ITP remains unchanged.
- **Zone 2**: If the drone is near the next waypoint and has covered at least 98% of the distance, the ITP moves forward.

## Usage
### Trajectory Formation & Tracking
1. Start your MAVLink connection (e.g., using a simulator like ArduPilot SITL).
2. Run the script:
   ```bash
   python tracking_script.py
   ```
3. Enter the destination latitude and longitude when prompted.
4. The script will track and visualize the drone’s movement.

### Trajectory Prediction
1. Start your MAVLink connection.
2. Run the script:
   ```bash
   python prediction_script.py
   ```
3. The script will continuously update and predict the drone's future positions.

## Code Structure
### Tracking Algorithm Code
- `latlon_to_meters()`, `meters_to_latlon()`: Convert GPS coordinates to local metric coordinates and vice versa.
- `DroneNavigation`: Handles waypoint generation and ITP updates.
- `get_mavlink_data()`: Fetches real-time drone position.
- `start_realtime_navigation()`: Runs the real-time tracking and visualization loop.

### Trajectory Prediction Code
- `geod.Inverse()`: Computes geodesic distance between two GPS points.
- `velocity` and `acceleration` calculations: Uses IMU data for precise motion estimation.
- `future_positions`: Predicts drone trajectory over a time horizon.
- Real-time visualization with Matplotlib.
### Video Demonstrations
- [Trajectory Formation & Tracking Video](https://youtu.be/TeVu1U0RcAI?si=EyHuwn_iCM-fspt4)
- [Trajectory Prediction Video](https://youtu.be/ktuHP_TYUwY?si=OkhxlZJ9WTY2YXLt)

Trajectory Prediction Video 
## Exit
Press `Ctrl+C` to safely exit the program.

## License
This project is licensed under the MIT License.

## Author
[PONNANTHIRAJA K]

