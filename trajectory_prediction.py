import time
import numpy as np
import matplotlib.pyplot as plt
from pymavlink import mavutil
from geographiclib.geodesic import Geodesic

# Connect to MAVLink SITL
mav = mavutil.mavlink_connection("udp:127.0.0.1:14550")
print("✅ Connected to MAVLink SITL")

# Wait for first GPS data
msg = None
while msg is None:
    msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

# Set up geographic reference
geod = Geodesic.WGS84
origin_lat = msg.lat * 1e-7
origin_lon = msg.lon * 1e-7
origin_alt = msg.relative_alt * 1e-3

# Initialize graph
plt.ion()
fig, ax = plt.subplots()
ax.set_xlim(-50, 50)
ax.set_ylim(-50, 50)
ax.set_xlabel("East (m)")
ax.set_ylabel("North (m)")
ax.set_title("Real-Time Drone Position, Trajectory, and Traveled Path")

# Plot elements
position_plot, = ax.plot([], [], 'ro', label="Current Position")
trajectory_plot, = ax.plot([], [], 'b--', label="Predicted Trajectory")
traveled_path_plot, = ax.plot([], [], 'g-', label="Traveled Path")  # New path plot
ax.legend()

# Initialize state variables
position = np.array([0.0, 0.0])
velocity = np.array([0.0, 0.0])
acceleration = np.array([0.0, 0.0])
dt = 0.1  # Time step
prediction_horizon = 20  # Predict next 5 seconds
traveled_path = []  # Store past positions

# Scale factor for converting RAW IMU acceleration (mg) to m/s²
ACCEL_SCALE = 9.80665 / 1000.0  # Convert milli-g to m/s²

while True:
    # Receive GPS data
    gps_msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    lat = gps_msg.lat * 1e-7
    lon = gps_msg.lon * 1e-7
    alt = gps_msg.relative_alt * 1e-3

    # Convert lat/lon to meters relative to the origin
    geo_result = geod.Inverse(origin_lat, origin_lon, lat, lon)
    distance = geo_result['s12']  # Geodesic distance in meters
    azimuth = np.radians(geo_result['azi1'])  # Convert azimuth to radians

    east_m = distance * np.sin(azimuth)
    north_m = distance * np.cos(azimuth)

    # Fetch acceleration from MAVLink (RAW IMU data)
    imu_msg = mav.recv_match(type='RAW_IMU', blocking=True)
    accel_x = imu_msg.xacc * ACCEL_SCALE  # Convert to m/s²
    accel_y = imu_msg.yacc * ACCEL_SCALE  # Convert to m/s²

    # Compute velocity and acceleration
    new_position = np.array([east_m, north_m])
    new_velocity = (new_position - position) / dt
    new_acceleration = np.array([accel_x, accel_y])  # Use real acceleration from IMU

    # Update state
    position = new_position
    velocity = new_velocity
    acceleration = new_acceleration

    # Store traveled path
    traveled_path.append(position.copy())
    traveled_path_array = np.array(traveled_path)

    # Predict trajectory using acceleration
    future_positions = []
    temp_position = position.copy()
    temp_velocity = velocity.copy()

    for _ in range(prediction_horizon):
        temp_velocity += acceleration * dt
        temp_position += temp_velocity * dt + 0.5 * acceleration * dt**2  # Apply acceleration
        future_positions.append(temp_position.copy())

    future_positions = np.array(future_positions)

    # Update plot
    position_plot.set_xdata(position[0])
    position_plot.set_ydata(position[1])

    trajectory_plot.set_xdata(future_positions[:, 0])
    trajectory_plot.set_ydata(future_positions[:, 1])

    if len(traveled_path) > 1:
        traveled_path_plot.set_xdata(traveled_path_array[:, 0])
        traveled_path_plot.set_ydata(traveled_path_array[:, 1])

    ax.relim()
    ax.autoscale_view()

    plt.draw()
    plt.pause(dt)

