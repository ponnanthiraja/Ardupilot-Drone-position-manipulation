import numpy as np
import matplotlib.pyplot as plt
from pymavlink import mavutil
import signal
import sys

# ✅ Convert lat/lon to meters (approximate)
def latlon_to_meters(lat, lon, ref_lat, ref_lon):
    R = 6378137  
    dlat = np.radians(lat - ref_lat)
    dlon = np.radians(lon - ref_lon)
    x = dlon * R * np.cos(np.radians(ref_lat))
    y = dlat * R
    return x, y

# ✅ Convert meters back to lat/lon
def meters_to_latlon(x, y, ref_lat, ref_lon):
    R = 6378137  
    dlat = y / R
    dlon = x / (R * np.cos(np.radians(ref_lat)))
    lat = ref_lat + np.degrees(dlat)
    lon = ref_lon + np.degrees(dlon)
    return lat, lon

class DroneNavigation:
    def __init__(self, start, end, waypoint_distance=15):
        self.start = start  
        self.end = end
        self.waypoint_distance = waypoint_distance
        self.track = self.generate_waypoints(start, end, waypoint_distance)
        self.itp_index = 0  # Start at first waypoint

    def calculate_distance(self, point1, point2):
        return np.linalg.norm(np.array(point1) - np.array(point2))

    def generate_waypoints(self, start, end, waypoint_distance):
        x_start, y_start = start
        x_end, y_end = end
        total_distance = self.calculate_distance(start, end)
        num_waypoints = int(np.ceil(total_distance / waypoint_distance))

        waypoints = [(x_start + (x_end - x_start) * i / num_waypoints, 
                      y_start + (y_end - y_start) * i / num_waypoints) 
                     for i in range(num_waypoints + 1)]
        return waypoints

    def compute_perpendicular_foot(self, current_position, itp, next_wp):
        """Find the perpendicular foot from current_position to line segment (itp → next_wp)."""
        A, B, P = np.array(itp), np.array(next_wp), np.array(current_position)
        AB, AP = B - A, P - A
        t = np.dot(AP, AB) / np.dot(AB, AB)
        t = np.clip(t, 0, 1)
        return tuple(A + t * AB)

    def update_itp(self, current_position):
        """Update the Intermediate Target Point (ITP) based on the drone's position."""
        if self.itp_index >= len(self.track) - 1:
            return self.itp_index, self.track[self.itp_index]  # Stop at the last waypoint

        itp = self.track[self.itp_index]
        distance = self.calculate_distance(current_position, itp)

        # ✅ Zone 0: If close to ITP, move forward
        if distance < 13:
            self.itp_index = min(self.itp_index + 1, len(self.track) - 1)
            return self.itp_index, self.track[self.itp_index]

        if self.itp_index + 1 < len(self.track):
            next_wp = self.track[self.itp_index + 1]
            perpendicular_foot = self.compute_perpendicular_foot(current_position, itp, next_wp)
            distance_to_next_wp = self.calculate_distance(current_position, next_wp)
            distance_to_perp_foot = self.calculate_distance(current_position, perpendicular_foot)

            # ✅ Zone 1: ITP stays if it's closer to track end than perpendicular foot
            if self.calculate_distance(itp, self.track[-1]) < self.calculate_distance(perpendicular_foot, self.track[-1]):
                return self.itp_index, itp

            # ✅ Zone 2: If drone is near next waypoint, move forward
            if distance / distance_to_next_wp > 0.98:
                self.itp_index = min(self.itp_index + 1, len(self.track) - 1)
                return self.itp_index, self.track[self.itp_index]

        return self.itp_index, itp  # Default: Stay at current ITP

def get_mavlink_data(master, ref_lat, ref_lon):
    """Get real-time drone position from MAVLink."""
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        vx, vy = msg.vx / 100, msg.vy / 100
        lat, lon = msg.lat / 1e7, msg.lon / 1e7
        x, y = latlon_to_meters(lat, lon, ref_lat, ref_lon)
        return (x, y), (vx, vy)
    return None, None

def signal_handler(sig, frame):
    """Handle Ctrl+C to safely exit."""
    print("\n[INFO] Exiting program...")
    sys.exit(0)

def start_realtime_navigation():
    """Run real-time navigation with ITP updates."""
    end_lat, end_lon = map(float, input("Enter destination point (lat, lon): ").split(','))
    master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

    while True:
        start_position, _ = get_mavlink_data(master, end_lat, end_lon)
        if start_position:
            break

    end_position = latlon_to_meters(end_lat, end_lon, end_lat, end_lon)
    drone_nav = DroneNavigation(start_position, end_position)

    plt.ion()
    fig, ax = plt.subplots()
    
    track_x, track_y = zip(*drone_nav.track)
    track_x, track_y = list(track_x), list(track_y)
    ax.plot(track_x, track_y, 'b', label="Waypoint Trajectory", alpha=0.6)

    drone_dot, = ax.plot([], [], 'ro', label="Drone Position")
    itp_dot, = ax.plot([], [], 'go', label="ITP Position")  # ✅ Added ITP visualization
    travelled_path, = ax.plot([], [], 'r-', label="Travelled Path")
    path_x, path_y = [], []

    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.legend()
    signal.signal(signal.SIGINT, signal_handler)

    print("Tracking in real-time... Press Ctrl+C to exit.")

    while True:
        current_position, _ = get_mavlink_data(master, end_lat, end_lon)
        if not current_position:
            continue  

        # ✅ Update ITP
        itp_index, itp = drone_nav.update_itp(current_position)

        # ✅ Update Drone Position
        drone_dot.set_data(current_position[0], current_position[1])
        itp_dot.set_data(itp[0], itp[1])  # Show ITP in real-time
        
        # Append to travelled path
        path_x.append(current_position[0])
        path_y.append(current_position[1])
        travelled_path.set_data(path_x, path_y)

        # Adjust plot limits dynamically
        ax.set_xlim(min(path_x + track_x) - 20, max(path_x + track_x) + 20)
        ax.set_ylim(min(path_y + track_y) - 20, max(path_y + track_y) + 20)

        plt.draw()
        plt.pause(0.1)

if __name__ == "__main__":
    start_realtime_navigation()

