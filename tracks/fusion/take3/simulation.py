import numpy as np
import json
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from matplotlib.widgets import Slider
from sensor_fusion import GlobalTrackManager

def latlon_to_meters(lat, lon, ref_lat=37.805, ref_lon=-122.415):
    # Quick localized coordinate projection
    x = (lon - ref_lon) * 111320.0 * np.cos(np.radians(ref_lat))
    y = (lat - ref_lat) * 111000.0
    return x, y

class SimulatedContact:
    def __init__(self, data):
        self.id = data["id"]
        self.name = data["name"]
        self.speed_mps = data["speed_knots"] * 0.51444
        self.waypoints = [latlon_to_meters(wp[1], wp[0]) for wp in data["waypoints"]]
        self.current_wp_idx = 0
        self.px, self.py = self.waypoints[0]
        self.vx, self.vy = 0.0, 0.0
        self._update_velocity()

    def _update_velocity(self):
        if self.current_wp_idx >= len(self.waypoints):
            self.vx, self.vy = 0.0, 0.0
            return
        tx, ty = self.waypoints[self.current_wp_idx]
        dx, dy = tx - self.px, ty - self.py
        dist = np.sqrt(dx**2 + dy**2)
        if dist < 15.0:
            self.current_wp_idx = (self.current_wp_idx + 1) % len(self.waypoints)
            tx, ty = self.waypoints[self.current_wp_idx]
            dx, dy = tx - self.px, ty - self.py
            dist = np.sqrt(dx**2 + dy**2)
        if dist > 0:
            self.vx = (dx / dist) * self.speed_mps
            self.vy = (dy / dist) * self.speed_mps

    def update_position(self, dt):
        self._update_velocity()
        self.px += self.vx * dt
        self.py += self.vy * dt

def run_interactive_simulation():
    try:
        with open("generated/contacts_placement.json", "r") as f:
            config = json.load(f)
    except FileNotFoundError:
        print("Error: contacts_placement.json not found in 'generated/' folder.")
        return
        
    contacts = [SimulatedContact(c) for c in config["contacts"]]
    manager = GlobalTrackManager()
    
    dt = 0.1
    fig, ax = plt.subplots(figsize=(10, 8))
    plt.subplots_adjust(bottom=0.25)
    
    ax.set_xlim(-2500, 2500)
    ax.set_ylim(-2500, 2500)
    
    # Draw Static Saildrone
    ax.scatter([0], [0], c='blue', marker='p', s=150, label='Stationary Saildrone', zorder=5)
    
    # Init Plot Handles
    true_scatter = ax.scatter([], [], c='green', marker='o', s=40, label='True Targets')
    est_scatter = ax.scatter([], [], c='red', marker='x', s=60, label='EKF Tracks')
    
    ax.set_title("Perception Stack Tuning Interface")
    ax.grid(True)
    ax.legend(loc='upper right')
    
    # Add UI Interactive Tuning Knobs
    ax_q = plt.axes([0.15, 0.16, 0.7, 0.02])
    ax_r_rad = plt.axes([0.15, 0.12, 0.7, 0.02])
    ax_r_cam = plt.axes([0.15, 0.08, 0.7, 0.02])
    ax_gate = plt.axes([0.15, 0.04, 0.7, 0.02])
    
    s_q = Slider(ax_q, 'Process Noise (Q)', 0.01, 5.0, valinit=0.25)
    s_r_rad = Slider(ax_r_rad, 'Radar Noise Scale', 0.1, 10.0, valinit=1.0)
    s_r_cam = Slider(ax_r_cam, 'Camera Noise Scale', 0.1, 10.0, valinit=1.0)
    s_gate = Slider(ax_gate, 'Chi Gate Threshold', 2.0, 20.0, valinit=11.34)
    
    writer = FFMpegWriter(fps=10)
    
    print("Launching dynamic simulation window... Close the GUI window to complete recording export.")
    
    t = 0.0
    try:
        # FIXED: Changed .saved() to the correct .saving() context manager syntax
        with writer.saving(fig, "generated/running_tracks.mp4", dpi=100):
            for frame in range(1500): # 15 seconds run
                t += dt
                
                # Fetch parameters from live slider positions
                q_val = s_q.val
                r_rad_scale = s_r_rad.val
                r_cam_scale = s_r_cam.val
                gate_val = s_gate.val
                
                # Update true vessel trajectory states
                true_x, true_y = [], []
                radar_measurements = []
                camera_bearings = []
                
                for c in contacts:
                    c.update_position(dt)
                    true_x.append(c.px)
                    true_y.append(c.py)
                    
                    # Generate observations with adjustable noise strengths
                    r_true = np.sqrt(c.px**2 + c.py**2)
                    b_true = np.arctan2(c.py, c.px)
                    
                    # Simulated sensors feed
                    if frame % 2 == 0: # 5Hz radar rate
                        r_meas = r_true + np.random.normal(0, 2.0 * r_rad_scale)
                        b_meas = b_true + np.random.normal(0, 0.01 * r_rad_scale)
                        r_dot_meas = (c.px*c.vx + c.py*c.vy)/r_true + np.random.normal(0, 0.2)
                        radar_measurements.append(np.array([[r_meas], [b_meas], [r_dot_meas]]))
                        
                    camera_bearings.append(b_true + np.random.normal(0, 0.01 * r_cam_scale))
                
                # Dynamic Filter Ingestion Update Cycle
                manager.predict_all(dt, q_val)
                
                R_rad_mat = np.diag([2.0, np.radians(0.5), 0.15])**2 * r_rad_scale
                R_cam_mat = np.array([[np.radians(1.0)**2]]) * r_cam_scale
                
                manager.process_radar_measurements(radar_measurements, t, R_rad_mat, gate_val)
                manager.process_camera_measurements(camera_bearings, t, R_cam_mat, gate_val)
                manager.manage_lifecycles(t)
                
                # Collect track maps
                est_x = [float(track.x[0]) for track in manager.tracks.values()]
                est_y = [float(track.x[1]) for track in manager.tracks.values()]
                
                # Re-plot UI artists
                true_scatter.set_offsets(np.c_[true_x, true_y])
                est_scatter.set_offsets(np.c_[est_x, est_y])
                
                fig.canvas.draw_idle()
                writer.grab_frame()
                plt.pause(0.01)
    except Exception as e:
        print(f"Simulation window stopped: {e}")
        
    print("Video output file rendered and fully generated within generated/running_tracks.mp4")

if __name__ == "__main__":
    run_interactive_simulation()
