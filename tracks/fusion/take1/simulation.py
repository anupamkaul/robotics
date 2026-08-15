import numpy as np
import matplotlib.pyplot as plt
from sensor_fusion import GlobalTrackManager

def run_maritime_simulation():
    np.random.seed(42)
    dt, total_time = 0.1, 25.0
    steps = int(total_time / dt)
    manager = GlobalTrackManager()
    
    true_px, true_py, true_vx, true_vy = 200.0, 150.0, -12.0, -3.0
    R_radar = np.diag([2.0, np.radians(0.5), 0.1])**2
    R_cam = np.array([[np.radians(1.0)**2]])
    
    true_trajectory, estimated_trajectory = [], []
    
    for step in range(steps):
        t = step * dt
        if t > 19.0: true_vx, true_vy = -6.0, -15.0 # Sharp Turn Evasion
        true_px += true_vx * dt; true_py += true_vy * dt
        true_trajectory.append((true_px, true_py))
        
        manager.predict_all(dt)
        
        # Corner Cases Conditions
        cam_reset = (5.0 <= t < 9.0)
        cam_drift = 0.14 if (9.0 <= t < 14.0) else 0.0  # Drift Error
        radar_clutter_fault = (14.0 <= t < 19.0)
        
        if step % 2 == 0: # 5Hz Radar
            radar_meas = []
            if radar_clutter_fault:
                for _ in range(3): # Inject false sea-clutter points
                    radar_meas.append(np.array([[np.sqrt(true_px**2+true_py**2)+np.random.uniform(-40,40)], [np.arctan2(true_py, true_px)+np.random.uniform(-0.3,0.3)], [0]]))
            else:
                radar_meas.append(np.array([[np.sqrt(true_px**2+true_py**2)+np.random.normal(0,1.5)], [np.arctan2(true_py, true_px)+np.random.normal(0,0.01)], [0]]))
            manager.process_radar_measurements(radar_meas, t, R_radar, anti_sea_clutter_active=radar_clutter_fault)
            
        if not cam_reset: # 10Hz Cameras
            z_cam = np.arctan2(true_py, true_px) + np.random.normal(0, 0.01) + cam_drift
            manager.process_camera_measurements([z_cam], t, R_cam, sensor_drift=cam_drift, reset_active=cam_reset)
            
        manager.manage_lifecycles(t)
        if 1 in manager.tracks:
            estimated_trajectory.append((manager.tracks[1].x[0,0], manager.tracks[1].x[1,0]))
        elif len(manager.tracks) > 0:
            active_id = list(manager.tracks.keys())[0]
            estimated_trajectory.append((manager.tracks[active_id].x[0,0], manager.tracks[active_id].x[1,0]))
        else:
            estimated_trajectory.append((np.nan, np.nan))
            
    # Draw Results Plot
    true_trajectory = np.array(true_trajectory); estimated_trajectory = np.array(estimated_trajectory)
    plt.figure(figsize=(10, 6))
    plt.plot(true_trajectory[:,0], true_trajectory[:,1], 'g-', label='Ground Truth Target')
    plt.plot(estimated_trajectory[:,0], estimated_trajectory[:,1], 'r--', label='Fused Perception Estimate')
    plt.scatter([0], [0], c='black', marker='p', s=100, label='Our Saildrone')
    plt.title("Perception Stack EKF Corner-Case Simulation")
    plt.grid(True); plt.legend(); plt.savefig("generated/perception_simulation_output.png")
    print("Simulation complete! Graph saved to generated/perception_simulation_output.png")

if __name__ == '__main__':
    run_maritime_simulation()
