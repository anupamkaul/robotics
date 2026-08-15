import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider
from sensor_fusion import MaritimeEKF, latlon_to_meters

def run_gui():
    with open('generated/contacts_placement.json', 'r') as f:
        data = json.load(f)
    
    ref_lat = data['reference_lat']
    ref_lon = data['reference_lon']
    
    contacts_raw = data['contacts']
    true_targets = {}
    for c in contacts_raw:
        mx, my = latlon_to_meters(c['lat'], c['lon'], ref_lat, ref_lon)
        true_targets[c['id']] = {
            'x': mx, 'y': my,
            'vx': c['v_east'], 'vy': c['v_north'],
            'desc': c['description'],
            'history': []
        }
        
    dt = 0.5
    total_steps = 60
    
    # Pre-generate positions over time to prevent live state side-effects in animation sliders
    for step in range(total_steps):
        for cid, target in true_targets.items():
            cx = target['x'] + target['vx'] * (step * dt)
            cy = target['y'] + target['vy'] * (step * dt)
            target['history'].append((cx, cy))

    fig, ax = plt.subplots(figsize=(10, 8))
    plt.subplots_adjust(bottom=0.35)
    
    ax.set_xlim(-1500, 1500)
    ax.set_ylim(-1500, 1500)
    ax.set_title("Saildrone Perception Stack Multi-Target Tracker")
    ax.set_xlabel("Meters East")
    ax.set_ylabel("Meters North")
    ax.grid(True, linestyle=':', alpha=0.6)
    
    # UI Sliders setup
    ax_q = plt.axes([0.15, 0.22, 0.7, 0.03])
    ax_r_noise = plt.axes([0.15, 0.17, 0.7, 0.03])
    ax_c_noise = plt.axes([0.15, 0.12, 0.7, 0.03])
    ax_gate = plt.axes([0.15, 0.07, 0.7, 0.03])
    
    s_q = Slider(ax_q, 'Process Noise (Q)', 0.01, 5.0, valinit=0.5, valfmt="%1.2f")
    s_r_noise = Slider(ax_r_noise, 'Radar Range Noise', 0.5, 30.0, valinit=5.0, valfmt="%1.1f m")
    s_c_noise = Slider(ax_c_noise, 'Cam Bearing Noise', 0.1, 5.0, valinit=1.0, valfmt="%1.1f deg")
    s_gate = Slider(ax_gate, 'Chi-Sq Gate Size', 1.0, 30.0, valinit=15.0, valfmt="%1.1f")
    
    # Graphic elements
    ax.scatter(0, 0, color='blue', marker='p', s=200, label='Saildrone (Us)', zorder=5)
    
    target_dots, = ax.plot([], [], 'go', label='Ground Truth Targets', markersize=6)
    est_dots, = ax.plot([], [], 'rx', label='EKF Tracks', markersize=8, markeredgewidth=2)
    
    text_labels = []
    
    legend = ax.legend(loc='upper right')
    
    def update_frame(frame_idx):
        for lbl in text_labels:
            lbl.remove()
        text_labels.clear()
        
        q_val = s_q.val
        r_noise_val = s_r_noise.val
        theta_noise_val = np.radians(0.5)
        c_noise_val = np.radians(s_c_noise.val)
        gate_val = s_gate.val
        
        # Instantiate active tracks and compute filter performance dynamically for this slider state
        active_tracks = {}
        next_tid = 1
        
        curr_est_x, curr_est_y = [], []
        curr_true_x, curr_true_y = [], []
        
        for s in range(frame_idx + 1):
            # Predict
            for tid, track in active_tracks.items():
                track.predict(dt, q_acc=q_val)
                
            # Simulate measurements at time s
            for cid, target in true_targets.items():
                tx, ty = target['history'][s]
                if s == frame_idx:
                    curr_true_x.append(tx)
                    curr_true_y.append(ty)
                
                # Radar measurement (Every step, 100% chance for simplicity)
                r_true = np.sqrt(tx**2 + ty**2)
                theta_true = np.arctan2(ty, tx)
                r_meas = r_true + np.random.normal(0, r_noise_val)
                theta_meas = theta_true + np.random.normal(0, theta_noise_val)
                
                # Camera measurement
                c_meas = theta_true + np.random.normal(0, c_noise_val)
                
                # Data Association
                assigned = False
                for tid, track in active_tracks.items():
                    if track.update_radar(r_meas, theta_meas, r_noise_val, theta_noise_val, gate_size=gate_val):
                        assigned = True
                        track.update_camera(c_meas, c_noise_val, gate_size=gate_val)
                        break
                
                if not assigned and s == frame_idx:
                    # Initialize track if it doesn't match
                    new_trk = MaritimeEKF(next_tid, r_meas*np.cos(theta_meas), r_meas*np.sin(theta_meas), q_acc=q_val)
                    active_tracks[next_tid] = new_trk
                    next_tid += 1
            
            # Age tracks
            for tid in list(active_tracks.keys()):
                if s != frame_idx:
                    active_tracks[tid].missed_frames += 1
                    if active_tracks[tid].missed_frames > 4:
                        del active_tracks[tid]
                        
        for tid, track in active_tracks.items():
            curr_est_x.append(track.x[0, 0])
            curr_est_y.append(track.x[1, 0])
            lbl = ax.text(track.x[0, 0]+15, track.x[1, 0]+15, f"ID:{track.track_id}", color='red', fontsize=8)
            text_labels.append(lbl)
            
        target_dots.set_data(curr_true_x, curr_true_y)
        est_dots.set_data(curr_est_x, curr_est_y)
        
        return [target_dots, est_dots] + text_labels

    # Create local preview animation object
    ani = animation.FuncAnimation(fig, update_frame, frames=total_steps, interval=200, blit=False)
    
    # Save a running video file of the tracks to disk
    print("Generating video file asset 'generated/running_tracks.mp4'...")
    try:
        # Save a high quality mp4 file representation
        ani.save('generated/running_tracks.mp4', writer='ffmpeg', fps=5)
        print("Video successfully generated!")
    except Exception as e:
        print(f"Could not save video via ffmpeg. Saving as GIF layout frame matrix instead... Details: {e}")
        try:
            ani.save('generated/running_tracks.gif', writer='pillow', fps=5)
            print("GIF fallback successfully created!")
        except Exception as gif_err:
            print(f"Fallback asset render failed: {gif_err}")

    plt.show()

if __name__ == '__main__':
    run_gui()
