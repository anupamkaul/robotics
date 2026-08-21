import numpy as np

class MaritimeExtendedKalmanFilter:
    def __init__(self, track_id, initial_state, initial_covariance=None):
        self.track_id = track_id
        self.x = np.array(initial_state, dtype=float).reshape(4, 1)
        if initial_covariance is not None:
            self.P = np.array(initial_covariance, dtype=float)
        else:
            self.P = np.diag([100.0, 100.0, 25.0, 25.0])
            
    def predict(self, dt, q_acc):
        F = np.array([
            [1.0, 0.0,  dt, 0.0],
            [0.0, 1.0, 0.0,  dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        Q = np.array([
            [dt4/4,   0.0, dt3/2,   0.0],
            [  0.0, dt4/4,   0.0, dt3/2],
            [dt3/2,   0.0,  dt2,   0.0],
            [  0.0, dt3/2,   0.0,  dt2]
        ]) * q_acc
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update_radar(self, z, R_base, gate_size):
        px, py, vx, vy = float(self.x[0,0]), float(self.x[1,0]), float(self.x[2,0]), float(self.x[3,0])
        rho = np.sqrt(px**2 + py**2)
        if rho < 1e-4: return False
        
        phi = np.arctan2(py, px)
        rho_dot = (px * vx + py * vy) / rho
        hx = np.array([[rho], [phi], [rho_dot]])
        
        H = np.zeros((3, 4))
        H[0, 0] = px / rho
        H[0, 1] = py / rho
        H[1, 0] = -py / (rho**2)
        H[1, 1] = px / (rho**2)
        H[2, 0] = (vx / rho) - (px * (px * vx + py * vy)) / (rho**3)
        H[2, 1] = (vy / rho) - (py * (px * vx + py * vy)) / (rho**3)
        H[2, 2] = px / rho
        H[2, 3] = py / rho
        
        y = z - hx
        y[1, 0] = (y[1, 0] + np.pi) % (2 * np.pi) - np.pi
        
        S = H @ self.P @ H.T + R_base
        try:
            S_inv = np.linalg.inv(S)
            mahalanobis_dist = float(y.T @ S_inv @ y)
        except np.linalg.LinAlgError:
            return False
            
        if mahalanobis_dist > gate_size:
            return False  
            
        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        return True

    def update_camera(self, bearing_z, R_base, gate_size, camera_bias=0.0):
        px, py = float(self.x[0,0]), float(self.x[1,0])
        denom = px**2 + py**2
        if denom < 1e-4: return False
        
        predicted_bearing = np.arctan2(py, px)
        hx = np.array([[predicted_bearing]])
        z = np.array([[bearing_z - camera_bias]])
        
        H = np.array([[-py / denom, px / denom, 0.0, 0.0]])
        y = z - hx
        y[0, 0] = (y[0, 0] + np.pi) % (2 * np.pi) - np.pi
        
        S = H @ self.P @ H.T + R_base
        try:
            S_inv = np.linalg.inv(S)
            mahalanobis_dist = float(y.T @ S_inv @ y)
        except np.linalg.LinAlgError:
            return False
            
        if mahalanobis_dist > gate_size:
            return False  
            
        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        return True

class GlobalTrackManager:
    def __init__(self):
        self.tracks = {}
        self.next_track_id = 1
        self.max_coast_time = 3.5   
        self.track_metadata = {} 
        
    def add_track(self, initial_state, current_time):
        tid = self.next_track_id
        self.tracks[tid] = MaritimeExtendedKalmanFilter(tid, initial_state)
        self.track_metadata[tid] = {'last_update': current_time, 'status': 'Confirmed'}
        self.next_track_id += 1
        return tid
        
    def predict_all(self, dt, q_acc):
        for tid, track in self.tracks.items():
            track.predict(dt, q_acc)

    def process_radar_measurements(self, measurements, current_time, R_base, gate_size, anti_sea_clutter=False):
        R_effective = R_base * 6.0 if anti_sea_clutter else R_base
        unassigned_meas = list(range(len(measurements)))
        
        for tid, track in list(self.tracks.items()):
            best_meas_idx = None
            min_dist = float('inf')
            
            for idx in unassigned_meas:
                z = measurements[idx]
                r_meas, b_meas = z[0, 0], z[1, 0]
                mx, my = r_meas * np.cos(b_meas), r_meas * np.sin(b_meas)
                dist = (mx - track.x[0,0])**2 + (my - track.x[1,0])**2
                if dist < min_dist and dist < 2500.0:
                    min_dist = dist
                    best_meas_idx = idx
                    
            if best_meas_idx is not None:
                if track.update_radar(measurements[best_meas_idx], R_effective, gate_size):
                    self.track_metadata[tid]['last_update'] = current_time
                    if best_meas_idx in unassigned_meas:
                        unassigned_meas.remove(best_meas_idx)
                    
        for idx in unassigned_meas:
            z = measurements[idx]
            r_meas, b_meas = z[0, 0], z[1, 0]
            mx, my = r_meas * np.cos(b_meas), r_meas * np.sin(b_meas)
            self.add_track([mx, my, 0, 0], current_time)

    def process_camera_measurements(self, bearings, current_time, R_base, gate_size, camera_bias=0.0, reset_active=False):
        if reset_active: return
        for bearing in bearings:
            best_tid = None
            min_innov = float('inf')
            for tid, track in self.tracks.items():
                tx, ty = track.x[0,0], track.x[1,0]
                innov = abs((bearing - camera_bias - np.arctan2(ty, tx) + np.pi) % (2 * np.pi) - np.pi)
                if innov < min_innov and innov < 0.3:
                    min_innov = innov
                    best_tid = tid
            if best_tid is not None:
                if self.tracks[best_tid].update_camera(bearing, R_base, gate_size, camera_bias):
                    self.track_metadata[best_tid]['last_update'] = current_time

    def manage_lifecycles(self, current_time):
        dead_tracks = [tid for tid, m in self.track_metadata.items() if current_time - m['last_update'] > self.max_coast_time]
        for tid in dead_tracks:
            del self.tracks[tid]
            del self.track_metadata[tid]
