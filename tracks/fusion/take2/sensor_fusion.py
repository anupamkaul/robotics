import numpy as np

def latlon_to_meters(lat, lon, ref_lat, ref_lon):
    # Quick equirectangular projection approximation for local tracking
    R = 6371000.0 # Earth radius
    dlat = np.radians(lat - ref_lat)
    dlon = np.radians(lon - ref_lon)
    y = dlat * R
    x = dlon * R * np.cos(np.radians(ref_lat))
    return x, y

class MaritimeEKF:
    def __init__(self, track_id, initial_x, initial_y, q_acc=0.5):
        self.track_id = track_id
        self.x = np.array([[initial_x], [initial_y], [0.0], [0.0]], dtype=float)
        self.P = np.diag([50.0, 50.0, 5.0, 5.0])
        self.q_acc = q_acc
        self.missed_frames = 0
        self.is_confirmed = False
        self.hit_count = 1

    def predict(self, dt, q_acc=None):
        if q_acc is not None:
            self.q_acc = q_acc
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
        ]) * self.q_acc
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update_radar(self, r_meas, theta_meas, r_noise, theta_noise, gate_size=15.0):
        px, py = self.x[0, 0], self.x[1, 0]
        r_pred = np.sqrt(px**2 + py**2)
        if r_pred < 1e-3: return False
        theta_pred = np.arctan2(py, px)
        
        z = np.array([[r_meas], [theta_meas]])
        hx = np.array([[r_pred], [theta_pred]])
        y = z - hx
        y[1, 0] = (y[1, 0] + np.pi) % (2 * np.pi) - np.pi
        
        H = np.array([
            [px / r_pred, py / r_pred, 0.0, 0.0],
            [-py / (r_pred**2), px / (r_pred**2), 0.0, 0.0]
        ])
        R = np.diag([r_noise**2, theta_noise**2])
        S = H @ self.P @ H.T + R
        
        try:
            S_inv = np.linalg.inv(S)
            mahalanobis = float(y.T @ S_inv @ y)
            if mahalanobis > gate_size:
                return False
            K = self.P @ H.T @ S_inv
            self.x = self.x + K @ y
            self.P = (np.eye(4) - K @ H) @ self.P
            self.missed_frames = 0
            self.hit_count += 1
            if self.hit_count >= 3:
                self.is_confirmed = True
            return True
        except np.linalg.LinAlgError:
            return False

    def update_camera(self, bearing_meas, bearing_noise, gate_size=9.0):
        px, py = self.x[0, 0], self.x[1, 0]
        denom = px**2 + py**2
        if denom < 1e-3: return False
        bearing_pred = np.arctan2(py, px)
        
        y = np.array([[bearing_meas - bearing_pred]])
        y[0, 0] = (y[0, 0] + np.pi) % (2 * np.pi) - np.pi
        
        H = np.array([[-py / denom, px / denom, 0.0, 0.0]])
        R = np.array([[bearing_noise**2]])
        S = H @ self.P @ H.T + R
        
        try:
            S_inv = np.linalg.inv(S)
            mahalanobis = float(y.T @ S_inv @ y)
            if mahalanobis > gate_size:
                return False
            K = self.P @ H.T @ S_inv
            self.x = self.x + K @ y
            self.P = (np.eye(4) - K @ H) @ self.P
            self.missed_frames = 0
            self.hit_count += 1
            if self.hit_count >= 3:
                self.is_confirmed = True
            return True
        except np.linalg.LinAlgError:
            return False
