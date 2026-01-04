import random
import math
import numpy as np

from geometry_msgs.msg import Pose

class Particle:
    def __init__(self, init_pose, init_weight):
        self.pose = {
            'x': init_pose.position.x,
            'y': init_pose.position.y,
            'yaw': self.quat_to_yaw(init_pose.orientation)
        }

        self.pose_history = [self.pose.copy()]
        self.map = []
        self.weight = init_weight

    def motion_update(self, x, y, quat, dt):
        # Add noise and update yaw
        yaw = self.quat_to_yaw(quat)
        self.pose['yaw'] = yaw + random.gauss(0, 0.02)

        # Add noise and calculation 2d pose
        x_hat = x + random.gauss(0, 0.01)
        y_hat = y + random.gauss(0, 0.01)

        self.pose['x'] = x_hat
        self.pose['y'] = y_hat

        # Save history snapshot
        self.pose_history.append(self.pose.copy())

    def quat_to_yaw(self, orien):
        yaw = math.atan2(
            2.0 * (orien.w*orien.z + orien.x*orien.y),
            1.0 - 2.0 * (orien.y*orien.y + orien.z*orien.z)
        )

        return yaw

    def update_map(self, c, dist_thresh=0.1):
        found = False

        x = self.pose['x']
        y = self.pose['y']

        centroid = c.centroid(x,y)
        covariance = c.covariance(x,y)

        for cluster in self.map:
            dx = centroid[0] - cluster['mu'][0]
            dy = centroid[1] - cluster['mu'][1]

            dist = np.sqrt(dx**2 + dy**2)

            if dist < dist_thresh:
                cluster['mu'], cluster['Sigma'], likelihood = self.ekf_landmark_update(cluster['mu'],cluster['Sigma'],centroid,covariance)
                cluster['points'].extend(c.points(x,y))
                self.weight *= likelihood
                found = True
                break
            
        if not found:
            self.map.append({'mu':centroid, 'Sigma':covariance, 'points':c.points(x,y)})
        
        return len(self.map)

    def ekf_landmark_update(self,mu, Sigma, z, R):
        innovation = z - mu

        EPS = 1e-6
        S = Sigma + R + EPS * np.eye(2)

        # S = Sigma + R
        K = Sigma @ np.linalg.inv(S)

        mu_new = mu + K @ innovation
        Sigma_new = (np.eye(2) - K) @ Sigma

        # compute likelihood contribution
        likelihood = np.exp(-0.5 * innovation.T @ np.linalg.inv(S) @ innovation)
        likelihood /= np.sqrt((2*np.pi)**2 * np.linalg.det(S))
        
        return mu_new, Sigma_new, likelihood

        return mu_new, Sigma_new

