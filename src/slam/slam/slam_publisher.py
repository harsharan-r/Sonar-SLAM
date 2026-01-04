import rclpy
import time
import math
import random
import numpy as np

from copy import deepcopy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import PointCloud, Imu
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from slam.ground_truth_marker import publish_ground_truth_path, publish_ground_truth_map
from slam.particle import Particle
from slam.cluster import Cluster



raw_accel = []
raw_vel = []
filter_accel = []
filter_vel = []
time_stamps = []
printed = False

class SlamPublisher(Node):

    def __init__(self):
        super().__init__('slam_publisher')

        # Start by publishing pre defined obstacles before declaring any time sensitive variables
        self.ground_truth_path_publisher = self.create_publisher(MarkerArray, "ground_truth_path", 10)
        self.ground_truth_map_publisher = self.create_publisher(MarkerArray, "ground_truth_map", 10)
        self.imu_pose_estimation = self.create_publisher(MarkerArray, "imu_pose", 10)
        self.sonar_scan_publisher = self.create_publisher(MarkerArray, "sonar_scan", 10)

        # publish ground truth markers
        # for i in range(20):
        #     publish_ground_truth_path(1, "base_link", self.ground_truth_path_publisher, self.get_clock())
        #     publish_ground_truth_map(1, "base_link", self.ground_truth_map_publisher, self.get_clock())
        #     time.sleep(1)

        # -- SLAM --
        self.num_of_particles = 50
        self.fused_pose = Pose()
        self.particles = [Particle(self.fused_pose, 1.0/self.num_of_particles) for _ in range(self.num_of_particles)]

        # --- Scan Processing
        self.clusters=[]

        # --- IMU Processing ---
        self.imu_pose = Pose()
        self.imu_pose_array = MarkerArray()
        self.imu_vel = {'x':0.0,'y':0.0,'z':0.0}
        self.imu_prev_time = -1

        # IMU zero velocity update variables
        self.imu_ZVU_counter = {'x': 0.0, 'y': 0.0}
        self.imu_ZVU_thres = 3.0
        self.imu_ZVU_counter_thres = 3


        self.sonar_subscription_ = self.create_subscription(
            PointCloud,
            '/occupancy_data',
            self.sonar_callback,
            10)
        
        self.imu_subscription_ = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10)
        

    def sonar_callback(self, msg):

        time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.imu_prev_time == -1:
            # this will lead the first message to be ignored 
            # but doesn't make a big difference due to the freq
            self.imu_prev_time = time

        dt = time - self.imu_prev_time
        self.imu_prev_time = time

        self.cluster_points(msg.points)

        for cluster in self.clusters:

            if cluster.length() <= 2:
                continue

            centroid = cluster.centroid()
            covariance = cluster.covariance() 
            
            for particle in self.particles:
                length = particle.update_map(cluster)
                
        # self.resample_particle_weight()
        best_particle = self.get_best_particle()

        sonar_scan = MarkerArray()
        sonar_point = Pose()

        sonar_point.orientation.x = 0.0
        sonar_point.orientation.y = 0.0
        sonar_point.orientation.z = 0.0
        sonar_point.orientation.w = 1.0

        colors = [
            {'r': 0.0,        'g': 0.0,        'b': 1.0},        # blue
            {'r': 0.0,        'g': 1.0,        'b': 0.0},        # green
            {'r': 1.0,        'g': 1.0,        'b': 0.0},        # yellow
            {'r': 1.0,        'g': 0.6471,     'b': 0.0},        # orange
            {'r': 0.5020,     'g': 0.0,        'b': 0.5020},     # purple
            {'r': 0.0,        'g': 1.0,        'b': 1.0},        # cyan
            {'r': 1.0,        'g': 0.7529,     'b': 0.7961},     # pink
            {'r': 0.5020,     'g': 0.5020,     'b': 0.5020},     # gray
            {'r': 0.0,        'g': 0.0,        'b': 0.0},        # black
        ]


        for num, landmark in enumerate(best_particle.map):
            for index, point in enumerate(landmark['points']):
                sonar_point.position.x = point['x']
                sonar_point.position.y = point['y']
                sonar_point.position.z = 0.0
                sonar_scan.markers.append(self.marker_from_pose(sonar_point, msg.header, len(sonar_scan.markers), colors[num%len(colors)], 0.005))

        self.sonar_scan_publisher.publish(sonar_scan)

        self.imu_pose.position.x = best_particle.pose['x']
        self.imu_pose.position.y = best_particle.pose['y']

        # self.get_logger().info(f"Local Pose {self.imu_pose.position.x}, {self.imu_pose.position.y}\n" f"Particle Pose {}, {best_particle.pose['y']}")


    def imu_callback(self, msg):
        time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.imu_prev_time == -1:
            # this will lead the first message to be ignored 
            # but doesn't make a big difference due to the freq
            self.imu_prev_time = time

        dt = time - self.imu_prev_time
        self.imu_prev_time = time

        self.update_imu_raw_motion(msg, dt, True)
        self.particle_motion_update(dt)

    def particle_motion_update(self, dt):
        x = self.imu_pose.position.x
        y = self.imu_pose.position.y
        quat = self.imu_pose.orientation

        for p in self.particles:
            p.motion_update(x, y, quat, dt)

    def get_best_particle(self):
        return max(self.particles, key=lambda p: p.weight)

    def resample_particle_weight(self):
        weights = [p.weight for p in self.particles]
        
        effective_sample_size = 1/sum(w**2 for w in weights)

        if effective_sample_size < self.num_of_particles/2:
            new_particles = random.choices(self.particles, weights=weights, k=self.num_of_particles)

            # Optionally, deep-copy so they are independent
            self.particles = [deepcopy(p) for p in new_particles]

            # Reset all weights to uniform
            for p in self.particles:
                p.weight = 1.0 / self.num_of_particles

    def cluster_points(self, points, dist_thresh = 0.1):
        clusters = []

        for pt in points:
            added = False
            for c in clusters:
                centroid = c.centroid()
                dx = pt.x - centroid[0]
                dy = pt.y - centroid[1]
                if (dx*dx + dy*dy)**0.5 < dist_thresh:
                    c.add_point(pt)
                    added = True
                    break
            if not added:
                clusters.append(Cluster(pt))
        
        self.clusters = clusters

    def update_imu_raw_motion(self, msg, dt, verbose=False):
        quat = msg.orientation
        roll, pitch, yaw  = self.quat_to_euler(quat.x, quat.y, quat.z, quat.w)

        # Account for sensor oreintation and global plane
        ax_raw = msg.linear_acceleration.y
        ay_raw = msg.linear_acceleration.x

        # Integrate velocity
        self.imu_vel['x'] += ax_raw * dt
        self.imu_vel['y'] += ay_raw * dt

        # Update ZVU counter
        if abs(ax_raw) < self.imu_ZVU_thres:
            self.imu_ZVU_counter['x']+=1
        else:
            self.imu_ZVU_counter['x']=0

        if abs(ay_raw) < self.imu_ZVU_thres:
            self.imu_ZVU_counter['y']+=1
        else:
            self.imu_ZVU_counter['y']=0

        # ZVU to zero the velocity when accel has settled
        if self.imu_ZVU_counter['x'] > self.imu_ZVU_counter_thres:
            self.imu_vel['x'] = 0
        if self.imu_ZVU_counter['y'] > self.imu_ZVU_counter_thres:
            self.imu_vel['y'] = 0

        # Integrate position and account for sensor alignment with global axis 
        x_dis = self.imu_vel['x'] * dt
        y_dis = self.imu_vel['y'] * dt

        # Transforming from local to global
        self.imu_pose.position.x += (x_dis * math.cos(-yaw)) + (y_dis * math.sin(-yaw))
        self.imu_pose.position.y += (x_dis * math.sin(-yaw)) + (y_dis * math.cos(-yaw))
        
        # Orientation stays from IMU quaternion
        self.imu_pose.orientation = msg.orientation

        # LOGGING — placed at the end so you see final states
        if verbose:
            self.get_logger().info(
                f"\n--- IMU Debug ---\n"
                f"Raw Accel: ax={ax_raw:.3f}, ay={ay_raw:.3f}\n"
                f"Velocity:  vx={self.imu_vel['x']:.3f}, vy={self.imu_vel['y']:.3f}\n"
                f"Yaw:  {yaw:.3f} rad\n"
                f"Position:  x={self.imu_pose.position.x:.3f}, y={self.imu_pose.position.y:.3f}\n"
            )
            self.imu_pose_array.markers.append(self.marker_from_pose(self.imu_pose, msg.header, len(self.imu_pose_array.markers), {'r': 0, 'g': 0, 'b': 255}))
            self.imu_pose_estimation.publish(self.imu_pose_array)

    def marker_from_pose(self, pose, header, id, color, scale=0.01, m_type=2):
        marker = Marker()
        marker.header = header

        marker.ns = "Ground Truth Path"
        marker.id = id
        marker.type = m_type
        marker.action = Marker.ADD

        # print(pose)

        marker.pose.position.x = pose.position.x
        marker.pose.position.y = pose.position.y
        marker.pose.position.z = pose.position.z
        marker.pose.orientation.x = pose.orientation.x
        marker.pose.orientation.y = pose.orientation.y
        marker.pose.orientation.z = pose.orientation.z
        marker.pose.orientation.w = pose.orientation.w

        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        marker.color.r = color['r']
        marker.color.g = color['g']
        marker.color.b = color['b']
        marker.color.a = 1.0              # MUST be > 0

        marker.lifetime.sec = 0           # auto-refresh

        return marker

    def quat_to_euler(self, x, y, z, w):
        roll = math.atan2(
            2.0 * (w*x + y*z),
            1.0 - 2.0 * (x*x + y*y)
        )

        pitch = math.asin(
            max(-1.0, min(1.0, 2.0 * (w*y - z*x)))
        )

        yaw = math.atan2(
            2.0 * (w*z + x*y),
            1.0 - 2.0 * (y*y + z*z)
        )

        return roll, pitch, yaw

    def ema_filter(self, value, prev_value, alpha):
        return value*alpha + (1-alpha)*prev_value

    def median_filter(self, value, prev_values, threshold):
        values = prev_values
        values.append(value)

        # if window not filled, output raw value for debugging
        if len(values) < threshold:
            return value, values  

        # Once threshold hit
        if len(values) > threshold:
            values.pop(0)

        median_value = sorted(values)[threshold // 2]
        return median_value, values

def main(args=None):
    rclpy.init(args=args)

    slam_publisher = SlamPublisher()

    rclpy.spin(slam_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    slam_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()