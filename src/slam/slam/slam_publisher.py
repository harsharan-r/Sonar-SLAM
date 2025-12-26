import rclpy
import time

from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import PointCloud, Imu
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from slam.ground_truth_marker import publish_ground_truth_path, publish_ground_truth_map
import math

import matplotlib.pyplot as plt

import os
style_path = os.path.join(os.path.dirname(__file__), "..", "figures","themes", "rose-pine.mplstyle")
plt.style.use("/app/figures/themes/rose-pine.mplstyle")

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

        # publish ground truth markers
        for i in range(20):
            publish_ground_truth_path(1, "base_link", self.ground_truth_path_publisher, self.get_clock())
            publish_ground_truth_map(1, "base_link", self.ground_truth_map_publisher, self.get_clock())
            time.sleep(1)

        
        self.fused_pose = Pose()

        self.imu_pose = Pose()
        self.imu_pose_array = MarkerArray()
        self.imu_filter_pose = Pose()
        self.imu_vel = {'x':0.0,'y':0.0,'z':0.0}
        self.imu_filter_vel = {'x':0.0,'y':0.0,'z':0.0}

        # IMU zero velocity update variables
        self.imu_ZVU_counter = {'x': 0.0, 'y': 0.0}
        self.imu_ZVU_thres = 0.3
        self.imu_ZVU_counter_thres = 3

        # story direction with 1 and -1
        self.imu_direction = 0

        # IMU EMA filter variables
        self.imu_prev_accel = 0
        self.imu_ema_alpha = 0.5

        self.imu_prev_time = -1
        self.ref = -1

        self.sonar_pose = Pose()

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
        


    def imu_callback(self, msg):
        time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.imu_prev_time == -1:
            # this will lead the first message to be ignored 
            # but doesn't make a big difference due to the freq
            self.imu_prev_time = time

        dt = time - self.imu_prev_time
        self.imu_prev_time = time

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
        # self.get_logger().info(
        #     f"\n--- IMU Debug ---\n"
        #     f"Raw Accel: ax={ax_raw:.3f}, ay={ay_raw:.3f}\n"
        #     f"Velocity:  vx={self.imu_vel['x']:.3f}, vy={self.imu_vel['y']:.3f}\n"
        #     f"Yaw:  {yaw:.3f} rad\n"
        #     f"Position:  x={self.imu_pose.position.x:.3f}, y={self.imu_pose.position.y:.3f}\n"
        # )
        
        self.imu_pose_array.markers.append(self.marker_from_pose(self.imu_pose, msg.header, len(self.imu_pose_array.markers)))
        self.imu_pose_estimation.publish(self.imu_pose_array)


    def sonar_callback(self, msg):
        # self.get_logger().info(f"points: {point_data}")
        self.get_logger().info("\n--- Point Cloud Message Recieved ---\n")

        point_data = [[point.x, point.y, point.z] for point in msg.points]

    def marker_from_pose(self, pose, header, id):
        marker = Marker()
        marker.header = header

        marker.ns = "Ground Truth Path"
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # print(pose)

        marker.pose.position.x = pose.position.x
        marker.pose.position.y = pose.position.y
        marker.pose.position.z = pose.position.z
        marker.pose.orientation.x = pose.orientation.x
        marker.pose.orientation.y = pose.orientation.y
        marker.pose.orientation.z = pose.orientation.z
        marker.pose.orientation.w = pose.orientation.w

        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        marker.color.r = id/1e3
        marker.color.g = id/5e2
        marker.color.b = 0.6
        marker.color.a = 1.0              # MUST be > 0

        marker.lifetime.sec = 20           # auto-refresh

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