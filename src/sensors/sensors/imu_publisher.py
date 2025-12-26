import rclpy
import RPi.GPIO as GPIO
import math

from time import sleep, perf_counter
from rclpy.node import Node

from sensor_msgs.msg import Imu

from sensors.sensor_drivers.mpu6050 import MPU

import os

DEG_RAD_CONV_FACTOR = math.pi/180

class IMUPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        self.imu_publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.08, self.publish_imu)

        ##### Sensor Setup #####
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        mpu_bus_addr = os.getenv("MPU_BUS_ADR") or "0x68"
        mpu_bus_addr = int(mpu_bus_addr, 16)
        self.imu = MPU(mpu_bus_addr)

        self.get_logger().info("Calibrating IMU ...")
        
        self.imu.reset()
        sleep(2)

        self.imu.set_accel_range(4)
        self.imu.set_gyro_range(500)

        self.imu.calibrate_accel()
        self.imu.calibrate_gyro()

        sleep(2)

        self.get_logger().info("Finished calibrating IMU ...")

        self.imu_prev_time = perf_counter()
        self.previous_yaw = 0
        self.previous_raw_yaw = 0
        self.previous_gz = 0
        self.prev_orien = {'roll':0.0, 'pitch':0.0, 'yaw':0.0}
    
    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        lin_accel, ang_vel, orientation = self.get_imu_data()
        qx, qy, qz, qw = self.euler_to_quaternion(
            orientation['roll'], 
            orientation['pitch'], 
            orientation['yaw']
        )

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.linear_acceleration.x = lin_accel['x']
        msg.linear_acceleration.y = lin_accel['y']
        msg.linear_acceleration.z = lin_accel['z']
        
        msg.angular_velocity.x = ang_vel['x']
        msg.angular_velocity.y = ang_vel['y']
        msg.angular_velocity.z = ang_vel['z']

        # self.get_logger().info(
        #     f"Roll: {orientation['roll']}\n"
        #     f"Pitch: {orientation['pitch']}\n"
        #     f"Z yaw: {orientation['yaw']}\n"
        # )

        self.imu_publisher_.publish(msg)
                

    def get_imu_data(self):
        # Constants for filters
        YAW_EMA_ALPHA = 0.5
        ACCEL_WEIGHT = 0.0
        G_DEADBAND = 1.0
        A_DEADBAND = 0.01

        # Get data from the mpu sensor
        a_val = self.imu.get_accel_data(calibrate=True)
        g_val = self.imu.get_gyro_data(calibrate=True)

        # Remove excess noise with a deadband
        g_deadzone_val = {k: self.deadband(v,G_DEADBAND) for k, v in g_val.items()}
        a_deadzone_val = {k: self.deadband(v,A_DEADBAND) for k, v in a_val.items()}

        # Get change in time from last measurement
        t_now = perf_counter()
        dt = t_now - self.imu_prev_time
        self.imu_prev_time = t_now

        # Calculate Roll Pitch and Yaw to get orientation
        roll, pitch = self.rp_complementary_filter(
            a_val, g_deadzone_val, self.prev_orien, dt, ACCEL_WEIGHT
        )

        gz_filtered = self.ema_filter(g_deadzone_val['z'], self.previous_gz, YAW_EMA_ALPHA)
        yaw = self.previous_yaw + gz_filtered * dt 

        self.previous_yaw = yaw
        self.previous_gz = gz_filtered

        orientation = {'roll':self.deg_to_rad(roll), 'pitch':self.deg_to_rad(pitch), 'yaw':self.deg_to_rad(yaw)}
        self.prev_orien = orientation

        # Remove gravity from value assuming device is always upright
        a_deadzone_val['z'] += self.imu.GRAVITY_MS2 

        return a_deadzone_val, g_deadzone_val, orientation
    
    def ema_filter(self, value, prev_value, alpha):
        return value*alpha + (1-alpha)*prev_value
    
    def rp_complementary_filter(self, a_val, g_val, prev_orien, dt, a_weight):
        a_roll = math.atan2(a_val['y'], -a_val['z'])
        a_pitch = math.atan2(a_val['x'], -a_val['z'])

        g_roll = prev_orien['roll'] + g_val['x'] * dt
        g_pitch = prev_orien['pitch'] + g_val['y'] * dt

        roll = a_roll*a_weight + g_roll*(1-a_weight)
        pitch = a_pitch*a_weight + g_pitch*(1-a_weight)

        return roll, pitch

    def deadband(self, value, threshold):
        return value if abs(value) > threshold else 0.0

    def deg_to_rad(self, angle):
        return angle * DEG_RAD_CONV_FACTOR


    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    imu_publisher_node = IMUPublisher()

    try:
        rclpy.spin(imu_publisher_node)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        imu_publisher_node.destroy_node()
        rclpy.shutdown()
        
    finally: 
            GPIO.cleanup()