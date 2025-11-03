import rclpy
import RPi.GPIO as GPIO
import math

from time import sleep, perf_counter
from rclpy.node import Node

from sensor_msgs.msg import Imu

from sensors.sensor_drivers.mpu6050 import MPU

class IMUPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        self.imu_publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.2, self.publish_imu)

        ##### Sensor Setup #####
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        mpu_bus_addr = 0x68
        self.imu = MPU(mpu_bus_addr)

        self.get_logger().info("Calibrating IMU ...")
        
        sleep(1)

        self.imu.calibrate_accel()
        self.imu.calibrate_gyro()

        sleep(1)

        self.get_logger().info("Finished calibrating IMU ...")

        self.imu_prev_time = perf_counter()
        self.previous_yaw = 0
    
    def publish_imu(self):
        
        if self.imu.data_ready():
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "imu_link"

            lin_accel, ang_accel, orientation = self.get_mpu_data()
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
            
            msg.angular_velocity.x = ang_accel['x']
            msg.angular_velocity.y = ang_accel['y']
            msg.angular_velocity.z = ang_accel['z']

            self.imu_publisher_.publish(msg)
            self.get_logger().info("Published IMU message")
            

    def get_mpu_data(self):
        a_val = self.imu.get_accel_data(calibrate=True)
        g_val = self.imu.get_gyro_data(calibrate=True)

        roll = math.atan2(a_val['y'], -a_val['z'])
        pitch = math.atan2(a_val['x'], -a_val['z'])

        t_now = perf_counter()
        dt = t_now - self.imu_prev_time
        self.imu_prev_time = t_now

        alpha = 0.97
        yaw = self.previous_yaw + g_val["z"] * dt
        yaw = alpha * yaw + (1 - alpha) * self.previous_yaw

        self.previous_yaw = yaw

        orientation = {'roll':roll, 'pitch':pitch, 'yaw':yaw}

        return a_val, g_val, orientation
    
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