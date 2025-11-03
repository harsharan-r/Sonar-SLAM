import rclpy
import RPi.GPIO as GPIO
import threading
import math

from time import sleep, perf_counter

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32

from sensors.sensor_drivers.ultrasonic import Ultrasonic
from sensors.sensor_drivers.stepper_motor import StepperMotor

ULTRASONIC_SEN_OFFSET = 0.3745

class SonarPublisher(Node):

    def __init__(self, ultrasonic_sen_offset):
        super().__init__('sonar_publisher')
        self.points_publisher_ = self.create_publisher(PointCloud, '/occupancy_data', 10)

        ##### Sensor Setup #####
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
    
        # Ultrasonic sensor
        self.inline_sen = Ultrasonic(27,22)
        self.perp_sen = Ultrasonic(26,19)
        self.dist_array = []
        self.ultrasonic_sen_offset = ultrasonic_sen_offset

        # Stepper motor
        self.motor = StepperMotor(21, 20, 16, 12)
        self.motor_min_angle = -90 # Degrees
        self.motor_max_angle = 0
        self.motor_target = self.motor_min_angle
        self.motor_speed = 30 # RPM
        self.sweep_completed = False

        ##### Timers #####
        self.record_distance = self.create_timer(0.07, self.record_distance_callback)    
        
        ##### Multithreading #####
        self.motor_sweep_thread = None
        self.stop_event = threading.Event()
        self.record_event = threading.Event()


    def asyncMotorSweep(self, motor, min_angle, max_angle, rotation_speed, stop_event, record_values):

        # Start motor at min angle position
        motor.move_to(min_angle, rotation_speed)
        target_angle = min_angle

        # sweep between angles
        while not stop_event.is_set():
            
            # move to target
            motor.move_to(target_angle, rotation_speed)

            # switch target and record values
            target_angle = min_angle if target_angle == max_angle else max_angle
            record_values.set()

            # delay a small amount a for smooth transition
            sleep(0.12)

    def record_distance_callback(self):

        # Start motor thread if it hasn't started
        if self.motor_sweep_thread == None:
            self.motor_sweep_thread = threading.Thread(
                target=self.asyncMotorSweep,
                args=(
                    self.motor,
                    self.motor_min_angle,
                    self.motor_max_angle,
                    self.motor_speed,
                    self.stop_event,
                    self.record_event
                )
            )
            self.motor_sweep_thread.start()

        # Record ultrasonic data
        self.dist_array.append([self.motor.position, self.inline_sen.filtered_distance() + self.ULTRASONIC_SEN_OFFSET])
        self.dist_array.append([self.motor.position+90, self.perp_sen.filtered_distance() + self.ULTRASONIC_SEN_OFFSET])

        if self.record_event.is_set():
            
            # Sort Data by the angle
            self.dist_array.sort(key=lambda x: x[0])

            # Publish the data
            self.publish_cloud(self.dist_array)

            # Clear data for next sweep
            self.dist_array.clear()

            # Reset the event to record data
            self.record_event.clear()

    def publish_cloud(self, polar_array):
        
        # Initialise a point cloud message
        msg = PointCloud()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "sonar"

        # Convert from polar to cartesian points
        msg.points = [
            Point32(x=r*math.cos(theta), y=r*math.sin(theta), z=0.0)
            for r, theta in polar_array
        ]
        
        # Publisher message
        msg = PointCloud()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "sonar"

        # Convert from polar to cartesian points
        msg.points = [
            Point32(x=r*math.cos(theta), y=r*math.sin(theta), z=0.0)
            for r, theta in polar_array
        ]
        
        # Publisher message
        self.points_publisher_.publish(msg)
        self.get_logger().info(f"Published polar cloud with {len(msg.points)} points")

def main(args=None):

    

    rclpy.init(args=args)
    sonar_publisher = SonarPublisher(ULTRASONIC_SEN_OFFSET)

    try:
        rclpy.spin(sonar_publisher)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        sonar_publisher.destroy_node()
        rclpy.shutdown()
        
    finally: 
            GPIO.cleanup()


