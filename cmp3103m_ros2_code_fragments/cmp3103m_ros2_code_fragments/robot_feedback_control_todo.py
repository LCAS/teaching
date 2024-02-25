import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time  
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
import std_msgs.msg as std_msg
import rclpy.qos as qos


class ControlStrategy(Node):
    def __init__(self, delta_t, ):
        super().__init__('control_strategy')
        self.control_pub = self.create_publisher(Twist, '/cmd_vel', 30)
        self.control_sub = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.set_pose, 20)
              
        self.i = 0
        self.set_robot_init_pose = None
        self.robot_current_pose = None 
        self.robot_current_control = []
        self.r = 0.3 # Wheel radius
        self.L = 1.25 # Axle length
        self.D = 0.07 # Distance between the front front wheel and rear axle
        self.Ts = delta_t # Sampling time
        self.t = np.arange(0, 10, self.Ts) # Simulation time
        self.end_controller = False
        self.timer = self.create_timer(self.Ts, self.timer_callback)
        
              
    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def listener_callback(self, msg):
        self.robot_current_control = [msg.linear.x, msg.angular.z]

    def stop_vehicle(self, ):
        self.send_vel(0.0, 0.0)    
    
    def wrap_to_pi(self, x):
        x = np.array([x])
        xwrap = np.remainder(x, 2*np.pi)
        mask = np.abs(xwrap)>np.pi
        xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])
        return xwrap[0]
    
    def diff_drive_init(self, left_wheel_v, right_wheel_v, duration=5):
        self.duration = duration
        self.wL = left_wheel_v # Left wheel velocity
        self.wR = right_wheel_v # Right wheel velocity
        self.time_utilized = 0.0
        
    def inter_point_diff_drive_init(self, duration=10, r_distance=0.5
                    , refPose=np.array([1,1,0]), k_p=0.5, k_w=0.7, dmin=0.7):
        
        self.duration = duration
        self.r_distance = r_distance
        self.refPose = refPose
        self.k_p = k_p 
        self.k_w = k_w
        self.dmin = dmin
        self.time_utilized = 0.0
        self.xT = self.refPose[0]  - self.r_distance*np.cos(self.refPose[2])
        self.yT = self.refPose[1]  - self.r_distance*np.sin(self.refPose[2])
        self.state = 0
        
    def inter_direction_diff_drive_init(self, duration=10, r_distance=1.3
                    , refPose=np.array([1.5,1.5,np.pi/2]), k_p=0.5, k_w=0.7, dmin=0.7):
        self.duration = duration
        self.r_distance = r_distance
        self.refPose = refPose
        self.k_p = k_p 
        self.k_w = k_w
        self.dmin = dmin
        self.time_utilized = 0.0
    
    def inter_direction_diff_drive(self, ):
        if(self.robot_current_pose is not None):
            if(self.duration < self.time_utilized):
                self.get_logger().info(f'End of simulation', once=True)
                self.stop_vehicle()
                self.end_controller = True
                return 

            # TODO Estimate the distance (self.D) between current pose (self.robot_current_pose) #
            # and reference pose (self.refPose) 
            # self.D =

            if(self.D < self.dmin):
                self.get_logger().info(f'Reach to the goal pose', once=True)
                self.stop_vehicle()
                self.end_controller = True
                return

            # TODO Estimate orientation residual (self.ePhi) 
                
            v = self.k_p*self.D
            w = self.k_w*self.ePhi
            print("Distance to the goal: ", self.D)
            dq = np.array([v*np.cos(self.robot_current_pose_real[2]+self.Ts*w/2)
                            , v*np.sin(self.robot_current_pose_real[2]+self.Ts*w/2), w])
            self.robot_current_pose = self.robot_current_pose + self.Ts*dq # Integration
            self.robot_current_pose[2] = self.wrap_to_pi(self.robot_current_pose[2]) # Map orientation angle to [-pi, pi]
            self.send_vel(v, w)
            self.time_utilized  =  self.time_utilized + self.Ts    
        
    def inter_point_diff_drive(self, ):
        if(self.robot_current_pose is not None):
            if(self.duration < self.time_utilized):
                self.stop_vehicle()
                self.get_logger().info(f'End of simulation', once=True)
                self.end_controller = True
                return

            # TODO Estimate the distance (self.D) between current pose (self.robot_current_pose) #
            # and reference pose (self.refPose) 
            # self.D = 

            if(self.D < self.dmin):
                self.stop_vehicle()
                self.get_logger().info(f'Reach to the goal pose', once=True)
                self.end_controller = True
                return

            # TODO Estimate orientation residual (self.ePhi) 
            
            v = self.k_p*self.D
            w = self.k_w*self.ePhi
            print("Distance to the goal: ", self.D)
            dq = np.array([v*np.cos(self.robot_current_pose_real[2]+self.Ts*w/2), v*np.sin(self.robot_current_pose_real[2]+self.Ts*w/2), w])
            self.robot_current_pose = self.robot_current_pose + self.Ts*dq # Integration
            self.robot_current_pose[2] = self.wrap_to_pi(self.robot_current_pose[2]) # Map orientation angle to [-pi, pi]
            self.send_vel(v, w)
            self.time_utilized  =  self.time_utilized + self.Ts 

    def perform_action_diff_drive_one_step(self):
        if(self.robot_current_pose is not None):
            if(self.duration < self.time_utilized):
                self.stop_vehicle()
                self.get_logger().info(f'End of simulation', once=True)
                self.end_controller = True
                return
            v = self.r/2*(self.wR+self.wL) # Robot velocity
            w = self.r/self.L*(self.wR-self.wL) # Robot angular velocity
            dq = np.array([v*np.cos(self.robot_current_pose[2]+self.Ts*w/2), v*np.sin(self.robot_current_pose[2]+self.Ts*w/2), w])
            self.robot_current_pose = self.robot_current_pose + self.Ts*dq # Integration
            self.robot_current_pose[2] = self.wrap_to_pi(self.robot_current_pose[2]) # Map orientation angle to [-pi, pi]
            self.send_vel(v, w)
            self.time_utilized  =  self.time_utilized + self.Ts

    def set_pose(self, msg):
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        if(self.set_robot_init_pose is None):
            self.set_robot_init_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])
            self.robot_current_pose = self.set_robot_init_pose
        self.robot_current_pose_real = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])

    def send_vel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.control_pub.publish(msg)

    def timer_callback(self, ):
        # self.perform_action_diff_drive_one_step()
        self.inter_point_diff_drive()
        # self.inter_direction_diff_drive()
        return 

def main(args=None):
    rclpy.init(args=args)
    control_strategy = ControlStrategy(delta_t=0.03)
    # control_strategy.diff_drive_init(0.2, 0.2, duration=20)
    # control_strategy.inter_direction_diff_drive_init(r_distance=0.6
    #                 , refPose=np.array([1.0,1.0,0]), k_p=0.1, k_w=0.5, dmin=0.7)
    control_strategy.inter_point_diff_drive_init(r_distance=0.5
                    , refPose=np.array([1,1,0]), k_p=0.1, k_w=0.5, dmin=0.7)
        
    while rclpy.ok():
        try:
            rclpy.spin_once(control_strategy)
        except KeyboardInterrupt:
            break
    control_strategy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# To reset ros2 service call /reset_simulation std_srvs/srv/Empty
