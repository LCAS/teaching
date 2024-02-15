import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np

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
        self.duration = 10
        self.time_utilized = 0.0
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
        

    def perform_action_diff_drive_one_step(self):
        if(self.robot_current_pose is not None):
            if(self.duration < self.time_utilized):
                self.stop_vehicle()
                self.get_logger().info(f'End of simulation', once=True)
                self.end_controller = True
                return
            
            #TODO Estimate the desired robot linear and angular velocity
            # v = 
            # w = 
            #TODO Estimate robot dynamics, the robot current pose (x_{t}) can be obtained by self.robot_current_pose
            # dq = 
            #TODO Update the self.robot_current_pose, i.e., x_{t+1} = x_{t} + self.Ts*dq
            #TODO Normalize the robot orientation [-pi, pi], you may use the self.wrap_to_pi(self.robot_current_pose[2])
            #TODO Send the control commands to via self.send_vel(v, w)

            self.time_utilized  =  self.time_utilized + self.Ts

    def set_pose(self, msg):
        # Setting the current pose of the robot 
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        if(self.set_robot_init_pose is None):
            self.set_robot_init_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])
            self.robot_current_pose = self.set_robot_init_pose

    def send_vel(self, v, w):
        # TODO Control the robot's movement by specifying its linear and angular velocities 
        # on the /cmd_vel (self.control_pub) topic
        pass 

    def timer_callback(self, ):
        # Use this timer callback to send the commands to robot
        self.perform_action_diff_drive_one_step()
        return 

def main(args=None):
    rclpy.init(args=args)
    control_strategy = ControlStrategy(delta_t=0.03)
    # TODO Utilize the control_strategy.diff_drive_init function to set the angular velocities
    # of the left and right side wheels of the robot
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
