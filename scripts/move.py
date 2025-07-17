#!/usr/bin/env python3
# Robot moves in a 1x1m square

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry 
from acs6121_team24_2025.msg import DecisionState

from acs6121_team24_2025_modules.tb3_tools import quaternion_to_euler
from math import sqrt, pow, pi 

class Move(Node):

    def __init__(self):
        super().__init__("move")

        self.vel_msg = Twist() 

        self.first_message = False        
        self.x = 0.0; self.y = 0.0; self.theta_z = 0.0
        self.xref = 0.0; self.yref = 0.0; self.theta_zref = 0.0
        self.yaw = 0.0 
        self.displacement = 0.0 

        self.decision = 0   # Initial decision, can be changed to 1
        self.angular_error = 1

        self.counter = 0        # Counter for stay in turning motion around edge


        self.vel_pub = self.create_publisher(
            msg_type=Twist,
            topic="cmd_vel",
            qos_profile=10,
        )

        self.odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic="odom",
            callback=self.odom_callback,
            qos_profile=10,
        )

        self.dec_sub = self.create_subscription(
            msg_type=DecisionState,
            topic="decision_state",
            callback=self.dec_callback,
            qos_profile=10,
        )

        ctrl_rate = 10 # hz
        self.timer = self.create_timer(
            timer_period_sec=1/ctrl_rate,
            callback=self.timer_callback,
        )

        self.shutdown = False

        self.get_logger().info(
            f"The '{self.get_name()}' node is initialised."
        )

    def on_shutdown(self):
        print("Stopping the robot...")
        self.vel_pub.publish(Twist())
        self.shutdown = True

    def odom_callback(self, msg_data: Odometry):
        pose = msg_data.pose.pose 

        (roll, pitch, yaw) = quaternion_to_euler(pose.orientation) 

        self.x = pose.position.x 
        self.y = pose.position.y
        self.theta_z = yaw # abs(yaw) makes mistakes in detection!!

        if not self.first_message: 
            self.first_message = True
            self.xref = self.x
            self.yref = self.y
            self.theta_zref = self.theta_z

    def dec_callback(self, msg_data: DecisionState):
        self.decision = msg_data.decision_state
        self.angular_error = msg_data.angular_error
        self.correction = msg_data.correction

    def timer_callback(self):
        # here is where the code to control the motion of the robot 
        # goes.         
        max_speed = 0.26    # normal speed; best working version 0.2 and 0.1
        min_speed = 0.15      # slow speed when turning
        

        match self.decision:
            case 0:                         # stop
                self.vel_msg.angular.z = 0.0
                self.vel_msg.linear.x = 0.0
                
            case 1:                         # straight
                self.vel_msg.angular.z = 0.0    # max(-0.7, min(0.7, 3.0 * self.angular_error))
                self.vel_msg.linear.x = max_speed 

            case 2:                         # right
                self.vel_msg.angular.z = -0.8
                self.vel_msg.linear.x = 0.0
            
            case 3:                         # left
                self.vel_msg.angular.z = 0.8
                self.vel_msg.linear.x = 0.0
            
            case 4:                         # around wall edge
                self.vel_msg.angular.z = -0.65
                self.vel_msg.linear.x = 0.2

            case 5:
                self.vel_msg.angular.z = self.correction

                angular_abs = abs(self.correction)
                speed = max(min_speed, max_speed * (1 - angular_abs / 0.8))
                self.vel_msg.linear.x = speed
                
            case _:
                return "Something is wrong with the decision message"
        
        # self.get_logger().info(
        #     f"Current Decision state: {self.decision}",
        #     throttle_duration_sec = 1,
        # )
        # publish whatever velocity command has been set above:
        self.vel_pub.publish(self.vel_msg)

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    move = Move()
    try:
        rclpy.spin(move)
    except KeyboardInterrupt:
        print(
            f"{move.get_name()} received a shutdown request (Ctrl+C)."
        )
    finally:
        move.on_shutdown()
        while not move.shutdown:
            continue
        move.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()