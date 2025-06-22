#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from robot_interfaces.action import CountUntil
import time
from rclpy.action.client import ClientGoalHandle, GoalStatus

class CountUntilClientNode(Node):
    def __init__(self):
        super().__init__("count_until_client")
        self.count_until_client_ = ActionClient(self, CountUntil, "count_until")
        self.get_logger().info("Action Client has been started...")

    def send_goal(self, target_number, period):
        # wait for the server 
        self.count_until_client_.wait_for_server() #no value i/p as i want to wait indefinitely for the server 

        # create a goal 
        goal =  CountUntil.Goal()
        goal.target_number = target_number
        goal.period = period
        
        # send the goal log
        self.get_logger().info("Sending the goal!!!")
        self.count_until_client_.send_goal_async(goal, feedback_callback= self.goal_feedback_callback).add_done_callback(self.goal_response_callback)

        # Send a cancel request 2 sec later 
        #self.timer_ = self.create_timer(2.0, self.cancel_goal)

    def cancel_goal(self):
        self.get_logger().info("Send a cancel request")
        self.goal_handle_.cancel_goal_async()
        self.timer_.cancel()

    def goal_response_callback(self, future):
        self.goal_handle_ : ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal Accepted")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("Goal REJECTED")

    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("SUCCESS")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("ABORTED")  
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().error("CANCELLED")
        self.get_logger().info("Result: " + str(result.reached_number))

    def goal_feedback_callback(self, feedback_msg):
        number = feedback_msg.feedback.current_number   
        self.get_logger().info("Feedabck: " + str(number))

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClientNode()
    node.send_goal(10, 1.0)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()