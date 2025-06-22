#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from robot_interfaces.action import MoveRobot
from example_interfaces.msg import Empty

class MoveRobotClientNode(Node):
    def __init__(self):
        super().__init__("move_robot_client")
        self.goal_handle_ = None
        self.move_robot_client_ = ActionClient(self,MoveRobot,"move_robot")
        self.get_logger().info("Action Client has been started")
        self.cancel_subscriber = self.create_subscription(Empty, "cancel_move", self.callback_cancel_move, 10)

    def send_goal(self, position, velocity):
         
        # Waiting For the server
        self.move_robot_client_.wait_for_server()

        # create a goal
        goal = MoveRobot.Goal()
        goal.position = position
        goal.velocity = velocity
        
        # Send the goal
        self.get_logger().info("Sending the Goal with position: " + str(position) + " velocity: " + str(velocity))
        self.move_robot_client_.send_goal_async(goal, feedback_callback=self.goal_feedback_callback).add_done_callback(self.goal_response_callback)
    
    # Callback to show whether the goal was accepted or rejected 
    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("GOAL ACCEPTED") 
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
             self.get_logger().info("GOAL REJECTED")

    # Callback group to show the Result to the client 
    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
             self.get_logger().info("SUCCESS")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("ABORTED")  
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().error("CANCELLED")
        self.get_logger().info("Position: " + str(result.position))
        self.get_logger().info("Message: " + str(result.message))

    # Adding feedback for the client:
    def goal_feedback_callback(self, feedback_msg):
        position = feedback_msg.feedback.current_position
        self.get_logger().info("Feedback: " + str(position))

    def callback_cancel_move(self, msg):
        self.cancel_goal()

    def cancel_goal(self):
        if self.goal_handle_ is not None:
            self.get_logger().info("Send a Cancel request")
            self.goal_handle_.cancel_goal_async()
        
def main(args=None):
        rclpy.init(args=args)
        node = MoveRobotClientNode()
        node.send_goal(75, 1)
        rclpy.spin(node)
        rclpy.shutdown()   
if __name__ == "__main__": 
    main()