#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.node import Node
from robot_interfaces.action import MoveRobot
from rclpy.action.server import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class MoveRobotServerNode(Node):
    def __init__(self):
        super().__init__("move_robot_server")
        self.robot_position_ = 50
        self.goal_lock_ = threading.Lock()
        self.goal_handle_ : ServerGoalHandle = None
        self.move_robot_server_ = ActionServer(self,
                                               MoveRobot,
                                               "move_robot",
                                               cancel_callback= self.cancel_callback,
                                               goal_callback= self.goal_callback,
                                               callback_group= ReentrantCallbackGroup(),
                                               execute_callback=self.execute_callback)
                                            
        self.get_logger().info("Action Server has been started")
        self.get_logger().info("Robot Position: " + str(self.robot_position_))

    def goal_callback(self, goal_request: MoveRobot.Goal):
        self.get_logger().info("Received a Goal")
        
        # Validate the Goal 
        if goal_request.position not in range(0, 100) or goal_request.velocity < 0:
            self.get_logger().error("Invalid position/velocity, the Goal is REJECTED")
            return GoalResponse.REJECT
        
        # Policy: New Goal is valid, abort existing goal and accept new goal 
        if self.goal_handle_ is not None and self.goal_handle_.is_active:
            self.get_logger().info("Aborting the current goal and accept new goal")
            self.goal_handle_.abort()

        self.get_logger().info("GOAL ACCEPTED")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a cancel request")
        return CancelResponse.ACCEPT
            
    def execute_callback(self, goal_handle: ServerGoalHandle):

        with self.goal_lock_:
            self.goal_handle_ = goal_handle

        # Request for Goal from the action
        goal_position = goal_handle.request.position
        velocity = goal_handle.request.velocity

        result = MoveRobot.Result()
        feedback = MoveRobot.Feedback()

        # Execute the goal 
        self.get_logger().info("Exeecuting the Goal")

        while rclpy.ok():

            if not goal_handle.is_active:
                result.position = self.robot_position_
                result.message = "Preempted by another goal"
                return result
            
            if goal_handle.is_cancel_requested:
                # self.get_logger().info("Cancelling the GOAL")
                # goal_handle.canceled()
                result.position = self.robot_position_
                if goal_position == self.robot_position_:
                    result.message= "SUCCESS"
                    goal_handle.succeed()
                else:
                    result.message= "Canceleld"
                    goal_handle.canceled()
                return result
            

            diff = goal_position - self.robot_position_
            if diff == 0:
                result.position = self.robot_position_
                result.message = "SUCCESS"
                goal_handle.succeed()
                return result
            elif diff > 0:
                if diff >= velocity:
                    self.robot_position_ += velocity
                else:
                    self.robot_position_ += diff
            elif diff < 0:
                if abs(diff) >= velocity:
                    self.robot_position_ -= velocity
                else:
                    self.robot_position_ += diff

            self.get_logger().info("Robot Position: " + str(self.robot_position_))
            feedback.current_position = self.robot_position_
            goal_handle.publish_feedback(feedback)
            time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args) 
    node = MoveRobotServerNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == "__main__":
    main()  