#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/move_robot.hpp"
#include "example_interfaces/msg/empty.hpp"

using MoveRobot = robot_interfaces::action::MoveRobot;
using MoveRobotGoalHandle = rclcpp_action::ClientGoalHandle<MoveRobot>;
using Empty = example_interfaces::msg::Empty;
using namespace std::placeholders;

class MoveRobotClientNode : public rclcpp::Node
{
public:
    MoveRobotClientNode() : Node("move_robot_client") 
    {
        move_robot_client_ = rclcpp_action::create_client<MoveRobot>(this, "move_robot");

        cancel_subscriber_ = this->create_subscription<Empty>("cancel_move", 10, std::bind(&MoveRobotClientNode::callback_cancel_move, this, _1));

        RCLCPP_INFO(this->get_logger(), "Action Client has been started");
    }
   
    void send_goal(int position, int velocity)
    {
        //waiting for the server
        move_robot_client_->wait_for_action_server();

        // define goal and velocity
        auto goal = MoveRobot::Goal();
        goal.position = position;
        goal.velocity = velocity;

        // ADD callbacks
        auto options = rclcpp_action::Client<MoveRobot>::SendGoalOptions();
        options.goal_response_callback = std::bind(&MoveRobotClientNode::goal_response_callback, this, _1);
        options.result_callback = std::bind(&MoveRobotClientNode::goal_result_callback, this, _1);
        options.feedback_callback = std::bind(&MoveRobotClientNode::goal_feedback_callback, this, _1, _2);
        
        //Send a goal 
        RCLCPP_INFO(this->get_logger(),"Sending a goal");
        move_robot_client_->async_send_goal(goal, options);
    }
    
private:

    // Callback to know if the goal was accpeted or rejected 
    void goal_response_callback(const MoveRobotGoalHandle::SharedPtr &goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Goal got rejected");
        }
        else
        {
            this->goal_handle_ = goal_handle;
            RCLCPP_INFO(this->get_logger(),"Goal is accepted");
        }
    }

    //callback to receive the result once the goal is done 
    void goal_result_callback(const MoveRobotGoalHandle::WrappedResult &result)
    {
        auto status = result.code;
        if (status == rclcpp_action::ResultCode::SUCCEEDED){
            RCLCPP_INFO(this->get_logger(),"SUCCEEDED");
        }
        else if (status == rclcpp_action::ResultCode::CANCELED){
            RCLCPP_INFO(this->get_logger(),"CANCELLED");
        }
        else if (status == rclcpp_action::ResultCode::ABORTED)
        {
            RCLCPP_INFO(this->get_logger(),"ABORTED");
        }
        int position = result.result->position;
        std::string message = result.result->message;
        RCLCPP_INFO(this->get_logger(), "Position: %d", position);
        RCLCPP_INFO(this->get_logger(), "Message: %s", message.c_str());
    }

    // To receive feedback during goal execution
    void goal_feedback_callback(const MoveRobotGoalHandle::SharedPtr &goal_handle, const std::shared_ptr<const MoveRobot::Feedback> feedback)
    {
        (void)goal_handle;
        int position = feedback->current_position;
        RCLCPP_INFO(this->get_logger(),"Feedback Position: %d", position);
    }

    // Self callback cancel move for cancel_subscriber_
    void callback_cancel_move(const Empty::SharedPtr msg)
    {
        (void)msg;
        cancel_goal();
    }
    void cancel_goal()
    {
        if(this->goal_handle_){
            this->move_robot_client_->async_cancel_goal(goal_handle_);
            goal_handle_.reset();
        }
    }

    rclcpp_action::Client<MoveRobot>::SharedPtr move_robot_client_;
    rclcpp::Subscription<Empty>::SharedPtr cancel_subscriber_ ; 
    MoveRobotGoalHandle::SharedPtr goal_handle_;

};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveRobotClientNode>();
    node->send_goal(75, 1);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}