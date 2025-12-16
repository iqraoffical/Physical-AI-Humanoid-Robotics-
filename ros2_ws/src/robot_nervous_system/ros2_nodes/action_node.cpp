// action_node.cpp
// Action node for long-running robot behaviors in the ROS 2 nervous system.
// Implements actions for asynchronous robot control (FR-002).

#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_nervous_system/action/move_robot.hpp"

using MoveRobot = robot_nervous_system::action::MoveRobot;
using GoalHandleMoveRobot = rclcpp_action::ServerGoalHandle<MoveRobot>;

class ActionNode : public rclcpp::Node
{
public:
    ActionNode() : Node("action_node")
    {
        using namespace std::placeholders;

        // Create action server for moving the robot
        this->action_server_ = rclcpp_action::create_server<MoveRobot>(
            this,
            "move_robot",
            std::bind(&ActionNode::handle_goal, this, _1, _2),
            std::bind(&ActionNode::handle_cancel, this, _1),
            std::bind(&ActionNode::handle_accepted, this, _1));

        RCLCPP_INFO(this->get_logger(), "Action Node initialized");
    }

private:
    rclcpp_action::Server<MoveRobot>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveRobot::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request to move robot");

        // Validate the target pose against 24+ DOF humanoid model (from clarifications)
        // In a real implementation, we would check if the pose is kinematically reachable
        (void)uuid;
        
        // For now, accept all goals
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
    {
        using namespace std::placeholders;
        
        // This needs to return quickly, so spin up a new thread
        std::thread{std::bind(&ActionNode::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal to move robot");

        // Get the goal
        const auto goal = goal_handle->get_goal();
        
        // Get the target pose
        auto target_pose = goal->target_pose;
        
        // In a real implementation, we would:
        // 1. Plan a trajectory to reach the target pose
        // 2. Execute the movement using the humanoid's joints
        // 3. Monitor the actual pose during execution
        
        // For simulation, we'll just report progress
        auto feedback = std::make_shared<MoveRobot::Feedback>();
        auto result = std::make_shared<MoveRobot::Result>();
        
        // Start with the current pose as the robot's starting position
        double current_x = 0.0;
        double current_y = 0.0;
        double current_z = 0.0;
        
        // Simulate the movement progress
        double distance_to_target = std::sqrt(
            std::pow(target_pose.position.x - current_x, 2) +
            std::pow(target_pose.position.y - current_y, 2) +
            std::pow(target_pose.position.z - current_z, 2)
        );
        
        // Initialize feedback
        feedback->current_pose.position.x = current_x;
        feedback->current_pose.position.y = current_y;
        feedback->current_pose.position.z = current_z;
        feedback->distance_remaining = distance_to_target;
        feedback->trajectory_progress = 0.0;
        
        // Simulate execution in steps
        rclcpp::Rate rate(10); // 10 Hz feedback rate (from API contract)
        double progress = 0.0;
        double max_progress = 1.0;
        const double step = 0.05;
        
        while (progress < max_progress) {
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->final_pose.position.x = feedback->current_pose.position.x;
                result->final_pose.position.y = feedback->current_pose.position.y;
                result->final_pose.position.z = feedback->current_pose.position.z;
                result->error_code = 5;  // Canceled
                
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
            
            // Update progress
            progress += step;
            if (progress > max_progress) {
                progress = max_progress;
            }
            
            // Update the robot's simulated position
            feedback->current_pose.position.x = current_x + 
                (target_pose.position.x - current_x) * progress;
            feedback->current_pose.position.y = current_y + 
                (target_pose.position.y - current_y) * progress;
            feedback->current_pose.position.z = current_z + 
                (target_pose.position.z - current_z) * progress;
                
            // Update remaining distance
            feedback->distance_remaining = std::sqrt(
                std::pow(target_pose.position.x - feedback->current_pose.position.x, 2) +
                std::pow(target_pose.position.y - feedback->current_pose.position.y, 2) +
                std::pow(target_pose.position.z - feedback->current_pose.position.z, 2)
            );
            
            // Update progress
            feedback->trajectory_progress = progress;
            
            // Publish feedback
            goal_handle->publish_feedback(feedback);
            
            rate.sleep();
        }
        
        // Check if goal was canceled during execution
        if (goal_handle->is_canceling()) {
            result->success = false;
            result->final_pose.position.x = feedback->current_pose.position.x;
            result->final_pose.position.y = feedback->current_pose.position.y;
            result->final_pose.position.z = feedback->current_pose.position.z;
            result->error_code = 5;  // Canceled
            
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled during execution");
        } else {
            // Success
            result->success = true;
            result->final_pose.position = target_pose.position;
            result->error_code = 0;  // Success
            
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActionNode>());
    rclcpp::shutdown();
    return 0;
}