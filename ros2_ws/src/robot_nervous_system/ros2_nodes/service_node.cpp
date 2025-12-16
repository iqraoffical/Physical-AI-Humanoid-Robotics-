// service_node.cpp
// Service node for robot configuration in the ROS 2 nervous system.
// Implements services for synchronous robot control (FR-002).

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_nervous_system/srv/load_urdf.hpp"
#include "robot_nervous_system/srv/execute_action.hpp"

using LoadURDF = robot_nervous_system::srv::LoadURDF;
using ExecuteAction = robot_nervous_system::srv::ExecuteAction;

class ServiceNode : public rclcpp::Node
{
public:
    ServiceNode() : Node("service_node")
    {
        // Create service for loading URDF model
        load_urdf_service_ = this->create_service<LoadURDF>(
            "load_urdf",
            [this](const std::shared_ptr<LoadURDF::Request> request,
                   std::shared_ptr<LoadURDF::Response> response) {
                this->load_urdf_callback(request, response);
            });

        // Create service for executing actions
        execute_action_service_ = this->create_service<ExecuteAction>(
            "execute_action",
            [this](const std::shared_ptr<ExecuteAction::Request> request,
                   std::shared_ptr<ExecuteAction::Response> response) {
                this->execute_action_callback(request, response);
            });

        RCLCPP_INFO(this->get_logger(), "Service Node initialized");
    }

private:
    void load_urdf_callback(
        const std::shared_ptr<LoadURDF::Request> request,
        std::shared_ptr<LoadURDF::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Loading URDF for robot: %s", request->robot_name.c_str());
        
        // In a real implementation, this would load the URDF model
        // For now, we'll just simulate a successful load
        response->success = true;
        response->message = "URDF loaded successfully for robot: " + request->robot_name;
        
        // Validate URDF content (from data model: must be well-formed XML with valid kinematic chain)
        if (request->robot_description.empty()) {
            response->success = false;
            response->message = "URDF content is empty";
            RCLCPP_ERROR(this->get_logger(), "Failed to load URDF: content is empty");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "URDF loaded successfully for robot: %s", 
                   request->robot_name.c_str());
    }

    void execute_action_callback(
        const std::shared_ptr<ExecuteAction::Request> request,
        std::shared_ptr<ExecuteAction::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Executing action: %s", request->action_name.c_str());
        
        // In a real implementation, this would execute the requested robot action
        // This might involve coordinating multiple joints to perform a complex movement
        response->success = true;
        response->action_id = "action_" + request->action_name + "_" + 
                             std::to_string(this->get_clock()->now().nanoseconds());
        
        RCLCPP_INFO(this->get_logger(), "Action executed with ID: %s", 
                   response->action_id.c_str());
    }

    rclcpp::Service<LoadURDF>::SharedPtr load_urdf_service_;
    rclcpp::Service<ExecuteAction>::SharedPtr execute_action_service_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServiceNode>());
    rclcpp::shutdown();
    return 0;
}