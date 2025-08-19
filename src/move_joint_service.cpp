#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <fr3_sim_interfaces/srv/move_single_joint.hpp>  // ROS2 .srv interface

using namespace std::placeholders;

class MoveSingleJointServer : public rclcpp::Node
{
public:
  using MoveSingleJoint = fr3_sim_interfaces::srv::MoveSingleJoint;
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  MoveSingleJointServer() : Node("move_single_joint_server")
  {
    // Subscribe to joint states
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&MoveSingleJointServer::jointStateCallback, this, _1));

    // Create service
    service_ = this->create_service<MoveSingleJoint>(
      "move_single_joint",
      std::bind(&MoveSingleJointServer::handleRequest, this, _1, _2));

    // Create action client
    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/fr3/arm_controller/follow_joint_trajectory");

    RCLCPP_INFO(this->get_logger(), "MoveSingleJoint service ready.");
  }

private:
  std::map<std::string, double> joint_positions_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Service<MoveSingleJoint>::SharedPtr service_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      joint_positions_[msg->name[i]] = msg->position[i];
    }
  }

  void handleRequest(
      const std::shared_ptr<MoveSingleJoint::Request> req,
      std::shared_ptr<MoveSingleJoint::Response> res)
  {
    // Ensure joint exists
    static const std::vector<std::string> CONTROLLER_JOINTS = {
      "fr3_joint1","fr3_joint2","fr3_joint3",
      "fr3_joint4","fr3_joint5","fr3_joint6","fr3_joint7"
    };
    if (std::find(CONTROLLER_JOINTS.begin(), CONTROLLER_JOINTS.end(), req->joint_name)
        == CONTROLLER_JOINTS.end())
    {
      res->success = false;
      res->message = "Joint not found: " + req->joint_name;
      return;
    }

    // Build trajectory goal
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = CONTROLLER_JOINTS;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (const auto & joint : CONTROLLER_JOINTS) {
      if (joint == req->joint_name) {
        point.positions.push_back(req->target_position);
      } else {
        point.positions.push_back(joint_positions_[joint]);
      }
    }
    point.time_from_start = rclcpp::Duration::from_seconds(req->duration);
    goal_msg.trajectory.points.push_back(point);
    goal_msg.trajectory.header.stamp = this->get_clock()->now();

    // Send goal
    if (!action_client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available");
      res->success = false;
      res->message = "Action server not available";
      return;
    }

    auto future_goal = action_client_->async_send_goal(goal_msg);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
      res->success = false;
      res->message = "Failed to send goal";
      return;
    }

    res->success = true;
    res->message = "Moved joint " + req->joint_name;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveSingleJointServer>());
  rclcpp::shutdown();
  return 0;
}
