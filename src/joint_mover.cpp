#include <memory>
#include <map>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

// Target joint to move
const std::string TARGET_JOINT = "fr3_joint7";  // rotate EE
const double TARGET_POSITION = 1.57;            // radians (90°)
const double TRAJECTORY_DURATION = 2.0;         // seconds

// The joint order expected by the controller
const std::vector<std::string> CONTROLLER_JOINTS = {
    "fr3_joint1", "fr3_joint2", "fr3_joint3",
    "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"
};

// Global storage of current joint positions
std::map<std::string, double> joint_positions;

class JointMover : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollow = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  JointMover()
  : Node("joint_mover")
  {
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&JointMover::jointStateCallback, this, std::placeholders::_1));

    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/fr3/arm_controller/follow_joint_trajectory");

    RCLCPP_INFO(get_logger(), "JointMover node initialized");
  }

  void run()
  {
    // Wait until we receive all joint states
    RCLCPP_INFO(get_logger(), "Waiting for /joint_states...");
    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
      bool all_found = true;
      for (const auto & joint : CONTROLLER_JOINTS) {
        if (joint_positions.find(joint) == joint_positions.end()) {
          all_found = false;
          break;
        }
      }
      if (all_found) break;
      rate.sleep();
    }
    RCLCPP_INFO(get_logger(), "Joint states received.");

    // Wait for the action server
    RCLCPP_INFO(get_logger(), "Waiting for action server...");
    if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
      return;
    }

    // Create goal
    FollowJointTrajectory::Goal goal_msg;
    goal_msg.trajectory.joint_names = CONTROLLER_JOINTS;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (const auto & joint : CONTROLLER_JOINTS) {
      if (joint == TARGET_JOINT) {
        point.positions.push_back(TARGET_POSITION);
      } else {
        point.positions.push_back(joint_positions[joint]);
      }
    }
    point.time_from_start = rclcpp::Duration::from_seconds(TRAJECTORY_DURATION);
    goal_msg.trajectory.points.push_back(point);
    goal_msg.trajectory.header.stamp = now();

    RCLCPP_INFO(get_logger(), "Sending trajectory goal to move %s", TARGET_JOINT.c_str());

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.result_callback = [this](const GoalHandleFollow::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "Movement complete.");
      } else {
        RCLCPP_ERROR(get_logger(), "Movement failed or was canceled/aborted.");
      }
      rclcpp::shutdown();
    };

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      joint_positions[msg->name[i]] = msg->position[i];
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointMover>();

  // Use an executor to spin callbacks
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  node->run();

  spin_thread.join();
  return 0;
}
