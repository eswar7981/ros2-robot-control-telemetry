#include "chrono"
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_control/robot_motion_utils.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "telemetry_interfaces/msg/robot_status.hpp"
#include "telemetry_interfaces/srv/reset_robot.hpp"
#include "telemetry_interfaces/action/drive_distance.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using RobotStatus = telemetry_interfaces::msg::RobotStatus;
using ResetRobot = telemetry_interfaces::srv::ResetRobot;
using DriveDistance = telemetry_interfaces::action::DriveDistance;
using GoalHandleDriveDistance = rclcpp_action::ServerGoalHandle<DriveDistance>;

using std::placeholders::_1;
using std::placeholders::_2;

enum class MotionPhase
{
  IDLE,
  ROTATING,
  TRANSLATING
};

class RobotSimulatorNode : public rclcpp::Node
{
public:
  RobotSimulatorNode()
  : Node("robot_simulator_node")
  {
    max_linear_velocity_ = this->declare_parameter<std::double_t>("max_linear_speed", 1.0);
    max_angular_velocity_ = this->declare_parameter<std::double_t>("max_angular_speed", 1.0);
    publish_rate_ = this->declare_parameter<std::double_t>("publish_rate", 10.0);

    auto period = std::chrono::duration<double>(
        1.0 / publish_rate_);

    callback_handle_ =
      this->add_on_set_parameters_callback(std::bind(&RobotSimulatorNode::parametersCallback, this,
                                                       _1));

    timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                                     std::bind(&RobotSimulatorNode::update_and_send_pose, this));
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
                                                                                 std::bind(
      &RobotSimulatorNode::subscription_callback, this, _1));
    odom_pose_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_pose", 10);
    robot_status_publisher_ = this->create_publisher<RobotStatus>("robot_status", 10);
    robot_reset_server_ = this->create_service<ResetRobot>("reset_robot",
                                                           std::bind(
      &RobotSimulatorNode::reset_robot_callback, this, _1, _2), 10);
    action_drive_server_ = rclcpp_action::create_server<DriveDistance>(this, "drive_distance",
                                                                       std::bind(
                                                                           &RobotSimulatorNode::
          handle_drive_distance_goal, this, _1, _2),
                                                                       std::bind(
                                                                           &RobotSimulatorNode::
          handle_drive_distance_cancel_goal, this, _1),
                                                                       std::bind(
                                                                           &RobotSimulatorNode::
          handle_drive_distance_accept_goal, this, _1));
  }

private:
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
      if (param.get_name() == "publish_rate") {
        publish_rate_ = param.as_double();
      } else if (param.get_name() == "max_linear_speed") {
        max_linear_velocity_ = param.as_double();
      } else if (param.get_name() == "max_angular_speed") {
        max_angular_velocity_ = param.as_double();
      }
    }

    return result;
  }

  rclcpp_action::GoalResponse handle_drive_distance_goal(
    const rclcpp_action::GoalUUID & uuid,
    const std::shared_ptr<const DriveDistance::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received drive distance goal request");
    (void)uuid;
    (void)goal;
    if (active_goal_) {
      RCLCPP_INFO(this->get_logger(), "Already action server is working on a goal.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!isValidDistanceGoal(goal->target_distance)) {
      RCLCPP_WARN(this->get_logger(), "Goal rejected. target distance cannot be negative or zero.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!isValidRotationAngle(goal->rotation_angle)) {
      RCLCPP_WARN(this->get_logger(),
                  "Goal rejected. invalid rotation angle, should be between 0° and 360°");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_drive_distance_cancel_goal(
    const std::shared_ptr<GoalHandleDriveDistance> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received drive distance goal cancel request");
    (void)goal_handle;
    distance_travelled_ = 0;
    active_goal_.reset();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_drive_distance_accept_goal(const std::shared_ptr<GoalHandleDriveDistance> goal_handle)
  {
    active_goal_ = goal_handle;
    initial_heading_ = theta_;
    auto goal = active_goal_->get_goal();
    target_heading_ = initial_heading_ + (goal->rotation_angle * M_PI / 180.0);
    current_phase_ = MotionPhase::ROTATING;
  }

  void update_and_send_pose()
  {
    if (active_goal_ && active_goal_->is_active()) {
      auto goal = active_goal_->get_goal();

      if (current_phase_ == MotionPhase::ROTATING) {
        const double remaining_angle = std::atan2(
            std::sin(target_heading_ - theta_),
            std::cos(target_heading_ - theta_));

        if (std::abs(remaining_angle) < angle_tolerance_) {
          angular_velocity_ = 0.0;
          current_phase_ = MotionPhase::TRANSLATING;
        } else {
          constexpr double kp = 1.0;

          angular_velocity_ = std::clamp(
              kp * remaining_angle,
              -max_angular_velocity_,
              max_angular_velocity_);

          theta_ += angular_velocity_ * delta_t;
        }
      }

      if (current_phase_ == MotionPhase::TRANSLATING) {
        angular_velocity_ = 0.0;

        x_ += linear_velocity_ * std::cos(theta_) * delta_t;
        y_ += linear_velocity_ * std::sin(theta_) * delta_t;

        distance_travelled_ +=
          std::abs(linear_velocity_) * delta_t;
      }
    }

    if (!active_goal_ || !active_goal_->is_active()) {
      robot_current_state_ = "STOPPED";
    }
    if (active_goal_ && active_goal_->is_active()) {
      if (linear_velocity_ == 0.0 && angular_velocity_ == 0.0) {
        robot_current_state_ = "STOPPED";
      } else if (linear_velocity_ == 0.0 && angular_velocity_ != 0.0) {
        robot_current_state_ = "ROTATING";
      } else {
        robot_current_state_ = "MOVING";
      }
    }

    auto odom_msg = nav_msgs::msg::Odometry();

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.orientation.z = sin(theta_ / 2.0);
    odom_msg.pose.pose.orientation.w = cos(theta_ / 2.0);

    odom_msg.twist.twist.linear.x = linear_velocity_;
    odom_msg.twist.twist.angular.z = angular_velocity_;

    RCLCPP_INFO(this->get_logger(), "robot x position : %.2f", x_);
    RCLCPP_INFO(this->get_logger(), "robot y position : %.2f", y_);
    RCLCPP_INFO(
        this->get_logger(),
        "robot yaw: %.2f degrees",
        theta_ * 180.0 / M_PI);

    RCLCPP_INFO(this->get_logger(), "robot linear velocity x : %.2f", linear_velocity_);
    RCLCPP_INFO(this->get_logger(), "robot angular velocity z : %.2f", angular_velocity_);

    odom_pose_publisher_->publish(odom_msg);

    auto robot_status_msg = telemetry_interfaces::msg::RobotStatus();

    robot_status_msg.battery_percentage = 100.0;
    robot_status_msg.linear_velocity = linear_velocity_;
    robot_status_msg.angular_velocity = angular_velocity_;
    robot_status_msg.current_state = robot_current_state_;
    robot_status_msg.emergency_stop = 0;

    robot_status_publisher_->publish(robot_status_msg);

    if (active_goal_) {
      auto goal = active_goal_->get_goal();

      const double remaining_angle = std::atan2(
          std::sin(target_heading_ - theta_),
          std::cos(target_heading_ - theta_));

      const bool distance_reached =
        distance_travelled_ >= goal->target_distance;

      const bool angle_reached =
        std::abs(remaining_angle) < angle_tolerance_;

      if (distance_reached && angle_reached) {
        auto result = std::make_shared<DriveDistance::Result>();
        result->success = true;
        result->distance_travelled = distance_travelled_;
        result->angle_rotated = theta_;
        result->message = "action goal completed succesfully.";

        RCLCPP_INFO(this->get_logger(), "Action goal completed successfully.");

        distance_travelled_ = 0;
        active_goal_->succeed(result);
        active_goal_.reset();
        robot_current_state_ = "STOPPED";
        current_phase_ = MotionPhase::IDLE;
      } else {
        auto feedback = std::make_shared<DriveDistance::Feedback>();
        feedback->distance_travelled = distance_travelled_;
        feedback->angle_rotated = theta_ * 180.0 / M_PI;
        RCLCPP_INFO(this->get_logger(), "Action feedback sent successfully.");
        active_goal_->publish_feedback(feedback);
      }
    }
  }

  void subscription_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "%.2f %.2f", max_linear_velocity_, max_angular_velocity_);
    if (msg->linear.x > max_linear_velocity_) {
      RCLCPP_WARN(this->get_logger(), "Max linear velocity allowed : %.2f", max_linear_velocity_);
      return;
    }
    if (msg->angular.z > max_angular_velocity_) {
      RCLCPP_WARN(this->get_logger(), "Max angular velocity allows: %.2f", max_angular_velocity_);
      return;
    }
    linear_velocity_ = getClampVelocity(msg->linear.x, max_linear_velocity_);
    angular_velocity_ = getClampVelocity(msg->angular.z, max_angular_velocity_);
  }

  void reset_robot_callback(
    const std::shared_ptr<ResetRobot::Request> request,
    std::shared_ptr<ResetRobot::Response> response)
  {
    (void)request;
    if (active_goal_ && active_goal_->is_active()) {
      response->success = false;
      response->message = "Cannot reset robot while goal is active.";

      RCLCPP_INFO(this->get_logger(), "Robot reset unsuccessful");
      return;
    }
    x_ = 0;
    y_ = 0;
    theta_ = 0.0;
    response->success = true;
    response->message = "robot reset successful.";

    RCLCPP_INFO(this->get_logger(), "Robot reset successful");
  }

  double delta_t = 0.1; // same as timer callback period
  double linear_velocity_;
  double angular_velocity_;
  double heading_;

  double x_ = 0;
  double y_ = 0;
  double theta_ = 0.0;
  double distance_travelled_ = 0;

  double max_linear_velocity_;
  double max_angular_velocity_;
  double publish_rate_;
  std::shared_ptr<GoalHandleDriveDistance> active_goal_;

  std::string robot_current_state_ = "STOPPED";
  MotionPhase current_phase_ = MotionPhase::IDLE;
  double angle_tolerance_{0.0174533};

  double initial_heading_{0.0};
  double target_heading_{0.0};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pose_publisher_;
  rclcpp::Publisher<RobotStatus>::SharedPtr robot_status_publisher_;
  rclcpp::Service<ResetRobot>::SharedPtr robot_reset_server_;
  rclcpp_action::Server<DriveDistance>::SharedPtr action_drive_server_;

  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
