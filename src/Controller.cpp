#include "rclcpp/rclcpp.hpp"

#include "lgsvl_msgs/msg/vehicle_odometry.hpp"
#include "lgsvl_msgs/msg/vehicle_control_data.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Controller : public rclcpp::Node
{
public:
  Controller()
      : Node("controller")
  {
    this->declare_parameter("target_velocity", 20.0);
    this->declare_parameter("kp", 0.0);
    this->declare_parameter("ki", 0.0);
    this->declare_parameter("kd", 0.0);
    this->declare_parameter("frequency", 10.0);
    this->declare_parameter("odometry_topic", "/lgsvl/vehicle_odom");
    this->declare_parameter("control_data_topic", "/lgsvl/vehicle_control_cmd");

    this->get_parameter("target_velocity", target_velocity);
    this->get_parameter("kp", Kp);
    this->get_parameter("kd", Kd);
    this->get_parameter("ki", Ki);
    this->get_parameter("frequency", frequency);
    this->get_parameter("odometry_topic", odometry_topic);
    this->get_parameter("control_data_topic", control_data_topic);

    RCLCPP_INFO(this->get_logger(), "----------------------------------------------------");
    RCLCPP_INFO(this->get_logger(), "target_velocity: %f", target_velocity);
    RCLCPP_INFO(this->get_logger(), "kp: %f", Kp);
    RCLCPP_INFO(this->get_logger(), "kd: %f", Kd);
    RCLCPP_INFO(this->get_logger(), "ki: %f", Ki);
    RCLCPP_INFO(this->get_logger(), "frequency: %f", frequency);
    RCLCPP_INFO(this->get_logger(), "odometry_topic: %s", odometry_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "control_data_topic: %s", control_data_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "----------------------------------------------------");
    RCLCPP_INFO(this->get_logger(), "Waiting for odometry data...");

    publisher_ = this->create_publisher<lgsvl_msgs::msg::VehicleControlData>(control_data_topic, static_cast<int>(frequency));

    subscription_ = this->create_subscription<lgsvl_msgs::msg::VehicleOdometry>(
        odometry_topic, static_cast<int>(frequency), std::bind(&Controller::topic_callback, this, _1));
  }

private:
  // Declare publisher and subscriber
  rclcpp::Publisher<lgsvl_msgs::msg::VehicleControlData>::SharedPtr publisher_;
  rclcpp::Subscription<lgsvl_msgs::msg::VehicleOdometry>::SharedPtr subscription_;

  // Parameters
  float target_velocity;
  float Kp;
  float Ki;
  float Kd;
  float frequency;
  std::string odometry_topic;
  std::string control_data_topic;

  // Variables
  float error_prev = 0.0;
  float error_sum = 0.0;

  // Callback function for odometry data
  void topic_callback(const lgsvl_msgs::msg::VehicleOdometry::ConstSharedPtr msg)
  {
    float curr_velocity = msg->velocity;
    // Control speed with PID
    float error = target_velocity - curr_velocity;
    error_sum += error / frequency;
    float error_diff = (error - error_prev) * frequency;
    error_prev = error;

    // Integrator anti-windup
    if (error_sum > 2.0)
    {
      error_sum = 2.0;
    }
    else if (error_sum < -2.0)
    {
      error_sum = -2.0;
    }

    float control_speed = Kp * error + Ki * error_sum + Kd * error_diff;
    // Limit control speed
    if (control_speed > 1.0)
    {
      control_speed = 1.0;
    }
    else if (control_speed < -1.0)
    {
      control_speed = -1.0;
    }

    // Send command to vehicle
    auto message = lgsvl_msgs::msg::VehicleControlData();
    message.target_gear = 1;
    if (control_speed > 0)
    {
      message.acceleration_pct = control_speed;
    }
    else
    {
      message.braking_pct = control_speed;
    }
    publisher_->publish(message);

    RCLCPP_INFO(this->get_logger(), "Current velocity: %f", curr_velocity);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}