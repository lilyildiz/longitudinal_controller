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
    publisher_ = this->create_publisher<lgsvl_msgs::msg::VehicleControlData>("/lgsvl/vehicle_control_cmd", hz);

    subscription_ = this->create_subscription<lgsvl_msgs::msg::VehicleOdometry>(
        "/lgsvl/vehicle_odom", hz, std::bind(&Controller::topic_callback, this, _1));

  }

private:
  rclcpp::Publisher<lgsvl_msgs::msg::VehicleControlData>::SharedPtr publisher_;
  rclcpp::Subscription<lgsvl_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
  float target_velocity = 20.0;
  float error_prev = 0.0;
  float error_sum = 0.0;
  float Kp = 0.4;
  float Ki = 0.2;
  float Kd = 0.01;
  int hz = 10;
  float curr_velocity = 0.0;

  void topic_callback(const lgsvl_msgs::msg::VehicleOdometry::ConstSharedPtr msg)
  {
    curr_velocity = msg->velocity;
    // Control speed with PID
    float error = target_velocity - curr_velocity;
    error_sum += error / hz;
    float error_diff = (error - error_prev) * hz;
    error_prev = error;

    // integrator anti-windup
    if (error_sum > 1.0)
    {
      error_sum = 1.0;
    }
    else if (error_sum < -1.0)
    {
      error_sum = -1.0;
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

    //Send command to vehicle
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