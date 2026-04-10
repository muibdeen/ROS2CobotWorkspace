#ifndef COBOT_CONTROLLER_HPP_
#define COBOT_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace mecanum_controller
{
  class MecanumController : public controller_interface::ControllerInterface
  {
  public:
  MecanumController();
    
  controller_interface::CallbackReturn on_init() override;
      
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  private:
  //store joint names
  std::vector<std::string> joint_names_;

  double expected_joints_;


 //create a subscriber for commands
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist> cmd_buffer_;

  void twistCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  

  };
}

#endif
