#include "cobot_controller/cobot_controller.hpp"
#include "controller_interface/helpers.hpp"
namespace cobot_controller
{
  CobotController::CobotController() {}

  //declare paramteres obtained from yaml file
  controller_interface::CallbackReturn CobotController::on_init()
  {   


    try {
      //declare params THAT ARE EXPOSED FROM THE YAML FILE
      auto_declare<std::vector<std::string>>("joints", std::vector<std::string>()); // MAKE SURE YAML JOINT NAMES ARE IN ORDER FL,FR,BL,BR
      auto_declare<double>("expected_joints", 0.0);
    }
    catch (const std::exception& error) {
      fprintf(stderr, "Exception during init: %s\n", error.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;

  }

  controller_interface::CallbackReturn CobotController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    //get parameters that have been previously declared
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    expected_joints_ = get_node()->get_parameter("expected_joints").as_double();
    if (joint_names_.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
      return controller_interface::CallbackReturn::ERROR;
    }
  

  //setup subscriber for commands_
  cmd_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&CobotController::twistCommandCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), "found %zu joints", joint_names_.size());
  
  return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn CobotController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    //clear all previous unsent commands_
    cmd_buffer_.reset();
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn CobotController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration CobotController::command_interface_configuration() const
  {
    // need to define command interfaces
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & joint : joint_names_)
    {
      config.names.push_back(joint + "/velocity");
    }
    return config;
  }

  controller_interface::InterfaceConfiguration CobotController::state_interface_configuration() const
  {
    //grab position  and velocity state interfaces
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & joint : joint_names_)
    {
      config.names.push_back(joint + "/position");
      config.names.push_back(joint + "/velocity");
    }
    return config;
  }
  
  void CobotController::twistCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (joint_names_.size() != expected_joints_)
    {
      RCLCPP_WARN(get_node()->get_logger(), "command size mismatch: expected %zu joints, got %f", expected_joints_, joint_names_.size());
    return;
    }
    RCLCPP_INFO(get_node()->get_logger(), "Data written to buffer"); // Check 2
    cmd_buffer_.writeFromNonRT(*msg);
  }

  controller_interface::return_type CobotController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    //get latest realtime buffer command
    auto commands_ = *cmd_buffer_.readFromRT();

    double v_j1 = (commands_.linear.x + commands_.linear.y + commands_.angular.z * 2);
    double v_j2 = (commands_.linear.x + commands_.linear.y + commands_.angular.z * 2);
    double v_j3 = (commands_.linear.x + commands_.linear.y + commands_.angular.z * 2);
    double v_j4 = (commands_.linear.x + commands_.linear.y + commands_.angular.z * 2);
    double v_j5 = (commands_.linear.x + commands_.linear.y + commands_.angular.z * 2);
    double v_j6 = (commands_.linear.x + commands_.linear.y + commands_.angular.z * 2);

    //write to command interfaces
    command_interfaces_[0].set_value(v_j1);
    command_interfaces_[1].set_value(v_j2);
    command_interfaces_[2].set_value(v_j3);
    command_interfaces_[3].set_value(v_j4);
    command_interfaces_[4].set_value(v_j5);
    command_interfaces_[5].set_value(v_j6);
    
    return controller_interface::return_type::OK;
  }

} //namespace CobotController
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    cobot_controller::CobotController, controller_interface::ControllerInterface
    )




