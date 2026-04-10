#include "mecanum_controller/mecanum_controller.hpp"
#include "controller_interface/helpers.hpp"
namespace mecanum_controller
{
  MecanumController::MecanumController() {}

  //declare paramteres obtained from yaml file
  controller_interface::CallbackReturn MecanumController::on_init()
  {   


    try {
      //declare params THAT ARE EXPOSED FROM THE YAML FILE
      auto_declare<std::vector<std::string>>("joints", std::vector<std::string>()); // MAKE SURE YAML JOINT NAMES ARE IN ORDER FL,FR,BL,BR
      auto_declare<double>("vehicle_length", 0.0);
      auto_declare<double>("vehicle_width", 0.0);
      auto_declare<double>("wheel_radius", 1.0);
      auto_declare<double>("expected_wheels", 0.0);

    }
    catch (const std::exception& error) {
      fprintf(stderr, "Exception during init: %s\n", error.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;

  }

  controller_interface::CallbackReturn MecanumController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    //get parameters that have been previously declared
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    vehicle_width_ = get_node()->get_parameter("vehicle_width").as_double();
    vehicle_length_ = get_node()->get_parameter("vehicle_length").as_double();
    wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
    expected_wheels_ = get_node()->get_parameter("expected_wheels").as_double();
    if (joint_names_.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
      return controller_interface::CallbackReturn::ERROR;
    }
  

  //setup subscriber for commands_
  cmd_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&MecanumController::twistCommandCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), "found %zu joints", joint_names_.size());
  
  return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn MecanumController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    //clear all previous unsent commands_
    cmd_buffer_.reset();
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn MecanumController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration MecanumController::command_interface_configuration() const
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

  controller_interface::InterfaceConfiguration MecanumController::state_interface_configuration() const
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
  
  void MecanumController::twistCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (joint_names_.size() != expected_wheels_)
    {
      RCLCPP_WARN(get_node()->get_logger(), "command size mismatch: expected %zu joints, got %f", joint_names_.size(), expected_wheels_);
    return;
    }
    
    cmd_buffer_.writeFromNonRT(*msg);
  }

  controller_interface::return_type MecanumController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    //get latest realtime buffer command
    auto commands_ = *cmd_buffer_.readFromRT();



    double vehicle_center = (vehicle_length_ + vehicle_width_)/2;


    double v_fl = (1/wheel_radius_)*(commands_.linear.x - commands_.linear.y - commands_.angular.z * vehicle_center);
    double v_fr = (1/wheel_radius_)*(commands_.linear.x + commands_.linear.y + commands_.angular.z * vehicle_center);
    double v_bl = (1/wheel_radius_)*(commands_.linear.x + commands_.linear.y - commands_.angular.z * vehicle_center);
    double v_br = (1/wheel_radius_)*(commands_.linear.x - commands_.linear.y + commands_.angular.z * vehicle_center);
    //write to command interfaces
    command_interfaces_[0].set_value(v_fl);
    command_interfaces_[1].set_value(v_fr);
    command_interfaces_[2].set_value(v_bl);
    command_interfaces_[3].set_value(v_br);
    
    return controller_interface::return_type::OK;
  }

} //namespace MecanumController
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    mecanum_controller::MecanumController, controller_interface::ControllerInterface
    )




