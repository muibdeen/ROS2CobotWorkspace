#include "cobot_hardware/cobot_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace cobot_hardware
{
  CobotHardware::CobotHardware() {}

  hardware_interface::CallbackReturn CobotHardware::on_init(const hardware_interface::HardwareInfo & info)
  {
    // parse through the urdf and populate the info_ member variable
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    //initialize space based on the number of joints in the urdf
    hw_positions_.resize(info_.joints.size(), 0.0);
    hw_velocities_.resize(info_.joints.size(), 0.0);
    hw_position_commands_.resize(info_.joints.size(), 0.0);
    hw_velocity_commands_.resize(info_.joints.size(), 0.0);


    //iterate through joints and validate that we have claimed the correct command and state command_interfaces
    for (const auto & joint : info_.joints)
    {
    //validate that the urdf config is what we expect
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(get_logger(), "joint '%s' needs 1 command interface", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      //check that the command interface is a velocity interface
      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(get_logger(), "Joint '%s' needs a velocity command interface", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      //check state interfaces
    
      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(get_logger(), "joint '%s' needs 2 states interfaces", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn CobotHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Configuring Robot Hardware");

    //typically this is where you define serial port conncetions , open ports, and initialize hardware
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn CobotHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Activating hardware");
    
    //set initial command values to the current state of the robot so we dont jump on activation
    for (size_t i = 0; i < hw_positions_.size(); i++)
    {
      hw_velocity_commands_[i] = hw_velocities_[i];
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn CobotHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_logger(), "Deactivating Hardware");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type CobotHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)

  {
  // Read actual hardware state (encoders, sensors, etc.)
  // Update hw_positions_ and hw_velocities_ with real data
  
  // For now, placeholder implementation:
  RCLCPP_DEBUG(get_logger(), "Reading joint states");
  
  return hardware_interface::return_type::OK;
  }


  // call this are control loop frequency, match to yaml file params
  // dont block any calls, this needs to be realtime state_interfaces
  hardware_interface::return_type CobotHardware::write(const rclcpp::Time & /*time*/ , const rclcpp::Duration & /*period*/)
  {
    //send commands to the hardware via motor drivers

    RCLCPP_DEBUG(get_logger(), "Writing commands %.2f, %.2f", hw_velocity_commands_[0], hw_velocity_commands_[1]);

    return hardware_interface::return_type::OK;
  }
                                               
} // namespace cobot_hardware
  //
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cobot_hardware::CobotHardware, hardware_interface::SystemInterface)
