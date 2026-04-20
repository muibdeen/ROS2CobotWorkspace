#pragma once

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace cobot_hardware
{
  class CobotHardware : public hardware_interface::SystemInterface
  {
    public:
      CobotHardware();
      hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
      hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
      hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
      hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
      hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
      hardware_interface::return_type write( const rclcpp::Time &time, const rclcpp::Duration & period) override;

    private:
      std::vector<double> hw_positions_;
      std::vector<double> hw_velocities_;
      std::vector<double> hw_position_commands_;
      std::vector<double> hw_velocity_commands_;
  };
} // namespace CobotHardware
