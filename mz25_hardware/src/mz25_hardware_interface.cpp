#include "mz25_hardware/mz25_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_mz25_hardware
{

  return_type MZ25SystemHardware::configure(
      const hardware_interface::HardwareInfo &info)
  {
    if (configure_default(info) != return_type::OK)
    {
      return return_type::ERROR;
    }

    state_position_.resize(mz25_robot::MZ25BaseInterface::NUM_AXIS, std::numeric_limits<double>::quiet_NaN());
    state_velocity_.resize(mz25_robot::MZ25BaseInterface::NUM_AXIS, std::numeric_limits<double>::quiet_NaN());
    state_effort_.resize(mz25_robot::MZ25BaseInterface::NUM_AXIS, std::numeric_limits<double>::quiet_NaN());
    command_position_.resize(mz25_robot::MZ25BaseInterface::NUM_AXIS, std::numeric_limits<double>::quiet_NaN());

    // for (const hardware_interface::ComponentInfo &joint : info_.joints)
    // {
    // }

    status_ = hardware_interface::status::CONFIGURED;
    return return_type::OK;
  }

  std::vector<hardware_interface::StateInterface>
  MZ25SystemHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(
              info_.joints[i].name, hardware_interface::HW_IF_POSITION, &state_position_[i]));
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(
              info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_velocity_[i]));
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(
              info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &state_effort_[i]));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  MZ25SystemHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(
          hardware_interface::CommandInterface(
              info_.joints[i].name, hardware_interface::HW_IF_POSITION, &command_position_[i]));
    }

    return command_interfaces;
  }

  return_type MZ25SystemHardware::start()
  {
    RCLCPP_INFO(
        rclcpp::get_logger("MZ25"),
        "Starting ...please wait...");
    if (mz25_.start() == mz25_robot::return_type::OK)
    {
      RCLCPP_INFO(rclcpp::get_logger("MZ25"), "System Sucessfully started!");
      status_ = hardware_interface::status::STARTED;
      return return_type::OK;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("MZ25"), mz25_.get_error_msg());
      return return_type::ERROR;
    }
  }

  return_type MZ25SystemHardware::stop()
  {
    RCLCPP_INFO(rclcpp::get_logger("MZ25"), "Stopping ...please wait...");
    if (mz25_.stop() == mz25_robot::return_type::OK)
    {
      RCLCPP_INFO(rclcpp::get_logger("MZ25"), "System sucessfully stopped!");
      status_ = hardware_interface::status::STOPPED;
      return return_type::OK;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("MZ25"), mz25_.get_error_msg());
      return return_type::ERROR;
    }
  }

  hardware_interface::return_type MZ25SystemHardware::read()
  {
    if (mz25_.read(state_position_, state_velocity_, state_effort_) == mz25_robot::return_type::OK)
    {
      RCLCPP_INFO(rclcpp::get_logger("MZ25"), "Joints sucessfully read!");
      for (auto i = 0u; i < state_position_.size(); i++)
      {
        RCLCPP_INFO(rclcpp::get_logger("MZ25"), "pos %.5f for joint %d!", state_position_[i], i);
      }
      return return_type::OK;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("MZ25"), mz25_.get_error_msg());
      return return_type::ERROR;
    }
  }

  hardware_interface::return_type MZ25SystemHardware::write()
  {
    if (mz25_.write(command_position_) == mz25_robot::return_type::OK)
    {
      RCLCPP_INFO(rclcpp::get_logger("MZ25"), "Joints sucessfully written!");
      // for (int i = 0; i < state_position_.size(); i++)
      // {
      //   RCLCPP_INFO(rclcpp::get_logger("MZ25"), "pos %.5f for joint %d!", state_position_[i], i);
      // }
      return return_type::OK;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("MZ25"), mz25_.get_error_msg());
      return return_type::ERROR;
    }
  }

} // namespace ros2_control_mz25_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ros2_control_mz25_hardware::MZ25SystemHardware,
    hardware_interface::SystemInterface)
