#ifndef MZ25_HARDWARE_INTERFACE_HPP_
#define MZ25_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/macros.hpp"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "mz25_hardware/visibility_control.h"
#include "mz25_hardware/mz25_robot.hpp"

using hardware_interface::return_type;

namespace ros2_control_mz25_hardware
{
  class MZ25SystemHardware : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MZ25SystemHardware)

    ROS2_CONTROL_MZ25_HARDWARE_PUBLIC
    return_type configure(const hardware_interface::HardwareInfo &info) override;

    ROS2_CONTROL_MZ25_HARDWARE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    ROS2_CONTROL_MZ25_HARDWARE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    ROS2_CONTROL_MZ25_HARDWARE_PUBLIC
    return_type start() override;

    ROS2_CONTROL_MZ25_HARDWARE_PUBLIC
    return_type stop() override;

    ROS2_CONTROL_MZ25_HARDWARE_PUBLIC
    return_type read() override;

    ROS2_CONTROL_MZ25_HARDWARE_PUBLIC
    return_type write() override;

  private:
    mz25_robot::MZ25Robot mz25_;
    std::vector<double> state_position_;
    std::vector<double> state_velocity_;
    std::vector<double> state_effort_;
    std::vector<double> command_position_;
    std::vector<double> command_is_end_point_;
    std::vector<double> command_is_updated_;
  };

} // namespace ros2_control_mz25_hardware

#endif
