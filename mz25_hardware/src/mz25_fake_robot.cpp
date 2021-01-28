#include "mz25_hardware/mz25_fake_robot.hpp"
#include <limits>
#include <cmath>

namespace mz25_robot
{
    MZ25FakeRobot::MZ25FakeRobot()
    {
        state_position_.resize(NUM_AXIS);
        command_position_.resize(NUM_AXIS);
        std::fill(state_position_.begin(), state_position_.end(), 0);
        std::fill(command_position_.begin(), command_position_.end(), std::numeric_limits<double>::quiet_NaN());
    }
    return_type MZ25FakeRobot::read(std::vector<double> &buff_pos, std::vector<double> &buff_vel, std::vector<double> &buff_tor)
    {
        if (!std::isnan(command_position_[0]))
        {
            state_position_ = command_position_;
        }
        buff_pos = state_position_;
        const std::vector<double> zeros(NUM_AXIS, 0);
        buff_vel = zeros;
        buff_tor = zeros;
        return return_type::OK;
    }
    return_type MZ25FakeRobot::write(std::vector<double> &cmd_pos, bool is_end_point)
    {
        (void)is_end_point;
        command_position_ = cmd_pos;
        return return_type::OK;
    }
    return_type MZ25FakeRobot::start()
    {
        return return_type::OK;
    }
    return_type MZ25FakeRobot::stop()
    {
        return return_type::OK;
    }
} // namespace mz25_robot