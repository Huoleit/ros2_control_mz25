#include "mz25_hardware/mz25_robot.hpp"
#include <string.h>
#include "mz25_hardware/OpenNR-IF.h"
#include <limits>
#include "angles/angles.h"
#include <cmath>

namespace mz25_robot
{
    MZ25Robot::MZ25Robot() : nXmlOpenId(-1)
    {
        state_position_.resize(NUM_AXIS, std::numeric_limits<double>::quiet_NaN());
        state_velocity_.resize(NUM_AXIS, std::numeric_limits<double>::quiet_NaN());
        state_effort_.resize(NUM_AXIS, std::numeric_limits<double>::quiet_NaN());
        command_position_.resize(NUM_AXIS, std::numeric_limits<double>::quiet_NaN());
    }
    MZ25Robot::~MZ25Robot()
    {
        stop();
    }
    return_type MZ25Robot::start()
    {
        return init_connection();
    }
    return_type MZ25Robot::stop()
    {
        if (nXmlOpenId > 0)
        {
            int nErr = NR_Close(nXmlOpenId);
            if (nErr == NR_E_NORMAL)
            {
                nXmlOpenId = -1;
                return return_type::OK;
            }
            else
            {
                set_error_msg(nErr, "Close error");
                return return_type::ERROR;
            }
        }
        else
        {
            return return_type::OK;
        }
    }
    return_type MZ25Robot::read(std::vector<double> &buff_pos, std::vector<double> &buff_vel, std::vector<double> &buff_tor)
    {
        if (read_all_status() == return_type::OK)
        {
            buff_pos = state_position_;
            buff_vel = state_velocity_;
            buff_tor = state_effort_;
            return return_type::OK;
        }
        else
        {
            return return_type::ERROR;
        }
    }
    return_type MZ25Robot::write(std::vector<double> &buff_pos, bool is_end_point)
    {
        command_position_ = buff_pos;
        return write_pos_cmd_to_robot(is_end_point);
    }
    return_type MZ25Robot::init_connection()
    {

        NACHI_COMMIF_INFO info;
        memset(&info, 0, sizeof(info));
        std::string ip_address("192.168.1.10");
        info.pcAddrs = &ip_address[0];
        info.lKind = NR_DATA_XML;

        nXmlOpenId = NR_Open(&info);
        if (nXmlOpenId > 0)
        {
            return return_type::OK;
        }
        else
        {
            set_error_msg(nXmlOpenId, "Connection failed");
            return return_type::ERROR;
        }
    }
    return_type MZ25Robot::read_all_status()
    {
        if (nXmlOpenId < 0)
        {
            set_error_msg(0, "Read error-No valid connection");
            return return_type::ERROR;
        }

        float buffer_pos[NUM_AXIS] = {0};
        float buffer_vel[NUM_AXIS] = {0};
        float buffer_tor[NUM_AXIS] = {0};
        int nErr_pos = NR_AcsAxisTheta(nXmlOpenId, buffer_pos, 1, NUM_AXIS, NR_PUSH_MODE_10);
        int nErr_vel = NR_AcsAxisSpeed(nXmlOpenId, buffer_vel, 1, NUM_AXIS, NR_PUSH_MODE_10);
        int nErr_tor = NR_AcsAxisTorque(nXmlOpenId, buffer_tor, 1, NUM_AXIS, NR_PUSH_MODE_10);

        if (nErr_pos == NR_E_NORMAL && nErr_vel == NR_E_NORMAL && nErr_tor == NR_E_NORMAL)
        {
            for (int i = 0; i < NUM_AXIS; i++)
            {
                state_position_[i] = angles::from_degrees(static_cast<double>(buffer_pos[i])) - (i == 1 ? M_PI_2 : 0);
                state_velocity_[i] = static_cast<double>(buffer_vel[i]);
                state_effort_[i] = static_cast<double>(buffer_tor[i]);
            }
            return return_type::OK;
        }
        else
        {
            set_error_msg(nErr_pos, "Read error");
            return return_type::ERROR;
        }
    }
    return_type MZ25Robot::write_pos_cmd_to_robot(bool is_end_point)
    {

        float fAngle[NUM_AXIS];
        for (auto i = 0u; i < command_position_.size(); i++)
        {
            if (std::isnan(command_position_[i]))
            {
                set_error_msg(0, "Command is nan");
                return return_type::ERROR;
            }
            fAngle[i] = angles::to_degrees(static_cast<float>(command_position_[i])) + (i == 1 ? 90.0f : 0);
        }
        int nErr = NR_CtrlMoveJ(nXmlOpenId, fAngle, NUM_AXIS, is_end_point ? 2 : 0);
        if (nErr == NR_E_NORMAL)
        {
            return return_type::OK;
        }
        else
        {
            set_error_msg(nErr, "Cannot move robot");
            return return_type::ERROR;
        }
    }

    bool MZ25Robot::is_joint_pos_error_larger_than_thres(std::vector<double> cur, std::vector<double> dst)
    {
        // error defined as the difference between current and desired
        if (cur.size() != dst.size())
            return false;
        for (auto i = 0u; i < cur.size(); i++)
        {
            double error = std::abs(angles::shortest_angular_distance(cur[i], dst[i]));
            if (error > JOINT_POS_CMD_UPDATE_THRES)
                return true;
        }
        return false;
    }

} // namespace mz25_robot