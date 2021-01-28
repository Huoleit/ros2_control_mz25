#ifndef MZ25_BASE_INTERFACE_H_
#define MZ25_BASE_INTERFACE_H_

#include <cstdint>
#include <vector>
#include <string>
#include <sstream>

namespace mz25_robot
{
    enum class return_type : std::uint8_t
    {
        OK = 0,
        ERROR = 1,
    };
    class MZ25BaseInterface
    {
    public:
        MZ25BaseInterface() = default;
        ~MZ25BaseInterface() = default;
        virtual return_type start() = 0;
        virtual return_type stop() = 0;
        virtual return_type read(std::vector<double> &, std::vector<double> &, std::vector<double> &) = 0;
        virtual return_type write(std::vector<double> &, bool is_end_point) = 0;
        inline void set_error_msg(int code, std::string msg)
        {
            error_code_ = code;
            std::stringstream ss;
            ss << "Error Code: " << error_code_ << "Msg: " << msg;
            error_msg_ = ss.str();
        }
        inline const char *get_error_msg()
        {
            return error_msg_.c_str();
        }

        static constexpr int NUM_AXIS = 6;
        static constexpr double JOINT_DIFF_THRES = 0.1;

    protected:
        std::vector<double> state_position_;
        std::vector<double> state_velocity_;
        std::vector<double> state_effort_;
        std::vector<double> command_position_;
        int error_code_;
        std::string error_msg_;
    };

} // namespace mz25_robot
#endif
