#include "mz25_hardware/mz25_base_interface.hpp"

namespace mz25_robot
{
    class MZ25Robot final : public MZ25BaseInterface
    {

    public:
        MZ25Robot();
        ~MZ25Robot();
        return_type start() override;
        return_type stop() override;
        return_type read(std::vector<double> &, std::vector<double> &, std::vector<double> &) override;
        return_type write(std::vector<double> &, bool) override;
        return_type init_connection();
        return_type read_all_status();
        return_type write_pos_cmd_to_robot(bool);
        bool is_joint_pos_error_larger_than_thres(std::vector<double> cur, std::vector<double> dst);
        
    private:
        int nXmlOpenId;
    };

} // namespace mz25_robot