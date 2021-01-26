#include "mz25_hardware/mz25_base_interface.hpp"

namespace mz25_robot
{
    class MZ25FakeRobot final : public MZ25BaseInterface
    {

    public:
        MZ25FakeRobot();
        ~MZ25FakeRobot() = default;
        return_type start() override;
        return_type stop() override;
        return_type read(std::vector<double> &, std::vector<double> &, std::vector<double> &) override;
        return_type write(std::vector<double> &) override;
    };

} // namespace mz25_robot