#include "forward_joint_trajectory_controller/trajectory.hpp"

#include <memory>

#include "hardware_interface/macros.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rcppmath/clamp.hpp"
#include "std_msgs/msg/header.hpp"
namespace forward_joint_trajectory_controller
{

Trajectory::Trajectory()
: sampled_already_(false)
{}

Trajectory::Trajectory(
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory)
: trajectory_msg_(joint_trajectory)
{
}


void
Trajectory::update(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory)
{
  trajectory_msg_ = joint_trajectory;
  sampled_already_ = false;
}

bool
Trajectory::sample(TrajectoryPointConstIter & cur_point_itr, bool & is_new_point)
{
  THROW_ON_NULLPTR(trajectory_msg_)

  is_new_point = false;
  if (trajectory_msg_->points.empty()) {
    cur_point_itr = end();
    return false;
  }

  // first sampling of this trajectory
  if (!sampled_already_) {
    cur_itr_ = begin();
    sampled_already_ = true;
    is_new_point = true;
  }
  else if (cur_itr_ != --end())
  {
    is_new_point = true;
    ++cur_itr_;
  }

  cur_point_itr = cur_itr_;
  return true;
}

TrajectoryPointConstIter
Trajectory::begin() const
{
  THROW_ON_NULLPTR(trajectory_msg_)

  return trajectory_msg_->points.begin();
}

TrajectoryPointConstIter
Trajectory::end() const
{
  THROW_ON_NULLPTR(trajectory_msg_)

  return trajectory_msg_->points.end();
}

}  // namespace forward_joint_trajectory_controller
