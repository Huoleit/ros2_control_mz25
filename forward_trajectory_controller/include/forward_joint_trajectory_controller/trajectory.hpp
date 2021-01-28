#ifndef FORWARD_JOINT_TRAJECTORY_CONTROLLER__TRAJECTORY_HPP_
#define FORWARD_JOINT_TRAJECTORY_CONTROLLER__TRAJECTORY_HPP_

#include <memory>
#include <vector>

#include "forward_joint_trajectory_controller/visibility_control.h"
#include "rclcpp/time.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
namespace forward_joint_trajectory_controller
{

using TrajectoryPointIter =
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::iterator;
using TrajectoryPointConstIter =
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator;

class Trajectory
{
public:
  FORWARD_JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  Trajectory();

  FORWARD_JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  explicit Trajectory(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory);

  FORWARD_JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  void
  update(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory);

  FORWARD_JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  bool
  sample(TrajectoryPointConstIter & cur_point_itr, bool & is_new_point);

  FORWARD_JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  TrajectoryPointConstIter
  begin() const;

  FORWARD_JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  TrajectoryPointConstIter
  end() const;

  FORWARD_JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  bool
  has_trajectory_msg() const {return trajectory_msg_.get();}

  FORWARD_JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory>
  get_trajectory_msg() const {return trajectory_msg_;}

private:
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg_;
  TrajectoryPointConstIter cur_itr_;
  bool sampled_already_;
};

/**
 * \return The map between \p t1 indices (implicitly encoded in return vector indices) to \p t2 indices.
 * If \p t1 is <tt>"{C, B}"</tt> and \p t2 is <tt>"{A, B, C, D}"</tt>, the associated mapping vector is
 * <tt>"{2, 1}"</tt>.
 */
template<class T>
inline std::vector<size_t> mapping(const T & t1, const T & t2)
{
  // t1 must be a subset of t2
  if (t1.size() > t2.size()) {return std::vector<size_t>();}

  std::vector<size_t> mapping_vector(t1.size());  // Return value
  for (auto t1_it = t1.begin(); t1_it != t1.end(); ++t1_it) {
    auto t2_it = std::find(t2.begin(), t2.end(), *t1_it);
    if (t2.end() == t2_it) {
      return std::vector<size_t>();
    } else {
      const size_t t1_dist = std::distance(t1.begin(), t1_it);
      const size_t t2_dist = std::distance(t2.begin(), t2_it);
      mapping_vector[t1_dist] = t2_dist;
    }
  }
  return mapping_vector;
}

}  // namespace joint_trajectory_controller

#endif  // JOINT_TRAJECTORY_CONTROLLER__TRAJECTORY_HPP_
