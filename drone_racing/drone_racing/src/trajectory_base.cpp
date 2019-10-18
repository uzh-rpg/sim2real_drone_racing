#include "drone_racing/trajectory_base.h"

#include <quadrotor_common/parameter_helper.h>

namespace drone_racing {

TrajectoryBase::TrajectoryBase(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) :
    nh_(nh), pnh_(pnh), yaw_trajectory_(0.0), time_start_best_trajectory_() {
  loadParameters();
}

TrajectoryBase::~TrajectoryBase() {
}

void TrajectoryBase::setStartTime() {
  time_start_best_trajectory_ = ros::Time::now();
}

void TrajectoryBase::loadParameters() {
  quadrotor_common::getParam<double>("trajectory_max_z", traj_max_z_, 2.5, pnh_);
  quadrotor_common::getParam<double>("trajectory_min_z", traj_min_z_, 0.0, pnh_);
  quadrotor_common::getParam<double>("max_velocity", max_velocity_, 1.0, pnh_);

  // Rapid Trajectory Parameters
  quadrotor_common::getParam<double>("min_normalized_thrust", min_normalized_thrust_, 5.0, pnh_);
  quadrotor_common::getParam<double>("max_normalized_thrust", max_normalized_thrust_, 15.0, pnh_);
  quadrotor_common::getParam<double>("max_roll_pitch_rate", max_roll_pitch_rate_, 0.5, pnh_);
  quadrotor_common::getParam<double>("min_trajectory_sampling_time", min_traj_sampling_time_, 0.02, pnh_);
}

} /* namespace drone_racing */
