#include "drone_racing/trajectory_manager.h"

#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <quadrotor_common/parameter_helper.h>

#include "drone_racing/drone_racing.h"

namespace drone_racing {

TrajectoryManager::TrajectoryManager(const ros::NodeHandle& pnh) :
    pnh_(pnh) {
  loadParameters();

  trajectory_implementation_.reset(
      new drone_racing::MinimumJerkTrajectory());
}

TrajectoryManager::~TrajectoryManager() = default;

bool TrajectoryManager::computeTrajectory(DroneRacing* reactive_nav,
                                          const quadrotor_common::TrajectoryPoint& start_state,
                                          const nav_msgs::Odometry state_estimate,
                                          Eigen::Vector3d network_selection) {

  quadrotor_common::TrajectoryPoint end_state;
  double desired_velocity = std::max(max_velocity_ * network_selection.z(), min_velocity_);
  double planning_length_rescaled;
  planning_length_rescaled = std::min(planning_length_max_,
                                      std::max(planning_length_min_, planning_length_ * desired_velocity));

  double yaw = -camera_semi_fov_rad_yaw_ * network_selection.x();
  double pitch = camera_semi_fov_rad_pitch_ * network_selection.y();

  // get vector from start_state to end_state
  Eigen::Vector3d goal_b = Eigen::Vector3d(
      planning_length_rescaled * cos(pitch) * cos(yaw),
      planning_length_rescaled * cos(pitch) * sin(yaw),
      planning_length_rescaled * sin(pitch));
  rpg::Pose T_B_goal;
  geometry_msgs::Pose goal_pose;
  goal_pose.position.x = goal_b.x();
  goal_pose.position.y = goal_b.y();
  goal_pose.position.z = goal_b.z();
  goal_pose.orientation.w = 1.0;
  goal_pose.orientation.x = 0.0;
  goal_pose.orientation.y = 0.0;
  goal_pose.orientation.z = 0.0;
  tf::poseMsgToKindr(goal_pose, &T_B_goal);

  // transform vector to body frame of quad
  rpg::Pose T_W_B;
  geometry_msgs::Pose state_estimate_pose;
  state_estimate_pose.position.x = state_estimate.pose.pose.position.x;
  state_estimate_pose.position.y = state_estimate.pose.pose.position.y;
  state_estimate_pose.position.z = state_estimate.pose.pose.position.z;
  state_estimate_pose.orientation = state_estimate.pose.pose.orientation;
  tf::poseMsgToKindr(state_estimate_pose, &T_W_B);

  rpg::Pose T_W_goal = T_W_B * T_B_goal;

  end_state.position = T_W_goal.getPosition();

  bool success = trajectory_implementation_->computeTrajectory(start_state,
                                                               end_state,
                                                               trajectory_implementation_->EnforceEndOfTraj::kPos,
                                                               desired_velocity,
                                                               false);
  if (!success) {
    ROS_ERROR("[%s] Failed to compute NN trajectory.", ros::this_node::getName().c_str());
  }
  return success;
}

void TrajectoryManager::clearTrajectories(const bool is_mb) {
  trajectory_implementation_->clearTrajectories(is_mb);
}

void TrajectoryManager::setBestTraj(const int best_traj_idx, const bool is_mb) {
  trajectory_implementation_->setBestTrajectory(best_traj_idx, is_mb);
  trajectory_implementation_->clearTrajectories(is_mb);
  use_mb_ = is_mb;
}

void TrajectoryManager::visualizeTrajectory(DroneRacing* reactive_nav) {
  double trajectory_dt = 1 / trajectory_viz_freq_; // [s]
  std::vector<quadrotor_common::TrajectoryPoint> trajectory;
  if (show_full_trajectory_) {
    trajectory = sampleTrajectory(trajectory_implementation_->getEndTimeFromTrajectory(), trajectory_dt);
  } else {
    double end_time = std::min(trajectory_implementation_->getEndTimeFromTrajectory(), viz_horizon_);
    trajectory = sampleTrajectory(end_time, trajectory_dt);
  }
  if (!use_mb_) {
    reactive_nav->visualizer_->visualizeTrajectory(trajectory, use_mb_, Visualizer::Color::kGreen, 0);
    return;
  }
  reactive_nav->visualizer_->visualizeTrajectory(trajectory, use_mb_, Visualizer::Color::kRed, 0);
}

std::vector<quadrotor_common::TrajectoryPoint>
TrajectoryManager::sampleTrajectory(const double duration, const double dt) {
  std::vector<quadrotor_common::TrajectoryPoint> trajectory;
  quadrotor_common::TrajectoryPoint current_state;

  for (double t = 0.0; t <= duration; t += dt) {
    current_state = trajectory_implementation_->getDesiredStateFromTrajectory(t);
    trajectory.push_back(current_state);
  }

  return trajectory;
}

quadrotor_common::TrajectoryPoint
TrajectoryManager::getDesiredStateFromTrajectory() {
  quadrotor_common::TrajectoryPoint state;
  state = trajectory_implementation_->getDesiredStateFromTrajectory();
  return state;
}

void TrajectoryManager::setStartTime() {
  trajectory_implementation_->setStartTime();
}

void TrajectoryManager::loadParameters() {
  double camera_fov_deg_yaw;
  double camera_fov_deg_pitch;
  quadrotor_common::getParam<double>("max_velocity", max_velocity_, 1.0, pnh_);
  quadrotor_common::getParam<double>("min_velocity", min_velocity_, 0.5, pnh_);
  quadrotor_common::getParam<double>("global_traj_horizon", global_traj_horizon_, 1.0, pnh_);

  // Planning Parameters
  quadrotor_common::getParam<double>("planning_length", planning_length_, 2.0, pnh_);
  quadrotor_common::getParam<double>("planning_length_min", planning_length_min_, 2.0, pnh_);
  quadrotor_common::getParam<double>("planning_length_max", planning_length_max_, 2.0, pnh_);
  quadrotor_common::getParam<double>("camera_fov_yaw", camera_fov_deg_yaw, 45.0, pnh_);
  quadrotor_common::getParam<double>("camera_fov_pitch", camera_fov_deg_pitch, 45.0, pnh_);
  quadrotor_common::getParam<double>("theta_gain", theta_gain_, 1.0, pnh_);

  // Trajectory Visualization
  quadrotor_common::getParam<double>("trajectory_viz_freq", trajectory_viz_freq_, 10.0, pnh_);
  quadrotor_common::getParam<bool>("show_full_trajectory", show_full_trajectory_, false, pnh_);
  quadrotor_common::getParam<double>("viz_horizon", viz_horizon_, 1.0, pnh_);

  camera_semi_fov_rad_yaw_ = camera_fov_deg_yaw * (M_PI / 180.0);
  camera_semi_fov_rad_pitch_ = camera_fov_deg_pitch * (M_PI / 180.0);
  viz_horizon_ = std::max(viz_horizon_, 1.0 / trajectory_viz_freq_);
}

} /* namespace drone_racing */
