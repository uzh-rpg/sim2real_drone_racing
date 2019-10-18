#pragma once

#include <ros/ros.h>

#include <polynomial_trajectories/minimum_snap_trajectories.h>
#include <quadrotor_common/trajectory_point.h>

namespace drone_racing {

class DroneRacing;

class Visualizer;

class GlobalTrajectory {
public:
  GlobalTrajectory(const ros::NodeHandle& pnh);

  virtual ~GlobalTrajectory();

  void generateGlobalTrajectory(std::shared_ptr<Visualizer>& visualizer,
                                std::vector<Eigen::Vector3d,
                                    Eigen::aligned_allocator<Eigen::Vector3d> >& goal_positions_);

  quadrotor_common::TrajectoryPoint getNextStateOnGlobalTraj(const double horizon);

  quadrotor_common::TrajectoryPoint projectOnTrajectory(quadrotor_common::TrajectoryPoint curr_desired_state);

  void findStartIndexGlobalTrajectory(const Eigen::Vector3d est_pos);

  void findStartIndexNWGlobalTrajectory(DroneRacing* reactive_nav);

  double getMaxVelocity();

  bool exists();

private:
  void saveGlobalTrajectory();

  void loadGlobalTrajectory();

  void loadParameters();

  ros::NodeHandle pnh_;

  std::vector<quadrotor_common::TrajectoryPoint> global_trajectory_sampled_;
  double global_v_max_ = 0.0;
  double global_v_min_ = 100.0;

  double global_traj_max_v_;
  double global_traj_horizon_;

  // hack for network
  int global_traj_idx_nw_ = 1;
  Eigen::Vector3d tracking_dir_nw_;
  double tracking_dist_nw_;

  // hack for projection
  int global_traj_idx_proj_ = 1;
  Eigen::Vector3d tracking_dir_proj_;
  double tracking_dist_proj_;
  bool handheld_experiment_;
  bool trajectory_exists_ = false;
  std::string root_dir_;

  std::string traj_dir_;

  bool load_existing_trajectory_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > waypoints_;

};

} /* namespace drone_racing */
