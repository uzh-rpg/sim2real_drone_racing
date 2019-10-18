#pragma once

#include <mutex>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <polynomial_trajectories/minimum_snap_trajectories.h>
#include <quadrotor_msgs/TrajectoryPoint.h>
#include <rapid_trajectories/RapidTrajectoryGenerator.h>
#include <rapid_trajectories/Vec3.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "drone_racing/data_saver.h"
#include "drone_racing/minimum_jerk_trajectory.h"
#include "drone_racing/trajectory_base.h"

using namespace RapidQuadrocopterTrajectoryGenerator;

namespace drone_racing {
class DroneRacing;

class Visualizer;

class DataSaver;

class TrajectoryManager {
public:
  explicit TrajectoryManager(const ros::NodeHandle& pnh);

  virtual ~TrajectoryManager();

  void clearTrajectories(const bool is_mb);

  void visualizeTrajectory(DroneRacing* reactive_nav);

  void setBestTraj(const int best_traj_idx, const bool is_mb);

  std::vector<quadrotor_common::TrajectoryPoint> sampleTrajectory(const double duration, const double dt);

  quadrotor_common::TrajectoryPoint getDesiredStateFromTrajectory();

  void setStartTime();

  bool computeTrajectory(DroneRacing* reactive_nav,
                         const quadrotor_common::TrajectoryPoint& start_state,
                         const nav_msgs::Odometry state_estimate,
                         Eigen::Vector3d network_selection);

private:
  void loadParameters();

  std::unique_ptr<drone_racing::TrajectoryBase> trajectory_implementation_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  double max_velocity_;
  double min_velocity_;
  // Planning Parameters
  double planning_length_;
  double planning_length_min_;
  double planning_length_max_;
  double global_traj_horizon_;

  double camera_semi_fov_rad_yaw_;
  double camera_semi_fov_rad_pitch_;
  double theta_gain_;

  // Trajectory Visualization
  double trajectory_viz_freq_;
  bool show_full_trajectory_;
  double viz_horizon_;

  bool use_mb_;
};

} /* namespace drone_racing */
