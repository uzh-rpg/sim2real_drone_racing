#include "drone_racing/global_trajectory.h"

#include <fstream>

#include <polynomial_trajectories/polynomial_trajectories_common.h>
#include <quadrotor_common/parameter_helper.h>

#include "drone_racing/drone_racing.h"

namespace drone_racing {

GlobalTrajectory::GlobalTrajectory(const ros::NodeHandle &pnh) :
    pnh_(pnh) {
  loadParameters();
}

GlobalTrajectory::~GlobalTrajectory() = default;

void GlobalTrajectory::generateGlobalTrajectory(std::shared_ptr<Visualizer> &visualizer,
                                                std::vector<Eigen::Vector3d,
                                                            Eigen::aligned_allocator<Eigen::Vector3d> > &waypoints) {
  // iterate over waypoints, extract waypoints only if further away than threshold from last one
  Eigen::Vector3d last = 999 * Eigen::Vector3d::Ones();
  std::vector<Eigen::Vector3d> waypoints_filtered;
  int j = 0;
  for (const auto &current : waypoints) {
    if ((current - last).norm() > 0.5) {
      waypoints_filtered.push_back(current);
      last = current;
      j++;
    }
  }

  ROS_INFO("Generating global trajectory through [%d] waypoints.", static_cast<int>(waypoints_filtered.size()));

  // Calculate segment times
  Eigen::VectorXd segment_times = Eigen::VectorXd::Ones(waypoints_filtered.size());
  Eigen::Vector3d last_filtered = waypoints_filtered.back();
  for (int i = 0; i < waypoints_filtered.size(); i++) {
    segment_times[i] = ((waypoints_filtered[i] - last_filtered).norm()) / global_traj_max_v_;
    last_filtered = waypoints_filtered[i];
  }

  Eigen::VectorXd minimization_weights(4);
  minimization_weights << 0.1, 10.0, 100.0, 100.0;

//  polynomial_trajectories::TrajectorySettings trajectory_settings;
  polynomial_trajectories::PolynomialTrajectorySettings trajectory_settings;
  trajectory_settings.way_points = waypoints_filtered;
  trajectory_settings.minimization_weights = minimization_weights;
  trajectory_settings.polynomial_order = 11;
  trajectory_settings.continuity_order = 4;

  double maximal_des_thrust = 18.0;
  double maximal_roll_pitch_rate = 1.5;

  if (!load_existing_trajectory_) {
//    polynomial_trajectories::Trajectory trajectory;
    polynomial_trajectories::PolynomialTrajectory trajectory;
    trajectory =
        polynomial_trajectories::minimum_snap_trajectories::generateMinimumSnapRingTrajectoryWithSegmentRefinement(
            segment_times, trajectory_settings, global_traj_max_v_, maximal_des_thrust,
            maximal_roll_pitch_rate);

    ROS_INFO("Done with trajectory! flight time: %1.1f", trajectory.T.toSec());
    double dt = 0.05;
    int n = trajectory.T.toSec() / dt;
    quadrotor_common::TrajectoryPoint state;
    double t = 0.0;
    Eigen::Vector3d acc;
    Eigen::Quaterniond q;
    global_trajectory_sampled_.clear();
    for (int i = 0; i < (n); i++) {
      t = i % n * dt;
      state = polynomial_trajectories::getPointFromTrajectory(trajectory, ros::Duration(t));
      global_trajectory_sampled_.push_back(state);
    }
    saveGlobalTrajectory();
  } else {
    loadGlobalTrajectory();
  }
  visualizer->visualizeTrajectory(global_trajectory_sampled_, true, Visualizer::Color::kPurple, 3);
  trajectory_exists_ = true;
}

void GlobalTrajectory::saveGlobalTrajectory() {
  std::ofstream outfile_trajectory;
  std::string filename_trajectory = traj_dir_ + "/global_trajectory.txt";
  outfile_trajectory.open(filename_trajectory, std::ios_base::app);

  for (auto state : global_trajectory_sampled_) {
    outfile_trajectory << std::to_string(state.position.x()) << ";"
                       << std::to_string(state.position.y()) << ";"
                       << std::to_string(state.position.z()) << ";"
                       << std::to_string(state.velocity.x()) << ";"
                       << std::to_string(state.velocity.y()) << ";"
                       << std::to_string(state.velocity.z()) << ";"
                       << std::to_string(state.acceleration.x()) << ";"
                       << std::to_string(state.acceleration.y()) << ";"
                       << std::to_string(state.acceleration.z()) << ";"
                       << std::to_string(state.jerk.x()) << ";"
                       << std::to_string(state.jerk.y()) << ";"
                       << std::to_string(state.jerk.z()) << ";"
                       << std::to_string(state.snap.x()) << ";"
                       << std::to_string(state.snap.y()) << ";"
                       << std::to_string(state.snap.z()) << ";"
                       << std::to_string(state.heading) << ";"
                       << std::to_string(state.heading_rate) << ";"
                       << std::to_string(state.heading_acceleration) << ";"
                       << std::to_string(state.orientation.w()) << ";"
                       << std::to_string(state.orientation.x()) << ";"
                       << std::to_string(state.orientation.y()) << ";"
                       << std::to_string(state.orientation.z()) << ";"
                       << std::to_string(state.bodyrates.x()) << ";"
                       << std::to_string(state.bodyrates.y()) << ";"
                       << std::to_string(state.bodyrates.z()) << ";"
                       << std::to_string(state.angular_acceleration.x()) << ";"
                       << std::to_string(state.angular_acceleration.y()) << ";"
                       << std::to_string(state.angular_acceleration.z()) << ";"
                       << std::to_string(state.angular_jerk.x()) << ";"
                       << std::to_string(state.angular_jerk.y()) << ";"
                       << std::to_string(state.angular_jerk.z()) << ";"
                       << std::to_string(state.angular_snap.x()) << ";"
                       << std::to_string(state.angular_snap.y()) << ";"
                       << std::to_string(state.angular_snap.z()) << ";"
                       << std::to_string(state.time_from_start.toSec()) << "\n";

  }
  outfile_trajectory.close();
  ROS_INFO("Saved global trajectory to %s", filename_trajectory.c_str());
}

void GlobalTrajectory::loadGlobalTrajectory() {
  global_trajectory_sampled_.clear();

  std::string line;
  std::string filename_trajectory = traj_dir_ + "/global_trajectory.txt";
  ROS_INFO("Loading saved trajectory from %s", filename_trajectory.c_str());
  std::ifstream infile_trajectory(filename_trajectory);
  if (infile_trajectory.is_open()) {
    std::string entry;
    while (getline(infile_trajectory, entry, ';')) {
      quadrotor_common::TrajectoryPoint state;
      // position & derivatives
      state.position[0] = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.position[1] = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.position[2] = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.velocity[0] = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.velocity[1] = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.velocity[2] = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.acceleration[0] = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.acceleration[1] = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.acceleration[2] = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.jerk[0] = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.jerk[1] = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.jerk[2] = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.snap[0] = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.snap[1] = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.snap[2] = atof(entry.c_str());

      // orientation & derivatives
      getline(infile_trajectory, entry, ';');
      state.heading = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.heading_rate = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.heading_acceleration = atof(entry.c_str());

      getline(infile_trajectory, entry, ';');
      double qw = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      double qx = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      double qy = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      double qz = atof(entry.c_str());
      state.orientation = Eigen::Quaternion<double>(qw, qx, qy, qz);
      getline(infile_trajectory, entry, ';');
      state.bodyrates[0] = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.bodyrates[1] = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.bodyrates[2] = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.angular_acceleration[0] = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.angular_acceleration[1] = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.angular_acceleration[2] = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.angular_jerk[0] = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.angular_jerk[1] = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.angular_jerk[2] = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.angular_snap[0] = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.angular_snap[1] = atof(entry.c_str());
      getline(infile_trajectory, entry, ';');
      state.angular_snap[2] = atof(entry.c_str());
      getline(infile_trajectory, entry, infile_trajectory.widen('\n'));
      state.time_from_start = ros::Duration(atof(entry.c_str()));

      global_trajectory_sampled_.push_back(state);
    }
    infile_trajectory.close();
  } else {
    ROS_WARN("Something went wrong while loading the global trajectory.");
  }
}

quadrotor_common::TrajectoryPoint
GlobalTrajectory::getNextStateOnGlobalTraj(const double horizon) {
  int forecast = 1;
  int eff_index = (global_traj_idx_proj_ + forecast) % global_trajectory_sampled_.size();
  while ((global_trajectory_sampled_[eff_index].position - global_trajectory_sampled_[global_traj_idx_proj_].position)
      .norm() < horizon) {
    forecast++;
    eff_index = (global_traj_idx_proj_ + forecast) % global_trajectory_sampled_.size();
  }
  return global_trajectory_sampled_[eff_index];
}

quadrotor_common::TrajectoryPoint
GlobalTrajectory::projectOnTrajectory(quadrotor_common::TrajectoryPoint curr_desired_state) {
  tracking_dir_proj_ = global_trajectory_sampled_[global_traj_idx_proj_].position -
      global_trajectory_sampled_[global_traj_idx_proj_ - 1].position;
  tracking_dist_proj_ = tracking_dir_proj_.norm();
  tracking_dir_proj_ /= tracking_dist_proj_;

  Eigen::Vector3d offset =
      curr_desired_state.position - global_trajectory_sampled_[global_traj_idx_proj_ - 1].position;

  double x = offset.dot(tracking_dir_proj_) / (tracking_dist_proj_);

  while (x > 1.0 && global_traj_idx_proj_ < global_trajectory_sampled_.size()) {
    global_traj_idx_proj_++;
    tracking_dir_proj_ = global_trajectory_sampled_[global_traj_idx_proj_].position -
        global_trajectory_sampled_[global_traj_idx_proj_ - 1].position;
    tracking_dist_proj_ = tracking_dir_proj_.norm();
    tracking_dir_proj_ /= tracking_dist_proj_;
    offset = curr_desired_state.position - global_trajectory_sampled_[global_traj_idx_proj_ - 1].position;
    x = offset.dot(tracking_dir_proj_) / (tracking_dist_proj_);
  }
  if (global_traj_idx_proj_ >= global_trajectory_sampled_.size()) {
    global_traj_idx_proj_ = 1; // hack, index is set to 1 to avoid issue above (index - 1) 
  }

  // search over whole trajectory if further away than threshold...
  if((curr_desired_state.position - global_trajectory_sampled_[global_traj_idx_proj_].position).norm() > 1.0) {
    findStartIndexGlobalTrajectory(curr_desired_state.position);
  }

  return global_trajectory_sampled_[global_traj_idx_proj_];
}

void GlobalTrajectory::findStartIndexGlobalTrajectory(const Eigen::Vector3d est_pos) {
  // iterate over trajectory, get point with closest distance to current quad position
  double min_dist = 1000.0;
  Eigen::Vector3d offset = Eigen::Vector3d::Ones();
  Eigen::Vector3d v;
  for (int i = 0; i < global_trajectory_sampled_.size(); i++) {
    offset = global_trajectory_sampled_[i].position - est_pos;
    v = global_trajectory_sampled_[i].velocity;

    if (offset.norm() < min_dist) {
      global_traj_idx_nw_ = i;
      global_traj_idx_proj_ = i;
      min_dist = offset.norm();
    }
    if (v.norm() > global_v_max_) {
      global_v_max_ = v.norm();
    }
    if (v.norm() < global_v_min_) {
      global_v_min_ = v.norm();
    }
  }
//  ROS_INFO("Max_v: [%f], Min_v: [%f], min_dist = [%f]", global_v_max_, global_v_min_, min_dist);
}

void GlobalTrajectory::findStartIndexNWGlobalTrajectory(DroneRacing *reactive_nav) {
  // iterate over trajectory, get point with closest distance to current quad position
  Eigen::Vector3d est_pos = reactive_nav->getEstimatedPos();
  double min_dist = 1000.0;
  Eigen::Vector3d offset;
  Eigen::Vector3d v;
  for (int i = 0; i < global_trajectory_sampled_.size(); i++) {
    offset = global_trajectory_sampled_[i].position - est_pos;
    v = global_trajectory_sampled_[i].velocity;
    if (offset.norm() < min_dist) {
      global_traj_idx_nw_ = i;
      min_dist = offset.norm();
    }
  }
  ROS_INFO("Max_v: [%f], Min_v: [%f]", global_v_max_, global_v_min_);
}

double GlobalTrajectory::getMaxVelocity() {
  return global_v_max_;
}

void GlobalTrajectory::loadParameters() {
  quadrotor_common::getParam<double>("global_traj_max_v", global_traj_max_v_, 1.0, pnh_);
  quadrotor_common::getParam<double>("global_traj_horizon", global_traj_horizon_, 1.0, pnh_);
  quadrotor_common::getParam<bool>("handheld_experiment", handheld_experiment_, false, pnh_);
  quadrotor_common::getParam<std::string>("root_dir", root_dir_, "", pnh_);
  quadrotor_common::getParam<bool>("load_existing_trajectory", load_existing_trajectory_, false, pnh_);
  quadrotor_common::getParam<std::string>("trajectory_path", traj_dir_, "", pnh_);
}

bool GlobalTrajectory::exists() {
  return trajectory_exists_;
}

} /* namespace drone_racing */
