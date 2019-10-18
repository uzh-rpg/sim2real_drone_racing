#include "drone_racing/minimum_jerk_trajectory.h"

#include <quadrotor_common/parameter_helper.h>

namespace drone_racing {

bool MinimumJerkTrajectory::computeTrajectory(const quadrotor_common::TrajectoryPoint& start_state,
                                              const quadrotor_common::TrajectoryPoint& end_state,
                                              const EnforceEndOfTraj end_constraint,
                                              const double& desired_speed,
                                              const bool is_mb) {

  bool success = false;
  Vec3 pos0 = Vec3(start_state.position(0),
                   start_state.position(1),
                   start_state.position(2));
  Vec3 vel0 = Vec3(start_state.velocity(0),
                   start_state.velocity(1),
                   start_state.velocity(2));
  Vec3 acc0 = Vec3(start_state.acceleration(0),
                   start_state.acceleration(1),
                   start_state.acceleration(2));
  double yaw = start_state.heading;

  //Define how gravity lies in our coordinate system
  Vec3 gravity = Vec3(0, 0, -9.81);//[m/s**2]

  std::shared_ptr<RapidTrajectoryGenerator> traj =
      std::make_shared<RapidTrajectoryGenerator>(pos0, vel0, acc0, gravity);

  Vec3 posf = Vec3(end_state.position(0),
                   end_state.position(1),
                   end_state.position(2));
  Vec3 velf = Vec3(end_state.velocity(0),
                   end_state.velocity(1),
                   end_state.velocity(2));
  Vec3 accf = Vec3(end_state.acceleration(0),
                   end_state.acceleration(1),
                   end_state.acceleration(2));

  double execution_time = std::max((posf - pos0).GetNorm2() / desired_speed,
                                   (posf - pos0).GetNorm2() / (start_state.velocity.norm() + 0.5));
  execution_time = std::max((posf - pos0).GetNorm2() / max_velocity_, execution_time);
  switch (end_constraint) {
    case EnforceEndOfTraj::kPos: {
      traj->SetGoalPosition(posf);
      break;
    }
    case EnforceEndOfTraj::kPosVel: {
      traj->SetGoalPosition(posf);
      traj->SetGoalVelocity(velf);
      break;
    }
    case EnforceEndOfTraj::kPosVelAcc: {
      traj->SetGoalPosition(posf);
      traj->SetGoalVelocity(velf);
      traj->SetGoalAcceleration(accf);
      break;
    }
  }
  traj->Generate(execution_time);

  RapidTrajectoryGenerator::InputFeasibilityResult res = traj->CheckInputFeasibility(
      min_normalized_thrust_,
      max_normalized_thrust_,
      max_roll_pitch_rate_,
      min_traj_sampling_time_);
  if (res == RapidTrajectoryGenerator::InputFeasible) {
    // check also for floor/ceiling feasibility
    Vec3 boundary_point = Vec3(0.0, 0.0, traj_min_z_);  // a point on the floor
    Vec3 boundary_normal = Vec3(0.0, 0.0, 1.0);  // we want to be in this direction of the point (upwards)
    RapidTrajectoryGenerator::StateFeasibilityResult positionFeasible_floor = traj->CheckPositionFeasibility(
        boundary_point, boundary_normal);
    boundary_point = Vec3(0.0, 0.0, traj_max_z_);  // a point on the floor
    boundary_normal = Vec3(0.0, 0.0, -1.0);  // we want to be in this direction of the point (upwards)
    RapidTrajectoryGenerator::StateFeasibilityResult positionFeasible_ceiling = traj->CheckPositionFeasibility(
        boundary_point, boundary_normal);
    if (positionFeasible_ceiling == RapidTrajectoryGenerator::StateFeasible &&
        positionFeasible_floor == RapidTrajectoryGenerator::StateFeasible) {
      if (is_mb) {
        trajectories_mb_.push_back(std::move(traj));
      } else {
        trajectories_nw_.push_back(std::move(traj));
      }
      success = true;
    }
  } else {
    ROS_WARN("[%s] Trajectory not input feasible.", ros::this_node::getName().c_str());
  }

  if (!success) {
    ROS_WARN("[%s] Failed to calculate trajectory.", ros::this_node::getName().c_str());
    ROS_WARN("[%s] Start: [%f, %f, %f].", ros::this_node::getName().c_str(), start_state.position.x(),
             start_state.position.y(), start_state.position.z());
    ROS_WARN("[%s] End: [%f, %f, %f].", ros::this_node::getName().c_str(), end_state.position.x(),
             end_state.position.y(), end_state.position.z());
    ROS_WARN("[%s] Desired speed [%f].", ros::this_node::getName().c_str(), desired_speed);
    ROS_WARN("[%s] Execution time [%f].", ros::this_node::getName().c_str(), execution_time);
  }
  return success;
}

quadrotor_common::TrajectoryPoint MinimumJerkTrajectory::getDesiredStateFromTrajectory() {
  quadrotor_common::TrajectoryPoint state;
  double t = (ros::Time::now() - time_start_best_trajectory_).toSec() + 0.02;
  //
  Vec3 position = best_traj_->GetPosition(t);
  Vec3 velocity = best_traj_->GetVelocity(t);
  Vec3 acceleration = best_traj_->GetAcceleration(t);

  state.position = Eigen::Vector3d(position.x, position.y, position.z);
  state.velocity = Eigen::Vector3d(velocity.x, velocity.y, velocity.z);
  state.acceleration = Eigen::Vector3d(acceleration.x, acceleration.y, acceleration.z);

  state.heading = std::atan2(state.velocity.y(), state.velocity.x());
  return state;
}

quadrotor_common::TrajectoryPoint MinimumJerkTrajectory::getDesiredStateFromTrajectory(const double t) {
  quadrotor_common::TrajectoryPoint state;

  Vec3 position = best_traj_->GetPosition(t);
  Vec3 velocity = best_traj_->GetVelocity(t);
  Vec3 acceleration = best_traj_->GetAcceleration(t);

  state.position = Eigen::Vector3d(position.x, position.y, position.z);
  state.velocity = Eigen::Vector3d(velocity.x, velocity.y, velocity.z);
  state.acceleration = Eigen::Vector3d(acceleration.x, acceleration.y, acceleration.z);

  state.heading = std::atan2(state.velocity.y(), state.velocity.x());
  return state;
}

void MinimumJerkTrajectory::clearTrajectories(const bool is_mb) {
  if (is_mb) {
    trajectories_mb_.clear();
  } else {
    trajectories_nw_.clear();
  }
}

double MinimumJerkTrajectory::getEndTimeFromTrajectory() {
  return best_traj_->GetEndTime();
}

void MinimumJerkTrajectory::setBestTrajectory(const int traj_idx, const bool is_mb) {
  if (is_mb) {
    best_traj_ = trajectories_mb_.at(traj_idx);
  } else {
    best_traj_ = trajectories_nw_.at(traj_idx);
  }
}

} /* namespace drone_racing */
