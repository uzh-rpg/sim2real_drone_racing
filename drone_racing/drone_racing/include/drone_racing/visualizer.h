#pragma once

#include <ros/ros.h>

#include <quadrotor_common/trajectory_point.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>

#include "rapid_trajectories/Vec3.h"

namespace drone_racing {

class Visualizer {
public:
  Visualizer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  Visualizer() :
      Visualizer(ros::NodeHandle(), ros::NodeHandle("~")) {
  }

  virtual ~Visualizer();

  enum class Color {
    kRed, kGreen, kBlue, kYellow, kPurple, kWhite, kBlack
  };

  void displayGoalMarker(
      const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& goal_positions);

  void
  displayDebug(const double& goal_x, const double& goal_y, const double& goal_z, int id, Color color);

  void create_vehicle_markers(int num_rotors, float arm_len, float body_width, float body_height);

  void visualizeTrajectory(std::vector<quadrotor_common::TrajectoryPoint> trajectory,
                           bool is_model_based_traj, Color color,
                           int orientation_index);

  void displayQuadrotor();

private:
  void loadParameters();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher goal_marker_pub_;
  ros::Publisher debug_marker_pub_;
  ros::Publisher vehicle_marker_pub_;
  ros::Publisher trajectory_cnn_viz_pub_;
  ros::Publisher trajectory_center_viz_pub_;
  ros::Publisher trajectory_global_viz_pub_;
  ros::Publisher gate_pub_;

  int traj_marker_id_;
  std::shared_ptr<visualization_msgs::MarkerArray> vehicle_marker_;
  std::string child_frame_id_;
  double marker_scale_ = 0.5;
  bool show_all_trajectories_;
};

} /* namespace drone_racing */
