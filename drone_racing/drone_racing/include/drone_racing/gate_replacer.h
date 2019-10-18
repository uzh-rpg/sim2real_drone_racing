#pragma once

#include <cstring>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_common/geometry_eigen_conversions.h>
#include <quadrotor_common/math_common.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace gate_replacer {

class GateReplacer {
 public:
  GateReplacer(const ros::NodeHandle &nh,
               const ros::NodeHandle &nh_private);

  GateReplacer() :
      GateReplacer(ros::NodeHandle(), ros::NodeHandle("~")) {
  }

  ~GateReplacer() = default;

  void replaceGatesExplicit();

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > getGoalPositions();
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> getGatePositions();

  void makeGatesMove();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher replace_gate_pub_;
  ros::Subscriber gazebo_model_states_sub_;
  ros::Subscriber replace_gate_sub_;

  void gazeboModelStatesSub(const gazebo_msgs::ModelStates::ConstPtr &msg);

  void replaceGatesCallback(const std_msgs::EmptyConstPtr &msg);

  void replaceGates();

  void loadParameters();

  void loadGoalPositions();

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > gates_positions_original_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > gates_positions_temp_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > random_phases_;

  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > only_gates_;
  std::vector<double> gates_orientations_original_;
  std::vector<double> gates_orientations_temp_;

  gazebo_msgs::ModelStates model_states_msg_;

  bool found_all_gates_ = false;
  int total_number_of_gates_;

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > gates_velocities_;

  double max_dynamic_amp_;
  double max_static_amp_;
  double max_dynamic_acc_;
  double gate_height_;
  double speed_;
  ros::WallTime start_time_;
  bool set_start_time_ = false;
};
}
