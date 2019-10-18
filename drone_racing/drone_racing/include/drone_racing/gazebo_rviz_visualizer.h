#pragma once

#include <string.h>

#include <Eigen/Dense>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_common/geometry_eigen_conversions.h>
#include <quadrotor_common/math_common.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace gazebo_rviz_visualizer {

class GazeboRvizVisualizer {
public:
  GazeboRvizVisualizer(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private);

  GazeboRvizVisualizer() :
      GazeboRvizVisualizer(ros::NodeHandle(), ros::NodeHandle("~")) {
  }

  ~GazeboRvizVisualizer();

  void visualizeGates();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber gazebo_model_states_sub_;

  void gazeboModelStatesSub(const gazebo_msgs::ModelStates::ConstPtr& msg);

  void loadParameters();

  std::vector<Eigen::Vector3d> gates_positions_;
  std::vector<Eigen::Quaterniond> gates_orientations_;
  std::vector<std::string> gates_meshes_;

  int total_number_of_gates_;

  ros::Publisher gate_publisher_;

  gazebo_msgs::ModelStates model_states_;

};
}
