#include "drone_racing/gazebo_rviz_visualizer.h"

#include <quadrotor_common/parameter_helper.h>

namespace gazebo_rviz_visualizer {
GazeboRvizVisualizer::GazeboRvizVisualizer(const ros::NodeHandle& nh,
                                           const ros::NodeHandle& nh_private) :
    nh_(nh), pnh_(nh_private) {
  loadParameters();
  gazebo_model_states_sub_ = nh_.subscribe(
      "/gazebo/model_states", 1, &GazeboRvizVisualizer::gazeboModelStatesSub, this);

  gate_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("rviz_gates", 1);

  ROS_INFO("[%s] Gazebo to RVIZ visualizer started!", ros::this_node::getName().c_str());

  for (int i = 0; i < total_number_of_gates_; i++) {
    gates_meshes_.emplace_back("package://drone_racing/resources/race_track/real_world/gate/meshes/gate.dae");
  }
}

GazeboRvizVisualizer::~GazeboRvizVisualizer() = default;

void GazeboRvizVisualizer::gazeboModelStatesSub(const gazebo_msgs::ModelStates::ConstPtr& msg) {
  model_states_ = *msg;
}

void GazeboRvizVisualizer::visualizeGates() {
  gates_positions_.clear();
  gates_orientations_.clear();
  Eigen::Vector3d next_position;
  Eigen::Quaterniond next_orientation;
  gazebo_msgs::ModelStates model_states_temp = model_states_;
  for (std::size_t i = 0, max = model_states_temp.name.size(); i < max; i++) {

    if (model_states_temp.name[i].find("gate") != std::string::npos) {

      next_position = quadrotor_common::geometryToEigen(
          model_states_temp.pose[i].position);

      gates_positions_.push_back(next_position);

      next_orientation = quadrotor_common::geometryToEigen(
          model_states_temp.pose[i].orientation);
      gates_orientations_.push_back(next_orientation);
    }
  }

  visualization_msgs::MarkerArray msg;
  for (size_t i = 0; i < gates_positions_.size(); i++) {
    visualization_msgs::Marker gate_marker;
    gate_marker.header.frame_id = "world";

    gate_marker.pose.position = quadrotor_common::vectorToPoint(
        quadrotor_common::eigenToGeometry(gates_positions_[i]));

    gate_marker.pose.orientation = quadrotor_common::eigenToGeometry(
        gates_orientations_[i]);

    gate_marker.scale.x = gate_marker.scale.y = gate_marker.scale.z = 1.0;
    gate_marker.action = visualization_msgs::Marker::ADD;
    gate_marker.type = visualization_msgs::Marker::MESH_RESOURCE;

    gate_marker.mesh_resource = gates_meshes_[i];

    gate_marker.mesh_use_embedded_materials = true;
    gate_marker.id = i;
    gate_marker.color.a = 1.0;
    gate_marker.color.r = 217.0 / 255.0;
    gate_marker.color.g = 100.0 / 255.0;
    gate_marker.color.b = 30.0 / 255.0;
    gate_marker.frame_locked = true;

    msg.markers.push_back(gate_marker);
  }

  gate_publisher_.publish(msg);
}

void GazeboRvizVisualizer::loadParameters() {
  quadrotor_common::getParam<int>("num_of_gates", total_number_of_gates_, 1, pnh_);
}

} // namespace
