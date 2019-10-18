#include "drone_racing/visualizer.h"

#include <quadrotor_common/geometry_eigen_conversions.h>
#include <quadrotor_common/math_common.h>
#include <quadrotor_common/parameter_helper.h>

namespace drone_racing {

Visualizer::Visualizer(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) :
    nh_(nh), pnh_(pnh), traj_marker_id_(0) {
  goal_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("goal_marker", 0);
  debug_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("debug", 0);
  vehicle_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("vehicle_marker", 10);
  trajectory_cnn_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("trajectories_cnn", 1);
  trajectory_center_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("trajectories_model_based", 1);
  trajectory_global_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("global_trajectory", 1);
  gate_pub_ = nh_.advertise<visualization_msgs::Marker>("gate", 1);

  loadParameters();
  int num_rotors = 4;
  float arm_len = 0.2;
  float body_width = 0.15;
  float body_height = 0.1;
  create_vehicle_markers(num_rotors, arm_len, body_width, body_height);
}

Visualizer::~Visualizer() = default;

void Visualizer::displayGoalMarker(
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &goal_positions) {
  // display goal in rviz
  for (int i = 0; i < goal_positions.size(); i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = goal_positions[i].x();
    marker.pose.position.y = goal_positions[i].y();
    marker.pose.position.z = goal_positions[i].z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 1.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    goal_marker_pub_.publish(marker);
  }
}

void Visualizer::displayDebug(const double &goal_x, const double &goal_y, const double &goal_z, const int id,
                              const Color color) {
  // display goal in rviz
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = goal_x;
  marker.pose.position.y = goal_y;
  marker.pose.position.z = goal_z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 1.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  switch (color) {
    case Color::kRed: {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      break;
    }
    case Color::kBlue: {
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      break;
    }
    case Color::kBlack: {
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      break;
    }
    case Color::kGreen: {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      break;
    }
    case Color::kPurple: {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      break;
    }
    case Color::kWhite: {
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      break;
    }
    case Color::kYellow: {
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      break;
    }
  }
  marker.color.a = 1.0;
  debug_marker_pub_.publish(marker);
}

void Visualizer::visualizeTrajectory(const std::vector<quadrotor_common::TrajectoryPoint> trajectory,
                                     const bool is_model_based_traj,
                                     const Color color,
                                     const int orientation_index) {
  // send visualization markers
  visualization_msgs::Marker msg;

  msg.header.frame_id = "world";
  msg.pose.orientation.w = 1.0;
  msg.scale.x = 0.05;
  if(show_all_trajectories_) {
    msg.id = ++traj_marker_id_;
  } else{
    msg.id = 1.0;
  }
    msg.color.a = 1.0;

  switch (color) {
    case Color::kRed: {
      msg.color.r = 1.0;
      msg.color.g = 0.0;
      msg.color.b = 0.0;
      break;
    }
    case Color::kBlue: {
      msg.color.r = 0.0;
      msg.color.g = 0.0;
      msg.color.b = 1.0;
      break;
    }
    case Color::kBlack: {
      msg.color.r = 0.0;
      msg.color.g = 0.0;
      msg.color.b = 0.0;
      break;
    }
    case Color::kGreen: {
      msg.color.r = 0.0;
      msg.color.g = 1.0;
      msg.color.b = 0.0;
      break;
    }
    case Color::kPurple: {
      msg.color.r = 1.0;
      msg.color.g = 0.0;
      msg.color.b = 1.0;
      break;
    }
    case Color::kWhite: {
      msg.color.r = 1.0;
      msg.color.g = 1.0;
      msg.color.b = 1.0;
      break;
    }
    case Color::kYellow: {
      msg.color.r = 1.0;
      msg.color.g = 1.0;
      msg.color.b = 0.0;
      break;
    }
  }
  msg.type = visualization_msgs::Marker::LINE_LIST;

  for (auto state : trajectory) {
    msg.points.push_back(quadrotor_common::vectorToPoint(quadrotor_common::eigenToGeometry(state.position)));
    msg.points.push_back(
        quadrotor_common::vectorToPoint(quadrotor_common::eigenToGeometry(state.position + (state.velocity) / 20.0)));
  }
  if (!is_model_based_traj) {
    trajectory_cnn_viz_pub_.publish(msg);
  } else {
    switch (orientation_index) {
      case 0: {
        trajectory_center_viz_pub_.publish(msg);
        break;
      }
      case 3: {
        msg.id = 0;
        trajectory_global_viz_pub_.publish(msg);
        break;
      }
    }
  }
}

void Visualizer::displayQuadrotor() {
    vehicle_marker_pub_.publish(*vehicle_marker_);
}

void Visualizer::create_vehicle_markers(int num_rotors, float arm_len, float body_width, float body_height) {
  if (num_rotors <= 0) num_rotors = 2;

  if (vehicle_marker_)
    return;

  vehicle_marker_ = std::make_shared<visualization_msgs::MarkerArray>();
  vehicle_marker_->markers.reserve(2 * num_rotors + 1);
  //child_frame_id_ = "hawk_corrected";
  // rotor marker template
  visualization_msgs::Marker rotor;
  rotor.header.stamp = ros::Time();
  rotor.header.frame_id = child_frame_id_;
  rotor.ns = "vehicle_rotor";
  rotor.action = visualization_msgs::Marker::ADD;
  rotor.type = visualization_msgs::Marker::CYLINDER;
  rotor.scale.x = 0.2 * marker_scale_;
  rotor.scale.y = 0.2 * marker_scale_;
  rotor.scale.z = 0.01 * marker_scale_;
  rotor.color.r = 0.4;
  rotor.color.g = 0.4;
  rotor.color.b = 0.4;
  rotor.color.a = 0.8;
  rotor.pose.position.z = 0;

  // arm marker template
  visualization_msgs::Marker arm;
  arm.header.stamp = ros::Time();
  arm.header.frame_id = child_frame_id_;
  arm.ns = "vehicle_arm";
  arm.action = visualization_msgs::Marker::ADD;
  arm.type = visualization_msgs::Marker::CUBE;
  arm.scale.x = arm_len * marker_scale_;
  arm.scale.y = 0.02 * marker_scale_;
  arm.scale.z = 0.01 * marker_scale_;
  arm.color.r = 0.0;
  arm.color.g = 0.0;
  arm.color.b = 1.0;
  arm.color.a = 1.0;
  arm.pose.position.z = -0.015 * marker_scale_;

  float angle_increment = 2 * M_PI / num_rotors;

  for (float angle = angle_increment / 2; angle <= (2 * M_PI); angle += angle_increment) {
    rotor.pose.position.x = arm_len * cos(angle) * marker_scale_;
    rotor.pose.position.y = arm_len * sin(angle) * marker_scale_;
    rotor.id++;

    arm.pose.position.x = rotor.pose.position.x / 2;
    arm.pose.position.y = rotor.pose.position.y / 2;
    arm.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
    arm.id++;

    vehicle_marker_->markers.push_back(rotor);
    vehicle_marker_->markers.push_back(arm);
  }

  // body marker template
  visualization_msgs::Marker body;
  body.header.stamp = ros::Time();
  body.header.frame_id = child_frame_id_;
  body.ns = "vehicle_body";
  body.action = visualization_msgs::Marker::ADD;
  body.type = visualization_msgs::Marker::CUBE;
  body.scale.x = body_width * marker_scale_;
  body.scale.y = body_width * marker_scale_;
  body.scale.z = body_height * marker_scale_;
  body.color.r = 0.0;
  body.color.g = 1.0;
  body.color.b = 0.0;
  body.color.a = 0.8;

  vehicle_marker_->markers.push_back(body);
}

void Visualizer::loadParameters() {
  quadrotor_common::getParam<std::string>("quad_frame", child_frame_id_, "", pnh_);
  quadrotor_common::getParam<bool>("show_all_trajectories", show_all_trajectories_, true, pnh_);
}

}
