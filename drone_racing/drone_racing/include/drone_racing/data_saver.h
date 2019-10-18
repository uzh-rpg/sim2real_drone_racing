#pragma once

#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <ros/ros.h>

#include <mav_msgs/Actuators.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/ControlCommand.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int16.h>

namespace drone_racing {
class DataSaver {
 public:
  DataSaver(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

  DataSaver() :
      DataSaver(ros::NodeHandle(), ros::NodeHandle("~")) {
  }

  virtual ~DataSaver();

  struct trajectory_info {
    double idx_right;
    double idx_up;
    double curr_vel;
    double max_vel;
  };

  void saveImage(const DataSaver::trajectory_info goal_coor,
                 const int model_based_frame);

  void labelAsBadData();

  bool createDirectory();

  void setRunIdxCallback(const std_msgs::Int16ConstPtr &msg);

  void setModelBasedPrediction(Eigen::Vector3d prediction);
  void setNetworkSelection(Eigen::Vector3d selection);

  void startRecording();
  void stopRecording();

 private:
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);

  void stateEstimateCallback(const nav_msgs::OdometryConstPtr &msg);

  void addPredictionAndRepublish();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber state_estimate_sub_;
  ros::Subscriber motor_speed_sub_;
  ros::Subscriber control_command_sub_;
  ros::Subscriber cam_info_sub_;

  ros::Subscriber run_idx_sub_;
  ros::Subscriber image_sub_;

  ros::Publisher image_pub_;

  std::mutex mtx_image_save_;

  void loadParameters();

  nav_msgs::Odometry state_estimate_;
  mav_msgs::Actuators motor_speed_;
  quadrotor_msgs::ControlCommand control_command_;
  cv::Mat image_;

  int w_;
  int h_;
  Eigen::Matrix<double, 3, 3> K_;
  std::string cam_frame_;

  Eigen::Vector3d model_based_prediction_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d network_selection_ = Eigen::Vector3d::Zero();

  int run_idx_;
  int frame_counter_ = 0;

  std::string root_dir_;
  double gate_x_;
  double gate_y_;
  float gate_rotation_;
  bool created_directory_ = false;

  double max_velocity_;
  bool record_data_ = false;
  int env_idx_;
  double camera_fov_deg_yaw_;
  double camera_fov_deg_pitch_;
};

} /* namespace drone_racing */
