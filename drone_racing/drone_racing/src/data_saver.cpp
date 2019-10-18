#include "drone_racing/data_saver.h"

#include <Eigen/Dense>
#include <fstream>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include <quadrotor_common/parameter_helper.h>
#include <visualization_msgs/MarkerArray.h>

namespace drone_racing {

DataSaver::DataSaver(const ros::NodeHandle& nh, const ros::NodeHandle& pnh) :
    nh_(nh), pnh_(pnh), w_(0), h_(0), cam_frame_("") {
  loadParameters();

  state_estimate_sub_ = nh_.subscribe("state_estimate", 1, &DataSaver::stateEstimateCallback, this);
  image_sub_ = nh_.subscribe("image_rgb", 1, &DataSaver::imageCallback, this);
  run_idx_sub_ = nh_.subscribe("run_idx", 1, &DataSaver::setRunIdxCallback, this);
  image_pub_ = nh_.advertise<sensor_msgs::Image>("image_with_prediction", 1);
}

DataSaver::~DataSaver() = default;

void DataSaver::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImageConstPtr cv_ptr;
  cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_8UC3);

  mtx_image_save_.lock();
  image_ = cv_ptr->image.clone();
  addPredictionAndRepublish();
  mtx_image_save_.unlock();
}

void DataSaver::setModelBasedPrediction(Eigen::Vector3d prediction) {
  model_based_prediction_ = prediction;
}

void DataSaver::setNetworkSelection(Eigen::Vector3d selection) {
  network_selection_ = selection;
}

void DataSaver::addPredictionAndRepublish() {
  cv::Mat imageRaw = image_.clone();
  cv::Mat image;
  // shrink image
  // get image dimensions
  int border_top_bottom = (int) (imageRaw.rows / 2.0 * (camera_fov_deg_pitch_ / 35.0 - 1.0));
  int border_left_right = (int) (imageRaw.cols / 2.0 * (camera_fov_deg_yaw_ / 50.0 - 1.0));
  int borderType = cv::BORDER_CONSTANT;
  cv::Scalar value = cv::Scalar(50.0, 50.0, 50.0);

  cv::copyMakeBorder(imageRaw, image, border_top_bottom, border_top_bottom, border_left_right, border_left_right,
                     borderType, value);
  // draw some grid lines
  for (int i = 0; i <= camera_fov_deg_yaw_; i += 30) {
    int yaw_px = (int) (i / ((float) camera_fov_deg_yaw_) * image.cols / 2.0 + image.cols / 2.0);
    cv::line(image, cv::Point(yaw_px, 0), cv::Point(yaw_px, image.rows), cv::Scalar(100, 100, 100, 0));
    yaw_px = (int) (-i / ((float) camera_fov_deg_yaw_) * image.cols / 2.0 + image.cols / 2.0);
    cv::line(image, cv::Point(yaw_px, 0), cv::Point(yaw_px, image.rows), cv::Scalar(100, 100, 100, 0));
  }

  for (int i = 0; i <= camera_fov_deg_pitch_; i += 30) {
    int pitch_px = (int) (i / ((float) camera_fov_deg_pitch_) * image.rows / 2.0 + image.rows / 2.0);
    cv::line(image, cv::Point(0, pitch_px), cv::Point(image.cols, pitch_px), cv::Scalar(100, 100, 100, 0));
    pitch_px = (int) (-i / ((float) camera_fov_deg_pitch_) * image.rows / 2.0 + image.rows / 2.0);
    cv::line(image, cv::Point(0, pitch_px), cv::Point(image.cols, pitch_px), cv::Scalar(100, 100, 100, 0));
  }

  int rows = image.rows;
  int cols = image.cols;
  int up_coor_mb = static_cast<int>(rows * (1.0 / 2.0 - model_based_prediction_.y() / 2.0));
  int right_coor_mb = static_cast<int>(cols * (1.0 / 2.0 + model_based_prediction_.x() / 2.0));
  int up_coor_nw = static_cast<int>(rows * (1.0 / 2.0 - network_selection_.y() / 2.0));
  int right_coor_nw = static_cast<int>(cols * (1.0 / 2.0 + network_selection_.x() / 2.0));

  // draw boxes behind velocity info to make it better readable
  cv::Mat roi = image(cv::Rect(3, image.rows - 25, 85, 20));
  cv::Mat color(roi.size(), CV_8UC3, cv::Scalar(125, 125, 125));
  double alpha = 0.8;
  cv::addWeighted(color, alpha, roi, 1.0 - alpha, 0.0, roi);

  cv::Mat roi2 = image(cv::Rect(image.cols - 88, image.rows - 25, 85, 20));
  cv::addWeighted(color, alpha, roi2, 1.0 - alpha, 0.0, roi2);

  char v_nw[200];
  sprintf(v_nw, "%f ", network_selection_.z());
  cv::putText(image, v_nw, cv::Point2f(5, image.rows - 10), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0, 255));

  char v_mb[200];
  sprintf(v_mb, "%f ", model_based_prediction_.z());
  cv::putText(image, v_mb, cv::Point2f(image.cols - 80, image.rows - 10), cv::FONT_HERSHEY_PLAIN, 1,
              cv::Scalar(255, 0, 0, 255));

  cv::circle(image, cv::Point(right_coor_mb, up_coor_mb), 0, cv::Scalar(255, 0, 0), 5, 8, 0);
  cv::circle(image, cv::Point(right_coor_nw, up_coor_nw), 0, cv::Scalar(0, 255, 0), 5, 8, 0);

  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = ros::Time::now(); // Same timestamp and tf frame as input image
  out_msg.encoding = "rgb8";
  out_msg.image = image;

  image_pub_.publish(out_msg.toImageMsg());
}

bool DataSaver::createDirectory() {
  if (!created_directory_) {
    std::ostringstream ss_run;
    ss_run << std::setw(4) << std::setfill('0') << run_idx_;
    std::string s_run(ss_run.str());
    // generate new directory for experimental data
    std::string folder_name = root_dir_ + "/Run_" + s_run + "/images";
    std::string directory_cmd = "mkdir -p " + folder_name;

    ROS_INFO("Creating directory %s", folder_name.c_str());

    const int dir_err = system(directory_cmd.c_str());
    if (-1 == dir_err) {
      ROS_WARN("[%s] Error creating directory!", ros::this_node::getName().c_str());
      return false;
    }
    frame_counter_ = 0;
    created_directory_ = true;
    return true;
  }
  ROS_WARN("[%s] Already created directory!", ros::this_node::getName().c_str());
  return true;
}

void DataSaver::setRunIdxCallback(const std_msgs::Int16ConstPtr& msg) {
  run_idx_ = msg->data;
  created_directory_ = false;
  ROS_INFO("[%s] Set run_idx to [%d].", ros::this_node::getName().c_str(), run_idx_);
}

void DataSaver::saveImage(const DataSaver::trajectory_info traj_coor,
                          const int model_based_frame) {
  if (record_data_) {
    // log state estimate
    std::ostringstream ss_run;
    ss_run << std::setw(4) << std::setfill('0') << run_idx_;
    std::string s_run(ss_run.str());

    // save image to file
    std::ostringstream ss;
    ss << std::setw(5) << std::setfill('0') << frame_counter_;
    std::string s2(ss.str());

    std::string filename_img = root_dir_ + "/Run_"
                               + s_run + "/images/frame_center_" + s2 + ".jpg";
    mtx_image_save_.lock();

    //convert from bgr to rgb
    cv::Mat image_save = image_.clone();
    cv::cvtColor(image_, image_save, CV_BGR2RGB);

    if (!cv::imwrite(filename_img, image_save)) {
      ROS_FATAL("Could not save image to location [%s].", filename_img.c_str());
    }
    mtx_image_save_.unlock();

    frame_counter_++;

    int new_dagger_batch = 0;
    if (model_based_frame == 0) {
      new_dagger_batch = 1;
    }
    // save label of best trajectory
    std::ofstream outfile;
    std::string filename_labels = root_dir_ + "/Run_"
                                  + s_run + "/labels.txt";
    outfile.open(filename_labels, std::ios_base::app);
    outfile << std::to_string(traj_coor.idx_right) + ";" << std::to_string(traj_coor.idx_up) + ";"
            << std::to_string(traj_coor.curr_vel) + ";" << std::to_string(traj_coor.max_vel) + ";"
            << std::to_string(new_dagger_batch) + "\n";
    outfile.close();
  }
}

void DataSaver::labelAsBadData() {
  // save label of best trajectory
  std::ofstream outfile;
  std::string filename_fails = root_dir_ + "/fails.txt";
  outfile.open(filename_fails, std::ios_base::app);
  outfile << std::to_string(run_idx_) + "\n";
  outfile.close();
}

void DataSaver::stopRecording() {
  record_data_ = false;
}

void DataSaver::startRecording() {
  record_data_ = true;
}

void DataSaver::stateEstimateCallback(const nav_msgs::OdometryConstPtr& msg) {
  state_estimate_ = *msg;
}

void DataSaver::loadParameters() {
  quadrotor_common::getParam<double>("max_velocity", max_velocity_, 1.0, pnh_);
  run_idx_ = 5000;
  quadrotor_common::getParam<std::string>("root_dir", root_dir_, "", pnh_);
  quadrotor_common::getParam<double>("gate_x", gate_x_, 0.0, pnh_);
  quadrotor_common::getParam<double>("gate_y", gate_y_, 0.0, pnh_);
  quadrotor_common::getParam<float>("gate_rotation", gate_rotation_, 0.0, pnh_);
  quadrotor_common::getParam<int>("environment_index", env_idx_, 1, pnh_);
  quadrotor_common::getParam<double>("camera_fov_yaw", camera_fov_deg_yaw_, 45.0, pnh_);
  quadrotor_common::getParam<double>("camera_fov_pitch", camera_fov_deg_pitch_, 45.0, pnh_);
}

}
