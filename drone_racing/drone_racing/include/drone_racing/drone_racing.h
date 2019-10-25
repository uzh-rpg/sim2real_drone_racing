#pragma once

#include <Eigen/Dense>
#include <mutex>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <kindr/minimal/quat-transformation.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_common/trajectory_point.h>
#include <quadrotor_msgs/AutopilotFeedback.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

#include "drone_racing/data_saver.h"
#include "drone_racing/gate_replacer.h"
#include "drone_racing/gazebo_rviz_visualizer.h"
#include "drone_racing/global_trajectory.h"
#include "drone_racing/trajectory_manager.h"
#include "drone_racing/visualizer.h"

using namespace RapidQuadrocopterTrajectoryGenerator;

namespace rpg {

	typedef kindr::minimal::QuatTransformation Pose;
	typedef kindr::minimal::RotationQuaternion Rotation;
	typedef Eigen::Vector3d Position;

}  // namespace rpg_common

namespace drone_racing {

class DroneRacing {
public:
  DroneRacing(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  DroneRacing() :
      DroneRacing(ros::NodeHandle(), ros::NodeHandle("~")) {
  }

  virtual ~DroneRacing();

  Eigen::Vector3d getEstimatedPos();

  std::shared_ptr<Visualizer> visualizer_;
  std::shared_ptr<DataSaver> data_saver_;
  std::shared_ptr<TrajectoryManager> trajectory_manager_;
  std::shared_ptr<GlobalTrajectory> global_trajectory_;
  std::shared_ptr<gate_replacer::GateReplacer> gate_replacer_;
  std::shared_ptr<gazebo_rviz_visualizer::GazeboRvizVisualizer> gazebo_rviz_visualizer_;

private:
  enum class State {
    kOff, kHover, kWaitingForFeedthroughActivated, kRacing
  };

  void mainloop(const ros::TimerEvent& time);

  void updateStateMachine();

  void performNavigation();

  void startNavigationCallback(const std_msgs::EmptyConstPtr& msg);

  void hardStopCallback(const std_msgs::EmptyConstPtr& msg);

  void stateEstimateCallback(const nav_msgs::OdometryConstPtr& msg);

  void networkCallback(const geometry_msgs::TwistStampedConstPtr& msg);

  void enableTestTimeCallback(const std_msgs::BoolConstPtr& msg);

  void publishDesiredState();

  bool setGoalWasTriggered() const;

  bool checkStateEstDesStateDivergence();

  void resetDesiredState();

  void loadParameters();

  void setRunIdxCallback(const std_msgs::Int16ConstPtr& msg);
  
  void setupEnvironmentCallback(const std_msgs::EmptyConstPtr& msg);

  void transformToQuadFrame();

  void transformToWorldFrame();

  void transformStateEstimateToWorldFrame();

  quadrotor_common::TrajectoryPoint getEndState();

  double getDesiredVelocity();

  void updateWaypointGoals();

  double constrain(const double& prev_value, const double& new_value, const double& threshold);

  void executeTestTime();

  void collectData();

  Eigen::Vector3d network_selection_ = Eigen::Vector3d(0.0, 0.0, 1.0);
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber copilot_feedback_sub_;
  ros::Subscriber state_estimate_sub_;
  ros::Subscriber feedthrough_sub_;
  ros::Subscriber start_navigation_sub_;
  ros::Subscriber soft_stop_sub_;
  ros::Subscriber hard_stop_sub_;
  ros::Subscriber cnn_sub_;
  ros::Subscriber test_time_sub_;
  ros::Subscriber run_idx_sub_;
  ros::Subscriber env_setup_sub_;

  ros::Publisher crashed_pub_;
  ros::Publisher passed_gate_pub_;
  ros::Publisher feedthrough_pub_;
  ros::Publisher desired_state_pub_;
  ros::Publisher divergence_pub_;

  ros::Timer main_loop_timer_;
  tf::TransformListener tf_listener_;
  ros::WallTime start_time_;

  rpg::Pose T_S_Q_;
  rpg::Pose T_W_Q_;
  rpg::Pose T_W_S_;

  quadrotor_common::TrajectoryPoint desired_state_quad_;
  quadrotor_common::TrajectoryPoint desired_state_world_;

  nav_msgs::Odometry state_estimate_quad_;
  nav_msgs::Odometry state_estimate_world_;
  bool environment_is_set_up_;
  State state_machine_;

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > goal_positions_;
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > gate_positions_;

  bool copilot_is_in_feedthrough_;
  bool goal_selected_;
  double gate_height_;
  double distance_to_goal_;
  int failed_trials_;
  bool moving_gates_;
  int curr_goal_index_ = 0;
  int last_goal_index_ = 0;
  double d_replan_;
  double camera_semi_fov_rad_yaw_;
  double camera_semi_fov_rad_pitch_;
  double traj_max_z_;
  double max_velocity_;
  double min_velocity_;
  double max_divergence_;
  int max_failed_trials_;
  double max_error_to_global_traj_start_;
  double max_error_to_global_traj_end_;
  bool test_time_ = false;
  int mainloop_iter_;
  int plan_every_n_iter_;
  int asked_teacher_n_times_ = 0;
  bool directory_created_ = false;
  bool record_data_;
  int model_based_frame_;
  int executed_nn_traj_;
  double sec_no_record_at_start_;
  std::string quad_frame_;
  double horizon_min_;
  double prev_des_velocity_;
  bool perturb_actions_;
  double gates_static_amp_;
};

} // namespace drone_racing
