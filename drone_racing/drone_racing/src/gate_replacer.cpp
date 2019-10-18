#include "drone_racing/gate_replacer.h"

#include <gazebo_msgs/SetModelState.h>
#include <quadrotor_common/parameter_helper.h>

namespace gate_replacer {
GateReplacer::GateReplacer(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) :
    nh_(nh), pnh_(nh_private) {
  loadParameters();
  loadGoalPositions();
  gazebo_model_states_sub_ = nh_.subscribe(
      "/gazebo/model_states", 1, &GateReplacer::gazeboModelStatesSub,
      this);

  replace_gate_sub_ = nh_.subscribe("/replace_gates", 1, &GateReplacer::replaceGatesCallback, this);
  replace_gate_pub_ = nh_.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 0);

  ROS_INFO("[%s] Gate replacer started!",
           ros::this_node::getName().c_str());

  // initialize random phase shifts for each gate
  for (int i = 0; i < total_number_of_gates_; i++) {
    double random_phase_x = 3.141592 * (((double) rand() / (RAND_MAX)) + 0.5);
    double random_phase_y = 3.141592 * (((double) rand() / (RAND_MAX)));
    double random_phase_z = 3.141592 * (((double) rand() / (RAND_MAX)) - 0.5);

    random_phases_.emplace_back(Eigen::Vector3d(random_phase_x,
                                                random_phase_y,
                                                random_phase_z));
  }
}

void GateReplacer::replaceGatesCallback(const std_msgs::EmptyConstPtr &msg) {
  replaceGates();
}

void GateReplacer::replaceGatesExplicit() {
  set_start_time_ = false;
  replaceGates();
}

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > GateReplacer::getGoalPositions() {
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > goal_positions;

  for (auto &i : gates_positions_temp_) {
    goal_positions.emplace_back(Eigen::Vector3d(i.x(),
                                                i.y(),
                                                i.z() + gate_height_));
  }
  return goal_positions;
}

std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> GateReplacer::getGatePositions() {

  return only_gates_;
}

void GateReplacer::replaceGates() {
  ROS_INFO("Replacing gates randomly with max %f meters in each direction.", max_static_amp_);
  srandom(static_cast<unsigned int>(ros::WallTime::now().toNSec()));
  double amplitude = max_static_amp_;
  int goal_idx = 0;
  gates_velocities_.clear();
  for (auto &i : model_states_msg_.name) {
    if (i.find("gate") != std::string::npos) {
        std::cout << "model_name: " << i << std::endl;
      gazebo_msgs::ModelState model_state;
      model_state.model_name = i;
      model_state.reference_frame = "world";

      model_state.pose.position.x = only_gates_[goal_idx].x() + 2 * amplitude * (((double) random() / (RAND_MAX)) - 0.5);
      model_state.pose.position.y = only_gates_[goal_idx].y() + 2 * amplitude * (((double) random() / (RAND_MAX)) - 0.5);
      model_state.pose.position.z = std::max(1.2, only_gates_[goal_idx].z() + 2 * amplitude * (((double) random() / (RAND_MAX)) - 0.5));

      gates_positions_temp_[goal_idx][0] = model_state.pose.position.x;
      gates_positions_temp_[goal_idx][1] = model_state.pose.position.y;
      gates_positions_temp_[goal_idx][2] = model_state.pose.position.z;

      model_state.pose.orientation.w = std::cos(only_gates_[goal_idx][3] / 2.0);
      model_state.pose.orientation.x = 0.0;
      model_state.pose.orientation.y = 0.0;
      model_state.pose.orientation.z = std::sin(only_gates_[goal_idx][3] / 2.0);
      model_state.twist.linear.x = 0.0;
      model_state.twist.linear.y = 0.0;
      model_state.twist.linear.z = 0.0;
      model_state.twist.angular.x = 0.0;
      model_state.twist.angular.y = 0.0;
      model_state.twist.angular.z = 0.0;

      goal_idx++;
      replace_gate_pub_.publish(model_state);
        ros::Duration(0.1).sleep();
    }
  }
  ROS_INFO("Finished replacing gates.");
}

void GateReplacer::gazeboModelStatesSub(const gazebo_msgs::ModelStates::ConstPtr &msg) {
  if (!found_all_gates_) {
    model_states_msg_ = *msg;
    unsigned int gates_found_so_far = 0;

    for (auto &i : model_states_msg_.name) {
      if (!found_all_gates_
          && i.find("gate") != std::string::npos) {
        gates_found_so_far++;
        if (gates_found_so_far == total_number_of_gates_) {
          ROS_INFO("[%s] All the gates have been found",
                   ros::this_node::getName().c_str());
          found_all_gates_ = true;
        }
      }
    }
  }
}

void GateReplacer::makeGatesMove() {
  int goal_idx = 0;
  if (!set_start_time_) {
    start_time_ = ros::WallTime::now();
    set_start_time_ = true;
  }
  for (auto &i : model_states_msg_.name) {
    if (i.find("gate") != std::string::npos) {
      gazebo_msgs::ModelState model_state;
      model_state.model_name = i;
      model_state.reference_frame = "world";
      double t = (ros::WallTime::now() - start_time_).toSec();
      gates_positions_temp_[goal_idx] = gates_positions_original_[goal_idx] +
          Eigen::Vector3d(max_dynamic_amp_ *
                              std::sin(speed_ * t + random_phases_[goal_idx].x()),
                          max_dynamic_amp_ *
                              std::sin(speed_ * t + random_phases_[goal_idx].y()),
                          0.5 * max_dynamic_amp_ *
                              std::sin(speed_ * t + random_phases_[goal_idx].z()));

      model_state.pose.position.x = gates_positions_temp_[goal_idx].x();
      model_state.pose.position.y = gates_positions_temp_[goal_idx].y();
      model_state.pose.position.z = gates_positions_temp_[goal_idx].z();
      model_state.pose.orientation.w = std::cos(gates_orientations_temp_[goal_idx] / 2.0);
      model_state.pose.orientation.x = 0.0;
      model_state.pose.orientation.y = 0.0;
      model_state.pose.orientation.z = std::sin(gates_orientations_temp_[goal_idx] / 2.0);

      model_state.twist.linear.x = 0.0;
      model_state.twist.linear.y = 0.0;
      model_state.twist.linear.z = 0.0;
      model_state.twist.angular.x = 0.0;
      model_state.twist.angular.y = 0.0;
      model_state.twist.angular.z = 0.0;

      goal_idx++;
      replace_gate_pub_.publish(model_state);
    }
  }
}

void GateReplacer::loadGoalPositions() {
  gates_positions_original_.clear();
  gates_orientations_original_.clear();
  gates_velocities_.clear();
  XmlRpc::XmlRpcValue raw_positions_list, raw_orientations_list;
  if (!pnh_.getParam("goal_positions", raw_positions_list)) {
    ROS_ERROR("[%s] Could not load goal positions list", pnh_.getNamespace().c_str());
  }
  if (!pnh_.getParam("goal_orientations", raw_orientations_list)) {
    ROS_ERROR("[%s] Could not load goal orientations list", pnh_.getNamespace().c_str());
  }

  for (int i = 0; i < raw_positions_list.size(); i++) {

    gates_positions_original_.emplace_back(
        Eigen::Vector3d(static_cast<double>(raw_positions_list[i]["x"]),
                        static_cast<double>(raw_positions_list[i]["y"]),
                        static_cast<double>(raw_positions_list[i]["z"])));

    gates_orientations_original_.push_back(static_cast<double>(raw_orientations_list[i]["yaw"]));
    ROS_INFO("Goal position: [%f, %f, %f][%f]", static_cast<double>(raw_positions_list[i]["x"]),
             static_cast<double>(raw_positions_list[i]["y"]),
             static_cast<double>(raw_positions_list[i]["z"]),
             static_cast<double>(raw_orientations_list[i]["yaw"]));

    gates_velocities_.emplace_back(Eigen::Vector3d::Zero());
    gates_positions_temp_ = gates_positions_original_;
    gates_orientations_temp_ = gates_orientations_original_;

    if (static_cast<double>(raw_positions_list[i]["gate"]) > 0.5) {
      // this is a real gate and not only a waypoint
      double yaw = static_cast<double>(raw_orientations_list[i]["yaw"]);
      Eigen::Vector2d pos_no_shift = Eigen::Vector2d(
          static_cast<double>(raw_positions_list[i]["x"]),
          static_cast<double>(raw_positions_list[i]["y"]));
      double offset = static_cast<double>(raw_orientations_list[i]["offset"]);
      Eigen::Vector2d offset_vec = Eigen::Vector2d(offset * std::cos(yaw + 1.57), offset * std::sin(yaw + 1.57));

      only_gates_.emplace_back(Eigen::Vector4d(
          pos_no_shift.x() + offset_vec.x(),
          pos_no_shift.y() + offset_vec.y(),
          static_cast<double>(raw_positions_list[i]["z"]),
          yaw
      ));
    }
  }
}

void GateReplacer::loadParameters() {
  quadrotor_common::getParam<int>("num_of_gates", total_number_of_gates_, 1, pnh_);
  quadrotor_common::getParam<double>("gates_dyn_amplitude", max_dynamic_amp_, 10, pnh_);
  quadrotor_common::getParam<double>("gates_static_amplitude", max_static_amp_, 1, pnh_);
  quadrotor_common::getParam<double>("speed_moving_gates", speed_, 1.0, pnh_);
  quadrotor_common::getParam<double>("gate_height", gate_height_, 1.0, pnh_);
}

} // namespace
