#pragma once

#include <mutex>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <quadrotor_common/trajectory_point.h>
#include <quadrotor_msgs/TrajectoryPoint.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "drone_racing/data_saver.h"

namespace drone_racing {

    class TrajectoryBase {
    public:
        TrajectoryBase(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

        TrajectoryBase() :
                TrajectoryBase(ros::NodeHandle(), ros::NodeHandle("~")) {
        }

        virtual ~TrajectoryBase();

        enum class EnforceEndOfTraj {
            kPos, kPosVel, kPosVelAcc
        };

        virtual void clearTrajectories(const bool is_mb) = 0;

        virtual quadrotor_common::TrajectoryPoint getDesiredStateFromTrajectory() = 0;

        virtual quadrotor_common::TrajectoryPoint getDesiredStateFromTrajectory(const double t) = 0;

        virtual double getEndTimeFromTrajectory() = 0;

        virtual bool computeTrajectory(const quadrotor_common::TrajectoryPoint &start_state,
                                       const quadrotor_common::TrajectoryPoint &end_state,
                                       const EnforceEndOfTraj end_constraint,
                                       const double &desired_speed,
                                       const bool is_mb) = 0;

        virtual void setBestTrajectory(const int traj_idx, const bool is_mb) = 0;

        void setStartTime();

    protected:
        ros::Time time_start_best_trajectory_;
        // Rapid Trajectory Parameters
        double min_normalized_thrust_;
        double max_normalized_thrust_;
        double max_roll_pitch_rate_;
        double min_traj_sampling_time_;

        double traj_max_z_;
        double traj_min_z_;
        double max_velocity_;

    private:
        std::unique_ptr<drone_racing::TrajectoryBase> trajectory_implementation_;
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        double yaw_trajectory_;

        void loadParameters();
    };

} /* namespace drone_racing */
