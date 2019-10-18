#pragma once

#include <mutex>
#include <ros/ros.h>

#include <quadrotor_common/trajectory_point.h>

#include "rapid_trajectories/RapidTrajectoryGenerator.h"
#include "rapid_trajectories/Vec3.h"
#include "drone_racing/trajectory_base.h"

using namespace RapidQuadrocopterTrajectoryGenerator;

namespace drone_racing {

    class MinimumJerkTrajectory : public TrajectoryBase {
    public:
        virtual void clearTrajectories(const bool is_mb) override;

        virtual quadrotor_common::TrajectoryPoint getDesiredStateFromTrajectory() override;

        virtual quadrotor_common::TrajectoryPoint getDesiredStateFromTrajectory(const double t) override;

        virtual double getEndTimeFromTrajectory() override;

        virtual bool computeTrajectory(const quadrotor_common::TrajectoryPoint &start_state,
                                       const quadrotor_common::TrajectoryPoint &end_state,
                                       const EnforceEndOfTraj end_constraint,
                                       const double &desired_speed,
                                       const bool is_mb) override;

        virtual void setBestTrajectory(const int traj_idx, const bool is_mb) override;

    private:
        std::vector<std::shared_ptr<RapidTrajectoryGenerator> > trajectories_mb_;
        std::vector<std::shared_ptr<RapidTrajectoryGenerator> > trajectories_nw_;
        std::shared_ptr<RapidTrajectoryGenerator> best_traj_;
    };

} /* namespace drone_racing */
