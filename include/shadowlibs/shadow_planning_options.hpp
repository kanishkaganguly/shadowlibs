//
// Created by kganguly on 5/8/19.
//

#pragma once

namespace shadow_planning {
/* Planning options */
    struct PlanningOptions {
        double set_planning_time;
        int num_attempts;
        bool allow_replanning;
        double goal_position_tolerance;
        double goal_orientation_tolerance;
        double goal_joint_tolerance;
        std::string end_effector_name;

        PlanningOptions() : set_planning_time(2.0),
                            allow_replanning(false),
                            num_attempts(1),
                            end_effector_name(""),
                            goal_position_tolerance(0.01),
                            goal_orientation_tolerance(0.01),
                            goal_joint_tolerance(0.01) {};

        PlanningOptions(double set_planning_time,
                        bool allow_replanning,
                        int num_attempts,
                        double goal_position_tolerance,
                        double goal_orientation_tolerance,
                        double goal_joint_tolerance,
                        std::string end_effector_name) :
                set_planning_time(set_planning_time),
                allow_replanning(allow_replanning),
                num_attempts(num_attempts),
                goal_position_tolerance(goal_position_tolerance),
                goal_orientation_tolerance(goal_orientation_tolerance),
                goal_joint_tolerance(goal_joint_tolerance) {};
    };
}