//
// Created by kganguly on 5/3/19.
//

#pragma once

#include <shadowlibs/shadow_utils.hpp>

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

    std::string test();

    /* Planning functions */
    geometry_msgs::Pose
    getRandomPose(moveit::planning_interface::MoveGroupInterface& move_group_interface, const std::string frame_name);

    bool getPlanToNamedTarget(shadow_planning::PlanningOptions& options,
                              moveit::planning_interface::MoveGroupInterface& move_group_interface,
                              std::string& target_name,
                              moveit::planning_interface::MoveGroupInterface::Plan& plan);

    bool getPlanToPoseTarget(shadow_planning::PlanningOptions& options,
                             moveit::planning_interface::MoveGroupInterface& move_group_interface,
                             geometry_msgs::Pose& target_pose, const std::string reference_frame,
                             moveit::planning_interface::MoveGroupInterface::Plan& plan,
                             const std::string end_effector_name);

    bool getPlanToJointTargets(shadow_planning::PlanningOptions& options,
                               moveit::planning_interface::MoveGroupInterface& move_group_interface,
                               const std::string reference_frame,
                               moveit::planning_interface::MoveGroupInterface::Plan& plan,
                               std::map<std::string, double>& joint_targets);

    bool executePlan(std::vector <ros::Publisher>& controller_pubs,
                     moveit::planning_interface::MoveGroupInterface::Plan& plan);


};
