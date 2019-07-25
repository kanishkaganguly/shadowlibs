//
// Created by kganguly on 5/3/19.
//

#pragma once

#include <shadowlibs/shadow_finger.hpp>
//#include <shadowlibs/shadow_hand.hpp>
#include <shadowlibs/shadow_utils.hpp>
#include <future>

namespace shadow_planning {
    /* Planning functions */
    geometry_msgs::Pose
    getRandomPose(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::string &frame_name);

    bool planToNamedTarget(shadow_planning::PlanningOptions &options,
                           moveit::planning_interface::MoveGroupInterface &move_group_interface,
                           std::string &target_name,
                           moveit::planning_interface::MoveGroupInterface::Plan &plan);

    bool planToNamedTarget(shadow_finger::Finger &finger, std::string &target_name);

    bool planToPoseTarget(shadow_planning::PlanningOptions &options,
                          moveit::planning_interface::MoveGroupInterface &move_group_interface,
                          geometry_msgs::Pose &target_pose, std::string &reference_frame,
                          moveit::planning_interface::MoveGroupInterface::Plan &plan,
                          std::string &end_effector_name);

    bool planToJointTargets(shadow_planning::PlanningOptions &options,
                            moveit::planning_interface::MoveGroupInterface &move_group_interface,
                            std::string &reference_frame,
                            moveit::planning_interface::MoveGroupInterface::Plan &plan,
                            std::map<std::string, double> &joint_targets);

    bool executePlan(std::vector <ros::Publisher> &controller_pubs,
                     moveit::planning_interface::MoveGroupInterface::Plan &plan);

    bool executePlan(shadow_finger::Finger &finger);

    void executePlanAsync(shadow_finger::Finger &finger);

//    void executePlanAsync(shadow_hand::Hand &hand);
};
