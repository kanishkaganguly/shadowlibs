//
// Created by kganguly on 4/30/19.
//

#include <shadowlibs/shadow_finger.hpp>

std::vector<double> shadow_finger::getJointValues(moveit::planning_interface::MoveGroupInterface& move_group_interface) {
    std::vector<double> joint_vals;
    joint_vals = move_group_interface.getCurrentJointValues();

    return joint_vals;
}

std::vector <std::string> shadow_finger::getJointNames(moveit::planning_interface::MoveGroupInterface& move_group_interface) {
    std::vector <std::string> joint_names;
    joint_names = move_group_interface.getJointNames();
    return joint_names;
}

std::vector <std::string> shadow_finger::getJointNames(moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    // This is the trajectory generated by MoveIt plan
    moveit_msgs::RobotTrajectory robotTrajectory = plan.trajectory_;
    // Convert MoveIt to trajectory_msgs joint trajectory
    trajectory_msgs::JointTrajectory planTrajectory =
            robotTrajectory.joint_trajectory;
    // Gives all the joint names in the planning group used when planning
    std::vector <std::string> jointNames = planTrajectory.joint_names;
    return jointNames;
}

shadow_finger::Finger::Finger(const std::string& finger_name) :
        _finger_name(finger_name),
        _finger_move_group(finger_name),
        _joints(shadow_finger::getJointNames(_finger_move_group)) {}