//
// Created by kganguly on 4/30/19.
//

#pragma once

#include <shadowlibs/shadow_planning.hpp>

namespace shadow_finger {
    /* Get preloaded plans for open or close */
    std::string getSavedStateName(std::string finger_name, std::string saved_state);

    /* Get planning group names */
    std::string getMoveGroupName(std::string finger_name);

    /* Get joint names and values */
    std::vector<double> getJointValues(moveit::planning_interface::MoveGroupInterface& move_group_interface);

    std::vector <std::string> getJointNames(moveit::planning_interface::MoveGroupInterface& move_group_interface);

    std::vector <std::string> getJointNames(moveit::planning_interface::MoveGroupInterface::Plan& plan);

    // Manage fingers using separate classes
    struct Finger {
        // Finger name
        std::string _finger_name;
        // Set of joints in finger
        std::vector <std::string> _joints;
        // MoveIt planning group
        moveit::planning_interface::MoveGroupInterface _finger_move_group;
        // MoveIt plan for finger
        moveit::planning_interface::MoveGroupInterface::Plan _plan;
        // Planning options for finger
        shadow_planning::PlanningOptions _planning_options;
        // Publisher list for inner-loop controller
        std::vector <ros::Publisher> _joint_controller_publishers;

        // Minimum requirement is the finger name, everything else can be populated from there
        Finger(std::string& finger_name) :
                _finger_name(finger_name),
                _finger_move_group(finger_name) {
            _joints = shadow_finger::getJointNames(_finger_move_group);
            std::cout << "Initialized Finger: " << _finger_name << std::endl;
        };
    };

    struct Thumb {
        Thumb();
    };
}
