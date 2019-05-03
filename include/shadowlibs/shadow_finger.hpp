//
// Created by kganguly on 4/30/19.
//

#pragma once

#include <shadowlibs/shadow_imports.hpp>
#include <shadowlibs/shadow_planning.hpp>
#include <shadowlibs/shadow_utils.hpp>

namespace shadow_finger {
    // Planning group names
    const std::string first_finger_name = "rh_first_finger";
    const std::string middle_finger_name = "rh_middle_finger";
    const std::string ring_finger_name = "rh_ring_finger";
    const std::string little_finger_name = "rh_little_finger";
    const std::string thumb_name = "rh_thumb";

    std::string getMoveGroupName(std::string finger_name){
        
    }

    /* Get joint names and values */
    std::vector<double> getJointValues(moveit::planning_interface::MoveGroupInterface& move_group_interface);

    std::vector <std::string> getJointNames(moveit::planning_interface::MoveGroupInterface& move_group_interface);

    std::vector <std::string> getJointNames(moveit::planning_interface::MoveGroupInterface::Plan& plan);

    // Manage fingers using separate class
    struct Finger {
    public:
        // Finger name
        const std::string _finger_name;
        // Set of joints in finger
        const std::vector <std::string> _joints;
        // MoveIt planning group
        moveit::planning_interface::MoveGroupInterface _finger_move_group;
        // MoveIt plan for finger
        moveit::planning_interface::MoveGroupInterface::Plan _plan;
        // Planning options for finger
        shadow_planning::PlanningOptions _planning_options;
        // Publisher list for inner-loop controller
        std::vector <ros::Publisher> _joint_controller_publishers;

        // Minimum requirement is the finger name, everything else can be populated from there
        Finger(const std::string& finger_name);

    };

    struct Thumb {
        Thumb();
    };
}
