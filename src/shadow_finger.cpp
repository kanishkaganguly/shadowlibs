//
// Created by kganguly on 4/30/19.
//

#include <shadowlibs/shadow_finger.hpp>

std::string shadow_finger::getMoveGroupName(std::string finger_name) {
    std::string move_group_name;
    std::size_t thumb_check = finger_name.find("thumb");
    if (thumb_check != std::string::npos)
        move_group_name = "rh_" + finger_name;
    else
        move_group_name = "rh_" + finger_name + "_finger";

    return move_group_name;
}

std::string shadow_finger::getSavedStateName(std::string finger_name, std::string saved_state) {
    std::string saved_state_name;

    // "open" or "pack"
    std::size_t thumb_check = finger_name.find("thumb");
    if (thumb_check != std::string::npos)
        saved_state_name = finger_name + "_" + saved_state;
    else
        saved_state_name = finger_name + "_finger_" + saved_state;
    return saved_state_name;
}

std::vector<double>
shadow_finger::getJointValues(moveit::planning_interface::MoveGroupInterface &move_group_interface) {
    std::vector<double> joint_vals;
    joint_vals = move_group_interface.getCurrentJointValues();

    return joint_vals;
}

std::vector <std::string>
shadow_finger::getJointNames(moveit::planning_interface::MoveGroupInterface &move_group_interface) {
    std::vector <std::string> joint_names;
    joint_names = move_group_interface.getJointNames();
    return joint_names;
}

std::vector <std::string> shadow_finger::getJointNames(moveit::planning_interface::MoveGroupInterface::Plan &plan) {
    // This is the trajectory generated by MoveIt plan
    moveit_msgs::RobotTrajectory robotTrajectory = plan.trajectory_;
    // Convert MoveIt to trajectory_msgs joint trajectory
    trajectory_msgs::JointTrajectory planTrajectory = robotTrajectory.joint_trajectory;
    // Gives all the joint names in the planning group used when planning
    std::vector <std::string> jointNames = planTrajectory.joint_names;
    return jointNames;
}

shadow_finger::Finger::BioTac shadow_finger::Finger::getBiotacPressure() {
    sr_robot_msgs::BiotacAllConstPtr biotac_packet = ros::topic::waitForMessage<sr_robot_msgs::BiotacAll>("/rh/tactile", this->_node_handle);
    sr_robot_msgs::Biotac biotac_data = biotac_packet->tactiles[this->_biotac_id];
    shadow_finger::Finger::BioTac biotac_out;
    biotac_out.pressure = biotac_data.pdc;
    return biotac_out;
}

shadow_finger::Finger::BioTac shadow_finger::Finger::getBiotacImpedance() {
    sr_robot_msgs::BiotacAllConstPtr biotac_packet = ros::topic::waitForMessage<sr_robot_msgs::BiotacAll>("/rh/tactile", this->_node_handle);
    sr_robot_msgs::Biotac biotac_data = biotac_packet->tactiles[this->_biotac_id];
    shadow_finger::Finger::BioTac biotac_out;
    biotac_out.impedance = biotac_data.electrodes;
    return biotac_out;
}

shadow_finger::Finger::BioTac shadow_finger::Finger::getBiotacImpedancePressure() {
    sr_robot_msgs::BiotacAllConstPtr biotac_packet = ros::topic::waitForMessage<sr_robot_msgs::BiotacAll>("/rh/tactile", this->_node_handle);
    sr_robot_msgs::Biotac biotac_data = biotac_packet->tactiles[this->_biotac_id];
    shadow_finger::Finger::BioTac biotac_out;
    biotac_out.pressure = biotac_data.pdc;
    biotac_out.impedance = biotac_data.electrodes;
    return biotac_out;
}