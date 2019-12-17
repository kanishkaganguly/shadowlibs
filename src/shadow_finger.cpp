//
// Created by kganguly on 4/30/19.
//

#include <shadowlibs/shadow_finger.hpp>

shadow_finger::Finger::BioTac shadow_finger::Finger::getBioTacPressure() {
    sr_robot_msgs::BiotacAllConstPtr biotac_packet = ros::topic::waitForMessage<sr_robot_msgs::BiotacAll>("/rh/tactile", this->_node_handle, ros::Duration(1));
    shadow_finger::Finger::BioTac biotac_out;
    if (biotac_packet) {
        sr_robot_msgs::Biotac biotac_data = biotac_packet->tactiles[this->_biotac_id];
        biotac_out.pressure = biotac_data.pdc;
    } else {
        biotac_out.pressure = 0;
        biotac_out.impedance.resize(24, 0);
    }
    return biotac_out;
}

shadow_finger::Finger::BioTac shadow_finger::Finger::getBioTacImpedance() {
    sr_robot_msgs::BiotacAllConstPtr biotac_packet = ros::topic::waitForMessage<sr_robot_msgs::BiotacAll>("/rh/tactile", this->_node_handle, ros::Duration(1));
    shadow_finger::Finger::BioTac biotac_out;
    if (biotac_packet) {
        sr_robot_msgs::Biotac biotac_data = biotac_packet->tactiles[this->_biotac_id];
        biotac_out.impedance = biotac_data.electrodes;
    } else {
        biotac_out.pressure = 0;
        biotac_out.impedance.resize(24, 0);
    }
    return biotac_out;
}

shadow_finger::Finger::BioTac shadow_finger::Finger::getBioTacImpedancePressure() {
    sr_robot_msgs::BiotacAllConstPtr biotac_packet = ros::topic::waitForMessage<sr_robot_msgs::BiotacAll>("/rh/tactile", this->_node_handle, ros::Duration(1));
    shadow_finger::Finger::BioTac biotac_out;
    if (biotac_packet) {
        sr_robot_msgs::Biotac biotac_data = biotac_packet->tactiles[this->_biotac_id];
        biotac_out.pressure = biotac_data.pdc;
        biotac_out.impedance = biotac_data.electrodes;
    } else {
        biotac_out.pressure = 0;
        biotac_out.impedance.resize(24, 0);
    }
    return biotac_out;
}