//
// Created by kganguly on 4/30/19.
//

#pragma once
#include <shadowlibs/shadow_utils.hpp>

/** @namespace shadow_finger shadow_finger.hpp
 * "include/shadowlibs/shadow_finger.hpp"
 *  @brief Utility functions for managing each finger of Shadow Hand.
 *  Contains struct to hold necessary data about each finger, and each instance
 * can be passed around to planning utilities. Also contains some functionality
 * for operating on BioTac sensor attached to the respective finger.
 */
namespace shadow_finger {

/** @brief Get preloaded plan as string for open or close
 *	@param finger_name Name of the finger, or thumb
 *	@param saved_state Either "open" or "close"
 *  @return Formatted string that is accepted by MoveIt planner
 */
inline std::string getSavedStateName(std::string finger_name,
                                     std::string saved_state) {
  std::string saved_state_name;
  // "open" or "pack"
  std::size_t thumb_check = finger_name.find("thumb");
  if (thumb_check != std::string::npos)
    saved_state_name = finger_name + "_" + saved_state;
  else
    saved_state_name = finger_name + "_finger_" + saved_state;
  return saved_state_name;
};

/** @brief Get planning group names given common name for finger
 *  @param finger_name The name of the finger to operate on
 *	@return Formatted string with the MoveGroup name
 */
inline std::string getMoveGroupName(std::string finger_name) {
  std::string move_group_name;
  std::size_t thumb_check = finger_name.find("thumb");
  if (thumb_check != std::string::npos)
    move_group_name = "rh_" + finger_name;
  else
    move_group_name = "rh_" + finger_name + "_finger";

  return move_group_name;
};

/** @brief Get names of joints from given MoveGroupInterface
 *  @param move_group_interface The MoveGroupInterface for the finger
 *  @return The vector of strings containing the joints in that MoveGroup
 */
inline std::vector<std::string> getJointNames(
    moveit::planning_interface::MoveGroupInterface &move_group_interface) {
  std::vector<std::string> joint_names;
  joint_names = move_group_interface.getJointNames();
  return joint_names;
};

/** @overload */
inline std::vector<std::string> getJointNames(
    moveit::planning_interface::MoveGroupInterface::Plan &plan) {
  // This is the trajectory generated by MoveIt plan
  moveit_msgs::RobotTrajectory robotTrajectory = plan.trajectory_;
  // Convert MoveIt to trajectory_msgs joint trajectory
  trajectory_msgs::JointTrajectory planTrajectory =
      robotTrajectory.joint_trajectory;
  // Gives all the joint names in the planning group used when planning
  std::vector<std::string> jointNames = planTrajectory.joint_names;
  return jointNames;
};

/** @brief Get current values of joints from given MoveGroupInterface, they are
 * in the same order as the names of joints obtained from getJointNames()
 *  @param move_group_interface The MoveGroupInterface for the finger
 *  @return The vector of doubles containing the values of joints in that
 * MoveGroup
 */
inline std::vector<double> getJointValues(
    moveit::planning_interface::MoveGroupInterface &move_group_interface) {
  std::vector<double> joint_vals;
  joint_vals = move_group_interface.getCurrentJointValues();
  return joint_vals;
}

/** @brief Get limits of each joint in given vector of joint names.
 * This is obtained from the /robot_description URDF passed to ROS from the
 * robot
 * @param joint_names The vector of strings containing the names of the joints
 * @return Vector of tuples, each with (upper,lower) limits of each joint, in
 * the same order as the input vector
 */
inline std::vector<std::tuple<float, float>> getFingerJointLimits(
    std::vector<std::string> &joint_names) {
  urdf::Model model;
  if (!model.initParam("/robot_description")) {
    ROS_ERROR("Failed to parse urdf file");
  }
  ROS_INFO("Successfully parsed urdf file");
  std::vector<std::tuple<float, float>> joint_limits;
  for (auto joint_name : joint_names) {
    urdf::JointConstSharedPtr j = model.getJoint(joint_name);
    joint_limits.push_back(std::make_tuple(j->limits->upper, j->limits->lower));
  }
  return joint_limits;
}

/** @var biotac_idx
 *  @brief Lookup table that maps finger name to BioTac index
 */
const static std::vector<std::string> biotac_idx = {"first", "middle", "ring",
                                                    "little", "thumb"};
/** @var finger_idx
 *  @brief Lookup table that maps finger name to tactile packet index
 */
const static std::vector<std::string> finger_idx = {"thumb", "first", "middle",
                                                    "ring", "little"};

/**
 * @brief Fetch BioTac index given name of finger
 * @param finger_name String with name of finger
 * @return Integer index of BioTac, to be used for fetching correct data from
 * ROS
 */
inline int getBiotacIdx(std::string &finger_name) {
  int idx = -1;
  for (auto &each_idx : biotac_idx) {
    idx += 1;
    std::size_t found = finger_name.find(each_idx);
    if (found != std::string::npos) {
      return idx;
    }
  }
  return idx;
}

/**
 * @brief Fetch finger index given name of finger
 * @param finger_name String with name of finger
 * @return Integer index of finger, to be used for fetching correct data from
 * ROS packet
 */
inline int getFingerIdx(std::string &finger_name) {
  int idx = -1;
  for (auto &each_idx : finger_idx) {
    idx += 1;
    std::size_t found = finger_name.find(each_idx);
    if (found != std::string::npos) {
      return idx;
    }
  }
  return idx;
}

/** @struct Finger shadow_finger.hpp "include/shadowlibs/shadow_finger.hpp"
 *  @brief Utility struct for managing each finger of Shadow Hand.
 *  Contains all the information about the Finger instance, including
 *  joint limits, MoveGroup, MoveIt plan, and joint control publishers.
 */
struct Finger {
  /**
   * @var _node_handle
   * @brief The ROS node handle
   */
  ros::NodeHandle _node_handle;
  /**
   * @var _finger_name
   * @brief The name of this finger instance
   */
  std::string _finger_name;
  /**
   * @var _finger_id
   * @brief The index of the finger in the order
   * {thumb, first, middle, ring, little}
   */
  int _finger_id;
  /**
   * @var _biotac_id
   * @brief The index of the BioTac sensor
   */
  int _biotac_id;
  /**
   * @var _joint_names
   * @brief Vector of strings of all joints in this finger
   */
  std::vector<std::string> _joint_names;
  /**
   * @var _joint_limits
   * @brief Vector of tuples of joint limits of each joint in finger
   */
  std::vector<std::tuple<float, float>> _joint_limits;
  /**
   * @var _finger_move_group
   * @brief MoveGroupInterface for this finger
   */
  moveit::planning_interface::MoveGroupInterface _finger_move_group;
  /**
   * @var _plan
   * @brief MoveIt plan generated for this finger
   */
  moveit::planning_interface::MoveGroupInterface::Plan _plan;
  /**
   * @var _plan
   * @brief MoveIt plan generated for this finger
   */
  shadow_planning::PlanningOptions _planning_options;
  /**
   * @var _joint_controller_publishers
   * @brief Vector of ROS publishers, for sending control data for each joint in
   * finger
   */
  std::vector<ros::Publisher> _joint_controller_publishers;
  /**
   * @var _fsr_topic
   * @brief Subscriber topic for getting FSR data for finger
   */
  std::string _fsr_topic;

  /**
   * @brief Constructor for each finger. Requires name of finger and a ROS node
   * handle reference. Everything else is populated by constructor.
   * @param finger_name Name of the finger
   * @param node_handle ROS node handle reference
   */
  Finger(std::string &finger_name, ros::NodeHandle &node_handle)
      : _finger_name(finger_name),
        _finger_move_group(shadow_finger::getMoveGroupName(finger_name)),
        _node_handle(node_handle) {
    _joint_names = shadow_finger::getJointNames(_finger_move_group);
    _joint_controller_publishers =
        shadow_utils::createJointControllerPublishers(_joint_names,
                                                      _node_handle);
    _biotac_id = shadow_finger::getBiotacIdx(_finger_name);
    _finger_id = shadow_finger::getFingerIdx(_finger_name);
    _joint_limits = shadow_finger::getFingerJointLimits(_joint_names);

    // Set FSR topic
    if (_finger_name != "thumb") {
      this->_fsr_topic = std::string(1, this->_finger_name.front()) + "f_fsr";
    } else {
      this->_fsr_topic = "th_fsr";
    }

    ROS_INFO_STREAM("Initialized Finger: " << _finger_name);
  };

  /**
   * @brief Getter function for finger name
   * @return String with the finger name
   */
  inline std::string getFingerName() { return _finger_name; }

  /**
   * @brief Getter function for finger ID
   * @return String with the finger name
   */
  inline int getFingerID() { return _finger_id; }

  /**
   * @brief Get joint limits from joint name, from vector of tuples populated by
   * getFingerJointLimits()
   * @param joint_name Name of the joint whose limits are to be fetched
   * @return tuple containing upper and lower limits of joint
   */
  inline std::tuple<float, float> jointLimitsFromName(std::string &joint_name) {
    std::vector<std::string>::iterator it =
        std::find(_joint_names.begin(), _joint_names.end(), joint_name);
    if (it != _joint_names.end()) {
      int idx = std::distance(_joint_names.begin(), it);
      return _joint_limits[idx];
    } else {
      ROS_ERROR_STREAM("Joint " << joint_name << " not found in "
                                << _finger_name << ".");
      return std::make_tuple(0.0, 0.0);
    }
  }

  /** @struct BioTac shadow_finger.hpp "include/shadowlibs/shadow_finger.hpp"
   *  @brief Utility struct for managing the BioTac sensor attached to each
   * finger. Holds the pressure and impedance data obtained from the sensor.
   */
  struct BioTac {
    int16_t pressure;
    std::vector<int16_t> impedance;
  };

  /**
   * @brief Get populated struct containing BioTac pressure data
   * @return shadow_finger::Finger::BioTac struct
   */
  shadow_finger::Finger::BioTac getBioTacPressure();

  /**
   * @brief Get populated struct containing BioTac impedance data
   * @return shadow_finger::Finger::BioTac struct
   */
  shadow_finger::Finger::BioTac getBioTacImpedance();

  /**
   * @brief Get populated struct containing BioTac pressure and impedance data
   * @return shadow_finger::Finger::BioTac struct
   */
  shadow_finger::Finger::BioTac getBioTacImpedancePressure();

  /**
   * @brief Get FSR value
   */
  float getFSRValue();
};

/** @overload */
inline std::vector<std::string> getJointNames(shadow_finger::Finger &finger) {
  std::vector<std::string> joint_names;
  joint_names = finger._finger_move_group.getJointNames();
  return joint_names;
};

/** @overload */
inline std::vector<double> getJointValues(shadow_finger::Finger &finger) {
  std::vector<double> joint_vals;
  joint_vals = finger._finger_move_group.getCurrentJointValues();
  return joint_vals;
}

/** @brief Get current joint efforts for each joint in finger
 * @return The vector of floats containing joint effort values
 */
inline std::vector<float> getJointEfforts(shadow_finger::Finger &finger) {
  std::vector<float> joint_efforts;
  sensor_msgs::JointState::ConstPtr joint_state_ptr;
  joint_state_ptr = ros::topic::waitForMessage<sensor_msgs::JointState>(
      "/joint_states", finger._node_handle, ros::Duration(1.0));

  for (auto jname : finger._joint_names) {
    auto it = std::find(joint_state_ptr->name.begin(),
                        joint_state_ptr->name.end(), jname);
    // Element found
    int joint_idx = -1;
    if (it != joint_state_ptr->name.end()) {
      joint_idx = it - joint_state_ptr->name.begin();
      joint_efforts.emplace_back(joint_state_ptr->effort[joint_idx]);
    }
  }

  return joint_efforts;
}

}  // namespace shadow_finger

inline std::ostream &operator<<(std::ostream &os,
                                const shadow_finger::Finger &finger) {
  os << finger._finger_name << '\n';
  return os;
}