//
// Created by kganguly on 2/18/18.
//
#pragma once

#include <shadowlibs/shadow_imports.hpp>

/**
 * @brief Print ROS Pose messages
 * @param out Output stream
 * @param pose The pose to print
 * @return Output stream, so that we can chain further calls to operator<<
 */
std::ostream &operator<<(std::ostream &out, const geometry_msgs::Pose &pose);

/**
 * @brief Print vector data
 * @tparam T Templated over whatever data type std::vector accepts
 * @param out Output stream
 * @param joint_data The data vector to print
 * @return Output stream, so that we can chain further calls to operator<<
 */
template <typename T>
std::ostream &operator<<(std::ostream &out, const std::vector<T> &joint_data) {
  for (T i : joint_data) {
    out << i << " ";
  }
  out << "\n";

  return out;  // return std::ostream so we can chain calls to operator<<
};

/**
 * @namespace shadow_utils shadow_utils.hpp
 * "include/shadowlibs/shadow_utils.hpp"
 * @brief Contains various utility functions for RViz visualization,
 * conversions, transformations and robot controllers.
 */
namespace shadow_utils {
/**
 * @enum shadow_utils::SHAPE_PRIMITIVES
 * @brief Primitive shapes used for RViz objects
 */
enum struct SHAPE_PRIMITIVES { BOX, CYLINDER, CONE, SPHERE };

/**
 * @class shadow_utils::COLORS_CLASS
 * @brief Colors for ROS_INFO printing
 */
class COLORS {
 public:
  static const std::string COLOR_NORMAL;
  static const std::string COLOR_RED;
  static const std::string COLOR_GREEN;
  static const std::string COLOR_YELLOW;
};  // class COLORS

void ROS_INFO_COLOR(const std::stringstream &print_str,
                    const std::string &color);

/** @brief Conversion from YPR to Quaternion */
Eigen::Quaternionf ypr2quat(float y, float p, float r);
/** @brief Conversion from Quaternion to YPR */
void quat2ypr(tf::Quaternion &q, double &r, double &p, double &y);

/** @brief Conversion from degree to radians */
template <typename F>
inline F deg2rad(F deg) {
  return deg * M_PI / 180.0;
};

/** @brief Conversion from radians to degrees */
template <typename F>
inline F rad2deg(F rad) {
  return rad * (180.0 / M_PI);
};

/**
 * @brief Rosbag recording utility function
 * @param bag_path The path where to record bag
 * @param bag_name The name of the bag file
 * @param topic_name
 */
rosbag::Bag startRecordBag(const std::string &bag_path,
                           const std::string &bag_name,
                           const std::string &topic_name);

inline void stopRecordBag(rosbag::Bag &bag) { bag.close(); };

/**
 * @brief Create collision object for the planning scene, used by MoveIt
 * planning
 * @param collision_obj The MoveIt collision object
 * @param primitive_id The ID of the collision object
 * @param primitive The primitive used by the collision object
 * @param obj_pose The pose of the collision object
 * @param primitive_type The shape of the collision object
 * @param primitive_dims The dimension of the collision object
 */
void createCollisionObjectFromPrimitive(
    moveit_msgs::CollisionObject &collision_obj, std::string primitive_id,
    shape_msgs::SolidPrimitive &primitive, geometry_msgs::Pose &obj_pose,
    shadow_utils::SHAPE_PRIMITIVES primitive_type,
    const std::vector<double> &primitive_dims);

/**
 * @brief Create attached object for the planning scene, used by MoveIt planning
 * when attaching object to robot This helps plan when an object is "attached"
 * to the end of a robot, after grasping.
 * @param attached_obj The MoveIt attached collision object
 * @param primitive_id The ID of the attached object
 * @param primitive The primitive used by the attached object
 * @param obj_pose The pose of the attached object
 * @param primitive_type The shape of the attached object
 * @param primitive_dims The dimension of the attached object
 */
void createAttachedObjectFromPrimitive(
    moveit_msgs::AttachedCollisionObject &attached_obj,
    std::string primitive_id, shape_msgs::SolidPrimitive &primitive,
    geometry_msgs::Pose &obj_pose,
    shadow_utils::SHAPE_PRIMITIVES primitive_type,
    const std::vector<double> &primitive_dims);

/**
 * @brief Add a collision object to the planning scene
 * @param collision_obj The object to add to scene
 * @param planning_scene_msg The PlanningScene to add object to
 * @param planning_scene_diff_publisher The publisher for updating the scene
 */
void addCollisionObjectToScene(moveit_msgs::CollisionObject &collision_obj,
                               moveit_msgs::PlanningScene &planning_scene_msg,
                               ros::Publisher &planning_scene_diff_publisher);

/**
 * @brief Add a collision object to the planning scene
 * @param attached_obj The attached object to add to scene
 * @param planning_scene_msg The PlanningScene to add object to
 * @param planning_scene_diff_publisher The publisher for updating the scene
 */
void addAttachedObjectToScene(
    moveit_msgs::AttachedCollisionObject &attached_obj,
    moveit_msgs::PlanningScene &planning_scene_msg,
    ros::Publisher &planning_scene_diff_publisher);

/**
 * @brief Move an existing collision object in the planning scene
 * @param collision_obj The collision object to move in the scene
 * @param obj_pose The pose of the collision object in the scene
 * @param planning_scene_msg The PlanningScene to add object to
 * @param planning_scene_diff_publisher The publisher for updating the scene
 */
void moveCollisionObjectInScene(moveit_msgs::CollisionObject &collision_obj,
                                geometry_msgs::Pose &obj_pose,
                                moveit_msgs::PlanningScene &planning_scene_msg,
                                ros::Publisher &planning_scene_diff_publisher);

/**
 * @brief Attach a collision object to the robot
 * @param attached_obj The collision object to attach
 * @param planning_scene_msg The PlanningScene to add object to
 * @param planning_scene_diff_publisher The publisher for updating the scene
 */
void attachObjectToRobot(moveit_msgs::AttachedCollisionObject &attached_obj,
                         moveit_msgs::PlanningScene &planning_scene_msg,
                         ros::Publisher &planning_scene_diff_publisher);

/**
 * @brief Detach an attached collision object from the robot
 * @param attached_obj The collision object to detach
 * @param planning_scene_msg The PlanningScene to add object to
 * @param planning_scene_diff_publisher The publisher for updating the scene
 */
void detachObjectFromRobot(moveit_msgs::AttachedCollisionObject &attached_obj,
                           moveit_msgs::PlanningScene &planning_scene_msg,
                           ros::Publisher &planning_scene_diff_publisher);

/**
 * @brief Remove all objects from the scene
 * @param planning_scene_interface The PlanningScene to remove objects from
 */
void removeAllObjectsFromScene(
    moveit::planning_interface::PlanningSceneInterface
        &planning_scene_interface);

/**
 * @brief Remove some object from the scene
 * @param planning_scene_interface The PlanningScene to remove objects from
 * @param obj_id The ID of the object to be removed
 */
void removeCollisionObjectFromScene(
    moveit::planning_interface::PlanningSceneInterface
        &planning_scene_interface,
    std::string obj_id);

/**
 * @brief Generate ROS Pose from position and YPR
 * @param pose The pose to be constructed
 * @param x X position
 * @param y Y position
 * @param z Z position
 * @param yaw Yaw orientation
 * @param pitch Pitch orientation
 * @param roll Roll orientation
 */
void getPoseFromPositionYPR(geometry_msgs::Pose &pose, float x, float y,
                            float z, float yaw, float pitch, float roll);

/**
 * @brief Generate ROS Pose from position and quaternion
 * @param pose The pose to be constructed
 * @param x X position
 * @param y Y position
 * @param z Z position
 * @param qx x quaternion
 * @param qy y quaternion
 * @param qz z quaternion
 * @param qw w quaternion
 */
void getPoseFromPositionQuaternion(geometry_msgs::Pose &pose, float x, float y,
                                   float z, float qx, float qy, float qz,
                                   float qw);

/** @overload */
void getPoseFromPositionYPR(geometry_msgs::Pose &pose, float x, float y,
                            float z, tf::Vector3 &v);

/** @overload */
void getPoseFromPositionQuaternion(geometry_msgs::Pose &pose, float x, float y,
                                   float z, tf::Quaternion &q);

/** @overload */
void getPoseFromPositionQuaternion(geometry_msgs::Pose &pose,
                                   tf::StampedTransform &tf, tf::Quaternion &q);

/**
 * @brief Get TF from parent and child frames
 * @param listener The TF listener
 * @param tf The transform data obtained
 * @param parent_frame The name of the parent frame
 * @param child_frame The name of the child frame
 */
void getTfFromFrames(tf::TransformListener &listener, tf::StampedTransform &tf,
                     std::string parent_frame, std::string child_frame);

/**
 * @brief Get the name of the controller responsible for controlling the given
 * joint name
 * @param joint_name Name of joint to fetch controller for
 * @return Name of the controller publisher
 */
std::string getControllerTopic(std::string &joint_name);

/**
 * @brief Create a vector of publishers for each given joint name, using the ROS
 * node handle
 * @param joint_names Names of joints to create controllers for
 * @param n The ROS node handle
 * @return Vector of publishers for each joint given as input
 */
std::vector<ros::Publisher> createJointControllerPublishers(
    std::vector<std::string> &joint_names, ros::NodeHandle &n);

/**
 * @brief Broadcasts a transform from the wrist to given frame, which can be
 * used for planning targets for finger
 * @param xyzrpy The pose of the transform
 * @param frame_name The frame name of the transform
 */
void broadcastTransformForGrasp(std::vector<double> xyzrpy,
                                const std::string frame_name);

/**
 * @brief Get the pose between two frames
 * @param parent_frame The parent frame
 * @param child_frame The child frame
 * @return The pose between parent and child frames
 */
geometry_msgs::Pose getPoseBetweenFrames(const std::string parent_frame,
                                         const std::string child_frame);

/**
 * @brief Creates a RobotState using the inverse kinematics solver, given a
 * target pose. Uses a service call to the "compute_ik" service to obtain the
 * joint angles needed for the given MoveGroup to reach the specified target
 * pose.
 * @see http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/RobotState.html
 * @see http://docs.ros.org/kinetic/api/moveit_msgs/html/srv/GetPositionIK.html
 * @param move_group_interface The MoveGroup to compute RobotState for
 * @param timeout The timeout for computation
 * @param num_attempts The number of tries for computing IK
 * @param target_pose The target pose to compute joint angles for
 * @return The RobotState computed by the IK service
 */
moveit_msgs::RobotState computeJointsFromPose(
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    double timeout, int num_attempts, geometry_msgs::Pose &target_pose);

/**
 * @brief Starts trajectory controller for arm or hand, using ControllerManager
 * service call
 * @param n - ROS node handle
 * @param controller_name - Name of controller, arm or hand
 * (rh_trajectory_controller OR ra_trajectory_controller)
 */
void startTrajectoryController(ros::NodeHandle n, std::string controller_name);

/**
 * @brief Stops trajectory controller for arm or hand, using ControllerManager
 * service call
 * @param n - ROS node handle
 * @param controller_name - Name of controller, arm or hand
 * (rh_trajectory_controller OR ra_trajectory_controller)
 */
void stopTrajectoryController(ros::NodeHandle n, std::string controller_name);
};  // namespace shadow_utils