//
// Created by kganguly on 2/18/18.
//
#pragma once

// ROS
#include <actionlib/client/simple_action_client.h>
#include <pluginlib/class_loader.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/master.h>
#include <ros/topic_manager.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// MoveIt
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

// RViz
#include <moveit_visual_tools/moveit_visual_tools.h>

// MoveIt Planning
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/MoveItErrorCodes.h>

// Robot State
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

// Kinematics
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/PositionIKRequest.h>

// CPP
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/scoped_ptr.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <map>

/* Pretty printing */
std::ostream &operator<<(std::ostream &out, const geometry_msgs::Pose &pose);

template<typename T>
std::ostream& operator<<(std::ostream& out, const std::vector <T>& joint_data) {
    for (T i:joint_data) {
        out << i << " ";
    }
    out << "\n";

    return out;  // return std::ostream so we can chain calls to operator<<
};

namespace shadow_utils {

/* Primitive types for object creation */
    enum struct primitive_type {
        BOX, CYLINDER, CONE, SPHERE
    };

/* Planning options */
    struct PlanningOptions {
        double set_planning_time;
        int num_attempts;
        bool allow_replanning;
        double goal_position_tolerance;
        double goal_orientation_tolerance;
        double goal_joint_tolerance;
        std::string end_effector_name;

        PlanningOptions();

        PlanningOptions(double set_planning_time, bool allow_replanning, int num_attempts,
                        double goal_position_tolerance,
                        double goal_orientation_tolerance, double goal_joint_tolerance, std::string end_effector_name);
    };

/* Conversions */
    Eigen::Quaternionf ypr2quat(float y, float p, float r);

    void quat2ypr(tf::Quaternion& q, double& r, double& p, double& y);

    double deg2rad(double deg);

    double rad2deg(double rad);

/* RViz object creation */
    void createCollisionObjectFromPrimitive(moveit_msgs::CollisionObject& collision_obj, std::string primitive_id,
                                            shape_msgs::SolidPrimitive& primitive, geometry_msgs::Pose& obj_pose,
                                            shadow_utils::primitive_type primitive_type,
                                            const std::vector<double>& primitive_dims);

    void createAttachedObjectFromPrimitive(moveit_msgs::AttachedCollisionObject& attached_obj, std::string primitive_id,
                                           shape_msgs::SolidPrimitive& primitive, geometry_msgs::Pose& obj_pose,
                                           shadow_utils::primitive_type primitive_type,
                                           const std::vector<double>& primitive_dims);

/* Add, move, remove primitives in scene */
    void addCollisionObjectToScene(moveit_msgs::CollisionObject& collision_obj,
                                   moveit_msgs::PlanningScene& planning_scene_msg,
                                   ros::Publisher& planning_scene_diff_publisher);

    void addAttachedObjectToScene(moveit_msgs::AttachedCollisionObject& attached_obj,
                                  moveit_msgs::PlanningScene& planning_scene_msg,
                                  ros::Publisher& planning_scene_diff_publisher);

    void moveCollisionObjectInScene(moveit_msgs::CollisionObject& collision_obj, geometry_msgs::Pose& obj_pose,
                                    moveit_msgs::PlanningScene& planning_scene_msg,
                                    ros::Publisher& planning_scene_diff_publisher);

    void attachObjectToRobot(moveit_msgs::AttachedCollisionObject& attached_obj,
                             moveit_msgs::PlanningScene& planning_scene_msg,
                             ros::Publisher& planning_scene_diff_publisher);

    void detachObjectFromRobot(moveit_msgs::AttachedCollisionObject& attached_obj,
                               moveit_msgs::PlanningScene& planning_scene_msg,
                               ros::Publisher& planning_scene_diff_publisher);

    void removeAllObjectsFromScene(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);

    void removeCollisionObjectFromScene(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                                        std::string obj_id);

/* Pose conversions */
    void
    getPoseFromPositionYPR(geometry_msgs::Pose& pose, float x, float y, float z, float yaw, float pitch, float roll);

    void
    getPoseFromPositionQuaternion(geometry_msgs::Pose& pose, float x, float y, float z, float qx, float qy, float qz,
                                  float qw);

    void getPoseFromPositionYPR(geometry_msgs::Pose& pose, float x, float y, float z, tf::Vector3& v);

    void getPoseFromPositionQuaternion(geometry_msgs::Pose& pose, float x, float y, float z, tf::Quaternion& q);

    void getPoseFromPositionQuaternion(geometry_msgs::Pose& pose, tf::StampedTransform& tf, tf::Quaternion& q);

/* Get joint names and values */
    std::vector<double> getJointValues(moveit::planning_interface::MoveGroupInterface& move_group_interface);

    std::vector <std::string> getJointNames(moveit::planning_interface::MoveGroupInterface& move_group_interface);

    std::vector <std::string> getJointNames(moveit::planning_interface::MoveGroupInterface::Plan& plan);

/* Fetch transformations */
    void getTfFromFrames(tf::TransformListener& listener, tf::StampedTransform& tf, std::string parent_frame,
                         std::string child_frame);

/* Controller */
    std::string getControllerTopic(std::string& joint_name);

    std::vector <ros::Publisher>
    createJointControllerPublishers(std::vector <std::string>& joint_names, ros::NodeHandle& n);

/* Planning functions */
    geometry_msgs::Pose
    getRandomPose(moveit::planning_interface::MoveGroupInterface& move_group_interface, const std::string frame_name);

    bool getPlanToNamedTarget(shadow_utils::PlanningOptions& options,
                              moveit::planning_interface::MoveGroupInterface& move_group_interface,
                              const std::string target_name,
                              moveit::planning_interface::MoveGroupInterface::Plan& plan);

    bool getPlanToPoseTarget(shadow_utils::PlanningOptions& options,
                             moveit::planning_interface::MoveGroupInterface& move_group_interface,
                             geometry_msgs::Pose& target_pose, const std::string reference_frame,
                             moveit::planning_interface::MoveGroupInterface::Plan& plan,
                             const std::string end_effector_name);

    bool getPlanToJointTargets(shadow_utils::PlanningOptions& options,
                               moveit::planning_interface::MoveGroupInterface& move_group_interface,
                               const std::string reference_frame,
                               moveit::planning_interface::MoveGroupInterface::Plan& plan,
                               std::map<std::string, double>& joint_targets);

    bool executePlan(std::vector <ros::Publisher>& controller_pubs,
                     moveit::planning_interface::MoveGroupInterface::Plan& plan);

/* TF related */
    void broadcastTransformForGrasp(std::vector<double> xyzrpy, const std::string frame_name);

    geometry_msgs::Pose getPoseBetweenFrames(const std::string parent_frame, const std::string child_frame);

/* Inverse Kinematics */
    moveit_msgs::RobotState computeJointsFromPose(moveit::planning_interface::MoveGroupInterface& move_group_interface,
                                                  double timeout, int num_attempts,
                                                  geometry_msgs::Pose& target_pose);
};