//
// Created by kganguly on 2/18/18.
//
#pragma once

#include <shadowlibs/shadow_imports.hpp>

/* Pretty printing */
std::ostream &operator<<(std::ostream &out, const geometry_msgs::Pose &pose);

template<typename T>
std::ostream &operator<<(std::ostream &out, const std::vector <T> &joint_data) {
    for (T i:joint_data) {
        out << i << " ";
    }
    out << "\n";

    return out;  // return std::ostream so we can chain calls to operator<<
};

namespace shadow_utils {
    /* Biotac index list */
    static std::vector <std::string> biotac_idx = {"first", "middle", "ring", "little", "thumb"};

/* Primitive types for object creation */
    enum struct primitive_type {
        BOX, CYLINDER, CONE, SPHERE
    };

/* Conversions */
    Eigen::Quaternionf ypr2quat(float y, float p, float r);

    void quat2ypr(tf::Quaternion &q, double &r, double &p, double &y);

    inline double deg2rad(double deg) { return deg * M_PI / 180.0; };

    inline double rad2deg(double rad) { return rad * (180 / M_PI); };

/* RViz object creation */
    void createCollisionObjectFromPrimitive(moveit_msgs::CollisionObject &collision_obj, std::string primitive_id,
                                            shape_msgs::SolidPrimitive &primitive, geometry_msgs::Pose &obj_pose,
                                            shadow_utils::primitive_type primitive_type,
                                            const std::vector<double> &primitive_dims);

    void createAttachedObjectFromPrimitive(moveit_msgs::AttachedCollisionObject &attached_obj, std::string primitive_id,
                                           shape_msgs::SolidPrimitive &primitive, geometry_msgs::Pose &obj_pose,
                                           shadow_utils::primitive_type primitive_type,
                                           const std::vector<double> &primitive_dims);

/* Add, move, remove primitives in scene */
    void addCollisionObjectToScene(moveit_msgs::CollisionObject &collision_obj,
                                   moveit_msgs::PlanningScene &planning_scene_msg,
                                   ros::Publisher &planning_scene_diff_publisher);

    void addAttachedObjectToScene(moveit_msgs::AttachedCollisionObject &attached_obj,
                                  moveit_msgs::PlanningScene &planning_scene_msg,
                                  ros::Publisher &planning_scene_diff_publisher);

    void moveCollisionObjectInScene(moveit_msgs::CollisionObject &collision_obj, geometry_msgs::Pose &obj_pose,
                                    moveit_msgs::PlanningScene &planning_scene_msg,
                                    ros::Publisher &planning_scene_diff_publisher);

    void attachObjectToRobot(moveit_msgs::AttachedCollisionObject &attached_obj,
                             moveit_msgs::PlanningScene &planning_scene_msg,
                             ros::Publisher &planning_scene_diff_publisher);

    void detachObjectFromRobot(moveit_msgs::AttachedCollisionObject &attached_obj,
                               moveit_msgs::PlanningScene &planning_scene_msg,
                               ros::Publisher &planning_scene_diff_publisher);

    void removeAllObjectsFromScene(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface);

    void removeCollisionObjectFromScene(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
                                        std::string obj_id);

/* Pose conversions */
    void
    getPoseFromPositionYPR(geometry_msgs::Pose &pose, float x, float y, float z, float yaw, float pitch, float roll);

    void
    getPoseFromPositionQuaternion(geometry_msgs::Pose &pose, float x, float y, float z, float qx, float qy, float qz,
                                  float qw);

    void getPoseFromPositionYPR(geometry_msgs::Pose &pose, float x, float y, float z, tf::Vector3 &v);

    void getPoseFromPositionQuaternion(geometry_msgs::Pose &pose, float x, float y, float z, tf::Quaternion &q);

    void getPoseFromPositionQuaternion(geometry_msgs::Pose &pose, tf::StampedTransform &tf, tf::Quaternion &q);

/* Fetch transformations */
    void getTfFromFrames(tf::TransformListener &listener, tf::StampedTransform &tf, std::string parent_frame,
                         std::string child_frame);

/* Controller */
    std::string getControllerTopic(std::string &joint_name);

    std::vector <ros::Publisher>
    createJointControllerPublishers(std::vector <std::string> &joint_names, ros::NodeHandle &n);

/* TF related */
    void broadcastTransformForGrasp(std::vector<double> xyzrpy, const std::string frame_name);

    geometry_msgs::Pose getPoseBetweenFrames(const std::string parent_frame, const std::string child_frame);

/* Inverse Kinematics */
    moveit_msgs::RobotState computeJointsFromPose(moveit::planning_interface::MoveGroupInterface &move_group_interface,
                                                  double timeout, int num_attempts,
                                                  geometry_msgs::Pose &target_pose);
};