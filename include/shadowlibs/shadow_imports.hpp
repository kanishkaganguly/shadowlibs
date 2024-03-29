//
// Created by kganguly on 5/3/19.
//

#pragma once

// ROS
#include <actionlib/client/simple_action_client.h>
#include <controller_manager_msgs/SwitchController.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>
#include <ros/console.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <ros/topic_manager.h>
#include <rosbag/bag.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
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
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/PlanningScene.h>

// Robot State
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <urdf/model.h>

// Kinematics
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/PositionIKRequest.h>

// BioTac
#include <sr_robot_msgs/Biotac.h>
#include <sr_robot_msgs/BiotacAll.h>

// Controller
#include <sr_mechanism_controllers/sr_controller.hpp>
#include <sr_robot_msgs/JointControllerState.h>
#include <sr_hardware_interface/tactile_sensors.hpp>
#include <sr_hardware_interface/sr_actuator.hpp>

// CPP
#include <time.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <boost/scoped_ptr.hpp>
#include <cmath>
#include <functional>
#include <future>
#include <iostream>
#include <map>
#include <mutex>
#include <condition_variable>
#include <numeric>
#include <string>
#include <thread>
#include <tuple>

#include "../shadowlibs/prettyprint.hpp"
#include "../shadowlibs/shadow_planning_options.hpp"