//
// Created by kganguly on 5/3/19.
//

#pragma once

#include <shadowlibs/shadow_hand.hpp>

/** @namespace shadow_planning shadow_planning.hpp "include/shadowlibs/shadow_planning.hpp"
 *  @brief Utility functions for planning and execution routines to control the ShadowHand
 */
namespace shadow_planning {
/**
 * @brief Construct random target pose for a given planning group
 * @param move_group_interface The MoveGroup to generate pose for
 * @param frame_name The name of the reference frame
 * @return The generated random pose
 */
geometry_msgs::Pose
getRandomPose(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::string &frame_name);

/**
 * @brief Construct plan to a given "named" target, these are obtained from predefined RViz names
 * @param options The shadow_planning::PlanningOptions object for computing the plan
 * @param move_group_interface The MoveGroup for which to plan
 * @param target_name The named target
 * @param plan The reference to the plan, which will be populated
 * @return Planning success or failure boolean
 */
bool planToNamedTarget(shadow_planning::PlanningOptions &options,
					   moveit::planning_interface::MoveGroupInterface &move_group_interface,
					   std::string &target_name,
					   moveit::planning_interface::MoveGroupInterface::Plan &plan);

/** @overload */
bool planToNamedTarget(shadow_finger::Finger &finger, std::string &target_name);

/**
 * @brief Construct plan to a given Pose target
 * @param options The shadow_planning::PlanningOptions object for computing the plan
 * @param move_group_interface The MoveGroup for which to plan
 * @param target_pose The target pose
 * @param reference_frame The frame of reference for planning
 * @param plan The reference to the plan, which will be populated
 * @param end_effector_name The name of the end-effector for which plan is computed
 * @return Planning success or failure boolean
 */
bool planToPoseTarget(shadow_planning::PlanningOptions &options,
					  moveit::planning_interface::MoveGroupInterface &move_group_interface,
					  geometry_msgs::Pose &target_pose, std::string &reference_frame,
					  moveit::planning_interface::MoveGroupInterface::Plan &plan,
					  std::string &end_effector_name);

/**
 * @brief Construct plan to given Joint target
 * @param options The shadow_planning::PlanningOptions object for computing the plan
 * @param move_group_interface The MoveGroup for which to plan
 * @param reference_frame The frame of reference for planning
 * @param plan The reference to the plan, which will be populated
 * @param joint_targets Map from joint name to the target value
 * @return Planning success or failure boolean
 */
bool planToJointTargets(shadow_planning::PlanningOptions &options,
						moveit::planning_interface::MoveGroupInterface &move_group_interface,
						std::string &reference_frame,
						moveit::planning_interface::MoveGroupInterface::Plan &plan,
						std::map<std::string, double> &joint_targets);

/**
 * @brief Execute plan for a given plan given its list of controllers
 * @param controller_pubs The list of controller publishers for execution
 * @param plan The plan to be executed
 * @return Execution success or failure
 */
bool executePlan(std::vector <ros::Publisher> &controller_pubs,
				 moveit::planning_interface::MoveGroupInterface::Plan &plan);

/** @overload */
bool executePlan(shadow_finger::Finger &finger);

/**
 * @brief Execute plan for a given Finger asynchronously, where multiple Fingers
 * can be executed simultaneously
 * @param finger The Finger whose plan is to be executed
 */
void executePlanAsync(shadow_finger::Finger &finger);

/** @overload */
void executePlanAsync(shadow_hand::Hand &hand);
};
