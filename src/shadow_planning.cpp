//
// Created by kganguly on 5/3/19.
//

#include <shadowlibs/shadow_planning.hpp>

geometry_msgs::Pose shadow_planning::getRandomPose(
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    std::string &frame_name) {

  geometry_msgs::PoseStamped random_pose;
  move_group_interface.setPoseReferenceFrame(frame_name);
  random_pose = move_group_interface.getRandomPose("rh_fftip");

  return random_pose.pose;
}

bool shadow_planning::planToNamedTarget(
    shadow_planning::PlanningOptions &options,
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    std::string &target_name,
    moveit::planning_interface::MoveGroupInterface::Plan &plan) {

  move_group_interface.clearPoseTargets();
  move_group_interface.setStartState(*move_group_interface.getCurrentState());
  move_group_interface.setPlanningTime(options.set_planning_time);
  move_group_interface.allowReplanning(options.allow_replanning);
  move_group_interface.setNumPlanningAttempts(options.num_attempts);
  move_group_interface.setNamedTarget(target_name);
  bool plan_success = false;

  plan_success = (move_group_interface.plan(plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Return success or failure
  if (plan_success) {
    ROS_INFO("Plan Succeeded");
  } else {
    ROS_ERROR("Plan Failed");
  }
  ros::Duration(2.0).sleep();

  return plan_success;
}

bool shadow_planning::planToNamedTarget(shadow_finger::Finger &finger,
                                        std::string &target_name) {

  finger._finger_move_group.clearPoseTargets();
  finger._finger_move_group.setStartState(
      *finger._finger_move_group.getCurrentState());
  finger._finger_move_group.setPlanningTime(
      finger._planning_options.set_planning_time);
  finger._finger_move_group.allowReplanning(
      finger._planning_options.allow_replanning);
  finger._finger_move_group.setNumPlanningAttempts(
      finger._planning_options.num_attempts);
  finger._finger_move_group.setNamedTarget(target_name);
  bool plan_success = false;

  plan_success = (finger._finger_move_group.plan(finger._plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Return success or failure
  if (plan_success) {
    ROS_INFO("Plan Succeeded");
  } else {
    ROS_ERROR("Plan Failed");
  }
  ros::Duration(2.0).sleep();

  return plan_success;
}

bool shadow_planning::planToPoseTarget(
    shadow_planning::PlanningOptions &options,
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    geometry_msgs::Pose &target_pose, std::string &reference_frame,
    moveit::planning_interface::MoveGroupInterface::Plan &plan,
    std::string &end_effector_name) {
  move_group_interface.clearPoseTargets();
  move_group_interface.setPoseReferenceFrame(reference_frame);
  move_group_interface.setPlanningTime(options.set_planning_time);
  move_group_interface.allowReplanning(options.allow_replanning);
  move_group_interface.setNumPlanningAttempts(options.num_attempts);
  move_group_interface.setStartState(*move_group_interface.getCurrentState());
  move_group_interface.setPoseTarget(target_pose);
  move_group_interface.setEndEffector(end_effector_name + "_ee");
  move_group_interface.setPlannerId("TRRTkConfigDefault");
  ROS_INFO("Planning for: %s", move_group_interface.getEndEffector().c_str());

  // Do planning for entire group
  bool plan_success = false;
  plan_success = (move_group_interface.plan(plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Return success or failure
  if (plan_success) {
    ROS_INFO("Plan Succeeded");
  } else {
    ROS_ERROR("Plan Failed");
  }
  ros::Duration(2.0).sleep();

  return plan_success;
}

bool shadow_planning::planToJointTargets(
    shadow_planning::PlanningOptions &options,
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    std::string &reference_frame,
    moveit::planning_interface::MoveGroupInterface::Plan &plan,
    std::map<std::string, double> &joint_targets) {
  move_group_interface.clearPoseTargets();
  move_group_interface.setPoseReferenceFrame(reference_frame);
  move_group_interface.setPlanningTime(options.set_planning_time);
  move_group_interface.allowReplanning(options.allow_replanning);
  move_group_interface.setNumPlanningAttempts(options.num_attempts);
  move_group_interface.setGoalPositionTolerance(
      options.goal_position_tolerance);
  move_group_interface.setGoalOrientationTolerance(
      options.goal_orientation_tolerance);
  move_group_interface.setGoalJointTolerance(options.goal_joint_tolerance);
  move_group_interface.setStartState(*move_group_interface.getCurrentState());
  move_group_interface.setPlannerId("TRRTkConfigDefault");

  move_group_interface.setJointValueTarget(joint_targets);
  ROS_INFO("Planning for: %s", move_group_interface.getEndEffector().c_str());

  // Do planning for entire group
  bool plan_success = false;
  plan_success = (move_group_interface.plan(plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Return success or failure
  if (plan_success) {
    ROS_INFO("Plan Succeeded");
  } else {
    ROS_ERROR("Plan Failed");
  }
  ros::Duration(2.0).sleep();

  return plan_success;
}

bool shadow_planning::executePlan(
    std::vector<ros::Publisher> &controller_pubs,
    moveit::planning_interface::MoveGroupInterface::Plan &plan) {

  // This is the trajectory generated by MoveIt plan
  moveit_msgs::RobotTrajectory robotTrajectory = plan.trajectory_;
  // Convert MoveIt to trajectory_msgs joint trajectory
  trajectory_msgs::JointTrajectory planTrajectory =
      robotTrajectory.joint_trajectory;
  // Gives all the joint names in the planning group used when planning
  std::vector<std::string> jointNames = planTrajectory.joint_names;
  // Extract trajectory points for each joint in the planning group
  std::vector<trajectory_msgs::JointTrajectoryPoint> trajectoryPoints =
      planTrajectory.points;
  ROS_INFO("Starting plan execution...");
  // Prepare for control commands
  std_msgs::Float64 control_target;
  ros::Rate loop_rate(250);
  // trajectoryPoints.size() is the total number of points in the trajectory
  // Each JointTrajectoryPoint has jointNames.size() values, one for each joint
  for (auto &trajectoryPoint : trajectoryPoints) {
    for (int j = 0; j < jointNames.size(); j++) {
      // MoveIt plans give joint order in reverse order
      control_target.data = trajectoryPoint.positions[jointNames.size() - j];
      controller_pubs[j].publish(control_target);
      loop_rate.sleep();
    }
  }
  ROS_INFO("Done Executing");
  return true;
}

void shadow_planning::moveFingerJoints(shadow_finger::Finger &finger,
                                       std::vector<double> &targetJointAngles) {
  // Prepare for control commands
  std_msgs::Float64 control_target;
  std::vector<std::string> joint_names = shadow_finger::getJointNames(finger);
  ros::Rate loop_rate(100);
  for (int i = 0; i < targetJointAngles.size(); i++) {
    control_target.data = targetJointAngles[i];

    // ROS_INFO("%s --> %s --> %f", joint_names[i].c_str(),
    // finger._joint_controller_publishers[i].getTopic().c_str(),
    // targetJointAngles[i]);

    finger._joint_controller_publishers[i].publish(control_target);
    ros::Duration(0.5).sleep();
  }
}

bool shadow_planning::executePlan(shadow_finger::Finger &finger) {

  // This is the trajectory generated by MoveIt plan
  moveit_msgs::RobotTrajectory robotTrajectory = finger._plan.trajectory_;
  // Convert MoveIt to trajectory_msgs joint trajectory
  trajectory_msgs::JointTrajectory planTrajectory =
      robotTrajectory.joint_trajectory;
  // Gives all the joint names in the planning group used when planning
  std::vector<std::string> jointNames = planTrajectory.joint_names;
  // Extract trajectory points for each joint in the planning group
  std::vector<trajectory_msgs::JointTrajectoryPoint> trajectoryPoints =
      planTrajectory.points;
  ROS_INFO_STREAM("Starting plan execution for " << finger._finger_name);
  // Prepare for control commands
  std_msgs::Float64 control_target;
  ros::Rate loop_rate(250);
  // trajectoryPoints.size() is the total number of points in the trajectory
  // Each JointTrajectoryPoint has jointNames.size() values, one for each joint
  for (auto &trajectoryPoint : trajectoryPoints) {
    for (int j = 0; j < jointNames.size(); j++) {
      // MoveIt plans give joint order in reverse order
      control_target.data = trajectoryPoint.positions[jointNames.size() - j];
      finger._joint_controller_publishers[j].publish(control_target);
      loop_rate.sleep();
    }
  }
  ROS_INFO("Done Executing");
  return true;
}

void shadow_planning::executePlanAsync(shadow_finger::Finger &finger) {
  std::future<bool> exec_success = std::async(
      std::launch::async,
      [](shadow_finger::Finger &f) { return shadow_planning::executePlan(f); },
      std::ref(finger));
  exec_success.get();
}

void shadow_planning::executePlanAsync(shadow_hand::Hand &hand) {
  std::vector<std::shared_ptr<shadow_finger::Finger>> fingers =
      hand.getFingers();
  for (auto &finger : fingers) {
    shadow_planning::executePlanAsync(*finger);
  }
}