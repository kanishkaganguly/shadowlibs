//
// Created by kganguly on 2/18/18.
//

#include <shadowlibs/shadow_utils.hpp>

std::ostream& operator<<(std::ostream& out, const geometry_msgs::Pose& pose) {
    out << "Trans(" << pose.position.x << ", " << pose.position.y << ", "
        << pose.position.z << ")\n"
        << "Rot(" << pose.orientation.x << ", " << pose.orientation.y << ", "
        << pose.orientation.z << ", " << pose.orientation.w << ")";

    return out; // return std::ostream so we can chain calls to operator<<
}

Eigen::Quaternionf shadow_utils::ypr2quat(float y, float p, float r) {
    Eigen::Quaternionf q;
    return q = Eigen::AngleAxisf(r, Eigen::Vector3f::UnitX()) *
               Eigen::AngleAxisf(p, Eigen::Vector3f::UnitY()) *
               Eigen::AngleAxisf(r, Eigen::Vector3f::UnitZ());
}

void shadow_utils::quat2ypr(tf::Quaternion& q, double& r, double& p, double& y) {
    tf::Matrix3x3 m(q);
    m.getRPY(r, p, y);
}

void shadow_utils::getPoseFromPositionQuaternion(geometry_msgs::Pose& pose,
                                                 float x, float y, float z,
                                                 float qx, float qy, float qz,
                                                 float qw) {
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.w = qw;
    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;
}

void shadow_utils::getPoseFromPositionQuaternion(geometry_msgs::Pose& pose,
                                                 float x, float y, float z,
                                                 tf::Quaternion& q) {
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
}

void shadow_utils::getPoseFromPositionQuaternion(geometry_msgs::Pose& pose,
                                                 tf::StampedTransform& tf,
                                                 tf::Quaternion& q) {
    pose.position.x = tf.getOrigin().x();
    pose.position.y = tf.getOrigin().y();
    pose.position.z = tf.getOrigin().z();
    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
}

void shadow_utils::getPoseFromPositionYPR(geometry_msgs::Pose& pose, float x,
                                          float y, float z, float yaw,
                                          float pitch, float roll) {
    tf::Quaternion q;
    q.setEuler(yaw, pitch, roll);
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
}

void shadow_utils::getPoseFromPositionYPR(geometry_msgs::Pose& pose, float x,
                                          float y, float z, tf::Vector3& v) {
    tf::Quaternion q;
    q.setEuler(v.x(), v.y(), v.z());
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
}

void shadow_utils::createCollisionObjectFromPrimitive(
        moveit_msgs::CollisionObject& collision_obj, std::string primitive_id,
        shape_msgs::SolidPrimitive& primitive, geometry_msgs::Pose& obj_pose,
        shadow_utils::primitive_type primitive_type,
        const std::vector<double>& primitive_dim) {
    try {
        ROS_INFO("Creating primitive object: %s", primitive_id.c_str());

        collision_obj.id = primitive_id;
        collision_obj.header.frame_id = "/world";

        switch (primitive_type) {
            case shadow_utils::primitive_type::BOX:
                primitive.type = primitive.BOX;
                primitive.dimensions.resize(3);
                break;
            case shadow_utils::primitive_type::SPHERE:
                primitive.type = primitive.SPHERE;
                primitive.dimensions.resize(1);
                break;
            case shadow_utils::primitive_type::CYLINDER:
                primitive.type = primitive.CYLINDER;
                primitive.dimensions.resize(2);
                break;
            case shadow_utils::primitive_type::CONE:
                primitive.type = primitive.CONE;
                primitive.dimensions.resize(2);
                break;
        }

        for (size_t i = 0; i < primitive_dim.size(); i++) {
            primitive.dimensions[i] = primitive_dim[i];
        }
        collision_obj.primitives.clear();
        collision_obj.primitives.push_back(primitive);
        collision_obj.primitive_poses.clear();
        collision_obj.primitive_poses.push_back(obj_pose);
    } catch (const std::exception& e) {
        std::cout << e.what() << std::endl;
    }
}

void shadow_utils::createAttachedObjectFromPrimitive(
        moveit_msgs::AttachedCollisionObject& attached_obj,
        std::string primitive_id, shape_msgs::SolidPrimitive& primitive,
        geometry_msgs::Pose& obj_pose, shadow_utils::primitive_type primitive_type,
        const std::vector<double>& primitive_dim) {
    attached_obj.object.id = primitive_id;
    attached_obj.object.header.frame_id = "/world";
    switch (primitive_type) {
        case shadow_utils::primitive_type::BOX:
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            break;
        case shadow_utils::primitive_type::SPHERE:
            primitive.type = primitive.SPHERE;
            primitive.dimensions.resize(1);
            break;
        case shadow_utils::primitive_type::CYLINDER:
            primitive.type = primitive.CYLINDER;
            primitive.dimensions.resize(2);
            break;
        case shadow_utils::primitive_type::CONE:
            primitive.type = primitive.CONE;
            primitive.dimensions.resize(2);
            break;
    }

    for (size_t i = 0; i < primitive_dim.size(); i++) {
        primitive.dimensions[i] = primitive_dim[i];
    }
    attached_obj.object.primitives.clear();
    attached_obj.object.primitives.push_back(primitive);
    attached_obj.object.primitive_poses.clear();
    attached_obj.object.primitive_poses.push_back(obj_pose);
}

void shadow_utils::addCollisionObjectToScene(
        moveit_msgs::CollisionObject& collision_obj,
        moveit_msgs::PlanningScene& planning_scene_msg,
        ros::Publisher& planning_scene_diff_publisher) {
    ROS_INFO("Adding object to scene: %s", collision_obj.id.c_str());

    collision_obj.operation = collision_obj.ADD;
    planning_scene_msg.world.collision_objects.clear();
    planning_scene_msg.world.collision_objects.push_back(collision_obj);
    planning_scene_msg.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene_msg);
    ros::Duration(5.0).sleep();
}

void shadow_utils::addAttachedObjectToScene(
        moveit_msgs::AttachedCollisionObject& attached_obj,
        moveit_msgs::PlanningScene& planning_scene_msg,
        ros::Publisher& planning_scene_diff_publisher) {
    ROS_INFO("Adding object to scene: %s", attached_obj.object.id.c_str());

    attached_obj.object.operation = attached_obj.object.ADD;
    planning_scene_msg.world.collision_objects.clear();
    planning_scene_msg.world.collision_objects.push_back(attached_obj.object);
    planning_scene_msg.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene_msg);
    ros::Duration(5.0).sleep();
}

void shadow_utils::moveCollisionObjectInScene(
        moveit_msgs::CollisionObject& collision_obj, geometry_msgs::Pose& obj_pose,
        moveit_msgs::PlanningScene& planning_scene_msg,
        ros::Publisher& planning_scene_diff_publisher) {
    ROS_INFO("Moving object in scene: %s", collision_obj.id.c_str());

    collision_obj.operation = collision_obj.MOVE;
    collision_obj.primitive_poses.clear();
    collision_obj.primitive_poses.push_back(obj_pose);
    planning_scene_msg.world.collision_objects.clear();
    planning_scene_msg.world.collision_objects.push_back(collision_obj);
    planning_scene_msg.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene_msg);
    ros::Duration(5.0).sleep();
}

void shadow_utils::attachObjectToRobot(
        moveit_msgs::AttachedCollisionObject& attached_obj,
        moveit_msgs::PlanningScene& planning_scene_msg,
        ros::Publisher& planning_scene_diff_publisher) {
    ROS_INFO("Attaching object to robot: %s", attached_obj.object.id.c_str());

    planning_scene_msg.world.collision_objects.clear();
    attached_obj.object.operation = attached_obj.object.REMOVE;
    planning_scene_msg.world.collision_objects.push_back(attached_obj.object);
    planning_scene_msg.is_diff = true;

    planning_scene_msg.robot_state.attached_collision_objects.clear();
    attached_obj.object.operation = attached_obj.object.ADD;
    planning_scene_msg.robot_state.attached_collision_objects.push_back(
            attached_obj);
    planning_scene_msg.robot_state.is_diff = true;

    planning_scene_diff_publisher.publish(planning_scene_msg);
    ros::Duration(5.0).sleep();
}

void shadow_utils::detachObjectFromRobot(
        moveit_msgs::AttachedCollisionObject& attached_obj,
        moveit_msgs::PlanningScene& planning_scene_msg,
        ros::Publisher& planning_scene_diff_publisher) {
    ROS_INFO("Detaching object from robot: %s", attached_obj.object.id.c_str());

    attached_obj.object.operation = attached_obj.object.REMOVE;
    planning_scene_msg.robot_state.attached_collision_objects.clear();
    planning_scene_msg.robot_state.attached_collision_objects.push_back(
            attached_obj);
    planning_scene_msg.robot_state.is_diff = true;

    attached_obj.object.operation = attached_obj.object.ADD;
    planning_scene_msg.world.collision_objects.clear();
    planning_scene_msg.world.collision_objects.push_back(attached_obj.object);
    planning_scene_msg.is_diff = true;

    planning_scene_diff_publisher.publish(planning_scene_msg);
    ros::Duration(5.0).sleep();
}

void shadow_utils::removeAllObjectsFromScene(moveit::planning_interface::PlanningSceneInterface
                                             & planning_scene_interface) {
    ROS_INFO("Remove all primitives from scene");
    planning_scene_interface.removeCollisionObjects(
            planning_scene_interface.getKnownObjectNames());
    ros::Duration(5.0).sleep();
}

void shadow_utils::removeCollisionObjectFromScene(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                                                  std::string obj_id) {
    ROS_INFO("Remove object from scene: %s", obj_id.c_str());
    planning_scene_interface.removeCollisionObjects(
            std::vector<std::string>({obj_id}));
    ros::Duration(5.0).sleep();
}

void shadow_utils::getTfFromFrames(tf::TransformListener& listener,
                                   tf::StampedTransform& tf,
                                   std::string parent_frame,
                                   std::string child_frame) {
    ROS_INFO("Fetching TF between %s and %s", parent_frame.c_str(),
             child_frame.c_str());
    try {
        listener.waitForTransform(parent_frame, child_frame, ros::Time(0),
                                  ros::Duration(3.0));
        listener.lookupTransform(parent_frame, child_frame, ros::Time(0), tf);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

std::string shadow_utils::getControllerTopic(std::string& joint_name) {
    std::string formatted_joint_name = joint_name.substr(3, std::string::npos);
    std::transform(formatted_joint_name.begin(), formatted_joint_name.end(),
                   formatted_joint_name.begin(), ::tolower);
    // We need to replace J1 and J2 with J0 for all fingers except thumb
    // This is due to coupling on the hardware simulator, and needs to be removed
    // for actual hardware
    std::size_t check_not_thumb = formatted_joint_name.find("th");
    std::size_t check_not_wrist = formatted_joint_name.find("wr");
    std::string out;
    if (check_not_thumb == std::string::npos &&
        check_not_wrist == std::string::npos) {
        std::size_t check_j1 = formatted_joint_name.find("j1");
        std::size_t check_j2 = formatted_joint_name.find("j2");
        if (check_j1 != std::string::npos) {
            formatted_joint_name.replace(check_j1, 2, "j0");
        } else if (check_j2 != std::string::npos) {
            formatted_joint_name.replace(check_j2, 2, "j0");
        }
        out = "/sh_rh_" + formatted_joint_name + "_position_controller/command";
    } else {
        out = "/sh_rh_" + formatted_joint_name + "_position_controller/command";
    }

    return out;
};

std::vector <ros::Publisher> shadow_utils::createJointControllerPublishers(std::vector <std::string>& joint_names, ros::NodeHandle& n) {
    std::vector <ros::Publisher> joint_controller_pubs;
    for (int i = 0; i < joint_names.size(); i++) {
        // Generate topic for joint controller to publish on
        std::string controller_name =
                shadow_utils::getControllerTopic(joint_names[i]);
        // Create publisher
        ros::Publisher controller_target_pub;
        // Advertise topic
        controller_target_pub =
                n.advertise<std_msgs::Float64>(controller_name.c_str(), 1000);
        ROS_INFO_STREAM(
                "Initializing joint controller publisher: " << controller_name);
        // Push back the publisher
        joint_controller_pubs.emplace_back(controller_target_pub);
    }
    return joint_controller_pubs;
}

geometry_msgs::Pose shadow_utils::getPoseBetweenFrames(const std::string parent_frame, const std::string child_frame) {
    /* Get transform from wrist to world for final grasping */
    tf::TransformListener lr;
    tf::StampedTransform tform;
    geometry_msgs::TransformStamped stamped_tf;
    geometry_msgs::Pose poseParentToChild;

    try {
        lr.waitForTransform(child_frame, parent_frame, ros::Time(0),
                            ros::Duration(1.0));
        lr.lookupTransform(child_frame, parent_frame, ros::Time(0), tform);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
    }

    tf::transformStampedTFToMsg(tform, stamped_tf);
    poseParentToChild.position.x = stamped_tf.transform.translation.x;
    poseParentToChild.position.y = stamped_tf.transform.translation.y;
    poseParentToChild.position.z = stamped_tf.transform.translation.z;
    poseParentToChild.orientation = stamped_tf.transform.rotation;

    return poseParentToChild;
}

void shadow_utils::broadcastTransformForGrasp(std::vector<double> xyzrpy, const std::string frame_name) {
    /*
    This publishes xyzrpy from final grasp positions w.r.t. wrist
    We can then use TF to get final grasp positions w.r.t. world frame for
    planning
    */
    static tf::TransformBroadcaster br;
    tf::Transform tform;
    tf::Quaternion q, x_axis, y_axis, z_axis;

    /*
    Map transforms from GraspIt to ROS
    (X)-->(-Y) (Y)-->(-Z) (Z)-->(X)
    */
    tform.setOrigin(tf::Vector3(-(xyzrpy[1]) / 1000.0, -(xyzrpy[2]) / 1000.0,
                                xyzrpy[0] / 1000.0));

    q = tf::createQuaternionFromRPY(xyzrpy[3], xyzrpy[4], xyzrpy[5]).normalize();
    x_axis = tf::createQuaternionFromRPY(1.5708, 0, 0).normalize();
    y_axis = tf::createQuaternionFromRPY(0, 1.5708, 0).normalize();
    z_axis = tf::createQuaternionFromRPY(0, 0, -1.5708).normalize();
    tform.setRotation(q.normalize());

    br.sendTransform(
            tf::StampedTransform(tform, ros::Time::now(), "/rh_wrist", frame_name));
}

moveit_msgs::RobotState shadow_utils::computeJointsFromPose(moveit::planning_interface::MoveGroupInterface& move_group_interface,
                                                            double timeout, int num_attempts, geometry_msgs::Pose& target_pose) {
    // Initialize IK service client
    ros::NodeHandle n;
    ros::ServiceClient client =
            n.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
    moveit_msgs::GetPositionIK ik_srv;

    // Get current state
    moveit::core::RobotState seed_state = *move_group_interface.getCurrentState();
    moveit_msgs::RobotState seed_state_msg;
    moveit::core::robotStateToRobotStateMsg(seed_state, seed_state_msg);

    // Set up IK service call
    moveit_msgs::PositionIKRequest ik_request;
    geometry_msgs::PoseStamped target_pose_stamped;
    target_pose_stamped.header.stamp = ros::Time(0);
    target_pose_stamped.pose = target_pose;
    ik_request.robot_state = seed_state_msg;
    ik_request.ik_link_name = move_group_interface.getEndEffector();
    ik_request.pose_stamped = target_pose_stamped;
    ik_request.group_name = move_group_interface.getName();

    // Create IK call
    ik_srv.request.ik_request = ik_request;

    // Call IK service
    moveit_msgs::RobotState computed_state;
    if (client.call(ik_srv)) {
        moveit_msgs::MoveItErrorCodes result;
        result = (moveit_msgs::MoveItErrorCodes) ik_srv.response.error_code;
        if (result.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
            ROS_INFO("Successfully computed IK");
            computed_state = (moveit_msgs::RobotState) ik_srv.response.solution;
        } else if (result.val == moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION) {
            ROS_INFO("ERROR: %d", moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION);
        }
    } else {
        ROS_ERROR("Failed to call IK service");
    }

    return computed_state;
}
