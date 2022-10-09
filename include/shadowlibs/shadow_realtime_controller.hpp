#pragma once

#include <shadowlibs/shadow_imports.hpp>

namespace controller {
class ShadowRealtimeController : public SrController {
 public:
  /**
   * The controller manager will instantiate one instance of this
   * class per controlled joint.
   */
  ShadowRealtimeController();

  /**
   * Destructor.
   */
  virtual ~ShadowRealtimeController();

  /**
   * The first init is used to read which joint is being controlled
   * from the parameter server.
   *
   * @param robot A pointer to the robot state, passed to the 2nd init function.
   * @param n The ROS nodehandle, to be able to access the parameter server.
   *
   * @return True if the 2nd init function succeeds.
   */
  bool init(ros_ethercat_model::RobotStateInterface *robot, ros::NodeHandle &n);

  /**
   * This method is called when the controller is started. The command is then
   * to be the current position (or effort / velocity / ... depending on what
   * you're controlling), so that the first command won't move the joint.
   *
   */
  virtual void starting(const ros::Time &time);

  /**
   * Issues commands to the joint. This method is called at the specified rate
   * by the main loop.
   */
  virtual void update(const ros::Time &time, const ros::Duration &period);

 private:
  // publish our joint controller state
  boost::scoped_ptr<
      realtime_tools::RealtimePublisher<sr_robot_msgs::JointControllerState>>
      controller_state_publisher_;

  sr_actuator::SrMotorActuator *actuator_;
};
}  // namespace controller
