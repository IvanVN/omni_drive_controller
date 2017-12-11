#include <boost/circular_buffer.hpp>

#include <controller_interface/multi_interface_controller.h>
#include <controller_interface/controller_base.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

namespace omni_drive_controller
{
namespace ControllerStates
{
enum ControllerState
{
  Init,
  Moving,
  Braking,
};
}
typedef ControllerStates::ControllerState ControllerState;

enum
{
  FRONT_RIGHT_TRACTION_JOINT = 0,
  FRONT_LEFT_TRACTION_JOINT = 1,
  BACK_RIGHT_TRACTION_JOINT = 2,
  BACK_LEFT_TRACTION_JOINT = 3,
  FRONT_RIGHT_DIRECTION_JOINT = 4,
  FRONT_LEFT_DIRECTION_JOINT = 5,
  BACK_RIGHT_DIRECTION_JOINT = 6,
  BACK_LEFT_DIRECTION_JOINT = 7,
  BEGIN_TRACTION_JOINT = 0,
  END_TRACTION_JOINT = 4,
  BEGIN_DIRECTION_JOINT = 4,
  END_DIRECTION_JOINT = 8,
  NUMBER_OF_JOINTS = 8,
  NUMBER_OF_WHEELS = 4
};

class OmniDriveController
    : public controller_interface::MultiInterfaceController<hardware_interface::VelocityJointInterface,
                                                            hardware_interface::PositionJointInterface>

/* The standard way on indigo is to inherit from controller_interface::Controller<T>,
where T is a JointInterface. So a controller can access to only one type of JointInterface

As a hack, if we inherit from ControllerBase instead, we can access to two different JointInterfaces
In that case, we need to implement:
    initRequest, which receives a RobotHW instead of a hardware_interface::XJointInterface, from where the interfaces
can be accessed
    getHardwareInterfaceType, which returns a string with the main type of JointInterface. In our case, it is a
VelocityJointInterface
*/

{
public:
  OmniDriveController();

  /**
  */
  bool init(hardware_interface::RobotHW* const robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

  bool initVelocityInterface(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh,
                             ros::NodeHandle& controller_nh);

  bool initPositionInterface(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& root_nh,
                             ros::NodeHandle& controller_nh);

  /**
   * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
   * \param time   Current time
   * \param period Time since the last called to update
   */
  virtual void update(const ros::Time& time, const ros::Duration& period);

  /**
   * \brief Starts controller
   * \param time Current time
   */
  virtual void starting(const ros::Time& time);

  /**
   * \brief Stops controller
   * \param time Current time
   */
  virtual void stopping(const ros::Time& /*time*/);

  virtual std::string getHardwareInterfaceType() const;

private:
  // Control stuff
  std::vector<hardware_interface::JointHandle> joints_;  // joint handles: to read current state and send commands
  std::vector<std::string> joint_names_;                 // joint names: to get the handle of each joint

  std::vector<std::pair<double, double> > joint_limits_;  // lower, upper limits

  std::vector<double> joint_states_;       // current joint state: position or velocity
  std::vector<double> joint_states_mean_;  // current joint state mean: used to calculate the reference according to the
                                           // constraints
  std::vector<double> joint_references_;   // current reference for each joint
  std::vector<double> joint_commands_;  // current command to be sent: may differ from reference is the wheels are not
                                        // in position or if the watchdog times out

  std::vector<boost::circular_buffer<double> > joint_states_history_;  // used to calculate the current joint state as
                                                                       // the mean of the previous joint states
  unsigned int joint_states_history_size_;                             // size of the joint history

  // Data
  geometry_msgs::Twist received_cmd_;  // holds last velocity command
  geometry_msgs::Twist current_cmd_;  // hold current used command (limited). it is updated on the limitCommand function
                                      // and in the writeJointCommands, because if the direction wheels are not in
                                      // position, the traction reference is 0
  geometry_msgs::Twist last_valid_cmd_;  // hold current used command (limited). it is updated on the limitCommand
                                         // function and in the writeJointCommands, because if the direction wheels are
                                         // not in position, the traction reference is 0
  ros::Time cmd_last_stamp_;             // holds last velocity command time stamp, used to check the watchdog
  nav_msgs::Odometry odom_;              // holds odometry
  geometry_msgs::Pose2D pose_encoder_;   // holds position calculated from encoders

  bool cmd_watchdog_timedout_;           // true is the watchdog has been activated
  ros::Duration cmd_watchdog_duration_;  // time that has to pass to activate the watchdog

  bool odom_broadcast_tf_;             // if true, odom must be published also in tf
  ros::Duration odom_publish_period_;  // time between odometry publication updates
  ros::Time odom_last_sent_;           // to check if the odometry must be sent
  ros::Time odom_last_update_;         // to use in the odometry calculation

  // Wheels configuration
  double wheel_base_;      // distance between front and rear axles
  double track_width_;     // distance between right and left wheels
  double wheel_diameter_;  // wheel diamater, to convert from angular speed to linear

  // Speed and acceleration limits
  double linear_speed_limit_;
  double linear_acceleration_limit_;
  double angular_speed_limit_;
  double angular_acceleration_limit_;

  bool wheels_on_position_;
  bool wait_for_reconfiguration_;
  bool do_not_write_commands_;
  bool robot_is_stopped_;

  ros::Publisher current_cmd_pub_;
  ros::Publisher commands_pub_;

  // ROS stuff
  std::string controller_name_;   // node name,
  std::string command_topic_;     // topic from where velocity commands are read
  std::string odom_topic_;        // name of the topic where the odometry is published
  std::string odom_frame_;        // name of the frame associated to the odometry
  std::string robot_base_frame_;  // name of the frame associated to the robot. odom_frame_ is published as it's parent

  ros::Publisher odom_publisher_;                    // topic publisher where the odometry is published
  ros::Subscriber cmd_vel_subscriber_;               // topic subscriber to receive velocity commands
  tf::TransformBroadcaster* transform_broadcaster_;  // to publish odom frame
  //
  void readJointStates();
  void writeJointCommands();
  void limitCommand(double period, geometry_msgs::Twist goal_cmd);
  void updateJointStateHistoryMean();
  void updateJointReferences();
  void setJointPositionReferenceWithLessChange(double& wheel_speed, double& wheel_angle, double current_wheel_speed,
                                               double current_wheel_angle);
  void setJointPositionReferenceBetweenMotorWheelLimits(double& wheel_speed, double& wheel_angle, int joint_number);
  void setJointVelocityReferenceBetweenLimits(std::vector<double>& wheel_speed);
  void updateOdometryFromEncoder();
  void publishOdometry();
  bool initController(ros::NodeHandle root_nh, ros::NodeHandle controller_nh);
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

  bool checkJointPositionReferenceIsBetweenMotorWheelLimits(double wheel_angle, double range, int joint_number);
  void setJointConfigurationAsMirror(double& wheel_speed, double& wheel_angle);

  void hardRobotBrake();
  void softRobotBrake(double period);
  bool areDirectionWheelsOriented(double max_range);
  bool areTractionWheelsOnSameDirection(double max_range);
  void orientWheels();
  void setJointCommandsAsReferences();

  ControllerState controller_state_;
};
PLUGINLIB_EXPORT_CLASS(omni_drive_controller::OmniDriveController, controller_interface::ControllerBase);
}
