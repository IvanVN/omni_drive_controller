#include <boost/circular_buffer.hpp>

#include <controller_interface/controller.h>
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
    
    enum {
        FRONT_RIGHT_W=0,
        FRONT_LEFT_W=1,
        BACK_RIGHT_W=2,
        BACK_LEFT_W=3,
        FRONT_RIGHT_MW=4,
        FRONT_LEFT_MW=5,
        BACK_RIGHT_MW=6,
        BACK_LEFT_MW=7,
        BEGIN_W=0,
        END_W=4,
        BEGIN_MW=4,
        END_MW=8,
        NUMBER_OF_JOINTS=8,
        NUMBER_OF_WHEELS=4
    };

    class OmniDriveController : 
    public controller_interface::ControllerBase
    
    /* The standard way on indigo is to inherit from controller_interface::Controller<T>,
    where T is a JointInterface. So a controller can access to only one type of JointInterface

    As a hack, if we inherit from ControllerBase instead, we can access to two different JointInterfaces
    In that case, we need to implement:
        initRequest, which receives a RobotHW instead of a hardware_interface::XJointInterface, from where the interfaces can be accessed
        getHardwareInterfaceType, which returns a string with the main type of JointInterface. In our case, it is a VelocityJointInterface
    */

{
    public:
    OmniDriveController();

    
    /**
    */ 
    bool initRequest(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh,
            std::set<std::string> &claimed_resources);
    
    bool initVelocityInterface(hardware_interface::VelocityJointInterface* hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle &controller_nh);


    bool initPositionInterface(hardware_interface::PositionJointInterface* hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle &controller_nh);

    /**
     * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
     * \param time   Current time
     * \param period Time since the last called to update
     */
    void update(const ros::Time& time, const ros::Duration& period);

    /**
     * \brief Starts controller
     * \param time Current time
     */
    void starting(const ros::Time& time);

    /**
     * \brief Stops controller
     * \param time Current time
     */
    void stopping(const ros::Time& /*time*/);
    
    virtual std::string getHardwareInterfaceType() const;

    private:

    // Control stuff. 
    std::vector<hardware_interface::JointHandle> joints_; // joint handles: to read current state and send commands, // XXX:initialized
    std::vector<std::string> joint_names_; // joint names: to get the handle of each joint, // XXX:initialized

    std::vector<std::pair<double, double> > joint_limits_; //lower, upper limits, // XXX:initialized
    
	std::vector<double> joint_states_; // current joint state: position or velocity, // XXX: initialized
    std::vector<double> joint_states_mean_; // current joint state mean: used to calculate the reference according to the constraints, // XXX: initialized
    std::vector<double> joint_references_; // current reference for each joint, // XXX: initialized
	std::vector<double> joint_commands_; // current command to be sent: may differ from reference is the wheels are not in position or if the watchdog times out, // XXX:initialized

    std::vector<boost::circular_buffer<double> > joint_states_history_; // used to calculate the current joint state as the mean of the previous joint states, // XXX:initialized
    unsigned int joint_states_history_size_; // size of the joint history,  XXX:initialized
    
    // Data
    geometry_msgs::Twist cmd_; // holds last velocity command, // XXX: initialized on starting
    ros::Time cmd_last_stamp_; // holds last velocity command time stamp, used to check the watchdog, // XXX: initialized on starting
    nav_msgs::Odometry odom_; // holds odometry, // XXX: initialized on starting
    geometry_msgs::Pose2D pose_encoder_; // holds position calculated from encders, // XXX: initialized on starting

    bool cmd_watchdog_timedout_; // true is the watchdog has been activated., // XXX: initializedon starting
    ros::Duration cmd_watchdog_duration_; // time that has to pass to activate the watchdog, // XXX: initialized

    bool odom_broadcast_tf_; // if true, odom must be published also in tf, // XXX: initialized
    ros::Duration odom_publish_period_; // time between odometry publication updates, // XXX: initialized
    ros::Time odom_last_sent_; // to check if the odometry must be sent // XXX: initialized on starting
    ros::Time odom_last_update_; // to use in the odometry calculation // XXX: initialized on starting

    // Wheels configuration
    double wheel_base_;     // distance between front and rear axles, //XXX initialized
    double track_width_;    // distance between right and left wheels, //XXX initialized
    double wheel_diameter_; // wheel diamater, to convert from angular speed to linear, //XXX initialized
    double wheel_torque_;   // ??, //XXX initialized
     
    // ROS stuff
    std::string controller_name_; // node name, //XXX initialized
    std::string command_topic_; // topic from where velocity commands are read //XXX initialized
    std::string odom_topic_; // name of the topic where the odometry is published //XXX initialized
    std::string odom_frame_; // name of the frame associated to the odometry //XXX initialized
    std::string robot_base_frame_; // name of the frame associated to the robot. odom_frame_ is published as it's parent //XXX initialized

    ros::Publisher odom_publisher_; // topic publisher where the odometry is published //XXX initialized
    ros::Subscriber cmd_vel_subscriber_; // topic subscriber to receive velocity commands //XXX initialized
    tf::TransformBroadcaster *transform_broadcaster_; // to publish odom frame
    //
    void readJointStates();
    void writeJointCommands();
    void updateJointStateHistoryMean();
    void updateJointReferences();
    void setJointReferenceWithLessChange(double &wheel_speed, double &wheel_angle, double current_wheel_speed, double current_wheel_angle);
    void setJointReferenceBetweenMotorWheelLimits(double &wheel_speed, double &wheel_angle, int joint_number);
    void updateOdometryFromEncoder();
    void publishOdometry();
    bool initController(ros::NodeHandle root_nh, ros::NodeHandle controller_nh);
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
      
};
PLUGINLIB_EXPORT_CLASS(omni_drive_controller::OmniDriveController, controller_interface::ControllerBase);
}
