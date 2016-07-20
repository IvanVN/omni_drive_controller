#include <controller_interface/controller.h>
//#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

namespace omni_drive_controller
{
    class OmniDriveController : 
    public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
    public:
    OmniDriveController();

    bool init(hardware_interface::VelocityJointInterface* hw,
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
    private:

    std::vector<hardware_interface::JointHandle> wheel_joints_;

};
PLUGINLIB_EXPORT_CLASS(omni_drive_controller::OmniDriveController, controller_interface::ControllerBase);
}
