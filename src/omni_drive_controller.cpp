#include <omni_drive_controller/omni_drive_controller.h>

#include <ros/ros.h>
namespace omni_drive_controller
{
    OmniDriveController::OmniDriveController()
    {
    }

    bool OmniDriveController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh)
    {
        wheel_joints_.resize(2);
        wheel_joints_[0] = hw->getHandle("back_left_wheel_joint");
        wheel_joints_[1] = hw->getHandle("back_right_wheel_joint");

        return true;
    }
    
    void OmniDriveController::update(const ros::Time& time, const ros::Duration& period)
    {
        ROS_INFO_STREAM_THROTTLE(1, "Update!");
        ROS_INFO_STREAM_THROTTLE(1, "j1: " << wheel_joints_[0].getPosition());
        ROS_INFO_STREAM_THROTTLE(1, "j2: " << wheel_joints_[1].getPosition());

        double commands[2] = {1.0, -0.5};
        wheel_joints_[0].setCommand(commands[0]);
        wheel_joints_[1].setCommand(commands[1]);
    }


    /**
     * \brief Starts controller
     * \param time Current time
     */
    void OmniDriveController::starting(const ros::Time& time)
    {
        ROS_INFO("Starting!");
    }

    /**
     * \brief Stops controller
     * \param time Current time
     */
    void OmniDriveController::stopping(const ros::Time& /*time*/)
    {
        ROS_INFO("Stopping!");
    }
}
