#include <algorithm>
#include <sstream>
#include <numeric>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <omni_drive_controller/omni_drive_controller.h>

namespace omni_drive_controller
{
    double radnorm( double value ) 
    {
        while (value > M_PI) value -= M_PI;
        while (value < -M_PI) value += M_PI;
        return value;
    }

    double radnorm2( double value ) 
    {
        while (value > 2.0*M_PI) value -= 2.0*M_PI;
        while (value < -2.0*M_PI) value += 2.0*M_PI;
        return value;
    }

    double radnormHalf( double value)
    { //norms the angle so it is between -M_PI/2 and M_PI/2
        double eps = 1e-5;
        while (value > 0.5*M_PI + eps) value -= M_PI;
        while (value < -0.5*M_PI - eps) value += M_PI;
        return value;
    }

    //norms a double value so if it is rounded to zero when it's below an epsylon
    double normToZero(double value)
    {
        double eps = 1e-4;
        if (std::abs(value) < eps)
            return 0;
        return value;
    }

    //return the sign, as -1 or 1, of the value. 0 is positive
    double sign( double value)
    {
        return (value < 0) ? -1 : 1;
    }

    //checks that v and w have the same sign. 0 is positive
    bool haveSameSign(double v, double w)
    {
        return (v < 0) == (w < 0);
    }

    OmniDriveController::OmniDriveController()
    {
        // TODO: initialize all variables
    }

    bool OmniDriveController::initRequest(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh,
            std::set<std::string> &claimed_resources)
    {

        // check if construction finished cleanly
        if (state_ != CONSTRUCTED){
            ROS_ERROR_STREAM_NAMED(controller_name_, "Cannot initialize this controller because it failed to be constructed");
            return false;
        }
        
        if (!initController(root_nh, controller_nh)) {
            ROS_ERROR_STREAM_NAMED(controller_name_, "Cannot initialize this controller because it failed to initialize");
            return false;
        }


        // get a pointer to the hardware interface
        hardware_interface::VelocityJointInterface * vel_hw = robot_hw->get<hardware_interface::VelocityJointInterface>();
        if (!vel_hw)
        {
            ROS_ERROR_STREAM_NAMED(controller_name_, "This controller requires a hardware interface of type 'hardware_interface::VelocityJointInterface'."
                    " Make sure this is registered in the hardware_interface::RobotHW class.");
            return false;
        }

        hardware_interface::PositionJointInterface * pos_hw = robot_hw->get<hardware_interface::PositionJointInterface>();
        if (!pos_hw)
        {
            ROS_ERROR_STREAM_NAMED(controller_name_, "This controller requires a hardware interface of type 'hardware_interface::PositionJointInterface'."
                    " Make sure this is registered in the hardware_interface::RobotHW class.");
            return false;
        }

        // return which resources are claimed by this controller
        vel_hw->clearClaims();
        pos_hw->clearClaims();

        if (!initVelocityInterface(vel_hw, root_nh, controller_nh) || !initPositionInterface(pos_hw, root_nh, controller_nh))
        {
            ROS_ERROR_STREAM_NAMED(controller_name_, "Failed to initialize the controller");
            return false;
        }
        claimed_resources.clear();

        std::set<std::string> vel_claims = vel_hw->getClaims();
        std::set<std::string> pos_claims = pos_hw->getClaims();

        claimed_resources.insert(vel_claims.begin(), vel_claims.end());
        claimed_resources.insert(pos_claims.begin(), pos_claims.end());

        vel_hw->clearClaims();
        pos_hw->clearClaims();

        // success
        state_ = INITIALIZED;
        return true;
    } 
    bool OmniDriveController::initVelocityInterface(hardware_interface::VelocityJointInterface* hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle &controller_nh)
    {
        joints_[FRONT_RIGHT_W] = hw->getHandle(joint_names_[FRONT_RIGHT_W]);
        joints_[FRONT_LEFT_W] = hw->getHandle(joint_names_[FRONT_LEFT_W]);
        joints_[BACK_RIGHT_W] = hw->getHandle(joint_names_[BACK_RIGHT_W]);
        joints_[BACK_LEFT_W] = hw->getHandle(joint_names_[BACK_LEFT_W]);

        return true;
    }

    bool OmniDriveController::initPositionInterface(hardware_interface::PositionJointInterface* hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle &controller_nh)
    {
        joints_[FRONT_RIGHT_MW] = hw->getHandle(joint_names_[FRONT_RIGHT_MW]);
        joints_[FRONT_LEFT_MW] = hw->getHandle(joint_names_[FRONT_LEFT_MW]);
        joints_[BACK_RIGHT_MW] = hw->getHandle(joint_names_[BACK_RIGHT_MW]);
        joints_[BACK_LEFT_MW] = hw->getHandle(joint_names_[BACK_LEFT_MW]);

        return true;
    }

    bool OmniDriveController::initController(ros::NodeHandle root_nh, ros::NodeHandle controller_nh)
    {

        // TODO: read parameters
        wheel_base_ = 0.934;    
        track_width_ = 0.57;    
        wheel_diameter_ = 0.15; 
        wheel_torque_ = 40.0;   

        controller_name_ = "omni_drive_controller"; // TODO: set name
        command_topic_ = "cmd_vel";
        odom_topic_ = "odom";
        odom_frame_ = "odom";
        robot_base_frame_ = "base_footprint";
        odom_broadcast_tf_ = true;


        cmd_vel_subscriber_ = root_nh.subscribe(command_topic_, 1, &OmniDriveController::cmdVelCallback, this); 
        odom_publisher_ = root_nh.advertise<nav_msgs::Odometry>(odom_topic_, 1);
        transform_broadcaster_ = new tf::TransformBroadcaster();
        

        joints_.resize(NUMBER_OF_JOINTS);

        joint_states_.resize(NUMBER_OF_JOINTS);
        joint_states_mean_.resize(NUMBER_OF_JOINTS);
        joint_references_.resize(NUMBER_OF_JOINTS);
        joint_commands_.resize(NUMBER_OF_JOINTS);

        joint_names_.resize(NUMBER_OF_JOINTS);
        joint_names_[FRONT_RIGHT_W] = "front_right_wheel_joint";
        joint_names_[FRONT_LEFT_W] = "front_left_wheel_joint";
        joint_names_[BACK_RIGHT_W] = "back_right_wheel_joint";
        joint_names_[BACK_LEFT_W] = "back_left_wheel_joint";
        joint_names_[FRONT_RIGHT_MW] = "front_right_motor_wheel_joint";
        joint_names_[FRONT_LEFT_MW] = "front_left_motor_wheel_joint";
        joint_names_[BACK_RIGHT_MW] = "back_right_motor_wheel_joint";
        joint_names_[BACK_LEFT_MW] = "back_left_motor_wheel_joint";
        
        joint_limits_.resize(NUMBER_OF_JOINTS);

        joint_states_history_size_ = 1;
        joint_states_history_.resize(NUMBER_OF_JOINTS);
        for (size_t i = 0; i < NUMBER_OF_JOINTS; i++) {
            joint_states_history_[i] = boost::circular_buffer<double>(joint_states_history_size_);
        }
        
        // set velocity limits
        for (size_t i = BEGIN_W; i < END_W; i++) {
            joint_limits_[i] = std::make_pair(-0.5, 0.5); // TODO: initialize limits 
        }

        // set direction limits
        for (size_t i = BEGIN_MW; i < END_MW; i++) {
            joint_limits_[i] = std::make_pair(-3.0, 3.0); // TODO: initialize limits
        }
        cmd_watchdog_duration_ = ros::Duration(0.1);
        odom_publish_period_ = ros::Duration(1/100);
       
        return true;
    }

    std::string OmniDriveController::getHardwareInterfaceType() const
    {
        // as result of being a Controller which uses different types of JointInterface, return the main interface type
        // in this case, it is a VelocityJointInterface
        return hardware_interface::internal::demangledTypeName<hardware_interface::VelocityJointInterface>();
    }

    /**
     * \brief Starts controller
     * \param time Current time
     */
    void OmniDriveController::starting(const ros::Time& time)
    {
        ROS_INFO_STREAM_NAMED(controller_name_, "Starting!");
        odom_last_sent_ = ros::Time::now();
        odom_last_update_ = ros::Time::now(); // check if there is some value that invalidates the result of a substraction (equals 0). if there isn't any, seta flag for first_update_odometry
        cmd_last_stamp_ = ros::Time(0); // maybe it is better to set it to 0, so if no cmd is received
   
        // TODO: service to restart odometry?
        odom_ = nav_msgs::Odometry();
        cmd_ = geometry_msgs::Twist();
        pose_encoder_ = geometry_msgs::Pose2D();

        cmd_watchdog_timedout_ = true;
    }

    /**
     * \brief Stops controller
     * \param time Current time
     */
    void OmniDriveController::stopping(const ros::Time& /*time*/)
    {
        // TODO: check what to do
        ROS_INFO_STREAM_NAMED(controller_name_, "Stopping!");
    }

    void OmniDriveController::update(const ros::Time& time, const ros::Duration& period)
    {
        // read joint states: 
        //  - convert wheel angular velocity to linear velocity
        //  - normalize motor wheel angular position between [-pi, pi] (only needed for simulation)
        //  - round to 0 if values are below and epsylon
        //  - update joint_states_history_ 
        readJointStates();

        // update odometry info:
        //  - from joint values (closed loop)
        //  - from command (open loop)
        updateOdometryFromEncoder();

        // calculate joint velocity and position references, taking into account some constrains
        updateJointReferences();

        // TODO: rate of odom publishing
        if ((time - odom_last_sent_) > odom_publish_period_) {
            odom_last_sent_ = time;
            publishOdometry();
        }

        cmd_watchdog_timedout_ = ((time - cmd_last_stamp_) > cmd_watchdog_duration_);
        writeJointCommands();

        // TODO: soft brake (slow slowing down) and hard brake (hard slowing down)
    }

    void OmniDriveController::readJointStates()
    {
        // read wheel velocity: convert from angular to linear
        for (size_t i = BEGIN_W; i < END_W; i++) {
            joint_states_[i]  = normToZero(joints_[i].getVelocity() * (wheel_diameter_ / 2.0));
        }

        // read motor wheel position
        for (size_t i = BEGIN_MW; i < END_MW; i++) {
            joint_states_[i] = radnorm(normToZero(joints_[i].getPosition()));
        }

        for (size_t i = 0; i < NUMBER_OF_JOINTS; i++) {
            joint_states_history_[i].push_back(joint_states_[i]);
            double sum = std::accumulate(joint_states_history_[i].begin(), joint_states_history_[i].end(), 0.0);
            joint_states_mean_[i] = sum / joint_states_history_[i].size();
        }
    }
        
    void OmniDriveController::writeJointCommands()
    {
        // set joint_commands_ to the values that must be sent to the actuators.
        // joint_commands_[i] can be, sorted by priority (from more to less):
        //   (1) if watchdog has timedout, vel = 0, pos is not sent
        //   (2) if motorowheels are not in position, vel = 0, pos is sent
        //   (3) default: vel and pos are sent


        if (cmd_watchdog_timedout_) {
            //  TODO: send 0 velocity always, or just the first time
            // set all commands to 0
            for (size_t i = 0; i < NUMBER_OF_JOINTS; i++) {
                joint_commands_[i] = joint_references_[i] = 0;
            }
            //but only send speed commands
            for (size_t i = BEGIN_W; i < END_W; i++) {
                joints_[i].setCommand(joint_commands_[i]);
            }
            return;
        }
        // check motorwheel orientation
        //double mw_orientation_range = 1e-1;
        //double mw_orientation_range = std::sqrt(v_ref_x_*v_ref_x_ + v_ref_y_*v_ref_y_);
        double mw_orientation_range = std::sqrt(odom_.twist.twist.linear.x*odom_.twist.twist.linear.x + odom_.twist.twist.linear.y*odom_.twist.twist.linear.y);

        double min_mw_orientation_range = 5e-2;

        if (mw_orientation_range < min_mw_orientation_range)
            mw_orientation_range = min_mw_orientation_range;

        bool motorwheels_on_position = true;
        for (size_t i = BEGIN_MW; i < END_MW; i++)
            motorwheels_on_position &= (std::abs(joint_references_[i] - joint_states_[i]) < mw_orientation_range);

        // if motorwheels are in position, set velocity commands as reference
        if (motorwheels_on_position) {
            for (size_t i = BEGIN_W; i < END_W; i++) 
                joint_commands_[i] = joint_references_[i] / (wheel_diameter_ / 2.0);

        } else {// if not, set to 0
            for (size_t i = BEGIN_W; i < END_W; i++)
                joint_commands_[i] = 0;
        }

        // always set position command if watchdog hasn't timed out
        for (size_t i = BEGIN_MW; i < END_MW; i++) 
            joint_commands_[i] = joint_references_[i];

        // send commands to actuators
        for (size_t i = 0; i < NUMBER_OF_JOINTS; i++)
            joints_[i].setCommand(joint_commands_[i]);
    }

    void OmniDriveController::updateJointReferences()
    {
        // Speed references for motor control	  
        double vx = cmd_.linear.x;
        double vy = cmd_.linear.y;
        double w = cmd_.angular.z;

        // Vehicle characteristics
        double L = wheel_base_; 
        double W = track_width_;

        // joint references are calculated so they are constrained in the following order:
        // - (1) motorwheel angle is between its rotation limits.
        // - (2) keep the less change between the reference and the current joint state, by setting 
        //   the calculated reference or it's mirrored reference ( by adding/substracting +/-M_PI
        //   to the motor_wheel orientation and switching the sign of the wheel speed.
        // - (3) keep the orientation between -M_PI/2 and +M_PI/2, so the encoders are below the base.
        // in the code, the constraints are checked in reversed order, so the more constraining (1) is checked at the end

       
        std::vector<double> q,a;
        q.resize(4);
        a.resize(4);

        double x1 = L/2.0; double y1 = W/2.0;
        double wx1 = vx + w * y1;
        double wy1 = vy + w * x1;
        q[0] = - sign(wx1) * sqrt( wx1*wx1 + wy1*wy1 );  
        //double a[0] = radnorm( atan2( wy1, wx1 ) );
        a[0] = radnormHalf( atan2( wy1,wx1 )); // contraint (3)
        
        double x2 = L/2.0; double y2 = W/2.0;
        double wx2 = vx - w * y2;
        double wy2 = vy + w * x2;
        q[1] = sign(wx2) * sqrt( wx2*wx2 + wy2*wy2 );
        //double a[1] = radnorm( atan2( wy2, wx2 ) );
        a[1] = radnormHalf( atan2 (wy2,wx2)); // contraint (3)
        
        double x3 = L/2.0; double y3 = W/2.0;
        double wx3 = vx - w * y3;
        double wy3 = vy - w * x3;
        q[2] = sign(wx3)*sqrt( wx3*wx3 + wy3*wy3 );
        //double a[2] = radnorm( atan2( wy3, wx3 ) );
        a[2] = radnormHalf( atan2(wy3 , wx3)); // contraint (3)
        
        double x4 = L/2.0; double y4 = W/2.0;
        double wx4 = vx + w * y4;
        double wy4 = vy - w * x4;
        q[3] = -sign(wx4)*sqrt( wx4*wx4 + wy4*wy4 );
        //double a[3] = radnorm( atan2( wy4, wx4 ) );
        a[3] = radnormHalf( atan2(wy4,wx4)); // contraint (3)
	  
        //constraint (2)
        setJointPositionReferenceWithLessChange(q[0], a[0], joint_states_mean_[FRONT_RIGHT_W], joint_references_[FRONT_RIGHT_MW]);
        setJointPositionReferenceWithLessChange(q[1], a[1], joint_states_mean_[FRONT_LEFT_W],  joint_references_[FRONT_LEFT_MW]);
        setJointPositionReferenceWithLessChange(q[2], a[2], joint_states_mean_[BACK_LEFT_W],   joint_references_[BACK_LEFT_MW]);
        setJointPositionReferenceWithLessChange(q[3], a[3], joint_states_mean_[BACK_RIGHT_W],  joint_references_[BACK_RIGHT_MW]);

        //constraint (1)
        setJointPositionReferenceBetweenMotorWheelLimits(q[0], a[0], FRONT_RIGHT_MW);
        setJointPositionReferenceBetweenMotorWheelLimits(q[1], a[1], FRONT_LEFT_MW);
        setJointPositionReferenceBetweenMotorWheelLimits(q[2], a[2], BACK_LEFT_MW);
        setJointPositionReferenceBetweenMotorWheelLimits(q[3], a[3], BACK_RIGHT_MW);

        //ROS_INFO_THROTTLE(1,"q1234=(%5.2f, %5.2f, %5.2f, %5.2f)   a1234=(%5.2f, %5.2f, %5.2f, %5.2f)", q[0],q[1],q[2],q[3], a[0],a[1],a[2],a[3]);

        // joint velocity references are scaled so each wheel does not exceed it's maximum velocity
        setJointVelocityReferenceBetweenLimits(q);

        // Motor control actions	  
        // Axis are not reversed in the omni (swerve) configuration
        joint_references_[FRONT_RIGHT_W] = q[0];  
        joint_references_[FRONT_LEFT_W] = q[1];
        joint_references_[BACK_LEFT_W] = q[2];
        joint_references_[BACK_RIGHT_W] = q[3];

        joint_references_[FRONT_RIGHT_MW] = a[0];  
        joint_references_[FRONT_LEFT_MW] = a[1];
        joint_references_[BACK_LEFT_MW] = a[2];
        joint_references_[BACK_RIGHT_MW] = a[3];
        
    }


    void OmniDriveController::updateOdometryFromEncoder()
    {
        // Linear speed of each wheel
        double v1, v2, v3, v4; 
        v1 = joint_states_[FRONT_RIGHT_W];
        v2 = joint_states_[FRONT_LEFT_W];
        v3 = joint_states_[BACK_LEFT_W];
        v4 = joint_states_[BACK_RIGHT_W];
        // Angular pos of each wheel
        double a1, a2, a3, a4; 
        a1 = joint_states_[FRONT_RIGHT_MW];  
        a2 = joint_states_[FRONT_LEFT_MW];
        a3 = joint_states_[BACK_LEFT_MW];
        a4 = joint_states_[BACK_RIGHT_MW];

        //    double v1x = -spin_ * v1 * cos( a1 ); double v1y = -spin_ * v1 * sin( a1 );  // spin for mirrored axes    
        //    double v2x = -v2 * cos( a2 ); double v2y = -v2 * sin( a2 );
        //    double v3x = -v3 * cos( a3 ); double v3y = -v3 * sin( a3 );
        //    double v4x = -spin_ * v4 * cos( a4 ); double v4y = -spin_ * v4 * sin( a4 );
        double v1x = -v1 * cos( a1 ); double v1y = -v1 * sin( a1 ); 
        double v2x = v2 * cos( a2 ); double v2y = v2 * sin( a2 );
        double v3x = v3 * cos( a3 ); double v3y = v3 * sin( a3 );
        double v4x = -v4 * cos( a4 ); double v4y = -v4 * sin( a4 );

        double C = (v4y + v1y) / 2.0;
        double B = (v2x + v1x) / 2.0;
        double D = (v2y + v3y) / 2.0;
        double A = (v3x + v4x) / 2.0;
        double E = (v1y + v2y) / 2.0;
        double F = (v3y + v4y) / 2.0;
        double G = (v1x + v4x) / 2.0;
        double H = (v2x + v3x) / 2.0;
        
        double w = ((E-F)/wheel_base_ + (G-H)/track_width_) / 2.0;
        double vx = (A+B) / 2.0;
        double vy = (C+D) / 2.0; 

        // Get real freq.
        ros::Time current_time = ros::Time::now();
        double seconds_since_last_update = (current_time - odom_last_update_ ).toSec();
        odom_last_update_ = current_time;

        // Compute Position
        double fSamplePeriod = seconds_since_last_update;
        pose_encoder_.x += cos(pose_encoder_.theta) * vx * fSamplePeriod + cos(M_PI_2 + pose_encoder_.theta) * vy * fSamplePeriod;
        pose_encoder_.y += sin(pose_encoder_.theta) * vx * fSamplePeriod + sin(M_PI_2 + pose_encoder_.theta) * vy * fSamplePeriod;
        pose_encoder_.theta += w * fSamplePeriod;  
        // ROS_INFO("Odom estimated x=%5.2f  y=%5.2f a=%5.2f", robot_pose_px_, robot_pose_py_, robot_pose_pa_);

        tf::Quaternion qt;
        tf::Vector3 vt;
        qt.setRPY(0,0,pose_encoder_.theta);
        vt = tf::Vector3 (pose_encoder_.x, pose_encoder_.y, 0);

        odom_.header.stamp = ros::Time::now();
        odom_.header.frame_id = odom_frame_;
        odom_.child_frame_id = robot_base_frame_;

        odom_.pose.pose.position.x = vt.x();
        odom_.pose.pose.position.y = vt.y();
        odom_.pose.pose.position.z = vt.z();

        odom_.pose.pose.orientation.x = qt.x();
        odom_.pose.pose.orientation.y = qt.y();
        odom_.pose.pose.orientation.z = qt.z();
        odom_.pose.pose.orientation.w = qt.w();

        odom_.twist.twist.linear.x = vx;
        odom_.twist.twist.linear.y = vy;    
        odom_.twist.twist.angular.z = w;
    }

    void OmniDriveController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
    {
        ROS_DEBUG_STREAM_NAMED(controller_name_, "Received command: (" << cmd_msg->linear.x << ")" << ", " << cmd_msg->linear.y << ", " << cmd_msg->angular.z << ")");
//        v_ref_x_ = cmd_msg->linear.x;
//        v_ref_y_ = cmd_msg->linear.y;
//        w_ref_ = cmd_msg->angular.z;
        cmd_ = *cmd_msg;
        cmd_last_stamp_ = ros::Time::now();
    }

    void OmniDriveController::publishOdometry()
    {
        odom_publisher_.publish(odom_); 

        tf::Quaternion qt = tf::Quaternion (odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w );
        tf::Vector3 vt = tf::Vector3 (odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z );

        tf::Transform base_footprint_to_odom(qt, vt);
        if (this->odom_broadcast_tf_) {
            transform_broadcaster_->sendTransform(
                    tf::StampedTransform(base_footprint_to_odom, ros::Time::now(),
                        odom_frame_, robot_base_frame_));
        }
    }

    //wheels can operate forward or backward, so each pair of angle+velocity has it's mirrored pair that results in the same movement
    //this function checks which one of both keeps the less change from the current configuration
    void OmniDriveController::setJointPositionReferenceWithLessChange(double &wheel_speed, double &wheel_angle, double current_wheel_speed, double current_wheel_angle)
    {
        double mirrored_wheel_speed = -wheel_speed;
        double mirrored_wheel_angle = (wheel_angle > 0) ? wheel_angle - M_PI : wheel_angle + M_PI;

        double change = std::abs(wheel_speed-current_wheel_speed)*std::abs(wheel_angle-current_wheel_angle);
        double mirrored_change = std::abs(mirrored_wheel_speed-current_wheel_speed)*std::abs(mirrored_wheel_angle-current_wheel_angle);

        if (mirrored_change < change)
        {
            wheel_speed = mirrored_wheel_speed;
            wheel_angle = mirrored_wheel_angle;
        }
    }

    //as the motorwheels can only rotate between their lower and upper limits, this function checks that the reference is between that limit
    //if it isn't, sets the mirrored pair
    void OmniDriveController::setJointPositionReferenceBetweenMotorWheelLimits(double &wheel_speed, double &wheel_angle, int joint_number)
    {
        double lower_limit = joint_limits_[joint_number].first;
        double upper_limit = joint_limits_[joint_number].second;

        // if angle is between limits, do nothing
        if (lower_limit <= wheel_angle && wheel_angle <= upper_limit)    
            return;

        // if angle is below the lower_limit, add pi and change speed sign
        if (wheel_angle < lower_limit) {
            wheel_angle += M_PI;
            wheel_speed = -wheel_speed;
            return;
            ROS_INFO_THROTTLE(1, "angle below limit");
        }
        // if angle is above the upper_limit, substract pi and change speed sign
        if (upper_limit < wheel_angle) {
            wheel_angle -= M_PI;
            wheel_speed = -wheel_speed;
            ROS_INFO_THROTTLE(1, "angle above limit");
            return;
        }
    }

    void OmniDriveController::setJointVelocityReferenceBetweenLimits(std::vector<double> &wheel_speed)
    {
        double max_scale_factor = 1.0;

        for (size_t i = BEGIN_W; i < END_W; i++) {
            double lower_limit = joint_limits_[i].first;
            double upper_limit = joint_limits_[i].second;

            double lower_scale_factor, upper_scale_factor;
            lower_scale_factor = upper_scale_factor = 1.0;

            if (wheel_speed[i] < lower_limit)
                lower_scale_factor = std::abs(wheel_speed[i]/lower_limit);
            if (upper_limit < wheel_speed[i])
                upper_scale_factor = std::abs(wheel_speed[i]/upper_limit);

            max_scale_factor = std::max(max_scale_factor, std::max(lower_scale_factor, upper_scale_factor));
        }

//        std::ostringstream oss;
//        oss << "scale_factor: " << max_scale_factor;
//        for (size_t i = BEGIN_W; i < END_W; i++) {
//            oss << " wheel " << i << " : " << wheel_speed[i] << " (" << wheel_speed[i] / max_scale_factor << ")";
//        }
//        
//        ROS_INFO_STREAM_THROTTLE(1, oss.str());

        for (size_t i = BEGIN_W; i < END_W; i++) {
            wheel_speed[i] /= max_scale_factor;
        }
        
    }
}
