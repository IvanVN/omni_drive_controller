#include <algorithm>
#include <sstream>
#include <numeric>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <omni_drive_controller/omni_drive_controller.h>
#include <std_msgs/Float64MultiArray.h>

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

     bool OmniDriveController::initRequest(hardware_interface::RobotHW *const robot_hw,
            ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh,
             ClaimedResources &claimed_resources)  /*  Changes here too like in the .h file */
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
	hardware_interface::internal::demangledTypeName<hardware_interface::PositionJointInterface>().c_str(); //I added this line
            return false;
        }

        hardware_interface::PositionJointInterface * pos_hw = robot_hw->get<hardware_interface::PositionJointInterface>();
        if (!pos_hw)
        {
            ROS_ERROR_STREAM_NAMED(controller_name_, "This controller requires a hardware interface of type 'hardware_interface::PositionJointInterface'."
                    " Make sure this is registered in the hardware_interface::RobotHW class.");
	hardware_interface::internal::demangledTypeName<hardware_interface::PositionJointInterface>().c_str();  //I added that line
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


	//The main changes are here


        hardware_interface::InterfaceResources vel_claims(hardware_interface::internal::demangledTypeName<hardware_interface::VelocityJointInterface>(), vel_hw->getClaims());

        hardware_interface::InterfaceResources pos_claims(hardware_interface::internal::demangledTypeName<hardware_interface::PositionJointInterface>(), pos_hw->getClaims());
      
        claimed_resources.push_back(vel_claims);
        claimed_resources.push_back(pos_claims);
	
	vel_hw->clearClaims();
	pos_hw->clearClaims();
	
/*

Originally it was that 

        std::set<std::string>  vel_claims = vel_hw->getClaims();
        std::set<std::string> pos_claims = pos_hw->getClaims();

        claimed_resources.insert(vel_claims.begin(), vel_claims.end());
       claimed_resources.insert(pos_claims.begin(), pos_claims.end());

        vel_hw->clearClaims();
        pos_hw->clearClaims(); 

 */


        // success
        state_ = INITIALIZED;
        return true;
    } 


    bool OmniDriveController::initVelocityInterface(hardware_interface::VelocityJointInterface* hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle &controller_nh)
    {
        joints_[FRONT_RIGHT_TRACTION_JOINT] = hw->getHandle(joint_names_[FRONT_RIGHT_TRACTION_JOINT]);
        joints_[FRONT_LEFT_TRACTION_JOINT] = hw->getHandle(joint_names_[FRONT_LEFT_TRACTION_JOINT]);
        joints_[BACK_RIGHT_TRACTION_JOINT] = hw->getHandle(joint_names_[BACK_RIGHT_TRACTION_JOINT]);
        joints_[BACK_LEFT_TRACTION_JOINT] = hw->getHandle(joint_names_[BACK_LEFT_TRACTION_JOINT]);

        return true;
    }

    bool OmniDriveController::initPositionInterface(hardware_interface::PositionJointInterface* hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle &controller_nh)
    {
        joints_[FRONT_RIGHT_DIRECTION_JOINT] = hw->getHandle(joint_names_[FRONT_RIGHT_DIRECTION_JOINT]);
        joints_[FRONT_LEFT_DIRECTION_JOINT] = hw->getHandle(joint_names_[FRONT_LEFT_DIRECTION_JOINT]);
        joints_[BACK_RIGHT_DIRECTION_JOINT] = hw->getHandle(joint_names_[BACK_RIGHT_DIRECTION_JOINT]);
        joints_[BACK_LEFT_DIRECTION_JOINT] = hw->getHandle(joint_names_[BACK_LEFT_DIRECTION_JOINT]);

        return true;
    }

    bool OmniDriveController::initController(ros::NodeHandle root_nh, ros::NodeHandle controller_nh)
    {
        bool everything_ok = true;


        controller_name_ = "omni_drive_controller";


        // the topics are hardcoded, the way to change them is by using the remap option
        command_topic_ = "cmd_vel";
        odom_topic_ = "odom";

        // default values for some variables that can change using the param server
        // related to kinematics
        wheel_base_ = 0.934;
        track_width_ = 0.57;
        wheel_diameter_ = 0.186;

        linear_speed_limit_ = 0.1;
        linear_acceleration_limit_ = 0.1;
        angular_speed_limit_ = 0.1;
        angular_acceleration_limit_ = 0.1;

        controller_nh.param("wheel_base", wheel_base_, wheel_base_);
        controller_nh.param("track_width", track_width_, track_width_);
        controller_nh.param("wheel_diameter", wheel_diameter_, wheel_diameter_); 
       

        if (controller_nh.hasParam("linear_speed_limit") == false) { // limit does not exist!
            ROS_WARN_STREAM_NAMED(controller_name_, controller_name_ << "::initController: Cannot find parameter linear_speed_limit parameter. I will gently set it to " << linear_speed_limit_);
        }
        else {
            controller_nh.param("linear_speed_limit", linear_speed_limit_, linear_speed_limit_);
            if (linear_speed_limit_ < 0) {
                ROS_WARN_STREAM_NAMED(controller_name_, controller_name_ << "::initController: Watch out! You set linear_speed_limit, which is the limit of the modulo of the linear speed, to a negative value. I will gently set it as positive.");
                linear_speed_limit_ = -linear_speed_limit_;
            }
        }
        
        if (controller_nh.hasParam("linear_acceleration_limit") == false) { // limit does not exist!
            ROS_WARN_STREAM_NAMED(controller_name_, controller_name_ << "::initController: Cannot find parameter linear_acceleration_limit parameter. I will gently set it to " << linear_acceleration_limit_);
        }
        else {
            controller_nh.param("linear_acceleration_limit", linear_acceleration_limit_, linear_acceleration_limit_);
            if (linear_acceleration_limit_ < 0) {
                ROS_WARN_STREAM_NAMED(controller_name_, controller_name_ << "::initController: Watch out! You set linear_acceleration_limit, which is the limit of the modulo of the linear acceleration, to a negative value. I will gently set it as positive.");
                linear_acceleration_limit_ = -linear_acceleration_limit_;
            }
        }
        
        if (controller_nh.hasParam("angular_speed_limit") == false) { // limit does not exist!
            ROS_WARN_STREAM_NAMED(controller_name_, controller_name_ << "::initController: Cannot find parameter angular_speed_limit parameter. I will gently set it to " << angular_speed_limit_);
        }
        else {
            controller_nh.param("angular_speed_limit", angular_speed_limit_, angular_speed_limit_);
            if (angular_speed_limit_ < 0) {
                ROS_WARN_STREAM_NAMED(controller_name_, controller_name_ << "::initController: Watch out! You set angular_speed_limit, which is the limit of the modulo of the angular speed, to a negative value. I will gently set it as positive.");
                angular_speed_limit_ = -angular_speed_limit_;
            }
        }
        
        if (controller_nh.hasParam("angular_acceleration_limit") == false) { // limit does not exist!
            ROS_WARN_STREAM_NAMED(controller_name_, controller_name_ << "::initController: Cannot find parameter angular_acceleration_limit parameter. I will gently set it to " << angular_acceleration_limit_);
        }
        else {
            controller_nh.param("angular_acceleration_limit", angular_acceleration_limit_, angular_acceleration_limit_);
            if (angular_acceleration_limit_ < 0) {
                ROS_WARN_STREAM_NAMED(controller_name_, controller_name_ << "::initController: Watch out! You set angular_acceleration_limit, which is the limit of the modulo of the angular acceleration, to a negative value. I will gently set it as positive.");
                angular_acceleration_limit_ = -angular_acceleration_limit_;
            }
        }

        // related to coordinate frames
        odom_frame_ = "odom";
        robot_base_frame_ = "base_footprint";
        odom_broadcast_tf_ = true;
        controller_nh.param("odom_frame", odom_frame_, odom_frame_);
        controller_nh.param("robot_base_frame", robot_base_frame_, robot_base_frame_);
        controller_nh.param("odom_broadcast_tf", odom_broadcast_tf_, odom_broadcast_tf_);
        // related to timing
        double cmd_watchdog = 0.3;
        double odom_frequency = 100;
        controller_nh.param("cmd_watchdog_duration", cmd_watchdog, cmd_watchdog);
        controller_nh.param("odom_publish_frequency", odom_frequency, odom_frequency);

        // begin to register ros stuff
        cmd_watchdog_duration_ = ros::Duration(cmd_watchdog);
        odom_publish_period_ = ros::Duration(1.0/odom_frequency);

        cmd_vel_subscriber_ = root_nh.subscribe(command_topic_, 1, &OmniDriveController::cmdVelCallback, this); 
        odom_publisher_ = root_nh.advertise<nav_msgs::Odometry>(odom_topic_, 1);
        transform_broadcaster_ = new tf::TransformBroadcaster();

        commands_pub_ = root_nh.advertise<std_msgs::Float64MultiArray>("commands", 1);

        joints_.resize(NUMBER_OF_JOINTS);

        joint_states_.resize(NUMBER_OF_JOINTS);
        joint_states_mean_.resize(NUMBER_OF_JOINTS);
        joint_references_.resize(NUMBER_OF_JOINTS);
        joint_commands_.resize(NUMBER_OF_JOINTS);

        // for now, the controller is for a robot with four wheel, and the joint names are these and only these
        joint_names_.resize(NUMBER_OF_JOINTS);
        joint_names_[FRONT_RIGHT_TRACTION_JOINT] = "front_right_wheel_joint";
        joint_names_[FRONT_LEFT_TRACTION_JOINT] = "front_left_wheel_joint";
        joint_names_[BACK_RIGHT_TRACTION_JOINT] = "back_right_wheel_joint";
        joint_names_[BACK_LEFT_TRACTION_JOINT] = "back_left_wheel_joint";
        joint_names_[FRONT_RIGHT_DIRECTION_JOINT] = "front_right_motor_wheel_joint";
        joint_names_[FRONT_LEFT_DIRECTION_JOINT] = "front_left_motor_wheel_joint";
        joint_names_[BACK_RIGHT_DIRECTION_JOINT] = "back_right_motor_wheel_joint";
        joint_names_[BACK_LEFT_DIRECTION_JOINT] = "back_left_motor_wheel_joint";
        
        joint_limits_.resize(NUMBER_OF_JOINTS);

        joint_states_history_size_ = 1;
        joint_states_history_.resize(NUMBER_OF_JOINTS);
        for (size_t i = 0; i < NUMBER_OF_JOINTS; i++) {
            joint_states_history_[i] = boost::circular_buffer<double>(joint_states_history_size_);
        }
        
        // set velocity limits
        for (size_t i = BEGIN_TRACTION_JOINT; i < END_TRACTION_JOINT; i++) {
            std::string param_limit_name;
            double max_speed = 0;

            param_limit_name = "joint/" + joint_names_[i] + "/max_speed";
            if (controller_nh.hasParam(param_limit_name) == false) { // limit does not exist!
                ROS_ERROR_STREAM_NAMED(controller_name_, controller_name_ << "::initController: cannot find parameter " << param_limit_name << ". It is required!");
                everything_ok = false;
            }
            else {
                controller_nh.param(param_limit_name, max_speed, max_speed);
            }
            
            joint_limits_[i] = std::make_pair(-max_speed, max_speed);
        }

        // set direction limits
        for (size_t i = BEGIN_DIRECTION_JOINT; i < END_DIRECTION_JOINT; i++) {
            std::string param_limit_name;
            double min_angle = 0, max_angle = 0;

            param_limit_name = "joint/" + joint_names_[i] + "/min_angle";
            if (controller_nh.hasParam(param_limit_name) == false) { // limit does not exist!
                ROS_ERROR_STREAM_NAMED(controller_name_, controller_name_ << "::initController: cannot find parameter " << param_limit_name << ". It is required!");
                everything_ok = false;
            }
            else {
                controller_nh.param(param_limit_name, min_angle, min_angle);
            }

            param_limit_name = "joint/" + joint_names_[i] + "/max_angle";
            if (controller_nh.hasParam(param_limit_name) == false) { // limit does not exist!
                ROS_ERROR_STREAM_NAMED(controller_name_, controller_name_ << "::initController: cannot find parameter " << param_limit_name << ". It is required!");
                everything_ok = false;
            }
            else {
                controller_nh.param(param_limit_name, max_angle, max_angle);
            }

            joint_limits_[i] = std::make_pair(min_angle, max_angle);
        }
        
        if (everything_ok) {
            ROS_INFO_STREAM_NAMED(controller_name_, controller_name_ << "::initController: everything is OK!");
        }
        
        return everything_ok;
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
        ROS_INFO_STREAM_NAMED(controller_name_, controller_name_ << ": Starting!");
        odom_last_sent_ = ros::Time::now();
        odom_last_update_ = ros::Time::now(); // check if there is some value that invalidates the result of a substraction (equals 0). if there isn't any, seta flag for first_update_odometry
        cmd_last_stamp_ = ros::Time(0); // maybe it is better to set it to 0, so if no cmd is received
   
        // TODO: service to restart odometry?
        odom_ = nav_msgs::Odometry();
        received_cmd_ = geometry_msgs::Twist();
        pose_encoder_ = geometry_msgs::Pose2D();

        cmd_watchdog_timedout_ = true;
        controller_state_ = ControllerState::Init;
        robot_is_stopped_ = true;
    }

    /**
     * \brief Stops controller
     * \param time Current time
     */
    void OmniDriveController::stopping(const ros::Time& time)
    {
        ROS_INFO_STREAM_NAMED(controller_name_, controller_name_ << "Stopping!");
    }


    void OmniDriveController::update(const ros::Time& time, const ros::Duration& period)
    {
        static int iter = 0;
        iter++;
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
        
        if ((time - odom_last_sent_) > odom_publish_period_) {
            odom_last_sent_ = time;
            publishOdometry();
        }
        
        cmd_watchdog_timedout_ = ((time - cmd_last_stamp_) > cmd_watchdog_duration_);

        if (cmd_watchdog_timedout_) {
            current_cmd_ = geometry_msgs::Twist(); // to set to 0
            hardRobotBrake();
            controller_state_ = ControllerState::Braking;
            //if (timeouts == 1)
                //ROS_INFO( "WATCH (to init) %d", iter);
            return;
        }

        if (received_cmd_.linear.x == 0 and received_cmd_.linear.y == 0 and received_cmd_.angular.z == 0) {
            controller_state_ = ControllerState::Braking;
        }

        switch (controller_state_)
        {
            case ControllerState::Init:
            {
                //current_cmd_ = geometry_msgs::Twist();
                //limitCommand(period.toSec(), received_cmd_);
                current_cmd_ = received_cmd_;

                // calculate joint velocity and position references, taking into account some constrains
                updateJointReferences();
                for (size_t i = BEGIN_DIRECTION_JOINT; i < END_DIRECTION_JOINT; i++) {
                    joint_commands_[i] = joint_references_[i];
                    joints_[i].setCommand(joint_commands_[i]);
                }
                double range = 0.05;
                if (areDirectionWheelsOriented(range) == true){
                    controller_state_ = ControllerState::Moving;
                    //ROS_INFO("SWITCH TO MOV %d", iter);
                    current_cmd_ = geometry_msgs::Twist(); // to set to 0
                    break;
                }
                //orientWheels();
                //writeJointCommands();

            break;
            }
            case ControllerState::Moving:
            {    
                limitCommand(period.toSec(), received_cmd_);

                // calculate joint velocity and position references, taking into account some constrains
                updateJointReferences();

                double range2 = 1;
                if ((areDirectionWheelsOriented(range2) == false /*or areTractionWheelsOnSameDirection(3) == false*/) and 
                        std::abs(odom_.twist.twist.linear.x) > 0.05 and
                        std::abs(odom_.twist.twist.linear.y) > 0.05)
                {
                    controller_state_ = ControllerState::Braking;
                    //ROS_INFO("SWITCH TO BRAK %d", iter);
                    break;
                }
                setJointCommandsAsReferences();
                writeJointCommands();
            break;
            }
            case ControllerState::Braking:
            {
                softRobotBrake(period.toSec());
                
                if (robot_is_stopped_) {
                    controller_state_ = ControllerState::Init;
                    current_cmd_ = geometry_msgs::Twist(); // to set to 0
                    //ROS_INFO("SWITCH TO INIT %d", iter);
                }
            break;
            }
        }
    }

    bool OmniDriveController::areDirectionWheelsOriented(double max_range)
    {
        for (size_t i = BEGIN_DIRECTION_JOINT; i < END_DIRECTION_JOINT; i++) {
            if (std::abs(joint_commands_[i] - joint_states_[i]) > max_range)
                return false;
        }
        return true;
    }
    bool OmniDriveController::areTractionWheelsOnSameDirection(double max_range)
    {
        for (size_t i = BEGIN_TRACTION_JOINT; i < END_TRACTION_JOINT; i++) {
            if (std::abs(joint_commands_[i] - joint_states_[i]) > max_range and sign(joint_commands_[i]) != sign(joint_states_[i]))
                return false;
        }
        return true;
    }
    
    void OmniDriveController::hardRobotBrake() {
        // set references to 0
        for (size_t i = 0; i < NUMBER_OF_JOINTS; i++) {
            joint_commands_[i] = joint_references_[i] = 0;
        }
        //but only send speed commands
        for (size_t i = BEGIN_TRACTION_JOINT; i < END_TRACTION_JOINT; i++) {
            joints_[i].setCommand(joint_commands_[i]);
        }
        return;
    }
    
    void OmniDriveController::softRobotBrake(double period)
    {
        //keep direction references, but slowly stop traction
        double max_acceleration = 40; // rad/s^2
        for (size_t i = BEGIN_TRACTION_JOINT; i < END_TRACTION_JOINT; i++) {
            double v = joint_states_[i]*2/wheel_diameter_;
            //double v = joint_commands_[i];
            double accel = -v/period;
            if (std::abs(accel) > max_acceleration)
                accel = sign(accel) * max_acceleration;
            v = v + accel*period;
            joint_commands_[i] = v;
            joints_[i].setCommand(joint_commands_[i]);
        }
        for (size_t i = BEGIN_DIRECTION_JOINT; i < END_DIRECTION_JOINT; i++) {
         //   joint_commands_[i] = joint_states_[i];
        }
    }

    void OmniDriveController::orientWheels() {
        for (size_t i = BEGIN_TRACTION_JOINT; i < END_TRACTION_JOINT; i++) {
            joint_commands_[i] = 0;
        } 
        for (size_t i = BEGIN_DIRECTION_JOINT; i < END_DIRECTION_JOINT; i++) {
            joint_commands_[i] = joint_references_[i];
        }
    }

    void OmniDriveController::limitCommand(double period, geometry_msgs::Twist goal_cmd)
    {
        double vx, vy, w;

        double accel_x = (goal_cmd.linear.x - current_cmd_.linear.x)/period;
        double accel_y = (goal_cmd.linear.y - current_cmd_.linear.y)/period;

        double total_accel = std::sqrt(accel_x*accel_x + accel_y*accel_y);
        if (total_accel > linear_acceleration_limit_) {
            accel_x = accel_x * linear_acceleration_limit_ / total_accel;
            accel_y = accel_y * linear_acceleration_limit_ / total_accel;
        }

        vx = current_cmd_.linear.x + accel_x * period;
        vy = current_cmd_.linear.y + accel_y * period;

        double total_vel = std::sqrt(vx*vx + vy*vy);
        if (total_vel > linear_speed_limit_) {
            vx = vx * linear_speed_limit_ / total_vel;
            vy = vy * linear_speed_limit_ / total_vel;
        }

        current_cmd_.linear.x = vx;
        current_cmd_.linear.y = vy;

        double accel_w = (goal_cmd.angular.z - current_cmd_.angular.z)/period;
        if (std::abs(accel_w) > angular_acceleration_limit_)
            accel_w = sign(accel_w) * angular_acceleration_limit_;

        w = current_cmd_.angular.z + accel_w * period;
        if (std::abs(w) > angular_speed_limit_)
            w = sign(w) * angular_speed_limit_;

        current_cmd_.angular.z = w;

    }

    void OmniDriveController::readJointStates()
    {
        // read wheel velocity: convert from angular to linear
        for (size_t i = BEGIN_TRACTION_JOINT; i < END_TRACTION_JOINT; i++) {
            joint_states_[i]  = normToZero(joints_[i].getVelocity() * (wheel_diameter_/2.0));
        }

        // read motor wheel position
        for (size_t i = BEGIN_DIRECTION_JOINT; i < END_DIRECTION_JOINT; i++) {
            joint_states_[i] = radnorm(normToZero(joints_[i].getPosition()));
        }

        for (size_t i = 0; i < NUMBER_OF_JOINTS; i++) {
            joint_states_history_[i].push_back(joint_states_[i]);
            double sum = std::accumulate(joint_states_history_[i].begin(), joint_states_history_[i].end(), 0.0);
            joint_states_mean_[i] = sum / joint_states_history_[i].size();
        }
    }
    
    void OmniDriveController::setJointCommandsAsReferences()
    {
        for (size_t i = 0; i < NUMBER_OF_JOINTS; i++) {
            joint_commands_[i] = joint_references_[i];
        }
    }

    void OmniDriveController::writeJointCommands()
    {
        for (size_t i = 0; i < NUMBER_OF_JOINTS; i++) {
            joints_[i].setCommand(joint_commands_[i]);
        }
    }

    void OmniDriveController::updateJointReferences()
    {
        // Speed references for motor control	  
        double vx = current_cmd_.linear.x;
        double vy = current_cmd_.linear.y;
        double w =  current_cmd_.angular.z;

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

        // q = rad/s
        // a = rad

        double x1 = L/2.0; double y1 = W/2.0;
        double wx1 = vx + w * y1;
        double wy1 = vy + w * x1;
        q[0] = - sign(wx1) * sqrt( wx1*wx1 + wy1*wy1 ); // m/s
        q[0] = q[0] / (wheel_diameter_/2.0); // convert to rad/s
        //double a[0] = radnorm( atan2( wy1, wx1 ) );
        a[0] = radnormHalf( atan2( wy1,wx1 )); // contraint (3)
        
        double x2 = L/2.0; double y2 = W/2.0;
        double wx2 = vx - w * y2;
        double wy2 = vy + w * x2;
        q[1] = sign(wx2) * sqrt( wx2*wx2 + wy2*wy2 ); // m/s
        q[1] = q[1] / (wheel_diameter_/2.0); // convert to rad/s
        //double a[1] = radnorm( atan2( wy2, wx2 ) );
        a[1] = radnormHalf( atan2 (wy2,wx2)); // contraint (3)
        
        double x3 = L/2.0; double y3 = W/2.0;
        double wx3 = vx - w * y3;
        double wy3 = vy - w * x3;
        q[2] = sign(wx3)*sqrt( wx3*wx3 + wy3*wy3 ); // m/s
        q[2] = q[2] / (wheel_diameter_/2.0); // convert to rad/s
        //double a[2] = radnorm( atan2( wy3, wx3 ) );
        a[2] = radnormHalf( atan2(wy3 , wx3)); // contraint (3)
        
        double x4 = L/2.0; double y4 = W/2.0;
        double wx4 = vx + w * y4;
        double wy4 = vy - w * x4;
        q[3] = -sign(wx4)*sqrt( wx4*wx4 + wy4*wy4 ); // m/s
        q[3] = q[3] / (wheel_diameter_/2.0); // convert to rad/s
        //double a[3] = radnorm( atan2( wy4, wx4 ) );
        a[3] = radnormHalf( atan2(wy4,wx4)); // contraint (3)
	 
        //constraint (2)
        setJointPositionReferenceWithLessChange(q[0], a[0], joint_states_mean_[FRONT_RIGHT_TRACTION_JOINT], joint_references_[FRONT_RIGHT_DIRECTION_JOINT]);
        setJointPositionReferenceWithLessChange(q[1], a[1], joint_states_mean_[FRONT_LEFT_TRACTION_JOINT],  joint_references_[FRONT_LEFT_DIRECTION_JOINT]);
        setJointPositionReferenceWithLessChange(q[2], a[2], joint_states_mean_[BACK_LEFT_TRACTION_JOINT],   joint_references_[BACK_LEFT_DIRECTION_JOINT]);
        setJointPositionReferenceWithLessChange(q[3], a[3], joint_states_mean_[BACK_RIGHT_TRACTION_JOINT],  joint_references_[BACK_RIGHT_DIRECTION_JOINT]);

        bool all_references_are_between_limits = true;
        double range_limit = 0.01; // if we are moving, only check for the limit, with a small epsilon
        if (controller_state_ == ControllerState::Init)
            range_limit = 0.26; // if we are initializing the movement, it is better to set the reference away from the limit, so if it is close to the limit, it will change the configuration

        all_references_are_between_limits &= checkJointPositionReferenceIsBetweenMotorWheelLimits(a[0], range_limit, FRONT_RIGHT_DIRECTION_JOINT);
        all_references_are_between_limits &= checkJointPositionReferenceIsBetweenMotorWheelLimits(a[1], range_limit, FRONT_LEFT_DIRECTION_JOINT);
        all_references_are_between_limits &= checkJointPositionReferenceIsBetweenMotorWheelLimits(a[2], range_limit, BACK_LEFT_DIRECTION_JOINT);
        all_references_are_between_limits &= checkJointPositionReferenceIsBetweenMotorWheelLimits(a[3], range_limit, BACK_RIGHT_DIRECTION_JOINT);
        
        if (all_references_are_between_limits == false) { 
            // we have to change the configuration of the robot! at least one wheel exceeds it's limit,
            // but let's also change other wheels that are near it's limit
            range_limit = 0.26; // rads ~ 15 degrees
            if (checkJointPositionReferenceIsBetweenMotorWheelLimits(a[0], range_limit, FRONT_RIGHT_DIRECTION_JOINT) == false)
                setJointConfigurationAsMirror(q[0], a[0]);
            if (checkJointPositionReferenceIsBetweenMotorWheelLimits(a[1], range_limit, FRONT_LEFT_DIRECTION_JOINT) == false)
                setJointConfigurationAsMirror(q[1], a[1]);
            if (checkJointPositionReferenceIsBetweenMotorWheelLimits(a[2], range_limit, BACK_LEFT_DIRECTION_JOINT) == false)
                setJointConfigurationAsMirror(q[2], a[2]);
            if (checkJointPositionReferenceIsBetweenMotorWheelLimits(a[3], range_limit, BACK_RIGHT_DIRECTION_JOINT) == false)
                setJointConfigurationAsMirror(q[3], a[3]);
        }
        //constraint (1)
        setJointPositionReferenceBetweenMotorWheelLimits(q[0], a[0], FRONT_RIGHT_DIRECTION_JOINT);
        setJointPositionReferenceBetweenMotorWheelLimits(q[1], a[1], FRONT_LEFT_DIRECTION_JOINT);
        setJointPositionReferenceBetweenMotorWheelLimits(q[2], a[2], BACK_LEFT_DIRECTION_JOINT);
        setJointPositionReferenceBetweenMotorWheelLimits(q[3], a[3], BACK_RIGHT_DIRECTION_JOINT);

        // joint velocity references are scaled so each wheel does not exceed it's maximum velocity
        setJointVelocityReferenceBetweenLimits(q);

        // Motor control actions	  
        // Axis are not reversed in the omni (swerve) configuration
        joint_references_[FRONT_RIGHT_TRACTION_JOINT] = q[0];  
        joint_references_[FRONT_LEFT_TRACTION_JOINT] = q[1];
        joint_references_[BACK_LEFT_TRACTION_JOINT] = q[2];
        joint_references_[BACK_RIGHT_TRACTION_JOINT] = q[3];

        joint_references_[FRONT_RIGHT_DIRECTION_JOINT] = a[0];  
        joint_references_[FRONT_LEFT_DIRECTION_JOINT] = a[1];
        joint_references_[BACK_LEFT_DIRECTION_JOINT] = a[2];
        joint_references_[BACK_RIGHT_DIRECTION_JOINT] = a[3];
        
    }


    void OmniDriveController::updateOdometryFromEncoder()
    {
        // Linear speed of each wheel
        double v1, v2, v3, v4; 
        v1 = joint_states_[FRONT_RIGHT_TRACTION_JOINT];
        v2 = joint_states_[FRONT_LEFT_TRACTION_JOINT];
        v3 = joint_states_[BACK_LEFT_TRACTION_JOINT];
        v4 = joint_states_[BACK_RIGHT_TRACTION_JOINT];
        // Angular pos of each wheel
        double a1, a2, a3, a4; 
        a1 = joint_states_[FRONT_RIGHT_DIRECTION_JOINT];  
        a2 = joint_states_[FRONT_LEFT_DIRECTION_JOINT];
        a3 = joint_states_[BACK_LEFT_DIRECTION_JOINT];
        a4 = joint_states_[BACK_RIGHT_DIRECTION_JOINT];

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
        
        if ( std::abs(odom_.twist.twist.linear.x) < 0.01 and
             std::abs(odom_.twist.twist.linear.y) < 0.01 and 
             std::abs(odom_.twist.twist.angular.z) < 0.01)
                robot_is_stopped_ = true;
        else
            robot_is_stopped_ = false;
    }

    void OmniDriveController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
    {
        ROS_DEBUG_STREAM_NAMED(controller_name_, "Received command: (" << cmd_msg->linear.x << ")" << ", " << cmd_msg->linear.y << ", " << cmd_msg->angular.z << ")");
        received_cmd_ = *cmd_msg;

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
    //if it isn't, sets the mirrored pair, which has the angle rotated half a turn and the speed is the negative speed
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
            ROS_DEBUG_STREAM_THROTTLE_NAMED(1.0, controller_name_, controller_name_ << ": desired angle is below the limit. Turning!");
        }
        // if angle is above the upper_limit, substract pi and change speed sign
        if (upper_limit < wheel_angle) {
            wheel_angle -= M_PI;
            wheel_speed = -wheel_speed;
            ROS_DEBUG_STREAM_THROTTLE_NAMED(1.0, controller_name_, controller_name_ << ": desired angle is above the limit. Turning!");
            return;
        }
    }
    
    void OmniDriveController::setJointConfigurationAsMirror(double &wheel_speed, double &wheel_angle)
    {
        // if angle is negative, add pi and change speed sign
        if (wheel_angle < 0) {
            wheel_angle += M_PI;
            wheel_speed = -wheel_speed;
            return;
        }
        // if angle is positive, substract pi and change speed sign
        if (0 < wheel_angle) {
            wheel_angle -= M_PI;
            wheel_speed = -wheel_speed;
            return;
        }
    }
    
    bool OmniDriveController::checkJointPositionReferenceIsBetweenMotorWheelLimits(double wheel_angle, double range, int joint_number)
    {
        double lower_limit = joint_limits_[joint_number].first;
        double upper_limit = joint_limits_[joint_number].second;

        // returns true if it is between the [limits +- range]
        if ((lower_limit + range) <= wheel_angle || wheel_angle <= (upper_limit-range))
            return true;

        return false;
    }

    void OmniDriveController::setJointVelocityReferenceBetweenLimits(std::vector<double> &wheel_speed)
    {
        double max_scale_factor = 1.0;

        for (size_t i = BEGIN_TRACTION_JOINT; i < END_TRACTION_JOINT; i++) {
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

        for (size_t i = BEGIN_TRACTION_JOINT; i < END_TRACTION_JOINT; i++) {
            wheel_speed[i] /= max_scale_factor;
        }
        
    }
}
