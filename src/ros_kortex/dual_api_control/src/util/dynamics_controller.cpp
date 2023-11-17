/*
Author(s): Djordje Vukcevic, Sven Schneider
Institute: Hochschule Bonn-Rhein-Sieg

Copyright (c) [2019]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <dynamics_controller.hpp>
#define SECOND 1000000 // 1sec = 1 000 000 us
const double MIN_NORM = 1e-3;

dynamics_controller::dynamics_controller(robot_mediator *robot_driver,
                                         const int rate_hz,
                                         const bool compensate_gravity):
    RATE_HZ_(rate_hz),
    DT_MICRO_(SECOND / RATE_HZ_), DT_1KHZ_MICRO_(SECOND / 1000), // Time period defined in microseconds: 1s = 1 000 000us
    DT_SEC_(1.0 / static_cast<double>(RATE_HZ_)),
    DT_STOPPING_MICRO_(SECOND / dynamics_parameter::STOPPING_MOTION_LOOP_FREQ),
    store_control_data_(false), desired_dynamics_interface_(dynamics_interface::CART_ACCELERATION), 
    desired_task_model_(task_model::full_pose),
    loop_start_time_(std::chrono::steady_clock::now()),
    total_time_sec_(0.0), loop_iteration_count_(0), stop_loop_iteration_count_(0),
    steady_stop_iteration_count_(0), feedforward_loop_count_(0), control_loop_delay_count_(0),
    robot_chain_(robot_driver->get_robot_model()),
    NUM_OF_JOINTS_(robot_chain_.getNrOfJoints()),
    NUM_OF_SEGMENTS_(robot_chain_.getNrOfSegments()),
    NUM_OF_FRAMES_(robot_chain_.getNrOfSegments() + 1),
    NUM_OF_CONSTRAINTS_(dynamics_parameter::NUMBER_OF_CONSTRAINTS),
    END_EFF_(NUM_OF_SEGMENTS_ - 1), ROBOT_ID_(robot_driver->get_robot_ID()),
    INITIAL_END_EFF_MASS_(robot_chain_.getSegment(END_EFF_).getInertia().getMass()),
    COMPENSATE_GRAVITY_(compensate_gravity),
    CTRL_DIM_(NUM_OF_CONSTRAINTS_, false), POS_TUBE_DIM_(NUM_OF_CONSTRAINTS_, false),
    MOTION_CTRL_DIM_(NUM_OF_CONSTRAINTS_, false), FORCE_CTRL_DIM_(NUM_OF_CONSTRAINTS_, false),
    stop_motion_setpoint_array_(0, std::deque<double>(0, 0.0)),
    fsm_result_(task_status::NOMINAL), fsm_force_task_result_(task_status::APPROACH),
    previous_task_status_(fsm_result_), tube_section_count_(0), 
    transform_drivers_(false), transform_force_drivers_(false),
    apply_feedforward_force_(false), compute_null_space_command_(false),
    write_contact_time_to_file_(false), compensate_unknown_weight_(false),
    trigger_stopping_sequence_(false), stopping_sequence_on_(false),
    JOINT_ACC_LIMITS_(robot_driver->get_joint_acceleration_limits()),
    JOINT_TORQUE_LIMITS_(robot_driver->get_joint_torque_limits()),
    JOINT_STOPPING_TORQUE_LIMITS_(robot_driver->get_joint_stopping_torque_limits()),
    JOINT_INERTIA_(robot_driver->get_joint_inertia()),
    ROOT_ACC_(robot_driver->get_root_acceleration()),
    current_error_twist_(KDL::Twist::Zero()),
    predicted_error_twist_(Eigen::VectorXd::Zero(NUM_OF_CONSTRAINTS_)),
    compensation_error_(Eigen::VectorXd::Zero(NUM_OF_CONSTRAINTS_)),
    horizon_amplitude_(1.0), 
    null_space_angle_(0.0), desired_null_space_angle_(0.0),
    max_command_(Eigen::VectorXd::Zero(NUM_OF_CONSTRAINTS_)),
    compensation_parameters_(Eigen::VectorXd::Constant(12, 0.0)),
    null_space_parameters_(Eigen::VectorXd::Constant(6, 0.1)),
    force_task_parameters_(Eigen::VectorXd::Zero(NUM_OF_CONSTRAINTS_)),
    min_sat_limits_(Eigen::VectorXd::Zero(NUM_OF_CONSTRAINTS_)),
    filtered_bias_(Eigen::VectorXd::Zero(NUM_OF_CONSTRAINTS_)),
    cart_force_command_(NUM_OF_SEGMENTS_, KDL::Wrench::Zero()), 
    zero_wrenches_(NUM_OF_SEGMENTS_, KDL::Wrench::Zero()),
    ext_wrench_(KDL::Wrench::Zero()), ext_wrench_base_(KDL::Wrench::Zero()),
    compensated_weight_(KDL::Wrench::Zero()),
    zero_joint_array_(NUM_OF_JOINTS_), gravity_torque_(NUM_OF_JOINTS_),
    fk_vereshchagin_(robot_chain_),
    safety_control_(robot_driver, true), 
    fsm_(NUM_OF_JOINTS_, NUM_OF_SEGMENTS_, NUM_OF_FRAMES_, NUM_OF_CONSTRAINTS_),
    predictor_(robot_chain_),
    robot_state_(NUM_OF_JOINTS_, NUM_OF_SEGMENTS_, NUM_OF_FRAMES_, NUM_OF_CONSTRAINTS_),
    robot_state_base_(robot_state_), desired_state_(robot_state_),
    desired_state_base_(robot_state_), predicted_state_(robot_state_),
    WRITE_FORMAT_STOP_MOTION(Eigen::IOFormat(6, Eigen::DontAlignCols, " ", "", "", "\n"))
{
    assert(("Robot is not initialized", robot_driver->is_initialized()));
    // KDL Vereshchagin-Solver constraint  
    assert(NUM_OF_JOINTS_ == NUM_OF_SEGMENTS_);

    // Control loop frequency must be higher than or equal to 1 Hz
    assert(("Selected frequency is too low", 1 <= RATE_HZ_));
    // Control loop frequency must be lower than or equal to 1000 Hz
    assert(("Selected frequency is too high", RATE_HZ_<= 1000));

    this->hd_solver_ = std::make_shared<KDL::Solver_Vereshchagin>(robot_chain_, JOINT_INERTIA_,
                                                                  JOINT_TORQUE_LIMITS_, !COMPENSATE_GRAVITY_,
                                                                  COMPENSATE_GRAVITY_? KDL::Twist::Zero() : ROOT_ACC_, 
                                                                  NUM_OF_CONSTRAINTS_);
    this->id_solver_ = std::make_shared<KDL::Solver_RNE>(robot_chain_, KDL::Vector(0.0, 0.0, -9.81289),
                                                         JOINT_INERTIA_, JOINT_TORQUE_LIMITS_, false);

    // Set default command interface to stop motion mode and initialize it as not safe
    desired_control_mode_.interface = control_mode::STOP_MOTION;
    desired_control_mode_.is_safe = false;

    error_logger_.error_source_ = error_source::empty;
    error_logger_.error_status_ = 0;

}

//Print information about controller settings
void dynamics_controller::print_settings_info()
{   
    #ifdef NDEBUG
        printf("The program is build in RELEASE mode.\n");
    #endif
    #ifndef NDEBUG
        printf("The program is build in DEBUG mode.\n");
    #endif
    
    printf("Selected controller settings:\n");
    printf("Control Loop Frequency: %d Hz", RATE_HZ_);

    printf("Joint Interface: ");
    switch(desired_control_mode_.interface) 
    {
        case control_mode::STOP_MOTION:
            printf("STOP JOINTS \n Stopping the robot!\n");
            break;

        case control_mode::POSITION:
            printf("Joint Position \n");
            break;

        case control_mode::VELOCITY:
            printf("Joint Velocity \n");
            break;

        case control_mode::TORQUE:
            printf("Joint Torque \n");
            break;
    }

    printf("Dynamics Interface: ");
    switch(desired_dynamics_interface_) 
    {
        case dynamics_interface::CART_ACCELERATION:
            printf("Cartesian EndEffector Acceleration Interface\n");
            break;

        case dynamics_interface::CART_FORCE:
            printf("Cartesian Force Interface\n");
            break;

        case dynamics_interface::FF_JOINT_TORQUE:
            printf("FeedForward Joint Torque Interface\n");
            break;

        default:
            printf("Stopping the robot!\n");
            break;
    }

    printf("Task Model: ");
    switch(desired_task_model_) 
    {
        case task_model::full_pose:
            printf("Control Full 6D Pose\n");
            break;

        case task_model::gravity_compensation:
            printf("Only compensate the gravity\n");
            break;

        case task_model::moveGuarded:
            printf("moveGuarded\n");
            break;

        case task_model::moveTo:
            printf("moveTo\n");
            break;

        case task_model::moveTo_weight_compensation:
            printf("moveTo_weight_compensation\n");
            break;

        case task_model::moveTo_follow_path:
            printf("moveTo_follow_path\n");
            break;

        case task_model::moveConstrained_follow_path:
            printf("moveConstrained_follow_path\n");
            break;

        default:
            printf("Stopping the robot!\n");
            break;
    }

    std::cout<< "\nInitial joint state: "<< std::endl;
    std::cout<< "Joint positions: "<< robot_state_.q << std::endl;
    std::cout<< "Joint velocities:"<< robot_state_.qd << "\n" << std::endl;

    std::cout<< "Initial Cartesian state: "<< std::endl;
    std::cout<< "End-effector position: "<< robot_state_.frame_pose[END_EFF_].p << std::endl;
    std::cout<< "End-effector orientation: \n"<< robot_state_.frame_pose[END_EFF_].M << std::endl;
    std::cout<< "End-effector velocity:"<< robot_state_.frame_velocity[END_EFF_] << "\n" << std::endl;
}


int dynamics_controller::check_fsm_status()
{
    switch (fsm_result_)
    {
        case task_status::NOMINAL:
            if (previous_task_status_ != task_status::NOMINAL) printf("Control status changed to NOMINAL\n");
            previous_task_status_ = fsm_result_;
            return 0;
            break;
        
        case task_status::START_TO_CRUISE:
            if (previous_task_status_ != task_status::START_TO_CRUISE) printf("Control status changed to START_TO_CRUISE\n");
            previous_task_status_ = fsm_result_;
            return 0;
            break;

        case task_status::CRUISE_TO_STOP:
            if (previous_task_status_ != task_status::CRUISE_TO_STOP) printf("Control status changed to CRUISE_TO_STOP\n");
            previous_task_status_ = fsm_result_;
            return 0;
            break;

        case task_status::CRUISE_THROUGH_TUBE:
            if ((previous_task_status_ != task_status::CRUISE_THROUGH_TUBE) && (previous_task_status_ != task_status::CHANGE_TUBE_SECTION)) printf("Control status changed to CRUISE_THROUGH_TUBE\n");
            previous_task_status_ = fsm_result_;
            return 0;
            break;
        
        case task_status::CRUISE:
            if (previous_task_status_ != task_status::CRUISE) printf("Control status changed to CRUISE\n");
            previous_task_status_ = fsm_result_;
            return 0;
            break;

        case task_status::CHANGE_TUBE_SECTION:
            // if (previous_task_status_ != task_status::CHANGE_TUBE_SECTION) printf("Control status changed to CHANGE_TUBE_SECTION\n");
            previous_task_status_ = fsm_result_;
            return 0;
            break;

        case task_status::APPROACH:
            if (previous_task_status_ != task_status::APPROACH) printf("Control status changed to APPROACH\n");
            previous_task_status_ = fsm_result_;
            return 0;
            break;

        case task_status::STOP_ROBOT:
            if (previous_task_status_ != task_status::STOP_ROBOT) printf("Control status changed to STOP_ROBOT\n");
            previous_task_status_ = fsm_result_;
            return 1;
            break;
        
        default:
            // printf("Stop control!\n");
            return -1;
            break;
    }
}

/* 
    If it is working on the real robot get sensor data from the driver 
    or if simulation is on, replace current state with 
    integrated joint velocities and positions.
*/
int dynamics_controller::update_current_state()
{
    // Get joint angles and velocities
    safety_control_.get_current_state(robot_state_);

    // Get Cart poses and velocities
    return fk_vereshchagin_.JntToCart(robot_state_.q, robot_state_.qd, robot_state_.frame_pose, robot_state_.frame_velocity);;
}

// Write control data to a file
void dynamics_controller::write_to_file()
{
    if (!stopping_sequence_on_)
    {
        // Write measured state
        for (int i = 0; i < 3; i++)
            log_file_cart_ << robot_state_.frame_pose[END_EFF_].p(i) << " ";
        for (int i = 3; i < 6; i++)
            log_file_cart_ << 0.0 << " ";
        log_file_cart_ << robot_state_.frame_velocity[END_EFF_](0) << " ";
        log_file_cart_ << robot_state_.frame_velocity[END_EFF_](5) << " ";
        for (int i = 2; i < 5; i++)
            log_file_cart_ << ext_wrench_(i) << " ";
        if (write_contact_time_to_file_) log_file_cart_ << loop_iteration_count_;
        else log_file_cart_ << 0.0;
        log_file_cart_ << std::endl;

        // Write desired state
        for (int i = 0; i < 3; i++)
            log_file_cart_ << desired_state_.frame_pose[END_EFF_].p(i) << " ";
        for (int i = 3; i < 6; i++)
            log_file_cart_ << 0.0 << " ";
        log_file_cart_ << desired_state_.frame_velocity[END_EFF_](0) << " ";
        log_file_cart_ << desired_state_.frame_velocity[END_EFF_](5) << " ";
        for (int i = 2; i < 5; i++)
            log_file_cart_ << desired_state_.external_force[END_EFF_](i) << " ";
        log_file_cart_ << std::endl;

        // Write control error
        log_file_cart_ << predicted_error_twist_(0) << " ";
   
        // Log data in the base frame
        if (desired_task_model_ != task_model::full_pose)
        {
            // Write measured state in base frame
            for (int i = 0; i < 3; i++)
                log_file_cart_base_ << robot_state_base_.frame_pose[END_EFF_].p(i) << " ";

            for (int i = 0; i < 6; i++)
                log_file_cart_base_ << ext_wrench_base_(i) << " ";

            if (write_contact_time_to_file_) log_file_cart_base_ << loop_iteration_count_ << " ";
            else log_file_cart_base_ << 0.0 << " ";

            if (compensate_unknown_weight_)
            {
                for (int i = 0; i < 3; i++)
                    log_file_cart_base_ << robot_state_.external_force[END_EFF_].force(i) << " ";
            }
            log_file_cart_base_ << std::endl;

            // Write desired state in base frame
            for (int i = 0; i < 3; i++)
                log_file_cart_base_ << desired_state_base_.frame_pose[END_EFF_].p(i) << " ";

            for (int i = 0; i < 6; i++)
                log_file_cart_base_ << desired_state_base_.external_force[END_EFF_](i) << " ";

            if (write_contact_time_to_file_) log_file_cart_base_ << loop_iteration_count_ << " ";
            else log_file_cart_base_ << 0.0 << " ";

            if (compensate_unknown_weight_)
            {
                for (int i = 0; i < 3; i++)
                    log_file_cart_base_ << 0.0 << " ";
            }
            log_file_cart_base_ << std::endl;
        }

        // Write joint space state
        log_file_joint_ << robot_state_.control_torque.data.transpose().format(dynamics_parameter::WRITE_FORMAT);

        // Write null-space control state
        log_file_null_space_ << RAD_TO_DEG(null_space_angle_) << " " << desired_null_space_angle_ << " "; // Measured and desired state
       
        if (write_contact_time_to_file_)
        {
            log_file_null_space_ << loop_iteration_count_;
            write_contact_time_to_file_ = false;
        }
        else log_file_null_space_ << 0.0;
        log_file_null_space_ << std::endl;
    }
    else
    {
        // Write joint space state
        log_file_joint_ << robot_state_.control_torque.data.transpose().format(dynamics_parameter::WRITE_FORMAT);

        // Write measured state
        for (int i = 0; i < NUM_OF_JOINTS_; i++)
            log_file_stop_motion_ << robot_state_.qd(i) << " ";
        log_file_stop_motion_ << std::endl;

        // Write desired state
        for (int i = 0; i < NUM_OF_JOINTS_; i++)
            log_file_stop_motion_ << desired_state_.qd(i) << " ";
        log_file_stop_motion_ << std::endl;
    }
}

// Set all values of desired state to 0 - public method
void dynamics_controller::reset_desired_state()
{
    reset_state(desired_state_);
}

// Set all values of selected state to 0 - Private method
void dynamics_controller::reset_state(state_specification &state)
{
    desired_state_.reset_values();
}

void dynamics_controller::define_moveConstrained_follow_path_task(
                                const std::vector<bool> &constraint_direction,
                                const std::vector< std::vector<double> > &tube_path_points,
                                const std::vector<double> &tube_tolerances,
                                const double tube_speed,
                                const double tube_force,
                                const double contact_threshold_linear,
                                const double contact_threshold_angular,
                                const double task_time_limit_sec,
                                const bool control_null_space,
                                const double desired_null_space_angle,
                                std::vector< std::vector<double> > &task_frame_poses)
{
    assert(constraint_direction.size() == NUM_OF_CONSTRAINTS_);
    assert(tube_tolerances.size()      == NUM_OF_CONSTRAINTS_ + 2);
    assert(task_frame_poses.size()     == tube_path_points.size() - 1);
    assert(tube_path_points[0].size()  == 3);
    assert(task_frame_poses[0].size()  == 12);

    moveConstrained_follow_path_task_.tf_poses   = std::vector<KDL::Frame>(tube_path_points.size() - 1);
    moveConstrained_follow_path_task_.goal_poses = moveConstrained_follow_path_task_.tf_poses;

    CTRL_DIM_           = constraint_direction;
    POS_TUBE_DIM_[0]    = CTRL_DIM_[0]; POS_TUBE_DIM_[1]    = CTRL_DIM_[1]; 
    MOTION_CTRL_DIM_[0] = CTRL_DIM_[0]; MOTION_CTRL_DIM_[1] = CTRL_DIM_[1]; MOTION_CTRL_DIM_[5] = CTRL_DIM_[5];
    FORCE_CTRL_DIM_[3]  = CTRL_DIM_[3]; FORCE_CTRL_DIM_[4]  = CTRL_DIM_[4];

    // X-Y-Z linear
    KDL::Vector x_world(1.0, 0.0, 0.0);
    std::vector<double> task_frame_pose(12, 0.0);
    KDL::Rotation task_orientation = KDL::Rotation::EulerZYZ(0.0, 0.0, -M_PI/4);
    for (int i = 0; (unsigned)i < tube_path_points.size() - 1; i++)
    {
        KDL::Vector tube_start_position = KDL::Vector(tube_path_points[i    ][0], tube_path_points[i    ][1], tube_path_points[i    ][2]);
        KDL::Vector tf_position         = KDL::Vector(tube_path_points[i + 1][0], tube_path_points[i + 1][1], tube_path_points[i + 1][2]);

        tube_start_position = task_orientation * tube_start_position;
        tf_position         = task_orientation * tf_position;

        KDL::Vector x_task  = tf_position - tube_start_position;
        x_task.Normalize();

        KDL::Vector cross_product = x_world * x_task;
        double cosine             = dot(x_world, x_task);
        double sine               = cross_product.Norm();
        double angle              = atan2(sine, cosine);

        task_frame_pose[0] = tf_position[0]; 
        task_frame_pose[1] = tf_position[1]; 
        task_frame_pose[2] = tf_position[2];

        KDL::Rotation tf_orientation;
        if(cosine < (-1 + 1e-6))
        {
            tf_orientation = KDL::Rotation::EulerZYZ(M_PI, 0.0, 0.0);
            task_frame_pose[3] = -1.0; task_frame_pose[4]  =  0.0; task_frame_pose[5]  = 0.0;
            task_frame_pose[6] =  0.0; task_frame_pose[7]  = -1.0; task_frame_pose[8]  = 0.0;
            task_frame_pose[9] =  0.0; task_frame_pose[10] =  0.0; task_frame_pose[11] = 1.0;
        }
        else if(sine < 1e-6)
        {
            tf_orientation = KDL::Rotation::Identity();
            task_frame_pose[3] = 1.0; task_frame_pose[4]  = 0.0; task_frame_pose[5]  = 0.0;
            task_frame_pose[6] = 0.0; task_frame_pose[7]  = 1.0; task_frame_pose[8]  = 0.0;
            task_frame_pose[9] = 0.0; task_frame_pose[10] = 0.0; task_frame_pose[11] = 1.0;
        }
        else
        {
            tf_orientation = geometry::exp_map_so3(cross_product / sine * angle);
            task_frame_pose[3] = tf_orientation.data[0]; task_frame_pose[4]  = tf_orientation.data[1]; task_frame_pose[5]  = tf_orientation.data[2];
            task_frame_pose[6] = tf_orientation.data[3]; task_frame_pose[7]  = tf_orientation.data[4]; task_frame_pose[8]  = tf_orientation.data[5];
            task_frame_pose[9] = tf_orientation.data[6]; task_frame_pose[10] = tf_orientation.data[7]; task_frame_pose[11] = tf_orientation.data[8];
        }

        task_frame_poses[i]                             = task_frame_pose;
        moveConstrained_follow_path_task_.tf_poses[i]   = KDL::Frame(tf_orientation, tf_position);
        moveConstrained_follow_path_task_.goal_poses[i] = KDL::Frame::Identity();
    }

    moveConstrained_follow_path_task_.tube_path_points             = tube_path_points;
    moveConstrained_follow_path_task_.tube_tolerances              = tube_tolerances;
    moveConstrained_follow_path_task_.tube_speed                   = tube_speed;
    moveConstrained_follow_path_task_.tube_force                   = tube_force;
    moveConstrained_follow_path_task_.contact_threshold_linear     = contact_threshold_linear;
    moveConstrained_follow_path_task_.contact_threshold_angular    = contact_threshold_angular;
    moveConstrained_follow_path_task_.time_limit                   = task_time_limit_sec;
    moveConstrained_follow_path_task_.tf_force                     = KDL::Rotation::Identity();
    moveConstrained_follow_path_task_.null_space_plane_orientation = KDL::Rotation::Identity();
    moveConstrained_follow_path_task_.null_space_force_direction   = KDL::Vector::Zero();

    // Set null-space error tolerance; small null-space oscillations are desired in this mode
    moveConstrained_follow_path_task_.null_space_tolerance = moveConstrained_follow_path_task_.tube_tolerances[5];

    desired_state_.frame_pose[END_EFF_]        = moveConstrained_follow_path_task_.goal_poses[0];
    desired_task_model_                        = task_model::moveConstrained_follow_path;
    desired_state_.frame_velocity[END_EFF_](0) = tube_speed;
    desired_state_.frame_velocity[END_EFF_](5) = 0.0;
    desired_state_.external_force[END_EFF_](2) = tube_force;
    desired_state_.external_force[END_EFF_](3) = 0.0;
    desired_state_.external_force[END_EFF_](4) = 0.0;
    compute_null_space_command_ = control_null_space;
    desired_null_space_angle_   = desired_null_space_angle;
    transform_force_drivers_    = true;
    compensate_unknown_weight_  = false;
}

void dynamics_controller::define_moveTo_follow_path_task(
                                const std::vector<bool> &constraint_direction,
                                const std::vector< std::vector<double> > &tube_path_points,
                                const std::vector<double> &tube_tolerances,
                                const double tube_speed,
                                const double contact_threshold_linear,
                                const double contact_threshold_angular,
                                const double task_time_limit_sec,
                                const bool control_null_space,
                                const double desired_null_space_angle,
                                std::vector< std::vector<double> > &task_frame_poses)
{
    assert(constraint_direction.size() == NUM_OF_CONSTRAINTS_);
    assert(tube_tolerances.size()      == NUM_OF_CONSTRAINTS_ + 2);
    assert(task_frame_poses.size()     == tube_path_points.size() - 1);
    assert(tube_path_points[0].size()  == 3);
    assert(task_frame_poses[0].size()  == 12);

    moveTo_follow_path_task_.tf_poses   = std::vector<KDL::Frame>(tube_path_points.size() - 1);
    moveTo_follow_path_task_.goal_poses = moveTo_follow_path_task_.tf_poses;

    CTRL_DIM_ = constraint_direction;
    MOTION_CTRL_DIM_ = CTRL_DIM_;

    // X-Y-Z linear
    KDL::Vector x_world(1.0, 0.0, 0.0);
    std::vector<double> task_frame_pose(12, 0.0);

    for (int i = 0; (unsigned)i < tube_path_points.size() - 1; i++)
    {
        KDL::Vector tube_start_position = KDL::Vector(tube_path_points[i    ][0], tube_path_points[i    ][1], tube_path_points[i    ][2]);
        KDL::Vector tf_position         = KDL::Vector(tube_path_points[i + 1][0], tube_path_points[i + 1][1], tube_path_points[i + 1][2]);
        KDL::Vector x_task              = tf_position - tube_start_position;
        x_task.Normalize();

        KDL::Vector cross_product = x_world * x_task;
        double cosine             = dot(x_world, x_task);
        double sine               = cross_product.Norm();
        double angle              = atan2(sine, cosine);

        task_frame_pose[0] = tf_position[0]; 
        task_frame_pose[1] = tf_position[1]; 
        task_frame_pose[2] = tf_position[2];

        KDL::Rotation tf_orientation;
        if(cosine < (-1 + 1e-6))
        {
            tf_orientation = KDL::Rotation::EulerZYZ(M_PI, 0.0, 0.0);
            task_frame_pose[3] = -1.0; task_frame_pose[4]  =  0.0; task_frame_pose[5]  = 0.0;
            task_frame_pose[6] =  0.0; task_frame_pose[7]  = -1.0; task_frame_pose[8]  = 0.0;
            task_frame_pose[9] =  0.0; task_frame_pose[10] =  0.0; task_frame_pose[11] = 1.0;
        }
        else if(sine < 1e-6)
        {
            tf_orientation = KDL::Rotation::Identity();
            task_frame_pose[3] = 1.0; task_frame_pose[4]  = 0.0; task_frame_pose[5]  = 0.0;
            task_frame_pose[6] = 0.0; task_frame_pose[7]  = 1.0; task_frame_pose[8]  = 0.0;
            task_frame_pose[9] = 0.0; task_frame_pose[10] = 0.0; task_frame_pose[11] = 1.0;
        }
        else
        {
            tf_orientation = geometry::exp_map_so3(cross_product / sine * angle);
            task_frame_pose[3] = tf_orientation.data[0]; task_frame_pose[4]  = tf_orientation.data[1]; task_frame_pose[5]  = tf_orientation.data[2];
            task_frame_pose[6] = tf_orientation.data[3]; task_frame_pose[7]  = tf_orientation.data[4]; task_frame_pose[8]  = tf_orientation.data[5];
            task_frame_pose[9] = tf_orientation.data[6]; task_frame_pose[10] = tf_orientation.data[7]; task_frame_pose[11] = tf_orientation.data[8];
        }

        task_frame_poses[i]                        = task_frame_pose;
        moveTo_follow_path_task_.tf_poses[i]       = KDL::Frame(tf_orientation, tf_position);
        moveTo_follow_path_task_.goal_poses[i]     = KDL::Frame::Identity();
    }

    moveTo_follow_path_task_.tube_path_points          = tube_path_points;
    moveTo_follow_path_task_.tube_tolerances           = tube_tolerances;
    moveTo_follow_path_task_.tube_speed                = tube_speed;
    moveTo_follow_path_task_.contact_threshold_linear  = contact_threshold_linear;
    moveTo_follow_path_task_.contact_threshold_angular = contact_threshold_angular;
    moveTo_follow_path_task_.time_limit                = task_time_limit_sec;
    
    // Set null-space error tolerance; small null-space oscillations are desired in this mode
    moveTo_follow_path_task_.null_space_force_direction = KDL::Vector::Zero();
    moveTo_follow_path_task_.null_space_tolerance       = tube_tolerances[7];

    desired_state_.frame_pose[END_EFF_]    = moveTo_follow_path_task_.goal_poses[0];
    desired_task_model_                    = task_model::moveTo_follow_path;
    transform_drivers_          = true;
    transform_force_drivers_    = false;
    compute_null_space_command_ = control_null_space;
    desired_null_space_angle_   = desired_null_space_angle;
    compensate_unknown_weight_  = false;
}


void dynamics_controller::define_moveTo_task(
                                const std::vector<bool> &constraint_direction,
                                const std::vector<double> &tube_start_position,
                                const std::vector<double> &tube_tolerances,
                                const double tube_speed,
                                const double contact_threshold_linear,
                                const double contact_threshold_angular,
                                const double task_time_limit_sec,
                                const bool control_null_space,
                                const double desired_null_space_angle,
                                std::vector<double> &task_frame_pose)
{
    assert(constraint_direction.size() == NUM_OF_CONSTRAINTS_);
    assert(tube_tolerances.size()      == NUM_OF_CONSTRAINTS_ + 2);
    assert(task_frame_pose.size()      == 12);
    assert(tube_start_position.size()  == 3);

    CTRL_DIM_ = constraint_direction;
    MOTION_CTRL_DIM_ = CTRL_DIM_;

    // X-Y-Z linear
    KDL::Vector x_world(1.0, 0.0, 0.0);
    KDL::Vector tf_position = KDL::Vector(task_frame_pose[0], task_frame_pose[1], task_frame_pose[2]);
    KDL::Vector x_task = tf_position - KDL::Vector(tube_start_position[0], tube_start_position[1], tube_start_position[2]);
    x_task.Normalize();

    KDL::Vector cross_product = x_world * x_task;
    double cosine             = dot(x_world, x_task);
    double sine               = cross_product.Norm();
    double angle              = atan2(sine, cosine);

    KDL::Rotation tf_orientation;
    if (cosine < (-1 + 1e-6))
    {
        tf_orientation = KDL::Rotation::EulerZYZ(M_PI, 0.0, 0.0);
        task_frame_pose[3] = -1.0; task_frame_pose[4]  =  0.0; task_frame_pose[5]  = 0.0;
        task_frame_pose[6] =  0.0; task_frame_pose[7]  = -1.0; task_frame_pose[8]  = 0.0;
        task_frame_pose[9] =  0.0; task_frame_pose[10] =  0.0; task_frame_pose[11] = 1.0;
    }
    else if(sine < 1e-6)
    {
        tf_orientation = KDL::Rotation::Identity();
        task_frame_pose[3] = 1.0; task_frame_pose[4]  = 0.0; task_frame_pose[5]  = 0.0;
        task_frame_pose[6] = 0.0; task_frame_pose[7]  = 1.0; task_frame_pose[8]  = 0.0;
        task_frame_pose[9] = 0.0; task_frame_pose[10] = 0.0; task_frame_pose[11] = 1.0;
    }
    else
    {
        tf_orientation = geometry::exp_map_so3(cross_product / sine * angle);
        task_frame_pose[3] = tf_orientation.data[0]; task_frame_pose[4]  = tf_orientation.data[1]; task_frame_pose[5]  = tf_orientation.data[2];
        task_frame_pose[6] = tf_orientation.data[3]; task_frame_pose[7]  = tf_orientation.data[4]; task_frame_pose[8]  = tf_orientation.data[5];
        task_frame_pose[9] = tf_orientation.data[6]; task_frame_pose[10] = tf_orientation.data[7]; task_frame_pose[11] = tf_orientation.data[8];
    }

    moveTo_task_.tf_pose                   = KDL::Frame(tf_orientation, tf_position);
    moveTo_task_.goal_pose                 = KDL::Frame::Identity();
    moveTo_task_.tube_start_position       = tube_start_position;
    moveTo_task_.tube_tolerances           = tube_tolerances;
    moveTo_task_.tube_speed                = tube_speed;
    moveTo_task_.contact_threshold_linear  = contact_threshold_linear;
    moveTo_task_.contact_threshold_angular = contact_threshold_angular;
    moveTo_task_.time_limit                = task_time_limit_sec;

    // Set null-space error tolerance; small null-space oscillations are desired in this mode
    moveTo_task_.null_space_force_direction = KDL::Vector::Zero();
    moveTo_task_.null_space_tolerance       = tube_tolerances[7];

    desired_state_.frame_pose[END_EFF_]    = moveTo_task_.goal_pose;
    desired_task_model_                    = task_model::moveTo;
    transform_drivers_          = true;
    transform_force_drivers_    = false;
    compute_null_space_command_ = control_null_space;
    desired_null_space_angle_   = desired_null_space_angle;
    compensate_unknown_weight_  = false;
}


// Define Cartesian Acceleration task on the end-effector - Public Method
void dynamics_controller::define_ee_acc_constraint(
                            const std::vector<bool> &constraint_direction,
                            const std::vector<double> &cartesian_acceleration)
{    
    //Call private method for this state
    set_ee_acc_constraints(desired_state_, 
                           constraint_direction, 
                           cartesian_acceleration);
}

// Define Cartesian Acceleration task on the end-effector - Private Method
void dynamics_controller::set_ee_acc_constraints(
                                state_specification &state,
                                const std::vector<bool> &constraint_direction, 
                                const std::vector<double> &cartesian_acceleration)
{    
    assert(constraint_direction.size()   == NUM_OF_CONSTRAINTS_);
    assert(cartesian_acceleration.size() == NUM_OF_CONSTRAINTS_);

    // Set directions in which constraint force should work. Alpha in the solver 
    KDL::Twist unit_force_x_l(
        KDL::Vector((constraint_direction[0] ? 1.0 : 0.0), 0.0, 0.0), 
        KDL::Vector(0.0, 0.0, 0.0));
    state.ee_unit_constraint_force.setColumn(0, unit_force_x_l);

    KDL::Twist unit_force_y_l(
            KDL::Vector(0.0, (constraint_direction[1] ? 1.0 : 0.0), 0.0),
            KDL::Vector(0.0, 0.0, 0.0));
    state.ee_unit_constraint_force.setColumn(1, unit_force_y_l);

    KDL::Twist unit_force_z_l(
            KDL::Vector(0.0, 0.0, (constraint_direction[2] ? 1.0 : 0.0)),
            KDL::Vector(0.0, 0.0, 0.0));
    state.ee_unit_constraint_force.setColumn(2, unit_force_z_l);

    KDL::Twist unit_force_x_a(
            KDL::Vector(0.0, 0.0, 0.0),
            KDL::Vector((constraint_direction[3] ? 1.0 : 0.0), 0.0, 0.0));
    state.ee_unit_constraint_force.setColumn(3, unit_force_x_a);

    KDL::Twist unit_force_y_a(
            KDL::Vector(0.0, 0.0, 0.0),
            KDL::Vector(0.0, (constraint_direction[4] ? 1.0 : 0.0), 0.0));
    state.ee_unit_constraint_force.setColumn(4, unit_force_y_a);

    KDL::Twist unit_force_z_a(
            KDL::Vector(0.0, 0.0, 0.0),
            KDL::Vector(0.0, 0.0, (constraint_direction[5] ? 1.0 : 0.0)));
    state.ee_unit_constraint_force.setColumn(5, unit_force_z_a);

    // Set desired acceleration on the end-effector. Beta in the solver
    for (int i = 0; i < NUM_OF_CONSTRAINTS_; i++)
        state.ee_acceleration_energy(i) = cartesian_acceleration[i];
}

// Define External force task - Public Method
void dynamics_controller::define_ee_external_force(const std::vector<double> &external_force)
{
    //Call private method for this state
    set_external_forces(desired_state_, external_force);
}

// Define External force task - Private Method
void dynamics_controller::set_external_forces(state_specification &state, 
                                              const std::vector<double> &external_force)
{
    //For now it is only updating forces on the end-effector
    //TODO: add forces on other segments as well
    assert(external_force.size() == NUM_OF_CONSTRAINTS_);

    state.external_force[END_EFF_] = KDL::Wrench (KDL::Vector(external_force[0],
                                                              external_force[1],
                                                              external_force[2]),
                                                  KDL::Vector(external_force[3],
                                                              external_force[4],
                                                              external_force[5]));
}

// Define FeedForward joint torques task - Public Method
void dynamics_controller::define_feedforward_torque(const std::vector<double> &ff_torque)
{
    //Call private method for this state
    set_feedforward_torque(desired_state_, ff_torque);
}

// Define FeedForward joint torques task - Private Method
void dynamics_controller::set_feedforward_torque(state_specification &state, 
                                                 const std::vector<double> &ff_torque)
{
    assert(ff_torque.size() == NUM_OF_JOINTS_);

    for (int i = 0; i < NUM_OF_JOINTS_; i++) state.feedforward_torque(i) = ff_torque[i];
}

//Make sure that the control loop runs exactly with the specified frequency
int dynamics_controller::enforce_loop_frequency(const int dt)
{
    loop_interval_ = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - loop_start_time_);

    if (loop_interval_ < std::chrono::microseconds(dt)) // Loop is sufficiently fast
    {
        while (loop_interval_ < std::chrono::microseconds(dt - 1))
        {
            loop_interval_ = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - loop_start_time_);
        }

        return 0;
    }
    else return -1; //Loop is too slow
}

/*
    Apply joint commands using safe control interface.
    If the computed commands are not safe, exit the program.
*/
int dynamics_controller::apply_joint_control_commands(const bool bypass_safeties)
{
    /*
        Safety controller checks if the commands are over the limits.
        If yes: stop the robot motion
        Else: use desired control mode
    */
    int safe_control_mode = safety_control_.set_control_commands(robot_state_,
                                                                 DT_SEC_,
                                                                 desired_control_mode_.interface,
                                                                 integration_method::SYMPLECTIC_EULER,
                                                                 bypass_safeties);
    if (!bypass_safeties)
    {
        // Check if the safety controller has changed the control mode.
        // Save the current decision: if desired control mode is safe or not.
        desired_control_mode_.is_safe = (desired_control_mode_.interface == safe_control_mode)? true : false;

        // Notify if the safety controller has changed the control mode
        switch (safe_control_mode)
        {
            case control_mode::TORQUE:
                if (!desired_control_mode_.is_safe)
                {
                    desired_control_mode_.interface = control_mode::STOP_MOTION;
                    error_logger_.error_source_ = error_source::safety_ctrl;
                    error_logger_.error_status_ = control_mode::STOP_MOTION;
                    return -1;
                }
                return 0;

            case control_mode::VELOCITY:
                if (!desired_control_mode_.is_safe) printf("WARNING: Control switched to velocity mode \n");
                return 0;

            case control_mode::POSITION:
                if (!desired_control_mode_.is_safe) printf("WARNING: Control switched to position mode \n");
                return 0;

            default:
                desired_control_mode_.interface = control_mode::STOP_MOTION;
                error_logger_.error_source_ = error_source::safety_ctrl;
                error_logger_.error_status_ = control_mode::STOP_MOTION;
                return -1;
        }
    }
    else 
    {
        if (safe_control_mode == control_mode::STOP_MOTION)
        {
            error_logger_.error_source_ = error_source::safety_ctrl;
            error_logger_.error_status_ = control_mode::STOP_MOTION;
        }
        return safe_control_mode;
    }
}

/*  
    Predict future robot Cartesian states given the current Cartesian state.
    I.e. Integrate Cartesian variables.
*/
void dynamics_controller::make_predictions(const double dt_sec, const int num_steps)
{
    predictor_.integrate_cartesian_space(robot_state_, predicted_state_, dt_sec, num_steps);
}


KDL::Twist dynamics_controller::finite_displacement_twist(const state_specification &state_a, 
                                                          const state_specification &state_b)
{
    /**
     * Difference between two poses: Decoupled calculation.
     * See "Modern Robotics" Book, 2017, sections 9.2.1 and 11.3.3.
    */

    // The default constructor initialises to Zero via the constructor of Vector!
    KDL::Twist twist;

    /**
     * This error part represents a linear motion necessary to go from 
     * predicted/measured to desired position (positive direction of translation).
    */
    twist.vel = state_a.frame_pose[END_EFF_].p - state_b.frame_pose[END_EFF_].p;

    /**
     * Describes rotation required to align R_m/p with R_d.
     * It represents relative rotation from measured/predicted state to 
     * desired state, expressed in the BASE frame!
     * Source: Luh et al. "Resolved-acceleration control of 
     * mechanical manipulators".
    */
    KDL::Rotation relative_rot_matrix = state_a.frame_pose[END_EFF_].M * \
                                        state_b.frame_pose[END_EFF_].M.Inverse();

    // Error calculation for angular part, i.e. logarithmic map on SO(3).
    twist.rot = geometry::log_map_so3(relative_rot_matrix);

    return twist;
}

double dynamics_controller::kinetic_energy(const KDL::Twist &twist, const int segment_index)
{
    return 0.5 * dot(twist, robot_chain_.getSegment(segment_index).getInertia() * twist);
}

void dynamics_controller::compute_moveConstrained_null_space_task_error()
{
    // X-axis of the null-space plane
    KDL::Vector plane_x(robot_state_.frame_pose[END_EFF_].p(0), robot_state_.frame_pose[END_EFF_].p(1), 0.0);
    double norm = plane_x.Normalize();
    
    if (norm < MIN_NORM)
    {
        moveConstrained_follow_path_task_.null_space_force_direction = KDL::Vector::Zero();
        printf("Null space norm too small 1 \n");
    }
    else 
    {
        // Calculate error: angle from the plane
        KDL::Vector plane_y(0.0, 0.0, 1.0);
        moveConstrained_follow_path_task_.null_space_plane_orientation = KDL::Rotation(plane_x, plane_y, plane_x * plane_y);
        KDL::Vector r_direction = moveConstrained_follow_path_task_.null_space_plane_orientation.Inverse() * robot_state_.frame_pose[3].p;
        r_direction.Normalize();
        null_space_angle_ = std::atan2(r_direction(2), r_direction(1)); // Angle between Y and R_yz

        // Calculate control/force direction: Cart force for null-space motion
        // plane_x = moveConstrained_follow_path_task_.null_space_plane_orientation.Inverse() * robot_state_.frame_pose[END_EFF_].p;
        // plane_x.Normalize();
        // KDL::Vector control_direction = r_direction * plane_x;

        KDL::Vector control_direction = r_direction * KDL::Vector(1.0, 0.0, 0.0);

        control_direction(0) = 0.0;
        norm = control_direction.Normalize();
    }
}

/**
 * For Gravity Compensation task, an error does not exist.
 * This is a pure feedforward control task.
 * Here, we only monitor task's time-threshold.
*/
void dynamics_controller::compute_gravity_compensation_task_error()
{
    fsm_result_ = fsm_.update_motion_task_status(robot_state_, desired_state_, current_error_twist_, ext_wrench_, total_time_sec_, tube_section_count_);
}

/**
 * Compute the general control error. 
 * Internally an error function will be called for each selected task
*/
void dynamics_controller::compute_control_error()
{    
    switch (desired_task_model_)
    {
        case task_model::moveConstrained_follow_path:
            compute_moveConstrained_follow_path_task_error();
            break;

        case task_model::moveTo_follow_path:
            compute_moveTo_follow_path_task_error();
            break;

        case task_model::moveTo:
            compute_moveTo_task_error();
            break;

        case task_model::moveGuarded:
            compute_moveGuarded_task_error();
            break;

        case task_model::moveTo_weight_compensation:
            compute_moveTo_weight_compensation_task_error();
            break;

        case task_model::full_pose:
            compute_full_pose_task_error(); 
            break;

        case task_model::gravity_compensation:
            compute_gravity_compensation_task_error(); 
            break;

        default:
            assert(("Unsupported task model", false));
            break;
    }
}

//Change the reference frame of external (virtual) forces, from task frame to base frame
void dynamics_controller::transform_force_driver()
{
    switch (desired_task_model_)
    {
        case task_model::moveConstrained_follow_path:
            cart_force_command_[END_EFF_] = moveConstrained_follow_path_task_.tf_force * cart_force_command_[END_EFF_];
            break;
        
        case task_model::moveTo_follow_path:
            cart_force_command_[END_EFF_] = moveTo_follow_path_task_.tf_poses[tube_section_count_].M * cart_force_command_[END_EFF_];
            break;

        case task_model::moveTo_weight_compensation:
            cart_force_command_[END_EFF_] = moveTo_weight_compensation_task_.tf_pose.M * cart_force_command_[END_EFF_];
            break;

        case task_model::moveGuarded:
            cart_force_command_[END_EFF_] = moveGuarded_task_.tf_pose.M * cart_force_command_[END_EFF_];
            break;

        default:
            cart_force_command_[END_EFF_] = moveTo_task_.tf_pose.M * cart_force_command_[END_EFF_];
            break;
    }
}

//Change the reference frame of constraint forces, from task frame to base frame
void dynamics_controller::transform_motion_driver()
{
    KDL::Wrench wrench_column;
    KDL::Twist twist_column;
    KDL::Jacobian alpha = robot_state_.ee_unit_constraint_force;

    //Change the reference frame of constraint forces, from task frame to base frame
    for (int c = 0; c < NUM_OF_CONSTRAINTS_; c++)
    {
        // Change Data Type of constraint forces to fit General KDL type
        wrench_column = KDL::Wrench(KDL::Vector(alpha(0, c), alpha(1, c), alpha(2, c)),
                                    KDL::Vector(alpha(3, c), alpha(4, c), alpha(5, c)));
        
        switch (desired_task_model_)
        {
            case task_model::moveConstrained_follow_path:
                wrench_column = moveConstrained_follow_path_task_.tf_poses[tube_section_count_].M * wrench_column;
                break;

            case task_model::moveTo_follow_path:
                wrench_column = moveTo_follow_path_task_.tf_poses[tube_section_count_].M * wrench_column;
                break;

            case task_model::moveTo_weight_compensation:
                wrench_column = moveTo_weight_compensation_task_.tf_pose.M * wrench_column;
                break;

            case task_model::moveGuarded:
                wrench_column = moveGuarded_task_.tf_pose.M * wrench_column;
                break;

            default:
                wrench_column = moveTo_task_.tf_pose.M * wrench_column;
                break;
        }

        // Change Data Type to fit Vereshchagin
        twist_column = KDL::Twist(wrench_column.force, wrench_column.torque);
        robot_state_.ee_unit_constraint_force.setColumn(c, twist_column);
    }
}

//Calculate robot dynamics - Resolve motion and forces using the Vereshchagin HD solver
int dynamics_controller::evaluate_dynamics()
{
    int hd_solver_result = this->hd_solver_->CartToJnt(robot_state_.q,
                                                       robot_state_.qd,
                                                       robot_state_.qdd,
                                                       robot_state_.ee_unit_constraint_force,
                                                       robot_state_.ee_acceleration_energy,
                                                       robot_state_.external_force,
                                                       cart_force_command_,
                                                       robot_state_.feedforward_torque);

    if (hd_solver_result != 0) return hd_solver_result;

    this->hd_solver_->get_control_torque(robot_state_.control_torque);

    return hd_solver_result;
}

int dynamics_controller::compute_gravity_compensation_control_commands()
{
    int id_solver_result = this->id_solver_->CartToJnt(robot_state_.q, zero_joint_array_, zero_joint_array_, zero_wrenches_, gravity_torque_);
    if (id_solver_result != 0) return id_solver_result;
    
    if (desired_task_model_ != task_model::gravity_compensation) robot_state_.control_torque.data = robot_state_.control_torque.data + gravity_torque_.data;
    else robot_state_.control_torque.data = gravity_torque_.data;

    for (int j = 0; j < NUM_OF_JOINTS_; j++)
    {
        if      (robot_state_.control_torque(j) >=  JOINT_TORQUE_LIMITS_[j]) robot_state_.control_torque(j) =  JOINT_TORQUE_LIMITS_[j] - 0.001;
        else if (robot_state_.control_torque(j) <= -JOINT_TORQUE_LIMITS_[j]) robot_state_.control_torque(j) = -JOINT_TORQUE_LIMITS_[j] + 0.001;
    }

    return id_solver_result;
}

void dynamics_controller::set_parameters(const double horizon_amplitude,
                                         const Eigen::VectorXd &max_command,
                                         const Eigen::VectorXd &error_alpha, 
                                         const Eigen::VectorXd &bias_threshold, 
                                         const Eigen::VectorXd &bias_step, 
                                         const Eigen::VectorXd &gain_threshold, 
                                         const Eigen::VectorXd &gain_step,
                                         const Eigen::VectorXd &min_bias_sat,
                                         const Eigen::VectorXd &min_command_sat,
                                         const Eigen::VectorXd &null_space_parameters,
                                         const Eigen::VectorXd &compensation_parameters,
                                         const Eigen::VectorXd &stop_motion_error_alpha,
                                         const Eigen::VectorXd &stop_motion_bias_threshold,
                                         const Eigen::VectorXd &stop_motion_bias_step,
                                         const Eigen::VectorXd &stop_motion_gain_threshold,
                                         const Eigen::VectorXd &stop_motion_gain_step)
{
    //First check input dimensions
    assert(max_command.size()             == NUM_OF_CONSTRAINTS_); 
    assert(error_alpha.size()             == NUM_OF_CONSTRAINTS_); 
    assert(bias_threshold.size()          == NUM_OF_CONSTRAINTS_); 
    assert(bias_step.size()               == NUM_OF_CONSTRAINTS_); 
    assert(gain_threshold.size()          == NUM_OF_CONSTRAINTS_); 
    assert(gain_step.size()               == NUM_OF_CONSTRAINTS_); 
    assert(null_space_parameters.size()   == NUM_OF_CONSTRAINTS_); 
    assert(compensation_parameters.size() == 2 * NUM_OF_CONSTRAINTS_); 
    assert(stop_motion_error_alpha.size()    == NUM_OF_JOINTS_); 
    assert(stop_motion_bias_threshold.size() == NUM_OF_JOINTS_); 
    assert(stop_motion_bias_step.size()      == NUM_OF_JOINTS_); 
    assert(stop_motion_gain_threshold.size() == NUM_OF_JOINTS_); 
    assert(stop_motion_gain_step.size()      == NUM_OF_JOINTS_); 


    this->horizon_amplitude_        = horizon_amplitude;
    this->max_command_              = max_command;

    // Setting parameters of the force controller DOF
    this->force_task_parameters_(0) = error_alpha(2);
    this->force_task_parameters_(1) = bias_threshold(2);
    this->force_task_parameters_(2) = bias_step(2);
    this->force_task_parameters_(3) = gain_threshold(2);
    this->force_task_parameters_(4) = gain_step(2);
    this->force_task_parameters_(5) = max_command(2);
    this->min_sat_limits_ = min_command_sat;

    this->null_space_parameters_   = null_space_parameters;
    this->compensation_parameters_ = compensation_parameters;
}

int dynamics_controller::initialize(const int desired_control_mode, 
                                    const int desired_dynamics_interface,
                                    const bool store_control_data,
                                    const int motion_profile)
{
    // Save current selection of desire control mode
    desired_control_mode_.interface = desired_control_mode;

    //Exit the program if the "Stop Motion" mode is selected
    assert(desired_control_mode_.interface != control_mode::STOP_MOTION); 

    desired_dynamics_interface_ = desired_dynamics_interface;

    switch (desired_task_model_)
    {
        case task_model::moveConstrained_follow_path:
            fsm_result_ = fsm_.initialize_with_moveConstrained_follow_path(moveConstrained_follow_path_task_, 
                                                                           motion_profile);
            break;

        case task_model::moveTo_follow_path:
            fsm_result_ = fsm_.initialize_with_moveTo_follow_path(moveTo_follow_path_task_, 
                                                                  motion_profile);
            break;

        case task_model::moveTo:
            fsm_result_ = fsm_.initialize_with_moveTo(moveTo_task_, motion_profile);
            break;

        case task_model::moveGuarded:
            fsm_result_ = fsm_.initialize_with_moveGuarded(moveGuarded_task_, motion_profile);
            break;

        case task_model::moveTo_weight_compensation:
            fsm_result_ = fsm_.initialize_with_moveTo_weight_compensation(moveTo_weight_compensation_task_, 
                                                                          motion_profile,
                                                                          compensation_parameters_);
            break;

        case task_model::full_pose:
            fsm_result_ = fsm_.initialize_with_full_pose(full_pose_task_, motion_profile);
            break;

        case task_model::gravity_compensation:
            fsm_result_ = fsm_.initialize_with_gravity_compensation(gravity_compensation_task_);
            break;
 
        default:
            printf("Unsupported task model\n");
            return -1;
            break;
    }

    store_control_data_ = store_control_data;

    if (store_control_data_) 
    {
        log_file_cart_.open(dynamics_parameter::LOG_FILE_CART_PATH);
        assert(log_file_cart_.is_open());

        for (int i = 0; i < NUM_OF_CONSTRAINTS_ + 2; i++)
        {
            if      (desired_task_model_ == task_model::moveConstrained_follow_path) log_file_cart_ << moveConstrained_follow_path_task_.tube_tolerances[i] << " ";            
            else if (desired_task_model_ == task_model::moveTo_follow_path)          log_file_cart_ << moveTo_follow_path_task_.tube_tolerances[i] << " ";
            else if (desired_task_model_ == task_model::moveTo_weight_compensation)  log_file_cart_ << moveTo_weight_compensation_task_.tube_tolerances[i] << " ";
            else if (desired_task_model_ == task_model::moveGuarded)                 log_file_cart_ << moveGuarded_task_.tube_tolerances[i] << " ";
            else                                                                     log_file_cart_ << moveTo_task_.tube_tolerances[i] << " ";
        }
        log_file_cart_ << std::endl;

        for (int i = 0; i < NUM_OF_CONSTRAINTS_; i++)
            log_file_cart_ << max_command_(i) << " ";
        log_file_cart_ << std::endl;

        log_file_stop_motion_.open(dynamics_parameter::LOG_FILE_STOP_MOTION_PATH);
        assert(log_file_stop_motion_.is_open());
        log_file_stop_motion_ << dynamics_parameter::STOPPING_MOTION_LOOP_FREQ  << std::endl;

        log_file_cart_base_.open(dynamics_parameter::LOG_FILE_CART_BASE_PATH);
        assert(log_file_cart_base_.is_open());

        for (int i = 0; i < NUM_OF_CONSTRAINTS_ + 2; i++)
        {
            if      (desired_task_model_ == task_model::moveConstrained_follow_path) log_file_cart_base_ << moveConstrained_follow_path_task_.tube_tolerances[i] << " ";            
            else if (desired_task_model_ == task_model::moveTo_follow_path)          log_file_cart_base_ << moveTo_follow_path_task_.tube_tolerances[i] << " ";
            else if (desired_task_model_ == task_model::moveTo_weight_compensation)  log_file_cart_base_ << moveTo_weight_compensation_task_.tube_tolerances[i] << " ";
            else if (desired_task_model_ == task_model::moveGuarded)                 log_file_cart_base_ << moveGuarded_task_.tube_tolerances[i] << " ";
            else                                                                     log_file_cart_base_ << moveTo_task_.tube_tolerances[i] << " ";
        }
        log_file_cart_base_ << std::endl;

        log_file_predictions_.open(dynamics_parameter::LOG_FILE_PREDICTIONS_PATH);
        assert(log_file_predictions_.is_open());

        log_file_null_space_.open(dynamics_parameter::LOG_FILE_NULL_SPACE_PATH);
        assert(log_file_null_space_.is_open());
        if      (desired_task_model_ == task_model::moveConstrained_follow_path) log_file_null_space_ << moveConstrained_follow_path_task_.null_space_tolerance << std::endl;           
        else if (desired_task_model_ == task_model::moveTo_follow_path)          log_file_null_space_ << moveTo_follow_path_task_.null_space_tolerance << std::endl;
        else if (desired_task_model_ == task_model::moveTo_weight_compensation)  log_file_null_space_ << moveTo_weight_compensation_task_.null_space_tolerance << std::endl;
        else if (desired_task_model_ == task_model::moveGuarded)                 log_file_null_space_ << moveGuarded_task_.null_space_tolerance << std::endl;
        else if (desired_task_model_ == task_model::moveTo)                      log_file_null_space_ << moveTo_task_.null_space_tolerance << std::endl;
        else                                                                     log_file_null_space_ << full_pose_task_.null_space_tolerance << std::endl;

        log_file_joint_.open(dynamics_parameter::LOG_FILE_JOINT_PATH);
        assert(log_file_joint_.is_open());
        for(int i = 0; i < NUM_OF_JOINTS_; i++) 
            log_file_joint_ << JOINT_TORQUE_LIMITS_[i] << " ";
        log_file_joint_ << std::endl;
    }

    KDL::SetToZero(robot_state_.feedforward_torque);
    KDL::SetToZero(desired_state_.qd);

    /* 
        Get sensor data from the robot driver or if simulation is on, 
        replace current state with the integrated joint velocities and positions.
    */
    int state_update_result = update_current_state();
    if (state_update_result != 0)
    {
        // Make sure that the robot is locked (freezed)
        engage_lock();

        error_logger_.error_source_ = error_source::fk_solver;
        error_logger_.error_status_ = state_update_result;
        deinitialize();
        return -1;
    }

    // First make sure that the robot is not moving
    stopping_sequence_on_ = true;
    if (control() == -1)
    {
        stopping_sequence_on_ = false;
        deinitialize();
        return -1;
    }
    stopping_sequence_on_ = false;
    trigger_stopping_sequence_ = false;
    loop_iteration_count_ = 0;
    stop_loop_iteration_count_ = 0;
    error_logger_.error_source_ = error_source::empty;
    error_logger_.error_status_ = 0;

    // Make sure that the robot is locked (freezed)
    engage_lock();

    //Print information about controller settings
    // print_settings_info();
    return 0;
}

void dynamics_controller::engage_lock()
{
    /** 
     * In the case of Kinova: Position-control based stopping mechanism.
     * - It stops the robot correctly. However, the deceleration is too strong.
     * - More specificily, it is not good for the robot's gears and motors on long-run.
     * - It is best if the deceleration is controlled explicitly before-hand.
     */
    safety_control_.stop_robot_motion();
}

/**
 * Torque-control based stopping mechanism:
 * - It stops the robot correctly. We can control deceleration explicitly.
 * - However, control loop (thus, torque control) is maintained on the external pc.
 */
int dynamics_controller::update_stop_motion_commands()
{
    if (stop_loop_iteration_count_ % dynamics_parameter::DECELERATION_UPDATE_DELAY == 0)
    {
        for (int i = 0; i < NUM_OF_JOINTS_; i++)
        {
            if (stop_motion_setpoint_array_[i].size() > 0)
            {
                desired_state_.qd(i) = stop_motion_setpoint_array_[i].front();
                stop_motion_setpoint_array_[i].pop_front();
            }
            else desired_state_.qd(i) = 0.0;
        }
    }

    // Compute control error
    int joint_stop_count = 0;


    // If all of the joints have 0 velocity -> stopping motion task completed
    if (joint_stop_count == NUM_OF_JOINTS_)
    {
        steady_stop_iteration_count_++;
        if (steady_stop_iteration_count_ == dynamics_parameter::STEADY_STOP_ITERATION_THRESHOLD) return 1;
    }
    else steady_stop_iteration_count_ = 0;

    return 0;
}

/**
 * Performs single update of control commands and dynamics computations
*/
int dynamics_controller::update_commands()
{
    int status = 0;

    // Cartesian Control Commands computed by the independent ABAG controllers
    if (desired_task_model_ != task_model::gravity_compensation) compute_cart_control_commands();
    if (compensate_unknown_weight_)
    {
        status = compute_weight_compensation_control_commands();
        if (status == -1)
        {
            error_logger_.error_source_ = error_source::weight_compensator;
            error_logger_.error_status_ = status;
            return -1;
        }
    }

    // Evaluate robot dynamics using the Vereshchagin HD solver
    if (desired_task_model_ != task_model::gravity_compensation)
    {
        status = evaluate_dynamics();
        if (status != 0)
        {
            error_logger_.error_source_ = error_source::vereshchagin_solver;
            error_logger_.error_status_ = status;
            return -1;
        }
    }

    // Compute necessary torques for compensating gravity, using the RNE ID solver
    if (COMPENSATE_GRAVITY_ || desired_task_model_ == task_model::gravity_compensation) 
    {
        status = compute_gravity_compensation_control_commands();
        if (status != 0)
        {
            error_logger_.error_source_ = error_source::rne_solver;
            error_logger_.error_status_ = status;
            return -1;
        }
    }
    return 0;
}


/**
 * Perform single step of the control loop, given current robot joint state
 * Required for either main internal control loop and the RTT's updateHook method
*/
int dynamics_controller::step(const KDL::JntArray &q_input,
                              const KDL::JntArray &qd_input,
                              const KDL::Wrench &ext_force,
                              Eigen::VectorXd &tau_output,
                              const double time_passed_sec,
                              const int main_loop_iteration,
                              const int stop_loop_iteration,
                              const bool stopping_behaviour_on)
{
    robot_state_.q  = q_input;
    robot_state_.qd = qd_input;
    ext_wrench_     = ext_force;
    total_time_sec_ = time_passed_sec;
    loop_iteration_count_ = main_loop_iteration;
    stop_loop_iteration_count_ = stop_loop_iteration;

    if (!stopping_behaviour_on)
    {
        // Get Cartesian poses and velocities
        int status = fk_vereshchagin_.JntToCart(robot_state_.q,
                                                robot_state_.qd,
                                                robot_state_.frame_pose,
                                                robot_state_.frame_velocity);
        if (status != 0)
        {
            error_logger_.error_source_ = error_source::fk_solver;
            error_logger_.error_status_ = status;
            return -1;
        }
        // Save the state expressed in base frame
        robot_state_base_.frame_pose = robot_state_.frame_pose;

        compute_control_error();

        if (check_fsm_status() == -1)
        {
            error_logger_.error_source_ = error_source::fsm;
            error_logger_.error_status_ = -1;
            return -1;
        }

        if (update_commands() == -1) return -1;
    }
    else
    {
        if (stop_loop_iteration_count_ == 0)
        {
            desired_control_mode_.interface = control_mode::TORQUE;
            steady_stop_iteration_count_ = 0;
            stop_motion_setpoint_array_.clear();
            double step = 0.0;
            for (int i = 0; i < NUM_OF_JOINTS_; i++)
            {
                step = JOINT_ACC_LIMITS_[i] * (dynamics_parameter::DECELERATION_UPDATE_DELAY / dynamics_parameter::STOPPING_MOTION_LOOP_FREQ) * ((robot_state_.qd(i) > 0.0)? -1.0 : 1.0); // rad/sec
                stop_motion_setpoint_array_.push_back(motion_profile::ramp_array(robot_state_.qd(i), 0.0, step, dynamics_parameter::LOWER_DECELERATION_RAMP_THRESHOLD));
            }
        }

        if (update_stop_motion_commands() == 1) return 1;
    }

    // Save final commands
    tau_output = robot_state_.control_torque.data;

    // Log control data for visualization and debuging
    if (store_control_data_) write_to_file();
    return 0;
}

// Main control loop
int dynamics_controller::control()
{
    // double loop_time = 0.0;
    KDL::JntArray state_q(NUM_OF_JOINTS_), state_qd(NUM_OF_JOINTS_), ctrl_torque(NUM_OF_JOINTS_);
    KDL::Wrench ext_force;
    total_time_sec_ = 0.0;
    int return_flag = 0;

    // One loop frequency for both communication and dynamics-command update
    while (1)
    {
        // Save current time point
        loop_start_time_ = std::chrono::steady_clock::now();
        if (!stopping_sequence_on_) total_time_sec_ = loop_iteration_count_ * DT_SEC_;

        //Get current robot state from the joint sensors: velocities and angles
        safety_control_.get_current_state(robot_state_);

        state_q   = robot_state_.q;
        state_qd  = robot_state_.qd;
        ext_force = ext_wrench_;

        // Make one control iteration (step) -> Update control commands
        return_flag = step(state_q, state_qd, ext_force, ctrl_torque.data, total_time_sec_, loop_iteration_count_, stop_loop_iteration_count_, stopping_sequence_on_);
        if (return_flag == -1) trigger_stopping_sequence_ = true;

        if (stopping_sequence_on_) // Robot will be controlled to stop its motion and eventually lock
        {
            if (return_flag == 1) // Stop motion task completed
            {
                // Make sure that the robot is locked (freezed)
                engage_lock();

                stopping_sequence_on_ = false;
                total_time_sec_ += (double)stop_loop_iteration_count_ / dynamics_parameter::STOPPING_MOTION_LOOP_FREQ;
                printf("Robot stopped!\n");
                return 0;
            }

            // Apply joint commands using torque control interface (bypass all safety checks)
            if (apply_joint_control_commands(true) == -1)
            {
                // Make sure that the robot is locked (freezed)
                engage_lock();

                stopping_sequence_on_ = false;
                total_time_sec_ += (double)stop_loop_iteration_count_ / dynamics_parameter::STOPPING_MOTION_LOOP_FREQ;
                printf("Robot stopped!\n");
                return -1;
            }

            stop_loop_iteration_count_++;
            if (enforce_loop_frequency(DT_STOPPING_MICRO_) != 0) control_loop_delay_count_++;
            // Testing loop time
            // loop_time += std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - loop_start_time_).count();
            // if (stop_loop_iteration_count_ == 2000) 
            // {
            //     printf("Stop loop time: %f\n", loop_time / 2000.0);
            //     return 0;
            // }
        }
        else // Nominal task execution mode
        {
            if (!trigger_stopping_sequence_)
            {
                // Apply joint commands using safe control interface
                if (apply_joint_control_commands(false) == -1) trigger_stopping_sequence_ = true;
            }

            if (trigger_stopping_sequence_)
            {
                trigger_stopping_sequence_ = false;
                stopping_sequence_on_ = true;
                stop_loop_iteration_count_ = 0;

                printf("Stopping behaviour triggered!\n");
                continue;
            }

            loop_iteration_count_++;
            if (enforce_loop_frequency(DT_MICRO_) != 0) control_loop_delay_count_++;
            // Testing loop time
            // loop_time += std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - loop_start_time_).count();
            // if (loop_iteration_count_ == 2000) 
            // {
            //     printf("Main loop time: %f\n", loop_time / 2000.0);
            //     return 0;
            // }
        }
    }
    return 0;
}

void dynamics_controller::deinitialize()
{
    if (store_control_data_) close_files();

    if (error_logger_.error_source_ != error_source::empty)
    {
        printf("Error Source: %d\n", static_cast<std::underlying_type<error_source>::type>(error_logger_.error_source_));
        printf("Error Status: %d\n\n", error_logger_.error_status_);
    }

    // printf("Loop Statistics: \n");
    // printf("   - Number of iterations: %d\n", loop_iteration_count_);
    // printf("   - Total time: %f sec\n", total_time_sec_);
    // printf("   - Delay in control loop occurred %d times\n\n", control_loop_delay_count_);
}

void dynamics_controller::close_files()
{
    log_file_cart_.close();
    log_file_stop_motion_.close();
    log_file_cart_base_.close();
    log_file_joint_.close();
    log_file_predictions_.close();
    log_file_null_space_.close();
}