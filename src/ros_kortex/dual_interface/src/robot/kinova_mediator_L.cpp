/*
Author(s): Djordje Vukcevic, Sven Schneider
Description: Mediator component for enabling conversion of data types.

Copyright (c) [2020]

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

#include "kinova_mediator_L.hpp"
#define IP_ADDRESS_1 "192.168.1.10"
#define IP_ADDRESS_2 "192.168.1.20"
#define PORT 10000
#define PORT_REAL_TIME 10001
#define ACTUATOR_COUNT 7

kinova_mediator_L::kinova_mediator_L(): 
    is_initialized_(false), kinova_id(robot_id::KINOVA_GEN3_1),
    kinova_model_(kinova_model::URDF),
    kinova_environment_(kinova_environment::SIMULATION),
    control_mode_(control_mode::STOP_MOTION),
    add_offsets_(false), connection_established_(false),
    linear_root_acc_(kinova_constants::root_acceleration_L[0],
                     kinova_constants::root_acceleration_L[1],
                     kinova_constants::root_acceleration_L[2]),
    angular_root_acc_(kinova_constants::root_acceleration_L[3],
                      kinova_constants::root_acceleration_L[4],
                      kinova_constants::root_acceleration_L[5]),
    root_acc_(linear_root_acc_, angular_root_acc_),
    transport_(nullptr), transport_real_time_(nullptr), router_(nullptr),
    router_real_time_(nullptr), session_manager_(nullptr),
    session_manager_real_time_(nullptr), base_(nullptr),
    base_cyclic_(nullptr), actuator_config_(nullptr)
{

}

kinova_mediator_L::~kinova_mediator_L()
{
    // Close API sessions and connections
    if (is_initialized_) deinitialize();
}

// Update joint space state: measured positions, velocities and torques
void kinova_mediator_L::get_joint_state(KDL::JntArray &joint_positions,
                                      KDL::JntArray &joint_velocities,
                                      KDL::JntArray &joint_torques)
{
    if (kinova_environment_ != kinova_environment::SIMULATION) base_feedback_ = base_cyclic_->RefreshFeedback();

    get_joint_positions(joint_positions);
    get_joint_velocities(joint_velocities);
    get_joint_torques(joint_torques);
}

// Get Joint Positions
void kinova_mediator_L::get_joint_positions(KDL::JntArray &joint_positions) 
{
    // Joint position given in deg
    for (int i = 0; i < ACTUATOR_COUNT; i++)
        joint_positions(i) = DEG_TO_RAD(base_feedback_.actuators(i).position());

    // Kinova API provides only positive angle values
    // This operation is required to align the logic with our safety controller
    // We need to convert some angles to negative values
    if (joint_positions(0) > DEG_TO_RAD(180.0)) joint_positions(0) = joint_positions(0) - DEG_TO_RAD(360.0);
    if (joint_positions(1) > DEG_TO_RAD(180.0)) joint_positions(1) = joint_positions(1) - DEG_TO_RAD(360.0);
    if (joint_positions(2) > DEG_TO_RAD(180.0)) joint_positions(2) = joint_positions(2) - DEG_TO_RAD(360.0);
    if (joint_positions(3) > DEG_TO_RAD(180.0)) joint_positions(3) = joint_positions(3) - DEG_TO_RAD(360.0);
    if (joint_positions(4) > DEG_TO_RAD(180.0)) joint_positions(4) = joint_positions(4) - DEG_TO_RAD(360.0);
    if (joint_positions(5) > DEG_TO_RAD(180.0)) joint_positions(5) = joint_positions(5) - DEG_TO_RAD(360.0);
    if (joint_positions(6) > DEG_TO_RAD(180.0)) joint_positions(6) = joint_positions(6) - DEG_TO_RAD(360.0);
}

//Set Joint Positions
int kinova_mediator_L::set_joint_positions(const KDL::JntArray &joint_positions)
{
    for (int i = 0; i < ACTUATOR_COUNT; i++)
    {
        base_command_.mutable_actuators(i)->set_position(RAD_TO_DEG(joint_positions(i)));
    }

    if (kinova_environment_ != kinova_environment::SIMULATION)
    {
        increment_command_id();

        // Send the commands
        try
        {
            base_feedback_ = base_cyclic_->Refresh(base_command_, 0);
        }
        catch (Kinova::Api::KDetailedException& ex)
        {
            std::cout << "Kortex exception: " << ex.what() << std::endl;
            std::cout << "Error sub-code: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
            return -1;
        }
        catch (std::runtime_error& ex2)
        {
            std::cout << "runtime error: " << ex2.what() << std::endl;
            return -1;
        }
        catch(...)
        {
            std::cout << "Unknown error." << std::endl;
            return -1;
        }
    }
    else // Necessary to safe current commands as the next state for the simulation environment
    {
        for (int i = 0; i < ACTUATOR_COUNT; i++)
            base_feedback_.mutable_actuators(i)->set_position(RAD_TO_DEG(joint_positions(i)));
    }
    
    return 0;
}

// Get Joint Velocities
void kinova_mediator_L::get_joint_velocities(KDL::JntArray &joint_velocities)
{
    // Joint velocity given in deg/sec
    for (int i = 0; i < ACTUATOR_COUNT; i++)
        joint_velocities(i) = DEG_TO_RAD(base_feedback_.actuators(i).velocity());
}

// Set Joint Velocities
int kinova_mediator_L::set_joint_velocities(const KDL::JntArray &joint_velocities)
{
    for (int i = 0; i < ACTUATOR_COUNT; i++)
    {
        base_command_.mutable_actuators(i)->set_position(base_feedback_.actuators(i).position());
        base_command_.mutable_actuators(i)->set_velocity(RAD_TO_DEG(joint_velocities(i)));
    }

    if (kinova_environment_ != kinova_environment::SIMULATION)
    {
        increment_command_id();
    
        // Send the commands
        try
        {
            base_feedback_ = base_cyclic_->Refresh(base_command_, 0);
        }
        catch (Kinova::Api::KDetailedException& ex)
        {
            std::cout << "Kortex exception: " << ex.what() << std::endl;
            std::cout << "Error sub-code: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
            return -1;
        }
        catch (std::runtime_error& ex2)
        {
            std::cout << "runtime error: " << ex2.what() << std::endl;
            return -1;
        }
        catch(...)
        {
            std::cout << "Unknown error." << std::endl;
            return -1;
        }
    }
    else // Necessary to safe current commands as the next state for the simulation environment
    {
        for (int i = 0; i < ACTUATOR_COUNT; i++)
            base_feedback_.mutable_actuators(i)->set_velocity(RAD_TO_DEG(joint_velocities(i)));
    }
    return 0;
}

// Get Joint Torques
void kinova_mediator_L::get_joint_torques(KDL::JntArray &joint_torques)
{
    // Joint torque given in Newton * meters
    for (int i = 0; i < ACTUATOR_COUNT; i++)
        joint_torques(i) = base_feedback_.actuators(i).torque();
}

// Set Joint Torques
int kinova_mediator_L::set_joint_torques(const KDL::JntArray &joint_torques) 
{
    for (int i = 0; i < ACTUATOR_COUNT; i++)
    {
        base_command_.mutable_actuators(i)->set_position(base_feedback_.actuators(i).position());
        base_command_.mutable_actuators(i)->set_torque_joint(joint_torques(i));
    }

    if (kinova_environment_ != kinova_environment::SIMULATION)
    {
        increment_command_id();

        // Send the commands
        try
        {
            base_feedback_ = base_cyclic_->Refresh(base_command_, 0);
        }
        catch (Kinova::Api::KDetailedException& ex)
        {
            std::cout << "Kortex exception: " << ex.what() << std::endl;
            std::cout << "Error sub-code: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
            return -1;
        }
        catch (std::runtime_error& ex2)
        {
            std::cout << "runtime error: " << ex2.what() << std::endl;
            return -1;
        }
        catch(...)
        {
            std::cout << "Unknown error." << std::endl;
            return -1;
        }
    }
    else // Necessary to safe current commands as the next state for the simulation environment
    {
        for (int i = 0; i < ACTUATOR_COUNT; i++)
            base_feedback_.mutable_actuators(i)->set_torque(joint_torques(i));
    }
    return 0;
}

int kinova_mediator_L::set_control_mode(const int desired_control_mode)
{
    control_mode_ = desired_control_mode;
    if (kinova_environment_ != kinova_environment::SIMULATION)
    {
        try
        {
            switch (desired_control_mode)
            {
                case control_mode::TORQUE:
                    // Set actuators in torque mode
                    control_mode_message_.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::TORQUE);
                    for (int actuator_id = 1; actuator_id < ACTUATOR_COUNT + 1; actuator_id++)
                        actuator_config_->SetControlMode(control_mode_message_, actuator_id);
                    return 0;

                case control_mode::VELOCITY:
                    // Set actuators in velocity mode
                    control_mode_message_.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::VELOCITY);
                    for (int actuator_id = 1; actuator_id < ACTUATOR_COUNT + 1; actuator_id++)
                        actuator_config_->SetControlMode(control_mode_message_, actuator_id);
                    return 0;

                case control_mode::POSITION:
                    // Set actuators in position mode
                    control_mode_message_.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::POSITION);
                    for (int actuator_id = 1; actuator_id < ACTUATOR_COUNT + 1; actuator_id++)
                        actuator_config_->SetControlMode(control_mode_message_, actuator_id);
                    return 0;

                default:
                    assert(("Unknown control mode!", false));
                    return -1;
            }
        }
        catch (Kinova::Api::KDetailedException& ex)
        {
            std::cout << "Kortex exception: " << ex.what() << std::endl;
            std::cout << "Error sub-code: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
            return -1;
        }
        catch (std::runtime_error& ex2)
        {
            std::cout << "runtime error: " << ex2.what() << std::endl;
            return -1;
        }
        catch(...)
        {
            std::cout << "Unknown error." << std::endl;
            return -1;
        }
    }
    return 0;
}

int kinova_mediator_L::set_joint_command(const KDL::JntArray &joint_positions,
                                       const KDL::JntArray &joint_velocities,
                                       const KDL::JntArray &joint_torques,
                                       const int desired_control_mode)
{
    assert(joint_positions.rows()  == ACTUATOR_COUNT);
    assert(joint_velocities.rows() == ACTUATOR_COUNT);
    assert(joint_torques.rows()    == ACTUATOR_COUNT);

    if (kinova_environment_ == kinova_environment::SIMULATION)
    {
        set_joint_torques(joint_torques);
        set_joint_velocities(joint_velocities);
        set_joint_positions(joint_positions);
        control_mode_ = desired_control_mode;
    }
    else
    {
        switch (desired_control_mode)
        {   
            case control_mode::TORQUE:
                if (control_mode_ != control_mode::TORQUE) set_control_mode(desired_control_mode);
                return set_joint_torques(joint_torques);

            case control_mode::VELOCITY:
                if (control_mode_ != control_mode::VELOCITY) set_control_mode(desired_control_mode);
                return set_joint_velocities(joint_velocities);

            case control_mode::POSITION:
                if (control_mode_ != control_mode::POSITION) set_control_mode(desired_control_mode);
                return set_joint_positions(joint_positions);

            default: 
                assert(("Unknown control mode!", false));
                return -1;
        }
    }
    return 0;
}

bool kinova_mediator_L::robot_stopped()
{
    // Check if velocity control mode is active
    // if (control_mode_message_.control_mode() != Kinova::Api::ActuatorConfig::ControlMode::VELOCITY) return false;

    base_feedback_ = base_cyclic_->RefreshFeedback();
    for (int i = 0; i < ACTUATOR_COUNT; i++)
    {
        // Check if velocity setpoint is zero
        if ((base_feedback_.actuators(i).velocity() != 0.0) || \
            !std::isfinite(base_feedback_.actuators(i).velocity())) return false;
    }
    return true;
}

// Set Zero Joint Velocities and wait until robot has stopped completely
int kinova_mediator_L::stop_robot_motion()
{
    if (kinova_environment_ != kinova_environment::SIMULATION)
    {
        base_feedback_ = base_cyclic_->RefreshFeedback();
        
        for (int i = 0; i < ACTUATOR_COUNT; i++)
            base_command_.mutable_actuators(i)->set_position(base_feedback_.actuators(i).position());
        
        if (control_mode_ != control_mode::POSITION) set_control_mode(control_mode::POSITION);

        increment_command_id();

        // Send the commands
        try
        {
            base_feedback_ = base_cyclic_->Refresh(base_command_, 0);
        }
        catch (Kinova::Api::KDetailedException& ex)
        {
            std::cout << "Kortex exception: " << ex.what() << std::endl;
            std::cout << "Error sub-code: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
            return -1;
        }
        catch (std::runtime_error& ex2)
        {
            std::cout << "runtime error: " << ex2.what() << std::endl;
            return -1;
        }
        catch(...)
        {
            std::cout << "Unknown error." << std::endl;
            return -1;
        }
    }
    else
    {
        // Simulated low-level velocity interface
        if (control_mode_ != control_mode::VELOCITY) set_control_mode(control_mode::VELOCITY);

        // Send the zero velocity commands to motors
        for (int i = 0; i < ACTUATOR_COUNT; i++)
        {
            base_command_.mutable_actuators(i)->set_position(base_feedback_.actuators(i).position());
            base_command_.mutable_actuators(i)->set_velocity(0.0);
        }
    }
    return 0;
}

// Increses index of the command's frame id (buffer)
void kinova_mediator_L::increment_command_id()
{
    // Incrementing identifier ensures actuators can reject out of time frames
    // Buffer?
    base_command_.set_frame_id(base_command_.frame_id() + 1);
    if (base_command_.frame_id() > 65535) base_command_.set_frame_id(0);

    for (int i = 0; i < ACTUATOR_COUNT; i++)
        base_command_.mutable_actuators(i)->set_command_id(base_command_.frame_id());
}

std::vector<double> kinova_mediator_L::get_maximum_joint_pos_limits()
{
    return kinova_constants::joint_position_limits_max_L;
}

std::vector<double> kinova_mediator_L::get_minimum_joint_pos_limits()
{
    return kinova_constants::joint_position_limits_min_L;
}

std::vector<double> kinova_mediator_L::get_joint_position_thresholds()
{
    return kinova_constants::joint_position_thresholds_L;
}

std::vector<double> kinova_mediator_L::get_joint_velocity_limits()
{
    return kinova_constants::joint_velocity_limits_L;
}

std::vector<double> kinova_mediator_L::get_joint_acceleration_limits()
{
    assert(ACTUATOR_COUNT == kinova_constants::joint_acceleration_limits_L.size());
    return kinova_constants::joint_acceleration_limits_L;
}

std::vector<double> kinova_mediator_L::get_joint_torque_limits()
{
    return kinova_constants::joint_torque_limits_L;
}

std::vector<double> kinova_mediator_L::get_joint_stopping_torque_limits()
{
    assert(ACTUATOR_COUNT == kinova_constants::joint_stopping_torque_limits_L.size());
    return kinova_constants::joint_stopping_torque_limits_L;
}

std::vector<double> kinova_mediator_L::get_joint_inertia()
{
    return kinova_constants::joint_inertia_L;
}

std::vector<double> kinova_mediator_L::get_joint_offsets()
{
    return kinova_constants::joint_offsets_L;
}

KDL::Twist kinova_mediator_L::get_root_acceleration()
{
    return root_acc_;
}

KDL::Chain kinova_mediator_L::get_robot_model() 
{
    return kinova_chain_; 
}

//Extract youBot model from URDF file
int kinova_mediator_L::get_model_from_urdf()
{
    if (!kinova_urdf_model_.initFile(kinova_constants::urdf_path_L))
    {
        printf("ERROR: Failed to parse urdf robot model \n");
        return -1;
    }

    //Extract KDL tree from the URDF file
    if (!kdl_parser::treeFromUrdfModel(kinova_urdf_model_, kinova_tree_))
    {
        printf("ERROR: Failed to construct kdl tree \n");
        return -1;
    }

    //Extract KDL chain from KDL tree
    kinova_tree_.getChain(kinova_constants::root_name_L, 
                          kinova_constants::tooltip_name_L, 
                          kinova_chain_);
    return 0;
}

int kinova_mediator_L::get_robot_ID()
{
    return kinova_id;
}

bool kinova_mediator_L::is_initialized()
{
    return is_initialized_;
}

// Initialize variables and calibrate the manipulator: 
void kinova_mediator_L::initialize(const int robot_model,
                                 const int robot_environment,
                                 const int id)
{
    kinova_model_       = robot_model;
    kinova_id           = KINOVA_GEN3_1;
    kinova_environment_ = robot_environment;
    kinova_chain_       = KDL::Chain();

    // Reset Flags
    is_initialized_   = false;
    add_offsets_      = false;
    int parser_result = 0;

    // If the real robot is controlled, settup the connection
    if (kinova_environment_ != kinova_environment::SIMULATION)
    {

        // Create API error-callback and objects
        // Connect all ports for real control
        auto error_callback = [](Kinova::Api::KError err){ cout << "_________ callback error _________" << err.toString(); };
        this->transport_ = std::make_shared<Kinova::Api::TransportClientTcp>();
        this->router_ = std::make_shared<Kinova::Api::RouterClient>(transport_.get(), error_callback);
        transport_->connect(IP_ADDRESS_1, PORT);
 

        this->transport_real_time_ = std::make_shared<Kinova::Api::TransportClientUdp>();
        this->router_real_time_ = std::make_shared<Kinova::Api::RouterClient>(transport_real_time_.get(), error_callback);
        transport_real_time_->connect(IP_ADDRESS_1, PORT_REAL_TIME);

        // Set session data connection information
        auto create_session_info = Kinova::Api::Session::CreateSessionInfo();
        create_session_info.set_username("admin");
        create_session_info.set_password("admin");
        create_session_info.set_session_inactivity_timeout(200);   // (milliseconds)
        create_session_info.set_connection_inactivity_timeout(200); // (milliseconds)

        // Session manager service wrapper
        this->session_manager_ = std::make_shared<Kinova::Api::SessionManager>(router_.get());
        session_manager_->CreateSession(create_session_info);

        this->session_manager_real_time_ = std::make_shared<Kinova::Api::SessionManager>(router_real_time_.get());
        session_manager_real_time_->CreateSession(create_session_info);

        // Create services
        this->base_ = std::make_shared<Kinova::Api::Base::BaseClient>(router_.get());
        this->base_cyclic_ = std::make_shared< Kinova::Api::BaseCyclic::BaseCyclicClient>(router_real_time_.get());
        this->actuator_config_ = std::make_shared< Kinova::Api::ActuatorConfig::ActuatorConfigClient>(router_.get());

        // std::cout << "Kinova sessions created" << std::endl;

        // Clearing faults
        try
        {
            base_->ClearFaults();
        }
        catch(...)
        {
            std::cout << "Unable to clear robot faults" << std::endl;
            return;
        }

        // Initializing actuators
        try
        {
            // Set the robot in low-level servoing mode
            servoing_mode_.set_servoing_mode(Kinova::Api::Base::ServoingMode::LOW_LEVEL_SERVOING);
            base_->SetServoingMode(servoing_mode_);

            // Wait
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // Get the initial state
            base_feedback_ = base_cyclic_->RefreshFeedback();

            // Initialize each actuator to their current position
            for (int i = 0; i < ACTUATOR_COUNT; i++)
                base_command_.add_actuators()->set_position(base_feedback_.actuators(i).position());

            // Send a first command (time frame) -> position command in this case
            base_feedback_ = base_cyclic_->Refresh(base_command_, 0);
        }
        catch (Kinova::Api::KDetailedException& ex)
        {
            std::cout << "API error: " << ex.what() << std::endl;
            return;
        }
        catch (std::runtime_error& ex2)
        {
            std::cout << "Run-time Error: " << ex2.what() << std::endl;
            return;
        }
        catch(...)
        {
            std::cout << "Unknown error" << std::endl;
            return;
        }

        // Set connection flag
        connection_established_ = true;
    }
    else
    {
        // Initialize each actuator to their current position
        for (int i = 0; i < ACTUATOR_COUNT; i++)
        {
            base_feedback_.add_actuators();
            base_command_.add_actuators()->set_position(base_feedback_.actuators(i).position());
        }
        // Set connection flag
        connection_established_ = true;
    }

    //Extract Kinova model from the URDF file
    if (kinova_model_ == kinova_model::URDF) parser_result = get_model_from_urdf();
    else
    {
        printf("Unsupported model\n");
        return;
    }

    if (parser_result != 0 || !connection_established_)  printf("Cannot create Kinova model! \n");
    else
    {
        // Set initialization flag for the user
        is_initialized_ = true;
        // printf("Kinova initialized successfully! \n\n");
    } 
}

void kinova_mediator_L::deinitialize()
{
    if (kinova_environment_ != kinova_environment::SIMULATION)
    {
        // Necessary to avoid hard restart of the arm for the next control trial
        stop_robot_motion();

        // Close API session
        session_manager_->CloseSession();
        session_manager_real_time_->CloseSession();

        // Deactivate the router and cleanly disconnect from the transport object
        router_->SetActivationStatus(false);
        transport_->disconnect();
        router_real_time_->SetActivationStatus(false);
        transport_real_time_->disconnect();
    }

    is_initialized_ = false;
    printf("Robot deinitialized! \n\n\n");
}
