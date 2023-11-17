/*
Author(s): Djordje Vukcevic, Sven Schneider
Institute: Hochschule Bonn-Rhein-Sieg
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
#include <kinova_mediator_L.hpp>
#include <kinova_mediator_R.hpp>
#include <chainfdsolver.hpp>
// #include <state_specification.hpp>
// #include <dynamics_controller.hpp>
#include <chainidsolver_recursive_newton_euler.hpp>
#include <chainexternalwrenchestimator.hpp>
// #include <safety_controller.hpp>
// #include <finite_state_machine.hpp>
// #include <motion_profile.hpp>

#define IP_ADDRESS_2 "192.168.1.20"  // right
#define PORT 10000

enum desired_pose
{
    CANDLE       = 0,
    HOME         = 1,
    RETRACT      = 2,
    PACKAGING    = 3
};

enum path_types
{
    SINE_PATH = 0,
    STEP_PATH = 1,
    INF_SIGN_PATH = 2
};

// Waiting time during actions
constexpr auto TIMEOUT_DURATION      = std::chrono::seconds{20};
std::chrono::steady_clock::time_point loop_start_time;
std::chrono::duration <double, std::micro> loop_interval{};

const int SECOND                     = 1000000;
const int MILLISECOND                = 1000;
const int JOINTS_R                   = 6;
const int NUMBER_OF_CONSTRAINTS      = 6;
int RATE_HZ                          = 1000; // Hz
int path_type                        = path_types::STEP_PATH;
int desired_pose_id                  = desired_pose::HOME;
int desired_control_mode             = control_mode::TORQUE;
int environment                      = kinova_environment::SIMULATION;
int robot_model_id                   = kinova_model::URDF;
int id                               = robot_id::KINOVA_GEN3_2;

const double time_horizon_amplitude  = 2.5;
double desired_null_space_angle      = 90.0; // Unit degrees
double task_time_limit_sec           = 600.0;

bool log_data                        = false;
bool control_null_space              = false;
bool compensate_gravity              = false;
bool use_mass_alternation            = false;

std::vector<bool> control_dims       = {true, true, true, // Linear
                                        false, false, false}; // Angular

const Eigen::VectorXd max_command         = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) << 20.0, 20.0, 20.0, 10.0, 10.0, 10.0).finished();

// Create an event listener that will set the promise action event to the exit value
// Will set promise to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(Kinova::Api::Base::ActionNotification)> create_event_listener_by_promise(std::promise<Kinova::Api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise] (Kinova::Api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
            case Kinova::Api::Base::ActionEvent::ACTION_END:
            case Kinova::Api::Base::ActionEvent::ACTION_ABORT:
                finish_promise.set_value(action_event);
                break;
            default:
                break;
        }
    };
}

/*****************************
 * Example related function *
 *****************************/
int64_t GetTickUs()
{
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
}

// Define the callback function used in Refresh_callback
auto lambda_fct_callback = [](const Kinova::Api::Error &err, const Kinova::Api::BaseCyclic::Feedback data)
{
    // We are printing the data of the moving actuator just for the example purpose,
    // avoid this in a real-time loop
    std::string serialized_data;
    google::protobuf::util::MessageToJsonString(data.actuators(6), &serialized_data);
    std::cout << serialized_data << std::endl << std::endl;
};

//Make sure that the control loop runs exactly with the specified frequency
int enforce_loop_frequency(const int dt)
{
    loop_interval = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - loop_start_time);

    if (loop_interval < std::chrono::microseconds(dt)) // Loop is sufficiently fast
    {
        while (loop_interval < std::chrono::microseconds(dt - 1))
            loop_interval = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - loop_start_time);

        return 0;
    }
    else return -1; //Loop is too slow
}

void run_test(kinova_mediator_R &robot_driver_2)
{
    RATE_HZ = 1000;
    double total_time_sec = 0.0;
    const int DT_MICRO = SECOND / RATE_HZ;
    const double DT_SEC = 1.0 / static_cast<double>(RATE_HZ);
    int id_solver_result = 0;
    int est_solver_result = 0;
    int return_flag = 0;
    int iteration_count = 0;
    // double loop_time = 0.0;
    
    // std::vector<double> my_vector = {0.0, 340.0, 214.0, 0.0, 310.0, 90.0};
    std::vector<double> my_vector = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    KDL::JntArray zero_joint_array_2(6), torque_command_2(6), joint_command_2(6), joint_pos_2(6), joint_vel_2(6), joint_torque_2(6);

    KDL::Chain robot_chain_2 = robot_driver_2.get_robot_model();
    KDL::Wrenches wrenches_2(robot_chain_2.getNrOfSegments(), KDL::Wrench::Zero());
    KDL::Wrench wrench_2;

    for(std::size_t i=0; i < my_vector.size(); ++i)
    {
        joint_command_2(i) = my_vector[i];
    }

    const double est_gain = 100;
    const double est_filter = 0.5;
    const double est_eps =  0.00001;
    const int est_maxiter = 150;

    // std::shared_ptr<KDL::ChainIdSolver_RNE> id_solver_2 = std::make_shared<KDL::ChainIdSolver_RNE>(robot_chain_2, KDL::Vector(0.0, 0.0, -9.81289), robot_driver_2.get_joint_inertia(), robot_driver_2.get_joint_torque_limits(), true);
    std::shared_ptr<KDL::ChainExternalWrenchEstimator> ft_solver_R = std::make_shared<KDL::ChainExternalWrenchEstimator>(robot_chain_2, KDL::Vector(0.0, 0.0, -9.81289), RATE_HZ, est_gain, est_filter, est_eps, est_maxiter );

    robot_driver_2.get_joint_state(joint_pos_2, joint_vel_2, joint_torque_2);
    est_solver_result = ft_solver_R->setInitialMomentum(joint_pos_2, joint_vel_2);

    printf("Test run started\n");

    // Real-time loop
    while (total_time_sec < task_time_limit_sec)
    {
        loop_start_time = std::chrono::steady_clock::now();
        iteration_count++;
        total_time_sec = iteration_count * DT_SEC;

        robot_driver_2.get_joint_state(joint_pos_2, joint_vel_2, joint_torque_2);
 

        // for(size_t i = 0; i < joint_pos_2.rows(); i++) 
        // {
        //     std::cout << "Joint pos " << i << ": " << joint_pos_2(i) << std::endl;
        // }
        // for(size_t i = 0; i < joint_torque_2.rows(); i++) 
        // {
        //     std::cout << "Joint tor " << i << ": " << joint_torque_2(i) << std::endl;
        // }

        std::cout << "control time : " << total_time_sec << std::endl;

        // Compute dynamics
        
        est_solver_result = ft_solver_R->JntToExtWrench(joint_pos_2, joint_vel_2, joint_torque_2, wrench_2);

        std::cout << "Wrench Esti Force: [" << wrench_2.force.x() << ", " << wrench_2.force.y() << ", " << wrench_2.force.z() << "]" << std::endl;
        std::cout << "Wrench Esti Torque: [" << wrench_2.torque.x() << ", " << wrench_2.torque.y() << ", " << wrench_2.torque.z() << "]" << std::endl;

        // id_solver_result = id_solver_2->CartToJnt(joint_pos_2, zero_joint_array_2, zero_joint_array_2, wrenches_2, torque_command_2);
        // if (id_solver_result != 0)
        // {
        //     robot_driver_2.stop_robot_motion();
        //     printf("Robot stoped: error in dynamics 2\n");
        //     return;
        // }

        // if (iteration_count == 1)
        // {

        //     if (robot_driver_2.set_control_mode(control_mode::TORQUE) == -1)
        //     {
        //         printf("Incorrect control mode 2\n");
        //         return;
        //     }
        // }

        // Set control commands

        return_flag = robot_driver_2.set_joint_velocities(joint_command_2);
        // return_flag = robot_driver_2.set_joint_torques(torque_command_2);
        if (return_flag == -1)
        {
            robot_driver_2.stop_robot_motion();
            printf("Robot stoped: error in control 2\n");
            return;
        }

        enforce_loop_frequency(DT_MICRO);

        // loop_time += std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - loop_start_time).count();
        // if (iteration_count == 2000) 
        // {
        //     printf("%f\n", loop_time / 2000.0);
        //     break;
        // }
    }

    robot_driver_2.stop_robot_motion();
    printf("Task completed\n");
}

int go_to(kinova_mediator_R &robot_driver_2, const int desired_pose_)
{
    std::vector<double> configuration_array_R(6, 0.0);
    // Angle value are in units of degree
    switch (desired_pose_)
    {
        case desired_pose::CANDLE:
            configuration_array_R = std::vector<double> {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            break;
        case desired_pose::PACKAGING:
            configuration_array_R = std::vector<double> {0.0, 330.0, 214.0, 0.0, 115.0, 270.0};
            break;
        case desired_pose::RETRACT:
            configuration_array_R = std::vector<double> {0.0, 340.0, 214.0, 0.0, 310.0, 90.0};
            break;       
        default:    // home
            configuration_array_R = std::vector<double> {0.0, 15.0, 230.0, 0.0, 55.0, 90.0};
            break;
    }

    if (environment != kinova_environment::SIMULATION)
    {
        auto error_callback = [](Kinova::Api::KError err){ cout << "_________ callback error _________" << err.toString(); };

        // Kinova 2 Create API objects (right)
        auto transport_2 = new Kinova::Api::TransportClientTcp();
        auto router_2 = new Kinova::Api::RouterClient(transport_2, error_callback);
        transport_2->connect(IP_ADDRESS_2, PORT);

        // Set session data connection information
        auto create_session_info_2 = Kinova::Api::Session::CreateSessionInfo();
        create_session_info_2.set_username("admin");
        create_session_info_2.set_password("admin");
        create_session_info_2.set_session_inactivity_timeout(200);   // (milliseconds)
        create_session_info_2.set_connection_inactivity_timeout(200); // (milliseconds)
        // Session manager service wrapper
        auto session_manager_2 = new Kinova::Api::SessionManager(router_2);
        session_manager_2->CreateSession(create_session_info_2);
        // Create services
        auto base_2 = new Kinova::Api::Base::BaseClient(router_2);

        // Make sure the arm is in Single Level Servoing before executing an Action
        auto servoingMode = Kinova::Api::Base::ServoingModeInformation();
        servoingMode.set_servoing_mode(Kinova::Api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
        base_2->SetServoingMode(servoingMode);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        auto constrained_joint_angles_R = Kinova::Api::Base::ConstrainedJointAngles();
        auto joint_angles_R = constrained_joint_angles_R.mutable_joint_angles();
        auto actuator_count_R = base_2->GetActuatorCount();

        for (size_t i = 0; i < actuator_count_R.count(); ++i) 
        {
            auto joint_angle_R = joint_angles_R->add_joint_angles();
            joint_angle_R->set_joint_identifier(i);
            joint_angle_R->set_value(configuration_array_R[i]);
        }

        // Connect to notification action topic (Promise alternative)
        // See cartesian examples for Reference alternative
        std::promise<Kinova::Api::Base::ActionEvent> finish_promise_2;
        auto finish_future_2 = finish_promise_2.get_future();
        auto promise_notification_handle_2 = base_2->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise_2),
            Kinova::Api::Common::NotificationOptions()
        );

        // std::cout << "Reaching joint angles..." << std::endl;
        base_2->PlayJointTrajectory(constrained_joint_angles_R);

        // Wait for future value from promise (Promise alternative)
        // See cartesian examples for Reference alternative
        const auto status_2 = finish_future_2.wait_for(TIMEOUT_DURATION);
        base_2->Unsubscribe(promise_notification_handle_2);

        if (status_2 != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait" << std::endl;
            std::cout << "Can't reach safe position, exiting" << std::endl;

            // Close API session
            session_manager_2->CloseSession();

            // Deactivate the router and cleanly disconnect from the transport object
              router_2->SetActivationStatus(false);
            transport_2->disconnect();

            // Destroy the API
            delete base_2;
            delete session_manager_2;
            delete router_2;
            delete transport_2;
            return -1;
        }

        const auto promise_event_2 = finish_future_2.get();

        // std::cout << "Joint angles reached" << std::endl;
        // std::cout << "Promise value : " << Kinova::Api::Base::ActionEvent_Name(promise_event) << std::endl;

        // Close API session
        session_manager_2->CloseSession();

        // Deactivate the router and cleanly disconnect from the transport object
        router_2->SetActivationStatus(false);

        transport_2->disconnect();

        // Destroy the API
        delete base_2;
        delete session_manager_2;
        delete router_2;
        delete transport_2;

        printf("High-Level Control Completed\n");
    }

    else
    {
        robot_driver_2.initialize(robot_model_id, environment, robot_id::KINOVA_GEN3_2);
        if (!robot_driver_2.is_initialized())
        {
            printf("Robot 2 (right) is not initialized\n");
            return -1;
        }

        if (robot_driver_2.set_control_mode(control_mode::POSITION) == -1)
        {
            printf("Incorrect control mode 2\n");
            return -1;
        }

        KDL::JntArray config_R(JOINTS_R);
        for (int i = 0; i < JOINTS_R; i++)
            config_R(i) = DEG_TO_RAD(configuration_array_R[i]);   
        robot_driver_2.set_joint_positions(config_R);
    }
    return 0;
}



int main(int argc, char **argv)
{
    // printf("kinova MAIN Started \n");
    RATE_HZ              = 1000; // Hz
    control_dims         = std::vector<bool>{true, true, true, // Linear
                                             false, false, false}; // Angular
    environment          = kinova_environment::REAL;
    robot_model_id       = kinova_model::URDF;
    desired_pose_id      = desired_pose::RETRACT;
    desired_control_mode = control_mode::VELOCITY;        //TORQUE   //VELOCITY
    task_time_limit_sec  = 20.0;
    compensate_gravity   = true;
    control_null_space   = false;
    use_mass_alternation = false;
    log_data             = false;

    kinova_mediator_R robot_driver_2;
    int return_flag = 0;
    if      (desired_pose_id == desired_pose::HOME)      return_flag = go_to(robot_driver_2, desired_pose::HOME);
    else if (desired_pose_id == desired_pose::CANDLE)    return_flag = go_to(robot_driver_2, desired_pose::CANDLE);
    else if (desired_pose_id == desired_pose::RETRACT)   return_flag = go_to(robot_driver_2, desired_pose::RETRACT);
    else if (desired_pose_id == desired_pose::PACKAGING) return_flag = go_to(robot_driver_2, desired_pose::PACKAGING);
    else return 0;

    if (return_flag != 0) return 0;

    // return_flag = go_to(robot_driver_2, desired_pose::HOME);

    // return_flag = go_to(robot_driver_2, desired_pose::RETRACT);

    // Extract robot model and if not simulation, establish connection with motor drivers
    printf("Robot 2 is initialize Starting...\n");

    if (!robot_driver_2.is_initialized()) 
    {
        robot_driver_2.initialize(robot_model_id, environment, robot_id::KINOVA_GEN3_2);
        printf("Robot 2 is initializing...\n");
    }
    
    if (!robot_driver_2.is_initialized())
    {
        printf("Robot 2 is not initialized\n");
        return 0;
    }


    printf("Robot 2 is initialized...\n");


    run_test(robot_driver_2);

    robot_driver_2.deinitialize();

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    return_flag = go_to(robot_driver_2, desired_pose::HOME);
    return_flag = go_to(robot_driver_2, desired_pose::RETRACT);
    return 0;
}