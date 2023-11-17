/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2018 Kinova inc. All rights reserved.
*
* This software may be modified and distributed
* under the terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/


#include <iostream>
#include <string>
#include <vector>
#include <math.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <tf2/LinearMath/Quaternion.h>
//#include <tf/tf.h>
//#include <tf2/LinearMath/Matrix3x3.h>
//#include <geometry_msgs/WrenchStamped.h>

#include <KDetailedException.h>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

#include "LowPassFilter.hpp"

#include <google/protobuf/util/json_util.h>

//#include "utilities.h"

#define IP_ADDRESS1 "192.168.1.10"
#define PORT1 10000
#define IP_ADDRESS2 "192.168.1.20"
#define PORT2 10000

#define PORT_REAL_TIME1 10001
#define PORT_REAL_TIME2 10001

namespace k_api = Kinova::Api;



class AdmittanceController {
private:
    double mass_; // Admittance mass
    double damping_;  // Admittance Damping
    double stiff_; // Admittance Stiffness


    std::vector<float> velocity_;
    std::vector<float> position_;

public:
    AdmittanceController() : mass_(0.05), damping_(0.2), stiff_(0.00001), velocity_(3, 0.0f), position_(3, 0.0f)
    {
        // velocity_[0] = 0.0;
        // velocity_[1] = 0.0;
        // velocity_[2] = 0.0;

        // position_[0] = 0.0;
        // position_[1] = 0.0;
        // position_[2] = 0.0;

    }  // Set your own parameters

    std::vector<float> calculateAdmitPose(const std::vector<float>& current_pose,
                                        const std::vector<float>& current_vel,
                                        const std::vector<float>& forcetorque,
                                         const double& dt) {
        std::vector<float> new_pose = current_pose;

        // Discrete implementation of M-C-K Admittance model
        // Assuming fixed time step dt = 0.01 seconds

        velocity_[0] = current_vel[0] + (1/mass_)*( forcetorque[0] - damping_* current_vel[0] - stiff_*current_pose[0] ) * dt;
        velocity_[1] = current_vel[1] + (1/mass_)*( forcetorque[1] - damping_* current_vel[1] - stiff_*current_pose[1] ) * dt;
        velocity_[2] = current_vel[2] + (1/mass_)*( forcetorque[2] - damping_* current_vel[2] - stiff_*current_pose[2] ) * dt;        

        // Update pose
        new_pose[0] += velocity_[0] * dt;
        new_pose[1] += velocity_[1] * dt;
        new_pose[2] += velocity_[2] * dt;

        return new_pose;
    }

};




// Waiting time during actions
const auto ACTION_WAITING_TIME = std::chrono::seconds(1);
constexpr auto TIMEOUT_DURATION = std::chrono::seconds{20};

// Create closure to set finished to true after an END or an ABORT
std::function<void(k_api::Base::ActionNotification)> 
check_for_end_or_abort(bool& finished)
{
    return [&finished](k_api::Base::ActionNotification notification)
    {
        std::cout << "EVENT : " << k_api::Base::ActionEvent_Name(notification.action_event()) << std::endl;

        // The action is finished when we receive a END or ABORT event
        switch(notification.action_event())
        {
        case k_api::Base::ActionEvent::ACTION_ABORT:
        case k_api::Base::ActionEvent::ACTION_END:
            finished = true;
            break;
        default:
            break;
        }
    };
}

std::function<void(k_api::Base::ActionNotification)>
create_event_listener_by_ref(k_api::Base::ActionEvent& returnAction)
{
    return [&returnAction](k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
        case k_api::Base::ActionEvent::ACTION_END:
        case k_api::Base::ActionEvent::ACTION_ABORT:
            returnAction = action_event;
            break;
        default:
            break;
        }
    };
}

void printException(k_api::KDetailedException& ex)
{
    // You can print the error informations and error codes
    auto error_info = ex.getErrorInfo().getError();
    std::cout << "KDetailedoption detected what:  " << ex.what() << std::endl;
    
    std::cout << "KError error_code: " << error_info.error_code() << std::endl;
    std::cout << "KError sub_code: " << error_info.error_sub_code() << std::endl;
    std::cout << "KError sub_string: " << error_info.error_sub_string() << std::endl;

    // Error codes by themselves are not very verbose if you don't see their corresponding enum value
    // You can use google::protobuf helpers to get the string enum element for every error code and sub-code 
    std::cout << "Error code string equivalent: " << k_api::ErrorCodes_Name(k_api::ErrorCodes(error_info.error_code())) << std::endl;
    std::cout << "Error sub-code string equivalent: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(error_info.error_sub_code())) << std::endl;
}

std::vector<float> get_measured_joint_pose(k_api::Base::BaseClient* base)
{
    k_api::Base::JointAngles input_joint_angles;

    int actuator_count = base->GetActuatorCount().count();

    std::vector<float> current_tooljoint(actuator_count, 0.0f);

    input_joint_angles = base->GetMeasuredJointAngles();

    try
    {
        
        for(int i = 0; i < actuator_count; i++)
        {
            current_tooljoint[i] = input_joint_angles.joint_angles(i).value();
        }
    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "Unable to get joint angles" << std::endl;
        printException(ex);
        
        return current_tooljoint;
    }

    return current_tooljoint;
}

std::vector<float> get_measured_toolpose(k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{
    k_api::BaseCyclic::Feedback base_feedback;

    std::vector<float> current_toolpose(6, 0.0f);

    try
    {
        base_feedback = base_cyclic->RefreshFeedback();

        current_toolpose[0] = base_feedback.base().tool_pose_x();
        current_toolpose[1] = base_feedback.base().tool_pose_y();
        current_toolpose[2] = base_feedback.base().tool_pose_z();
        current_toolpose[3] = base_feedback.base().tool_pose_theta_x();
        current_toolpose[4] = base_feedback.base().tool_pose_theta_y();
        current_toolpose[5] = base_feedback.base().tool_pose_theta_z();      
    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "measure toolPose Error" << std::endl;
        
        return current_toolpose;
    }

    return current_toolpose;
}

std::vector<float> get_measured_toolvel(k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{
    k_api::BaseCyclic::Feedback base_feedback;

    std::vector<float> current_toolvel(6, 0.0f);

    try
    {
        base_feedback = base_cyclic->RefreshFeedback();
        
        current_toolvel[0] = base_feedback.base().tool_twist_linear_x();
        current_toolvel[1] = base_feedback.base().tool_twist_linear_y();
        current_toolvel[2] = base_feedback.base().tool_twist_linear_z();
        current_toolvel[3] = base_feedback.base().tool_twist_angular_x();
        current_toolvel[4] = base_feedback.base().tool_twist_angular_y();
        current_toolvel[5] = base_feedback.base().tool_twist_angular_z();     
    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "measure tool Velocity Error" << std::endl;
        
        return current_toolvel;
    }

    return current_toolvel;
}


bool example_forward_kinematics(k_api::Base::BaseClient* base)
{
    // Current arm's joint angles
    k_api::Base::JointAngles input_joint_angles;
    try
    {
        std::cout << "Getting Angles for every joint..." << std::endl;
        input_joint_angles = base->GetMeasuredJointAngles();
    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "Unable to get joint angles" << std::endl;
        printException(ex);
        return false;
    }

    std::cout << "Joint ID : Joint Angle" << std::endl;
    for (auto joint_angle : input_joint_angles.joint_angles()) 
    {
        std::cout << joint_angle.joint_identifier() << " : " << joint_angle.value() << std::endl;
    }
    std::cout << std::endl;

    // Computing Foward Kinematics (Angle -> cartesian convert) from arm's current joint angles
    k_api::Base::Pose pose;
    try
    {
        std::cout << "Computing Foward Kinematics using joint angles..." << std::endl;
        pose = base->ComputeForwardKinematics(input_joint_angles);
    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "Unable to compute forward kinematics" << std::endl;
        printException(ex);
        return false;
    }

    std::cout << "Pose calculated : " << std::endl;
    std::cout << "Coordinate (x, y, z)  : (" << pose.x() << ", " << pose.y() << ", " << pose.z() << ")" << std::endl;
    std::cout << "Theta (theta_x, theta_y, theta_z)  : (" << pose.theta_x() << ", " << pose.theta_y() << ", " << pose.theta_z() << ")" << std::endl << std::endl;

    return true;
}

void move_to_home_position(k_api::Base::BaseClient* base)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to a safe position" << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list()) 
    {
        if (action.name() == "Home") 
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0) 
    {
        std::cout << "Can't reach safe position, exiting" << std::endl;
    } 
    else 
    {
        bool action_finished = false; 
        // Notify of any action topic event
        auto options = k_api::Common::NotificationOptions();
        auto notification_handle = base->OnNotificationActionTopic(
            check_for_end_or_abort(action_finished),
            options
        );

        base->ExecuteActionFromReference(action_handle);

        while(!action_finished)
        { 
            std::this_thread::sleep_for(ACTION_WAITING_TIME);
        }

        base->Unsubscribe(notification_handle);
    }
}

void move_to_retract_position(k_api::Base::BaseClient* base)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to a safe position" << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list()) 
    {
        if (action.name() == "Retract") 
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0) 
    {
        std::cout << "Can't reach safe position, exiting" << std::endl;
    } 
    else 
    {
        bool action_finished = false; 
        // Notify of any action topic event
        auto options = k_api::Common::NotificationOptions();
        auto notification_handle = base->OnNotificationActionTopic(
            check_for_end_or_abort(action_finished),
            options
        );

        base->ExecuteActionFromReference(action_handle);

        while(!action_finished)
        { 
            std::this_thread::sleep_for(ACTION_WAITING_TIME);
        }

        base->Unsubscribe(notification_handle);
    }
}

bool cartesian_initial_movement(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, std::vector<float> target_pose) 
{
    std::cout << "Starting Cartesian movement to target ..." << std::endl;

    auto feedback = base_cyclic->RefreshFeedback();
    auto action = k_api::Base::Action();
    action.set_name("Cartesian initial movement");
    action.set_application_data("");

    auto constrained_pose = action.mutable_reach_pose();
    auto pose = constrained_pose->mutable_target_pose();
    pose->set_x(target_pose[0]);                // x (meters)
    pose->set_y(target_pose[1]);          // y (meters)
    pose->set_z(target_pose[2]);          // z (meters)
    pose->set_theta_x(target_pose[3]);    // theta x (degrees)
    pose->set_theta_y(target_pose[4]);    // theta y (degrees)
    pose->set_theta_z(target_pose[5]);    // theta z (degrees)

    // Connect to notification action topic
    // (Reference alternative)
    // See angular examples for Promise alternative
    k_api::Base::ActionEvent event = k_api::Base::ActionEvent::UNSPECIFIED_ACTION_EVENT;
    auto reference_notification_handle = base->OnNotificationActionTopic(
        create_event_listener_by_ref(event),
        k_api::Common::NotificationOptions()
    );

    std::cout << "Executing action" << std::endl;
    base->ExecuteAction(action);

    std::cout << "Waiting for movement to finish ..." << std::endl;

    // Wait for reference value to be set
    // (Reference alternative)
    // See angular examples for Promise alternative
    // Set a timeout after 20s of wait
    const auto timeout = std::chrono::system_clock::now() + TIMEOUT_DURATION;
    while(event == k_api::Base::ActionEvent::UNSPECIFIED_ACTION_EVENT &&
        std::chrono::system_clock::now() < timeout)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    base->Unsubscribe(reference_notification_handle);

    if(event == k_api::Base::ActionEvent::UNSPECIFIED_ACTION_EVENT)
    {
        std::cout << "Timeout on action notification wait" << std::endl;
        return false;
    }

    std::cout << "Cartesian movement completed" << std::endl;
    std::cout << "Reference value : " << k_api::Base::ActionEvent_Name(event) << std::endl;

    return true;
}

bool cartesian_action_movement(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, std::vector<float> dP) 
{
    std::cout << "Starting Cartesian action movement ..." << std::endl;

    auto feedback = base_cyclic->RefreshFeedback();
    auto action = k_api::Base::Action();
    action.set_name("Cartesian action movement");
    action.set_application_data("");

    auto constrained_pose = action.mutable_reach_pose();
    auto pose = constrained_pose->mutable_target_pose();
    pose->set_x(feedback.base().tool_pose_x() + dP[0]);                // x (meters)
    pose->set_y(feedback.base().tool_pose_y() + dP[1]);          // y (meters)
    pose->set_z(feedback.base().tool_pose_z() + dP[2]);          // z (meters)
    pose->set_theta_x(feedback.base().tool_pose_theta_x() + dP[3]);    // theta x (degrees)
    pose->set_theta_y(feedback.base().tool_pose_theta_y() + dP[4]);    // theta y (degrees)
    pose->set_theta_z(feedback.base().tool_pose_theta_z() + dP[5]);    // theta z (degrees)

    // Connect to notification action topic
    // (Reference alternative)
    // See angular examples for Promise alternative
    k_api::Base::ActionEvent event = k_api::Base::ActionEvent::UNSPECIFIED_ACTION_EVENT;
    auto reference_notification_handle = base->OnNotificationActionTopic(
        create_event_listener_by_ref(event),
        k_api::Common::NotificationOptions()
    );

    std::cout << "Executing action" << std::endl;
    base->ExecuteAction(action);

    std::cout << "Waiting for movement to finish ..." << std::endl;

    // Wait for reference value to be set
    // (Reference alternative)
    // See angular examples for Promise alternative
    // Set a timeout after 20s of wait
    const auto timeout = std::chrono::system_clock::now() + TIMEOUT_DURATION;
    while(event == k_api::Base::ActionEvent::UNSPECIFIED_ACTION_EVENT &&
        std::chrono::system_clock::now() < timeout)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    base->Unsubscribe(reference_notification_handle);

    if(event == k_api::Base::ActionEvent::UNSPECIFIED_ACTION_EVENT)
    {
        std::cout << "Timeout on action notification wait" << std::endl;
        return false;
    }

    std::cout << "Cartesian movement completed" << std::endl;
    std::cout << "Reference value : " << k_api::Base::ActionEvent_Name(event) << std::endl;

    return true;
}

bool run_twist_command(k_api::BaseCyclic::BaseCyclicClient* base_cyclic, k_api::Base::BaseClient* base, std::vector<float> desired_pose, int dt)
{
    auto command = k_api::Base::TwistCommand();
    
    command.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_BASE);
    // command.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);
    command.set_duration(0);  // Unlimited time to execute

    int actuator_count = base->GetActuatorCount().count();
  
    std::vector<float> current_pose = get_measured_toolpose(base_cyclic);
    std::vector<float> des_vel(6);

    for(int i = 0; i < 6; i++)
    {
        des_vel[i] = desired_pose[i] - current_pose[i];
    }

    auto twist = command.mutable_twist();
    twist->set_linear_x(des_vel[0]);
    twist->set_linear_y(des_vel[1]);
    twist->set_linear_z(des_vel[2]);
    twist->set_angular_x(des_vel[3]);
    twist->set_angular_y(des_vel[4]);
    twist->set_angular_z(des_vel[5]);

    base->SendTwistCommand(command);

    // Let time for twist to be executed
    std::this_thread::sleep_for(std::chrono::milliseconds(dt));

    std::cout << "Stopping robot ..." << std::endl;

    // Make movement stop
    //base->Stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(dt));

    return true;
}


int main(int argc, char **argv)
{
    // Create API objects
    auto error_callback1 = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    auto transport1 = new k_api::TransportClientTcp();
    auto router1 = new k_api::RouterClient(transport1, error_callback1);
    transport1->connect(IP_ADDRESS1, PORT1);

    // Real-time
    auto transport_rt1 = new k_api::TransportClientUdp();
    auto router_rt1 = new k_api::RouterClient(transport_rt1, error_callback1);
    transport_rt1->connect(IP_ADDRESS1, PORT_REAL_TIME1);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)


    // Session manager service wrapper
    std::cout << "Creating session Left for communication" << std::endl;
    auto session_manager1 = new k_api::SessionManager(router1);
    session_manager1->CreateSession(create_session_info);
    auto session_manager_rt1 = new k_api::SessionManager(router_rt1);
    session_manager_rt1->CreateSession(create_session_info);
    std::cout << "Session Left created" << std::endl;

    // Create services
    auto base_L = new k_api::Base::BaseClient(router1);
    auto base_cyclic_L = new k_api::BaseCyclic::BaseCyclicClient(router_rt1);

    // controller 
    AdmittanceController controller;

    // ********************************************************************************************************************
    // *******     main code     ******************************************************************************************
    // ********************************************************************************************************************
    // ********************************************************************************************************************
    
    // variable for main

    std::vector<float> current_pose_L(6);
    std::vector<float> current_vel_L(6);
    std::vector<float> current_wrench_L(6);
    std::vector<float> fix_wrench_L(6);
    std::vector<float> diff_wrench_L(6);
    std::vector<float> desired_pose_L(6);
    std::vector<float> current_joint_L(6);
    std::vector<float> desired_joint_L(6);

    double loop_time = 0.02;     //second
    int loop_ms = 20;

    std::vector<float> desired_pose_delta_1 = {0.2f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    std::vector<float> desired_pose_delta_2 = {-0.2f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};


    // example core
    bool success = true;
    
    // example core

    std::thread thread1(move_to_retract_position, base_L);
    thread1.join();

    cartesian_action_movement(base_L, base_cyclic_L, desired_pose_delta_2);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));


    while(true)
    {   

        cartesian_action_movement(base_L, base_cyclic_L, desired_pose_delta_1);
        usleep(2000000); // Sleep for u second
    
        cartesian_action_movement(base_L, base_cyclic_L, desired_pose_delta_2);
        usleep(2000000); // Sleep for u second

    }
    
    // Close API session
    session_manager1->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router1->SetActivationStatus(false);
    transport1->disconnect();

    // Destroy the API
    delete base_L;
    delete base_cyclic_L;
    delete session_manager1;
    delete session_manager_rt1;
    delete router1;
    delete router_rt1;
    delete transport1;
    delete transport_rt1;

    return success? 0: 1;
    
    
}