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

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

//#include "utilities.h"

#define IP_ADDRESS1 "192.168.1.10"
#define PORT1 10000
#define IP_ADDRESS2 "192.168.1.20"
#define PORT2 10000

#define PORT_REAL_TIME1 10001
#define PORT_REAL_TIME2 10001

namespace k_api = Kinova::Api;


// Waiting time during actions
const auto ACTION_WAITING_TIME = std::chrono::seconds(1);

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


bool example_inverse_kinematics(k_api::Base::BaseClient* base)
{
    // get robot's pose (by using forward kinematics)
    k_api::Base::JointAngles input_joint_angles;
    k_api::Base::Pose pose;
    try
    {
        input_joint_angles = base->GetMeasuredJointAngles();
        pose = base->ComputeForwardKinematics(input_joint_angles);
    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "Unable to get current robot pose" << std::endl;
        printException(ex);
        return false;
    }

    // Object containing cartesian coordinates and Angle Guess
    k_api::Base::IKData input_IkData; 
    
    // Fill the IKData Object with the cartesian coordinates that need to be converted
    input_IkData.mutable_cartesian_pose()->set_x(pose.x());
    input_IkData.mutable_cartesian_pose()->set_y(pose.y());
    input_IkData.mutable_cartesian_pose()->set_z(pose.z());
    input_IkData.mutable_cartesian_pose()->set_theta_x(pose.theta_x());
    input_IkData.mutable_cartesian_pose()->set_theta_y(pose.theta_y());
    input_IkData.mutable_cartesian_pose()->set_theta_z(pose.theta_z());

    // Fill the IKData Object with the guessed joint angles
    Kinova::Api::Base::JointAngle* jAngle; 
    for(auto joint_angle : input_joint_angles.joint_angles())
    {
        jAngle = input_IkData.mutable_guess()->add_joint_angles();
        // '- 1' to generate an actual "guess" for current joint angles
        jAngle->set_value(joint_angle.value() - 1); 
    }

    // Computing Inverse Kinematics (cartesian -> Angle convert) from arm's current pose and joint angles guess
    k_api::Base::JointAngles computed_joint_angles;
    try
    {
        std::cout << "Computing Inverse Kinematics using joint angles and pose..." << std::endl;
        computed_joint_angles = base->ComputeInverseKinematics(input_IkData);
    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "Unable to compute inverse kinematics" << std::endl;
        printException(ex);
        return false;
    }

    std::cout << "Joint ID : Joint Angle" << std::endl;
    int joint_identifier = 0;
    for (auto joint_angle : computed_joint_angles.joint_angles()) 
    {
        std::cout << joint_identifier << " : " << joint_angle.value() << std::endl;
        joint_identifier++;
    }

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


int main(int argc, char **argv)
{
    // Create API objects
    auto error_callback1 = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    auto transport1 = new k_api::TransportClientTcp();
    auto router1 = new k_api::RouterClient(transport1, error_callback1);
    transport1->connect(IP_ADDRESS1, PORT1);

    auto error_callback2 = [](k_api::KError err){ cout << "_________ callback error 2 _________" << err.toString(); };
    auto transport2 = new k_api::TransportClientTcp();
    auto router2 = new k_api::RouterClient(transport2, error_callback2);
    transport2->connect(IP_ADDRESS2, PORT2);
    
    // Real-time
    auto transport_rt1 = new k_api::TransportClientUdp();
    auto router_rt1 = new k_api::RouterClient(transport_rt1, error_callback1);
    transport_rt1->connect(IP_ADDRESS1, PORT_REAL_TIME1);

    auto transport_rt2 = new k_api::TransportClientUdp();
    auto router_rt2 = new k_api::RouterClient(transport_rt2, error_callback2);
    transport_rt2->connect(IP_ADDRESS2, PORT_REAL_TIME2);

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

    std::cout << "Creating session Right for communication" << std::endl;
    auto session_manager2 = new k_api::SessionManager(router2);
    session_manager2->CreateSession(create_session_info);
    auto session_manager_rt2 = new k_api::SessionManager(router_rt2);
    session_manager_rt2->CreateSession(create_session_info);
    std::cout << "Session Right created" << std::endl;

    // Create services
    auto base_L = new k_api::Base::BaseClient(router1);
    auto base_cyclic_L = new k_api::BaseCyclic::BaseCyclicClient(router_rt1);

    auto base_R = new k_api::Base::BaseClient(router2);
    auto base_cyclic_R = new k_api::BaseCyclic::BaseCyclicClient(router_rt2);


    // ********************************************************************************************************************
    // *******     main code     ******************************************************************************************
    // ********************************************************************************************************************
    // ********************************************************************************************************************

    // example core
    bool success = true;
    
    // example core

    std::thread thread1(move_to_home_position, base_L);
    std::thread thread2(move_to_home_position, base_R);
    thread1.join();
    thread2.join();

    thread1 = std::thread(move_to_retract_position, base_L);
    thread2 = std::thread(move_to_retract_position, base_R);
    thread1.join();
    thread2.join();

    //success &= move_to_retract_position(base_L);

    while(true)
    {
        
        success &= example_forward_kinematics(base_L);
        success &= example_forward_kinematics(base_R);
        success &= example_inverse_kinematics(base_L);
        success &= example_inverse_kinematics(base_R);
        if(!success)
        {  
        std::cerr << "One of the examples failed!" << std::endl;
        break;
        }

        usleep(5000000); // Sleep for 1 second
    }
    

    // Close API session
    session_manager1->CloseSession();
    session_manager2->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router1->SetActivationStatus(false);
    transport1->disconnect();
    router2->SetActivationStatus(false);
    transport2->disconnect();

    // Destroy the API
    delete base_L;
    delete base_cyclic_L;
    delete base_R;
    delete base_cyclic_R;
    delete session_manager1;
    delete session_manager_rt1;
    delete session_manager2;
    delete session_manager_rt2;
    delete router1;
    delete router_rt1;
    delete router2;
    delete router_rt2;
    delete transport1;
    delete transport_rt1;
    delete transport2;
    delete transport_rt2;

    return success? 0: 1;
    
    
}