
#include <ros/ros.h>
#include <iostream>

#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <thread>
#include <atomic>

#include <kortex_driver/Base_ClearFaults.h>
#include <kortex_driver/CartesianSpeed.h>
#include <kortex_driver/BaseCyclic_Feedback.h>
#include <kortex_driver/ReadAction.h>
#include <kortex_driver/ExecuteAction.h>
#include <kortex_driver/SetCartesianReferenceFrame.h>
#include <kortex_driver/CartesianReferenceFrame.h>
#include <kortex_driver/SendGripperCommand.h>
#include <kortex_driver/GripperMode.h>
#include <kortex_driver/ActionNotification.h>
#include <kortex_driver/ActionEvent.h>
#include <kortex_driver/OnNotificationActionTopic.h>
#include <kortex_driver/ExecuteWaypointTrajectory.h>
#include <kortex_driver/ValidateWaypointList.h>
#include <kortex_driver/GetProductConfiguration.h>
#include <kortex_driver/ModelId.h>
#include <kortex_driver/ActionType.h>
#include <kortex_driver/GetMeasuredCartesianPose.h>


#define RETRACT_ACTION_IDENTIFIER 1
#define HOME_ACTION_IDENTIFIER 2

bool all_notifs_succeeded = true;

std::atomic<int> last_action_notification_event{0};
std::atomic<int> last_action_notification_id{0};

void notification_callback(const kortex_driver::ActionNotification& notif)
{
  last_action_notification_event = notif.action_event;
  last_action_notification_id = notif.handle.identifier;
}

bool wait_for_action_end_or_abort()
{
  while (ros::ok())
  {
    if (last_action_notification_event.load() == kortex_driver::ActionEvent::ACTION_END)
    {
      ROS_INFO("Received ACTION_END notification for action %d", last_action_notification_id.load());
      return true;
    }
    else if (last_action_notification_event.load() == kortex_driver::ActionEvent::ACTION_ABORT)
    {
      ROS_INFO("Received ACTION_ABORT notification for action %d", last_action_notification_id.load());
      all_notifs_succeeded = false;
      return false;
    }
    ros::spinOnce();
  }
  return false;
}


bool example_clear_faults(ros::NodeHandle n, const std::string &robot_name)
{
  ros::ServiceClient service_client_clear_faults = n.serviceClient<kortex_driver::Base_ClearFaults>("/" + robot_name + "/base/clear_faults");
  kortex_driver::Base_ClearFaults service_clear_faults;

  // Clear the faults
  if (!service_client_clear_faults.call(service_clear_faults))
  {
    std::string error_string = "Failed to clear the faults";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // Wait a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return true;
}


bool example_retract_the_robot(ros::NodeHandle n, const std::string &robot_name)
{
  ros::ServiceClient service_client_read_action = n.serviceClient<kortex_driver::ReadAction>("/" + robot_name + "/base/read_action");
  kortex_driver::ReadAction service_read_action;
  last_action_notification_event = 0;

  // The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
  service_read_action.request.input.identifier = RETRACT_ACTION_IDENTIFIER;

  if (!service_client_read_action.call(service_read_action))
  {
    std::string error_string = "Failed to call ReadAction";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // We can now execute the Action that we read 
  ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;

  service_execute_action.request.input = service_read_action.response.output;
  
  if (service_client_execute_action.call(service_execute_action))
  {
    ROS_INFO("The retract position action was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteAction";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  return wait_for_action_end_or_abort();
}

bool example_home_the_robot(ros::NodeHandle n, const std::string &robot_name)
{
  ros::ServiceClient service_client_read_action = n.serviceClient<kortex_driver::ReadAction>("/" + robot_name + "/base/read_action");
  kortex_driver::ReadAction service_read_action;
  last_action_notification_event = 0;

  // The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
  service_read_action.request.input.identifier = HOME_ACTION_IDENTIFIER;

  if (!service_client_read_action.call(service_read_action))
  {
    std::string error_string = "Failed to call ReadAction";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // We can now execute the Action that we read 
  ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;

  service_execute_action.request.input = service_read_action.response.output;
  
  if (service_client_execute_action.call(service_execute_action))
  {
    ROS_INFO("The Home position action was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteAction";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  return wait_for_action_end_or_abort();
}

bool example_cartesian_action(ros::NodeHandle n, const std::string &robot_name)
{
  kortex_driver::ConstrainedPose my_constrained_pose;
  kortex_driver::CartesianSpeed my_cartesian_speed;

  my_cartesian_speed.translation = 0.1f;
  my_cartesian_speed.orientation = 15.0f;

  my_constrained_pose.constraint.oneof_type.speed.push_back(my_cartesian_speed);

  my_constrained_pose.target_pose.x = 0.374f;
  my_constrained_pose.target_pose.y = 0.081f;
  my_constrained_pose.target_pose.z = 0.450f;
  my_constrained_pose.target_pose.theta_x = -57.6f;
  my_constrained_pose.target_pose.theta_y = 91.1f;
  my_constrained_pose.target_pose.theta_z = 2.3f;

  ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;

  service_execute_action.request.input.oneof_action_parameters.reach_pose.push_back(my_constrained_pose);
  service_execute_action.request.input.name = "pose1";
  service_execute_action.request.input.handle.identifier = 1001;
  service_execute_action.request.input.handle.action_type = kortex_driver::ActionType::REACH_POSE;
  
  last_action_notification_event = 0;
  if (service_client_execute_action.call(service_execute_action))
  {
    ROS_INFO("Pose 1 was sent to the robot.");
  }
  else
  {
  
    std::string error_string = "Failed to call ExecuteAction on pose 1";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // Waiting for the pose 1 to end
  wait_for_action_end_or_abort();

  // Change the pose and give it a new identifier
  service_execute_action.request.input.handle.identifier = 1002;
  service_execute_action.request.input.name = "pose2";

  my_constrained_pose.target_pose.z = 0.2f;

  service_execute_action.request.input.oneof_action_parameters.reach_pose.clear();
  service_execute_action.request.input.oneof_action_parameters.reach_pose.push_back(my_constrained_pose);

  last_action_notification_event = 0;
  if (service_client_execute_action.call(service_execute_action))
  {
    ROS_INFO("Pose 2 was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteAction on pose 2";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // Waiting for the pose 2 to end.
  wait_for_action_end_or_abort();

  // Change the pose and give it a new identifier
  service_execute_action.request.input.handle.identifier = 1003;
  service_execute_action.request.input.name = "pose3";
  
  my_constrained_pose.target_pose.x = 0.45f;

  service_execute_action.request.input.oneof_action_parameters.reach_pose.clear();
  service_execute_action.request.input.oneof_action_parameters.reach_pose.push_back(my_constrained_pose);

  last_action_notification_event = 0;
  if (service_client_execute_action.call(service_execute_action))
  {
    ROS_INFO("Pose 3 was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteAction on pose 3";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // Waiting for the pose 3 to end
  wait_for_action_end_or_abort();

  return true;
}

bool example_set_cartesian_reference_frame(ros::NodeHandle n, const std::string &robot_name)
{
  // Initialize the ServiceClient
  ros::ServiceClient service_client_set_cartesian_reference_frame = n.serviceClient<kortex_driver::SetCartesianReferenceFrame>("/" + robot_name + "/control_config/set_cartesian_reference_frame");
  kortex_driver::SetCartesianReferenceFrame service_set_cartesian_reference_frame;

  service_set_cartesian_reference_frame.request.input.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED;
  if (!service_client_set_cartesian_reference_frame.call(service_set_cartesian_reference_frame))
  {
    std::string error_string = "Failed to call SetCartesianReferenceFrame";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // Wait a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  return true;
}


//using namespace std;


int main(int argc, char** argv)
{
	// initializing for ros 
	ros::init(argc, argv, "apimove_test");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

    bool success = true;

	std::string robot_name = "my_right_arm";   //my_gen3
    std::string robot_name_L = "my_left_arm";   //my_gen3

    int degrees_of_freedom = 6;
    int degrees_of_freedom_L = 7;
    bool is_gripper_present = false;
    bool is_gripper_present_L = false;

    // Parameter robot_name
  if (!ros::param::get("~robot_name", robot_name))
  {
    std::string error_string = "Parameter robot_name was not specified, defaulting to " + robot_name + " as namespace";
    ROS_WARN("%s", error_string.c_str());
  }
  else 
  {
    std::string error_string = "Using robot_name " + robot_name + " as namespace";
    ROS_INFO("%s", error_string.c_str());
  }

  // Parameter degrees_of_freedom
  if (!ros::param::get("/" + robot_name + "/degrees_of_freedom", degrees_of_freedom))
  {
    std::string error_string = "Parameter /" + robot_name + "/degrees_of_freedom was not specified, defaulting to " + std::to_string(degrees_of_freedom) + " as degrees of freedom";
    ROS_WARN("%s", error_string.c_str());
  }
  else 
  {
    std::string error_string = "Using degrees_of_freedom " + std::to_string(degrees_of_freedom) + " as degrees_of_freedom";
    ROS_INFO("%s", error_string.c_str());
  }

  // Parameter is_gripper_present
  if (!ros::param::get("/" + robot_name + "/is_gripper_present", is_gripper_present))
  {
    std::string error_string = "Parameter /" + robot_name + "/is_gripper_present was not specified, defaulting to " + std::to_string(is_gripper_present);
    ROS_WARN("%s", error_string.c_str());
  }
  else 
  {
    std::string error_string = "Using is_gripper_present " + std::to_string(is_gripper_present);
    ROS_INFO("%s", error_string.c_str());
  }
  
    // Parameter robot_name  // left_arm
  if (!ros::param::get("~robot_name", robot_name_L))
  {
    std::string error_string = "Parameter robot_name was not specified, defaulting to " + robot_name_L + " as namespace";
    ROS_WARN("%s", error_string.c_str());
  }
  else 
  {
    std::string error_string = "Using robot_name " + robot_name_L + " as namespace";
    ROS_INFO("%s", error_string.c_str());
  }

  // Parameter degrees_of_freedom
  if (!ros::param::get("/" + robot_name_L + "/degrees_of_freedom", degrees_of_freedom_L))
  {
    std::string error_string = "Parameter /" + robot_name_L + "/degrees_of_freedom was not specified, defaulting to " + std::to_string(degrees_of_freedom) + " as degrees of freedom";
    ROS_WARN("%s", error_string.c_str());
  }
  else 
  {
    std::string error_string = "Using degrees_of_freedom " + std::to_string(degrees_of_freedom_L) + " as degrees_of_freedom";
    ROS_INFO("%s", error_string.c_str());
  }

  // Parameter is_gripper_present
  if (!ros::param::get("/" + robot_name_L + "/is_gripper_present", is_gripper_present_L))
  {
    std::string error_string = "Parameter /" + robot_name_L + "/is_gripper_present was not specified, defaulting to " + std::to_string(is_gripper_present);
    ROS_WARN("%s", error_string.c_str());
  }
  else 
  {
    std::string error_string = "Using is_gripper_present " + std::to_string(is_gripper_present_L);
    ROS_INFO("%s", error_string.c_str());
  }
  //*******************************************************************************

  // Subscribe to the Action Topic
  ros::Subscriber sub = node_handle.subscribe("/" + robot_name  + "/action_topic", 1000, notification_callback);
  ros::Subscriber sub_L = node_handle.subscribe("/" + robot_name_L  + "/action_topic", 1000, notification_callback);

  // We need to call this service to activate the Action Notification on the kortex_driver node.
  ros::ServiceClient service_client_activate_notif = node_handle.serviceClient<kortex_driver::OnNotificationActionTopic>("/" + robot_name + "/base/activate_publishing_of_action_topic");
  //ros::ServiceClient service_client_activate_notif_L = node_handle.serviceClient<kortex_driver::OnNotificationActionTopic>("/" + robot_name_L + "/base/activate_publishing_of_action_topic");
  kortex_driver::OnNotificationActionTopic service_activate_notif;
  if (service_client_activate_notif.call(service_activate_notif))
  {
    ROS_INFO("Action notification activated!");
  }
  else 
  {
    std::string error_string = "Action notification publication failed";
    ROS_ERROR("%s", error_string.c_str());
    success = false;
  }

 

  //*******************************************************************************
  // Make sure to clear the robot's faults else it won't move if it's already in fault
  success &= example_clear_faults(node_handle, robot_name);
  success &= example_clear_faults(node_handle, robot_name_L);
  //*******************************************************************************

  //*************    Motion Control  ***************************************************
  
  // Set the reference frame to "Mixed"
  success &= example_set_cartesian_reference_frame(node_handle, robot_name);
  success &= example_set_cartesian_reference_frame(node_handle, robot_name_L);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));


  success &= example_retract_the_robot(node_handle, robot_name);
  success &= example_home_the_robot(node_handle, robot_name_L);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  success &= example_home_the_robot(node_handle, robot_name);
  success &= example_retract_the_robot(node_handle, robot_name_L);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  success &= example_cartesian_action(node_handle, robot_name);
  success &= example_home_the_robot(node_handle, robot_name_L);
  success &= all_notifs_succeeded;

  //*****Position Move Test*********************************
  // Example of cartesian pose
  success &= example_home_the_robot(node_handle, robot_name);
  success &= example_retract_the_robot(node_handle, robot_name_L);
  

  // Report success for testing purposes
  //ros::param::set("/kortex_examples_test_results/full_arm_movement_cpp", success);
  
  return success ? 0 : 1;

}
