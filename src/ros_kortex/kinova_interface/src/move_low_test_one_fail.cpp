
#include <ros/ros.h>
#include <iostream>

#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/WrenchStamped.h>

#include <thread>
#include <atomic>

#include <kortex_driver/Base_ClearFaults.h>
#include <kortex_driver/CartesianSpeed.h>
#include <kortex_driver/BaseCyclic_Feedback.h>
#include <kortex_driver/ReadAction.h>
#include <kortex_driver/ExecuteAction.h>
#include <kortex_driver/MoveToPosition.h>
#include <kortex_driver/SetCartesianReferenceFrame.h>
#include <kortex_driver/CartesianReferenceFrame.h>
#include <kortex_driver/ActionNotification.h>
#include <kortex_driver/ActionEvent.h>
#include <kortex_driver/OnNotificationActionTopic.h>
#include <kortex_driver/ExecuteWaypointTrajectory.h>
#include <kortex_driver/ValidateWaypointList.h>
#include <kortex_driver/GetProductConfiguration.h>
#include <kortex_driver/ModelId.h>
#include <kortex_driver/ActionType.h>
#include <kortex_driver/GetMeasuredCartesianPose.h>
#include <kortex_driver/ComputeInverseKinematics.h>
#include <kortex_driver/ComputeForwardKinematics.h>

#include <std_msgs/Empty.h>
#include <kortex_driver/ReadAllDevices.h>
#include <kortex_driver/DeviceHandle.h>
#include <kortex_driver/DeviceTypes.h>
#include <kortex_driver/SetDeviceID.h>
#include <kortex_driver/GetControlLoopParameters.h>
#include <kortex_driver/SetControlLoopParameters.h>
#include <kortex_driver/ActuatorConfig_ClearFaults.h>
#include <kortex_driver/ControlLoopSelection.h>
#include <kortex_driver/SetServoingMode.h>
#include <kortex_driver/GetActuatorCount.h>




#define RETRACT_ACTION_IDENTIFIER 1
#define HOME_ACTION_IDENTIFIER 2
#define SINGLE_LEVEL_SERVOING 2
#define LOW_LEVEL_SERVOING 3


// ***************** Class ***********************************************
class ForceTorqueSensor
{
public:
  ForceTorqueSensor()
  {
    force_.x = 0.0;
    force_.y = 0.0;
    force_.z = 0.0;

    torque_.x = 0.0;
    torque_.y = 0.0;
    torque_.z = 0.0;
    sub_ = nh_.subscribe("RFT_FORCE", 1, &ForceTorqueSensor::callback, this);
  }

  geometry_msgs::Vector3 getForce() const { return force_; }
  geometry_msgs::Vector3 getTorque() const { return torque_; }

private:
  void callback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
  {
    force_ = msg->wrench.force;
    torque_ = msg->wrench.torque;
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  geometry_msgs::Vector3 force_;
  geometry_msgs::Vector3 torque_;
};

class AdmittanceController {
private:
    double mass_;  // Mass gain
    double damping_;  // Damping gain
    double spring_;  // Spring gain

    geometry_msgs::Vector3 velocity_;
    geometry_msgs::Vector3 position_;

public:
    AdmittanceController() : mass_(0.03), damping_(0.1), spring_(0.0) 
    {
        velocity_.x = 0.0;
        velocity_.y = 0.0;
        velocity_.z = 0.0;

        position_.x = 0.0;
        position_.y = 0.0;
        position_.z = 0.0;

    }  // Set your own parameters

    geometry_msgs::Pose calculateNewPose(const geometry_msgs::Pose& current_pose,
                                         const geometry_msgs::Vector3& force,
                                         const geometry_msgs::Vector3& torque) {
        geometry_msgs::Pose new_pose = current_pose;

        // Discrete implementation of M-C-K Admittance model
        // Assuming fixed time step dt = 0.01 seconds
        const double dt = 0.01;

        // Compute acceleration using F = ma
        geometry_msgs::Vector3 acceleration;
        acceleration.x = (force.y - damping_ * velocity_.x - spring_ * position_.x) / mass_;
        acceleration.y = (force.x - damping_ * velocity_.y - spring_ * position_.y) / mass_;
        acceleration.z = (-force.z - damping_ * velocity_.z - spring_ * position_.z) / mass_;

        // Update velocity and position
        velocity_.x += dt * acceleration.x;
        velocity_.y += dt * acceleration.y;
        velocity_.z += dt * acceleration.z;

        position_.x += dt * velocity_.x;
        position_.y += dt * velocity_.y;
        position_.z += dt * velocity_.z;

        // Update pose
        new_pose.position.x += position_.x;
        new_pose.position.y += position_.y;
        new_pose.position.z += position_.z;

        return new_pose;
    }
};



// ***************** Function ***********************************************

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

  // The retract Action is used to retract the robot. It cannot be deleted and is always ID #1:
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

//*************************************************************88

geometry_msgs::Pose Get_measred_toolPose(ros::NodeHandle n, const std::string &robot_name)
{
  last_action_notification_event = 0;
  // Get the actual cartesian pose to increment it
  // You can create a subscriber to listen to the base_feedback
  // Here we only need the latest message in the topic though
  auto feedback = ros::topic::waitForMessage<kortex_driver::BaseCyclic_Feedback>("/" + robot_name + "/base_feedback");
  
  geometry_msgs::Pose new_pose;
  // Initialize input

  float current_x = feedback->base.commanded_tool_pose_x;
  float current_y = feedback->base.commanded_tool_pose_y;
  float current_z = feedback->base.commanded_tool_pose_z;
  float current_theta_x = feedback->base.commanded_tool_pose_theta_x;
  float current_theta_y = feedback->base.commanded_tool_pose_theta_y;
  float current_theta_z = feedback->base.commanded_tool_pose_theta_z;
  tf2::Quaternion current_quaternion;
  current_quaternion.setRPY(current_theta_x, current_theta_y, current_theta_z);
  geometry_msgs::Quaternion quaternion_msg = tf2::toMsg(current_quaternion);

  new_pose.orientation = quaternion_msg;
  new_pose.position.x  = current_x;
  new_pose.position.y  = current_y;
  new_pose.position.z  = current_z;

  return new_pose;
} 


std::vector<float> example_inv_kinematics(ros::NodeHandle n, const std::string &robot_name, const std::vector<float> &current_joint, geometry_msgs::Pose pose)
{
  ros::ServiceClient service_client_compute_inverse_kinematics = n.serviceClient<kortex_driver::ComputeInverseKinematics>("/" + robot_name + "/base/compute_inverse_kinematics");
  kortex_driver::ComputeInverseKinematics service_compute_inverse_kinematics;
  
  double roll, pitch, yaw;
  tf2::Matrix3x3(pose.orientation).getRPY(roll, pitch, yaw);

  service_compute_inverse_kinematics.request.input.cartesian_pose.x = pose.position.x;
  service_compute_inverse_kinematics.request.input.cartesian_pose.y = pose.position.y;
  service_compute_inverse_kinematics.request.input.cartesian_pose.z = pose.position.z;
  service_compute_inverse_kinematics.request.input.cartesian_pose.theta_x = roll;
  service_compute_inverse_kinematics.request.input.cartesian_pose.theta_y = pitch;
  service_compute_inverse_kinematics.request.input.cartesian_pose.theta_z = yaw;
  
  std::vector<float> JAngle;
  std::vector<float> Com_angles;


  ros::ServiceClient service_client_get_actuator_count = n.serviceClient<kortex_driver::GetActuatorCount>("/" + robot_name + "/base/get_actuator_count");
  kortex_driver::GetActuatorCount service_get_actuator_count;


  if (service_client_get_actuator_count.call(service_get_actuator_count))
  {
    ROS_INFO("The GetActuatorCount was done.");
  }

  int actuator_count = service_get_actuator_count.response.output.count;


  for(int i = 0; i < actuator_count; i++)
  {
    JAngle[i] = current_joint[i];
    service_compute_inverse_kinematics.request.input.guess[i] = JAngle[i] - 1;
    
  }

  //service_compute_inverse_kinematics.request.input.mutable_guess() = JAngle;
  

  if (service_client_compute_inverse_kinematics.call(service_compute_inverse_kinematics))
  {
    ROS_INFO("The Inv Computation action was sent to the robot.");
  }
  {
    std::string error_string = "Failed to call ComputeInverseKinematics";
    ROS_ERROR("%s", error_string.c_str());
    return Com_angles;
  }

  Com_angles = service_compute_inverse_kinematics.response.output.joint_angles;
  
  return Com_angles;
} 


bool Set_Servo_Mode(ros::NodeHandle n, const std::string &robot_name ,uint32_t modeservo )
{
  

  ros::ServiceClient service_client_set_servoing_mode = n.serviceClient<kortex_driver::SetServoingMode>("/" + robot_name + "/base/set_servoing_mode");
  kortex_driver::SetServoingMode service_set_servoing_mode;


  service_set_servoing_mode.request.input.servoing_mode = modeservo;


  if (service_client_set_servoing_mode.call(service_set_servoing_mode))
  {
    ROS_INFO("The Servoing_mode was changed.");
  }
  {
    std::string error_string = "Failed to call Servoing Change";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  return true;
}

bool Move_desired_joints(ros::NodeHandle n, const std::string &robot_name, const std::vector<float> &desired_joints )
{
  
  std::vector<float> des_joint;

  //kortex_driver::Command act_command

  ros::ServiceClient service_client_get_actuator_count = n.serviceClient<kortex_driver::GetActuatorCount>("/" + robot_name + "/base/get_actuator_count");
  kortex_driver::GetActuatorCount service_get_actuator_count;


  if (service_client_get_actuator_count.call(service_get_actuator_count))
  {
    ROS_INFO("The GetActuatorCount was done.");
  }
  {
    std::string error_string = "Failed to call GetActuatorCount";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }




  int actuator_count = service_get_actuator_count.response.output.count;


  for(int i = 0; i < actuator_count; i++)
  {
    des_joint[i] = desired_joints[i];
    //act_command.input.mutable_actuators(i)->set_position(des_joint[i]);

  }

  last_action_notification_event = 0;
  

  // Waiting for the pose 1 to end
  wait_for_action_end_or_abort();

  return true;
}

bool run_cartesian_action(ros::NodeHandle n, const std::string &robot_name, geometry_msgs::Pose des_pose)
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
	ros::init(argc, argv, "move_lowone");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();


    // Initialize sensor and controller
    ForceTorqueSensor sensor;
    //AdmittanceController controller;

    // FT sensor Read
    geometry_msgs::Vector3 force = sensor.getForce();
    geometry_msgs::Vector3 torque = sensor.getTorque();

    ROS_INFO_STREAM("Force: " << "x: " << force.x << ", y: " << force.y << ", z: " << force.z);
    ROS_INFO_STREAM("Torque: " << "x: " << torque.x << ", y: " << torque.y << ", z: " << torque.z);


  bool success = true;

	std::string robot_name = "my_right_arm";   //my_gen3


    int device_id = 1;
    int degrees_of_freedom = 6;
    bool is_gripper_present = false;

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

  //*******************************************************************************

  // Subscribe to the Action Topic
  ros::Subscriber sub = node_handle.subscribe("/" + robot_name  + "/action_topic", 1000, notification_callback);
  
  // We need to call this service to activate the Action Notification on the kortex_driver node.
  ros::ServiceClient service_client_activate_notif = node_handle.serviceClient<kortex_driver::OnNotificationActionTopic>("/" + robot_name + "/base/activate_publishing_of_action_topic");
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
  //*******************************************************************************
  //Set_Servo_Mode(node_handle, robot_name , SINGLE_LEVEL_SERVOING );   // LOW_LEVEL_SERVOING  // SINGLE_LEVEL_SERVOING
  
  //*************    Motion Control  ***************************************************

  // get current Pose of tool
  geometry_msgs::Pose currentPose;
  currentPose = Get_measred_toolPose(node_handle, robot_name);
  cout<<currentPose<<endl;

  force = sensor.getForce();
  torque = sensor.getTorque();  
  ROS_INFO_STREAM("Force: " << "x: " << force.x << ", y: " << force.y << ", z: " << force.z);
  ROS_INFO_STREAM("Torque: " << "x: " << torque.x << ", y: " << torque.y << ", z: " << torque.z);
  ros::Rate rate(40);
  bool should_exit = false;

  // Set the reference frame to "Mixed"
  success &= example_retract_the_robot(node_handle, robot_name);
  success &= example_set_cartesian_reference_frame(node_handle, robot_name);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  

  success &= example_home_the_robot(node_handle, robot_name);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));


  // Joint Control Test
  
  std::vector<float> Des_J;
  Des_J[0] = 10;
  Des_J[1] = 340;
  Des_J[2] = 214;
  Des_J[3] = 360;
  Des_J[4] = 310;
  Des_J[5] = 90;

  Set_Servo_Mode(node_handle, robot_name , LOW_LEVEL_SERVOING );   // LOW_LEVEL_SERVOING  // SINGLE_LEVEL_SERVOING
  Move_desired_joints(node_handle, robot_name, Des_J );
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));


  Des_J[0] = -10;
  Des_J[1] = 340;
  Des_J[2] = 214;
  Des_J[3] = 360;
  Des_J[4] = 310;
  Des_J[5] = 90;
  Move_desired_joints(node_handle, robot_name, Des_J );
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));



  // Admittance 

  while ( ros::ok() && !should_exit ) 
  {
    currentPose = Get_measred_toolPose(node_handle, robot_name);
    force = sensor.getForce();
    torque = sensor.getTorque();
    ROS_INFO_STREAM("Force: " << "x: " << force.x << ", y: " << force.y << ", z: " << force.z);
    ROS_INFO_STREAM("Torque: " << "x: " << torque.x << ", y: " << torque.y << ", z: " << torque.z); 

    //geometry_msgs::Pose new_pose = controller.calculateNewPose(currentPose, force, torque);


    rate.sleep();
  }
  


  success &= all_notifs_succeeded;



  // Report success for testing purposes
  //ros::param::set("/kortex_examples_test_results/full_arm_movement_cpp", success);
  
  return success ? 0 : 1;

}
