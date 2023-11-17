
#include <ros/ros.h>
#include <iostream>

#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Matrix3x3.h>
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
#include <kortex_driver/GetMeasuredJointAngles.h>
#include <kortex_driver/SendJointSpeedsCommand.h>

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
    double mass_; // Admittance mass
    double damping_;  // Admittance Damping


    geometry_msgs::Vector3 velocity_;
    geometry_msgs::Vector3 position_;

public:
    AdmittanceController() : mass_(0.1), damping_(2)
    {
        velocity_.x = 0.0;
        velocity_.y = 0.0;
        velocity_.z = 0.0;

        position_.x = 0.0;
        position_.y = 0.0;
        position_.z = 0.0;

    }  // Set your own parameters

    std::vector<float> calculateNewPose(const std::vector<float>& current_pose,
                                        const std::vector<float>& current_vel,
                                        const geometry_msgs::Vector3& force,
                                         const geometry_msgs::Vector3& torque,
                                         const double& dt) {
        std::vector<float> new_pose = current_pose;

        // Discrete implementation of M-C-K Admittance model
        // Assuming fixed time step dt = 0.01 seconds

        // velocity_.x = ( force.y*dt + mass_* current_vel[0] )/ (mass_ + damping_ * dt );
        // velocity_.y = ( force.x*dt + mass_* current_vel[1] )/ (mass_ + damping_ * dt );
        // velocity_.z = ( -force.z*dt + mass_* current_vel[2] )/ (mass_ + damping_ * dt );

        velocity_.x = ( force.x*dt + mass_* current_vel[0] )/ (mass_ + damping_ * dt );
        velocity_.y = ( force.y*dt + mass_* current_vel[1] )/ (mass_ + damping_ * dt );
        velocity_.z = ( force.z*dt + mass_* current_vel[2] )/ (mass_ + damping_ * dt );
        

        // Update pose
        // new_pose[0] += position_.x;
        // new_pose[1] += position_.y;
        // new_pose[2] += position_.z;
        new_pose[0] += velocity_.x * dt;
        new_pose[1] += velocity_.y * dt;
        new_pose[2] += velocity_.z * dt;

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

std::vector<float> Get_measured_toolPose(ros::NodeHandle n, const std::string &robot_name)
{
  last_action_notification_event = 0;
  // Get the actual cartesian pose to increment it
  // You can create a subscriber to listen to the base_feedback
  // Here we only need the latest message in the topic though
  auto feedback = ros::topic::waitForMessage<kortex_driver::BaseCyclic_Feedback>("/" + robot_name + "/base_feedback");
  
  std::vector<float> new_pose(6);
  // Initialize input

  float current_x = feedback->base.commanded_tool_pose_x;
  float current_y = feedback->base.commanded_tool_pose_y;
  float current_z = feedback->base.commanded_tool_pose_z;
  float current_theta_x = feedback->base.commanded_tool_pose_theta_x;
  float current_theta_y = feedback->base.commanded_tool_pose_theta_y;
  float current_theta_z = feedback->base.commanded_tool_pose_theta_z;
  // tf2::Quaternion current_quaternion;
  // current_quaternion.setRPY(current_theta_x, current_theta_y, current_theta_z);
  // geometry_msgs::Quaternion quaternion_msg = tf2::toMsg(current_quaternion);

  new_pose[0]  = current_x;
  new_pose[1]  = current_y;
  new_pose[2]  = current_z;
  new_pose[3]  = current_theta_x;
  new_pose[4]  = current_theta_y;
  new_pose[5]  = current_theta_z;

  return new_pose;
} 

std::vector<float> Get_measured_toolVel(ros::NodeHandle n, const std::string &robot_name)
{
  last_action_notification_event = 0;
  // Get the actual cartesian pose to increment it
  // You can create a subscriber to listen to the base_feedback
  // Here we only need the latest message in the topic though
  auto feedback = ros::topic::waitForMessage<kortex_driver::BaseCyclic_Feedback>("/" + robot_name + "/base_feedback");
  
  std::vector<float> new_vel(6);
  // Initialize input

  float current_vx = feedback->base.tool_twist_linear_x;
  float current_vy = feedback->base.tool_twist_linear_y;
  float current_vz = feedback->base.tool_twist_linear_z;
  float current_vtheta_x = feedback->base.tool_twist_angular_x;
  float current_vtheta_y = feedback->base.tool_twist_angular_y;
  float current_vtheta_z = feedback->base.tool_twist_angular_z;
  // tf2::Quaternion current_quaternion;
  // current_quaternion.setRPY(current_theta_x, current_theta_y, current_theta_z);
  // geometry_msgs::Quaternion quaternion_msg = tf2::toMsg(current_quaternion);

  new_vel[0]  = current_vx;
  new_vel[1]  = current_vy;
  new_vel[2]  = current_vz;
  new_vel[3]  = current_vtheta_x;
  new_vel[4]  = current_vtheta_y;
  new_vel[5]  = current_vtheta_z;

  return new_vel;
} 



std::vector<float> Get_Joint_Angles(ros::NodeHandle n, const std::string &robot_name)
{
  
  ros::ServiceClient service_client_get_measured_joint_angles = n.serviceClient<kortex_driver::GetMeasuredJointAngles>("/" + robot_name + "/base/get_measured_joint_angles");
  kortex_driver::GetMeasuredJointAngles service_get_measured_joint_angles;
  
  if (service_client_get_measured_joint_angles.call(service_get_measured_joint_angles))
  {
    ROS_INFO("The Get Joint Angles was done.");
  }

  ros::ServiceClient service_client_get_actuator_count = n.serviceClient<kortex_driver::GetActuatorCount>("/" + robot_name + "/base/get_actuator_count");
  kortex_driver::GetActuatorCount service_get_actuator_count;

  if (service_client_get_actuator_count.call(service_get_actuator_count))
  {
    ROS_INFO("The GetActuatorCount was done.");
  }

  int actuator_count = service_get_actuator_count.response.output.count;

  std::vector<float> JAngle(actuator_count);
  std::vector<float> Com_angles(actuator_count);

  for(int i = 0; i < actuator_count; i++)
  {
    
    JAngle[i] = service_get_measured_joint_angles.response.output.joint_angles[i].value;
    
  }

  return JAngle;
} 

std::vector<float> example_inv_kinematics(ros::NodeHandle n, const std::string &robot_name, const std::vector<float> &current_joint, std::vector<float> pose)
{
  ros::ServiceClient service_client_get_actuator_count = n.serviceClient<kortex_driver::GetActuatorCount>("/" + robot_name + "/base/get_actuator_count");
  kortex_driver::GetActuatorCount service_get_actuator_count;


  if (service_client_get_actuator_count.call(service_get_actuator_count))
  {
    ROS_INFO("The GetActuatorCount was done.");
  }

  int actuator_count = service_get_actuator_count.response.output.count;
  
  
  ros::ServiceClient service_client_compute_inverse_kinematics = n.serviceClient<kortex_driver::ComputeInverseKinematics>("/" + robot_name + "/base/compute_inverse_kinematics");
  kortex_driver::ComputeInverseKinematics service_compute_inverse_kinematics;
  
  service_compute_inverse_kinematics.request.input.cartesian_pose.x = pose[0];
  service_compute_inverse_kinematics.request.input.cartesian_pose.y = pose[1];
  service_compute_inverse_kinematics.request.input.cartesian_pose.z = pose[2];
  service_compute_inverse_kinematics.request.input.cartesian_pose.theta_x = pose[3];
  service_compute_inverse_kinematics.request.input.cartesian_pose.theta_y = pose[4];
  service_compute_inverse_kinematics.request.input.cartesian_pose.theta_z = pose[5];
   

  std::vector<float> JAngle(actuator_count);
  std::vector<float> Com_angles(actuator_count);
  std::vector<float> Com_angles_fail(actuator_count);

  service_compute_inverse_kinematics.request.input.guess.joint_angles.resize(actuator_count);


  for(int i = 0; i < actuator_count; i++)
  {
    JAngle[i] = current_joint[i];
    service_compute_inverse_kinematics.request.input.guess.joint_angles[i].value = JAngle[i]-1;
    
  }
  

  if (!service_client_compute_inverse_kinematics.call(service_compute_inverse_kinematics))
  {
    std::string error_string = "Failed to call ComputeInverseKinematics";
    ROS_ERROR("%s", error_string.c_str());

    for(int i = 0; i < actuator_count; i++)
    {
      Com_angles[i] = current_joint[i];;  
      //ROS_INFO_STREAM("Inv Fail Angles " << " : " << Com_angles[i]);
    }

    return Com_angles;
  }

  {
    ROS_INFO("The Inv Computation action was sent to the robot.");
    service_compute_inverse_kinematics.response.output.joint_angles.resize(actuator_count);

    for(int i = 0; i < actuator_count; i++)
    {
      Com_angles[i] = service_compute_inverse_kinematics.response.output.joint_angles[i].value;
      if (Com_angles[i] < 0 )
      {
        Com_angles[i] = Com_angles[i] + 360;
      }  
      //ROS_INFO_STREAM("Inv Kine Angles " << " : " << Com_angles[i]);
    }

    
    return Com_angles;
    
  }

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


bool Send_Joint_SpeedCommand(ros::NodeHandle n, const std::string &robot_name, std::vector<float> current_joints, std::vector<float> desired_joints, double dt)
{
  ros::ServiceClient service_client_send_joint_speeds_command = n.serviceClient<kortex_driver::SendJointSpeedsCommand>("/" + robot_name + "/base/send_joint_speeds_command");
  kortex_driver::SendJointSpeedsCommand service_send_joint_speeds_command;

  ros::ServiceClient service_client_get_actuator_count = n.serviceClient<kortex_driver::GetActuatorCount>("/" + robot_name + "/base/get_actuator_count");
  kortex_driver::GetActuatorCount service_get_actuator_count;


  if (service_client_get_actuator_count.call(service_get_actuator_count))
  {
    // ROS_INFO("The GetActuatorCount was done.");
  }

  int actuator_count = service_get_actuator_count.response.output.count;

  ROS_INFO_STREAM("actuator_count " << " : " << actuator_count);
  
  std::vector<float> diff_joints(actuator_count);
  std::vector<float> des_Jvel(actuator_count);

  for(int i = 0; i < actuator_count; i++)
  {
    
    diff_joints[i] = desired_joints[i] - current_joints[i];
    if (diff_joints[i] < -180.0f) diff_joints[i] += 360.0f;
    if (diff_joints[i] > 180.0f) diff_joints[i] -= 360.0f;

    des_Jvel[i] = diff_joints[i]/dt;  
  }

  // ROS_INFO("The des_Jvel was done.");

  service_send_joint_speeds_command.request.input.joint_speeds.resize(actuator_count);

  for(int i = 0; i < actuator_count; i++)
  {
    service_send_joint_speeds_command.request.input.joint_speeds[i].joint_identifier = i;
    service_send_joint_speeds_command.request.input.joint_speeds[i].duration = 0.05;
    
    service_send_joint_speeds_command.request.input.joint_speeds[i].value = des_Jvel[i];
    ROS_INFO_STREAM("des_Jvel " << " : " << des_Jvel[i]);
    ROS_INFO_STREAM("send_joint_speeds_command " << " : " << service_send_joint_speeds_command.request.input.joint_speeds[i].value);
    //ROS_INFO(" send Joint speeds ....");
  }


  last_action_notification_event = 0;

  if (service_client_send_joint_speeds_command.call(service_send_joint_speeds_command))
  {
    ROS_INFO("Joints command was sent to the robot.");
  }
  else
  {
  
    std::string error_string = "Failed to call Joints command";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }


  return true;
}


bool example_cartesian_action(ros::NodeHandle n, const std::string &robot_name )
{
  kortex_driver::ConstrainedPose my_constrained_pose;
  kortex_driver::CartesianSpeed my_cartesian_speed;

  my_cartesian_speed.translation = 0.1f;
  my_cartesian_speed.orientation = 15.0f;

  my_constrained_pose.constraint.oneof_type.speed.push_back(my_cartesian_speed);

  my_constrained_pose.target_pose.x = 0.374f;
  my_constrained_pose.target_pose.y = 0.057f;
  my_constrained_pose.target_pose.z = 0.43f;
  my_constrained_pose.target_pose.theta_x = 88.5f;
  my_constrained_pose.target_pose.theta_y = -0.192f;
  my_constrained_pose.target_pose.theta_z = 134.3f;

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
    AdmittanceController controller;

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
  std::vector<float> currentPose(6);
  std::vector<float> currentVel(6);
  currentPose = Get_measured_toolPose(node_handle, robot_name);
  std::vector<float> currentJoint(6);
  ROS_INFO_STREAM("toolPose: " << currentPose[0] << " "<< currentPose[1] << " "<< currentPose[2] << " "<< currentPose[3] << " "<< currentPose[4] << " "<< currentPose[5] );

  force = sensor.getForce();
  torque = sensor.getTorque();  
  ROS_INFO_STREAM("Force: " << "x: " << force.x << ", y: " << force.y << ", z: " << force.z);
  ROS_INFO_STREAM("Torque: " << "x: " << torque.x << ", y: " << torque.y << ", z: " << torque.z);
  
  ros::Rate rate(20);
  double loop_time = 0.05; //second
  double loop_ms = 50; //ms

  bool should_exit = false;

  // Set the reference frame to "Mixed"
  success &= example_retract_the_robot(node_handle, robot_name);
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  // success &= example_set_cartesian_reference_frame(node_handle, robot_name);
  // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  

  // success &= example_home_the_robot(node_handle, robot_name);
  // std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  
  currentJoint = Get_Joint_Angles(node_handle, robot_name);
  //ROS_INFO_STREAM("Joint Pos: " << "J[0]: " << currentJoint[0] << "J[1]: " << currentJoint[1]<< "J[2]: " << currentJoint[2]<< "J[3]: " << currentJoint[3] );
  ROS_INFO_STREAM("Torque: " << "x: " << torque.x << ", y: " << torque.y << ", z: " << torque.z);

  // // Joint Move test

  std::vector<float> des_Joint(6);
  std::vector<float> des_Joint2(6);
  std::vector<float> new_pose2(6); 

  // for(int j = 1; j < 6; j++)
  // {
    
  //   currentPose = Get_measured_toolPose(node_handle, robot_name);

  //   for(int i = 0; i < 6; i++)
  //   {
  //     des_Joint[i] = currentJoint[i];
  //     new_pose2[i] = currentPose[i] + 0.1;     
  //   }
  //   des_Joint[0] = currentJoint[0] + 1;
  //   des_Joint[3] = currentJoint[3] + 1;
  //   //new_pose2[5] = currentPose[5];

  //   des_Joint2 = example_inv_kinematics(node_handle, robot_name, currentJoint, new_pose2);

  //   Send_Joint_SpeedCommand(node_handle, robot_name, currentJoint, des_Joint, loop_time);

    
  //   ROS_INFO_STREAM("Desired Joint : " << des_Joint[0] << ' ' << des_Joint[1] << ' ' << des_Joint[2] << ' ' << des_Joint[3] << ' ' << des_Joint[4] << ' ' << des_Joint[5] );
  //   ROS_INFO_STREAM("Current des : " << currentJoint[0] << ' ' << currentJoint[1] << ' ' << currentJoint[2] << ' ' << currentJoint[3] << ' ' << currentJoint[4] << ' ' << currentJoint[5] );

  //   std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // }

  // Send_Joint_SpeedCommand(node_handle, robot_name, des_Joint, des_Joint, loop_time);
  // std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // success = true;

  
  // ROS_INFO_STREAM(" New control " );
  // ROS_INFO_STREAM(" New control " );
  // ROS_INFO_STREAM(" New control " );


  // for(int j = 1; j < 6; j++)
  // {
    
  //   currentPose = Get_measured_toolPose(node_handle, robot_name);

  //   for(int i = 0; i < 6; i++)
  //   {
  //     des_Joint[i] = currentJoint[i];
  //     new_pose2[i] = currentPose[i] + 0.01;    
  //   }
  //   des_Joint[0] = currentJoint[0] - 1.5;
  //   des_Joint[3] = currentJoint[3] - 2;

  //   des_Joint2 = example_inv_kinematics(node_handle, robot_name, currentJoint, new_pose2);
  //   ROS_INFO_STREAM("Inv_ kine _ joint : " << des_Joint2[0] << ' ' << des_Joint2[1] << ' ' << des_Joint2[2] << ' ' << des_Joint2[3] << ' ' << des_Joint2[4] << ' ' << des_Joint2[5] );
  //   Send_Joint_SpeedCommand(node_handle, robot_name, currentJoint, des_Joint, loop_time);

  //   ROS_INFO_STREAM("Desired Joint : " << des_Joint[0] << ' ' << des_Joint[1] << ' ' << des_Joint[2] << ' ' << des_Joint[3] << ' ' << des_Joint[4] << ' ' << des_Joint[5] );
  //   ROS_INFO_STREAM("Current des : " << currentJoint[0] << ' ' << currentJoint[1] << ' ' << currentJoint[2] << ' ' << currentJoint[3] << ' ' << currentJoint[4] << ' ' << currentJoint[5] );

  //   std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // }

  // Send_Joint_SpeedCommand(node_handle, robot_name, des_Joint, des_Joint, loop_time);
  // std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // success = true;


  // success &= example_retract_the_robot(node_handle, robot_name);
  // std::this_thread::sleep_for(std::chrono::milliseconds(2000));


  // Admittance Initial Position

  for(int j = 1; j < 6; j++)
  {
    
    currentJoint = Get_Joint_Angles(node_handle, robot_name);
    currentVel = Get_measured_toolVel(node_handle, robot_name);    
    currentPose = Get_measured_toolPose(node_handle, robot_name);

    for(int i = 0; i < 6; i++)
    {
      des_Joint[i] = currentJoint[i];
      new_pose2[i] = currentPose[i] + 0.01;    
    }
    des_Joint[0] = currentJoint[0] - 1.0;
    des_Joint[2] = currentJoint[2] + 1.0;
    des_Joint[4] = currentJoint[4] - 1.0;

    des_Joint2 = example_inv_kinematics(node_handle, robot_name, currentJoint, new_pose2);
    ROS_INFO_STREAM("Inv_ kine _ joint : " << des_Joint2[0] << ' ' << des_Joint2[1] << ' ' << des_Joint2[2] << ' ' << des_Joint2[3] << ' ' << des_Joint2[4] << ' ' << des_Joint2[5] );
    Send_Joint_SpeedCommand(node_handle, robot_name, currentJoint, des_Joint, loop_time);

    ROS_INFO_STREAM("Desired Joint : " << des_Joint[0] << ' ' << des_Joint[1] << ' ' << des_Joint[2] << ' ' << des_Joint[3] << ' ' << des_Joint[4] << ' ' << des_Joint[5] );
    ROS_INFO_STREAM("Current des : " << currentJoint[0] << ' ' << currentJoint[1] << ' ' << currentJoint[2] << ' ' << currentJoint[3] << ' ' << currentJoint[4] << ' ' << currentJoint[5] );

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

  }
  
  currentJoint = Get_Joint_Angles(node_handle, robot_name);
  currentVel = Get_measured_toolVel(node_handle, robot_name);    
  currentPose = Get_measured_toolPose(node_handle, robot_name);
  Send_Joint_SpeedCommand(node_handle, robot_name, currentJoint, currentJoint, loop_time);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));


  // //Admittance 

  ROS_INFO_STREAM("Admittance Control ....ing");
  ROS_INFO_STREAM("Admittance Control ....ing");
  ROS_INFO_STREAM("Admittance Control ....ing");
  ROS_INFO_STREAM("Admittance Control ....ing");


  while ( ros::ok() && !should_exit ) 
  {
    currentPose = Get_measured_toolPose(node_handle, robot_name);
    currentVel = Get_measured_toolVel(node_handle, robot_name);    
    currentJoint = Get_Joint_Angles(node_handle, robot_name);
    
    force = sensor.getForce();
    torque = sensor.getTorque();
    ROS_INFO_STREAM("Force: " << "x: " << force.x << ", y: " << force.y << ", z: " << force.z);
    ROS_INFO_STREAM("Torque: " << "x: " << torque.x << ", y: " << torque.y << ", z: " << torque.z); 
    ROS_INFO_STREAM("Current Pose : " << currentPose[0] << ' ' << currentPose[1] << ' ' << currentPose[2] << ' ' << currentPose[3] << ' ' << currentPose[4] << ' ' << currentPose[5] );
    ROS_INFO_STREAM("Current Velocity : " << currentVel[0] << ' ' << currentVel[1] << ' ' << currentVel[2] << ' ' << currentVel[3] << ' ' << currentVel[4] << ' ' << currentVel[5] );

    std::vector<float> new_pose = controller.calculateNewPose(currentPose, currentVel, force, torque, loop_time);
    
    ROS_INFO_STREAM("New Pose : " << new_pose[0] << ' ' << new_pose[1] << ' ' << new_pose[2] << ' ' << new_pose[3] << ' ' << new_pose[4] << ' ' << new_pose[5] );

    des_Joint = example_inv_kinematics(node_handle, robot_name, currentJoint, new_pose);

    ROS_INFO_STREAM("Current Joint : " << currentJoint[0] << ' ' << currentJoint[1] << ' ' << currentJoint[2] << ' ' << currentJoint[3] << ' ' << currentJoint[4] << ' ' << currentJoint[5] );
    ROS_INFO_STREAM("Desired Joint : " << des_Joint[0] << ' ' << des_Joint[1] << ' ' << des_Joint[2] << ' ' << des_Joint[3] << ' ' << des_Joint[4] << ' ' << des_Joint[5] );

    Send_Joint_SpeedCommand(node_handle, robot_name, currentJoint, des_Joint, loop_time);
    
    ROS_INFO_STREAM("Admittance Control ....ing  , Command send");

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  


  success &= all_notifs_succeeded;



  // Report success for testing purposes
  //ros::param::set("/kortex_examples_test_results/full_arm_movement_cpp", success);
  
  return success ? 0 : 1;

}
