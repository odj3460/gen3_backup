
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


using namespace std;


int main(int argc, char** argv)
{
	// initializing for ros 
	ros::init(argc, argv, "kinova_move");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// initializing for moveit
	cout<<0<<endl;
	moveit::planning_interface::MoveGroupInterface right_arm("right_arm");
	cout<<0<<endl;
	right_arm.setGoalJointTolerance(0.001);
	right_arm.setMaxAccelerationScalingFactor(0.2);
	right_arm.setMaxVelocityScalingFactor(0.5);

	right_arm.setNamedTarget("retract");
	right_arm.move();
	sleep(1);

	// go to home pose
	right_arm.setNamedTarget("home");
	right_arm.move();
	sleep(1);

	// get current Pose of end
	geometry_msgs::PoseStamped currentPose;
	currentPose = right_arm.getCurrentPose();
	cout<<currentPose<<endl;

	// move x 10cm forward
	geometry_msgs::Pose movePose;
	movePose.position.x=currentPose.pose.position.x;
	movePose.position.y=currentPose.pose.position.y+0.1;
	movePose.position.z=currentPose.pose.position.z;
	movePose.orientation=currentPose.pose.orientation;
	right_arm.setPoseTarget(movePose);
	right_arm.move();
	sleep(1);

	// get current Pose of end
	currentPose = right_arm.getCurrentPose();
	cout<<currentPose<<endl;

	// move x 10cm backward
	movePose.position.x=currentPose.pose.position.x;
	movePose.position.y=currentPose.pose.position.y-0.1;
	movePose.position.z=currentPose.pose.position.z;
	movePose.orientation=currentPose.pose.orientation;
	right_arm.setPoseTarget(movePose);
	right_arm.move();
	sleep(1);

	// back to retract pose
	right_arm.setNamedTarget("retract");
	right_arm.move();
	sleep(1);


	
	return 0;
}
