
#include <ros/ros.h>
#include <iostream>

#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/WrenchStamped.h>


using namespace std;


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
    sub_ = nh_.subscribe("RFT_FORCE", 1000, &ForceTorqueSensor::callback, this);
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


int main(int argc, char** argv)
{
	// initializing for ros 
	ros::init(argc, argv, "kinova_move");
	ros::NodeHandle node_handle;

    ForceTorqueSensor sensor;

    //ros::Rate rate(10);

	ros::AsyncSpinner spinner(1);
	spinner.start();

    


	// initializing for moveit
	cout<<0<<endl;
	moveit::planning_interface::MoveGroupInterface right_arm("right_arm");
	cout<<0<<endl;
	right_arm.setGoalJointTolerance(0.001);
	right_arm.setMaxAccelerationScalingFactor(0.2);
	right_arm.setMaxVelocityScalingFactor(0.5);
    
    // FT sensor Read
    geometry_msgs::Vector3 force = sensor.getForce();
    geometry_msgs::Vector3 torque = sensor.getTorque();

    ROS_INFO_STREAM("Force: " << "x: " << force.x << ", y: " << force.y << ", z: " << force.z);
    ROS_INFO_STREAM("Torque: " << "x: " << torque.x << ", y: " << torque.y << ", z: " << torque.z);


    // Initial Position

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

    // FT sensor Read
    force = sensor.getForce();
    torque = sensor.getTorque();
    
    ROS_INFO_STREAM("Force: " << "x: " << force.x << ", y: " << force.y << ", z: " << force.z);
    ROS_INFO_STREAM("Torque: " << "x: " << torque.x << ", y: " << torque.y << ", z: " << torque.z);

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
