
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
    double damping_;  // Damping gain


    geometry_msgs::Vector3 velocity_;
    geometry_msgs::Vector3 position_;

public:
    AdmittanceController() : damping_(0.0003)
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

        position_.x += force.y*damping_;
        position_.y += force.x*damping_;
        position_.z += -force.z*damping_;

        // Update pose
        new_pose.position.x += position_.x;
        new_pose.position.y += position_.y;
        new_pose.position.z += position_.z;

        return new_pose;
    }
};

int main(int argc, char** argv)
{
	// initializing for ros 
	ros::init(argc, argv, "kinova_move");
	ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
	spinner.start();

    // Initialize sensor and controller
    ForceTorqueSensor sensor;
    AdmittanceController controller;


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

	
    // go to retract pose
    right_arm.setNamedTarget("retract");
	right_arm.move();
	sleep(1);

	// get current Pose of end
	geometry_msgs::PoseStamped currentPose;
	currentPose = right_arm.getCurrentPose();
	cout<<currentPose<<endl;
	// move x 10cm forward
	geometry_msgs::Pose movePose;
	movePose.position.x=currentPose.pose.position.x+0.1;
	movePose.position.y=currentPose.pose.position.y+0.1;
	movePose.position.z=currentPose.pose.position.z;
	movePose.orientation=currentPose.pose.orientation;
	right_arm.setPoseTarget(movePose);
	right_arm.move();
	sleep(1);

    // Main control loop

    force = sensor.getForce();
    torque = sensor.getTorque();

    ROS_INFO_STREAM("Force: " << "x: " << force.x << ", y: " << force.y << ", z: " << force.z);
    ROS_INFO_STREAM("Torque: " << "x: " << torque.x << ", y: " << torque.y << ", z: " << torque.z);

    ros::Rate rate(1000);
    ROS_INFO_STREAM("Admittance Control" << "Start" );
    bool should_exit = false;

    
    while ( ros::ok() && !should_exit ) 
    {
        currentPose = right_arm.getCurrentPose();
        geometry_msgs::Pose currentPose_ = currentPose.pose;
        force = sensor.getForce();
        torque = sensor.getTorque();
        ROS_INFO_STREAM("Force: " << "x: " << force.x << ", y: " << force.y << ", z: " << force.z);
        ROS_INFO_STREAM("Torque: " << "x: " << torque.x << ", y: " << torque.y << ", z: " << torque.z); 

        geometry_msgs::Pose new_pose = controller.calculateNewPose(currentPose_, force, torque);

        right_arm.setPoseTarget(new_pose);
        right_arm.asyncMove();

        rate.sleep();
    }


	// back to retract pose
	right_arm.setNamedTarget("retract");
	right_arm.move();
	sleep(1);

    

	
	return 0;
}
