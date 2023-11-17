
#include <ros/ros.h>
#include <iostream>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/WrenchStamped.h>

#include <thread>
#include <atomic>
#include <std_msgs/Empty.h>

// #include <kinova_mediator_L.hpp>
// #include <kinova_mediator_R.hpp>

#include <KDetailedException.h>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>

#include <google/protobuf/util/json_util.h>


#define IP_ADDRESS_1 "192.168.1.10"  // left
#define IP_ADDRESS_2 "192.168.1.20"  // right
#define PORT 10000


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



//using namespace std;


int main(int argc, char** argv)
{

  // Initialize sensor and controller
  ForceTorqueSensor sensor;
  AdmittanceController controller;

  // FT sensor Read
  geometry_msgs::Vector3 force = sensor.getForce();
  geometry_msgs::Vector3 torque = sensor.getTorque();

  ROS_INFO_STREAM("Force: " << "x: " << force.x << ", y: " << force.y << ", z: " << force.z);
  ROS_INFO_STREAM("Torque: " << "x: " << torque.x << ", y: " << torque.y << ", z: " << torque.z);


  
	//***********************************

  
  force = sensor.getForce();
  torque = sensor.getTorque();  
  ROS_INFO_STREAM("Force: " << "x: " << force.x << ", y: " << force.y << ", z: " << force.z);
  ROS_INFO_STREAM("Torque: " << "x: " << torque.x << ", y: " << torque.y << ", z: " << torque.z);
  
  ros::Rate rate(1000);
  double loop_time = 0.001; //second
  double loop_ms = 1; //ms


  while ( ros::ok() ) 
  {
    
    force = sensor.getForce();
    torque = sensor.getTorque();
    ROS_INFO_STREAM("Force: " << "x: " << force.x << ", y: " << force.y << ", z: " << force.z);
    ROS_INFO_STREAM("Torque: " << "x: " << torque.x << ", y: " << torque.y << ", z: " << torque.z); 
    

    std::this_thread::sleep_for(std::chrono::milliseconds(std::lround(loop_ms)));
  }


  
  return 0;

}
