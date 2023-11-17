#include <constants.hpp>

#define PI 3.14159265358979323846
#define DEG_TO_RAD(x) (x) * PI / 180.0

namespace kinova_constants
{
   // Left

   //Robot ID/Name
   const std::string ID_L("kinova_gen3_arm");

   // Number of joints in the manipulator
   const int NUMBER_OF_JOINTS_L(7);
   const int NUMBER_OF_SEGMENTS_L(7);
   const int NUMBER_OF_FRAMES_L(8);

   //Arm's root acceleration
   const std::vector<double> root_acceleration_L {0.0, 0.0, 9.81289, 0.0, 0.0, 0.0};

   // Limits from Kinova manual-> Must be confirmed
   const std::vector<double> joint_position_limits_max_L {DEG_TO_RAD(9999.0), DEG_TO_RAD(127.0), DEG_TO_RAD(9999.0), DEG_TO_RAD(147.8), DEG_TO_RAD(9999.0), DEG_TO_RAD(120.3), DEG_TO_RAD(9999.0)};
   const std::vector<double> joint_position_limits_min_L {DEG_TO_RAD(-9999.0), DEG_TO_RAD(-127.0), DEG_TO_RAD(-9999.0), DEG_TO_RAD(-147.8), DEG_TO_RAD(-9999.0), DEG_TO_RAD(-120.3), DEG_TO_RAD(-9999.0)};

   const std::vector<double> joint_velocity_limits_L {DEG_TO_RAD(50.0), DEG_TO_RAD(50.0), DEG_TO_RAD(50.0), DEG_TO_RAD(50.0), DEG_TO_RAD(50.0), DEG_TO_RAD(50.0), DEG_TO_RAD(50.0)};
   const std::vector<double> joint_acceleration_limits_L {5.19, 5.19, 5.19, 5.19, 9.99, 9.99, 9.99};
   const std::vector<double> joint_torque_limits_L {39.0, 39.0, 39.0, 39.0, 9.0, 9.0, 9.0};
   const std::vector<double> joint_stopping_torque_limits_L {39.0, 39.0, 39.0, 39.0, 13.0, 13.0, 13.0};

   //  const std::vector<double> joint_position_thresholds {DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0)};
   const std::vector<double> joint_position_thresholds_L {DEG_TO_RAD(10), DEG_TO_RAD(10), DEG_TO_RAD(10), DEG_TO_RAD(10), DEG_TO_RAD(10), DEG_TO_RAD(10), DEG_TO_RAD(10)};

   const std::vector<double> joint_offsets_L {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // Rotor inertia - "d" in the algorithm:
   const std::vector<double> joint_inertia_L {0.5580, 0.5580, 0.5580, 0.5580, 0.1389, 0.1389, 0.1389};
    // const std::vector<double> joint_inertia {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

   const std::string urdf_path_L = "/home/djole/Master/Thesis/GIT/MT_testing/Controller/urdf/kinova-gen3_urdf_V12.urdf";

   // 7 joints, 7 links, 8 frames
   const std::string root_name_L = "base_link_L";

   /**
    * With Bracelet_Link parameter, the last frame is at joint 7.
    * Mass and COM of the last (end-effector) link are included but not the real end-effector's frame.
    * Arm length: 1.12586m
    */
   const std::string tooltip_name_L = "Bracelet_Link_L";

   /**
    * With EndEffector_Link parameter, last frame is at the real end-effector's frame.
    * However, in the urdf model, joint between Bracelet_Link and EndEffector_Link is fixed (not counted in KDL). 
    * Vereshchagin does not support un-equal number of joints and segments
    * Arm length: 1.1873m
    */ 
//    const std::string tooltip_name = "EndEffector_Link";

    // Right

   //Robot ID/Name
   const std::string ID_R("kinova_gen3_arm_R");

   // Number of joints in the manipulator
   const int NUMBER_OF_JOINTS_R(6);
   const int NUMBER_OF_SEGMENTS_R(6);
   const int NUMBER_OF_FRAMES_R(7);

   //Arm's root acceleration
   const std::vector<double> root_acceleration_R {0.0, 0.0, 9.81289, 0.0, 0.0, 0.0};

   // Limits from Kinova manual-> Must be confirmed
   const std::vector<double> joint_position_limits_max_R {DEG_TO_RAD(9999.0), DEG_TO_RAD(128.9), DEG_TO_RAD(147.8), DEG_TO_RAD(9999.0), DEG_TO_RAD(120.3), DEG_TO_RAD(9999.0)};
   const std::vector<double> joint_position_limits_min_R {DEG_TO_RAD(-9999.0), DEG_TO_RAD(-128.9), DEG_TO_RAD(-147.8), DEG_TO_RAD(-9999.0), DEG_TO_RAD(-120.3), DEG_TO_RAD(-9999.0)};

   const std::vector<double> joint_velocity_limits_R {DEG_TO_RAD(50.0), DEG_TO_RAD(50.0), DEG_TO_RAD(50.0), DEG_TO_RAD(50.0), DEG_TO_RAD(50.0), DEG_TO_RAD(50.0)};
   const std::vector<double> joint_acceleration_limits_R {5.19, 5.19, 5.19, 9.99, 9.99, 9.99};
   const std::vector<double> joint_torque_limits_R {39.0, 39.0, 39.0, 9.0, 9.0, 9.0};
   const std::vector<double> joint_stopping_torque_limits_R {39.0, 39.0, 39.0, 13.0, 13.0, 13.0};

   //  const std::vector<double> joint_position_thresholds {DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0)};
   const std::vector<double> joint_position_thresholds_R {DEG_TO_RAD(10), DEG_TO_RAD(10), DEG_TO_RAD(10), DEG_TO_RAD(10), DEG_TO_RAD(10), DEG_TO_RAD(10), DEG_TO_RAD(10)};

   const std::vector<double> joint_offsets_R {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // Rotor inertia - "d" in the algorithm:
   const std::vector<double> joint_inertia_R {0.5580, 0.5580, 0.5580, 0.1389, 0.1389, 0.1389};
    // const std::vector<double> joint_inertia {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

   const std::string urdf_path_R = "/home/djole/Master/Thesis/GIT/MT_testing/Controller/urdf/kinova-gen3_urdf_V12.urdf";

   // 7 joints, 7 links, 8 frames
   const std::string root_name_R = "base_link_R";
   const std::string tooltip_name_R = "Bracelet_Link_R";

}


namespace dynamics_parameter
{
    // Number of task constraints imposed on the robot, i.e. Cartesian DOFS
    const int NUMBER_OF_CONSTRAINTS(6);
    const int DECELERATION_UPDATE_DELAY = 5; // Iterations
    const int STEADY_STOP_ITERATION_THRESHOLD = 40; // Iterations
    const double LOWER_DECELERATION_RAMP_THRESHOLD = 0.05; // rad/sec
    const double STOPPING_MOTION_LOOP_FREQ = 750.0; // Hz  ... Higher than 750 Hz not yet feasible with the current Kinova API
    const Eigen::VectorXd MAX_CART_FORCE = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) << 50.0, 50.0, 200.0, 2.0, 2.0, 2.0).finished();
    const Eigen::VectorXd MAX_CART_ACC = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) << 100.0, 100.0, 200.0, 2.0, 2.0, 2.0).finished();
    const Eigen::IOFormat WRITE_FORMAT(6, Eigen::DontAlignCols, " ", "", "", "\n");
    const std::string LOG_FILE_CART_PATH("/home/djole/Master/Thesis/GIT/MT_testing/Controller/visualization/archive/control_error.txt");
    const std::string LOG_FILE_STOP_MOTION_PATH("/home/djole/Master/Thesis/GIT/MT_testing/Controller/visualization/archive/stop_motion_error.txt");
    const std::string LOG_FILE_CART_BASE_PATH("/home/djole/Master/Thesis/GIT/MT_testing/Controller/visualization/archive/control_base_error.txt");
    const std::string LOG_FILE_JOINT_PATH("/home/djole/Master/Thesis/GIT/MT_testing/Controller/visualization/archive/joint_torques.txt");
    const std::string LOG_FILE_PREDICTIONS_PATH("/home/djole/Master/Thesis/GIT/MT_testing/Controller/visualization/archive/prediction_effects.txt");
    const std::string LOG_FILE_NULL_SPACE_PATH("/home/djole/Master/Thesis/GIT/MT_testing/Controller/visualization/archive/null_space_error.txt");
}

namespace prediction_parameter
{
		const std::string CURRENT_POSE_DATA_PATH = "/home/djole/Master/Thesis/GIT/MT_testing/Controller/visualization/archive/measured_pose.txt";
		const std::string PREDICTED_POSE_DATA_PATH = "/home/djole/Master/Thesis/GIT/MT_testing/Controller/visualization/archive/predicted_pose.txt";
		const std::string TWIST_DATA_PATH = "/home/djole/Master/Thesis/GIT/MT_testing/Controller/visualization/archive/current_twist.txt";
}