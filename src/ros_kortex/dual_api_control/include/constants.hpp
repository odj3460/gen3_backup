/*
Author(s): Djordje Vukcevic, Sven Schneider
Institute: Hochschule Bonn-Rhein-Sieg

Copyright (c) [2019]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef CONSTANTS_HPP_
#define CONSTANTS_HPP_
#include <Eigen/Core>
#include <vector>
#include <stdlib.h> /* abs */
#include <unistd.h>
#include <cmath>

#define PI 3.14159265358979323846
#define DEG_TO_RAD(x) (x) * PI / 180.0
#define RAD_TO_DEG(x) (x) * 180.0 / PI

namespace kinova_constants
{
    // Left
    
    //Robot ID/Name
    extern const std::string ID_L;

    extern const int NUMBER_OF_JOINTS_L;
    extern const int NUMBER_OF_SEGMENTS_L;
    extern const int NUMBER_OF_FRAMES_L;

    //Arm's root acceleration
    extern const std::vector<double> root_acceleration_L;

    extern const std::vector<double> joint_position_limits_max_L;
    extern const std::vector<double> joint_position_limits_min_L;
    extern const std::vector<double> joint_position_thresholds_L;
    extern const std::vector<double> joint_offsets_L;

    extern const std::vector<double> joint_velocity_limits_L;
    extern const std::vector<double> joint_acceleration_limits_L;
    extern const std::vector<double> joint_torque_limits_L;
    extern const std::vector<double> joint_stopping_torque_limits_L;

    extern const std::vector<double> joint_inertia_L;

    extern const std::string config_path_L;    
    extern const std::string urdf_path_L;

    extern const std::string root_name_L;
    extern const std::string tooltip_name_L;


    // Right
    
    //Robot ID/Name
    extern const std::string ID_R;

    extern const int NUMBER_OF_JOINTS_R;
    extern const int NUMBER_OF_SEGMENTS_R;
    extern const int NUMBER_OF_FRAMES_R;

    //Arm's root acceleration
    extern const std::vector<double> root_acceleration_R;

    extern const std::vector<double> joint_position_limits_max_R;
    extern const std::vector<double> joint_position_limits_min_R;
    extern const std::vector<double> joint_position_thresholds_R;
    extern const std::vector<double> joint_offsets_R;

    extern const std::vector<double> joint_velocity_limits_R;
    extern const std::vector<double> joint_acceleration_limits_R;
    extern const std::vector<double> joint_torque_limits_R;
    extern const std::vector<double> joint_stopping_torque_limits_R;

    extern const std::vector<double> joint_inertia_R;

    extern const std::string config_path_R;    
    extern const std::string urdf_path_R;

    extern const std::string root_name_R;
    extern const std::string tooltip_name_R;
}


namespace dynamics_parameter
{
    // Number of task constraints imposed on the robot, i.e. Cartesian DOFS
    extern const int NUMBER_OF_CONSTRAINTS;
    extern const int DECELERATION_UPDATE_DELAY;
    extern const int STEADY_STOP_ITERATION_THRESHOLD;
    extern const double LOWER_DECELERATION_RAMP_THRESHOLD;
    extern const double STOPPING_MOTION_LOOP_FREQ; // Hz
    extern const Eigen::VectorXd MAX_CART_FORCE;
    extern const Eigen::VectorXd MAX_CART_ACC;
    extern const Eigen::IOFormat WRITE_FORMAT;
}

#endif /* CONSTANTS_HPP_ */