#pragma once

#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <webots/Supervisor.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <webots/Keyboard.hpp>

using Eigen::Matrix3d;
using Eigen::Vector3d;

class WebotsInterface
{
public:
    WebotsInterface();
    ~WebotsInterface();
    void recvState(Eigen::VectorXd &state_vector, Eigen::Vector3d &object_position,
        Eigen::Vector3d &object_linear_velocity, Eigen::Vector3d &object_angular_velocity,
        Eigen::Matrix3d &object_rotation); // second
    void sendCmd(const Eigen::VectorXd &tau, const Eigen::VectorXd &force); // second
    bool isRunning();
    double current_time() { return current_time_; }
    void resetSim() { supervisor_->simulationReset(); }
    void step() { supervisor_->step(time_step_); }

private:
    void initRecv();
    void initSend();

    int time_step_;
    double current_time_;

    Eigen::VectorXd last_q_;

    // webots interface
    webots::Supervisor *supervisor_;
    webots::Node *robot_node_;
    webots::Node *object_node_;

    webots::Motor *joint_motor_[18];
    webots::PositionSensor *joint_sensor_[18];

    webots::InertialUnit *imu_;
    webots::Gyro *gyro_;

    std::string object_name_ = "CUBE";
    std::string robot_name_ = "GalileoV1d6X5";
    std::string imu_name_ = "imu";
    std::string gyro_name_ = "gyro";
    std::vector<std::string> joint_sensor_name_ = {"FL_hip_joint_sensor", "FL_thigh_joint_sensor", "FL_calf_joint_sensor", 
                                                    "RL_hip_joint_sensor", "RL_thigh_joint_sensor", "RL_calf_joint_sensor", 
                                                    "FR_hip_joint_sensor", "FR_thigh_joint_sensor", "FR_calf_joint_sensor", 
                                                    "RR_hip_joint_sensor", "RR_thigh_joint_sensor", "RR_calf_joint_sensor", 
                                                    "joint1_sensor", "joint2_sensor", "joint3_sensor", "joint4_sensor", "joint5_sensor", "joint6_sensor"};

                                                   
    std::vector<std::string> joint_motor_name_ = {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", 
                                                  "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", 
                                                  "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", 
                                                  "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint", 
                                                  "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};


};