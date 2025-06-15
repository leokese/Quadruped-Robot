#include "webots_interface.hpp"

WebotsInterface::WebotsInterface()
{
    supervisor_ = new webots::Supervisor();
    time_step_ = (int)supervisor_->getBasicTimeStep();
    std::cout << "timeStep in simulation is :" << time_step_ << std::endl;

    initRecv();
    initSend();

    last_q_.resize(joint_sensor_name_.size());
    // 在更新循环中读取每个传感器的值并赋值
    for (int i = 0; i < 18; ++i) {
        last_q_[i] = joint_sensor_[i]->getValue();
    }
    
}

WebotsInterface::~WebotsInterface()
{
    delete supervisor_;
}

void WebotsInterface::recvState(Eigen::VectorXd &state_vector, Eigen::Vector3d &object_position,
                                Eigen::Vector3d &object_linear_velocity, Eigen::Vector3d &object_angular_velocity,
                                Eigen::Matrix3d &object_rotation)
{
    current_time_ = supervisor_->getTime();
    Eigen::VectorXd q(25), v(24);

    // sensor data
    auto imu_data = imu_->getQuaternion();
    Eigen::Quaterniond quaternion(imu_data[3], imu_data[0], imu_data[1], imu_data[2]);        // x,y,z,w
    Eigen::Vector3d angular_vel_B = Eigen::Map<const Eigen::Vector3d>(gyro_->getValues());    // expressed in BODY frame
    Eigen::Vector3d robotPos = Eigen::Map<const Eigen::Vector3d>(robot_node_->getPosition()); // expressed in WORLD frame
    Eigen::Vector3d robotVel = Eigen::Map<const Eigen::Vector3d>(robot_node_->getVelocity()); // expressed in WORLD frame
    Eigen::Vector3d robotVel_B = quaternion.toRotationMatrix().transpose() * robotVel;        // expressed in BODY frame

    q.head(6) << robotPos, quaternion.coeffs();
    v.head(6) << robotVel_B, angular_vel_B;

    for (int i = 0; i < 18; i++)
    {
        q(7 + i) = joint_sensor_[i]->getValue();
        v(6 + i) = (q(7 + i) - last_q_(i)) / double(time_step_) * 1000;
        last_q_(i) = q(7 + i);
    }
    state_vector << q, v;
    std::cout << "state_vector: " << state_vector.transpose() << std::endl;

    const double *pos = object_node_->getPosition();   // world position [x, y, z]
    const double *vel = object_node_->getVelocity();   // [vx, vy, vz, wx, wy, wz]
    // 怎么获取object的rotation？
    const double *orientation = object_node_->getOrientation(); // 旋转矩阵

    // 转换到 Eigen 类型
    object_position << pos[0], pos[1], pos[2];

    object_linear_velocity << vel[0], vel[1], vel[2];
    object_angular_velocity << vel[3], vel[4], vel[5];

    object_rotation << orientation[0], orientation[1], orientation[2],
                orientation[3], orientation[4], orientation[5],
                orientation[6], orientation[7], orientation[8];

    

}

void WebotsInterface::sendCmd(const Eigen::VectorXd &tau, const Eigen::VectorXd &force)
{
    for (int i = 0; i < 18; i++)
    {
        joint_motor_[i]->setTorque(tau(i));
    }

    const double f[3] = { force(0), force(1), force(2) };
    object_node_->addForce(f, false);  // false = 世界坐标系下施加
}

bool WebotsInterface::isRunning()
{
    if (supervisor_->step(time_step_) != -1)
        return true;
    else
        return false;
}

void WebotsInterface::initRecv()
{
    // supervisor init
    robot_node_ = supervisor_->getFromDef(robot_name_);
    object_node_ = supervisor_->getFromDef(object_name_);
    if (object_node_ == NULL)
    {
        printf("error object\n");
        exit(1);
    }
    if (robot_node_ == NULL)
    {
        printf("error supervisor\n");
        exit(1);
    }

    // sensor init
    imu_ = supervisor_->getInertialUnit(imu_name_);
    imu_->enable(time_step_);
    gyro_ = supervisor_->getGyro(gyro_name_);
    gyro_->enable(time_step_);
    for (int i = 0; i < 18; i++)
    {
        joint_sensor_[i] = supervisor_->getPositionSensor(joint_sensor_name_[i]);
        joint_sensor_[i]->enable(time_step_);
    }

}

void WebotsInterface::initSend()
{
    for (int i = 0; i < 18; i++)
        joint_motor_[i] = supervisor_->getMotor(joint_motor_name_[i]);

    const double f[3] = { 0.0, 0.0, 0.0 }; // 初始力为零
    object_node_->addForce(f, false);  
}