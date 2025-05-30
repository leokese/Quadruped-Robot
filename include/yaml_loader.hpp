#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <iostream>
#include <map>

struct YamlLoader
{
    YAML::Node config;

    Eigen::VectorXd w_x_body_pos;
    Eigen::VectorXd w_x_leg_pos;
    Eigen::VectorXd w_x_arm_pos;
    Eigen::VectorXd w_x_body_vel;
    Eigen::VectorXd w_x_leg_vel;
    Eigen::VectorXd w_x_arm_vel;

    Eigen::VectorXd w_u_foot_force;
    Eigen::VectorXd w_u_arm_force;
    Eigen::VectorXd w_u_leg_acc;
    Eigen::VectorXd w_u_arm_acc;

    Eigen::VectorXd w_fly_high;
    double fly_high_slope;

    Eigen::VectorXd w_zmp;

    Eigen::VectorXd w_arm_pos;
    Eigen::VectorXd w_arm_vel;

    Eigen::VectorXd w_foot_pos;

    YamlLoader(const std::string &file_path)
    {
        try
        {
            config = YAML::LoadFile(file_path);

            // Load vectors
            w_x_body_pos = loadVector("w_x_body_pos");
            w_x_leg_pos = loadVector("w_x_leg_pos");
            w_x_arm_pos = loadVector("w_x_arm_pos");
            w_x_body_vel = loadVector("w_x_body_vel");
            w_x_leg_vel = loadVector("w_x_leg_vel");
            w_x_arm_vel = loadVector("w_x_arm_vel");

            w_u_foot_force = loadVector("w_u_foot_force");
            w_u_arm_force = loadVector("w_u_arm_force");
            w_u_leg_acc = loadVector("w_u_leg_acc");
            w_u_arm_acc = loadVector("w_u_arm_acc");

            w_fly_high = loadVector("w_fly_high");
            fly_high_slope = config["fly_high_slope"].as<double>();

            w_zmp = loadVector("w_zmp");

            w_arm_pos = loadVector("w_arm_pos");
            w_arm_vel = loadVector("w_arm_vel");

            w_foot_pos = loadVector("w_foot_pos");
        }
        catch (const YAML::Exception &e)
        {
            std::cerr << "Error loading YAML file: " << e.what() << std::endl;
            throw;
        }
    }

private:
    Eigen::VectorXd loadVector(const std::string &key)
    {
        if (!config[key])
        {
            throw std::runtime_error("Key not found: " + key);
        }

        const auto &values = config[key].as<std::vector<double>>();
        Eigen::VectorXd vector(values.size());
        for (size_t i = 0; i < values.size(); ++i)
        {
            vector[i] = values[i];
        }
        return vector;
    }
};