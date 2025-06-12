#pragma once
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include "object_qp/object_planner.hpp"
#include "object_qp/get_trajectory.hpp"


void saveVectorsToCsv(const std::string &filename, const std::vector<Eigen::VectorXd> &vectors);

bool readTrajectoryFromFile(const std::string &filepath, std::vector<Waypoint> &trajectory);

int findNearestWaypointIndex(const std::vector<Waypoint> &trajectory, double currentTime);