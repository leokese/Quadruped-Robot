#include "utils/logger.hpp"

void saveVectorsToCsv(const std::string &filename, const std::vector<Eigen::VectorXd> &vectors)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        throw "Unable to open file for writing";
    }

    for (const auto &vec : vectors)
    {
        for (int i = 0; i < vec.size(); ++i)
        {
            file << vec(i);
            if (i < vec.size() - 1)
                file << ",";
        }
        file << "\n";
    }
    file.close();
    std::cout << "Results saved to " << filename << std::endl;
}

bool readTrajectoryFromFile(const std::string &filepath, std::vector<Waypoint> &trajectory)
{
    std::ifstream fin(filepath);
    if (!fin.is_open()) {
        std::cerr << "Failed to open file: " << filepath << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(fin, line)) {
        std::istringstream iss(line);
        std::string tmp;
        double time, px, py, pz, vx, vy, vz;

        // 按格式解析，先读time:
        if (!(iss >> tmp) || tmp != "time:") continue;
        if (!(iss >> time)) continue;

        if (!(iss >> tmp) || tmp != ";") continue;
        if (!(iss >> tmp) || tmp != "p:") continue;
        if (!(iss >> px >> py >> pz)) continue;

        if (!(iss >> tmp) || tmp != ";") continue;
        if (!(iss >> tmp) || tmp != "v:") continue;
        if (!(iss >> vx >> vy >> vz)) continue;

        Waypoint wp(VectorXd(3), VectorXd(3), time);
        wp.position_ << px, py, pz;
        wp.velocity_ << vx, vy, vz;

        trajectory.push_back(wp);
    }

    fin.close();
    return !trajectory.empty();
}

int findNearestWaypointIndex(const std::vector<Waypoint> &trajectory, double currentTime) 
{
    int index = 0;
    double minDiff = std::numeric_limits<double>::max();
    for (int i = 0; i < static_cast<int>(trajectory.size()); ++i) {
        double diff = std::abs(trajectory[i].time_ - currentTime);
        if (diff < minDiff) {
            minDiff = diff;
            index = i;
        }
    }
    return index;
}