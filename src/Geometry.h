#ifndef GEOMETRY_H
#define GEOMETRY_H
#include "../external/json.hpp"
#include <string>
#include <Eigen/Dense>

using namespace std;

class Geometry{
    public:

    Geometry(const nlohmann::json j);
    string type;
    float ka, kd, ks, pc, radius;
    vector<float> ac, dc, sc;
    Eigen::Vector3f p1, p2, p3, p4, centre, normal, AB, AC;
    Eigen::Matrix<float, 4, 4> transform;
    Eigen::Matrix<float, Eigen::Dynamic, 3> pntArray;
    int nbOfTriangle;
};

#endif