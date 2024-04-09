#ifndef LIGHT_H
#define LIGHT_H
#include "../external/json.hpp"
#include <string>
#include <Eigen/Dense>

using namespace std;

class Light{
    public:

    Light(const nlohmann::json j);
    string type;
    vector<float> id, is;
    Eigen::Vector3f p1, p2, p3, p4, centre, areaCellWidth, areaCellHeight;
    Eigen::Matrix<float, 4, 4> transform;
    unsigned int n;
    bool usecenter;
};
#endif