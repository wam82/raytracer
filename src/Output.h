#ifndef OUTPUT_H
#define OUTPUT_H
#include "../external/json.hpp"
#include <string>
#include <Eigen/Dense>

using namespace std;

class Output{
    public:

    Output(const nlohmann::json j);
    string filename;
    vector<unsigned int>size, raysperpixel;
    float fov, maxbounces, probterminate;
    vector<float> ai, bkc;
    Eigen::Vector3f centre, up, lookat;
    bool antialiasing, twosiderender, globalillum;
    int rpp[3];
};
#endif