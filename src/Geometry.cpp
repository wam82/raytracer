#include "Geometry.h"
#include <Eigen/Dense>
#include <iostream>


Geometry::Geometry(const nlohmann::json geometry){
    type = geometry["type"];
    ka = geometry["ka"];
    ks = geometry["ks"];
    pc = geometry["pc"];
    kd = geometry["kd"];

    ac = {geometry["ac"][0], geometry["ac"] [1], geometry["ac"] [2]};
    dc = {geometry["dc"] [0], geometry["dc"] [1], geometry["dc"] [2]};
    sc = {geometry["sc"] [0], geometry["sc"] [1], geometry["sc"] [2]};
    // Rectangle specifics
    // if (type == "rectangle"){
    //     p1 = Eigen::Vector3f(geometry["p1"][0], geometry["p1"] [1], geometry["p1"] [2]);
    //     p2 = Eigen::Vector3f(geometry["p2"][0], geometry["p2"] [1], geometry["p2"] [2]);
    //     p3 = Eigen::Vector3f(geometry["p3"] [0], geometry["p3"] [1], geometry["p3"][2]);
    //     p4 = Eigen::Vector3f(geometry["p4"] [0], geometry["p4"] [1], geometry["p4"] [2]);
    //     normal = (p2-p1).cross(p3-p1).normalized();
    // }
    // Sphere specifics
    if (type == "sphere"){
        centre = Eigen::Vector3f(geometry["centre"] [0], geometry["centre"][1], geometry ["centre"] [2]);
        radius = geometry["radius"];
    }
    else {
        for (auto it = geometry.begin(); it != geometry.end(); ++it) {
            const std::string& key = it.key();
            if (key[0] == 'p' && key != "pc") {
                //std::cout << "key:"<< key << std::endl;
                Eigen::RowVector3f r(it.value()[0], it.value()[1], it.value()[2]);
                pntArray.conservativeResize(pntArray.rows() + 1, Eigen::NoChange);
                pntArray.row(pntArray.rows() - 1) = r;
            }
        }
       // std::cout << "array:"<< pntArray << std::endl;
        nbOfTriangle = pntArray.rows()-2;
        normal = (pntArray.row(1)-pntArray.row(0)).cross(pntArray.row(2)-pntArray.row(0)).normalized();
    }
        // Optional
    if (geometry.contains("transform"))
    {
        nlohmann::json json_array = geometry["transform"];
        for (int i = 0; i < 4; ++i) 
            for (int j = 0; j < 4; ++j)
            transform(i, j) = json_array[i][j];
    }
}
