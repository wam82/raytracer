#include "Light.h"
#include <Eigen/Dense>


Light::Light(const nlohmann::json light){
    type = light["type"];
    id = {light["id"][0], light["id"] [1], light["id"] [2]};
    is = {light["is"][0], light["is"] [1], light["is"] [2]};
    n = 1;

    if (light.contains("n")){
        n = light["n"];
    }
    if (type == "point"){
        centre = Eigen::Vector3f(light["centre"][0], light["centre"][1], light["centre"][2]);
    }
    if (type == "area"){
        p1 = Eigen::Vector3f(light["p1"][0], light["p1"] [1], light["p1"] [2]);
        p2 = Eigen::Vector3f(light["p2"][0], light["p2"] [1], light["p2"] [2]);
        p3 = Eigen::Vector3f(light["p3"] [0], light["p3"] [1], light["p3"][2]);
        p4 = Eigen::Vector3f(light["p4"] [0], light["p4"] [1], light["p4"] [2]);
        centre = (p1+p2+p3+p4)/4.0;

        float sizeX = (p3-p2).norm()/n;
        areaCellWidth = sizeX*(p3-p2).normalized();

        float sizeY = (p1-p2).norm()/n;
        areaCellHeight = sizeY*(p1-p2).normalized();
    }
        // Optional
    if (light.contains("transform")) {
        auto json_array = light["transform"];
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                transform(i, j) = json_array[i][j];
    }
    if (light.contains("usecenter")){
        usecenter = light["usecenter"];
    }
}
