#ifndef SCENEINFO_H
#define SCENEINFO_H
#include "Output.h"
#include "Light.h"
#include "Geometry.h"


class SceneInfo{
    public:

    SceneInfo(const nlohmann::json j);
    vector<Geometry> geo;
    vector<Light> li;
    vector<Output> out;
};
#endif