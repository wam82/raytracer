#ifndef RAY_H
#define RAY_H
#include "SceneInfo.h"


class Ray{
    public:
    Ray(Eigen::Vector3f origin, Eigen::Vector3f destination);
    Eigen::Vector3f origin;
    Eigen::Vector3f destination;
};
#endif