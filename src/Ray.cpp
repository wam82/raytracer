#include "Ray.h"

Ray::Ray(Eigen::Vector3f o, Eigen::Vector3f d){
    origin = o;
    destination = d.normalized();
}