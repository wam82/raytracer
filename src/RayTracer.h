// RayTracer.h

#ifndef RAYTRACER_H
#define RAYTRACER_H

#include "../external/json.hpp"
#include "SceneInfo.h"
#include "Ray.h"
#include <Eigen/Dense>

class RayTracer {
public:
    // Constructor that takes a nlohmann::json object
    RayTracer(const nlohmann::json j);
    SceneInfo sc;
    // Method to run the ray tracer
    void run();
    void fill(vector<double> &buff, vector<float> color);
    float renderSphere(Ray r, int activeGeo, Output &output);
    float renderPolygon(Ray r, int activeGeo);
    void localIllumination(vector<double> &buff, Output &output);
    float intersectScene(Ray ray, Output &output, Geometry **referenceGeo);
    float getShortestDistance(Ray r, Output &out, Geometry **geo, Geometry **currentGeo);
    void antialiasing(vector<double> &buff, Output &output);
    void globalIllumination(vector<double> &buff, Output &output);
};

#endif // RAYTRACER_H
