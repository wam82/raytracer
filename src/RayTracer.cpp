#include "RayTracer.h"
#include "../external/simpleppm.h"
#include "SceneInfo.h"
#include "Ray.h"
#include <iostream>
#include <cmath>
#include <random>
#include <chrono>
#define PI 3.1415926535897932384626433

using namespace std;

// Constructor implementation
RayTracer::RayTracer(const nlohmann::json j) :sc(j){
}

// Method implementation
void RayTracer::run() {
    for (Output output: sc.out) {
        int dimX = output.size[0]; //Width of Scene
        int dimY = output.size[1]; //Height of Scene
        vector<double> buffer(3*dimX*dimY); //Colour buffer
        string fn = output.filename;//Scene Title


        for(Light l:sc.li){
            if(l.type == "area" && !l.usecenter){
                output.antialiasing = false;
                break;
            }
        }

        if(output.globalillum){
            output.antialiasing = false;
            for(Light l:sc.li){
                l.usecenter = true;
            }
        }

        if(output.globalillum){
            globalIllumination(buffer, output);
        }
        else if(output.antialiasing){
            antialiasing(buffer, output);
        }
        else{
            localIllumination(buffer, output);
        }

        std::cerr << "\nDone.\n";
        save_ppm(fn, buffer, dimX, dimY);
    }
}

void RayTracer::globalIllumination(vector<double> &buff, Output &output){
    unsigned int seed = static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count());
    std::mt19937 generator(seed);
    std::uniform_real_distribution<double> distribution(0.0, 1.0);

    int width = output.size[0];
    int height = output.size[1];

    float pxlSz = 2 * tan(output.fov * (PI/180) / 2) / height; //Pixel Size
    Eigen::Vector3f up = output.up; //Up vector
    Eigen::Vector3f side = output.lookat.cross(up).normalized(); //Side vector
    Eigen::Vector3f scrnCntr = output.centre+output.lookat; //Center of Scene

    Eigen::Matrix<float, Eigen::Dynamic, 2> distances; //Array of pairs of geometry indexes and distance to the object

    float xCoordinate = -(width*pxlSz)/2;
    float yCoordinate = (height * pxlSz / 2) - pxlSz;

    int raysPerCell = output.rpp[2];
    int numberOfColumns = output.rpp[0];
    int numberOfRows = output.rpp[1];
    int raysPerGrid = raysPerCell * numberOfColumns * numberOfRows;

    for(int y=0; y<height;y++) { //Parse through rows of pixel in scene
        Eigen::Vector3f row = up * (yCoordinate - y * pxlSz); //Row of position

        //Progression bar
        int prog = (y + 1) * 100 / height;
        std::cerr << "\rProgression: " << prog << " %" << std::flush;
        for (int x = 0; x < width; x++) { //Parse through columns of pixels in scene
            Eigen::Vector3f column = side * (xCoordinate + x * pxlSz); //Column of position

            Eigen::Vector3f pxlBtmLft = scrnCntr + row + column;

            float cellHeight = pxlSz / numberOfRows;
            float cellWidth = pxlSz / numberOfColumns;

            Eigen::Vector3f horizontalStrata = cellWidth * side;
            Eigen::Vector3f verticalStrata = cellHeight * up;

            float red = 0;
            float green = 0;
            float blue = 0;

            int raysNotCounted = 0;

            for (int cellRow = 0; cellRow < numberOfRows; ++cellRow) {
                Eigen::Vector3f verticalCellJump = cellRow * verticalStrata;

                for (int cellColumn = 0; cellColumn < numberOfColumns; ++cellColumn) {
                    Eigen::Vector3f horizontalCellJump = cellColumn * horizontalStrata;

                    Eigen::Vector3f cellBtmLft = pxlBtmLft + horizontalCellJump + verticalCellJump;
                    for (int nbOfRaysPerCell = 0; nbOfRaysPerCell < raysPerCell; ++nbOfRaysPerCell) {

                        float randomX = distribution(generator);
                        float randomY = distribution(generator);

                        Eigen::Vector3f sampledPoint = cellBtmLft + randomY * horizontalStrata + randomX * verticalStrata;

                        Ray ray = Ray(output.centre, sampledPoint - output.centre);

                        float redIntensity = 1;
                        float blueIntensity = 1;
                        float greenIntensity = 1;

                        int nbOfBounces = 0;

                        while (true) {
//                            if(y==20&&x==20&&cellRow==5&&cellColumn==5){
//                                cout<<"Bounce #"<<nbOfBounces<<" starts from "<<ray.origin<<endl;
//                            }
                            Geometry *referenceGeo = nullptr;
                            float shortestDistance = -1;

                            shortestDistance = intersectScene(ray, output, &referenceGeo);

                            if (shortestDistance < 0 && nbOfBounces == 0) {
                                //There are no objects and no bounces (i.e. the ray went straight to the void)
                                red += output.bkc[0];
                                green += output.bkc[1];
                                blue += output.bkc[2];

                                break;
                            }

                            if (shortestDistance < 0) {
                                //There are no objects to intersect with after a bounce
                                raysNotCounted++;
                                break;
                            }

                            Eigen::Vector3f pointOfIntersection = ray.origin + shortestDistance * ray.destination;
                            Eigen::Vector3f normal;
                            if (referenceGeo->type == "sphere") {
                                normal = (pointOfIntersection - referenceGeo->centre).normalized();
                            } else {
                                normal = referenceGeo->normal;
                            }

                            redIntensity *= referenceGeo->kd * referenceGeo->dc[0];
                            greenIntensity *= referenceGeo->kd * referenceGeo->dc[1];
                            blueIntensity *= referenceGeo->kd * referenceGeo->dc[2];

                            auto chanceOfTermination = distribution(generator);
                            if (nbOfBounces >= output.maxbounces || chanceOfTermination < output.probterminate) {
//                                if(nbOfBounces >= output.maxbounces){
//                                    cout<<"Ray terminated with maxbounces: "<<nbOfBounces<<endl;
//                                }
//                                if(chanceOfTermination < output.probterminate){
//                                    cout<<"Ray terminated randomly with a chance of "<<chanceOfTermination<<" and "<<nbOfBounces<<" bounces.";
//                                }
                                //Max number of bounces reached or ray is randomly terminated
                                for (Light light: sc.li) {
                                    Ray lightRay = Ray(pointOfIntersection, (light.centre - pointOfIntersection));
                                    float distanceToLight = (light.centre - pointOfIntersection).norm();

                                    Geometry *lightReferenceGeo = nullptr;
                                    float shortestLightDistance = getShortestDistance(lightRay, output,
                                                                                      &lightReferenceGeo,
                                                                                      &referenceGeo);
                                    if (!(shortestLightDistance > 0 && shortestLightDistance < distanceToLight)) {
                                        red += redIntensity * light.id[0] * max(lightRay.destination.dot(normal), 0.0f);
                                        green += greenIntensity * light.id[1] *
                                                 max(lightRay.destination.dot(normal), 0.0f);
                                        blue += blueIntensity * light.id[2] *
                                                max(lightRay.destination.dot(normal), 0.0f);
                                    }
                                }
                                break;
                            }
                            Eigen::Vector3f randomVector = Eigen::Vector3f(2 * distribution(generator) - 1,
                                                                           2 * distribution(generator) - 1,
                                                                           2 * distribution(generator) - 1);
                            randomVector.normalize();

                            Eigen::Vector3f xVector = randomVector.cross(normal);
                            Eigen::Vector3f zVector = xVector.cross(normal);

                            Eigen::Matrix3f randomMatrix;
                            randomMatrix << xVector[0], normal[0], zVector[0],
                                    xVector[1], normal[1], zVector[1],
                                    xVector[2], normal[2], zVector[2];

                            float newRandomX, newRandomY, newRandomZ;

                            while (true) {
                                //Generating a point on the hemisphere
                                newRandomX = 2 * distribution(generator) - 1;
                                newRandomZ = 2 * distribution(generator) - 1;

                                if ((newRandomX * newRandomX) + (newRandomZ * newRandomZ) <= 1) {
                                    randomY = sqrt(1 - (newRandomX * newRandomX) - (newRandomZ * newRandomZ));
                                    break;
                                }
                            }

                            Eigen::Vector3f pointOnHemisphere = Eigen::Vector3f(newRandomX, newRandomY, newRandomZ);
                            pointOnHemisphere.normalize();
//                            cout<<pointOnHemisphere<<endl;
                            pointOnHemisphere = randomMatrix * pointOnHemisphere;

                            if (pointOnHemisphere.dot(normal) < 0) {
                                pointOnHemisphere *= -1;
                            }

                            Ray bounceRay = Ray(pointOfIntersection, pointOnHemisphere);
//                            cout<<bounceRay.destination.dot(normal)<<endl;
                            float dampingCoefficient = max(normal.dot(bounceRay.destination), 0.0f);

                            redIntensity *= dampingCoefficient;
                            greenIntensity *= dampingCoefficient;
                            blueIntensity *= dampingCoefficient;

                            ray = bounceRay;
                            nbOfBounces++;
                        }
                    }
                }
            }
            float raysCounted = raysPerGrid - raysNotCounted;
            if(raysCounted == 0){
                buff[3 * y * width + 3 * x + 0] = 0;
                buff[3 * y * width + 3 * x + 1] = 0;
                buff[3 * y * width + 3 * x + 2] = 0;
                break;
            }
            red = min((red/raysCounted),1.0f);
            green = min((green/raysCounted), 1.0f);
            blue = min((blue/raysCounted),1.0f);

            buff[3 * (x + output.size[0] * y ) + 0] = (double) sqrt(red);
            buff[3 * (x + output.size[0] * y) + 1] = (double) sqrt(green);
            buff[3 * (x + output.size[0] * y) + 2] = (double) sqrt(blue);
        }
    }
}

void RayTracer::antialiasing(vector<double> &buff, Output &output){
    unsigned int seed = static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count());
    std::mt19937 generator(seed);
    std::uniform_real_distribution<double> distribution(0.0, 1.0);

    int width = output.size[0];
    int height = output.size[1];

    float pxlSz = 2 * tan(output.fov * (PI/180) / 2) / height; //Pixel Size
    Eigen::Vector3f up = output.up; //Up vector
    Eigen::Vector3f side = output.lookat.cross(up).normalized(); //Side vector
    Eigen::Vector3f scrnCntr = output.centre+output.lookat; //Center of Scene

    Eigen::Matrix<float, Eigen::Dynamic, 2> distances; //Array of pairs of geometry indexes and distance to the object

    float xCoordinate = -(width*pxlSz)/2;
    float yCoordinate = (height * pxlSz / 2) - pxlSz;

    int raysPerCell = output.rpp[2];
    int numberOfColumns = output.rpp[0];
    int numberOfRows = output.rpp[1];
    int raysPerGrid = raysPerCell * numberOfColumns * numberOfRows;

    for(int y=0; y<height;y++) { //Parse through rows of pixel in scene
        Eigen::Vector3f row = up * (yCoordinate - y * pxlSz); //Row of position

        //Progression bar
        int prog = (y + 1) * 100 / height;
        std::cerr << "\rProgression: " << prog << " %" << std::flush;
        for (int x = 0; x < width; x++) { //Parse through columns of pixels in scene
            Eigen::Vector3f column = side*(xCoordinate + x * pxlSz); //Column of position

            Eigen::Vector3f pxlBtmLft = scrnCntr + row + column;

            float cellHeight = pxlSz / numberOfRows;
            float cellWidth = pxlSz / numberOfColumns;

            Eigen::Vector3f horizontalStrata = cellWidth * side;
            Eigen::Vector3f verticalStrata = cellHeight * up;

            double red = 0;
            double green = 0;
            double blue = 0;

            for (int cellRow = 0; cellRow < numberOfRows; ++cellRow) {
                Eigen::Vector3f verticalCellJump = cellRow * verticalStrata;

                for (int cellColumn = 0; cellColumn < numberOfColumns; ++cellColumn) {
                    Eigen::Vector3f horizontalCellJump = cellColumn * horizontalStrata;

                    Eigen::Vector3f cellBtmLft = pxlBtmLft + horizontalCellJump + verticalCellJump;

                    for (int nbOfRaysPerCell = 0; nbOfRaysPerCell < raysPerCell; ++nbOfRaysPerCell) {

                        float randomX = distribution(generator);
                        float randomY = distribution(generator);

                        Eigen::Vector3f sampledPoint = cellBtmLft + randomY * horizontalStrata + randomX * verticalStrata;

                        Ray ray = Ray(output.centre, sampledPoint - output.centre);

                        Geometry *referenceGeo = nullptr;
                        float shortestDistance = -1;

                        shortestDistance = intersectScene(ray, output, &referenceGeo);
                        if (referenceGeo != nullptr) {
                            Eigen::Vector3f point = ray.origin + shortestDistance * ray.destination;
                            Eigen::Vector3f normal;
                            if (referenceGeo->type == "sphere") {
                                normal = (point - referenceGeo->centre).normalized();
                            } else {
                                normal = referenceGeo->normal;
                            }
                            red+=output.ai[0]*referenceGeo->ac[0]*referenceGeo->ka;
                            green+=output.ai[1]*referenceGeo->ac[1]*referenceGeo->ka;
                            blue+=output.ai[2]*referenceGeo->ac[2]*referenceGeo->ka;

                            for (Light light: sc.li) {
                                Ray lightRay = Ray(point, (light.centre - point));
                                Geometry *lightReferenceGeo = nullptr;
                                float shortestLightDistance = getShortestDistance(lightRay, output, &lightReferenceGeo,
                                                                                  &referenceGeo);
                                if (shortestLightDistance > 0 &&
                                    shortestLightDistance < (light.centre - point).norm()) {
                                    continue;
                                }

                                float diffuse = referenceGeo->kd * max(0.0f, normal.dot(lightRay.destination));
                                red += light.id[0] * referenceGeo->dc[0] * diffuse;
                                green += light.id[1] * referenceGeo->dc[1] * diffuse;
                                blue += light.id[2] * referenceGeo->dc[2] * diffuse;

                                float specular = (2 * normal * (normal.dot(lightRay.destination)) -
                                                  lightRay.destination).dot(-ray.destination);
                                specular = referenceGeo->ks * pow(max(0.0f, specular), referenceGeo->pc);
                                red += light.is[0] * referenceGeo->sc[0] * specular;
                                green += light.is[1] * referenceGeo->sc[1] * specular;
                                blue += light.is[2] * referenceGeo->sc[2] * specular;
                            }
                        }
                        else{
                            red+=output.bkc[0];
                            green+=output.bkc[1];
                            blue+=output.bkc[2];
                        }
                    }
                }
            }
            red = red/raysPerGrid;
            green = green/raysPerGrid;
            blue = blue/raysPerGrid;
            buff[3 * (x + output.size[0] * y ) + 0] = min(red,1.0);
            buff[3 * (x + output.size[0] * y) + 1] = min(green,1.0);
            buff[3 * (x + output.size[0] * y) + 2] = min(blue,1.0);
        }
    }
}
void RayTracer::localIllumination(vector<double> &buff, Output &output){
    int dimX = output.size[0]; //Width of Scene
    int dimY = output.size[1]; //Height of Scene
    //First fill with a background
    fill(buff,output.bkc);

    float pxlSz = (2*tan((output.fov*PI/180)/2))/dimY; //Size of one pixel
    Eigen::Vector3f scrnCntr = output.centre+output.lookat; //Center of Scene
    float yCo = (dimY*pxlSz-pxlSz)/2; //Y coefficient
    float xCo = -(dimX*pxlSz-pxlSz)/2; //X Coordinate
    Eigen::Vector3f up = output.up; //Up Vector
    Eigen::Vector3f side = output.lookat.cross(up).normalized(); //???

    Eigen::Matrix<float, Eigen::Dynamic, 2> distances; //Array of pairs of geometry indexes and distance to the object

    for(int y=0; y<dimY;y++){ //Parse through rows of pixel in scene
        int prog = (y + 1) * 100 / dimY;
        std::cerr << "\rProgression: " << prog << " %" << std::flush;
        auto row = up*(yCo-y*pxlSz); //Rows
        for(int x=0; x<dimX;x++){ //Parse through columns of pixels in scene
            auto column = side*(xCo+x*pxlSz); //Columns
            auto pxlCntr = scrnCntr+row+column; //Center of pixels
            if(x==0&&y==0) {
                auto btmX = pxlCntr.x()+pxlSz/2;
                auto btmY = pxlCntr.y()-pxlSz/2;
                Eigen::Vector3f pxlBtmLft;
                pxlBtmLft.x() = btmX;
                pxlBtmLft.y() = btmY;
                pxlBtmLft.z() = pxlCntr.z();

            }

            auto v = (pxlCntr-output.centre); //Ray direction
            Ray ray = Ray(output.centre, v); //Ray instantiation
            Geometry *referenceGeo = nullptr;
            float shortestDistance=-1;

            shortestDistance = intersectScene(ray, output, &referenceGeo);
            if (referenceGeo != nullptr){
                //Set ambient light as basis
                double red = 0;
                double green = 0;
                double blue = 0;

                Eigen::Vector3f point = ray.origin + shortestDistance*ray.destination;
                Eigen::Vector3f normal;
                if(referenceGeo->type=="sphere"){
                    normal = (point - referenceGeo->centre).normalized();
                }
                else {
                    normal = referenceGeo->normal;
                }

                for(Light light : sc.li){
                    if(light.type=="point" || light.usecenter){
                        Ray lightRay = Ray(point, (light.centre-point));
                        Geometry *lightReferenceGeo = nullptr;
                        float shortestLightDistance = getShortestDistance(lightRay, output, &lightReferenceGeo,
                                                                          &referenceGeo);

                        if(shortestLightDistance>0 && shortestLightDistance<(light.centre-point).norm()){
                            continue;
                        }

                        float diffuse = referenceGeo->kd * max(0.0f, normal.dot(lightRay.destination));
                        red += light.id[0]*referenceGeo->dc[0]*diffuse;
                        green += light.id[1]*referenceGeo->dc[1]*diffuse;
                        blue += light.id[2]*referenceGeo->dc[2]*diffuse;

                        float specular = (2*normal*(normal.dot(lightRay.destination))-lightRay.destination).dot(-ray.destination);
                        specular = referenceGeo->ks*pow(max(0.0f, specular), referenceGeo->pc);
                        red += light.is[0]*referenceGeo->sc[0]*specular;
                        green += light.is[1]*referenceGeo->sc[1]*specular;
                        blue += light.is[2]*referenceGeo->sc[2]*specular;
                    }
                    else{
                        //Random seed generating shenanigans
                        auto seed = static_cast<unsigned int>(chrono::system_clock::now().time_since_epoch().count());
                        mt19937 gen(seed);
                        uniform_real_distribution<float> randomGenerator(0.0f, 1.0f);

                        for (int areaRow = 0; areaRow < light.n; ++areaRow) {
                            for (int areaColumn = 0; areaColumn < light.n; ++areaColumn) {
                                Eigen::Vector3f bottomLeftPnt = light.p2+areaColumn*light.areaCellWidth+areaRow*light.areaCellHeight;

                                Eigen::Vector3f rndmCellPnt
                                = bottomLeftPnt+randomGenerator(gen)*light.areaCellWidth+randomGenerator(gen)*light.areaCellHeight;

                                Ray lightRay = Ray(point, (rndmCellPnt-point));
                                Geometry *lightReferenceGeo = nullptr;
                                float shortestLightDistance = getShortestDistance(lightRay, output, &lightReferenceGeo,
                                                                                  &referenceGeo);

                                if(shortestLightDistance>0 && shortestLightDistance<(rndmCellPnt-point).norm()){
                                    continue;
                                }

                                float diffuse = referenceGeo->kd * max(0.0f, normal.dot(lightRay.destination));
                                red += light.id[0]*referenceGeo->dc[0]*diffuse;
                                green += light.id[1]*referenceGeo->dc[1]*diffuse;
                                blue += light.id[2]*referenceGeo->dc[2]*diffuse;

                                float specular = (2*normal*(normal.dot(lightRay.destination))-lightRay.destination).dot(-ray.destination);
                                specular = referenceGeo->ks*pow(max(0.0f, specular), referenceGeo->pc);
                                red += light.is[0]*referenceGeo->sc[0]*specular;
                                green += light.is[1]*referenceGeo->sc[1]*specular;
                                blue += light.is[2]*referenceGeo->sc[2]*specular;
                            }
                        }
                        red = red/(light.n*light.n);
                        green = green/(light.n*light.n);
                        blue = blue/(light.n*light.n);
                    }
                }
                red+=output.ai[0]*referenceGeo->ac[0]*referenceGeo->ka;
                green+=output.ai[1]*referenceGeo->ac[1]*referenceGeo->ka;
                blue+=output.ai[2]*referenceGeo->ac[2]*referenceGeo->ka;
                buff[3 * (x + output.size[0] * y) + 0] = min(red,1.0);
                buff[3 * (x + output.size[0] * y) + 1] = min(green,1.0);
                buff[3 * (x + output.size[0] * y) + 2] = min(blue,1.0);
            }
        }
    }
}

float RayTracer::intersectScene(Ray ray, Output &output, Geometry **referenceGeo){
    float shortestDistance=-1;
    for(int g=0;g<sc.geo.size();g++) { //Parse through all geometries
        float objectDistance=-1;
        if(sc.geo[g].type=="sphere"){
            objectDistance = renderSphere(ray, g, output);
        }
        else{
            objectDistance = renderPolygon(ray, g);
        }

        if(objectDistance > 0 && (shortestDistance < 0 || objectDistance < shortestDistance) && objectDistance > 1e-4) {
            shortestDistance = objectDistance;
            *referenceGeo = &sc.geo[g];
        }
    }
    return shortestDistance;
}

float RayTracer::getShortestDistance(Ray r, Output &out, Geometry **geo, Geometry **currentGeo){
    float distance=-1;
    for(int g=0;g<sc.geo.size();g++) { //Parse through all geometries
        float objectDistance=-1;
        if(*currentGeo==&sc.geo[g]){
            continue;
        }
        if(sc.geo[g].type=="sphere"){
            objectDistance = renderSphere(r, g, out);
        }
        else{
            objectDistance = renderPolygon(r, g);
        }
        if(objectDistance > 0 && (distance < 0 || objectDistance < distance)) {
            distance = objectDistance;
            *geo = &sc.geo[g];
        }
    }
    return distance;
}

float RayTracer::renderPolygon(Ray r, int activeGeo){
    double SBC, SCA, SAB;
    float t;
    int counter=0;
    Eigen::Vector3f A; //Point A
    Eigen::Vector3f B; //Point B
    Eigen::Vector3f C; //Point C


        counter=0; //Counter for the number of points in one geometry
        for(int i=0; i<sc.geo[activeGeo].nbOfTriangle;i++) { //Parse through all triangles of geometry
            //Point p is in the plane if (P-A).dot(N)=0
            //In other terms, (O+t*d-A).dot(N)=0 --> t= (A.dot(N) - O.dot(N))/(d.dot(N))
            if (r.destination.dot(sc.geo[activeGeo].normal) != 0) {
                A = sc.geo[activeGeo].pntArray.row(counter);
                B = sc.geo[activeGeo].pntArray.row(counter + 1);
                C = sc.geo[activeGeo].pntArray.row(sc.geo[activeGeo].pntArray.rows() - 1);

                t = (A.dot(sc.geo[activeGeo].normal) - r.origin.dot(sc.geo[activeGeo].normal)) /
                    (r.destination.dot(sc.geo[activeGeo].normal));

                auto P = r.origin + t * r.destination; //Point P
                auto dotBC = (C - B).cross(P - B).dot(sc.geo[activeGeo].normal);
                SBC = signbit(dotBC);
                auto dotCA = (A - C).cross(P - C).dot(sc.geo[activeGeo].normal);
                SCA = signbit(dotCA);
                auto dotAB = (B - A).cross(P - A).dot(sc.geo[activeGeo].normal);
                SAB = signbit(dotAB);
                counter++;
                if (SBC == SAB && SBC == SCA && SAB == SCA) { //If all signs are equal that we are in the plane
                    return t;
                }
            } else {
                //Error: the ray is parallel to the plane or lies on the plane, and there might be no intersection
                // or infinite intersections.
            }
        }
        return -1;
}

float RayTracer::renderSphere(Ray r, int activeGeo, Output &output){
    //Sphere Intersection
    float t;
    auto OC = r.origin-sc.geo[activeGeo].centre;
    float a = r.destination.dot(r.destination);
    float b = 2*OC.dot(r.destination);
    float c = OC.dot(OC)-sc.geo[activeGeo].radius*sc.geo[activeGeo].radius;
    //Big Bertha
    float discriminant, real, imaginary;
    discriminant = b*b - 4*a*c;
    if(discriminant==0){
        //Only one root
        t=-b/(2*a);
        return t;
    }
    else if(discriminant>0){
        //Two roots
        float x1, x2;
        x1 = (-b + sqrt(discriminant)) / (2*a);
        x2 = (-b - sqrt(discriminant)) / (2*a);
        float result = min(x1, x2);
        if (result > 0){
            return result;
        }
        return -1;
    }
    else{
        //No root
        return -1;
    }
}

void RayTracer::fill(vector<double> &buff, vector<float> color){
    for(int i=0; i<buff.size();i++){
        buff[i]=color[i%3];
    }
}


//void RayTracer::antialiasing(vector<double> &buff, Output &output) {
//    int dimX = output.size[0]; //Width of Scene
//    int dimY = output.size[1]; //Height of Scene
//    //First fill with a background
//    fill(buff, output.bkc);
//
//    auto seed = static_cast<unsigned int>(chrono::system_clock::now().time_since_epoch().count());
//    mt19937 gen(seed);
//    uniform_real_distribution<float> randomGenerator(0.0f, 1.0f);
//
//
//    float pxlSz = (2*tan((output.fov*PI/180)/2))/dimY; //Size of one pixel
//    cout<<"AA pxlSz: "<<pxlSz<<endl;
//    Eigen::Vector3f scrnCntr = output.centre+output.lookat; //Center of Scene
////    cout<<"AA scrnCntr: "<<scrnCntr<<endl;
//    float yCo = (dimY*pxlSz)/2-pxlSz ; //Y coordinate
//    float xCo = -(dimX*pxlSz)/2; //X coordinate
//    Eigen::Vector3f up = output.up; //Up Vector
//    Eigen::Vector3f side = output.lookat.cross(up).normalized(); //???
//    Eigen::Vector3f cellBtmLft;
//
//    Eigen::Matrix<float, Eigen::Dynamic, 2> distances; //Array of pairs of geometry indexes and distance to the object
//
//    for (int y = 0; y < dimY; y++) { //Parse through rows of pixel in scene
//        int prog = (y + 1) * 100 / dimY;
//        std::cerr << "\rProgression: " << prog << " %" << std::flush;
//        Eigen::Vector3f row = up*(yCo-y*pxlSz); //Rows
//        for(int x = 0; x < dimX; x++){ //Parse through columns of pixels in scene
//            Eigen::Vector3f column = side*(xCo+x*pxlSz); //Columns
//            Eigen::Vector3f pxlCntr = scrnCntr+row+column; //Center of pixels
//            auto btmX = scrnCntr.x()+pxlSz/2;
//            auto btmY = pxlCntr.y()-pxlSz/2;
//            Eigen::Vector3f pxlBtmLft;
//            pxlBtmLft.x() = pxlCntr.x();
//            pxlBtmLft.y() = btmY;
//            pxlBtmLft.z() = pxlCntr.z();
////            Eigen::Vector3f pxlBtmLft = up*(yCo*pxlSz+0.076196-y*pxlSz)+scrnCntr+column;
////            Eigen::Vector3f pxlBtmLft = pxlCntr;
//            //pxlBtmLft.y() = pxlBtmLft.y() -0;
//            if(y==0&&x==0){
//
//
//                cout<<"AA Top Left pxl, pxlCntr: "<<pxlCntr<<endl;
//                cout<<"AA Top Left pxl, btm lft: "<<pxlBtmLft<<endl;
//            }
//            double red = 0;
//            double green = 0;
//            double blue = 0;
//
//            for (int gridY = 0; gridY < output.rpp[1]; ++gridY) {//Column of grid inside pixel
//                for (int gridX = 0; gridX < output.rpp[0]; ++gridX) {
//                    cellBtmLft = pxlBtmLft+ gridX * (pxlSz / output.rpp[0]) * side + gridY * (pxlSz / output.rpp[1]) * up;
//                    if(y==0&&x==0&&gridY==0&&gridX==0){
//                        //cout<<"AA Top Left pxl, bottom left corner: "<<cellBtmLft<<endl;
//                    }
//                    for (int nbOfRayPerCell = 0; nbOfRayPerCell < output.rpp[2]; ++nbOfRayPerCell) {
//                        auto rayDestination = cellBtmLft+ randomGenerator(gen) * ((pxlSz / output.rpp[1]) * up) +
//                                              randomGenerator(gen) * ((pxlSz / output.rpp[0]) * side);
//                        Ray ray(output.centre, rayDestination);
//
//                        Geometry *referenceGeo = nullptr;
//                        float shortestDistance = -1;
//
//                        shortestDistance = intersectScene(ray, output, &referenceGeo);
//                        if (referenceGeo != nullptr) {
//                            Eigen::Vector3f point = ray.origin + shortestDistance * ray.destination;
//                            Eigen::Vector3f normal;
//                            if (referenceGeo->type == "sphere") {
//                                normal = (point - referenceGeo->centre).normalized();
//                            } else {
//                                normal = referenceGeo->normal;
//                            }
//                            red+=output.ai[0]*referenceGeo->ac[0]*referenceGeo->ka;
//                            green+=output.ai[1]*referenceGeo->ac[1]*referenceGeo->ka;
//                            blue+=output.ai[2]*referenceGeo->ac[2]*referenceGeo->ka;
//
//                            for (Light light: sc.li) {
//                                Ray lightRay = Ray(point, (light.centre - point));
//                                Geometry *lightReferenceGeo = nullptr;
//                                float shortestLightDistance = getShortestDistance(lightRay, output, &lightReferenceGeo,
//                                                                       &referenceGeo);
//                                if (shortestLightDistance > 0 &&
//                                    shortestLightDistance < (light.centre - point).norm()) {
//                                    continue;
//                                }
//
//                                float diffuse = referenceGeo->kd * max(0.0f, normal.dot(lightRay.destination));
//                                red += light.id[0] * referenceGeo->dc[0] * diffuse;
//                                green += light.id[1] * referenceGeo->dc[1] * diffuse;
//                                blue += light.id[2] * referenceGeo->dc[2] * diffuse;
//
//                                float specular = (2 * normal * (normal.dot(lightRay.destination)) -
//                                                  lightRay.destination).dot(-ray.destination);
//                                specular = referenceGeo->ks * pow(max(0.0f, specular), referenceGeo->pc);
//                                red += light.id[0] * referenceGeo->sc[0] * specular;
//                                green += light.id[1] * referenceGeo->sc[1] * specular;
//                                blue += light.id[2] * referenceGeo->sc[2] * specular;
//                            }
//                        }
//                        else{
//                            red+=output.bkc[0];
//                            green+=output.bkc[1];
//                            blue+=output.bkc[2];
//                        }
//                    }
//                }
//            }
//            red = red/(output.rpp[0]*output.rpp[1]*output.rpp[2]);
//            green = green/(output.rpp[0]*output.rpp[1]*output.rpp[2]);
//            blue = blue/(output.rpp[0]*output.rpp[1]*output.rpp[2]);
//            buff[3 * (x + output.size[0] * y ) + 0] = min(red,1.0);//height - y -1
//            buff[3 * (x + output.size[0] * y) + 1] = min(green,1.0);
//            buff[3 * (x + output.size[0] * y) + 2] = min(blue,1.0);
//        }
//    }
//}


