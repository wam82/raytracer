#include "Output.h"
#include <Eigen/Dense>


Output::Output(const nlohmann::json out){
    filename = out["filename"];
    size = {out["size"][0], out["size"][1]};
    fov = out["fov"];
    ai = {out["ai"][0], out["ai"][1], out["ai"][2]};
    bkc = {out["bkc"][0], out["bkc"][1], out["bkc"][2]};
    centre = Eigen::Vector3f(out["centre"][0], out["centre"] [1], out["centre"] [2]);
    up = Eigen::Vector3f(out["up"][0], out["up"] [1], out["up"] [2]).normalized();
    lookat = Eigen::Vector3f(out["lookat"][0], out["lookat"] [1], out["lookat"] [2]).normalized();

    //Optional
    if(out.contains("raysperpixel")){
        for(unsigned int k:out["raysperpixel"]) {
            raysperpixel.push_back(k);
        }
        rpp[0]=1;
        rpp[1]=1;
        rpp[2]=1;
        if(raysperpixel.size()==1){
            rpp[0]=1;
            rpp[1]=1;
            rpp[2]=raysperpixel[0];
        }
        if(raysperpixel.size()==2){
            rpp[0]=raysperpixel[0];
            rpp[1]=raysperpixel[0];
            rpp[2]=raysperpixel[1];
        }
        if(raysperpixel.size()==3){
            rpp[0]=raysperpixel[0];
            rpp[1]=raysperpixel[1];
            rpp[2]=raysperpixel[2];
        }
    }
    if (out.contains("antialiasing")){
        antialiasing = out["antialiasing"];
    }
    if (out.contains("twosiderender")){
        twosiderender = out["twosiderender"];
    }
    if (out.contains("globalillum")){
        globalillum = out["globalillum"];
    }
    if (out.contains("maxbounces")){
        maxbounces = out["maxbounces"];
    }
    if (out.contains("probterminate")){
        probterminate = out["probterminate"];
    }
}

// raysperpixel,
// antialiasing,
// twosiderender,
// globalillum