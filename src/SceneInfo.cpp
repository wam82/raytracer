#include "SceneInfo.h"

SceneInfo::SceneInfo(const nlohmann::json sc){
    for(nlohmann::json k:sc["geometry"]){
        if(k.contains("visible")) {
            if(k["visible"] == false) {
                continue;
            }
        }
        geo.push_back(Geometry(k));
    }
    for(nlohmann::json l:sc["light"]) {
        if (l.contains("use")) {
            if (l["use"] == false) {
                continue;
            }
        }
        li.push_back(Light(l));
    }
    for(nlohmann::json o:sc["output"])
            out.push_back(Output(o));
}