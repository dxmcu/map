//
// Created by bohuan on 18-1-2.
//

#ifndef MAP_CORE_GPS_H
#define MAP_CORE_GPS_H


#include <json/json.h>
#include "common.h"
#include "junk_map.h"

struct GPS
{
    double latitude = -1;
    double longtitude = -1;
    double altitude = -1;
    Eigen::Matrix4d transformational_matrix = junk_map::returnEigen();

    Json::Value json()
    {
        Json::Value root;
        root["latitude"] = latitude;
        root["longtitude"] = longtitude;
        root["altitude"] =  altitude;
        root["transformational_matrix"] = junk_map::mat2Json(transformational_matrix);
        return std::move( root );
    }

    void loadFromJson(Json::Value root)
    {
        latitude = root["latitude"].asDouble();
        longtitude = root["longtitude"].asDouble();
        altitude = root["altitude"].asDouble();
        transformational_matrix = junk_map::json2Mat(root);
    }
};

#endif //MAP_CORE_GPS_H
