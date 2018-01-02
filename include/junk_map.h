//
// Created by bohuan on 18-1-2.
//

#ifndef MAP_CORE_JUNK_MAP_H
#define MAP_CORE_JUNK_MAP_H

#include "cloud2map.h"
#include "common.h"

namespace junk_map {

    Cloud2Map cloud2map;

    const std::vector<int> compression_params(CV_IMWRITE_PNG_COMPRESSION, 100);

    Eigen::Matrix4d returnEigen(double th = 0) {
        Eigen::Matrix4d T;
        T << cos(th), -sin(th), 0, 0,
                sin(th), cos(th), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
        return std::move(T);
    }

    Eigen::Matrix4d json2Mat(Json::Value root) {
        Eigen::Matrix4d ret;
        std::stringstream x;
        x << root["transformational_matrix"].asString().c_str();
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
            {
                double tmp;
                x >> tmp;
                ret(i, j) = tmp;
            }
        return std::move(ret);
    }

    Json::Value mat2Json(Eigen::Matrix4d mat) {
        Json::Value root;
        std::stringstream trans_mat;
        trans_mat << mat;
        return root["transformational_matrix"] = trans_mat.str();
    }

    const std::string getCurrentSystemTime()
    {
        struct timeval s_now;
        struct tm* p_tm;
        gettimeofday( &s_now,NULL );
        p_tm = localtime( ( const time_t* )&s_now.tv_sec );
        char date[ 60 ] = { 0 };
        sprintf(date, "%d-%02d-%02d      %02d:%02d:%02d",
                ( int )p_tm->tm_year + 1900,( int )p_tm->tm_mon + 1,( int )p_tm->tm_mday,
                ( int )p_tm->tm_hour,( int )p_tm->tm_min,( int )p_tm->tm_sec);
        return std::move( std::string( date ) );
    }
}

#endif //MAP_CORE_JUNK_MAP_H
