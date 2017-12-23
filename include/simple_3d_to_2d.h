//
// Created by bohuan on 17-12-19.
//

#ifndef SIMPLE_3D_TO_2D_SIMPLE_3D_TO_2D_H
#define SIMPLE_3D_TO_2D_SIMPLE_3D_TO_2D_H




#include "pointmatcher/PointMatcher.h"
#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include "common.h"

//just for debug
//#define pr(x)   cout<<#x<<"  = "<<x<<" "
//#define prln(x)   cout<<#x<<"  = "<<x<<endl

//#define IMSHOW




//using namespace PointMatcherSupport;

//typedef PointMatcher<double> PM;
//typedef PointMatcher<double>::DataPoints DP;

#define DP PointMatcher<double>::DataPoints

class Simple3DTo2D {
private:
    cv::Mat mat_;
    //   DP data_points_;
    std::vector<geometry_msgs::Point> points_;
    /*
    double min_x_=-1;
    double max_x_=1;
    double min_y_=-1;
    double max_y_=1;
    int X_;
    int Y_;
    unsigned char flag_ = 0;//初始化了多少个变量
     */
    BoundingBox bounding_box_;

public:

    Simple3DTo2D() {}

    void setDefault() {
        //TODO
    }

    void setBoundingBox(BoundingBox bounding_box)
    {
        bounding_box_ = bounding_box;
    }

    void solve(const DP & dp, cv::Mat &mat, std::function<bool(geometry_msgs::Point)> isVaild =
    [](geometry_msgs::Point point)->bool {
        return true;
        //判断一个point是否合法，合法返回true,否则返回false. 可以简单的滤掉很多点
        point.z -= 0.75;
        if (point.z < 0.2)    return false;
        if (point.z > 1.5)    return false;
        return true;
    });//= BoundingBox(-200,200,-200,200,1000,1000) );

};


#undef DP

#endif //SIMPLE_3D_TO_2D_SIMPLE_3D_TO_2D_H
