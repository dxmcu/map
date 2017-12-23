
#include <bits/stdc++.h>
#include "simple_3d_to_2d.h"
#include "common.h"

#define DP PointMatcher<double>::DataPoints



void Simple3DTo2D::solve(const DP & dp, cv::Mat &mat, std::function<bool(geometry_msgs::Point)> isVaild)
{

    int X = bounding_box_.X();
    int Y = bounding_box_.Y();

    mat = cv::Mat(X, Y, CV_8U);   //init the cv::Mat
    for (int i = 0; i < X; ++ i)
        for (int j = 0; j < Y; ++ j)   mat.at<uchar>(i, j) = 0;

    const auto &points = dp.features;
    points_.clear();
    int cnt=0;
    for (int i = 0; i < points.cols(); ++ i)
    {
        points_.push_back(geometry_msgs::Point());
        auto &tmp = points_[ points_.size() - 1];//.rbegin();
        tmp.x = points(0, i);
        tmp.y = points(1, i);
        tmp.z = points(2, i);
    }

    for (auto & point : points_)
    {
        if ( !bounding_box_.isVaild( point ))    continue;
        if ( !isVaild( point ) )  continue;
        auto p = bounding_box_.point2XY( point );
        int &x = p.first, &y = p.second;
        mat.at<uchar>(x, y) = 255;
    }
#ifdef IMSHOW
    std::string windows_name = "simple 3d to 2d";
    cv::namedWindow(windows_name.c_str(), 0);
    cv::imshow(windows_name.c_str(), mat);
    cv::waitKey(-1);
#endif
}

#undef DP
