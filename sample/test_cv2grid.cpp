//
// Created by bohuan on 17-12-21.
//

#include <bits/stdc++.h>
#include "cvMat2GridMap.h"

#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cv2grid");
    ros::NodeHandle nh;
    //ros::Subscriber sub = nh.subscribe<nav_msgs::OccupancyGrid_>("/voxel_filter", 5, callback);
    ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("cost_map", 1);
    nav_msgs::OccupancyGrid og;
    CVMat mat;
    mat() = cv::imread("/home/bohuan/下载/52.png");

    CvMat2GridMap::cvMat2GridMap(mat, og);

//貌似程序不对

    cv::imshow("abc", mat());
    cv::namedWindow("abc", 0);
    cv::waitKey(-1);


    ros::Rate r(10);

    while (ros::ok())
    {
        pub.publish(og);
        r.sleep();
    }
}

