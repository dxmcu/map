//
// Created by bohuan on 17-12-21.
//

#include <bits/stdc++.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

#include <opencv2/opencv.hpp>
#include <pointmatcher/PointMatcher.h>
#include "cloud2map.h"

using std::cin;
using std::endl;
using std::cout;

Cloud2Map cm;

int main(int argc, char** argv)
{


    ros::init(argc, argv, "cloud_to_map_node");
    ros::NodeHandle nh;
    ros::Rate r(10);

    ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);
    cm.loadPointCloudFromPCL("/home/bohuan/map_data/pcd/3.pcd");
    cm.PCL2Map();
    auto tmp = cm.occupancy_grid();

    pub.publish(tmp);
    pub.publish(tmp);
    pub.publish(tmp);
    pub.publish(tmp);
    pub.publish(tmp);
    pub.publish(tmp);
    pub.publish(tmp);

    auto cvmat = cm.getCVMat();
    cv::namedWindow("abc",0);
    cv::imshow("abc",cvmat.mat);
    cv::waitKey(-1);


    while (ros::ok())
    {
        pub.publish(tmp);
        r.sleep();
    }

}

