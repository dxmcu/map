//
// Created by bohuan on 17-12-22.
//

#ifndef MAP_SHOWPOINTCLOUD_H
#define MAP_SHOWPOINTCLOUD_H

#include "pointmatcher/PointMatcher.h"
#include "common.h"
#include <bits/stdc++.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <ros/ros.h>


#define PM PointMatcher<double>
#define DP PointMatcher<double>::DataPoints

class ShowPointCloud_LPM
{
private:
    DP data_;
    DP old_data_;
    std::vector< geometry_msgs::Point32 > point32s_;
    sensor_msgs::PointCloud point_cloud_;
    std::string topic_name_;
    ros::NodeHandle* node_handle_;
    ros::Publisher pub;// = nh.advertise<sensor_msgs::PointCloud>(argv[2], 1);

public:

    DP& dataRef()   {   return data_;   }

    ShowPointCloud_LPM(std::string topic_name, std::string file_, ros::NodeHandle &node_handle)
    {
        //topic_name_ = topic_name;
        data_ = DP::load(file_.c_str());


        //node_handle = &node_handle_;
        pub = node_handle.advertise<sensor_msgs::PointCloud>(topic_name.c_str(), 1);
    }

    void filter(const std::string & ymal)
    {
        std::ifstream  ifs( ymal.c_str() );
        PM::DataPointsFilters f(ifs);
        old_data_ = data_;

        std::cout<<"应用滤波器之前的点数: "<<data_.features.size() << std::endl;
        f.apply(data_);
        std::cout<<"应用滤波器之后的点数: "<< data_.features.size() << std::endl;
    }


    void transform(const Eigen::Matrix4d & trans)
    {
        PM::ICP icp;
        icp.setDefault();
        icp.transformations.apply(data_, trans);
    }

    void initPointCloud(std::string frame_id = param::ros::fream_id )   //参数是fream_id
    {
        point_cloud_.header.seq = 1;
        point_cloud_.header.frame_id = frame_id;
        //TODO

        auto &points = data_.features;
        point32s_.resize( points.cols() );
        for (int i = 0; i < points.cols(); ++ i)
        {
            point32s_[i].x = points(0, i);
            point32s_[i].y = points(1, i);
            point32s_[i].z = points(2, i);
        }
        point_cloud_.points =  point32s_;
    }

    void publish()
    {
        pub.publish(point_cloud_);
    }
};

#undef DP
#undef PM

#endif //MAP_SHOWPOINTCLOUD_H
