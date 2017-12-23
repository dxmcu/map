//
// Created by bohuan on 17-12-23.
//

#ifndef MAP_POINT_CLOUD_H
#define MAP_POINT_CLOUD_H

/*************************************************************************
	> File Name: point_cloud.h
	> Author: YHY
	> Mail: hyyezju@gmail.com
	> Created Time: Thursday, November 17, 2016 AM11:34:38 HKT
 ************************************************************************/

#include "pointmatcher/PointMatcher.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"

namespace ros
{
    struct Time;
};
namespace tf
{
    struct TransformListener;
};

namespace PointMatcher_ros
{
    template<typename T>
    typename PointMatcher<T>::DataPoints rosMsgToPointMatcherCloud(const sensor_msgs::PointCloud2& rosMsg);

    template<typename T>
    typename PointMatcher<T>::DataPoints rosMsgToPointMatcherCloud(const sensor_msgs::LaserScan& rosMsg, const tf::TransformListener* listener = 0, const std::string& fixed_frame = "/world", const bool force3D = false, const bool addTimestamps=false, const bool addObservationDirection=false);

    template<typename T>
    sensor_msgs::PointCloud2 pointMatcherCloudToRosMsg(const typename PointMatcher<T>::DataPoints& pmCloud, const std::string& frame_id, const ros::Time& stamp);

}; // PointMatcher_ros


#endif //MAP_POINT_CLOUD_H
