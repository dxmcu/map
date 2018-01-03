/*************************************************************************
  > File Name: mapping_load_icp.cpp
  > Author: YHY
  > Mail: hyyezju@gmail.com
  > Created Time: Friday, Feb 27, 2017 14:44:00 HKT
  > 17.12.23 修改整合
 ************************************************************************/

#ifndef MAP_MAP_DB_NODE_H
#define MAP_MAP_DB_NODE_H

#include <iostream>
#include <fstream>
#include <string>
#include <limits>

#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

#include <cmath>
#include <time.h>

#include "pointmatcher/PointMatcher.h"
#include "point_cloud.h"

#include "yaml-cpp/yaml.h"



class MapDbNode
{
private:
    PointMatcher<float>::TransformationParameters T_localToGlobal = PointMatcher<float>::TransformationParameters::Identity(4,4);
    Eigen::Vector3f Txyz = Eigen::Vector3f(0, 0, 0);

    PointMatcher<float>::DataPoints globalMap_;// = PointMatcher<float>::DataPoints();     //全局地图点云

    PointMatcher<float>::TransformationParameters transformationInverse(const PointMatcher<float>::TransformationParameters inTransformation);

public:

    //double
    void transform(const Eigen::Matrix4d & trans);

    //double
    void invTransform(const Eigen::Matrix4d & trans);

    //float
    void transform(const Eigen::Matrix4f & trans);

    //float
    void invTransform(const Eigen::Matrix4f & trans);

    void loadGlobalMapFromPCD(const std::string & pcd_name);  //从pcd文件获取global map


    void pos2cloud(const geometry_msgs::PoseWithCovarianceStamped &pose, sensor_msgs::PointCloud2 &cloud);

    bool retrieve(const geometry_msgs::PoseStamped &mapLoc, sensor_msgs::PointCloud2 &res);
};

#endif
