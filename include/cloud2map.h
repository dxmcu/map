//
// Created by bohuan on 17-12-21.
//

#ifndef MAP_CORE_CLOUD2MAP_H
#define MAP_CORE_CLOUD2MAP_H


#include <bits/stdc++.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>



#include <opencv2/opencv.hpp>
#include <pointmatcher/PointMatcher.h>
#include "common.h"
#include <opencv2/imgproc.hpp>
#include "cvMat2GridMap.h"

/* Define the two point cloud types used in this code */
//typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
//typedef pcl::PointCloud<pcl::Normal> NormalCloud;
#define LPM 1
#define PCL 2
#define UNKNOW 0

class Cloud2Map
{

private:

    void genOccupancyGrid();

    void calcSurfaceNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);

    void calcSurfaceNormals(pcl::PointCloud<pcl::Normal>::Ptr normals);

    void calcSize(double &x_max, double &y_max, double &x_min, double &y_min);

    void initGrid();

    void updateGrid(int xCells, int yCells, double originX, double originY);

    void populateMap(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, std::vector<int> &map, double x_min, double y_min,
                     int x_cells, int y_cells);

public:

    void loadPointCloudFromPCL(const std::string & file_name);

    void transFromPointCloud(const Eigen::Matrix4d &trans);

    void LPM2Map();

    void setPCL(const pcl::PointCloud<pcl::PointXYZRGB> & pcl_dp);

    //TODO 从其他PCL类型直接得到
    void setPCL();

    void PCL2Map(cv::Rect *rect = NULL);//默认空指针

    nav_msgs::OccupancyGrid& occupancy_grid();

    CVMat& getCVMat();


private:
    int flag = UNKNOW;  //确认点云类型
    CVMat cvmat_;
    nav_msgs::OccupancyGrid grid_;
    //PointMatcher<double>::DataPoints lpm_dp_;   //lib point matcher的DP类型
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_dp_ptr_=boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>());              //PCL的DP类型
    std::vector<int> map_;                      //整个地图
    std::vector<int> countGrid_;//(y_cells * x_cells);
    std::vector<signed char> ocGrid_;//(yCells * xCells);
};

#undef UNKNOW
#undef LCP
#undef LPM
#endif //MAP_CLOUD2MAP_H
