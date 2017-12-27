//
// Created by bohuan on 17-12-21.
//

#ifndef MAP_CLOUD2MAP_H
#define MAP_CLOUD2MAP_H


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
#include <pcl/common/transforms.h>

#include <opencv2/opencv.hpp>
#include <pointmatcher/PointMatcher.h>
#include "common.h"
#include <opencv2/imgproc.hpp>
#include "cvMat2GridMap.h"

/* Define the two point cloud types used in this code */
//typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
//typedef pcl::PointCloud<pcl::Normal> NormalCloud;

class Cloud2Map
{
#define LPM 1
#define PCL 2
#define UNKNOW 0

private:

    void genOccupancyGrid() {
        for (int i = 0; i < countGrid_.size(); i++) {
            if (countGrid_[i]  == 0)
            {
                ocGrid_[i] = -1;
                continue;
            }
            if (countGrid_[i] > param::cloud2map::buffer)
            {
                ocGrid_[i] = 100;
                continue;
            }
            ocGrid_[i] = 0; // TODO Should be -1
        }
    }

    void calcSurfaceNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
    {
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(param::cloud2map::search_radius);
        ne.compute(*normals);
    }


    void calcSurfaceNormals(pcl::PointCloud<pcl::Normal>::Ptr normals)
    {
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
        ne.setInputCloud(pcl_dp_ptr_);
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>());
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(param::cloud2map::search_radius);
        ne.compute(*normals);
    }


    void calcSize(double &x_max, double &y_max, double &x_min, double &y_min)
    {
        for (size_t i = 0; i < pcl_dp_ptr_ -> size(); i++)
        {
            double x = pcl_dp_ptr_ -> points[i].x;
            double y = pcl_dp_ptr_ -> points[i].y;
            x_max = std::max(x_max, x);
            x_min = std::min(x_min, x);
            y_max = std::max(y_max, y);
            y_min = std::min(y_min, y);
        }
    }

    void initGrid()
    {
        grid_.header.seq = 1;
        grid_.header.frame_id = param::cloud2map::frame;
        grid_.info.origin.position.z = 0;
        grid_.info.origin.orientation.w = 1;
        grid_.info.origin.orientation.x = 0;
        grid_.info.origin.orientation.y = 0;
        grid_.info.origin.orientation.z = 0;
    }

    void updateGrid(int xCells, int yCells, double originX, double originY) {
        grid_.header.seq++;
        grid_.header.stamp.sec = ros::Time::now().sec;
        grid_.header.stamp.nsec = ros::Time::now().nsec;
        grid_.info.map_load_time = ros::Time::now();
        grid_.info.resolution = param::cloud2map::cell_resolution;
        grid_.info.width = xCells;
        grid_.info.height = yCells;
        grid_.info.origin.position.x = originX;
        grid_.info.origin.position.y = originY;
        grid_.data = ocGrid_;
        map_.resize(ocGrid_.size());
        for (int i = 0; i < ocGrid_.size();++i)
        {
            map_[i] = ocGrid_[i];
        }
    }

    void populateMap(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, std::vector<int> &map, double x_min, double y_min,
                     int x_cells, int y_cells) {
        double deviation = param::cloud2map::deviation;
        double cell_resolution = param::cloud2map::cell_resolution;
        for (auto &it : map)    it=0;

        for (size_t i = 0; i < pcl_dp_ptr_ -> size(); i++) {
            double x = pcl_dp_ptr_ -> points[i].x;
            double y = pcl_dp_ptr_ -> points[i].y;
            double z = cloud_normals -> points[i].normal_z;

            double phi = acos(fabs(z));
            int x_cell, y_cell;
            if ( 1 ) { //TODO implement cutoff height!!!
                x_cell = (int) ((x - x_min) / cell_resolution);
                y_cell = (int) ((y - y_min) / cell_resolution);
                if ((y_cell * x_cells + x_cell) > (x_cells * y_cells)) {
                    std::cout << "x: " << x << ", y: " << y << ", x_cell: " << x_cell << ", y_cell: " << y_cell
                              << "\n";
                }
                if (phi > deviation) {
                    map[y_cell * x_cells + x_cell]++;
                }
                else {
                    map[y_cell * x_cells + x_cell]--;
                }
            }
        }
    }

public:
    /*
    void loadPointCloudFromLPM(const std::string & file_name)
    {
        lpm_dp_ = PointMatcher<double>::DataPoints::load(file_name);  //TODO
        flag = LPM;
    }
     */

    void loadPointCloudFromPCL(const std::string & file_name)
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (file_name.c_str(), *pcl_dp_ptr_) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
        {
            PCL_ERROR ((std::string("Couldn't read file from [") + file_name + std::string("]\n")).c_str());
            exit(0);
        }
        flag = PCL;
    }

    void transFromPointCloud(const Eigen::Matrix4d &trans)
    {
        pcl::transformPointCloud(*pcl_dp_ptr_, *pcl_dp_ptr_, trans);
    }

    void LPM2Map()
    {
        //TODO
    }

    void setPCL(const pcl::PointCloud<pcl::PointXYZRGB> & pcl_dp)
    {
        *pcl_dp_ptr_ = pcl_dp;
        flag = PCL;
    }

    //TODO 从其他PCL类型直接得到
    void setPCL()
    {

    }

    void PCL2Map(cv::Rect *rect = NULL) //默认为空指针
    {
        CHECK_EQ(flag, PCL, "不是PCL地图，但是调用了PCL转化为栅格地图的函数  cloud2map.h::PLC2Map");
        initGrid();
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
        calcSurfaceNormals(pcl_dp_ptr_, cloud_normals);
        double x_max(0);
        double y_max(0);
        double x_min(0);
        double y_min(0);

        if (rect == NULL)
        {
            calcSize(x_max, y_max, x_min, y_min);//寻找点云最大值，最小值
        }
        else
        {
            //手动指定x,y方向最大，最小值
            x_min = rect -> x;
            x_max = rect -> x + rect -> height;
            y_min = rect -> y;
            y_max = rect -> y + rect -> width;
        }
        int x_cells = ((int) ((x_max - x_min) / param::cloud2map::cell_resolution)) + 1;
        int y_cells = ((int) ((y_max - y_min) / param::cloud2map::cell_resolution)) + 1;


#if 1
        prln("PCLMap:");
        pr(x_min), pr(x_max), pr(y_min), prln(y_max);
        pr(x_cells), prln(y_cells);
        prln("PCLmap end");
#endif


        countGrid_.resize(y_cells * x_cells);
        ocGrid_.resize(y_cells * x_cells);

        populateMap(cloud_normals, countGrid_, x_min, y_min, x_cells, y_cells);
        genOccupancyGrid();
        updateGrid(x_cells, y_cells, x_min, y_min);

        CvMat2GridMap::gridMap2cvMat(cvmat_, grid_);    //转换为CVMat类型
    }

    nav_msgs::OccupancyGrid& occupancy_grid()
    {
        return grid_;
    }

    CVMat& getCVMat()
    {
        return cvmat_;
    }



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
