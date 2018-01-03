//
// Created by bohuan on 17-12-21.
//

#ifndef MAP_CORE_CVMAT2GRIDMAP_H
#define MAP_CORE_CVMAT2GRIDMAP_H

#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include "common.h"

//TODO

class CvMat2GridMap {
private:

    static void initGrid(nav_msgs::OccupancyGrid& grid) {
        grid.header.seq = 1;
        grid.header.frame_id = param::cloud2map::frame;
        grid.info.origin.position.z = 0;
        grid.info.origin.orientation.w = 1;
        grid.info.origin.orientation.x = 0;
        grid.info.origin.orientation.y = 0;
        grid.info.origin.orientation.z = 0;
    }

    static void updateGrid(nav_msgs::OccupancyGrid &grid,
                           int x_cells, int y_cells,
                           double origin_x, double origin_y ) {
        grid.header.seq++;
        grid.header.stamp.sec = ros::Time::now().sec;
        grid.header.stamp.nsec = ros::Time::now().nsec;
        grid.info.map_load_time = ros::Time::now();
        grid.info.resolution = param::cloud2map::cell_resolution;
        grid.info.width = x_cells;
        grid.info.height = y_cells;
        grid.info.origin.position.x = origin_x;
        grid.info.origin.position.y = origin_y;
    }

public:


    //cvmat转ros的gridmap
    static void cvMat2GridMap(CVMat &cvmat, nav_msgs::OccupancyGrid &grid) {
        initGrid(grid);
        int width = cvmat.rows();
        int height = cvmat.cols();
        grid.data.resize(width * height);

        int debug=0;
        for (int i = 0; i < width; ++ i)
            for (int j = 0; j < height; ++ j)
            {
                grid.data[i * height + j] = cvmat.at(i, j);
            }
        updateGrid(grid, width, height, cvmat.origin_x, cvmat.origin_y);
    }

    //ros的gridmap转cvmat
    static void gridMap2cvMat(CVMat &cvmat, nav_msgs::OccupancyGrid &grid)
    {
        int width = grid.info.width;
        int height = grid.info.height;
        cvmat() = cv::Mat(grid.info.height, grid.info.width, CV_8UC1);
        //memmove( cvmat.mat.data ,grid.data.data(), grid.data.size()* sizeof(uchar));
        for (int x = 0; x < height; ++ x)
            for (int y = 0; y < width; ++ y)
                cvmat.at(x, y) = grid.data[x * width + y];
    }
};

#endif //MAP_CVMAT2GRIDMAP_H
