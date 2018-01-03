//
// Created by bohuan on 17-12-21.
//

#ifndef MAP_CORE_COMMON_AREA_H
#define MAP_CORE_COMMON_AREA_H

#include "common.h"
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/imgproc.hpp>
//#include "cloud2map.h"


class CommonArea
{
private:
    static double origin_x ;

    static double origin_y ;

    static int x_cells ;

    static int y_cells ;

    static cv::Mat map ;//= cv::Mat(1, 1, CV_8U);

    static std::vector< std::vector<bool> > vis_;//(0);


    static void OG2cvMat(nav_msgs::OccupancyGrid &og, cv::Mat &mat);

    static void addOG2Mat(nav_msgs::OccupancyGrid &og);

    static cv::Rect findMinRect(const cv::Mat1b& src);

    static cv::RotatedRect largestRectInNonConvexPoly(const cv::Mat1b& src);

    //根据map的数据，得到公共区域
    static std::vector<cv::Mat> getContor();

    //2块点云已经被拼好，并投射为OccupancyGrid后的样子。
    //返回值为一个point的vector,size为4.分别为4个坐标。表示一个矩形。
public:
    static std::vector< std::vector< geometry_msgs::Point > >commonOccupancyGrid(nav_msgs::OccupancyGrid &og1, nav_msgs::OccupancyGrid &og2);

    static void commonPoints();  //直接求2点云? TODO
};




#endif //MAP_COMMON_AREA_H
