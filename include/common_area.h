//
// Created by bohuan on 17-12-21.
//

#ifndef MAP_COMMON_AREA_H
#define MAP_COMMON_AREA_H

#include "common.h"
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/imgproc.hpp>
//#include "cloud2map.h"


namespace CommonArea {

    static double origin_x = 0;
    static double origin_y = 0;
    static int x_cells = 0;
    static int y_cells = 0;
    static cv::Mat map = cv::Mat(1, 1, CV_8U);
    static std::vector< std::vector<bool> > vis_(0);

    static void OG2cvMat(nav_msgs::OccupancyGrid &og, cv::Mat &mat) {

        int rows = og.info.height;
        int cols = og.info.width;
        mat = cv::Mat(og.info.height, og.info.width, CV_8UC1);
        if (mat.isContinuous()) {
            std::memcpy(mat.data, og.data.data(), og.data.size() * sizeof(uint8_t));
        } else {
            for (int i = 0; i != rows; ++i) {
                for (int j = 0; j != cols; ++j) {
                    mat.data[i * cols + j] = og.data.data()[i * cols + j];
                }
            }
        }
    }


    static void addOG2Mat(nav_msgs::OccupancyGrid &og) {
#define CA_UNKNOW -1
#define CA_NO 100
#define CA_YES 0
        CHECK_EQ(og.info.resolution, param::cloud2map::cell_resolution, "居然全局参数的分辨率，和读入的分辨率不一样？");
        int dx = (og.info.origin.position.x - origin_x) / param::cloud2map::cell_resolution;
        int dy = (og.info.origin.position.y - origin_y) / param::cloud2map::cell_resolution;
       // pr(dx), pr(dy), pr(origin_x), prln(origin_y);
       // pr(og.info.height), prln(og.info.width);

        for (int y = 0; y < og.info.height; ++y)
            for (int x = 0; x < og.info.width; ++x) {
                //pr(map.cols * map.rows), pr(x),pr(y),prln(x*og.info.width+y);
                auto tmp = og.data.data()[y * og.info.width + x];
                int map_x = x + dx;
                int map_y = y + dy;

                CHECK_GE(map_x, 0, "栅格地图中的x坐标小于0,出问题, common_area.h");
                CHECK_GE(map_y, 0, "栅格地图中的y坐标小于0,common_area.h");
                CHECK_LT(map_x, x_cells, "common_area.h");
                CHECK_LT(map_y, y_cells, "common_area.h");
                vis_[map_x][map_y] = true;
                auto &idx = map.at<uchar>(map_x, map_y);
                if (tmp == CA_YES)  idx += 100;
            }
#undef CA_YES
#undef CA_NO
#undef CA_UNKNOW
   }


    cv::Rect findMinRect(const cv::Mat1b& src)
    {
        cv::Mat1f W(src.rows, src.cols, float(0));
        cv::Mat1f H(src.rows, src.cols, float(0));

        cv::Rect maxRect(0,0,0,0);
        float maxArea = 0.f;

        for (int r = 0; r < src.rows; ++r)
        {
            for (int c = 0; c < src.cols; ++c)
            {
                if (src(r, c) == 0)
                {
                    H(r, c) = 1.f + ((r>0) ? H(r-1, c) : 0);
                    W(r, c) = 1.f + ((c>0) ? W(r, c-1) : 0);
                }

                float minw = W(r,c);
                for (int h = 0; h < H(r, c); ++h)
                {
                    minw = cv::min(minw, W(r-h, c));
                    float area = (h+1) * minw;
                    if (area > maxArea)
                    {
                        maxArea = area;
                        maxRect = cv::Rect(cv::Point(c - minw + 1, r - h), cv::Point(c+1, r+1));
                    }
                }
            }
        }
        return maxRect;
    }


    cv::RotatedRect largestRectInNonConvexPoly(const cv::Mat1b& src)
    {
        // Create a matrix big enough to not lose points during rotation
        std::vector<cv::RotatedRect> ret;
        std::vector<cv::Point> ptz;
        cv::findNonZero(src, ptz);
        cv::Rect bbox = cv::boundingRect(ptz);
        int maxdim = cv::max(bbox.width, bbox.height);
        cv::Mat1b work(2*maxdim, 2*maxdim, uchar(0));
        src(bbox).copyTo(work(cv::Rect(maxdim - bbox.width/2, maxdim - bbox.height / 2, bbox.width, bbox.height)));

        // Store best data
        cv::Rect bestRect;
        int bestAngle = 0;

        // For each angle
        for (int angle = 0; angle < 90; angle += 1)
        {
            // Rotate the image
            cv::Mat R = cv::getRotationMatrix2D(cv::Point(maxdim,maxdim), angle, 1);
            cv::Mat1b rotated;
            warpAffine(work, rotated, R, work.size());

            // Keep the crop with the polygon
            std::vector<cv::Point> pts;
            findNonZero(rotated, pts);
            cv::Rect box = cv::boundingRect(pts);
            cv::Mat1b crop = rotated(box).clone();

            // Invert colors
            crop = ~crop;
            // Solve the problem: "Find largest rectangle containing only zeros in an binary matrix"
            // https://stackoverflow.com/questions/2478447/find-largest-rectangle-containing-only-zeros-in-an-n%C3%97n-binary-matrix
            cv::Rect r = findMinRect(crop);
            // If best, save result
            if (r.area() > bestRect.area())
            {
                bestRect = r + box.tl();    // Correct the crop displacement
                bestAngle = angle;
            }
        }

        // Apply the inverse rotation
        cv::Mat Rinv = cv::getRotationMatrix2D(cv::Point(maxdim, maxdim), -bestAngle, 1);
        std::vector<cv::Point> rectPoints{bestRect.tl(), cv::Point(bestRect.x + bestRect.width, bestRect.y), bestRect.br(), cv::Point(bestRect.x, bestRect.y + bestRect.height)};
        std::vector<cv::Point> rotatedRectPoints;
        transform(rectPoints, rotatedRectPoints, Rinv);

        // Apply the reverse translations
        for (int i = 0; i < rotatedRectPoints.size(); ++i)
        {
            rotatedRectPoints[i] += bbox.tl() - cv::Point(maxdim - bbox.width / 2, maxdim - bbox.height / 2);
        }

        // Get the rotated rect
        cv::RotatedRect rrect = minAreaRect(rotatedRectPoints);

        return rrect;
    }




    //根据map的数据，得到公共区域
    static cv::Mat getContor()
    {
        cv::threshold(map, map, 150, 255, CV_THRESH_BINARY);
        std::vector <std::vector<cv::Point>>contours;
        cv::findContours(map,
                         contours,
                         CV_RETR_TREE,
                         CV_CHAIN_APPROX_NONE);
        cv::Mat conter(map.size(), CV_8U, cv::Scalar(255));

        auto func = [](const std::vector<cv::Point> &it1,
                       const std::vector<cv::Point> &it2)->bool {
            return it1.size() > it2.size();
        };
        std::sort(contours.begin(), contours.end(), func);
        std::vector<cv::Point> component = contours[0];
        for (auto &it:component)
            conter.at<uchar>(it.y, it.x) = 0;
        cv::drawContours(conter,contours, 0, cv::Scalar(0),CV_FILLED);
        cv::bitwise_not(conter, conter);
        return std::move(conter);
    }


    //2块点云已经被拼好，并投射为OccupancyGrid后的样子。
    //返回值为一个point的vector,size为4.分别为4个坐标。表示一个矩形。
    static  std::vector< geometry_msgs::Point > commonOccupancyGrid(nav_msgs::OccupancyGrid &og1, nav_msgs::OccupancyGrid &og2)
    {
        CHECK_EQ(og1.info.resolution, og2.info.resolution, "分辨率有问题. common_area.h");
        CHECK_EQ(og1.info.resolution, param::cloud2map::cell_resolution, "分辨率有问题 common.area.h");

        double og1x = og1.info.origin.position.x;
        double og1y = og1.info.origin.position.y;
        double og2x = og2.info.origin.position.x;
        double og2y = og2.info.origin.position.y;

        double max_og1x = og1x + og1.info.width * og1.info.resolution;
        double max_og1y = og1y + og1.info.height * og1.info.resolution;

        double max_og2x = og2x + og2.info.width * og2.info.resolution;
        double max_og2y = og2y + og2.info.height * og2.info.resolution;
        //pr(og1x), pr(og1y),pr(max_og1x),prln(max_og1y);
        //pr(og2x), pr(og2y),pr(max_og2x),prln(max_og2y);

        double x_min = std::min(og1x, og2x);
        double y_min = std::min(og1y, og2y);
        double x_max = std::max(max_og1x, max_og2x);
        double y_max = std::max(max_og1y, max_og2y);

        origin_x = x_min;
        origin_y = y_min;

        x_cells = std::ceil( (x_max - x_min) / og1.info.resolution );
        y_cells = std::ceil( (y_max - y_min) / og1.info.resolution );
        pr(x_min),pr(x_max),pr(y_min),pr(y_max),pr(x_cells),prln(y_cells);

        /*//可通行 0
        //不可通行100
        //未知-1*/

        map = cv::Mat(x_cells, y_cells, CV_8U);
        for (int i = 0; i < x_cells * y_cells; ++ i)    map.data[i] = 0;
        vis_.resize(x_cells);
        for (auto &it : vis_)   it.resize(y_cells);
        for (int i = 0; i < x_cells; ++ i)
            for (int j = 0; j < y_cells; ++ j)
                vis_[i][j] = false;

        addOG2Mat(og2);
        addOG2Mat(og1);

        for (int i = 0; i < x_cells;++i)
            for (int j = 0 ; j < y_cells; ++ j)
                map.at<uchar>(i,j)= !vis_[i][j]?0:map.at<uchar>(i,j);

        //cv::namedWindow("拼接地图", 0);
        //cv::imshow("拼接地图", map);
        //cv::waitKey(-1);
        auto conter = getContor();

        /*
        //cv::medianBlur(map, map, 5);
        //cv::threshold(map, map, 150, 255, CV_THRESH_BINARY);
        //这个map现在是2值后的map,现在需要一个处理
        //cv::namedWindow("threadsholded_map", 0);
        //cv::imshow("threadsholded_map", map);
        //cv::waitKey(-1);
         */
        /*
        //bitwise_not(map, binaryImage);
        std::vector <std::vector<cv::Point>>contours;

        // binaryImage.convertTo(mat2, CV_32SC1);

        cv::findContours(map,
                         contours,
                         CV_RETR_TREE,
                         CV_CHAIN_APPROX_NONE);

        cv::Mat conter(map.size(), CV_8U, cv::Scalar(255));
        //    drawContours(conter, contours,
        //                 -1,      //绘制所有轮廓
        //                 cv::Scalar(0),   //颜色为黑色
        //                 2);      //轮廓线的绘制宽度为2

        auto func = [](const std::vector<cv::Point> &it1,
                       const std::vector<cv::Point> &it2)->bool {
            return it1.size() > it2.size();
        };
        std::sort(contours.begin(), contours.end(), func);
        std::vector<cv::Point> component = contours[0];
        for (auto &it:component)
            conter.at<uchar>(it.y, it.x) = 0;
        cv::drawContours(conter,contours, 0, cv::Scalar(0),CV_FILLED);
        cv::bitwise_not(conter, conter);

        //cv::Rect r0 = cv::boundingRect(cv::Mat(component));
         */


        //------------
        cv::RotatedRect r = largestRectInNonConvexPoly(conter);
        // Show
        cv::Mat3b res;
        cv::cvtColor(conter, res, cv::COLOR_GRAY2BGR);
        cv::Point2f points[4];
        r.points(points);


        std::vector< geometry_msgs::Point > ret;
        ret.resize(4);

        for (int i = 0; i< 4; ++ i)
        {
            ret[i].y = points[i].x / param::cloud2map::cell_resolution + origin_y;
            ret[i].x = points[i].y / param::cloud2map::cell_resolution + origin_x;
            ret[i].z = 0;
            //pr(points[i].x / param::cloud2map::cell_resolution + origin_y), prln(points[i].y / param::cloud2map::cell_resolution + origin_x);
            //pr(points[i].x), prln(points[i].y);
        }
        return std::move( ret );
        /*
         * //画图用的
        for (int i = 0; i < 4; ++i)
        {
            line(res, points[i], points[(i + 1) % 4], cv::Scalar(0, 0, 255), 2);
        }


        cv::namedWindow("Result", 0);
        cv::imshow("Result", res);
        cv::waitKey();
         */
    }




    static void commonPoints()  //直接求2点云?
    {
        //TODO
    }


};




#endif //MAP_COMMON_AREA_H
