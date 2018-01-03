//
// Created by bohuan on 18-1-2.
//

#ifndef MAP_CORE_MAP_CORE_H
#define MAP_CORE_MAP_CORE_H

#include "pointmatcher/PointMatcher.h"
#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/OccupancyGrid.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <ugv_localizer/LocalMapRetrieve.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>

#include <json/json.h>
#include "common.h"
#include "cloud2map.h"
#include "common_area.h"
#include "map_db_node.h"
#include "NodeInfo.h"
#include "EdgeInfo.h"
#include "junk_map.h"



class Map {

public:

    ~Map();

    Map(int max_nodes);

    void setCurNodeID(int id);

    void setPose();

    void setPaths(const std::vector<int> & path);

    void initNodeHandle(ros::NodeHandlePtr nh = nullptr);

    void setInitPose(geometry_msgs::Pose pose);

    void core();

    cv::Mat &grid_map() { return grid_map_; }

    void setPointCloud(int idx, const std::string &file);

    void setXY(int X, int Y);

    void setGridMapFromFile(const std::string & file);

    //有向边
    void add_edge(int from, int to, int edge_id, Eigen::Matrix4d trans_mat);

    void save(const std::string &file_name);

    void load(const std::string &file_name);

    void SPFA(int from, int to, std::vector<int> & nodes); //求from 到to的路径，经过的路径保存在nodes中，经过的边保存在edges中

    void debug();

private:
    int X_, Y_;
    std::vector<std::vector< EdgeInfo > >  map_;

    cv::Mat grid_map_;

    int max_nodes_;             //the number of nodes

    std::vector<NodeInfo> nodes_info_;
    std::string cv_file_ = "no_file";

    std::vector<int> paths_; //要经过的路径. front为即将要去的点。 如果为空，则当前已经在目标点内。
    int cur_pos_;   //表示当前到path的第几个位置.
    int cur_node_id_;   //当前所处的node

    ros::NodeHandlePtr node_handle_ptr_;
    ros::Publisher pub_initial_pose_;
    std::vector<ros::Publisher> pub_markers_;   //不止一个marker
    ros::Publisher pub_local_map_;

    ros::Subscriber sub_cur_node_;
    ros::Subscriber sub_paths_;
    ros::Subscriber sub_reboot_;

    geometry_msgs::PoseWithCovarianceStamped pose_;
    std::vector<visualization_msgs::Marker> markers_;
    int num_marker_;        //活动的marker的数量
    bool need_reboot_;       //是否要重启。 默认是false，不用重启。因为修改cur_node和paths后，相当于需要重启

    void subCurNodeCallBack(const std_msgs::Int32 & cur_node_id);

    void subPathsCallBack(const std_msgs::Int32MultiArray & paths );

    void subRebootCallBack(const std_msgs::Int32& need_reboot);

    void setMarker(const std::vector< std::vector<cv::Point2f> > &polygon);

    void pubMarkers();

    //判断pose是否在多边形其中一个里面。是的话返回true,否则返回false
    bool pointPolygonsTest(std::vector< std::vector<cv::Point2f> > * contours_ptr, cv::Point2f pose);

    bool retrieveCallback(ugv_localizer::LocalMapRetrieve::Request &req,
                          ugv_localizer::LocalMapRetrieve::Response &res,
                          bool * flag, std::vector< std::vector<cv::Point2f> > * contours_ptr, Eigen::Matrix4d *trans_ptr);

    //在一个node里 到达公共区域，就结束
    void run(Eigen::Matrix4d * trans_ptr, std::vector< std::vector<cv::Point2f>> * contours_ptr = NULL);

    int odometryCallBack(const nav_msgs::OdometryConstPtr &odo, nav_msgs::Odometry * odometry);

    void transforPose(Eigen::Matrix4d trans);

    void publishPose();

    void publishLocalMap();


};



#endif //MAP_CORE_MAP_CORE_H
