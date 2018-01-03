//
// Created by bohuan on 18-1-2.
//

#ifndef MAP_CORE_NODEINFO_H
#define MAP_CORE_NODEINFO_H

#include "GPS.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <map_db_node.h>
#include "junk_map.h"

class NodeInfo {
private:
    //DP point_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr_ =
            boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>());;
    std::string file_ = "no_file";  //点云的载入路径
    int node_number_ = -1;          //默认点的编号
    std::vector<GPS> gpss_;
    CVMat cvmat;                    //保存点云到2D的图,好像没啥用？
    MapDbNode map_db_node_;         //pcd格式，用来显示某区域点，做定位用的。

public:


    void pos2cloud(const geometry_msgs::PoseWithCovarianceStamped &pose, sensor_msgs::PointCloud2 &cloud);

    std::string PCLFile()  const;

    bool retrieve(const geometry_msgs::PoseStamped &mapLoc, sensor_msgs::PointCloud2 &res);

    //下面4个函数，实现的是map_db_node_里的地图的变换
    //double
    void transform(const Eigen::Matrix4d & trans);

    //double
    void invTransform(const Eigen::Matrix4d & trans);

    //float
    void transform(const Eigen::Matrix4f & trans);

    //float
    void invTransform(const Eigen::Matrix4f & trans);

    NodeInfo();

    int node_number()   const;

    GPS& getGPS(int idx);
    GPS getGPS(int idx) const;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr & point_cloud_ptr_ref();

    void setGPSs(const std::vector< GPS > & gpss);

    NodeInfo(int node_number, const std::string & file); //节点编号， pcd格式路径。

    Json::Value json();

    void loadFromJson(Json::Value root);
};



#endif //MAP_CORE_NODEINFO_H
