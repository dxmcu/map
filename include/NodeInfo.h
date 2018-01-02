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


    void pos2cloud(const geometry_msgs::PoseWithCovarianceStamped &pose, sensor_msgs::PointCloud2 &cloud)
    {
        geometry_msgs::PoseStamped mapLoc;
        mapLoc.header = pose.header;
        mapLoc.pose = pose.pose.pose;
        retrieve(mapLoc, cloud);
    }

    std::string PCLFile()  const { return file_;   }

    bool retrieve(const geometry_msgs::PoseStamped &mapLoc,
                  sensor_msgs::PointCloud2 &res)
    {
        return map_db_node_.retrieve(mapLoc, res);
    }

    //下面4个函数，实现的是map_db_node_里的地图的变换
    //double
    void transform(const Eigen::Matrix4d & trans)
    {
        map_db_node_.transform(trans);
    }

    //double
    void invTransform(const Eigen::Matrix4d & trans)
    {
        map_db_node_.invTransform(trans);
    }

    //float
    void transform(const Eigen::Matrix4f & trans)
    {
        map_db_node_.transform(trans);
    }

    //float
    void invTransform(const Eigen::Matrix4f & trans)
    {
        map_db_node_.invTransform(trans);
    }

    NodeInfo(){}

    int node_number()   const { return node_number_;    }

    GPS& getGPS(int idx) {   return gpss_[idx];   }
    GPS getGPS(int idx) const { return gpss_[idx];  }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr & point_cloud_ptr_ref()   {   return point_cloud_ptr_;    }

    void setGPSs(const std::vector< GPS > & gpss)
    {
        gpss_ = gpss;
    }

    NodeInfo(int node_number, const std::string & file) //节点编号， pcd格式路径。
    {
        node_number_ = node_number;
        file_ = file;
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (file_.c_str(), *point_cloud_ptr_) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
        {
            PCL_ERROR ((std::string("Couldn't read file from [") + file_ + std::string("]\n")).c_str());
            exit(0);
        }

        //3D点云，给一个点云，转成2D信息。
        junk_map::cloud2map.setPCL(*point_cloud_ptr_);
        junk_map::cloud2map.PCL2Map();//做一次处理
        cvmat = junk_map::cloud2map.getCVMat();

        //TODO保存png格式
        cvmat.file_name = file_ + ".png";
        cv::imwrite(cvmat.file_name.c_str(), cvmat.mat, junk_map::compression_params);

        //读取pcd信息
        map_db_node_.loadGlobalMapFromPCD(file_);
    }

    Json::Value json()
    {
        Json::Value root;
        root["file"] = file_;
        root["number"] = node_number_;
        int cnt=0;
        for (int i = 0; i != gpss_.size(); ++ i)
        {
            root["GPSs"][i] = gpss_[i].json();
        }
        root["cvmat"] = cvmat.json();
        return root;
    }

    void loadFromJson(Json::Value root)
    {
        file_ = root["file"].asString();
        node_number_ = root["number"].asInt();
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (file_.c_str(), *point_cloud_ptr_) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
        {
            PCL_ERROR ((std::string("Couldn't read file from [") + file_ + std::string("]\n")).c_str());
            exit(0);
        }
        gpss_.clear();
        for (auto gps : root["GPSs"])
        {
            gpss_.push_back(GPS());
            gpss_[ gpss_.size() - 1 ].loadFromJson( gps );
        }
        cvmat.loadFromJson(root["cvmat"]);
        map_db_node_.loadGlobalMapFromPCD(file_);
    }
};



#endif //MAP_CORE_NODEINFO_H
