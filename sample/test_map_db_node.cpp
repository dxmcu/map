//
// Created by bohuan on 17-12-23.
//



/*
修改后的map_db_node主要功能：
 首先map_db_node::loadGlobalMapFromPCD( "xx/xx.pcd") 读取pcd地图，初始化。
 然后就可以直接使用map_db_node::retrieve(const geometry_msgs::PoseStamped &mapLoc,
                                 sensor_msgs::PointCloud2 &res)
 也就是给一个位姿，输出这个位姿的局部地图的点云信息。范围为x,y,z坐标轴上都是正负50范围的点云。
 具体用法见此程序。
 ugv_localizer为直接从 YHY程序复制出来的srv文件...
 为了贴合YHY的某个程序，这里简单的封装了一个call back的函数，提供给service用。
 不懂了问我..  bohuan.xue@gmail.com
*/

#include "map_db_node.h"
//#include "tic_toc.h"
#include "ugv_localizer/LocalMapRetrieve.h"

ros::Publisher pubLocalMap;  //发布生成的局部地图
//ros::Publisher pubPointCloud;

MapDbNode map;

bool retrieveCallback(ugv_localizer::LocalMapRetrieve::Request &req,
                      ugv_localizer::LocalMapRetrieve::Response &res)
{
    map.retrieve(req.poseWithStamp, res.pointcloud);   //给一个位姿，返回一个点云。

    pubLocalMap.publish(res.pointcloud);    //发布出这个地图
    sensor_msgs::PointCloud2 ddd;
    std::cout<<"published"<<std::endl;
    return true;
}


const std::string pcd_name = "/home/bohuan/map_data/pcd/0.pcd";

int main(int argc, char** argv)
{
    // ros init
    ros::init(argc, argv, "map_db_node");       //初始化
    ros::NodeHandle nh("~");
    ros::ServiceServer service = nh.advertiseService("/local_map_retrieve", retrieveCallback);


    map.loadGlobalMapFromPCD(pcd_name);    //读取一个pcd文件。
    pubLocalMap = nh.advertise<sensor_msgs::PointCloud2> ("/retrived_map", 10);

    ros::spin();
    return 0;
}
