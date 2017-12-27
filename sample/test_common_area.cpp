//
// Created by bohuan on 17-12-21.
//
#include "common.h"
#include "common_area.h"
#include "cloud2map.h"

#include "showPointCloud.h"
#include <visualization_msgs/Marker.h>


Cloud2Map cm0;
Cloud2Map cm1;

ShowPointCloud_LPM *spc0;
ShowPointCloud_LPM *spc1;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_common_area");
    ros::NodeHandle nh;
    ros::Rate r(2);
    ros::Publisher pub_gridmap1 = nh.advertise<nav_msgs::OccupancyGrid>("gridmap0", 10);
    ros::Publisher pub_gridmap2 = nh.advertise<nav_msgs::OccupancyGrid>("gridmap1", 10);

    //处理第一个点云
    cm0.loadPointCloudFromPCL("/home/bohuan/map_data/pcd/0.pcd");
    cm0.PCL2Map();
    auto og1 = cm0.occupancy_grid();   //得到点云1的栅格地图
    spc0 = new ShowPointCloud_LPM("point_cloud_topic0", "/home/bohuan/map_data/vtk/0.vtk" , nh );
    spc0->initPointCloud();


    //读取第二个点云，并处理
    cm1.loadPointCloudFromPCL("/home/bohuan/map_data/pcd/1.pcd");
    cm1.transFromPointCloud(param::from1to0()); //根据变换，要先做对齐
    cm1.PCL2Map();
    auto og2 = cm1.occupancy_grid();   //得到点云2的栅格地图
    spc1 = new ShowPointCloud_LPM("point_cloud_topic1", "/home/bohuan/map_data/vtk/1.vtk" , nh );
    spc1->transform(param::from1to0());     //坐标变换一下
    spc1->initPointCloud();



    //得到公共区域
    auto points = CommonArea::commonOccupancyGrid(og1, og2);
    std::cout<<"公共区域的矩形坐标为:"<<std::endl;
    for (auto point : points)
        pr(point.x),prln(point.y);

    //显示公共区域为marker类型
    visualization_msgs::Marker marker;

    marker.header.frame_id = param::ros::fream_id;
    marker.header.stamp = ros::Time::now();
    marker.header.seq = 0;
    marker.type = marker.LINE_STRIP;
    marker.action = marker.ADD;
    marker.lifetime = ros::Duration(0.0);
    marker.id =1 ;
    marker.pose.position.x =  0;
    marker.pose.position.y = 0;
    marker.pose.position.z =  0;
    marker.color.a=1;
    marker.color.r=1;
    marker.color.g=0;
    marker.color.b=1;

    marker.scale.x = 2;
    marker.scale.y = 2;
    marker.scale.z = 2;

    marker.points = points;
    marker.points.push_back(points[0]);
    auto pub_marker = nh.advertise<visualization_msgs::Marker>("marker", 1000);


    while (ros::ok())
    {
        pub_gridmap1.publish(og1);
        pub_gridmap2.publish(og2);
        spc1 -> publish();
        spc0 -> publish();
        pub_marker.publish(marker);
        r.sleep();
    }

    return 0;
}
