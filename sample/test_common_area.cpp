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


std::vector<visualization_msgs::Marker> markers_;
std::vector<ros::Publisher> pub_markers_;   //不止一个marker
int num_marker_;        //活动的marker的数量


void initPubMarker(ros::NodeHandle & nh)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "global_frame";
    marker.header.stamp = ros::Time::now();
    marker.header.seq = 0;
    marker.type = marker.LINE_STRIP;
    marker.action = marker.ADD;
    marker.id = 1;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.color.a = 1;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 1;
    marker.scale.x = 2;
    marker.scale.y = 2;
    marker.scale.z = 2;

    markers_.resize(5);
    for (int i = 0; i < 5; ++ i)
        markers_[i] = marker;


    pub_markers_.resize(5); //TODO 是否需要参数化
    pub_markers_[0] = nh.advertise<visualization_msgs::Marker>("marker0", 10);
    pub_markers_[1] = nh.advertise<visualization_msgs::Marker>("marker1", 10);
    pub_markers_[2] = nh.advertise<visualization_msgs::Marker>("marker2", 10);
    pub_markers_[3] = nh.advertise<visualization_msgs::Marker>("marker3", 10);
    pub_markers_[4] = nh.advertise<visualization_msgs::Marker>("marker4", 10);
}


void setMarker(const std::vector< std::vector<cv::Point2f> > &polygon)
{
    num_marker_ = polygon.size();

    for (int i =0; i < num_marker_; ++ i)
    {
        markers_[i].points.clear();
        markers_[i].points.resize( polygon[i].size() );
        for (int j = 0; j < polygon[i].size(); ++ j)
        {
            markers_[i].points[j].x = polygon[i][j].x;
            markers_[i].points[j].y = polygon[i][j].y;
            markers_[i].points[j].z = 0;
        }
    }
}

void pubMarkers()
{
    for (int i = 0; i < num_marker_; ++ i)
    {
        pub_markers_[i].publish(markers_[i]);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_common_area");
    ros::NodeHandle nh;
    ros::Rate r(2);
    ros::Publisher pub_gridmap1 = nh.advertise<nav_msgs::OccupancyGrid>("gridmap0", 10);
    ros::Publisher pub_gridmap2 = nh.advertise<nav_msgs::OccupancyGrid>("gridmap1", 10);
    initPubMarker(nh);

    //处理第一个点云
    cm0.loadPointCloudFromPCL("/home/bohuan/map_data/pcd/6.pcd");
    cm0.PCL2Map();
    auto og1 = cm0.occupancy_grid();   //得到点云1的栅格地图
    spc0 = new ShowPointCloud_LPM("point_cloud_topic0", "/home/bohuan/map_data/vtk/6.vtk" , nh );
    spc0->initPointCloud();


    //读取第二个点云，并处理
    cm1.loadPointCloudFromPCL("/home/bohuan/map_data/pcd/7.pcd");
    cm1.transFromPointCloud(param::from7to6()); //根据变换，要先做对齐
    cm1.PCL2Map();
    auto og2 = cm1.occupancy_grid();   //得到点云2的栅格地图
    spc1 = new ShowPointCloud_LPM("point_cloud_topic1", "/home/bohuan/map_data/vtk/7.vtk" , nh );
    spc1->transform(param::from7to6());     //坐标变换一下
    spc1->initPointCloud();



    //得到公共区域
    std::vector< std::vector< geometry_msgs::Point > >com= CommonArea::commonOccupancyGrid(og1, og2);

    //取出轮廓线，用cv来判断点是否在轮廓内
    std::vector< std::vector<cv::Point2f> >  contours_;
    contours_.resize(com.size());
    for (int p = 0; p < contours_.size(); ++ p)
    {
        contours_[p].resize(5);
        for (int i = 0 ; i < 5; ++ i)
        {
            contours_[p][i].x = com[p][i%4].x;
            contours_[p][i].y = com[p][i%4].y;
        }
    }

    setMarker(contours_);


    while (ros::ok())
    {
        pub_gridmap1.publish(og1);
        pub_gridmap2.publish(og2);
        spc1 -> publish();
        spc0 -> publish();
        pubMarkers();
        r.sleep();
    }

    return 0;
}
