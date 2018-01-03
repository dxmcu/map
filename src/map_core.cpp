//
// Created by bohuan on 18-1-2.
//

#include "map_core.h"


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



void Map::subCurNodeCallBack(const std_msgs::Int32 & cur_node_id)
{
    setCurNodeID(cur_node_id.data);
    ROS_WARN_STREAM("set node:" << cur_node_id);
    need_reboot_ = true;
}

void Map::subPathsCallBack(const std_msgs::Int32MultiArray & paths )
{
    setPaths(paths.data);
    std::stringstream ss;
    for (auto x : paths.data)   ss<<x<<" ";
    ROS_WARN_STREAM("set paths:" << ss.str());
    need_reboot_ = true;
}

void Map::subRebootCallBack(const std_msgs::Int32& need_reboot)
{
    need_reboot_ = need_reboot.data;
}

void Map::setMarker(const std::vector< std::vector<cv::Point2f> > &polygon)
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

void Map::pubMarkers()
{
#ifdef DEBUG
    ROS_INFO_STREAM("num_marker = " << num_marker_);
#endif
    for (int i = 0; i < num_marker_; ++ i)
    {
        pub_markers_[i].publish(markers_[i]);
    }
}

//判断pose是否在多边形其中一个里面。是的话返回true,否则返回false
bool Map::pointPolygonsTest(std::vector< std::vector<cv::Point2f> > * contours_ptr, cv::Point2f pose)
{
    for (auto &contour : *contours_ptr)
    {
        if (cv::pointPolygonTest(contour, pose, false) == 1)
        {
            return true;
        }
    }
    return false;
}

bool Map::retrieveCallback(ugv_localizer::LocalMapRetrieve::Request &req,
                           ugv_localizer::LocalMapRetrieve::Response &res,
                           bool * flag, std::vector< std::vector<cv::Point2f> > * contours_ptr, Eigen::Matrix4d *trans_ptr)
{
    cv::Point2f pose;

    pose.x = req.poseWithStamp.pose.position.x;
    pose.y = req.poseWithStamp.pose.position.y;
    pose_.pose.pose = req.poseWithStamp.pose;

    if (contours_ptr && contours_ptr->size() && true == pointPolygonsTest(contours_ptr, pose))
    {
        ROS_INFO("switching map..");

        //ROS_INFO_STREAM(pose_);
        *flag = true;   //需要切换地图
        cur_node_id_ = paths_[cur_pos_++];
        if (cur_pos_ == paths_.size() - 1)
        {
            //是最后一个了。
            //TODO
            prln(cur_pos_);
            std::cout<<"see map::retrieveCallback"<<std::endl;
            contours_ptr -> clear();
            setMarker(*contours_ptr);
            transforPose(trans_ptr->inverse());
        }
        else
        {
            auto &edge = map_[cur_node_id_][paths_[cur_pos_]];
            *contours_ptr = edge.contours();
            setMarker(*contours_ptr);
            transforPose(trans_ptr->inverse());
        }
    }

    auto &node = nodes_info_[cur_node_id_];
    req.poseWithStamp.pose = pose_.pose.pose;
    node.retrieve(req.poseWithStamp, res.pointcloud);   //给一个位姿，返回一个点云。同时自带发布消息

    if (*flag)  publishPose();          //切换地图，才发布init pose
    pub_local_map_.publish(res.pointcloud);    //发布出这个地图
    pubMarkers();
    std::cout<<"published && cur_node_id = "<<cur_node_id_<<std::endl;
    return true;
}

//在一个node里
void Map::run(Eigen::Matrix4d * trans_ptr, std::vector< std::vector<cv::Point2f>> * contours_ptr) //到达公共区域，就结束
{
    //auto &node = nodes_info_;
    bool flag = false;  //判定是否要更换地图。默认是不用，false

    boost::function<bool(ugv_localizer::LocalMapRetrieve::Request&, ugv_localizer::LocalMapRetrieve::Response&)> call_back ;
    //static ros::Publisher pub_local_map = node_handle_ptr_ -> advertise<sensor_msgs::PointCloud2> ("/retrived_map", 10);    //默认用来发布新local map的
    ROS_INFO("init pub_loacal_map OK");
    call_back = boost::bind(&Map::retrieveCallback, this, _1, _2, &flag, contours_ptr, trans_ptr);
    ros::ServiceServer service = node_handle_ptr_ -> advertiseService("/local_map_retrieve", call_back);
    ROS_INFO("running node");

    //int cnt=0;
    while (ros::ok() && false == flag)
    {
        ros::spinOnce();
        if (true == need_reboot_)
        {
            ROS_FATAL("need reboot");
            return;
        }
    }
    ROS_INFO("switching map");
}


int Map::odometryCallBack(const nav_msgs::OdometryConstPtr &odo, nav_msgs::Odometry * odometry)
{
    *odometry = *odo;
    return 0;
};

void Map::transforPose(Eigen::Matrix4d trans)
{
    Eigen::Isometry3d I = Eigen::Isometry3d::Identity();
    double x = pose_.pose.pose.orientation.x;
    double y = pose_.pose.pose.orientation.y;
    double z = pose_.pose.pose.orientation.z;
    double w = pose_.pose.pose.orientation.w;
    Eigen::Quaterniond Q(w, x, y, z);

    x = pose_.pose.pose.position.x;
    y = pose_.pose.pose.position.y;
    z = pose_.pose.pose.position.z;

    Eigen::Vector3d V(x, y, z);
    I.rotate(Q);
    I.pretranslate(V);
    trans = trans * I.matrix();
    Q = Eigen::Quaterniond(trans.block<3,3>(0,0));
    V = trans.block<3, 1>(0,3);


    pose_.pose.pose.orientation.w = Q.w();
    pose_.pose.pose.orientation.x = Q.x();
    pose_.pose.pose.orientation.y = Q.y();
    pose_.pose.pose.orientation.z = Q.z();

    pose_.pose.pose.position.x = V(0);
    pose_.pose.pose.position.y = V(1);
    pose_.pose.pose.position.z = V(2);
    //   ROS_INFO_STREAM(pose_);
}

void Map::publishPose()
{
    //多发几次，确保能收到
    for (int i = 1; i<= 1;++i)
    {
        pub_initial_pose_.publish(pose_);
        //ros::Duration(1).sleep();
    }
    ROS_INFO("/initialpose is published");
}

void Map::publishLocalMap()
{
    static sensor_msgs::PointCloud2 cloud;
    nodes_info_[cur_node_id_].pos2cloud(pose_, cloud);
    ROS_INFO_STREAM("cur node id is:" << cur_node_id_);
    for (int i = 1;i<=1;++i)
    {
        pub_local_map_.publish(cloud);  //发布地图
    }
}

Map::~Map()
{
    std::cout<<"@@@"<<std::endl;
    sub_reboot_.shutdown();
    sub_cur_node_.shutdown();
    sub_paths_.shutdown();
    pub_initial_pose_.shutdown();
    for (auto &it : pub_markers_)   it.shutdown();
    pub_local_map_.shutdown();
    //delete sub_odometry_ptr_;
}

void Map::setCurNodeID(int id)
{
    cur_node_id_ = id;
}

void Map::setPose()
{
    //TODO
}

void Map::setPaths(const std::vector<int> & path)
{
    paths_ = path;
}

void Map::initNodeHandle(ros::NodeHandlePtr nh )
{
    if (nh == nullptr)
    {
        node_handle_ptr_.reset( new ros::NodeHandle());
    }
    else
    {
        //TODO 不然在delete函数那可能出问题
        ROS_INFO("do not need nodehandler, see map::initNoideHandle");
        exit(0);
        node_handle_ptr_ = nh;
    }

    //设置位姿发布器
    pub_initial_pose_ = node_handle_ptr_ -> advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);

    //发布公共区域框框用的
    pub_markers_.resize(param::max_markers); //TODO 是否需要参数化
    for (int i = 0; i < param::max_markers; ++ i)
    {
        char num[32];
        sprintf(num,"%d", i);
        pub_markers_[i] = node_handle_ptr_->advertise<visualization_msgs::Marker>(std::string("marker") + num, 10);
    }

    pub_local_map_ = node_handle_ptr_ -> advertise<sensor_msgs::PointCloud2> ("/retrived_map", 10);

    {
        //设置各种订阅器
        //设置订阅当前node
        boost::function<void(const std_msgs::Int32 &)> sub_cur_node_call_back;
        sub_cur_node_call_back = boost::bind(&Map::subCurNodeCallBack, this, _1);
        sub_cur_node_ = node_handle_ptr_-> subscribe<const std_msgs::Int32 &>("/cur_node", 10, sub_cur_node_call_back);
        //sub_cur_node_ = node_handle_ptr_-> subscribe("/cur_node", 10, &Map::subCurNodeCallBack, this);

        //设置当前路径
        boost::function<void(const std_msgs::Int32MultiArray &)> sub_paths_call_back;
        sub_paths_call_back = boost::bind(&Map::subPathsCallBack, this, _1);
        sub_paths_ = node_handle_ptr_-> subscribe<const std_msgs::Int32MultiArray &>("/paths", 10, sub_paths_call_back);
        //sub_paths_ = node_handle_ptr_-> subscribe("/paths", 10, &Map::subPathsCallBack, this);
        //订阅重启开关
        //一旦修改当前路径，或当前node,就会重启
        boost::function<void(const std_msgs::Int32 &)> sub_reboot_call_back;
        sub_cur_node_call_back = boost::bind(&Map::subRebootCallBack, this, _1);
        sub_reboot_ = node_handle_ptr_-> subscribe<const std_msgs::Int32 &>("/reboot", 10, sub_cur_node_call_back);
        //sub_reboot_ = node_handle_ptr_-> subscribe("/reboot", 10, &Map::subRebootCallBack, this);
    }

    //设置初始的第一个pose
    pose_.header.seq = 0;
    pose_.header.stamp = ros::Time::now();
    pose_.header.frame_id = param::cloud2map::frame;

    //设置画common area的粉色方框。
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

    markers_.resize(param::max_markers);
    for (int i = 0; i < param::max_markers; ++ i)
        markers_[i] = marker;
    need_reboot_ = false;   //默认不需要重启
}

void Map::setInitPose(geometry_msgs::Pose pose)
{
    pose_.pose.pose = pose;
}

void Map::core()
{

    GOTO_CORE_AGAIN:;
    while (true == need_reboot_)
    {
        std::chrono::milliseconds dura(1000);
        std::this_thread::sleep_for(dura);
        ROS_WARN_STREAM("need reboot:" << need_reboot_);
        ros::spinOnce();
        //等待一个手动信号，来启动程序
    }

    if (paths_.size() && paths_[0] == cur_node_id_)
    {
        paths_.erase(paths_.begin());
        //ROS_WARN("path第一个位置为当前位置,我会删除这个位置（这个消息没用）");
        ROS_WARN("delete the first position");
    }
    else
    {
        // ROS_WARN("path第一个位置不是当前位置(这个消息没用)");
        ROS_WARN("do not need delete the first position");
    }

    if (paths_.size() == 0)
    {
        ROS_WARN_STREAM("only one node");
        std::vector< std::vector<cv::Point2f> > contours;
        contours.clear();
        setMarker(contours);    //设置Marker,实际上就是空
        Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
        publishPose();
        publishLocalMap();
        run(&trans, &contours);
    }
    else
    {
        //给marker设置公共区域
        auto &edge = map_[cur_node_id_][paths_[0]];
        auto contours = edge.contours();
        setMarker(contours);


        Eigen::Matrix4d last_trans = Eigen::Matrix4d::Identity();
        cur_pos_ = 0;
        publishPose();   //第一次给一个init pose的信息
        publishLocalMap();  //发布一下第一个localMap

        for (int i = 0; i != paths_.size() - 1; ++i) {
            auto &edge = map_[cur_node_id_][paths_[i]];
            //std::cout<< cur_node_id_<<" "<<paths_[i] << std::endl;
            CHECK_EQ(edge.is_connceted(), true, "the two node is not connected!");
            last_trans = edge.transformational_matrix();

            run(&last_trans, &contours);
            if (true == need_reboot_)   break;  //需要重启
        }
        //run(&last_trans, &contours);
        run(NULL, NULL);
    }

    need_reboot_ = true;
    goto GOTO_CORE_AGAIN;
}


void Map::setPointCloud(int idx, const std::string &file) {
    CHECK_GE( max_nodes_,idx, "新建的node编号过大，不符合要求（0 based）")
    nodes_info_[idx] = NodeInfo(idx, file);

}

void Map::setXY(int X, int Y) {
    X_ = X;
    Y_ = Y;
    grid_map_ = cv::Mat(X, Y, CV_8U);
}

void Map::setGridMapFromFile(const std::string & file) {
    grid_map_ = cv::imread(file, CV_LOAD_IMAGE_COLOR);
    X_ = grid_map_.rows;
    Y_ = grid_map_.cols;
    cv_file_ = file;
}

Map::Map(int max_nodes) {
    map_.resize(max_nodes);
    for (int i = 0; i < max_nodes; ++i)
    {
        map_[i].resize(max_nodes);
    }
    max_nodes_ = max_nodes;
    //max_edges_ = UNKNOW;
    nodes_info_.resize(max_nodes);
}

void Map::add_edge(int from, int to, int edge_id, Eigen::Matrix4d trans_mat)        //directed edge!!
{
    //max_edges_ = std::max(max_edges_, edge_id + 1);
    auto &edge = map_[from][to];

    if (edge.is_connceted())
    {
        std::cout<<from<<"->"<<to<<std::endl;
        std::cout<<"如果你确定有重复边，请修改代码map::add_edge函数" << std::endl;
        exit(0);
    } //重复边
    edge = EdgeInfo(&nodes_info_[from], &nodes_info_[to], edge_id,  trans_mat);
}

void Map::save(const std::string &file_name) {
    Json::Value root;
    root["date"] = junk_map::getCurrentSystemTime();
    root["max_nodes"] = max_nodes_;
    root["cv_file"] = cv_file_;
    int cnt=0;
    for (int i = 0; i < max_nodes_; ++i)
    {
        for (int j = 0; j < max_nodes_; ++ j)
        {
            if (false == map_[i][j].is_connceted())  continue;
            root["edges"][cnt++] = map_[i][j].json();
        }
        root["nodes"][i] = nodes_info_[i].json();
    }
    for (int i = 0; i < paths_.size(); ++ i)
    {
        root["path"][i] = paths_[i];
    }
    root["init_pose"]["position"]["x"] = pose_.pose.pose.position.x;
    root["init_pose"]["position"]["y"] = pose_.pose.pose.position.y;
    root["init_pose"]["position"]["z"] = pose_.pose.pose.position.z;

    root["init_pose"]["orientation"]["x"] = pose_.pose.pose.orientation.x;
    root["init_pose"]["orientation"]["y"] = pose_.pose.pose.orientation.y;
    root["init_pose"]["orientation"]["z"] = pose_.pose.pose.orientation.z;
    root["init_pose"]["orientation"]["w"] = pose_.pose.pose.orientation.w;

    std::ofstream of(file_name);
    of << root;
    of.flush();
}

void Map::load(const std::string &file_name) {
    Json::Value root;
    std::ifstream ifs(file_name);
    ifs >> root;
    ifs.close();

    //assert(max_edges_ == root["max_edges"].asInt());
    max_nodes_ = root["max_nodes"].asInt();
    cv_file_ = root["cv_file"].asString();
    if (cv_file_ != "no_file")
    {
        setGridMapFromFile(cv_file_);
    }
    map_.resize(max_nodes_);
    for (int i = 0; i < max_nodes_; ++i) {
        map_[i].resize(max_nodes_);
    }

    nodes_info_.resize(max_nodes_);

    for (auto json : root["nodes"])
    {
        int node_number = json["number"].asInt();
        nodes_info_[ node_number ].loadFromJson(json);
    }

    for (auto json : root["edges"])
    {
        int from = json["from"].asInt();
        int to = json["to"].asInt();
        map_[from][to].loadFromJson(json, nodes_info_);
    }
    paths_.clear();
    for (auto json : root["path"])
    {
        paths_.push_back(json.asInt());
    }

    pose_.pose.pose.position.x = root["init_pose"]["position"]["x"].asDouble() ;
    pose_.pose.pose.position.y = root["init_pose"]["position"]["y"].asDouble() ;
    pose_.pose.pose.position.z = root["init_pose"]["position"]["z"].asDouble() ;

    pose_.pose.pose.orientation.x = root["init_pose"]["orientation"]["x"].asDouble();
    pose_.pose.pose.orientation.y = root["init_pose"]["orientation"]["y"].asDouble();
    pose_.pose.pose.orientation.z = root["init_pose"]["orientation"]["z"].asDouble();
    pose_.pose.pose.orientation.w = root["init_pose"]["orientation"]["w"].asDouble();
}

void Map::SPFA(int from, int to, std::vector<int> & nodes) //球from 到to的路径，经过的路径保存在nodes中，经过的边保存在edges中
{
#define MAX_DIST 0x3f3f3f3f
#define NOT !
    nodes.clear();
    static std::bitset<1<<12> in_queue( 0 );
    static std::vector<int> dist;
    static std::queue<int> q;
    static std::vector<int> pre;
    for (int i=0;i<1<<12;++ i) in_queue[i] = 0;
    assert( q.empty() );
    dist.resize( max_nodes_ );
    pre.resize( max_nodes_ );
    for (auto &x : dist)    x = MAX_DIST;
    for (auto &x : pre) x = -1;
    in_queue[from] = 1;
    dist[from] = 0;
    q.push(from);

    while ( NOT q.empty() )
    {
        int from = q.front();
        q.pop();
        in_queue[from] = false;
        for (int to = 0; to < max_nodes_; ++ to)
        {
            if (NOT map_[from][to].is_connceted())  continue;
            int cost = map_[from][to].cost();   //maybe TODO
            if (dist[to] > dist[from] + cost)
            {
                dist[to] = dist[from] + cost;
                if ( NOT in_queue[to])
                {
                    in_queue[to] = true;
                    q.push(to);
                }
                pre[to] = from;
            }
        }
    }
    for (int i = 0; i < max_nodes_; ++ i)
    {
        pr(i),prln(dist[i]);
    }

    for (int node = to; node != from; node = pre[node])
    {
        nodes.push_back( node );
    }

    nodes.push_back(from);

    std::reverse( nodes.begin(), nodes.end() );

#undef NOT
#undef MAX_DIST
}

void Map::debug()
{
    std::vector<GPS> x;
    x.resize(5);
    nodes_info_[0].setGPSs( x );

    x.resize(4);
    nodes_info_[2].setGPSs( x );
    //and so on... just for test
}

