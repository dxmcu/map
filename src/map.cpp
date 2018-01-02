#include "pointmatcher/PointMatcher.h"
#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/OccupancyGrid.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <nav_msgs/OccupancyGrid.h>

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

#include <json/json.h>
#include "common.h"
#include "cloud2map.h"
#include "common_area.h"
#include "map_db_node.h"
#include "ugv_localizer/LocalMapRetrieve.h"
#include "visualization_msgs/Marker.h"

//debug

namespace junk_map {

    Cloud2Map cloud2map;

    const std::vector<int> compression_params(CV_IMWRITE_PNG_COMPRESSION, 100);

    Eigen::Matrix4d returnEigen(double th = 0) {
        Eigen::Matrix4d T;
        T << cos(th), -sin(th), 0, 0,
                sin(th), cos(th), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
        return std::move(T);
    }

    Eigen::Matrix4d json2Mat(Json::Value root) {
        Eigen::Matrix4d ret;
        std::stringstream x;
        x << root["transformational_matrix"].asString().c_str();
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
            {
                double tmp;
                x >> tmp;
                ret(i, j) = tmp;
            }
        return std::move(ret);
    }

    Json::Value mat2Json(Eigen::Matrix4d mat) {
        Json::Value root;
        std::stringstream trans_mat;
        trans_mat << mat;
        return root["transformational_matrix"] = trans_mat.str();
    }

    const std::string getCurrentSystemTime()
    {
        struct timeval s_now;
        struct tm* p_tm;
        gettimeofday( &s_now,NULL );
        p_tm = localtime( ( const time_t* )&s_now.tv_sec );
        char date[ 60 ] = { 0 };
        sprintf(date, "%d-%02d-%02d      %02d:%02d:%02d",
                ( int )p_tm->tm_year + 1900,( int )p_tm->tm_mon + 1,( int )p_tm->tm_mday,
                ( int )p_tm->tm_hour,( int )p_tm->tm_min,( int )p_tm->tm_sec);
        return std::move( std::string( date ) );
    }
}



struct GPS
{
    double latitude = -1;
    double longtitude = -1;
    double altitude = -1;
    Eigen::Matrix4d transformational_matrix = junk_map::returnEigen();

    Json::Value json()
    {
        Json::Value root;
        root["latitude"] = latitude;
        root["longtitude"] = longtitude;
        root["altitude"] =  altitude;
        root["transformational_matrix"] = junk_map::mat2Json(transformational_matrix);
        return std::move( root );
    }

    void loadFromJson(Json::Value root)
    {
        latitude = root["latitude"].asDouble();
        longtitude = root["longtitude"].asDouble();
        altitude = root["altitude"].asDouble();
        transformational_matrix = junk_map::json2Mat(root);
    }
};

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

class EdgeInfo
{
private:
    int edge_id_;           //这条边的编号
    Eigen::Matrix4d transformational_matrix_; //= junk_map::returnEigen();    //to到from的变换矩阵
    NodeInfo* from_point_cloud_ptr_;        //保存from指向的node
    NodeInfo* to_point_cloud_ptr_;
    bool is_connected_;                     //是否连接。
    std::vector< std::vector<cv::Point2f> > contours_;  //两个node的公共区域（在from身上画出,5个点，最后一个点和第一个点相同的一个矩形） 公共区域不止一个区域


public:

    std::vector< std::vector<cv::Point2f> > contours()  const
    {
        return contours_;
    }

    EdgeInfo()
    {
        is_connected_ = false;
    }

    Eigen::Matrix4d transformational_matrix()  const
    {
        return transformational_matrix_;
    }

    EdgeInfo(NodeInfo * from, NodeInfo * to, int edge_id, const  Eigen::Matrix4d & trans_mat)
    {
        is_connected_ = true;
        edge_id_ = edge_id;
        from_point_cloud_ptr_ = from;
        to_point_cloud_ptr_ = to;
        transformational_matrix_ = trans_mat;

        //计算contour_;
        //TODO 改为不用再拼命读地图
        static Cloud2Map cm0;
        static Cloud2Map cm1;
        cm0.loadPointCloudFromPCL(from -> PCLFile());
        cm0.PCL2Map();
        auto og1 = cm0.occupancy_grid();

        cm1.loadPointCloudFromPCL(to -> PCLFile());
        cm1.transFromPointCloud(trans_mat);
        cm1.PCL2Map();
        auto og2 = cm1.occupancy_grid();

        std::vector< std::vector< geometry_msgs::Point > >com= CommonArea::commonOccupancyGrid(og1, og2);

        //取出轮廓线，用cv来判断点是否在轮廓内
        contours_.resize(com.size());
        for (int p = 0; p < contours_.size(); ++ p)
        {
            contours_[p].resize(com[p].size());
            for (int i = 0 ; i < contours_[p].size(); ++ i)
            {
                contours_[p][i].x = com[p][i].x;
                contours_[p][i].y = com[p][i].y;
            }
            contours_[p].push_back( contours_[p][0] );  //保证为一个圈,首尾闭合（其实也不是特别重要）
        }
    }

    int from()  const { return from_point_cloud_ptr_ -> node_number();  }
    int to()    const { return to_point_cloud_ptr_ -> node_number();    }
    bool is_connceted()  const   {   return is_connected_;    }

    int cost()      //可以改写,表示这个边的权重。
    {
        //TODO something
        return 1;
    }

    Json::Value json()
    {
        Json::Value root;
        root["is_connected"] = is_connected_;
        root["edge_id"] = edge_id_;
        root["from"] = from();
        root["to"] = to();
        root["transformational_matrix"] = junk_map::mat2Json(transformational_matrix_);
        for (int i = 0; i < contours_.size(); ++ i)
        {
            for (int j = 0; j < contours_[i].size(); ++ j) {
                root["contour"][i][j]["x"] = contours_[i][j].x;
                root["contour"][i][j]["y"] = contours_[i][j].y;
            }
        }
        return std::move( root );
    }

    void loadFromJson(Json::Value root, std::vector< NodeInfo > &nodes_info)
    {
        if ( ! root["is_connected"].asBool() )
        {
            std::cout<<"有一条边的状态是不存在？如果确实有不存在的边，请修改edge::loadFromJson"<<std::endl;
            exit(0);
            return;
        }
        is_connected_ = true;
        edge_id_ = root["edge_id"].asInt();
        from_point_cloud_ptr_ = &nodes_info[ root["from"].asInt() ];
        to_point_cloud_ptr_ = &nodes_info[ root["to"].asInt() ];
        transformational_matrix_ = junk_map::json2Mat(root);
        int cnt = 0;
        contours_.resize(root["contour"].size());
        for (int i = 0; i < root["contour"].size(); ++ i)
        {
            contours_[i].resize( root["contour"][i].size() );
            for  (int j = 0; j < root["contour"][i].size(); ++ j)
            {
                contours_[i][j].x = root["contour"][i][j]["x"].asDouble();
                contours_[i][j].y = root["contour"][i][j]["y"].asDouble();
            }
        }
    }
};


class Map {
private:
    int X_, Y_;
    std::vector<std::vector< EdgeInfo > >  map_;

    cv::Mat grid_map_;

    int max_nodes_;             //the number of nodes

    std::vector<NodeInfo> nodes_info_;
    std::string cv_file_ = "no_file";

    std::vector<int> paths_; //要经过的路径. front为即将要去的点。 如果为空，则当前已经在目标点内。
    int cur_pos_;   //表示当前到path的第几个位置.
    //geometry_msgs::Pose cur_pose_;  //当前位置的pose_, 世界坐标系的pose
    int cur_node_id_;   //当前所处的node

    ros::NodeHandle *node_handle_ptr_;
    ros::Publisher pub_initial_pose_;
    std::vector<ros::Publisher> pub_markers_;   //不止一个marker
    ros::Publisher pub_local_map_;
    //nav_msgs::Odometry cur_odometry_;   //貌似没用到这个信息

    ros::Subscriber sub_cur_node_;
    ros::Subscriber sub_paths_;
    ros::Subscriber sub_reboot_;

    geometry_msgs::PoseWithCovarianceStamped pose_;
    std::vector<visualization_msgs::Marker> markers_;
    int num_marker_;        //活动的marker的数量
    bool need_reboot_;       //是否要重启。 默认是false，不用重启。因为修改cur_node和paths后，相当于需要重启

    void subCurNodeCallBack(const std_msgs::Int32 & cur_node_id)
    {
        setCurNodeID(cur_node_id.data);
        ROS_WARN_STREAM("set node:" << cur_node_id);
        need_reboot_ = true;
    }

    void subPathsCallBack(const std_msgs::Int32MultiArray & paths )
    {
        setPaths(paths.data);
        std::stringstream ss;
        for (auto x : paths.data)   ss<<x<<" ";
        ROS_WARN_STREAM("set paths:" << ss.str());
        need_reboot_ = true;
    }

    void subReboot(const std_msgs::Int32& need_reboot)
    {
        need_reboot_ = need_reboot.data;
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
#ifdef DEBUG
        ROS_INFO_STREAM("num_marker = " << num_marker_);
#endif
        for (int i = 0; i < num_marker_; ++ i)
        {
            pub_markers_[i].publish(markers_[i]);
        }
    }

    //判断pose是否在多边形其中一个里面。是的话返回true,否则返回false
    bool pointPolygonsTest(std::vector< std::vector<cv::Point2f> > * contours_ptr, cv::Point2f pose)
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

    bool retrieveCallback(ugv_localizer::LocalMapRetrieve::Request &req,
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
    void run(Eigen::Matrix4d * trans_ptr, std::vector< std::vector<cv::Point2f>> * contours_ptr = NULL) //到达公共区域，就结束
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


    int odometryCallBack(const nav_msgs::OdometryConstPtr &odo, nav_msgs::Odometry * odometry)
    {
        *odometry = *odo;
        return 0;
    };

    void transforPose(Eigen::Matrix4d trans)
    {
        //auto pose = cur_odometry_.pose;
        std::cout << trans << std::endl;
        //   ROS_INFO_STREAM(trans);
        //   ROS_INFO_STREAM(pose_);

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

    void publishPose()
    {
        //多发几次，确保能收到
        for (int i = 1; i<= 1;++i)
        {
            pub_initial_pose_.publish(pose_);
            //ros::Duration(1).sleep();
        }
        ROS_INFO("/initialpose is published");
    }

    void publishLocalMap()
    {
        static sensor_msgs::PointCloud2 cloud;
        nodes_info_[cur_node_id_].pos2cloud(pose_, cloud);
        //发布3次
       // std::cout << cur_node_id_ << std::endl;
        ROS_INFO_STREAM("cur node id is:" << cur_node_id_);
        for (int i = 1;i<=1;++i)
        {
            pub_local_map_.publish(cloud);  //发布地图
        }
    }


public:

    ~Map()
    {
        //delete sub_odometry_ptr_;
        delete node_handle_ptr_;
    }

    void setCurNodeID(int id)
    {
        cur_node_id_ = id;
    }

    void setPose()
    {
        //TODO
    }

    void setPaths(const std::vector<int> & path)
    {
        paths_ = path;
    }

    void initNodeHandle(ros::NodeHandle * nh = NULL)
    {
        if (nh == NULL)
        {
            node_handle_ptr_ = new ros::NodeHandle();
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
            char num[10];
            sprintf(num,"%d", i);
            pub_markers_[i] = node_handle_ptr_->advertise<visualization_msgs::Marker>(std::string("marker") + num, 10);
        }

        pub_local_map_ = node_handle_ptr_ -> advertise<sensor_msgs::PointCloud2> ("/retrived_map", 10);

        {
            //设置各种订阅器
            //设置订阅当前node
            boost::function<void(const std_msgs::Int32 &)> sub_cur_node_call_back;
            sub_cur_node_call_back = boost::bind(&Map::subCurNodeCallBack, this, _1);
            sub_cur_node_ = node_handle_ptr_->subscribe<const std_msgs::Int32 &>("/cur_node", 10,
                                                                                 sub_cur_node_call_back);

            //设置当前路径
            boost::function<void(const std_msgs::Int32MultiArray &)> sub_paths_call_back;
            sub_paths_call_back = boost::bind(&Map::subPathsCallBack, this, _1);
            sub_paths_ = node_handle_ptr_->subscribe<const std_msgs::Int32MultiArray &>("/paths", 10,
                                                                                        sub_paths_call_back);
            //订阅重启开关
            //一旦修改当前路径，或当前node,就会重启
            boost::function<void(const std_msgs::Int32 &)> sub_reboot_call_back;
            sub_cur_node_call_back = boost::bind(&Map::subReboot, this, _1);
            sub_reboot_ = node_handle_ptr_->subscribe<const std_msgs::Int32 &>("/reboot", 10, sub_cur_node_call_back);
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

    void setInitPose(geometry_msgs::Pose pose)
    {
        pose_.pose.pose = pose;
    }

    void core()
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

    cv::Mat &grid_map() { return grid_map_; }

    void setPointCloud(int idx, const std::string &file) {
        CHECK_GE( max_nodes_,idx, "新建的node编号过大，不符合要求（0 based）")
        nodes_info_[idx] = NodeInfo(idx, file);

    }

    void setXY(int X, int Y) {
        X_ = X;
        Y_ = Y;
        grid_map_ = cv::Mat(X, Y, CV_8U);
    }

    void setGridMapFromFile(const std::string & file) {
        grid_map_ = cv::imread(file, CV_LOAD_IMAGE_COLOR);
        X_ = grid_map_.rows;
        Y_ = grid_map_.cols;
        cv_file_ = file;
    }

    Map(int max_nodes) {
        map_.resize(max_nodes);
        for (int i = 0; i < max_nodes; ++i)
        {
            map_[i].resize(max_nodes);
        }
        max_nodes_ = max_nodes;
        //max_edges_ = UNKNOW;
        nodes_info_.resize(max_nodes);
    }

    void add_edge(int from, int to, int edge_id, Eigen::Matrix4d trans_mat)        //directed edge!!
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

    void save(const std::string &file_name) {
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

    void load(const std::string &file_name) {
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

    void SPFA(int from, int to, std::vector<int> & nodes) //球from 到to的路径，经过的路径保存在nodes中，经过的边保存在edges中
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

    void debug()
    {
        std::vector<GPS> x;
        x.resize(5);
        nodes_info_[0].setGPSs( x );

        x.resize(4);
        nodes_info_[2].setGPSs( x );
        //and so on... just for test
    }
};

Map map(11);

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "map");
    map.initNodeHandle();


#define LOAD
#ifdef LOAD

    map.load("/home/bohuan/map_test/json");
    std::vector< int > nodes;
    map.SPFA(0, 4, nodes);
    std::cout <<"nodes" << std::endl;
    for (auto x : nodes)
        std::cout << x<<" ";std::cout<<std::endl;
    map.save("/home/bohuan/map_test/json_test");
    map.core();
    return 0;

#endif

    ROS_INFO("-------------loading nodes-----------");
    map.setPointCloud(0, "/home/bohuan/map_data/pcd/0.pcd");
    map.setPointCloud(1, "/home/bohuan/map_data/pcd/1.pcd");
    map.setPointCloud(2, "/home/bohuan/map_data/pcd/2.pcd");
    map.setPointCloud(3, "/home/bohuan/map_data/pcd/3.pcd");
    map.setPointCloud(4, "/home/bohuan/map_data/pcd/4.pcd");
    map.setPointCloud(5, "/home/bohuan/map_data/pcd/5.pcd");
    map.setPointCloud(6, "/home/bohuan/map_data/pcd/6.pcd");
    map.setPointCloud(7, "/home/bohuan/map_data/pcd/7.pcd");
    map.setPointCloud(8, "/home/bohuan/map_data/pcd/8.pcd");
    map.setPointCloud(9, "/home/bohuan/map_data/pcd/9.pcd");
    map.setPointCloud(10, "/home/bohuan/map_data/pcd/10.pcd");


    ROS_INFO("-------------loading edges-----------");
    map.add_edge(0, 1, 0, param::from1to0());
    map.add_edge(1, 0, 0, param::from1to0().inverse());

    map.add_edge(1, 2, 1, param::from2to1());
    map.add_edge(2,1,1, param::from2to1().inverse());

    map.add_edge(2,3,2, param::from3to2());
    map.add_edge(3,2,2, param::from3to2().inverse());


    map.add_edge(3,4, 3, param::from4to3());
    map.add_edge(4,3, 3, param::from4to3().inverse());

    map.add_edge(4,5, 4, param::from5to4());
    map.add_edge(5,4, 4, param::from5to4().inverse());

    map.add_edge(0,5, 5, param::from5to0());
    map.add_edge(5,0, 5, param::from5to0().inverse());

    map.add_edge(1,5, 6, param::from5to1());
    map.add_edge(5,1, 6, param::from5to1().inverse());

    map.add_edge(1,6, 7, param::from6to1());
    map.add_edge(6,1, 7, param::from6to1().inverse());

    map.add_edge(6,7, 8, param::from7to6());
    map.add_edge(7,6, 8, param::from7to6().inverse());

    map.add_edge(7,8, 9, param::from8to7());
    map.add_edge(8,7, 9, param::from8to7().inverse());

    map.add_edge(4,8, 10, param::from8to4());
    map.add_edge(8,4, 10, param::from8to4().inverse());

    map.add_edge(3,6, 11, param::from6to3());
    map.add_edge(6,3, 11, param::from6to3().inverse());

    map.add_edge(1,8, 12, param::from8to1());
    map.add_edge(8,1, 12, param::from8to1().inverse());

    map.add_edge(1,7, 13, param::from7to1());
    map.add_edge(7,1, 13, param::from7to1().inverse());

    map.add_edge(3,7, 14, param::from7to3());
    map.add_edge(7,3, 14, param::from7to3().inverse());


    map.add_edge(8,9, 15, param::from9to8());
    map.add_edge(9,8, 15, param::from9to8().inverse());

    map.add_edge(4,9, 16, param::from9to4());
    map.add_edge(9,4, 16, param::from9to4().inverse());

    map.add_edge(5,9, 17, param::from9to5());
    map.add_edge(9,5, 17, param::from9to5().inverse());

    map.add_edge(9,10, 18, param::from10to9());
    map.add_edge(10,9, 18, param::from10to9().inverse());

    map.add_edge(6,10, 19, param::from10to6());
    map.add_edge(10,6, 19, param::from10to6().inverse());

    map.add_edge(7,10, 20, param::from10to7());
    map.add_edge(10,7, 20, param::from10to7().inverse());

    map.add_edge(8,10, 20, param::from10to8());
    map.add_edge(10,8, 20, param::from10to8().inverse());

    map.add_edge(5,10, 20, param::from10to5());
    map.add_edge(10,5, 20, param::from10to5().inverse());

    map.add_edge(4,10, 20, param::from10to4());
    map.add_edge(10,4, 20, param::from10to4().inverse());

    //map.debug();debug用，可以手动塞一些gps信息进去。暂且不重要。
    //
    map.setCurNodeID(0);
       std::vector<int> path = {0,1,2,3,4,5,1,6,3,7,1,8,9,5,1,6,10,4,3,2,1,0};
    //std::vector<int> path = {0,1,2,3,4};
    map.setPaths(path);


    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = -0.703935902556;
    pose.orientation.w = 0.710263503984;
    map.setInitPose(pose);



    //map.core();

    map.save("/home/bohuan/map_test/json");
    return 0;
}

