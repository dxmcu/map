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

#include <json/json.h>
#include "common.h"
#include "cloud2map.h"
#include "common_area.h"
#include "map_db_node.h"
#include "ugv_localizer/LocalMapRetrieve.h"
#include "visualization_msgs/Marker.h"

//debug
#define DEBUG

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
    int edge_id_;
    Eigen::Matrix4d transformational_matrix_; //= junk_map::returnEigen();    //to到from的变换矩阵
    NodeInfo* from_point_cloud_ptr_;
    NodeInfo* to_point_cloud_ptr_;
    bool is_connected_;

public:

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
    }
};


/*class MapNavigation{
private:
    std::queue<int> paths_; //要经过的路径. front为即将要去的点。 如果为空，则当前已经在目标点内。
    geometry_msgs::Pose pose_;  //当前位置的pose_
    int Node_ID_;   //当前所在的Node 的id
public:
    //TODO
};*/

class Map {
private:
    int X_, Y_;
    std::vector<std::vector< EdgeInfo > >  map_;

    cv::Mat grid_map_;

    int max_nodes_;             //the number of nodes

    std::vector<NodeInfo> nodes_info_;
    std::string cv_file_ = "no_file";

    std::vector<int> paths_; //要经过的路径. front为即将要去的点。 如果为空，则当前已经在目标点内。
    geometry_msgs::Pose cur_pose_;  //当前位置的pose_, 世界坐标系的pose
    int cur_node_id_;   //当前所处的node

    ros::NodeHandle *node_handle_ptr_;
    ros::Subscriber *sub_odometry_ptr_;
    ros::Publisher pub_initial_pose_;
    ros::Publisher pub_marker_;
    ros::Publisher pub_local_map_;
    nav_msgs::Odometry cur_odometry_;

    geometry_msgs::PoseWithCovarianceStamped pose_;
    visualization_msgs::Marker marker_;


    bool retrieveCallback(ugv_localizer::LocalMapRetrieve::Request &req,
                          ugv_localizer::LocalMapRetrieve::Response &res,
                          cv::Point2f * pose, ros::Publisher & pub_local_map)
    {
        //TODO 考虑 pose的变换关系,去掉pose和pub_local_map的东西
        auto &node = nodes_info_[cur_node_id_];
        node.retrieve(req.poseWithStamp, res.pointcloud);   //给一个位姿，返回一个点云。同时自带发布消息
        pose -> x =req.poseWithStamp.pose.position.x;
        pose -> y =req.poseWithStamp.pose.position.y;
        std::cout<<pose -> x<<" "<<pose -> y<<std::endl;

        pub_local_map_.publish(res.pointcloud);    //发布出这个地图
        pub_marker_.publish(marker_); //发布地图公共区域的问题。

        std::cout<<"published && cur_node_id = "<<cur_node_id_<<std::endl;
        return true;
    }

    //在一个node里
    void run(std::vector<cv::Point2f> * contour_ptr = NULL) //到达公共区域，就结束
    {
        auto &node = nodes_info_;
        cv::Point2f pose;
        pose.x = pose_.pose.pose.position.x;
        pose.y = pose_.pose.pose.position.y;




        boost::function<bool(ugv_localizer::LocalMapRetrieve::Request&, ugv_localizer::LocalMapRetrieve::Response&)> call_back ;
        static ros::Publisher pub_local_map = node_handle_ptr_ -> advertise<sensor_msgs::PointCloud2> ("/retrived_map", 10);    //默认用来发布新local map的
        ROS_INFO("init pub_loacal_map OK");
        call_back = boost::bind(&Map::retrieveCallback, this, _1, _2, &pose, pub_local_map);
        ros::ServiceServer service = node_handle_ptr_ -> advertiseService("/local_map_retrieve", call_back);
        ROS_INFO("running node");

        int cnt=0;
        while (ros::ok())
        {
#ifdef DEBUG
            ++cnt;
            if (cnt%1000000==0)
            {
                std::cout << pose.x << " " << pose.y << std::endl;
            }
#endif
            if (contour_ptr && cv::pointPolygonTest(*contour_ptr, pose, false) == 1)
            {
                //如果在公共区域，则切换地图。
                break;
            }

            if (contour_ptr)
            {
                //发布marker
                pub_marker_.publish(marker_);
            }
            ros::spinOnce();
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
        Eigen::Isometry3d I = Eigen::Isometry3d::Identity();
        auto x = pose_.pose.pose.orientation.x;
        auto y = pose_.pose.pose.orientation.y;
        auto z = pose_.pose.pose.orientation.y;
        auto w = pose_.pose.pose.orientation.w;
        Eigen::Quaterniond Q(w, x, y, z);
        x = pose_.pose.pose.position.x;
        y = pose_.pose.pose.position.y;
        z = pose_.pose.pose.position.z;
        Eigen::Vector3d V(x, y, z);
        I.rotate(Q);
        I.translate(V);
        trans *= I.matrix();
        Q = Eigen::Quaterniond(I.matrix().block<3,3>(0,0));
        V = I.matrix().block<3, 1>(0,3);

        pose_.pose.pose.orientation.w = Q.matrix()(0);
        pose_.pose.pose.orientation.x = Q.matrix()(1);
        pose_.pose.pose.orientation.y = Q.matrix()(2);
        pose_.pose.pose.orientation.z = Q.matrix()(3);

        pose_.pose.pose.position.x = V(0);
        pose_.pose.pose.position.y = V(1);
        pose_.pose.pose.position.z = V(2);
    }

    void publishPose()
    {
        //发10此，确保发到
        for (int i = 1; i <= 10; ++ i)
        {
            pub_initial_pose_.publish(pose_);
        }
    }


public:

    ~Map()
    {
        delete sub_odometry_ptr_;
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
            node_handle_ptr_ = nh;
        }
    }




    void core()
    {
        if (paths_[0] == cur_node_id_)
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


        //TODO 改为可以直接读取的数据
        pose_.header.seq = 0;
        pose_.header.stamp = ros::Time::now();
        pose_.header.frame_id = param::cloud2map::frame;
        pose_.pose.pose.position.x = 2.0682258606;
        pose_.pose.pose.position.y = -18.2951946259;
        pose_.pose.pose.position.z = 0;
        pose_.pose.pose.orientation.x = 0;
        pose_.pose.pose.orientation.y = 0;
        pose_.pose.pose.orientation.z = -0.719839325025;
        pose_.pose.pose.orientation.w = 0.694140725032;




        pub_initial_pose_ = node_handle_ptr_ -> advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);


        boost::function<int(const nav_msgs::OdometryConstPtr&)> odom_handler ;
        odom_handler = boost::bind(&Map::odometryCallBack, this , _1, &cur_odometry_);
        sub_odometry_ptr_ = new ros::Subscriber( node_handle_ptr_ -> subscribe<nav_msgs::Odometry>("/laser_odom_to_map", 1, odom_handler));

        //发布公共区域框框用的
        pub_marker_ = node_handle_ptr_ -> advertise<visualization_msgs::Marker>("marker", 10);

        Eigen::Matrix4d last_trans = Eigen::Matrix4d::Identity();

        for (int i = 0; i != paths_.size(); ++ i)
        {
            //TODO  直接保存公共区域，应该是更好的方案，就不用在这里求一下耽误时间了。
            //TODO 也就是预处理就直接保存公共区域即可。

            transforPose(last_trans.inverse());
            //发布一次新的地图


            publishPose();  //因为地图坐标变换了，所以要发布位姿和地图。

            auto &edge = map_[cur_node_id_][paths_[i]];
            CHECK_EQ(edge.is_connceted(), true,  "the two node is not connected!");
            auto trans = edge.transformational_matrix();
            //nodes_info_[paths_[i]].transform( trans );

            static Cloud2Map cm0;
            static Cloud2Map cm1;
            cm0.loadPointCloudFromPCL(nodes_info_[cur_node_id_].PCLFile());
            cm0.PCL2Map();
            auto og1 = cm0.occupancy_grid();

            cm1.loadPointCloudFromPCL(nodes_info_[paths_[i]].PCLFile());
            cm1.transFromPointCloud(trans);
            cm1.PCL2Map();
            auto og2 = cm1.occupancy_grid();


            std::vector< geometry_msgs::Point > com= CommonArea::commonOccupancyGrid(og1, og2);
            ROS_INFO("the common area are:");
            for (auto point : com)
                printf("%lf %lf\n", point.x, point.y);


            //取出轮廓线，用cv来判断点是否在轮廓内
            std::vector<cv::Point2f> contour;
            contour.resize(4);
            for (int i = 0 ; i < 4; ++ i)
            {
                contour[i].x = com[i].x;
                contour[i].y = com[i].y;
            }


            //显示公共区域为marker类型
            //TODO写到参数里
            marker_.header.frame_id = "global_frame";
            marker_.header.stamp = ros::Time::now();
            marker_.header.seq = 0;
            marker_.type = marker_.LINE_STRIP;
            marker_.action = marker_.ADD;
            //marker.lifetime = ros::Duration(0.0);
            marker_.id = 1;
            marker_.pose.position.x = 0;
            marker_.pose.position.y = 0;
            marker_.pose.position.z = 0;
            marker_.color.a = 1;
            marker_.color.r = 1;
            marker_.color.g = 0;
            marker_.color.b = 1;

            marker_.scale.x = 2;
            marker_.scale.y = 2;
            marker_.scale.z = 2;

            marker_.points = com;
            marker_.points.push_back(com[0]);
            //junk_map::pub_marker.publish(junk_map::marker);



            run(&contour);

            std::cout<<"切换地图"<<std::endl;

            //nodes_info_[paths_[i]].invTransform( trans );
            cur_node_id_ = paths_[i];
            //TODO 坐标转换
        }




        //没有下一个点了
        //先实现一个node
        //run();
    }

    cv::Mat &grid_map() { return grid_map_; }

    void setPointCloud(int idx, const std::string &file) {
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
            std::cout<<"如果你确定有重复边，请修改代码map::add_edge函数" << std::endl;
            exit(0);
        } //重复边
        edge = EdgeInfo(&nodes_info_[from], &nodes_info_[to], edge_id,  trans_mat);
    }

    void save(const std::string &file_name) {
        Json::Value root;
        root["date"] = "2017.02.30_TODO"; //TODO
        //root["max_edges"] = max_edges_;
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

Map map(6);

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "map");
    ros::NodeHandle map_nh;
    map.initNodeHandle(&map_nh);


//#define LOAD
#ifdef LOAD

    map.load("/home/bohuan/map_test/json");
    std::vector< int > nodes;
    map.SPFA(0, 4, nodes);
    std::cout <<"nodes" << std::endl;
    for (auto x : nodes)
        std::cout << x<<" ";std::cout<<std::endl;
    map.save("/home/bohuan/map_test/json_test");
    return 0;

#endif

    map.setPointCloud(0, "/home/bohuan/map_data/pcd/0.pcd");
    map.setPointCloud(1, "/home/bohuan/map_data/pcd/1.pcd");
    map.setPointCloud(2, "/home/bohuan/map_data/pcd/2.pcd");
    map.setPointCloud(3, "/home/bohuan/map_data/pcd/3.pcd");
    map.setPointCloud(4, "/home/bohuan/map_data/pcd/4.pcd");
    map.setPointCloud(5, "/home/bohuan/map_data/pcd/5.pcd");


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

    //map.debug();debug用，可以手动塞一些gps信息进去。暂且不重要。

    //
    map.setCurNodeID(0);
    std::vector<int> path = {0,1,2,3,4,5};
    map.setPaths(path);
    map.core();


    map.save("/home/bohuan/map_test/json");
    return 0;
}

