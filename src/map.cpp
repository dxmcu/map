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

#include <json/json.h>
#include "common.h"
#include "cloud2map.h"

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

class NodeInfo
{
private:
    //DP point_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr_ = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>()); ;
    std::string file_ = "no_file";  //点云的载入路径
    int node_number_ = -1;          //默认点的编号
    std::vector< GPS > gpss_;
    CVMat cvmat;

public:

    NodeInfo(){}

    int node_number()   const { return node_number_;    }

    GPS& getGPS(int idx) {   return gpss_[idx];   }
    GPS getGPS(int idx) const { return gpss_[idx];  }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr & point_cloud_ptr_ref()   {   return point_cloud_ptr_;    }

    void setGPSs(const std::vector< GPS > & gpss)
    {
        gpss_ = gpss;
    }

    NodeInfo(int node_number, const std::string & file)
    {
        node_number_ = node_number;
        file_ = file;
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (file_.c_str(), *point_cloud_ptr_) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
        {
            PCL_ERROR ((std::string("Couldn't read file from [") + file_ + std::string("]\n")).c_str());
            exit(0);
        }


        junk_map::cloud2map.setPCL(*point_cloud_ptr_);
        junk_map::cloud2map.PCL2Map();//做一次处理


        cvmat = junk_map::cloud2map.getCVMat();
        //TODO保存png格式
        cvmat.file_name = file_ + ".png";
        //保存为png格式，这里有问题。 保存的格式不理想
        /*
        cv::imshow(file_, cvmat());
        cv::namedWindow(file_, 0);
        cv::waitKey(-1);
         */
        cv::imwrite(cvmat.file_name.c_str(), cvmat.mat, junk_map::compression_params);
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

class Map {
private:
    int X_, Y_;
    std::vector<std::vector< EdgeInfo > >  map_;

    cv::Mat grid_map_;

    int max_nodes_;             //the number of nodes

    std::vector<NodeInfo> nodes_info_;
    std::string cv_file_ = "no_file";

public:

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


#define LOAD
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

    map.debug();

    map.save("/home/bohuan/map_test/json");
    return 0;
}

