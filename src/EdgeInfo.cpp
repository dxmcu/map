//
// Created by bohuan on 18-1-3.
//

#include "EdgeInfo.h"

#include "common.h"
#include "cloud2map.h"
#include "common_area.h"
#include "junk_map.h"



std::vector< std::vector<cv::Point2f> > EdgeInfo::contours()  const
{
    return contours_;
}

EdgeInfo::EdgeInfo()
{
    is_connected_ = false;
}

Eigen::Matrix4d EdgeInfo::transformational_matrix()  const
{
    return transformational_matrix_;
}

EdgeInfo::EdgeInfo(NodeInfo * from, NodeInfo * to, int edge_id, const  Eigen::Matrix4d & trans_mat)
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

int EdgeInfo::from()  const { return from_point_cloud_ptr_ -> node_number();  }
int EdgeInfo::to()    const { return to_point_cloud_ptr_ -> node_number();    }
bool EdgeInfo::is_connceted()  const   {   return is_connected_;    }

int EdgeInfo::cost()      //可以改写,表示这个边的权重。
{
    //TODO something
    return 1;
}

Json::Value EdgeInfo::json()
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

void EdgeInfo::loadFromJson(Json::Value root, std::vector< NodeInfo > &nodes_info)
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

