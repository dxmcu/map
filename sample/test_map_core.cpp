//
// Created by bohuan on 18-1-2.
//

#include "map_core.h"

Map map(11);

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "map");
    map.initNodeHandle();

#define LOAD
#ifdef LOAD

    map.load("../config/config.json");
    std::vector< int > nodes;
    map.SPFA(0, 4, nodes);
    std::cout <<"nodes" << std::endl;
    for (auto x : nodes)
        std::cout << x<<" ";std::cout<<std::endl;
    map.save("../config/config(copy).json");
    map.core();
    return 0;

#endif


    ROS_INFO("-------------loading nodes-----------");
    map.setPointCloud(0,  "../pcd/0.pcd");
    map.setPointCloud(1,  "../pcd/1.pcd");
    map.setPointCloud(2,  "../pcd/2.pcd");
    map.setPointCloud(3,  "../pcd/3.pcd");
    map.setPointCloud(4,  "../pcd/4.pcd");
    map.setPointCloud(5,  "../pcd/5.pcd");
    map.setPointCloud(6,  "../pcd/6.pcd");
    map.setPointCloud(7,  "../pcd/7.pcd");
    map.setPointCloud(8,  "../pcd/8.pcd");
    map.setPointCloud(9,  "../pcd/9.pcd");
    map.setPointCloud(10, "../pcd/10.pcd");


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


    map.setCurNodeID(0);
    std::vector<int> path = {0,1,2,3,4,5,1,6,3,7,1,8,9,5,1,6,10,4,3,2,1,0};


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


    map.save("../config/config.json");
    return 0;
}

