//
// Created by bohuan on 17-12-19.
//

#include "simple_3d_to_2d.h"

//默认DP类型为double，而不是float。 不保证用float程序能正确运行(也许能正确运行)
typedef PointMatcher<double> PM;
typedef PointMatcher<double>::DataPoints DP;

DP dp;
Simple3DTo2D solve;
cv::Mat mat;

int main(int argc, char* argv[])
{
    dp = DP::load("/home/bohuan/map/map0.vtk");
    //设置
    BoundingBox bb;
    bb.setMaxX(150.5); //设置需要选中的点的X，Y坐标的范围
    bb.setMaxY(200.4);
    bb.setMinX(-50.3);
    bb.setMinY(-100.1);
    bb.setX(1000);   //设置地图分辨率，也就是地图的X，Y轴上格点数。必须为整数。
    bb.setY(2000);
    solve.setBoundingBox(bb);
    auto isVaild1 = [](geometry_msgs::Point point)->bool {
        //判断一个point是否合法，合法返回true,否则返回false. 可以简单的滤掉很多点
        point.z -= 0.75;
        if (point.z < 0.2)    return false;
        if (point.z > 1.5)    return false;
        return true;
    };

    auto isVaild2 = [](geometry_msgs::Point point)->bool {
        //一切都合法
        return true;
    };

    solve.setBoundingBox(bb);
    //可以分别尝试isVaild1和isVaild2这2个参数，对比效果
    solve.solve(dp, mat, isVaild2);
}

