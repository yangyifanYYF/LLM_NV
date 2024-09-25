/*********************************************************************
 *
 * Author: Zhang Qianyi
 * E-mail: zhangqianyi@mail.nankai.edu.cn
 * Last update: 2022.10.3
 *
 *********************************************************************/
#include "planner_voronoi.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    voronoi_planner::VoronoiPlanner planner_(n);
    ros::spin();
    return 0;
}