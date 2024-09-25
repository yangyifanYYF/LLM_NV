/*********************************************************************
 *
 * Author: Zhang Qianyi
 * E-mail: zhangqianyi@mail.nankai.edu.cn
 * Last update: 2022.10.3
 *
 *********************************************************************/
#include "planner_voronoi.h"
namespace voronoi_planner {

VoronoiPlanner::VoronoiPlanner(ros::NodeHandle& nh){
    nh_ = nh;
    voronoi_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("voronoi_grid", 1);
    costmap_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/map",100, boost::bind(&VoronoiPlanner::costmapUpdateCallback, this ,_1));
    // costmap_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/move_base/local_costmap/costmap",100, boost::bind(&VoronoiPlanner::costmapUpdateCallback, this ,_1));

}

void VoronoiPlanner::costmapUpdateCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    // receives the map information from topic
    nx_ = msg->info.width;
    ny_ = msg->info.height;
    origin_x_ = msg->info.origin.position.x;
    origin_y_ = msg->info.origin.position.y;
    resolution_ = msg->info.resolution;
    frame_id_ = msg->header.frame_id;
    int map_size = msg->data.size();
    auto map_char = msg->data;

    // construct a 2D map. true represents the point is obstacle-occupied
    bool **map_bool = NULL;
    map_bool = new bool*[nx_];
    for (int x=0; x<nx_; x++) {
        map_bool[x] = new bool[ny_];
    }
    for (int i=0; i<map_size; i++){
        if ((int)map_char[i] != 100){
            map_bool[i%nx_][i/nx_] = false;
        }
        else{
            map_bool[i%nx_][i/nx_] = true;
        }
    }
    ros::Time t = ros::Time::now();
    // initialize voronoi object it with the map
    voronoi_.initializeMap(nx_, ny_, map_bool);
    // update distance map and Voronoi diagram
    voronoi_.update(); 
    // prune the Voronoi
    voronoi_.prune();  
    ROS_INFO("Time (voronoi): %f sec", (ros::Time::now() - t).toSec());

    publishVoronoiGrid(&voronoi_);

    for (int x=0; x<nx_; x++) {
        delete []map_bool[x];
    }
    delete []map_bool;
}






void VoronoiPlanner::publishVoronoiGrid(DynamicVoronoi *voronoi)
{
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = frame_id_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution_;

    grid.info.width = nx_;
    grid.info.height = ny_;

    grid.info.origin.position.x = origin_x_;
    grid.info.origin.position.y = origin_y_;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx_ * ny_);

    for (unsigned int x = 0; x < nx_; x++)
    {
        for (unsigned int y = 0; y < ny_; y++)
        {
            if(voronoi->isVoronoi(x,y) && !(x==0 || x==nx_-1 || y==0 || y==ny_-1))
                grid.data[x + y*nx_] = (char)127;
            else
                grid.data[x + y*nx_] = (char)0;
        }
    }

    voronoi_grid_pub_.publish(grid);
}



} //end namespace voronoi_planner

