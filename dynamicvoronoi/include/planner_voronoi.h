#ifndef _PLANNERCORE_H
#define _PLANNERCORE_H
/*********************************************************************
 *
 * Author: Zhang Qianyi
 * E-mail: zhangqianyi@mail.nankai.edu.cn
 * Last update: 2022.10.3
 *
 *********************************************************************/
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/OccupancyGrid.h>
#include "dynamicvoronoi.h"


namespace voronoi_planner {

/**
 * @class PlannerCore
 * @brief Provides a ROS wrapper for the voronoi_planner planner on a costmap.
 */

class VoronoiPlanner{
    public:
        /**
         * @brief  Default constructor for the VoronoiPlanners object
         */
        VoronoiPlanner(ros::NodeHandle& nh);
        /**
         * @brief  Publish the voronoiMap
         */
        void publishVoronoiGrid(DynamicVoronoi *voronoi);
        /**
         * @brief  Subscribe global costmap and call dynamicvoronoi.h
         */
        void costmapUpdateCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    private:
        ros::NodeHandle nh_;
        std::string frame_id_;
        int nx_, ny_;
        double resolution_;
        double origin_x_,origin_y_;
        ros::Publisher voronoi_grid_pub_;
        ros::Subscriber costmap_sub_;
        DynamicVoronoi voronoi_;

};

} //end namespace voronoi_planner

#endif
