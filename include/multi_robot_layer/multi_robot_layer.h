#ifndef MULTI_ROBOT_LAYER_H_
#define MULTI_ROBOT_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <boost/algorithm/string.hpp>
#include <multi_robot_layer/robot_type.h>

namespace costmap_2d{
	class MultiRobotLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D{
		public:
		  MultiRobotLayer();
		  virtual void onInitialize();
		  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
				                     double* max_y); 
		  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
		  bool isDiscretized();
		  virtual void matchSize();

		private:
		  std::vector <RobotType *> robots;
		  std::string nmspc;
		  bool robotsSet;
		  std::vector<std::vector<geometry_msgs::Point> > polys, polysPrev;
		  bool rolling_window_;
		  void findPolys();
		  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
		  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
	};
}
#endif
