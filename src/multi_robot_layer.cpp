#include <multi_robot_layer/multi_robot_layer.h>
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(costmap_2d::MultiRobotLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;
using costmap_2d::NO_INFORMATION;

namespace costmap_2d{
	MultiRobotLayer::MultiRobotLayer() {}

	void MultiRobotLayer::onInitialize(){
	
		ros::NodeHandle nh("~/" + name_);
		
		nmspc = nh.getNamespace();
		std::vector<std::string> lv_elems;
		char lc_delim[2];
		lc_delim[0] = '/';
		lc_delim[1] = '\0';
		
		boost::algorithm::split( lv_elems, nh.getNamespace(), boost::algorithm::is_any_of( lc_delim ) );
		nmspc = lv_elems[1];
		
		robotsSet = false;
		
		rolling_window_ = layered_costmap_->isRolling();
		current_ = true;
		default_value_ = NO_INFORMATION;
		matchSize();
		dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
		dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
									&MultiRobotLayer::reconfigureCB, this, _1, _2);
		dsrv_->setCallback(cb);
	}



	void MultiRobotLayer::matchSize(){
	
		Costmap2D* master = layered_costmap_->getCostmap();
		resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
					master->getOriginX(), master->getOriginY());
	}
	


	void MultiRobotLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level){
	
		enabled_ = config.enabled;
	}
	
	

	void MultiRobotLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
		                                   double* min_y, double* max_x, double* max_y){
		if (rolling_window_){
			
			updateOrigin(robot_x - getSizeInMetersX() / 2 , robot_y - getSizeInMetersY() / 2);
		}

		if (!enabled_) return;

		double mark_x = robot_x + cos(robot_yaw), mark_y = robot_y + sin(robot_yaw);
		
		*min_x = std::min(*min_x, mark_x);
		*min_y = std::min(*min_y, mark_y);
		*max_x = std::max(*max_x, mark_x);
		*max_y = std::max(*max_y, mark_y);
	}
	
	
	
	bool MultiRobotLayer::isDiscretized(){
	
		return true;
	}
	
	
	
	void MultiRobotLayer::findPolys(){
	
		if(!robotsSet) return;
			
		int size = robots.size();
		
		for(int i = 0; i < size; i++){
			int numOfPoints = robots[i]->getFootprint().polygon.points.size();
			std::vector<geometry_msgs::Point> poly;
			for(int j = 0; j < numOfPoints; j++){
				geometry_msgs::Point p;
				p.x = robots[i]->getFootprint().polygon.points[j].x;
				p.y = robots[i]->getFootprint().polygon.points[j].y;
				poly.push_back(p);
			}
			
			polys.push_back(poly);
		}
	}
	
	

	void MultiRobotLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
		                                      int max_j){
		if (!enabled_) return;
		
		if(!robotsSet && robots.size()==0){
		
			robotsSet = true;
			std::vector<std::string> lv_elems;
			char lc_delim[2];
			lc_delim[0] = '/';
			lc_delim[1] = '\0';
			
			ros::master::V_TopicInfo topic_infos;
			ros::master::getTopics(topic_infos);
			
			for(int k = 0; k< topic_infos.size(); k++){
			
				boost::algorithm::split( lv_elems, topic_infos[k].name, boost::algorithm::is_any_of( lc_delim ) );

				if(lv_elems[1] != nmspc && lv_elems.size() == 5 && lv_elems[3]=="global_costmap" && lv_elems[4]=="footprint"){
					RobotType *robot = new RobotType(lv_elems[1]);
					robots.push_back(robot);
				}

			}
		}
		
		findPolys();
		
		int numOfPrevRobots = polysPrev.size();
		
		for(int i = 0; i < numOfPrevRobots; i++)
			master_grid.setConvexPolygonCost(polysPrev[i], FREE_SPACE);
			
		polysPrev = polys;
		
		int numOfOtherRobots = polys.size();
		
		for(int i = 0; i <numOfOtherRobots; i++)
			master_grid.setConvexPolygonCost(polys[i], LETHAL_OBSTACLE);

	}
} // end namespace
