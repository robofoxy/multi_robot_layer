#ifndef ROBOT_TYPE_H_
#define ROBOT_TYPE_H_

#include <multi_robot_layer/multi_robot_layer.h>

class RobotType {
	public:
		RobotType(std::string nm);
		geometry_msgs::PolygonStamped getFootprint() const;
		void footprint_collector(const geometry_msgs::PolygonStamped::ConstPtr&);
		bool isSet() const;
		std::string getName() const;
		
	private:
		std::string name;
		ros::NodeHandle n;
		ros::Subscriber sub;
		ros::Publisher pub;
		geometry_msgs::PolygonStamped footprint;
		bool flag;
		
};



RobotType::RobotType(std::string nm){
	flag = false;
	name = nm;
	footprint.header.frame_id = name;
	std::string subscriber_link = "/" + name + "/move_base_node/global_costmap/footprint";
	sub = n.subscribe(subscriber_link, 1000, &RobotType::footprint_collector, this);
}

void RobotType::footprint_collector(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	int size = msg->polygon.points.size();
	this->footprint.polygon.points.clear();
	for(int i = 0; i < size; i++){
		geometry_msgs::Point32 p;
		p.x = msg-> polygon.points[i].x;
		p.y = msg-> polygon.points[i].y;
		this->footprint.polygon.points.push_back(p);
	}
	this->flag = true;
}

geometry_msgs::PolygonStamped RobotType::getFootprint() const{
	return footprint;
}

bool RobotType::isSet() const{
	return flag;
}

std::string RobotType::getName() const{
	return name;
}

#endif
