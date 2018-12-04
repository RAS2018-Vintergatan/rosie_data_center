#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <sstream>
#include <math.h>
#include <iostream>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <math.h>
#include <stdlib.h>
#include <algorithm>
#include <vector>
#include <iterator>
#include <string>

#include <rosie_object_detector/RAS_Evidence.h>
#include <rosie_servo_controller/ControlGates.h>
#include <rosie_map_controller/RequestMapStoring.h>
#include <rosie_map_controller/RequestObjStoring.h>
#include <rosie_map_controller/RequestLoading.h>
#include <rosie_path_finder/RequestRerun.h>
#include <rosie_path_finder/rrtService.h>
#include <rosie_map_controller/MapStoring.h>
#include <rosie_map_controller/BatteryPosition.h>
#include <rosie_map_controller/ObjectPosition.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define OBJECT 0
#define RED_CYLINDER 1
#define RED_CUBE 2
#define RED_HOLLOW_CUBE 3
#define RED_BALL 4
#define ORANGE_CROSS 5
#define PATRIC 6
#define YELLOW_BALL 7
#define YELLOW_CUBE 8
#define GREEN_CUBE 9
#define GREEN_HOLLOW_CUBE 10
#define GREEN_CYLINDER 11
#define BLUE_CUBE 12
#define BLUE_TRIANGLE 13
#define PURPLE_CROSS 14
#define PURPLE_STAR 15

int red_cylinder_val = 0;
int red_cube_val = 0;
int red_hollow_cube_val = 0;
int red_ball_val = 0;

int orange_cross_val = 0;
int patric_val = 0;

int yellow_ball_val = 0;
int yellow_cube_val = 0;

int green_cube_val = 0;
int green_hollow_cube_val = 0;
int green_cylinder_val = 0;

int blue_cube_val = 0;
int blue_triangle_val = 0;

int purple_cross_val = 0;
int purple_star_val = 0;

float pi = 3.14159265359;
float resolution = 0.2;
int width = 1;
int height = 1;
int numbMarkers = 0;

float robotsize = 0.2;
nav_msgs::OccupancyGrid occGrid;

std::vector<float> home(2,0);


float* pointArray;

std_msgs::String say_this;
ros::Publisher speak_pub;
ros::Publisher objStack_pub;
ros::Subscriber map_sub;
ros::Subscriber battery_sub;
ros::Subscriber evidence_sub;
ros::ServiceClient storeObjClient;
ros::ServiceClient loadClient;
//ros::ServiceClient gateClient;
ros::ServiceClient collisionClient;
ros::ServiceClient rrtClient;

//rosie_map_controller::StartRRT startSrv;
rosie_map_controller::RequestObjStoring objSrv;
rosie_map_controller::RequestLoading loadSrv;
//rosie_servo_controller::ControlGates gateSrv;
rosie_path_finder::RequestRerun collisionSrv;
rosie_path_finder::rrtService rrtSrv;
// Objects and Obstacles (Walls and Batteries)
int objNumber = 14;
float objSize = 0.05;
std::vector<float> objPoseX;
std::vector<float> objPoseY;
int batNumber = 4;
float batSize = 0.1;
std::vector<float> batPoseX;
std::vector<float> batPoseY;

char mapInitialized = 0;
char mapInitializing = 0;
void initializeMap(visualization_msgs::MarkerArray msg){
	if(mapInitializing){
		return;
	}
	mapInitializing = 1;

	ROS_INFO("Initializing!");

	map_sub = ros::Subscriber();
	numbMarkers = msg.markers.size();

	int czone = robotsize/((float)2*resolution) + 0.02/resolution; //additional extra security distance

	std::free(pointArray);
	pointArray = (float*)malloc(sizeof(float)*4*numbMarkers);
	for(int i = 0; i < 4*numbMarkers; ++i){
		pointArray[i] = 0.0f;
	}

	float minX, minY, maxX, maxY;
	minX = minY = maxX = maxY = 0;
	for(int k = 0; k < numbMarkers; ++k){
		//Extract point data
		std::string pointsText = msg.markers[k].text;
		std::stringstream ss(pointsText);
		float sX, sY, eX, eY;
		ss>>sX;
		ss>>sY;
		ss>>eX;
		ss>>eY;

		//Set point data on every 4th index
		pointArray[k<<2]=sX;
		pointArray[(k<<2)+1]=sY;
		pointArray[(k<<2)+2]=eX;
		pointArray[(k<<2)+3]=eY;

		if(sX < minX){
			minX = sX;
		}
		if(sX > maxX){
			maxX = sX;
		}
		if(eX < minX){
			minX = eX;
		}
		if(eX > maxX){
			maxX = eX;
		}
		if(sY < minY){
			minY = sY;
		}
		if(sY > maxY){
			maxY = sY;
		}
		if(eY < minY){
			minY = eY;
		}
		if(eY > maxY){
			maxY = eY;
		}
		ROS_INFO("sX: %f, sY: %f, eX: %f, eY: %f", sX, sY, eX, eY);
	}

	float offsetX = minX-resolution/2;
	float offsetY = minY-resolution/2;

	width = (int) (maxX - minX)/resolution;
	height = (int) (maxY - minY)/resolution;
	float movex = fmod((maxX-minX),resolution)/2;
	float movey = fmod((maxY-minY),resolution)/2;

	ROS_INFO("minX: %f, minY: %f, maxX: %f, maxY: %f, offsetX: %f, offsetY: %f", minX, minY, maxX, maxY, offsetX, offsetY);
	ROS_ERROR("width: %d, height: %d", width, height);

	for(int o = 0; o < width*height; ++o){
		occGrid.data.push_back(0);
	}
	for(int k = 0; k < numbMarkers; ++k){
		float x1 = (pointArray[k<<2]) - movex;
		float y1 = (pointArray[(k<<2)+1]) - movey;
		float x2 = (pointArray[(k<<2)+2]) - movex;
		float y2 = (pointArray[(k<<2)+3]) - movey;

		float diffX = (x2-x1);
		float diffY = (y2-y1);

		float wallDist = sqrt((diffY*diffY)+(diffX*diffX));
		if(wallDist == 0){
			continue;
		}

		for(float d = 0.0; d <= wallDist; d+=resolution){
			int px = (int)((d*((x2-x1)/wallDist)+x1)/resolution);
			int py = (int)((d*((y2-y1)/wallDist)+y1)/resolution);

			occGrid.data[py*width+px] = 125;

			for(int y = -czone; y <= czone; y++){
				for(int x = -czone; x <= czone; x++){
					if((px+x) >= 0 && (px+x) < width && (py+y) >= 0 && (py+y) < height){
						if(occGrid.data[(py+y)*width+px+x] != 125){
							occGrid.data[(py+y)*width+px+x] = 125;
						}
					}
				}
			}
		}
	}
	ROS_INFO("occGrid set!");
	home[0] = 0.2;
	home[1] = 0.4;
	mapInitialized = 1;
	//for(int i = 0; i<width*height; i++){
		//ROS_ERROR("%d",occGrid.data[i]);
	//}
}

int mini(std::vector<float> dist){
	int index = 0;
	float min_dist = dist[0];
	for(int i = 0; i<dist.size(); i++){
		if( dist[i]<min_dist){
			min_dist = dist[i];
			index = i;
		}
	}
	return index;
}
int mini2(std::vector<int> dist){
	int index = 0;
	int min_dist = dist[0];
	for(int i = 0; i<dist.size(); i++){
		if( dist[i]<min_dist){
			min_dist = dist[i];
			index = i;
		}
	}
	return index;
}

std::vector<std::vector<float> > finalpath;
std::vector<std::vector<float> > poses;
std::vector<float> p(2,0);
void findPath(){
	poses.clear();
	for(int y = 0; y<height; y++){
		for(int x = 0; x<width; x++){
			if(occGrid.data[y*width+x]==0){
				p[0] = (x+1)*resolution;
				p[1] = (y+1)*resolution;
				poses.push_back(p);
			}else{
				p[0] = (x+1)*resolution;
				p[1] = (y+1)*resolution;
				//ROS_ERROR("%f %f", p[0], p[1]);
			}
		}
	}




	int cellsInY = height;
	//int cellsIntervalY = 1;
	int cellsInX = width;
	//int cellsIntervalX  = 1;

	int hidden_nodes = cellsInY*cellsInX;
	int max_epochs = 2000;
	float eta = 0.2;
	int neigh_size = cellsInY/2;

	//initialize w 
	std::vector<std::vector<float> > w;
	std::vector<float> inner_w;
	std::vector<float> dist;
	for(int j = 0; j<hidden_nodes; j++){
		inner_w.clear();
		for(int i = 0; i<2; i++){
			std::srand(unsigned ( std::time(0) ));
			inner_w.push_back(((float)(std::rand()%100))/100.0*((float)cellsInX));
			inner_w.push_back(((float)(std::rand()%100))/100.0*((float)cellsInY));
		}
		w.push_back(inner_w);
	}

	//SOM
	ROS_ERROR("C1");
	int index;
	int bound1;
	int bound2;
	std::vector<int> indA;
	std::vector<int> indB;
	std::vector<int> uniqueV;
	for(int i = 0; i<=max_epochs; i++){
		neigh_size = floor(sqrt(max_epochs-i))+1 ;
		for(int j =0; j<poses.size(); j++){
			dist.clear();
			for(int k = 0; k<w.size(); k++){
				dist.push_back(std::sqrt(pow(w[k][0]-poses[j][0],2)+pow(w[k][1]-poses[j][1],2)));
			}
			index = mini(dist);
			bound1 = (index - neigh_size/2);
			bound2 = (index + neigh_size/2);
			indA.clear();
			indB.clear();
			if(bound1 < 1){
				bound1 = hidden_nodes + bound1-1;
				for(int b1 =bound1; b1 < hidden_nodes; b1++){
					indA.push_back(b1);
				}
			}else{
				for(int b1 =bound1; b1 < index; b1++){
					indA.push_back(b1);
				}
			}
			if(bound2 > hidden_nodes){
				bound2 = bound2 - hidden_nodes;
				for(int b2 = 0; b2 < bound2 ; b2++){
					indB.push_back(b2);
				}
			}else{
				for(int b2 = index; b2 < bound2; b2++){
					indB.push_back(b2);
				}
			}
			
			uniqueV.clear();
			uniqueV.insert(uniqueV.end(), indA.begin(), indA.end());
			uniqueV.insert(uniqueV.end(), indB.begin(), indB.end());		
			uniqueV.push_back(index);
			std::sort(uniqueV.begin(), uniqueV.end());
			std::vector<int>::iterator last = std::unique(uniqueV.begin(), uniqueV.end());
			//ROS_ERROR("%d",last[0]);
			uniqueV.erase(last, uniqueV.end());
			for(int u = 0; u<uniqueV.size(); u++){
				w[uniqueV[u]][0] = w[uniqueV[u]][0] + eta*(poses[j][0]-w[uniqueV[u]][0]);
				w[uniqueV[u]][1] = w[uniqueV[u]][1] + eta*(poses[j][1]-w[uniqueV[u]][1]);
			}
		}
	}
	std::vector<float> finalX;
	std::vector<float> finalY;
	finalpath.clear();
	std::vector<std::vector<float> > w_temp;
	std::vector<int> indices;
	int tempIdx;
	w_temp.insert(w_temp.end(), w.begin(), w.end());
	//ROS_ERROR("size %d",poses.size());
	for(int i= 0; i<poses.size(); i++){
		dist.clear();
		for(int k = 0; k<w_temp.size(); k++){
			dist.push_back(std::sqrt(std::pow(w_temp[k][0]-poses[i][0],2)+std::pow(w_temp[k][1]-poses[i][1],2)));
			//ROS_ERROR("dist %f",dist[k]);
		}
		tempIdx = mini(dist);
		indices.push_back(tempIdx);
		finalpath.push_back(w_temp[tempIdx]);
		ROS_ERROR("path %f %f",finalpath[i][0],finalpath[i][1]);
		w_temp[tempIdx][0] += 1000.0;
		w_temp[tempIdx][1] += 1000.0;

	}
	finalpath.push_back(home);
	/*int idx;
	finalpath.clear();
	for(int i = 0; i<indices.size(); i++){
		//ROS_ERROR("%d",indices[i]);
		//finalX.push_back(poses[indices[i]][0]);
		//finalY.push_back(poses[indices[i]][1]);
		idx = indices[i];
		finalpath.push_back(w_temp[idx]);
		//ROS_ERROR("C3");
		//indices[ind] = 1000;
		ROS_ERROR("path %f %f",finalpath[i][0],finalpath[i][1]);
	}*/
	//finalX.push_back(finalX[0]);
	//finalY.push_back(finalY[0]);
	//ROS_ERROR("path %f %f",finalX[finalX.size()-1],finalY[finalY.size()-1]);
}


// **************************
float objTemp1[4];
float objTemp2[4];
bool objInitialized = 0;
float PI = 3.1415926f;

bool checkedStoredObjects = 0;
rosie_map_controller::ObjectStoring objStack;
//std::vector<rosie_map_controller::BatteryPosition> batStack;
ros::Time lastObjStoring;
std::vector<ros::Time> lastBatObservation;

void batteryCallback(visualization_msgs::Marker msg){
	if(!checkedStoredObjects){
		return;
	}
	float posX = msg.pose.position.x;
	float posY = msg.pose.position.y;
	bool standing = msg.id; // 0 laying, 1 standing
	float accuracy = 0.10;
	if(!standing){
		accuracy = 0.15;
	}
	bool pushed = 0;
	for(int i = 0; i< objStack.Batteries.size(); ++i){
	    if(pow(posX-objStack.Batteries[i].x,2)+pow(posY-objStack.Batteries[i].y,2) < (accuracy*accuracy)){
	      ROS_INFO("Battery is already mapped. - Updated battery position");
		objStack.Batteries[i].x = posX;
		objStack.Batteries[i].y = posY;
		if(ros::Time::now().toSec() - lastBatObservation[i].toSec() > 2*objStack.Batteries[i].certainty)
			// when we detect an object again (after a given time interval) the certainty gets bigger
			// instead of forgetting an object the path planner will not take batteries into account that we are not certain about(threshold);
		objStack.Batteries[i].certainty++;
		lastBatObservation[i] = ros::Time::now();
		pushed = 1;
		continue;
	    	}else{
				//nothing yet. No explicite forgetting.

		}
	}
	if(!pushed){
		ROS_INFO("NEW battery");
		say_this.data = "' New Battery '";
		speak_pub.publish(say_this);
		rosie_map_controller::BatteryPosition battery;
		battery.x = posX;
		battery.y = posY;
		battery.certainty = 0;
		battery.standing = standing;
		objStack.Batteries.push_back(battery);
		lastBatObservation.push_back(ros::Time::now());
	}	
}

void evidenceCallback(const rosie_object_detector::RAS_Evidence evidence){
	if(!checkedStoredObjects){
		loadSrv.request.request = 1;
		if(loadClient.call(loadSrv)){
			objStack = loadSrv.response.objects;
			std::vector<ros::Time> initBatObs (objStack.Batteries.size(), ros::Time::now());
			lastBatObservation.insert(lastBatObservation.end(), initBatObs.begin(), initBatObs.end());
//			object.id = id;
//			object.x = x;
//			object.y = y;
//			object.value = value;
//			object.name = name;
			checkedStoredObjects = 1;
		}

	}else{
		std::string obj_string_id = evidence.object_id;
		int obj_id = 0;
		int obj_val = 0;
		float obj_size = objSize;
		/*if(obj_string_id.compare(evidence.red_cylinder)){
			obj_id = RED_CYLINDER;
			obj_val = red_cylinder_val;
		}*/
		if(obj_string_id.compare(evidence.red_cube)){
			obj_id = RED_CUBE;
			obj_val = red_cube_val;
		}
		if(obj_string_id.compare(evidence.red_hollow_cube)){
			obj_id = RED_HOLLOW_CUBE;
			obj_val = red_hollow_cube_val;
		}
		if(obj_string_id.compare(evidence.red_ball)){
			obj_id = RED_BALL;
			obj_val = red_ball_val;
		}
		/*if(obj_string_id.compare(evidence.orange_cross)){
			obj_id = ORANGE_CROSS;
			obj_val = orange_cross_val;
		}*/
		if(obj_string_id.compare(evidence.patric)){
			obj_id = PATRIC;
			obj_val = patric_val;
		}
		if(obj_string_id.compare(evidence.yellow_ball)){
			obj_id = YELLOW_BALL;
			obj_val = yellow_ball_val;
		}
		if(obj_string_id.compare(evidence.yellow_cube)){
			obj_id = YELLOW_CUBE;
			obj_val = yellow_cube_val;
		}
		if(obj_string_id.compare(evidence.green_cube)){
			obj_id = GREEN_CUBE;
			obj_val = green_cube_val;
		}
		/*if(obj_string_id.compare(evidence.green_hollow_cube)){
			obj_id = GREEN_HOLLOW_CUBE;
			obj_val = green_hollow_cube_val;
		}*/
		if(obj_string_id.compare(evidence.green_cylinder)){
			obj_id = GREEN_CYLINDER;
			obj_val = green_cylinder_val;
		}
		if(obj_string_id.compare(evidence.blue_cube)){
			obj_id = BLUE_CUBE;
			obj_val = blue_cube_val;
		}
		if(obj_string_id.compare(evidence.blue_triangle)){
			obj_id = BLUE_TRIANGLE;
			obj_val = blue_triangle_val;
		}
		if(obj_string_id.compare(evidence.purple_cross)){
			obj_id = PURPLE_CROSS;
			obj_val = purple_cross_val;
		}
		if(obj_string_id.compare(evidence.purple_star)){
			obj_id = PURPLE_STAR;
			obj_val = purple_star_val;
		}

		float posX = evidence.object_location.x;
		float posY = evidence.object_location.y;

		if(obj_id != OBJECT){
			rosie_map_controller::ObjectPosition object;
			float accuracy = 0.05; //5cm radius for faulty measurement
			bool idAlreadyListed = 0;
			int listedIndex = -1;
			for(int i =0; i<objStack.Objects.size(); i++){
				if(objStack.Objects[i].id = obj_id){
					idAlreadyListed = 1;
					listedIndex = i;
				}
			}
			if(!idAlreadyListed){ //true if id is not present
				object.id =obj_id;
				object.x = posX;
				object.y = posY;
				object.value = obj_val; //weighting
				object.name = obj_string_id;
				objStack.Objects.push_back(object);
				say_this.data = object.name;
				speak_pub.publish(say_this);
			}else{
				if(pow(posX-objStack.Objects[listedIndex].x,2)+pow(posY-objStack.Objects[listedIndex].y,2) > (accuracy*accuracy)){
					objStack.Objects[listedIndex].x = posX;
					objStack.Objects[listedIndex].y = posY;
					//evtl remove obj and push again -> generate new walls/lines
					ROS_INFO("This object has been moved.");

					say_this.data = "' This object has been moved. '";
					speak_pub.publish(say_this);
				}else{
					ROS_INFO("This object is already mapped.");
					say_this.data = "' This object is already mapped. '";
					speak_pub.publish(say_this);
				}
			}

		}
		if(ros::Time::now().toSec()-lastObjStoring.toSec() > 1){
			rosie_map_controller::ObjectStoring send;
			for(int i=0; i<objStack.Objects.size();i++){
				send.Objects.push_back(objStack.Objects[i]);
			}
			for(int i=0; i<objStack.Batteries.size();i++){
				send.Batteries.push_back(objStack.Batteries[i]);
			}
			objSrv.request.send = send;
			if(storeObjClient.call(objSrv)){
				lastObjStoring = ros::Time::now();
			}
		}
		objStack_pub.publish(objStack);
	}
}
nav_msgs::Odometry pose;

void currentPoseCallback(nav_msgs::Odometry msg){ // for re-calculation of the path when needed
    pose = msg;
		//pose.pose.pose.position.x = 0.25f;
		//pose.pose.pose.position.y = 0.40f;
}
/*
void actuateGripper(bool command){
	gateSrv.request.control = command;
	gateClient.call(gateSrv);
	//	ROS_INFO("");
	if(gateSrv.response.result == 1){
		if(command == 0){
			ROS_INFO("Gripper closed");
		} else if(command == 1){
			ROS_INFO("Gripper opened");
		}
	}else{
		ROS_INFO("Gripper don't react. Please have a look.");
	}
}
*/
int startInitialized = 0;
float lastGoalX;
float lastGoalY;
int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_explorer");

		//target_tfl_ptr.reset(new tf::TransformListener);

    ros::NodeHandle n;
		speak_pub = n.advertise<std_msgs::String>("/espeak/string",1);
	  	evidence_sub = n.subscribe<rosie_object_detector::RAS_Evidence>("/evidence",10, evidenceCallback);
		//ros::Subscriber rviz_goal = n.subscribe<geometry_msgs::PoseStamped>("/rviz_object_pose",10,rvizTargetPoseCallback);
		storeObjClient = n.serviceClient<rosie_map_controller::RequestObjStoring>("request_store_objects");
		loadClient = n.serviceClient<rosie_map_controller::RequestLoading>("request_load_mapping");
		//gateClient = n.serviceClient<rosie_servo_controller::ControlGates>("control_gates");
		collisionClient = n.serviceClient<rosie_path_finder::RequestRerun>("request_rerun");
		rrtClient = n.serviceClient<rosie_path_finder::rrtService>("/rrt");
		//rosie_map_controller::StartRRT startSrv;
		map_sub = n.subscribe<visualization_msgs::MarkerArray>("/maze_map",10,initializeMap);
		objStack_pub = n.advertise<rosie_map_controller::ObjectStoring>("/object_stack",10);
		battery_sub = n.subscribe<visualization_msgs::Marker>("/visualization_marker_battery", 1, batteryCallback);

    n.getParam("red_cylinder", red_cylinder_val);
    n.getParam("red_cube", red_cube_val);
    n.getParam("red_hollow_cube", red_hollow_cube_val);
    n.getParam("red_ball", red_ball_val);
    n.getParam("orange_cross", orange_cross_val);
    n.getParam("patric", patric_val);
    n.getParam("yellow_ball", yellow_ball_val);
    n.getParam("yellow_cube", yellow_cube_val);
    n.getParam("green_cube", green_cube_val);
    n.getParam("green_hollow_cube", green_hollow_cube_val);
    n.getParam("green_cylinder", green_cylinder_val);
    n.getParam("blue_cube", blue_cube_val);
    n.getParam("blue_triangle", blue_triangle_val);
    n.getParam("purple_cross", purple_cross_val);
    n.getParam("purple_star", purple_star_val);



		static tf::TransformBroadcaster br;

    ros::Rate loop_rate(10);
	ros::Time load_time = ros::Time::now();
	bool collisionDetected = 0;
	bool pathInitialized= 0;
	bool nextStepInit = 0;
	bool pathSend = 0;
	int poscnt = 0;
    while(ros::ok()){
//***********************************
//STATE-MACHINE
//***********************************
		//ROS_ERROR("collisionDetected %d, poscnt %d, pathInitialized: %d, pathSend: %d", collisionDetected, poscnt, pathInitialized, pathSend);
		if(mapInitialized){
			if(!pathInitialized){
				ROS_ERROR("Find Tree");
				findPath();
				pathInitialized = 1;
			}else{
				//ROS_ERROR("-- finalpath element %f %f", finalpath[poscnt][0], finalpath[poscnt][1]);

				collisionSrv.request.question = 1;
				if(collisionClient.call(collisionSrv)){
					if(collisionSrv.response.answer){
						//collisionDetected = 1;
					}
				}

				if((collisionDetected || !pathSend) && poscnt < finalpath.size()){
					collisionDetected = 0;
					rrtSrv.request.goalx = finalpath[poscnt][0];
					rrtSrv.request.goaly = finalpath[poscnt][1];
					rrtSrv.request.mode = 0; //go to target
					if(rrtClient.call(rrtSrv)){
						//finalpath[poscnt][0] = rrtSrv.response.goalx;
						//finalpath[poscnt][1] = rrtSrv.response.goaly;
						ROS_ERROR("finalpath element %f %f", finalpath[poscnt][0], finalpath[poscnt][1]);
					}
					pathSend = 1;
				}

				if((pow(pose.pose.pose.position.x-finalpath[poscnt][0],2)+pow(pose.pose.pose.position.y-finalpath[poscnt][1],2)) < 0.2*0.2 && poscnt < finalpath.size()-1){
					poscnt++;
					pathSend = 0;
				}
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}
