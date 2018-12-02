#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <sstream>
#include <math.h>
#include <iostream>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

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
float resolution = 0.4;
int width = 1;
int height = 1;
int numbMarkers = 0;

float robotsize = 0.2;
nav_msgs::OccupancyGrid occGrid;

float* pointArray;

static ros::Subscriber map_sub;

char mapInitialized = 0;
char mapInitializing = 0;
void initializeMap(const visualization_msgs::MarkerArray msg){
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

	float offsetX = minX;
	float offsetY = minY;

	width = (int) (maxX - minX)/resolution;
	height = (int) (maxY - minY)/resolution;

	ROS_INFO("minX: %f, minY: %f, maxX: %f, maxY: %f, offsetX: %f, offsetY: %f", minX, minY, maxX, maxY, offsetX, offsetY);
	ROS_INFO("width: %d, height: %d", width, height);

	for(int o = 0; o < width*height; ++o){
		occGrid.data.push_back(0);
	}
	for(int k = 0; k < numbMarkers; ++k){
		float x1 = (pointArray[k<<2]) - offsetX;
		float y1 = (pointArray[(k<<2)+1]) - offsetY;
		float x2 = (pointArray[(k<<2)+2]) - offsetX;
		float y2 = (pointArray[(k<<2)+3]) - offsetY;

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
	mapInitialized = 1;
}


std::vector<std::vector<int> > edges; //vertex
bool notInVertexList(std::vector<int> e){
	bool ne = 0;
	for(int i = 0; i<edges.size(); i++){
		if( (edges[i][0] == e[0] and edges[i][1] == e[1] and edges[i][2] == e[2] and edges[i][3] == e[3]) or (edges[i][0] == e[2] and edges[i][1] == e[3] and edges[i][2] == e[0] and edges[i][3] == e[1]) ){
			ne = 1;
			break;
		}
	}
	return ne;
}
int nextIdxVertexList(std::vector<int> e){
	int idx = -1;
	for(int i = 0; i<edges.size(); i++){
		if( (edges[i][0] == e[0] and edges[i][1] == e[1]) or (edges[i][2] == e[0] and edges[i][3] == e[1]) ){
			if(edges[i][0] != -1 and edges[i][1] != -1 and edges[i][2] != -1 and edges[i][3] != -1){
				idx = i;
				break;
			}
		}
	}
	return idx;
}

std::vector<std::vector<int> > nodepoint;
bool isNodePoint(std::vector<int> n){
	bool nn = 0;
	for(int i = 0; i<nodepoint.size(); i++){
		if( (nodepoint[i][0] == n[0] and nodepoint[i][1] == n[1] ) or (nodepoint[i][2] == n[0] and nodepoint[i][3] == n[1]) ){
			nn = 1;
			break;
		}
	}
	return nn;
}


std::vector<int> edge(4,0);
std::vector<int> ep(2,0);
std::vector<std::vector<int> > endpoint;
std::vector<int> sp(2,0);
std::vector<std::vector<int> > solopoint;
std::vector<int> np(2,0);
std::vector<std::vector<std::vector<int> > > allstab;
std::vector<std::vector<std::vector<int> > > endstab;
std::vector<std::vector<std::vector<int> > > combistab;
std::vector<std::vector<int> > cleanpath;
std::vector<std::vector<int> > cleanpath_f;
std::vector<std::vector<int> > stab;
int cnt_edges;
void findTree(){
	edges.clear();
	for(int i = 0; i<height; i++){ //y direction
		for(int j = 0; j<width; j++){
			edge[0] = j;
			edge[1] = i;
			cnt_edges = 0;
			if((j) > 0 && (j) < width-1 && (i) > 0 && (1) < height-1){
				if(occGrid.data[(j-1)*width+i] == 125){
					edge[2] = j-1;
					edge[3] = i;
					if(nextIdxVertexList(edge)<0){ //not in list --> 1, append
						cnt_edges++;
						edges.push_back(edge);
					}
				}
				if(occGrid.data[(j+1)*width+i] == 125){
					edge[2] = j+1;
					edge[3] = i;
					cnt_edges++;
					if(nextIdxVertexList(edge)<0){ //not in list --> 1, append
						cnt_edges++;
						edges.push_back(edge);
					}
				}
				if(occGrid.data[(j)*width+i-1] == 125){
					edge[2] = j;
					edge[3] = i-1;
					cnt_edges++;	
					if(nextIdxVertexList(edge)<0){ //not in list --> 1, append
						cnt_edges++;
						edges.push_back(edge);
					}			
				}
				if(occGrid.data[(j)*width+i+1] == 125){
					edge[2] = j;
					edge[3] = i+1;
					cnt_edges++;	
					if(nextIdxVertexList(edge)<0){ //not in list --> 1, append
						cnt_edges++;
						edges.push_back(edge);
					}			
				}
				if(cnt_edges == 0){
					sp[0] = j;
					sp[1] = i;
					solopoint.push_back(sp);
				}
				if(cnt_edges == 1){
					ep[0] = j;
					ep[1] = i;
					endpoint.push_back(ep);
				}
				if(cnt_edges > 1){
					np[0] = j;
					np[1] = i;
					nodepoint.push_back(np);
				}
			}
		}
	}
	std::vector<int> point(2,0);
	int nextIdx = -1;
	for(int i = 0; i<endpoint.size(); i++){
		stab.clear();
		point[0] = endpoint[i][0];
		point[1] = endpoint[i][1];
		stab.push_back(point);
		nextIdx = 0;
		while(!isNodePoint(point) and i!=0){
			nextIdx = nextIdxVertexList(point);
			if(nextIdx<0){
			break;
			}
			point[0] = edges[nextIdx][0];
			point[1] = edges[nextIdx][1];
			edges[nextIdx][0]= -1;
			edges[nextIdx][0]= -1;
			stab.push_back(point);	
		}
		if(nextIdx>= 0){
			allstab.push_back(stab);
		}
	}
	for(int i = 0; i<nodepoint.size(); i++){
		stab.clear();
		point[0] = nodepoint[i][0];
		point[1] = nodepoint[i][1];
		stab.push_back(point);
		nextIdx = 0;
		while(!isNodePoint(point) and i!=0){
			nextIdx = nextIdxVertexList(point);
			if(nextIdx<0){
			break;
			}
			point[0] = edges[nextIdx][0];
			point[1] = edges[nextIdx][1];
			edges[nextIdx][0]= -1;
			edges[nextIdx][0]= -1;
			stab.push_back(point);	
		}
		if(nextIdx>= 0){
			endstab.push_back(stab);
		}
	}
	std::vector<int> sbegin(2,0);
	std::vector<int> send(2,0);
	std::vector<std::vector<int> > empty;
	std::vector<int> emp(2,0);
	empty.push_back(emp);

	for(int i = 0; i<endpoint.size(); i++){
		int idxStab = -1;
		int be = -1;
		for(int j = 0; j<allstab.size(); j++){
			sbegin[0] = allstab[i][0][0];
			sbegin[1] = allstab[i][0][1];
			send[0] = allstab[i][allstab[i].size()-1][0];
			send[0] = allstab[i][allstab[i].size()-1][0];
			if(sbegin[0] == endpoint[i][0] and sbegin[1] == endpoint[i][1]){
				idxStab = j;
				std::reverse(allstab[i].begin(),allstab[i].end()); //turn around, because i use insert in later preocess
				combistab.push_back(allstab[i]);
				allstab[i].clear();
				break;
			}else if(send[0] == endpoint[i][0] and send[1] == endpoint[i][1]){
				idxStab = j;
				combistab.push_back(allstab[i]);
				allstab[i].clear();
				break;
			}
		}
	}
	std::vector<int> sendold(2,0);
	std::vector<int> idxToDelete;
	for(int i = 0; i<combistab.size(); i++){
		sendold[0] = combistab[i][combistab[i].size()-1][0]; 
		sendold[1] = combistab[i][combistab[i].size()-1][1]; 
		while(sendold[0] != 1 and sendold[1] != 1){
			std::vector<int> idxlist;
			std::vector<int>::iterator it;
			for(int j = 0; j<allstab.size(); j++){
				//it.clear();
				it = std::find(idxlist.begin(), idxlist.end(), (int) j);
				if(it == idxlist.end() && j!=0){
					continue;
				}				
				int be = -1;
				sbegin[0] = allstab[j][0][0];
				sbegin[1] = allstab[j][0][1];
				send[0] = allstab[j][allstab[i].size()-1][0];
				send[1] = allstab[j][allstab[i].size()-1][1];
				if(sbegin[0] == sendold[0] and sbegin[1]== sendold[1]){
					std::reverse(allstab[j].begin(), allstab[j].end());
					combistab[i].insert(combistab[i].begin(), allstab[j].begin(), allstab[j].end());
				}else if(send[0] == sendold[0] and send[1]== sendold[1]){
					combistab[i].insert(combistab[i].begin(), allstab[j].begin(), allstab[j].end());
				}
				idxlist.push_back((int) j);
			}		
			sendold[0] = combistab[i][combistab[i].size()-1][0]; 
			sendold[1] = combistab[i][combistab[i].size()-1][1]; 
		}
	}

	std::vector<int> length;
	for(int i = 0; i<combistab.size(); i++){
		length.push_back((int) combistab[i].size());
	}
	std::vector<std::vector<int> > dirtypath;
	std::vector<int>::iterator result;
	while(length.size()>0){
		result = std::max_element(length.begin(), length.end());
		dirtypath.insert(dirtypath.begin(), combistab[result[0]].begin(), combistab[result[0]].end());
		length.erase(length.begin() + result[0]);
	}

	int cnt = 0;
	std::vector<std::vector<int> > temp;
	for(int i =0; i<dirtypath.size(); i++){
		cnt = 0;
		sbegin[0] = dirtypath[i][0];
		sbegin[1] = dirtypath[i][1];
		for(int j =0; j<cleanpath.size(); j++){
			if(cleanpath[j][0] == sbegin[0] and cleanpath[j][1] == sbegin[1]){
				cnt++;
			}
		}
		if(cnt == 0){
			temp.clear();
			temp.push_back(sbegin);
			cleanpath.insert(cleanpath.begin(), temp.begin(), temp.end());
		}
	}
	std::vector<float> sbegin_f(2,0);
	//sbegin[0] = dirtypath[0][0];
	//sbegin[1] = dirtypath[0][1];
	for(int i = 0; i < solopoint.size(); i++){
		temp.clear();
		temp.push_back(solopoint[i]);
		cleanpath.insert(cleanpath.begin(), temp.begin(), temp.end());
	}
	cleanpath_f.resize(cleanpath.size());
	for(int i = 0; i < cleanpath.size(); i++){
		cleanpath_f[i][0] = cleanpath[i][0]*resolution + 0.5*resolution;
		cleanpath_f[i][1] = cleanpath[i][1]*resolution + 0.5*resolution ;
	}

}


ros::Subscriber evidence_sub;
ros::ServiceClient storeObjClient;
ros::ServiceClient loadClient;
ros::ServiceClient gateClient;
ros::ServiceClient collisionClient;
ros::ServiceClient rrtClient;

//rosie_map_controller::StartRRT startSrv;
rosie_map_controller::RequestObjStoring objSrv;
rosie_map_controller::RequestLoading loadSrv;
rosie_servo_controller::ControlGates gateSrv;
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

// **************************
float objTemp1[4];
float objTemp2[4];
bool objInitialized = 0;
float PI = 3.1415926f;


bool checkedStoredObjects = 0;
rosie_map_controller::ObjectStoring objStack;
//std::vector<rosie_map_controller::BatteryPosition> batStack;
ros::Time lastObjStoring = ros::Time::now();
std::vector<ros::Time> lastBatObservation;
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
		else if(obj_string_id.compare(evidence.an_object)){
			obj_id = OBJECT;
			obj_val = 0;
			obj_size = batSize;
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
			}else{
				if(pow(posX-objStack.Objects[listedIndex].x,2)+pow(posY-objStack.Objects[listedIndex].y,2) > (accuracy*accuracy)){
					objStack.Objects[listedIndex].x = posX;
					objStack.Objects[listedIndex].y = posY;
					//evtl remove obj and push again -> generate new walls/lines
					ROS_INFO("This object has been moved.");
				}else{
					ROS_INFO("This object is already mapped.");
				}
			}

		}else{
			float posX = evidence.object_location.x;
			float posY = evidence.object_location.y;
			float accuracy = 0.05; //5cm radius for faulty measurement
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

				rosie_map_controller::BatteryPosition battery;
				battery.x = posX;
				battery.y = posY;
				battery.certainty = 0;
				objStack.Batteries.push_back(battery);
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
	}
}

nav_msgs::Odometry pose;

void currentPoseCallback(nav_msgs::Odometry msg){ // for re-calculation of the path when needed
    pose = msg;
		//pose.pose.pose.position.x = 0.25f;
		//pose.pose.pose.position.y = 0.40f;
}

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

int startInitialized = 0;
float lastGoalX;
float lastGoalY;
int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_explorer");

		//target_tfl_ptr.reset(new tf::TransformListener);

    ros::NodeHandle n;
	  evidence_sub = n.subscribe<rosie_object_detector::RAS_Evidence>("/evidence",10, evidenceCallback);
		//ros::Subscriber rviz_goal = n.subscribe<geometry_msgs::PoseStamped>("/rviz_object_pose",10,rvizTargetPoseCallback);
		storeObjClient = n.serviceClient<rosie_map_controller::RequestObjStoring>("request_store_objects");
		loadClient = n.serviceClient<rosie_map_controller::RequestLoading>("request_load_mapping");
		gateClient = n.serviceClient<rosie_servo_controller::ControlGates>("control_gates");
		collisionClient = n.serviceClient<rosie_path_finder::RequestRerun>("request_rerun");
		rrtClient = n.serviceClient<rosie_path_finder::rrtService>("/rrt");
		//rosie_map_controller::StartRRT startSrv;

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
		if(mapInitialized){
			if(!pathInitialized){
				findTree();
			}else{

				collisionSrv.request.question = 1;
				if(collisionClient.call(collisionSrv)){
					if(collisionSrv.response.answer){
						collisionDetected = 1;
					}
				}
	
				if(collisionDetected || !pathSend){
					rrtSrv.request.goalx = cleanpath_f[poscnt][0];
					rrtSrv.request.goaly = cleanpath_f[poscnt][1];
					rrtSrv.request.mode = 3; //go to target
					if(rrtClient.call(rrtSrv)){
						cleanpath_f[poscnt][0] = rrtSrv.response.goalx;
						cleanpath_f[poscnt][1] = rrtSrv.response.goaly;
					}
					pathSend = 1;
				}
				
				if((pow(pose.pose.pose.position.x-cleanpath[poscnt][0],2)+pow(pose.pose.pose.position.y-cleanpath[poscnt][1],2)) < 0.2*0.2){
					poscnt++;
					pathSend = 0;
				}
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}

