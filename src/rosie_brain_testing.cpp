#include <ros/ros.h>
#include <ros/time.h>

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

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
#include <rosie_map_controller/ObjectStoring.h>
#include <rosie_map_controller/BatteryPosition.h>
#include <rosie_map_controller/ObjectPosition.h>

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

// Target pose
boost::shared_ptr<geometry_msgs::PoseStamped> targetPose_ptr;
boost::shared_ptr<geometry_msgs::PoseStamped> lastTargetPose_ptr;

ros::Subscriber evidence_sub;
ros::Subscriber battery_sub;
ros::Publisher objStack_pub;
ros::Publisher speak_pub;

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

// Espeak
std_msgs::String say_this;


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

bool gotNewTarget = 0;
float tarPoseX=0;
float tarPoseY=0;
//geometry_msgs::PoseStamped rviz_pose;
void rvizTargetPoseCallback(const geometry_msgs::PoseStamped& pose){
	targetPose_ptr->pose.position.x = pose.pose.position.x;
	targetPose_ptr->pose.position.y = pose.pose.position.y;
	targetPose_ptr->header.seq = pose.header.seq;
	targetPose_ptr->header.frame_id = pose.header.frame_id;
	targetPose_ptr->header.stamp = pose.header.stamp;

	//if((tarPoseX != msg.pose.position.x) || (tarPoseY != msg.pose.position.y)){
	//	tarPoseX = msg.pose.position.x;
	//	tarPoseY = msg.pose.position.y;
		ROS_ERROR("target x: %f y: %f", targetPose_ptr->pose.position.x, targetPose_ptr->pose.position.y);
	//	gotNewTarget = 1;
	//}
}

nav_msgs::Odometry pose;

void currentPoseCallback(nav_msgs::Odometry msg){ // for re-calculation of the path when needed
    pose = msg;
		//pose.pose.pose.position.x = 0.25f;
		//pose.pose.pose.position.y = 0.40f;
}

void actuateGripper(bool command){
	gateSrv.request.control = command;
	if(!gateClient.call(gateSrv)){
		ROS_ERROR("**********************************");
		ROS_ERROR("Gate callback worked not");
		ROS_ERROR("**********************************");
	}
	//	ROS_INFO("");
	if(gateSrv.response.result == 1){
		if(command == 0){
			ROS_INFO("Gripper closed");
			say_this.data = "' Close Gripper '";
			speak_pub.publish(say_this);
		} else if(command == 1){
			ROS_INFO("Gripper opened");
			say_this.data = "' Open Gripper '";
			speak_pub.publish(say_this);
		}
	}else{
		//ROS_INFO("Gripper don't react. Please have a look.");
	}
}


int startInitialized = 0;
float lastGoalX;
float lastGoalY;
int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_rrt");

		//target_tfl_ptr.reset(new tf::TransformListener);

    ros::NodeHandle n;
		speak_pub = n.advertise<std_msgs::String>("/espeak/string",1);
		battery_sub = n.subscribe<visualization_msgs::Marker>("/visualization_marker_battery", 1, batteryCallback);
	  evidence_sub = n.subscribe<rosie_object_detector::RAS_Evidence>("/evidence",10, evidenceCallback);
		//ros::Subscriber rviz_goal = n.subscribe<geometry_msgs::PoseStamped>("/rviz_object_pose",10,rvizTargetPoseCallback);
		ros::Subscriber rviz_goal = n.subscribe("/rviz_object_pose",10, rvizTargetPoseCallback);
		storeObjClient = n.serviceClient<rosie_map_controller::RequestObjStoring>("request_store_objects");
		loadClient = n.serviceClient<rosie_map_controller::RequestLoading>("request_load_mapping");
		gateClient = n.serviceClient<rosie_servo_controller::ControlGates>("rosie_servo_service");
		collisionClient = n.serviceClient<rosie_path_finder::RequestRerun>("request_rerun");
		rrtClient = n.serviceClient<rosie_path_finder::rrtService>("/rrt");
		objStack_pub = n.advertise<rosie_map_controller::ObjectStoring>("/object_stack",10);
ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odom",100,currentPoseCallback);
		//rosie_map_controller::StartRRT startSrv;
    lastObjStoring = ros::Time::now();
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

	targetPose_ptr.reset(new geometry_msgs::PoseStamped);
	lastTargetPose_ptr.reset(new geometry_msgs::PoseStamped);

		static tf::TransformBroadcaster br;

    ros::Rate loop_rate(10);
	  ros::Time load_time = ros::Time::now();
		bool collisionDetected = 0;

    while(ros::ok()){
//***********************************
//STATE-MACHINE
//***********************************
		if(checkedStoredObjects){
			collisionSrv.request.question = 1;
			if(collisionClient.call(collisionSrv)){
				if(collisionSrv.response.answer){
					collisionDetected = 1;
				}
			}
		}
		if((targetPose_ptr->header.seq != lastTargetPose_ptr->header.seq) || collisionDetected){
			lastTargetPose_ptr->pose.position.x = targetPose_ptr->pose.position.x;
			lastTargetPose_ptr->pose.position.y = targetPose_ptr->pose.position.y;
			lastTargetPose_ptr->header.seq = targetPose_ptr->header.seq;
			lastTargetPose_ptr->header.frame_id = targetPose_ptr->header.frame_id;
			lastTargetPose_ptr->header.stamp = targetPose_ptr->header.stamp;

			rrtSrv.request.goalx = lastTargetPose_ptr->pose.position.x;
			rrtSrv.request.goaly = lastTargetPose_ptr->pose.position.y;
			rrtSrv.request.mode = 0; //go to target
			if(rrtClient.call(rrtSrv)){
				ROS_ERROR("published");
			}else{
				ROS_ERROR("problems publishing");
			}

		}
		ROS_ERROR("pose %f %f target %f %f",pose.pose.pose.position.x,pose.pose.pose.position.y,(lastTargetPose_ptr->pose.position.x),(lastTargetPose_ptr->pose.position.y));
		if(0.05*0.05 < (pow(pose.pose.pose.position.x-(lastTargetPose_ptr->pose.position.x),2)+pow(pose.pose.pose.position.y-(lastTargetPose_ptr->pose.position.y),2)) < 0.2*0.2){
			ROS_ERROR("open gripper");
			actuateGripper(1); //open
		}else if (0 < (pow(pose.pose.pose.position.x-(lastTargetPose_ptr->pose.position.x),2)+pow(pose.pose.pose.position.y-(lastTargetPose_ptr->pose.position.y),2)) < 0.05*0.05){
			ROS_ERROR("close gripper");
			actuateGripper(0); //close
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}
