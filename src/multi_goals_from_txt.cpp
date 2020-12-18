#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <math.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

ros::Publisher marker_pub;
int status = 3;
geometry_msgs::Point current_point;
std::vector<geometry_msgs::Pose> pose_list;

void init_goalList(const std::string& file, int& count);
void init_markers(visualization_msgs::Marker *marker);
void activeCb();
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
double computeDistance(geometry_msgs::Point& m_current_point, geometry_msgs::Point& m_goal);


void init_goalList(const std::string& file, int& count)
{
        std::ifstream fin(file);
        if(!fin){
            ROS_INFO("can not find the path files");
            return;
         }
        while(!fin.eof()){
            float x, y, angle;
            fin>>x>>y>>angle;
            geometry_msgs::Pose pose;
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = 0;
            geometry_msgs::Quaternion quaternions = tf::createQuaternionMsgFromYaw(angle*M_PI/180.0);
            pose.orientation = quaternions;
            pose_list.push_back(pose);
            ++count;
        }
        fin.close();
        ROS_INFO("total goals is : %d ", count);
}

// Init markers
void init_markers(visualization_msgs::Marker *marker)
{
	marker->ns = "waypoints";
	marker->id = 0;
	marker->type = visualization_msgs::Marker::CUBE_LIST;
	marker->action = visualization_msgs::Marker::ADD;
	marker->lifetime = ros::Duration();
	marker->scale.x = 0.2;
	marker->scale.y = 0.2;
	marker->color.r = 1.0;
	marker->color.g = 0.7;
	marker->color.b = 1.0;
	marker->color.a = 1.0;

	marker->header.frame_id = "map";
	marker->header.stamp = ros::Time::now();
}

void activeCb()
{
	ROS_INFO("Goal Received");
}


void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
	current_point.x = feedback->base_position.pose.position.x;
	current_point.y = feedback->base_position.pose.position.y;
	current_point.z = feedback->base_position.pose.position.z;
}

double computeDistance(geometry_msgs::Point& m_current_point, geometry_msgs::Point& m_goal)
{
	double m_distance;
	m_distance = sqrt(pow(fabs(m_goal.x - m_current_point.x), 2) + pow(fabs(m_goal.y - m_current_point.y), 2));
	return m_distance;
}

void status_callback(const move_base_msgs::MoveBaseActionResult& msg)
{	
	if(msg.status.status == 4)
	{
		status = 4;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "nav_move_base");
	ros::NodeHandle node;

	ros::Rate loop_rate(10);

	//声明客户端
	Client ac("move_base", true);


	marker_pub = node.advertise<visualization_msgs::Marker>("waypoint_markers", 10);
        ros::Subscriber goal_status =node.subscribe("move_base/result", 10, status_callback);

	//从.txt文件中得到导航点，装入pose_list
        std::string file;
        ros::param::get("/nav_move_base/file", file);
        ROS_INFO("The file path is %s .", file.c_str());

	int count = 0, index = 0;
	double distance = 0.0;
	init_goalList(file, count);

	visualization_msgs::Marker  marker_list;
	init_markers(&marker_list);

	for (int i = 0; i < count; ++i)
	{
		marker_list.points.push_back(pose_list[i].position);
	}

	//等待连接server
	ROS_INFO("Waiting for move_base action server...");
	if (!ac.waitForServer(ros::Duration(60)))
	{
		ROS_INFO("Can't connected to move base server");
		return 1;
	}

	ROS_INFO("Connected to move base server");
	ROS_INFO("Starting navigation test");


	//构造需要发送的goal
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose = pose_list[index];

        //send goal
        // activeCb   --- 给server发送目标的回调函数
        // feedbackCb --- server实时反馈的回调函数
	ac.sendGoal(goal, Client::SimpleDoneCallback(), &activeCb, &feedbackCb);
	ROS_INFO("navigating to the %dth goal", index);		

	while (ros::ok())
	{

		marker_pub.publish(marker_list);

		//计算当前位置与目标点的距离
		distance = computeDistance(current_point, goal.target_pose.pose.position);  
		ROS_INFO("distance = %f", distance);

		if (distance <= 0.2)
		{
                        ROS_INFO("achieve the %dth goal successfully!", index);
			++index;
			if (index == count)
			{
				ROS_INFO("All goals have been achieves!");
                                break;
			}

			goal.target_pose.header.stamp = ros::Time::now();
			goal.target_pose.pose = pose_list[index];
			ac.sendGoal(goal, Client::SimpleDoneCallback(), &activeCb, &feedbackCb);
                        ROS_INFO("navigating to the %dth goal", index);
		}
                //这里要获取状态，以防一直到达不了目标点
                if(status == 4){
                      ROS_INFO("the %dth goal failed", index);  
                      ++index;
                      if (index == count)
		      {
				ROS_INFO("All goals have been achieves!");
                                break;
		      }
                      goal.target_pose.header.stamp = ros::Time::now();
                      goal.target_pose.pose = pose_list[index];
                      ac.sendGoal(goal, Client::SimpleDoneCallback(), &activeCb, &feedbackCb);
                      ROS_INFO("navigating to the %dth goal", index);
                      status = 3;
                }
		loop_rate.sleep();
	}
	return 0;
}
