#include <cmath>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8MultiArray.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"

#define PI 3.14159265

double current_x;
double current_y;
//double current_z;
double current_heading;

double desired_x;
double desired_y;
//double desired_z;
double desired_heading;



void current_pose_Callback(const geometry_msgs::Pose::ConstPtr& msg)
{
  current_x = msg->position.x;
  current_y = msg->position.y;
  current_heading = msg->orientation.x;
  ROS_INFO("I heard: [%f, %f, %f], [%f, %f, %f]", msg->position.x, msg->position.y,msg->position.z,msg->orientation.x,msg->orientation.x,msg->orientation.x);
}
void desired_pose_Callback(const geometry_msgs::Pose::ConstPtr& msg)
{
  desired_x = msg->position.x;
  desired_y = msg->position.y;
  desired_heading = msg->orientation.x;
  ROS_INFO("I heard: [%f, %f, %f], [%f, %f, %f]", msg->position.x, msg->position.y,msg->position.z,msg->orientation.x,msg->orientation.x,msg->orientation.x);
}


int main(int argc, char **argv)
{
  	ros::init(argc, argv, "dubins_pathplan");
  	//handle for nodes.
	ros::NodeHandle n;
	ros::Publisher heading_pub = n.advertise<std_msgs::Int8MultiArray>("dubins_cmd_vel", 1000);  
  ros::Subscriber current_sub = n.subscribe("current_pose", 1000, current_pose_Callback);
  ros::Subscriber desired_sub = n.subscribe("desired_pose", 1000, desired_pose_Callback);

  ros::Rate loop_rate(50);
  uint heading[8] = {0,0,0,0,0,0,0,0};
  while (ros::ok())
  {
     	std_msgs::Int8MultiArray msg;


      //No velocity preference, adding all 0's to the system, hopefully won't make a difference
      for(int x = 0; x<22; x++)
    	{
    			msg.data.push_back(0);
    	}
      //Pushing back heading

      //Need to find desired heading: 
      double distanceX = (desired_x - current_x);
      double distanceY = (desired_y - current_y);
//      double distance = sqrt(distanceX*distanceX + distanceY*distanceY);

      ROS_INFO("distanceX: %f distanceY: %f", distanceX, distanceY);
      double current_angle = atan2(distanceY, distanceX);
      ROS_INFO("current_angle: %f", current_angle);

      if(abs(current_angle)>PI/2.0)
      {
          if(current_angle>0)
            current_angle = PI/2.0;
          else if(current_angle<0)
            current_angle = -PI/2.0;
      }      
      int discrete_angle = (int)round(current_angle/PI*10.0);
      msg.data[discrete_angle+16] = 1;
      ROS_INFO("current_angle: %f discrete_angle: %d", current_angle, discrete_angle+16);
      heading_pub.publish(msg);

    	//Published a message
//	    heading_pub.publish(msg);
    	//Wait for callbacks, not needed, but useful here to wait on other things. Look into other descriptions. 
    	ros::spinOnce();
    	//wait until loop_rate is hit. 
    	loop_rate.sleep();
	}

	return 0;
}
