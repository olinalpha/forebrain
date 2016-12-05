#include <cmath>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"

#define PI 3.14159265

bool isInitialized;

double current_x;
double current_y;
double current_heading;

double desired_x;
double desired_y;
double desired_heading;

double next_desired_x;
double next_desired_y;
double next_desired_heading;

//Set current pose based on input
void current_pose_Callback(const geometry_msgs::Pose::ConstPtr& msg)
{
  current_x = msg->position.x;
  current_y = msg->position.y;
  current_heading = msg->orientation.x;
  //ROS_INFO("I heard: [%f, %f, %f], [%f, %f, %f]", msg->position.x, msg->position.y,msg->position.z,msg->orientation.x,msg->orientation.x,msg->orientation.x);
}
//set desired pose based on input
void desired_pose_Callback(const geometry_msgs::Pose::ConstPtr& msg)
{
//Initializes Positions if have not recieved the next desired pose prior to this running the first time. 
  if(isInitialized == false)
	{
		next_desired_y = msg->position.x;
		next_desired_x = msg->position.y;
		next_desired_heading = msg->orientation.x;
		isInitialized = true;
	}

  	desired_x = msg->position.x;
  	desired_y = msg->position.y;
  	desired_heading = msg->orientation.x;
    //ROS_INFO("I heard: [%f, %f, %f], [%f, %f, %f]", msg->position.x, msg->position.y,msg->position.z,msg->orientation.x,msg->orientation.x,msg->orientation.x);
}

void next_desired_Callback(const geometry_msgs::Pose::ConstPtr& msg)
{
  //sets initial. DEPRECATED
 // if(isInitialized == false){
  //	isInitialized = true;}


  next_desired_x = msg->position.x;
  next_desired_y = msg->position.y;
  next_desired_heading = msg->orientation.x;
  //ROS_INFO("I heard: [%f, %f, %f], [%f, %f, %f]", msg->position.x, msg->position.y,msg->position.z,msg->orientation.x,msg->orientation.x,msg->orientation.x);
  ROS_INFO("NEXT DESIRED POSE GOT");
}


int main(int argc, char **argv)
{
	//Initialization of a bunch of things for ROS
  	ros::init(argc, argv, "dubins_pathplan");
  	isInitialized = false;
  	//handle for nodes.
	ros::NodeHandle n;
	//Sets up publishers and subscribers
	ros::Publisher heading_pub = n.advertise<std_msgs::Float32MultiArray>("dubins_cmd_vel", 1000);  
	ros::Publisher  at_location_pub= n.advertise<std_msgs::Bool>("at_location", 1000);  
    ros::Subscriber current_sub = n.subscribe("current_pose", 1000, current_pose_Callback);
    ros::Subscriber desired_sub = n.subscribe("desired_pose", 1000, desired_pose_Callback);
    ros::Subscriber next_desired_sub = n.subscribe("next_desired_pose", 1000, next_desired_Callback);

  ros::Rate loop_rate(50);
//Loop Over this
  while (ros::ok())
  {
     	
     	std_msgs::Float32MultiArray msg;
     	//Avoid sending anything until it gets the desired locations. 
     	if(isInitialized){

	      //No velocity preference, adding all 0's to the system, hopefully won't make a difference
	      for(int x = 0; x<22; x++)
	    	{
    			msg.data.push_back(0);
	    	}
	
    	  //Need to find desired heading vector from current to desired: 
	      double distanceX = (desired_x - current_x);
	      double distanceY = (desired_y - current_y);
	      //Get distance via distance formula.
    	  double distance = sqrt(distanceX*distanceX + distanceY*distanceY);
    	  //Gets the heading of the vector from current to desired in terms of global
      	  double global_angle = atan2(distanceY, distanceX);
	      //Find relative heading
    	  double local_heading = global_angle - current_heading;
      // Limit heading to +/- 90 degrees (PI/2 radians)
      if(abs(local_heading)>PI/2.0)
      {
          if(local_heading>0)
            local_heading = PI/2.0;
          else if(local_heading<0)
            local_heading = -PI/2.0;
      }      

      //If distance is within 1 meter, switch to next waypoint, and publish a message saying it got to the waypoint. 
      if(distance <1.0){
      	ROS_INFO("AT LOCATION");
      	std_msgs::Bool atlocation;
      	atlocation.data = true;
      	at_location_pub.publish(atlocation);
		desired_x = next_desired_x;
		desired_y = next_desired_y;
		desired_heading = next_desired_heading;
		atlocation.data = false;
      }


      //Maps the relative heading to a discrete point 0-10
      int discrete_angle = (int)round(local_heading/PI*10.0);
      msg.data[discrete_angle+16] = 1;
      //Gaussian Distribution

      for(int x = 11; x<22; x++){
        double distribution = exp(-((double)x-(double)(discrete_angle+16))*((double)x-(double)(discrete_angle+16))/2.0);
        msg.data[x] = distribution;

      }

      //Print for details.

     ROS_INFO("current_angle: %f discrete_angle: %d, %d", global_angle, discrete_angle+16, discrete_angle);
     ROS_INFO("Current Distribution %.2f, %.2f, %.2f, %.2f, %.2f %.2f %.2f %.2f %.2f %.2f %.2f", msg.data[11],msg.data[12],msg.data[13],msg.data[14],msg.data[15],msg.data[16],msg.data[17],msg.data[18],msg.data[19],msg.data[20],msg.data[21]);
     ROS_INFO("Current: %f %f %f", current_x, current_y, current_heading);
     ROS_INFO("Desired: %f %f %f", desired_x, desired_y, desired_heading);
      heading_pub.publish(msg);
      	}
    	//Published a message
//	    heading_pub.publish(msg);
    	//Wait for callbacks, not needed, but useful here to wait on other things. Look into other descriptions. 
    	ros::spinOnce();
    	//wait until loop_rate is hit. 
    	loop_rate.sleep();
	}

	return 0;
}
