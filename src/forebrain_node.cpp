#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8MultiArray.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TwistStamped.h"




int main(int argc, char **argv)
{
  	ros::init(argc, argv, "forebrain2Arbiter");
  	//handle for nodes.
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::Int8MultiArray>("image_cmd_vel", 1000);
  	ros::Rate loop_rate(10);
  
  	while (ros::ok())
  	{
    	std_msgs::Int8MultiArray msg;
    	
    	//Image Processing Function
    	//Below code is temporary
    	for(int x = 0; x<22; x++)
    	{
    			msg.data.push_back(x);
    	}
    	//Above Code is temporary



    	//Published a message
	    chatter_pub.publish(msg);
    	//Wait for callbacks, not needed, but useful here to wait on other things. Look into other descriptions. 
    	ros::spinOnce();
    	//wait until loop_rate is hit. 
    	loop_rate.sleep();
	}

	return 0;
}
