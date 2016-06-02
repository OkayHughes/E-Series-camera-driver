#include<ros/ros.h>
#include <image_transport/image_transport.h>

#include <stdlib.h>
#include <stdio.h>

#include <fz_api.h>

#define HERTZ 10

int main(int argc, char**argv) {

	//create a new ros node
	ros::init(argc, argv, "Fotonic E-Series Publisher");
	//get handler for ros node
	ros::NodeHandle nh;
	//Establish it as image transport node and start publishing.
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("camera/image", 1);
	

	/*
	=================================================================================
	TODO: Get raw data from E-Series Camera
	*/
	
	//Message format that works with publishing
	sensor_msgs::ImagePtr msg;

	//get frames at HERTZ times per second.
	ros::Rate loop_rate(HERTZ);

	while (nh.ok()){
		//TODO: Construct Msg from raw data
		
		pub.publish(msg);

		ros.spinOnce();
		loop_rate.sleep();
	}
}
