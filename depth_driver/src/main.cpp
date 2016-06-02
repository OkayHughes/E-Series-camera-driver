#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <stdlib.h>
#include <stdio.h>

#include <fz_api.h>

#define HERTZ 10
#define MAX_DEVICES 40


int main(int argc, char**argv) {

	//create a new ros node
	ros::init(argc, argv, "ESeries_Publisher");
	//get handler for ros node
	ros::NodeHandle nh;
	//Establish it as image transport node and start publishing.
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("camera/image", 1);
	
	//Variables for initializing FZ api
	//Gets device info from EnumDevices2
	FZ_DEVICE_INFO deviceInfo[MAX_DEVICES];
	//records return value from pretty much every API operation
	FZ_Result result;
	//Device handle for communicating with camera
	FZ_Device_Handle_t device;
	
	//config variables
	//mode may be wrong. Alternative is DE_MODE_640X480_640X480; 
	uint16_t mode = DE_MODE_640X480_30;
	uint16_t shutterMs = 20;//2ms
	uint16_t iFPS = 40;

	//Initialize the API
	/FZ_Init();
	
	//set up logging 
	int iFlags = FZ_LOG_TO_STDOUT;
	iFlags |= FZ_LOG_ERROR | FZ_LOG_WARN | FZ_LOG_INFO;
	FZ_SetLogging(iFlags, NULL, FZLogCallback);
	//TODO: SET UP LOGGING FOR PRODUCTION
	
	//enumerate connected devices
	int numDevices = MAX_DEVICES;
	result = FZ_EnumDevices2(deviceInfo, &iNumDevices);
	if(result!=FZ_Success || numDevices<1){
		printf("ERROR: Devices not found\n");
		return 1;
	}

	//open first device
	result = FZ_Open(deviceInfo[0].szPath, 0,  &device);
	if(result != FZ_Success){
		printf("ERROR: Could not open device\n");
		return 1;
	}
	
	//note: first NULL could be useful if camera becoes unresponsive.
	//Returns wheter acknowledged or not;
	result = FZ_IOCtl(device, CMD_DE_SET_MODE, &mode, sizeof(mode), NULL, NULL, NULL);
	if(result !=FZ_Success){
		printf("ERROR: Could not set camera mode\n");
		return 1;
	}

	//set shutter
	result = FZ_IOCtl(device, CMD_DE_SET_SHUTTER, &shutterMs, sizeof(shutterMs), NULL, NULL, NULL);
	if(result != FZ_Success){
		printf("ERROR: Could not set shutter speed\n");
		return 1;
	}

	
	//Message format that works with publishing
	sensor_msgs::ImagePtr msg;

	//get frames at HERTZ times per second.
	ros::Rate loop_rate(HERTZ);

	while (nh.ok()){
		//TODO: Construct Msg from raw data
		
		pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	}
}
