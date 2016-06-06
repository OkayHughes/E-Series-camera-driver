#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_ros/point_cloud.h>

#include <stdlib.h>
#include <stdio.h>
#include <vector>

#include <fz_api.h>
#include <iterator>

#include <ctime>

#define HERTZ 10
#define MAX_DEVICES 40
#define HEIGHT 160
#define WIDTH 120
#define PFIELDS 4
#define PBYTESPER 2
#define ROWBYTES (HEIGHT*WIDTH*PBYTESPER)



int main(int argc, char**argv) {

	//create a new ros node
	ros::init(argc, argv, "ESeries_Publisher");
	//get handler for ros node
	ros::NodeHandle nh;
	//Establish it as image transport node and start publishing.
	ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("/pointcloud/ESeries", 1);
	
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
	uint16_t FPS = 40;
	uint16_t* image = new uint16_t[WIDTH*HEIGHT*PFIELDS];
	FZ_FRAME_HEADER frameHeader;
	size_t bufsize = sizeof(image);

	//Initialize the API
	FZ_Init();
	
	//set up logging 
	int iFlags = FZ_LOG_TO_STDOUT;
	iFlags |= FZ_LOG_ERROR | FZ_LOG_WARN | FZ_LOG_INFO;
	FZ_SetLogging(iFlags, NULL, NULL);
	//TODO: SET UP LOGGING FOR PRODUCTION
	
	//enumerate connected devices
	int numDevices = MAX_DEVICES;
	result = FZ_EnumDevices2(deviceInfo, &numDevices);
	if(result!=FZ_Success || numDevices<1){
		ROS_INFO("ERROR: Devices not found\n");
		return 1;
	}

	//open first device
	result = FZ_Open(deviceInfo[0].szPath, 0,  &device);
	if(result != FZ_Success){
		ROS_INFO("ERROR: Could not open device\n");
		return 1;
	}
	
	//note: first NULL could be useful if camera becoes unresponsive.
	//Returns wheter acknowledged or not;
	result = FZ_IOCtl(device, CMD_DE_SET_MODE, &mode, sizeof(mode), NULL, NULL, NULL);
	if(result !=FZ_Success){
		ROS_INFO("ERROR: Could not set camera mode\n");
		return 1;
	}

	//set shutter
	result = FZ_IOCtl(device, CMD_DE_SET_SHUTTER, &shutterMs, sizeof(shutterMs), NULL, NULL, NULL);
	if(result != FZ_Success){
		ROS_INFO("ERROR: Could not set shutter speed\n");
		return 1;
	}
	
	//set frame rate
	result = FZ_IOCtl(device, CMD_DE_SET_FPS, &FPS, sizeof(FPS), NULL, NULL, NULL);
	if(result != FZ_Success){
		ROS_INFO("ERROR: couldn't set FPS\n");
		return 1;
	}

	//start sensor!
	result = FZ_IOCtl(device, CMD_DE_SENSOR_START, NULL, 0, NULL, NULL, NULL);
	if(result != FZ_Success){
		ROS_INFO("ERROR: couldn't start sensor\n");
		return 1;
	}

	

	//get frames at HERTZ times per second.
	ros::Rate loop_rate(HERTZ);
	
	
	pcl::PointCloud<pcl::PointXYZI> cloud;
	

	while (nh.ok()){
		result = FZ_GetFrame(device, &frameHeader, image, &bufsize);
		if(result != FZ_Success){
			ROS_INFO("ERROR: Could not get frame\n");
			return 1;
		}

		//Convert image to point cloud
		cloud = pcl::PointCloud<pcl::PointXYZI>();
		cloud.width = WIDTH;
		cloud.height = HEIGHT;
		
		cloud.points.resize(WIDTH * HEIGHT);
		
		
		//Image lists fields by row. Don't ask me why.
		for(int i = 0; i < HEIGHT*WIDTH; i++){
			cloud.points[i].intensity = (float) image[i];
			cloud.points[i].z = (float) image[i + HEIGHT * WIDTH ];
			cloud.points[i].x = (float) image[i + HEIGHT * WIDTH  * 2] ;
			cloud.points[i].y = (float) image[i + HEIGHT * WIDTH * 3] ;
		}
		

		
		pub.publish(cloud);

		ros::spinOnce();
		loop_rate.sleep();
	}

	//stop sensor
	result = FZ_IOCtl(device, CMD_DE_SENSOR_STOP, NULL, 0, NULL, NULL, NULL);
	if(result != FZ_Success){
		ROS_INFO("ERROR: Could not stop sensor\n");
		return 1;
	}
	
	//disconnect
	FZ_Close(device);

	FZ_Exit();

	return 0;

}
