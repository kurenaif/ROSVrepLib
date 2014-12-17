#include "myros.h"
#include "myvrep.h"
#include <iostream>
#include <memory>
#include <string>
#include <std_msgs/Int64.h>
#include <v_repConst.h>
#include <std_msgs/Float64.h>

int main(int argc, char** argv)
{
	MyROS ros(argc,argv,"node");

	// MyPublisher<std_msgs::Int64> pub(ros.GetNodeHandlePointer(), "MyPublish");
	// MySubscriber<std_msgs::Int64> sub(ros.GetNodeHandlePointer(), "MyPublish");
	MyVrep myVrep(ros.GetNodeHandlePointer());

	myVrep.LoadModel("/home/shimizu/vrep/models/vehicles/helicopter.ttm");

	MyPublisher<std_msgs::Float64> pub(ros.GetNodeHandlePointer(), "motor");
	myVrep.EnableSubscriber(pub.GetTopicName(), 1, simros_strmcmd_set_joint_force, myVrep.GetObjectHandle("motor"), -1, "");

	myVrep.Start();

	float i=0;
	while(ros.Run())
	{

		int a;
		// std::cout << a << std::endl;
		std::cout << myVrep.GetTime() << std::endl;
		pub << i;
		i+=1.0;

	}
	return 0;
}
