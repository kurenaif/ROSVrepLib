#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <string>

#include "MyROS.h"

int main(int argc, char** argv)
{
	MyROS ros(argc,argv,"node");

	MyPublisher<std_msgs::Int64> pub(ros.GetNodeHandlePointer(), "MyPublish");
	MySubscriber<std_msgs::Int64> sub(ros.GetNodeHandlePointer(), "MyPublish");

	int i=0;
	while(ros.Run())
	{
		int a;
		sub >> a;
		std::cout << a << std::endl;
		pub << i;
		++i;
	}
	return 0;
}
