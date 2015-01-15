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
	//ROSを使用するための準備をする
	MyROS ros(argc,argv,"sample1pub");

	//publisherの準備をする
	MyPublisher<std_msgs::Int64> pub(ros.GetNodeHandlePointer(), "MyPublish");

	int i=0;
	while(ros.Run()) //ROSが続いている限りここが回り続ける.(subscribeも行う.)
	{
		pub << i; //iをpublishする
		i+=1.0;
	}
	return 0;
}
