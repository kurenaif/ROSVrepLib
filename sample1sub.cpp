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
	//ROSの準備をする
	MyROS ros(argc,argv,"sample1sub");

	//subscriberの準備をする
	MySubscriber<std_msgs::Int64> sub(ros.GetNodeHandlePointer(), "/sample1pub/MyPublish");
	while(ros.Run()) //ROSのsubscribeと終われば終了するという処理を並行して行う.
	{
		int i;
		sub >> i; //指定したtopicからデータを受け取り, iに代入する.
		std::cout << i << std::endl;
	}
	return 0;
}
