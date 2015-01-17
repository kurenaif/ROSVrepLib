#include "myros.h"
#include "myvrep.h"
#include <iostream>
#include <memory>
#include <string>
#include <cmath>
#include <std_msgs/Int64.h>
#include <v_repConst.h>
#include <std_msgs/Float64.h>

int main(int argc, char** argv)
{
	//ROSの準備をする
	MyROS ros(argc,argv,"sample2");
	//Vrepの準備をする
	MyVrep myVrep(ros.GetNodeHandlePointer());

	//modelの読み込み [ToDo]必ずモデルのパスに合わせてここの行を変えてください.
	myVrep.LoadModel("/home/shimizu/vrep/models/vehicles/helicopter.ttm");

	//Vrepのシミュレーションを実行させる.
	myVrep.Start();
	//Vrepのシミュレーションがちゃんと受け取れる状態になるまで待機する.
	while(ros.Run() && !myVrep.IsRun());

	float i=0;
	while(ros.Run())
	{
		//シミュレーションが終了したらこのプログラムも終了する
		if(!myVrep.IsRun()) break;
		//時間を取得して標準出力
		std::cout << myVrep.GetTime() << std::endl;
		//motorに値を出力する.
		myVrep.SetJointTargetVelocity("motor", i);
		//座標を取得して表示する. 
		std::cout << myVrep.GetObjectPose(myVrep.GetObjectHandle("Cuboid0")) << std::endl;
		//モーターの出力をsin波にする.
		i = std::sin(myVrep.GetTime());
	}
	return 0;
}
