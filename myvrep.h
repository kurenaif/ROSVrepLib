/** @file
 * @brief VrepとROSノードをつなぐためのライブラリ
 * @author kurenaif
 * @date 2014-12-17
 */

#ifndef MYVREP_H
#define MYVREP_H

#include "myros.h"

#include <vrep_common/VrepInfo.h>
#include <vrep_common/simRosGetObjectHandle.h>
#include <vrep_common/simRosLoadModel.h>
#include <vrep_common/simRosRemoveModel.h>
#include <vrep_common/simRosStartSimulation.h>
#include <vrep_common/simRosStopSimulation.h>
#include <vrep_common/simRosEnablePublisher.h>
#include <vrep_common/simRosEnableSubscriber.h>
#include <vrep_common/simRosSetJointTargetVelocity.h>
#include <vrep_common/simRosSetJointTargetPosition.h>

class MyVrep
{
	private:
		MySubscriber<vrep_common::VrepInfo> m_infoSub;
		std::string m_ns;
		ros::NodeHandle* m_node;
	public:
		/**
		 * @brief コンストラクタ, 名前空間とinfocallbackのスタンバイ
		 *
		 * @param node nodeHandleのポインタ
		 */
		MyVrep(ros::NodeHandle* node):m_ns(ros::this_node::getNamespace()),
			m_infoSub(node, ros::this_node::getNamespace()+"vrep/info", 1),
			m_node(node){}

		/**
		 * @brief 取得した時間を返す
		 *
		 * @return Vrep上での時間
		 */
		float GetTime(){return m_infoSub.Get().simulationTime.data;}
		/**
		 * @brief Vrepがいま動いているかどうかを返す
		 *
		 * @retval true Vrepがいま動いている
		 * @retval false Vrepがいま止まっている
		 */
		bool IsRun(){return (m_infoSub.Get().simulatorState.data & 1) != 0;}

		/**
		 * @brief ObjectのHandleを取得する
		 *
		 * @param objectName 取得するオブジェクトの名前
		 *
		 * @return 取得したオブジェクト
		 * @retval -1 オブジェクトのハンドル取得に失敗
		 */
		int GetObjectHandle(std::string objectName){
			ros::ServiceClient getHandleClient = m_node->serviceClient<vrep_common::simRosGetObjectHandle>(m_ns+"/vrep/simRosGetObjectHandle");
			vrep_common::simRosGetObjectHandle s;
			s.request.objectName = objectName;
			getHandleClient.call(s);
			return s.response.handle;
		}

		/**
		 * @brief Modelを読み込む
		 *
		 * @param filename 読み込むファイル(.ttm)
		 */
		void LoadModel(std::string filename){
			ros::ServiceClient client = m_node->serviceClient<vrep_common::simRosLoadModel>(m_ns+"/vrep/simRosLoadModel");
			vrep_common::simRosLoadModel s;
			s.request.fileName = filename;

			client.call(s);
		}

		/**
		 * @brief Modelを除去する
		 *
		 * @param handle 除去するModelのハンドル
		 */
		void RemoveModel(int handle){
			ros::ServiceClient client = m_node->serviceClient<vrep_common::simRosRemoveModel>(m_ns+"/vrep/simRosRemoveModel");
			vrep_common::simRosRemoveModel s;
			s.request.handle = handle;
			client.call(s);
		}

		/**
		 * @brief Modelを除去する
		 *
		 * @param modelName 除去するModelの名前
		 */
		void RemoveModel(std::string modelName){
			RemoveModel(GetObjectHandle(modelName));
		}

		/**
		 * @brief VrepをRunさせる
		 */
		void Start(){
			ros::ServiceClient client = m_node->serviceClient<vrep_common::simRosStartSimulation>(m_ns+"/vrep/simRosStartSimulation");
			vrep_common::simRosStartSimulation s;
			client.call(s);
		}

		/**
		 * @brief Vrepを止める
		 */
		void Stop(){
			ros::ServiceClient client = m_node->serviceClient<vrep_common::simRosStopSimulation>(m_ns+"/vrep/simRosStopSimulation");
			vrep_common::simRosStopSimulation s;
			client.call(s);
		}

		/**
		 * @brief Vrep上のオブジェクトに対し, Publisherを要求する
		 *
		 * @param topicName Publishするときのtopic
		 * @param queueSize queueサイズ
		 * @param streamCmd streamCmd
		 * @param auxInt1 auxInt1(streamCmd参照)
		 * @param auxInt2 auxInt2(streamCmd参照)
		 * @param auxString auxString(streamCmd参照)
		 *
		 * @retval true 成功
		 * @retval false 失敗
		 */
		bool EnablePublisher(std::string topicName, int queueSize, int streamCmd, int auxInt1, int auxInt2, std::string auxString){
			ros::ServiceClient client = m_node->serviceClient<vrep_common::simRosEnablePublisher>(m_ns+"/vrep/simRosEnablePublisher");
			vrep_common::simRosEnablePublisher s;
			s.request.topicName = topicName;
			s.request.queueSize = queueSize;
			s.request.streamCmd = streamCmd;
			s.request.auxInt1 = auxInt1;
			s.request.auxInt2 = auxInt2;
			s.request.auxString = auxString;

			if(!client.call(s)) return false;
			return true;
		}

		/**
		 * @brief Subscriberを要求する
		 *
		 * @param topicName 要求するtopicの名前
		 * @param queueSize queueのサイズ
		 * @param streamCmd streamCmd
		 * @param auxInt1(streamCmd参照)
		 * @param auxInt2(streamCmd参照)
		 * @param auxString(streamCmd参照)
		 *
		 * @retval true 成功
		 * @retval false 失敗
		 */
		bool EnableSubscriber(std::string topicName, int queueSize, int streamCmd, int auxInt1, int auxInt2, std::string auxString){
			ros::ServiceClient client = m_node->serviceClient<vrep_common::simRosEnableSubscriber>(m_ns+"/vrep/simRosEnableSubscriber");
			vrep_common::simRosEnableSubscriber s;
			s.request.topicName = topicName;
			s.request.queueSize = queueSize;
			s.request.streamCmd = streamCmd;
			s.request.auxInt1 = auxInt1;
			s.request.auxInt2 = auxInt2;
			s.request.auxString = auxString;

			if(!client.call(s) || (s.response.subscriberID == -1)) return false;
			return true;
		}

		/**
		 * @brief Jointの速度を設定する
		 *
		 * @param handle Jointのハンドル
		 * @param value 設定する速度
		 *
		 * @retval true 成功
		 * @retval false 失敗
		 */
		bool SetJointTargetVelocity(int handle, float value){
			ros::ServiceClient client = m_node->serviceClient<vrep_common::simRosSetJointTargetVelocity>(m_ns+"/vrep/simRosSetJointTargetVelocity");
			vrep_common::simRosSetJointTargetVelocity s;
			s.request.handle = handle;
			s.request.targetVelocity = value;

			if(!client.call(s) || (s.response.result == -1)) return false;
			return true;
		}

		/**
		 * @brief Jointの速度を設定する　
		 *
		 * @param objectName Jointの名前
		 * @param value 設定する速度
		 *
		 * @retval true 成功
		 * @retval false 失敗
		 */
		bool SetJointTargetVelocity(std::string objectName, float value){
			return SetJointTargetVelocity(objectName, value);
		}
};

#endif
