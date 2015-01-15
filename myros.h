/** @file
 * @brief ROSの本体及びPublisherとSubscriberのライブラリ
 * @author kurenaif
 * @date 2014-12-17
 */

#ifndef MYROS_H
#define MYROS_H

#include <memory>
#include <ros/ros.h>
#include <string>

template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args)
{
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

/**
 * @brief ROSの用意をする(いろいろする前に, まずこいつを呼んでください)
 */
class MyROS
{
	private:
		std::unique_ptr<ros::NodeHandle> m_node;
		std::unique_ptr<ros::Rate> m_loopRate;

	public:
		/**
		 * @brief ROSのnodeを制作する
		 *
		 * @param argc コマンドライン引数の数
		 * @param argv コマンドライン引き数
		 * @param nodeName nodeの名前
		 * @param loopRate sleepさせる時間
		 */
	MyROS(int argc, char** argv, const std::string &nodeName, const int loopRate=10){
		ros::init(argc, argv, nodeName);
		m_node = make_unique<ros::NodeHandle>("~");
		m_loopRate = make_unique<ros::Rate>(loopRate);

		ros::Time::init();
	}

	/**
	 * @brief While(Run()){}で使用する. Subscriberのcallbackを呼んだり, sleep()を呼んだりする
	 *
	 * @return ROSを続行するかどうか
	 */
	bool Run(){
		if(!ros::ok()) return false;
		m_loopRate->sleep();
		ros::spinOnce();
		return true;
	}

	/**
	 * @brief nodeHandleのポインタを取得する
	 *
	 * @return nodeHandleのポインタ
	 */
	ros::NodeHandle* GetNodeHandlePointer(){return m_node.get();}
};


/**
 * @brief Subscriberオブジェクト
 *
 * @tparam T Subscribeする肩
 */
template<class T>
class MySubscriber 
{
	private:
		T m_value;
		ros::Subscriber m_sub;
		bool m_isNew = false;
	public:
		/**
		 * @brief callback関数. 外向けのものではない
		 *
		 * @param msg callbackされてくるmessage
		 */
		void callback(const typename T::ConstPtr& msg){
			m_value = *msg;
			m_isNew = true;
		}

		/**
		 * @brief subscriberを制作する.
		 *
		 * @param node subscribeするnode
		 * @param topicNama topicの名前
		 * @param queue_size queueのサイズ
		 */
		MySubscriber(ros::NodeHandle *node, const std::string& topicName, const int queue_size=1):m_sub(node->subscribe(topicName, queue_size, &MySubscriber::callback, this)){}


		/**
		 * @brief 取得している値を返す
		 *
		 * @tparam U 変換してほしい型の名前(不可能な場合, コンパイルエラー)
		 *
		 * @return 取得している値
		 */
		template<class U>
		U Get()
		{
			m_isNew = false;
			return m_value.data;
		}

		/**
		 * @brief 取得している値を返す
		 *
		 * @return 取得している値(内部dataではなく, まるまる全てを返すので注意)
		 */
		T Get()
		{
			m_isNew = false;
			return m_value;
		}

		/**
		 * @brief 前回のGet()から更新があったかどうか
		 *
		 * @retval true 更新があった
		 * @retval false 更新がなかった
		 */
		bool IsNew(){return m_isNew;}

		template<class U>
		void operator >> (U& dst){dst=m_value.data;}
};

/**
 * @brief Publisherオブジェクト
 *
 * @tparam T publishする型の名前
 */
template<class T>
class MyPublisher
{
	private:
		ros::Publisher m_pub;
		std::string m_topicName;

	public:
		/**
		 * @brief Publisherを準備する
		 *
		 * @param node 準備するnodeの名前
		 * @param topicName topicの名前
		 * @param queue_size queueのサイズ
		 */
		MyPublisher(ros::NodeHandle *node, std::string topicName, int queue_size=1):m_topicName(topicName),m_pub(node->advertise<T>(topicName,1)){}


		/**
		 * @brief 値をpublishする
		 *
		 * @tparam U publishする型(T.dataに変換できなければコンパイルエラ-)
		 * @param value publishする値
		 */
		template<class U>
		void Push(U value)
		{
			T msg;
			msg.data = value;
			m_pub.publish(msg);
		}

		/**
		 * @brief コンストラクタ時に設定したtopicの名前を取得する(Subscribeにでもどうぞ)
		 *
		 * @return topicの名前
		 */
		std::string GetTopicName(){return m_topicName;}


		/**
		 * @brief .Pushする
		 *
		 * @tparam U pushする型
		 * @param value pushする値
		 */
		template<class U>
		void operator << (U value){Push(value);}
};
#endif /* MYROS_H */

