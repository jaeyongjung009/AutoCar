#include "ros/ros.h"
#include "std_msgs/Int32.h"

// 전역 변수로 카운터를 선언
int encoder_count = 0;

// 콜백 함수: 엔코더 데이터를 수신하면 호출됨
void encoderCallback(const std_msgs::Int32::ConstPtr& msg)
{
    encoder_count = msg->data;
    ROS_INFO("Encoder count: %d", encoder_count);
}

int main(int argc, char **argv)
{
    // ROS 노드 초기화
    ros::init(argc, argv, "encoder_listener");

    // 노드 핸들러 생성
    ros::NodeHandle nh;

    // 카운터 초기화
    encoder_count = 0;
    ROS_INFO("Encoder count initialized to: %d", encoder_count);

    // 토픽 구독자 생성: "encoder_count" 토픽을 구독
    ros::Subscriber sub = nh.subscribe("encoder_count", 1000, encoderCallback);

    // 콜백 함수 호출을 위한 스핀 함수
    ros::spin();

    return 0;
}
