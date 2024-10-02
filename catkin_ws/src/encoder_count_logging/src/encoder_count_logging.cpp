#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <fstream>
#include <string>

// CSV 파일 스트림
std::ofstream encoder_log_file;

// 콜백 함수: 엔코더 데이터를 수신하면 호출됨
void encoderCallback(const std_msgs::Int32::ConstPtr& msg)
{
    // 현재 시간(타임스탬프) 가져오기
    ros::Time current_time = ros::Time::now();

    // 엔코더 값을 로그 파일에 기록
    encoder_log_file << current_time << "," << msg->data << std::endl;

    // 터미널에도 출력
    ROS_INFO("Encoder count: %d", msg->data);
}

int main(int argc, char **argv)
{
    // ROS 노드 초기화
    ros::init(argc, argv, "encoder_logger");

    // 노드 핸들러 생성
    ros::NodeHandle nh;

    // 로그 파일 열기
    encoder_log_file.open("encoder_log.csv");

    // CSV 파일 헤더 추가
    encoder_log_file << "Timestamp,EncoderCount" << std::endl;

    // 토픽 구독자 생성: "encoder_count" 토픽을 구독
    ros::Subscriber sub = nh.subscribe("encoder_count", 1000, encoderCallback);

    // 콜백 함수 호출을 위한 스핀 함수
    ros::spin();

    // 노드가 종료되면 로그 파일 닫기
    encoder_log_file.close();

    return 0;
}
