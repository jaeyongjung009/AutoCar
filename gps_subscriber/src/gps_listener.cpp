#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <sstream>
#include <iomanip>
#include <ctime>

// 전역 변수로 파일 스트림 객체와 파일 이름
std::ofstream file;
std::string filename;

// 함수로 현재 시간 문자열 생성
std::string getCurrentTimeString()
{
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

// 파일 열기 함수
void openFile()
{
    filename = "/home/jjuuoo/gps_data_" + getCurrentTimeString() + ".csv";
    file.open(filename);
    if (file.is_open())
    {
        file << "timestamp,latitude,longitude\n"; // 헤더 추가
    }
    else
    {
        ROS_ERROR("Unable to open file for writing.");
    }
}

// 콜백 함수 정의
void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    if (file.is_open())
    {
        // 데이터 저장
        file << std::fixed << std::setprecision(8) // 소수점 이하 8자리까지 출력
             << msg->header.stamp.toSec() << "," 
             << msg->latitude << "," 
             << msg->longitude << "\n";
           
    }
    else
    {
        ROS_ERROR("File stream is not open.");
    }
    
    // 콘솔에도 출력
    ROS_INFO_STREAM("Latitude: " << std::fixed << std::setprecision(8) << msg->latitude 
                        << ", Longitude: " << std::fixed << std::setprecision(8) << msg->longitude);
}

// 종료 핸들러
void shutdownHandler(int signal)
{
    if (file.is_open())
    {
        file.close();
        ROS_INFO("File %s closed successfully.", filename.c_str());
    }
    ros::shutdown();
}

int main(int argc, char** argv)
{
    // ROS 초기화
    ros::init(argc, argv, "gps_listener");
    ros::NodeHandle nh;

    // 종료 핸들러 등록
    signal(SIGINT, shutdownHandler);

    // 파일 열기
    openFile();

    // /ublox_gps/fix 토픽을 구독
    ros::Subscriber sub = nh.subscribe("/ublox_gps/fix", 10, fixCallback);

    // 콜백 함수 호출을 위한 루프
    ros::spin();

    return 0;
}

