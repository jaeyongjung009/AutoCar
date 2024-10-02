#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>
#include <fstream>
#include <iomanip>  // for std::fixed and std::setprecision

// CSV 파일 출력을 위한 ofstream 객체
std::ofstream csv_file;

// IMU 자기장 데이터를 수신하고 CSV 파일에 기록하는 콜백 함수
void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg) {
    // 현재 시간 (초 단위)과 자기장 데이터를 추출하여 CSV 형식으로 기록
    double timestamp = msg->header.stamp.toSec();
    
    // 자기장 데이터를 CSV 형식으로 기록
    csv_file << std::fixed << std::setprecision(6) << timestamp << ","
             << msg->magnetic_field.x << ","
             << msg->magnetic_field.y << ","
             << msg->magnetic_field.z << "\n";
}

int main(int argc, char** argv) {
    // ROS 노드 초기화
    ros::init(argc, argv, "imu_mag_logging");
    ros::NodeHandle nh;

    // CSV 파일 열기 (파일명을 필요에 따라 변경 가능)
    csv_file.open("imu_mag_data.csv");

    // CSV 파일에 헤더 작성
    csv_file << "Timestamp,MagneticField_x,MagneticField_y,MagneticField_z\n";

    // /imu/mag 토픽을 구독
    ros::Subscriber sub = nh.subscribe("/imu/mag", 1, magCallback);
    
    // ROS 이벤트 루프 실행
    ros::spin();

    // 프로그램 종료 시 CSV 파일 닫기
    csv_file.close();

    return 0;
}
