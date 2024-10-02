#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <fstream>
#include <iomanip>  // for std::fixed and std::setprecision

// CSV 파일 출력을 위한 ofstream 객체
std::ofstream csv_file;

// IMU 데이터를 수신하고 CSV 파일에 기록하는 콜백 함수
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // 현재 시간 (초 단위)과 IMU 데이터를 추출하여 CSV 형식으로 기록
    double timestamp = msg->header.stamp.toSec();
    
    // IMU 데이터를 CSV 형식으로 기록
    csv_file << std::fixed << std::setprecision(6) << timestamp << ","
             << msg->orientation.x << ","
             << msg->orientation.y << ","
             << msg->orientation.z << ","
             << msg->orientation.w << ","
             << msg->angular_velocity.x << ","
             << msg->angular_velocity.y << ","
             << msg->angular_velocity.z << ","
             << msg->linear_acceleration.x << ","
             << msg->linear_acceleration.y << ","
             << msg->linear_acceleration.z << "\n";
}

int main(int argc, char** argv) {
    // ROS 노드 초기화
    ros::init(argc, argv, "imu_logging");
    ros::NodeHandle nh;

    // CSV 파일 열기 (파일명을 필요에 따라 변경 가능)
    csv_file.open("imu_data.csv");

    // CSV 파일에 헤더 작성
    csv_file << "Timestamp,Orientation_x,Orientation_y,Orientation_z,Orientation_w,"
             << "Angular_Velocity_x,Angular_Velocity_y,Angular_Velocity_z,"
             << "Linear_Acceleration_x,Linear_Acceleration_y,Linear_Acceleration_z\n";

    // /imu/filtered_data 토픽을 구독
    
    ros::Subscriber sub = nh.subscribe("imu/data", 1, imuCallback);

    // ROS 이벤트 루프 실행
    ros::spin();

    // 프로그램 종료 시 CSV 파일 닫기
    csv_file.close();

    return 0;
}
