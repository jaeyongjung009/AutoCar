#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

// 보정 변수 (적절한 값으로 수정)
double accel_offset_x = 0.0;
double accel_offset_y = 0.0;
double accel_offset_z = 0.0;

double gyro_offset_x = 0.0;
double gyro_offset_y = 0.0;
double gyro_offset_z = 0.0;

double mag_offset_x = 0.0;
double mag_offset_y = 0.0;
double mag_offset_z = 0.0;

double mag_scale_x = 1.0;
double mag_scale_y = 1.0;
double mag_scale_z = 1.0;

ros::Publisher imu_pub;
ros::Publisher mag_pub;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    sensor_msgs::Imu imu_data = *msg;

    // 가속도계 보정
    imu_data.linear_acceleration.x = (imu_data.linear_acceleration.x + accel_offset_x);
    imu_data.linear_acceleration.y = (imu_data.linear_acceleration.y + accel_offset_y);
    imu_data.linear_acceleration.z = (imu_data.linear_acceleration.z + accel_offset_z);

    // 자이로스코프 보정
    imu_data.angular_velocity.x = (imu_data.angular_velocity.x + gyro_offset_x);
    imu_data.angular_velocity.y = (imu_data.angular_velocity.y + gyro_offset_y);
    imu_data.angular_velocity.z = (imu_data.angular_velocity.z + gyro_offset_z);

    // 보정된 IMU 데이터 퍼블리시
    imu_pub.publish(imu_data);
}

void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg)
{
    sensor_msgs::MagneticField mag_data = *msg;

    // 자기장 센서 보정
    mag_data.magnetic_field.x = (mag_data.magnetic_field.x + mag_offset_x) * mag_scale_x;
    mag_data.magnetic_field.y = (mag_data.magnetic_field.y + mag_offset_y) * mag_scale_y;
    mag_data.magnetic_field.z = (mag_data.magnetic_field.z + mag_offset_z) * mag_scale_z;

    // 보정된 자기장 데이터 퍼블리시
    mag_pub.publish(mag_data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_calibration");
    ros::NodeHandle nh;

    imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data_calibrated", 1);
    mag_pub = nh.advertise<sensor_msgs::MagneticField>("imu/mag_calibrated", 1);

    ros::Subscriber imu_sub = nh.subscribe("imu/data", 1, imuCallback);
    ros::Subscriber mag_sub = nh.subscribe("imu/mag", 1, magCallback);

    ros::spin();

    return 0;
}
