#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <deque>

class IMUDataFilter
{
public:
    IMUDataFilter(ros::NodeHandle& nh, int window_size)
        : window_size_(window_size), imu_data_queue_(window_size), mag_data_queue_(window_size)
    {
        // Subscriber와 Publisher 초기화
        imu_sub_ = nh.subscribe("/imu/data", 10, &IMUDataFilter::imuCallback, this);
        mag_sub_ = nh.subscribe("/imu/mag", 10, &IMUDataFilter::magCallback, this);

        filtered_imu_pub_ = nh.advertise<sensor_msgs::Imu>("/imu/filtered_data", 10);
        filtered_mag_pub_ = nh.advertise<sensor_msgs::MagneticField>("/imu/filtered_mag_data", 10);
    }

private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        // IMU 데이터 큐에 추가
        imu_data_queue_.push_back(*msg);

        // 큐가 윈도우 크기를 초과하면 가장 오래된 데이터를 제거
        if (imu_data_queue_.size() > window_size_)
        {
            imu_data_queue_.pop_front();
        }

        // 데이터가 충분히 쌓였을 때 필터링을 수행
        if (imu_data_queue_.size() == window_size_)
        {
            sensor_msgs::Imu filtered_imu = calculateMovingAverageIMU();
            filtered_imu.header = msg->header;  // 원본 메시지의 헤더 사용
            filtered_imu_pub_.publish(filtered_imu);
        }
    }

    void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg)
    {
        // 자기장 데이터 큐에 추가
        mag_data_queue_.push_back(*msg);

        // 큐가 윈도우 크기를 초과하면 가장 오래된 데이터를 제거
        if (mag_data_queue_.size() > window_size_)
        {
            mag_data_queue_.pop_front();
        }

        // 데이터가 충분히 쌓였을 때 필터링을 수행
        if (mag_data_queue_.size() == window_size_)
        {
            sensor_msgs::MagneticField filtered_mag = calculateMovingAverageMag();
            filtered_mag.header = msg->header;  // 원본 메시지의 헤더 사용
            filtered_mag_pub_.publish(filtered_mag);
        }
    }

    sensor_msgs::Imu calculateMovingAverageIMU()
    {
        sensor_msgs::Imu avg_msg;
        // 초기화
        avg_msg.orientation.x = avg_msg.orientation.y = avg_msg.orientation.z = avg_msg.orientation.w = 0.0;
        avg_msg.angular_velocity.x = avg_msg.angular_velocity.y = avg_msg.angular_velocity.z = 0.0;
        avg_msg.linear_acceleration.x = avg_msg.linear_acceleration.y = avg_msg.linear_acceleration.z = 0.0;

        // 이동 평균 계산
        for (const auto& data : imu_data_queue_)
        {
            avg_msg.orientation.x += data.orientation.x;
            avg_msg.orientation.y += data.orientation.y;
            avg_msg.orientation.z += data.orientation.z;
            avg_msg.orientation.w += data.orientation.w;

            avg_msg.angular_velocity.x += data.angular_velocity.x;
            avg_msg.angular_velocity.y += data.angular_velocity.y;
            avg_msg.angular_velocity.z += data.angular_velocity.z;

            avg_msg.linear_acceleration.x += data.linear_acceleration.x;
            avg_msg.linear_acceleration.y += data.linear_acceleration.y;
            avg_msg.linear_acceleration.z += data.linear_acceleration.z;
        }

        // 평균값 계산
        avg_msg.orientation.x /= window_size_;
        avg_msg.orientation.y /= window_size_;
        avg_msg.orientation.z /= window_size_;
        avg_msg.orientation.w /= window_size_;

        avg_msg.angular_velocity.x /= window_size_;
        avg_msg.angular_velocity.y /= window_size_;
        avg_msg.angular_velocity.z /= window_size_;

        avg_msg.linear_acceleration.x /= window_size_;
        avg_msg.linear_acceleration.y /= window_size_;
        avg_msg.linear_acceleration.z /= window_size_;

        return avg_msg;
    }

    sensor_msgs::MagneticField calculateMovingAverageMag()
    {
        sensor_msgs::MagneticField avg_msg;
        // 초기화
        avg_msg.magnetic_field.x = avg_msg.magnetic_field.y = avg_msg.magnetic_field.z = 0.0;

        // 이동 평균 계산
        for (const auto& data : mag_data_queue_)
        {
            avg_msg.magnetic_field.x += data.magnetic_field.x;
            avg_msg.magnetic_field.y += data.magnetic_field.y;
            avg_msg.magnetic_field.z += data.magnetic_field.z;
        }

        // 평균값 계산
        avg_msg.magnetic_field.x /= window_size_;
        avg_msg.magnetic_field.y /= window_size_;
        avg_msg.magnetic_field.z /= window_size_;

        return avg_msg;
    }

    ros::Subscriber imu_sub_;
    ros::Subscriber mag_sub_;
    ros::Publisher filtered_imu_pub_;
    ros::Publisher filtered_mag_pub_;
    std::deque<sensor_msgs::Imu> imu_data_queue_;
    std::deque<sensor_msgs::MagneticField> mag_data_queue_;
    int window_size_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_moving_ave_filter_node");
    ros::NodeHandle nh;

    int window_size = 10;  // 윈도우 크기 조절
    if (nh.hasParam("window_size"))
    {
        nh.getParam("window_size", window_size);
    }

    IMUDataFilter imu_filter(nh, window_size);

    ros::spin();

    return 0;
}
