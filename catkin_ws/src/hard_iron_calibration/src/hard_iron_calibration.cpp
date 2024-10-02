#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>
#include <vector>
#include <limits>

class HardIronCalibration
{
public:
    HardIronCalibration()
    {
        // Initialize the subscriber
        mag_subscriber_ = nh_.subscribe("imu/mag", 1000, &HardIronCalibration::magCallback, this);
    }

    void calibrate()
    {
        if (mag_data_x_.empty() || mag_data_y_.empty() || mag_data_z_.empty())
        {
            ROS_WARN("No magnetic field data collected for calibration.");
            return;
        }

        // Calculate min and max for each axis
        double x_min = *std::min_element(mag_data_x_.begin(), mag_data_x_.end());
        double x_max = *std::max_element(mag_data_x_.begin(), mag_data_x_.end());
        double y_min = *std::min_element(mag_data_y_.begin(), mag_data_y_.end());
        double y_max = *std::max_element(mag_data_y_.begin(), mag_data_y_.end());
        double z_min = *std::min_element(mag_data_z_.begin(), mag_data_z_.end());
        double z_max = *std::max_element(mag_data_z_.begin(), mag_data_z_.end());

        // Calculate offsets
        x_offset_ = (x_max + x_min) / 2.0;
        y_offset_ = (y_max + y_min) / 2.0;
        z_offset_ = (z_max + z_min) / 2.0;

        ROS_INFO("Calibration complete.");
        ROS_INFO("X offset: %f", x_offset_);
        ROS_INFO("Y offset: %f", y_offset_);
        ROS_INFO("Z offset: %f", z_offset_);
    }

    // 보정된 자기장 데이터를 반환하는 함수
    sensor_msgs::MagneticField getCalibratedData(const sensor_msgs::MagneticField& raw_data)
    {
        sensor_msgs::MagneticField calibrated_data = raw_data;
        calibrated_data.magnetic_field.x -= x_offset_;
        calibrated_data.magnetic_field.y -= y_offset_;
        calibrated_data.magnetic_field.z -= z_offset_;
        return calibrated_data;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber mag_subscriber_;
    std::vector<double> mag_data_x_, mag_data_y_, mag_data_z_;
    double x_offset_ = 0.0;
    double y_offset_ = 0.0;
    double z_offset_ = 0.0;

    void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg)
    {
        // Collect magnetic field data
        mag_data_x_.push_back(msg->magnetic_field.x);
        mag_data_y_.push_back(msg->magnetic_field.y);
        mag_data_z_.push_back(msg->magnetic_field.z);
        ROS_INFO("Magnetic Field Data Collected: X: %f, Y: %f, Z: %f",
                 msg->magnetic_field.x, msg->magnetic_field.y, msg->magnetic_field.z);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hard_iron_calibration");
    HardIronCalibration calibrator;

    ROS_INFO("Collecting magnetic field data for calibration...");

    // 데이터 수집을 위한 30초 루프
    ros::Time start_time = ros::Time::now();
    ros::Rate rate(10); // 10 Hz로 루프를 돌립니다.
    
    while (ros::ok())
    {
        ros::spinOnce();

        // 30초 후 보정을 수행
        if (ros::Time::now() - start_time > ros::Duration(30.0))
        {
            ROS_INFO("Starting calibration...");
            calibrator.calibrate();
            break;  // 보정 후 루프를 종료합니다.
        }

        rate.sleep();
    }

    // 여기서부터 보정된 데이터를 사용할 수 있습니다
    ROS_INFO("Calibration completed. Now running calibrated data processing...");

    ros::spin();  // 계속해서 ROS 콜백을 처리
    return 0;
}
