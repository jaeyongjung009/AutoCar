#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <geodesy/utm.h>
#include <array>

class IMUGPSEncoderOdom
{
public:
    IMUGPSEncoderOdom()
    {
        imu_sub_ = nh_.subscribe("imu/data", 10, &IMUGPSEncoderOdom::imuCallback, this);
        encoder_sub_ = nh_.subscribe("/encoder_count", 10, &IMUGPSEncoderOdom::encoderCallback, this);
        gps_sub_ = nh_.subscribe("/ublox_gps/rtcm", 10, &IMUGPSEncoderOdom::gpsCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/vehicle_path", 10);

        path_.header.frame_id = "map";
        prev_encoder_count_ = 0;
        position_ = {0.0, 0.0};
        orientation_ = 0.0;
        prev_time_ = ros::Time::now();

        // Timer to periodically publish the path
        timer_ = nh_.createTimer(ros::Duration(0.1), &IMUGPSEncoderOdom::timerCallback, this);
    }

private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        tf::Quaternion quat(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        orientation_ = yaw;
    }

    void encoderCallback(const std_msgs::Int32::ConstPtr& msg)
    {
        int current_encoder_count = msg->data;
        int delta_count = current_encoder_count - prev_encoder_count_;
        prev_encoder_count_ = current_encoder_count;

        // Example wheel radius (27.6 cm diameter -> 0.138 m radius)
        double wheel_radius = 0.138; // meters
        
        // Calculate speed in m/s
        double velocity_m_per_s = delta_count * wheel_radius;

        ros::Duration delta_time = ros::Time::now() - prev_time_;
        prev_time_ = ros::Time::now();

        if (delta_time.toSec() > 0)
        {
            double distance = velocity_m_per_s * delta_time.toSec();
            double delta_x = distance * cos(orientation_);
            double delta_y = distance * sin(orientation_);
            position_[0] += delta_x;
            position_[1] += delta_y;
        }
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        if (msg->status.status != sensor_msgs::NavSatStatus::STATUS_NO_FIX)
        {
            // Convert GPS coordinates to UTM
            geographic_msgs::GeoPoint geo_pt;
            geo_pt.latitude = msg->latitude;
            geo_pt.longitude = msg->longitude;
            geo_pt.altitude = msg->altitude;

            geodesy::UTMPoint utm_pt;
            geodesy::fromMsg(geo_pt, utm_pt);

            position_[0] = utm_pt.easting;
            position_[1] = utm_pt.northing;
        }
    }

    void timerCallback(const ros::TimerEvent&)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.position.x = position_[0];
        pose.pose.position.y = position_[1];
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(orientation_);

        path_.header.stamp = pose.header.stamp;
        path_.poses.push_back(pose);

        path_pub_.publish(path_);
    }

    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Subscriber encoder_sub_;
    ros::Subscriber gps_sub_;
    ros::Publisher path_pub_;
    ros::Timer timer_;

    nav_msgs::Path path_;
    int prev_encoder_count_;
    std::array<double, 2> position_;
    double orientation_;
    ros::Time prev_time_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_encoder_gps_odom");
    IMUGPSEncoderOdom estimator;
    ros::spin();
    return 0;
}
