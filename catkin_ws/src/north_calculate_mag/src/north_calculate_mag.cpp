#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>
#include <cmath> // for atan2 and M_PI

class MagneticFieldProcessor
{
public:
    MagneticFieldProcessor()
    {
        // Initialize the subscriber
        mag_subscriber_ = nh_.subscribe("imu/mag_calibrated", 1, &MagneticFieldProcessor::magCallback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber mag_subscriber_;

    double m_x, m_y, m_z; // Initial reference magnetic field values
    bool initialized = false;

    const double magnetic_declination = 8.0; // 한국의 자북 편차(대략적)

    void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg)
    {
        // Extract magnetic field data
        double n_x = msg->magnetic_field.x;
        double n_y = msg->magnetic_field.y;
        double n_z = msg->magnetic_field.z;

        if (!initialized)
        {
            // Initialize the reference point (fixed world frame reference)
            m_x = n_x;
            m_y = n_y;
            m_z = n_z;
            initialized = true;
            ROS_INFO("Reference point initialized.");
            return;
        }

        // Calculate angles using atan2
        double a = atan2(m_z, m_x);
        double b = atan2(n_z, n_x);

        // Calculate Yaw drift
        double yaw_drift = (a - b) * (180.0 / M_PI); // Convert to degrees

        // Calculate current heading (north direction)
        double heading = atan2(n_y, n_x); // in radians
        double heading_degrees = heading * (180.0 / M_PI); // convert to degrees

        // Adjust the heading with yaw drift and magnetic declination
        double adjusted_heading = heading_degrees + yaw_drift + magnetic_declination;

        // Normalize the heading to be within 0-360 degrees
        if (adjusted_heading < 0)
            adjusted_heading += 360.0;
        else if (adjusted_heading >= 360.0)
            adjusted_heading -= 360.0;

        ROS_INFO("Magnetic Field Data: X: %f, Y: %f, Z: %f", n_x, n_y, n_z);
        ROS_INFO("Heading (degrees): %f", adjusted_heading);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "north_calculate_mag");

    MagneticFieldProcessor processor;

    ros::spin();

    return 0;
}