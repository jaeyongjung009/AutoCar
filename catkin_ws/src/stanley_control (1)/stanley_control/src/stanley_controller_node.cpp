#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Float32MultiArray.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>

double wheel_base = 0.75;
double steer_angle;
std::vector<double> cte_list;
std::vector<double> cte_term_list;
std::vector<double> angular_velocity_list;
std::vector<double> heading_error_term_list;
std::vector<double> feedforward_term_list;
double steering_vel_constraint = 150; // 핸들의 조향속도를 제한
double max_rad = 0.2618; // 최대 조향각을 라디안으로 제한 (15도)
double min_rad = -0.2618;
double soft_term = 2.0; // 속도가 낮을 때 조향 불안정성을 완화하기 위한 값
double velocity = 0.0;
double p_gain = 0.2; // CTE에 대한 비례 이득
double h_gain = 0.3; // 헤딩 에러에 대한 비례 이득
double f_gain = 0.3; // 피드포워드 제어 이득
double dt = 0.025; // 시간 간격
double prev_delta = 0.0; // 이전 조향각
double target_velocity_ = 1.0; // 목표속도 임의 지정
ros::Publisher goal_angle_pub_;
ros::Publisher goal_velocity_pub_;

// StanleyController 관련 변수
nav_msgs::Path path_;
double x_, y_, yaw_, v_;
double k_ = 0.5;  // Stanley 제어 게인
double max_steer_ = M_PI / 12;  // 최대 조향각 (15도)

// TF listener
tf::TransformListener* listener_;

// current_velocity를 구독할 때 사용하는 콜백 함수
void velocityCallback(const std_msgs::Float64::ConstPtr& msg) {
    velocity = msg->data;
    v_ = msg->data * 0.27778;  // km/h -> m/s 변환
}

// planned_path 구독할 때 사용하는 콜백 함수
void pathCallbackStanley(const nav_msgs::Path::ConstPtr& msg) {
    path_ = *msg;
}

double calculateSteeringAngle()
{
    // 현재 위치에서 가장 가까운 경로점 찾기
    double min_dist = std::numeric_limits<double>::max();
    int target_index = 0;

    for (int i = 0; i < path_.poses.size(); ++i)
    {   
        double dx = path_.poses[i].pose.position.x - x_;
        double dy = path_.poses[i].pose.position.y - y_;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < min_dist)
        {
            min_dist = dist;
            target_index = i;
        }
    }

    // 경로와의 횡오차 계산
    double dx = path_.poses[target_index].pose.position.x - x_;
    double dy = path_.poses[target_index].pose.position.y - y_;
    double path_yaw = std::atan2(dy, dx);
    double cross_track_error = std::sin(path_yaw - yaw_) * min_dist;

    // 스탠리 알고리즘 적용하여 조향각 계산
    double steering = path_yaw - yaw_ + std::atan2(k_ * cross_track_error, v_);

    // 조향각을 최대 조향각으로 제한
    steering = std::max(-max_steer_, std::min(max_steer_, steering));

    return steering;
}

void errorCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    double a3 = msg->data[0];
    double a2 = msg->data[1];
    double a1 = msg->data[2];
    double a0 = msg->data[3];

    // 경로 곡률을 이용한 피드포워드 제어
    double n = 3 * a3 * pow(2.5, 2) + 2 * a2 * 2.5 + a1;
    double n_2 = 6 * a3 * 2.5 + 2 * a2;
    double radius = ((pow(n, 2) + 1) * sqrt(pow(n, 2) + 1)) / n_2;
    double feedforward_term = f_gain * atan(wheel_base / radius);
    feedforward_term_list.push_back(feedforward_term * 180 / M_PI); // 단위 변환

    // 주행 경로에서의 CTE 계산
    double cte = a3 * pow(7, 3) + a2 * pow(7, 2) + a1 * 7 + a0;
    cte = std::max(std::min(cte, 3.0), -3.0); // CTE를 -3에서 3 사이로 제한
    cte_list.push_back(cte);

    // CTE에 대한 조향 명령 계산
    double crosstrack_error_term = p_gain * atan((p_gain * cte) / (velocity + soft_term));
    cte_term_list.push_back(crosstrack_error_term * 180 / M_PI); // 단위 변환

    // 헤딩 에러 계산
    double heading_error_term = h_gain * atan(n);
    heading_error_term_list.push_back(heading_error_term * 180 / M_PI); // 단위 변환

    // 최종 조향각 계산 및 제한 적용
    double delta = crosstrack_error_term + heading_error_term + feedforward_term;

    // 스탠리 알고리즘에 의한 조향각 계산
    if (!path_.poses.empty()) {
        tf::StampedTransform transform;
        try {
            listener_->lookupTransform("map", "gps_footprint", ros::Time(0), transform);
        } catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            return;
        }

        // gps_footprint_pose를 x_, y_, yaw_에 대입
        x_ = transform.getOrigin().x();
        y_ = transform.getOrigin().y();
        yaw_ = tf::getYaw(transform.getRotation());

        // Stanley 제어 계산
        double stanley_steering_angle = calculateSteeringAngle();
        delta += stanley_steering_angle;
    }

    delta = std::max(std::min(delta, max_rad), min_rad); // 최대/최소 조향각 제한

    // 조향속도 제어
    double delta_rate = (delta - prev_delta) * 180 / M_PI / dt; // 단위 변환 후 조향각 변화율 계산
    if (delta_rate > steering_vel_constraint) { // 최대 조향 속도 제한
        delta = prev_delta + steering_vel_constraint * dt * M_PI / 180;
    } else if (delta_rate < -steering_vel_constraint) {
        delta = prev_delta - steering_vel_constraint * dt * M_PI / 180;
    }

    // 조향각 및 각속도 저장
    steer_angle = delta * 180 / M_PI; // 단위 변환 후 저장
    angular_velocity_list.push_back(delta_rate);

    // 이전 조향각 업데이트
    prev_delta = delta;

    // Ackermann 제어 메시지 생성 및 퍼블리시
    std_msgs::Float64 goal_angle_msg;
    goal_angle_msg.data = steer_angle;
    goal_angle_pub_.publish(goal_angle_msg);

   // 목표 속도 퍼블리시
    std_msgs::Float64 goal_velocity_msg;
    goal_velocity_msg.data = target_velocity_;
    goal_velocity_pub_.publish(goal_velocity_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "stanley_controller_node");
    ros::NodeHandle nh;

    goal_angle_pub_ = nh.advertise<std_msgs::Float64>("/goal_angle", 10);
    goal_velocity_pub_ = nh.advertise<std_msgs::Float64>("/goal_velocity", 10);

    // coefficients와 current_velocity 구독
    ros::Subscriber sub_coeff = nh.subscribe("coefficients", 10, errorCallback);
    ros::Subscriber sub_velocity = nh.subscribe("current_velocity", 10, velocityCallback);

    // Stanley controller에서 사용할 planned_path와 TF 구독 설정
    ros::Subscriber sub_path = nh.subscribe("/planned_path", 10, pathCallbackStanley);
    listener_ = new tf::TransformListener();

    ros::spin();
    delete listener_;
    return 0;
}
