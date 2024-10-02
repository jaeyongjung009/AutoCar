#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <Eigen/Dense>

ros::Publisher pub_coefficients;
ros::Publisher pub_path;

// 기존의 pathCallback 함수
void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    std::vector<double> x;
    std::vector<double> y; // 각각 경로의 x좌표와 y좌표를 저장함
    std::vector<double> new_y;
    int number = 20;
 
    for (const auto& pose : msg->poses) { // 메시지에서 모든 pose의 position.x 와 position.y 를 읽어 x와 y벡터에 저장
        x.push_back(pose.pose.position.x); 
        y.push_back(pose.pose.position.y); 
    }

    // << 백터타입의 x y를 에이겐 벡터 타입으로 변환하는 과정 >>
    Eigen::VectorXd x_vec(x.size());
    Eigen::VectorXd y_vec(y.size()); // 벡터의 크기를 구한 후 x_vec과 y_vec이라는 두개의 벡터를 생성
    for (size_t i = 0; i < x.size(); ++i) {
        x_vec[i] = x[i];
        y_vec[i] = y[i]; // for문을 통해 에이겐의 행렬 연산에서 사용하도록 함
    }

    // << 다항 회귀를 위한 디자인 행렬을 설정 >> 
    Eigen::MatrixXd X(x.size(), 4); // x.size 행과 4 열을 가진 행렬 X 생성
    for (int i = 0; i < x.size(); ++i) {
        X(i, 0) = 1;
        X(i, 1) = x_vec[i];
        X(i, 2) = x_vec[i] * x_vec[i];
        X(i, 3) = x_vec[i] * x_vec[i] * x_vec[i];
    }

    // << 3차 다항식 회귀의 최적 계수를 구하고, 피팅된 값을 계산하는 과정 >>
    Eigen::VectorXd coefs = (X.transpose() * X).ldlt().solve(X.transpose() * y_vec);
    Eigen::VectorXd ffit(x.size());
    for (size_t i = 0; i < x.size(); ++i) {
        ffit[i] = coefs[0] + coefs[1] * x_vec[i] + coefs[2] * x_vec[i] * x_vec[i] + coefs[3] * x_vec[i] * x_vec[i] * x_vec[i];
    }

    // << 3차 다항식 회귀로 얻은 피팅된 경로를 nav_msgs::Path로 변환 >>
    nav_msgs::Path path;
    path.header.frame_id = "base_link";
    path.header.stamp = ros::Time::now();
    for (size_t i = 0; i < x.size(); ++i) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = path.header.stamp;
        pose.header.frame_id = path.header.frame_id;
        pose.pose.position.x = x[i];
        pose.pose.position.y = ffit[i];
        path.poses.push_back(pose);
    }

    pub_path.publish(path);

    std_msgs::Float32MultiArray coefficients_msg;
    coefficients_msg.data = {coefs[0], coefs[1], coefs[2], coefs[3]};
    pub_coefficients.publish(coefficients_msg);
}

// 추가된 plannedPathCallback 함수
void plannedPathCallback(const nav_msgs::Path::ConstPtr& msg) {
    ROS_INFO("Received path with %lu poses", msg->poses.size());
    for (size_t i = 0; i < msg->poses.size(); ++i) {
        const geometry_msgs::PoseStamped& pose = msg->poses[i];
        ROS_INFO("Pose[%lu]: x=%f, y=%f", i, pose.pose.position.x, pose.pose.position.y);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "polyfitting_node");
    ros::NodeHandle nh;

    pub_coefficients = nh.advertise<std_msgs::Float32MultiArray>("coefficients", 10);
    pub_path = nh.advertise<nav_msgs::Path>("/path/polyfit", 1);

    // 기존의 LocalWaypoint 토픽 구독
    ros::Subscriber sub_path = nh.subscribe("/Path/LocalWaypoint/OnBody/", 10, pathCallback);

    // 추가된 planned_path 토픽 구독
    ros::Subscriber sub_planned_path = nh.subscribe("/planned_path", 10, plannedPathCallback);

    ros::spin();
    return 0;
}
