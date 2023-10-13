#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std;
using std::placeholders::_1;



class OdomPublisher:public rclcpp ::Node{
    // 订阅者
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    // 发布者
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr head_publisher_;

    // 创建tf广播
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 变量初始化
    // 速度系数
    double linear_scale_x_ = 0.0;
    double linear_scale_y_ = 0.0;
    double angular_scale_z_ = 0.0;
    // 时间
    double vel_dt_ = 0.0;
    // 位置
    double x_pos_ = 0.0;
    double y_pos_ = 0.0;
    // 朝向
    double heading_ = 0.0;
    // 速度
    double linear_velocity_x_ = 0.0;
    double linear_velocity_y_ = 0.0;
    double angular_velocity_z_ = 0.0;
    // 是否发布tf广播
    bool pub_odom_tf_ = true;
    
    bool pub_head_ = true;
    // 上个时间初始化
    rclcpp::Time last_vel_time_  ;

    public:
        // 类的构造函数
        OdomPublisher(): Node("odom_data"){
            // 参数创建           
            this->declare_parameter<double>("linear_scale_x",1.0);
            this->declare_parameter<double>("linear_scale_y",1.0);
            this->declare_parameter<double>("angular_scale_z",1.2);
            this->declare_parameter<bool>("pub_odom_tf",false);
            this->declare_parameter<bool>("pub_head",true);

            this->get_parameter<double>("linear_scale_x",linear_scale_x_);
            this->get_parameter<double>("linear_scale_y",linear_scale_y_);
            this->get_parameter<double>("angular_scale_z",angular_scale_z_);
            this->get_parameter<bool>("pub_odom_tf",pub_odom_tf_);
            this->get_parameter<bool>("pub_head",pub_head_);

            // 创建tf实例
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            // 订阅者实例
            subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("twist_cmd",50,std::bind(&OdomPublisher::twist_to_odom,this,_1));
            // 发布者实例
            odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_data", 50);
            head_publisher_ = this->create_publisher<std_msgs::msg::Float64>("head_val", 50);
        }
    private:
        void twist_to_odom(const std::shared_ptr<geometry_msgs::msg::Twist > msg){
            // 速度计算
            linear_velocity_x_ = msg->linear.x * linear_scale_x_;
    		linear_velocity_y_ = msg->linear.y * linear_scale_y_;
    		angular_velocity_z_ = msg->angular.z * angular_scale_z_;
            // 时间计算(当前时间:curren_time,上个时间:last_vel_time_ )
            rclcpp::Time curren_time = rclcpp::Clock().now();
            vel_dt_ = (curren_time - last_vel_time_).seconds();
            last_vel_time_ = curren_time;
            // 位置的微分 = 速度 * 时间的微分
            double delta_heading = angular_velocity_z_ * vel_dt_ * 3.14; //radians
    		double delta_x = (linear_velocity_x_ * cos(heading_)-linear_velocity_y_*sin(heading_)) * vel_dt_; //m
    		double delta_y = (linear_velocity_x_ * sin(heading_)+linear_velocity_y_*cos(heading_)) * vel_dt_; //m	
 			// 更新位置
            x_pos_ += delta_x;
    		y_pos_ += delta_y;
			heading_ += delta_heading;
            // 四元数创建(四元数 和 欧拉角 可相互转换，用于表示空间角度信息)
            tf2::Quaternion myQuaternion;
			geometry_msgs::msg::Quaternion odom_quat; 
			myQuaternion.setRPY(0.00,0.00,heading_ );
			// 里程计创建
			nav_msgs::msg::Odometry odom;
			odom.header.stamp = curren_time;
			odom.header.frame_id = "odom";
			odom.child_frame_id = "base_footprint";
            // 里程计odom添加四元数
            odom_quat.x = myQuaternion.x();
            odom_quat.y = myQuaternion.y();
            odom_quat.z = myQuaternion.z();
            odom_quat.w = myQuaternion.w();
            odom.pose.pose.orientation = odom_quat;
            // 里程计odom添加位置
			odom.pose.pose.position.x = x_pos_;
			odom.pose.pose.position.y = y_pos_;
			odom.pose.pose.position.z = 0.0;
            // 里程计odom位置协方差矩阵初始化，便于卡尔曼滤波
			odom.pose.covariance[0] = 0.001;
			odom.pose.covariance[7] = 0.001;
			odom.pose.covariance[35] = 0.001;
            // 里程计odom添加线速度
			odom.twist.twist.linear.x = linear_velocity_x_;
			odom.twist.twist.linear.y = linear_velocity_y_;
			odom.twist.twist.linear.z = 0.0;
			// 里程计odom添加角速度
			odom.twist.twist.angular.x = 0.0;
			odom.twist.twist.angular.y = 0.0;
			odom.twist.twist.angular.z = angular_velocity_z_;
            // 里程计odom速度协方差矩阵初始化，便于卡尔曼滤波
			odom.twist.covariance[0] = 0.0001;
			odom.twist.covariance[7] = 0.0001;
			odom.twist.covariance[35] = 0.0001;
            // 里程计发布;	
			odom_publisher_ -> publish(odom);
            // 判断是否发布tf广播
            if (pub_odom_tf_)
            {
                geometry_msgs::msg::TransformStamped tf;
                rclcpp::Time now = this->get_clock()->now();
                tf.header.stamp = now;
                tf.header.frame_id = "odom";
                tf.child_frame_id = "base_footprint";
                // tf广播位置
                tf.transform.translation.x = x_pos_;
                tf.transform.translation.y = y_pos_;
                tf.transform.translation.z = 0.0;
                // tf广播四元数
                tf.transform.rotation.x = myQuaternion.x();
                tf.transform.rotation.y = myQuaternion.y();
                tf.transform.rotation.z = myQuaternion.z();
                tf.transform.rotation.w = myQuaternion.w();
                // tf广播发送
                tf_broadcaster_->sendTransform(tf);
            }
            if (pub_head_)
            {
                auto head_val_ = std_msgs::msg::Float64();
                head_val_.data = heading_;
                head_publisher_->publish(head_val_);
            }
        }
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OdomPublisher>());
	rclcpp::shutdown();
    return 0;
}