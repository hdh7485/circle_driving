#define _USE_MATH_DEFINES // for C++  
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <cmath>

class CircleDriving{
	private:
		ros::NodeHandle nh_;
		ros::Subscriber sub_imu_;
		ros::Subscriber sub_speed_;
		ros::Publisher twist_pub_;

		float target_circle_radius_;
		float measured_circle_radius_;
		float target_speed_;
		float measured_speed_;
		float angular_v_z_radian_;
		float angular_v_z_degree_;
		float wheelbase_;
		float steer_standard_;
		float steer_output_;
		float KP_;
		float KI_;
		float error_;
		float error_sum_;

		geometry_msgs::Twist twist_msg_;
	
	public:
		CircleDriving(){
			sub_imu_ = nh_.subscribe< sensor_msgs::Imu >("imu/data", 10, &CircleDriving::imuCallback, this);
			sub_speed_ = nh_.subscribe< std_msgs::Float32 >("front_encoder", 10, &CircleDriving::speedCallback, this);
			twist_pub_ = nh_.advertise < geometry_msgs::Twist > ("twist_msg", 10);

			measured_speed_ = 0;
			target_speed_ = 1.5;
			wheelbase_ = 0.257;
			//target_circle_radius_ = 3.5;
			target_circle_radius_ = 3.1;
			//steer_standard_ = -4.1996;
			steer_standard_ = -3.8;
			KP_ = 0.1;
			KI_ = 0.008;
		}

		float radian_to_angular_velocity(float radian){
			return radian * 180 / M_PI;
		}

		void calculate_velocity(){
			//speed_ = circle_radius_ * angular_v_z_degree_;
			if (angular_v_z_radian_ >= 0.001){
				measured_circle_radius_ = measured_speed_ / angular_v_z_radian_;
				error_ = target_circle_radius_ - measured_circle_radius_;
				error_sum_ += error_;
			}
			else{
				error_ = 0;
				error_sum_ = 0;
			}
			/*
			if(error_sum_ > 10) error_sum_ = 100;
			if(error_sum_ < -10) error_sum_ = -100;
			*/
			steer_output_ = steer_standard_ + error_ * KP_ + error_sum_ * KI_;
			//steer_output_ = steer_standard_ + error_ * KP_;
			ROS_INFO("Sensor degree Error: %f", error_);
			ROS_INFO("Sensor radian: %f", angular_v_z_radian_);
			//ROS_INFO("Measured circle radius: %f", measured_circle_radius_);
			ROS_INFO("Encoder velocity: %f", measured_speed_);
			ROS_INFO("Output steer degree Z: %f", steer_output_);
		}

		void speedCallback(const std_msgs::Float32::ConstPtr& speedMsg){
			measured_speed_ = speedMsg->data;
		}
		void imuCallback(const sensor_msgs::Imu::ConstPtr& imuMsg){
			angular_v_z_radian_ = imuMsg->angular_velocity.z;
			angular_v_z_degree_ = radian_to_angular_velocity(angular_v_z_radian_);

			calculate_velocity();

			twist_msg_.linear.x = target_speed_;
			twist_msg_.angular.z = steer_output_;
			twist_pub_.publish(twist_msg_);
		}	
};

int main(int argc, char **argv){
	ros::init(argc, argv, "circle_driving_node");
	CircleDriving circle_driving;
	ros::spin();
}


