#define _USE_MATH_DEFINES // for C++  
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <cmath>

class CircleDriving{
	private:
		ros::NodeHandle nh_;
		ros::Subscriber sub_;
		ros::Publisher twist_pub_;

		float target_circle_radius_;
		float measured_circle_radius_;
		float speed_;
		float angular_v_z_radian_;
		float angular_v_z_degree_;
		float wheelbase_;
		float steer_standard_;
		float steer_output_;
		float KP_;
		float error_;

		geometry_msgs::Twist twist_msg_;
	
	public:
		CircleDriving(){
			sub_ = nh_.subscribe<sensor_msgs::Imu>("imu/data", 10, &CircleDriving::imuCallback, this);
			twist_pub_ = nh_.advertise < geometry_msgs::Twist > ("twist_msg", 10);
			speed_ = 3;
			wheelbase_ = 0.257;
			target_circle_radius_ = 2;
			steer_standard_ = -4.2;
			KP_ = -3.0;
		}

		float radian_to_angular_velocity(float radian){
			return radian * 180 / M_PI;
		}

		void calculate_velocity(){
			//speed_ = circle_radius_ * angular_v_z_degree_;
			measured_circle_radius_ = angular_v_z_radian_ / speed_;
			error_ = target_circle_radius_ - measured_circle_radius_;
			steer_output_ = steer_standard_ + error_ * KP_;
			ROS_INFO("Sensor degree Error: %f", error_);
			ROS_INFO("Sensor radian: %f", angular_v_z_radian_);
			ROS_INFO("Measured circle radius: %f", measured_circle_radius_);
		}

		void imuCallback(const sensor_msgs::Imu::ConstPtr& imuMsg){
			angular_v_z_radian_ = imuMsg->angular_velocity.z;
			angular_v_z_degree_ = radian_to_angular_velocity(angular_v_z_radian_);

			calculate_velocity();

			ROS_INFO("Output degree velocity Z: %f", steer_output_);
			twist_msg_.linear.x = speed_;
			twist_msg_.angular.z = steer_output_;
			twist_pub_.publish(twist_msg_);
		}	
};

int main(int argc, char **argv){
	ros::init(argc, argv, "circle_driving_node");
	CircleDriving circle_driving;
	ros::spin();
}


