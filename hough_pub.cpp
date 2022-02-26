#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "iii_interfaces/msg/powerline_direction.hpp"

#include <algorithm>
#include <cstdlib>
#include <stdlib.h> 
#include <iostream>   
#include <math.h>  
#include <vector>
#include <string>


#define PI 3.14159265


using namespace std::chrono_literals;

//creates a HoughTFPub class that subclasses the generic rclcpp::Node base class.
class HoughTFPub : public rclcpp::Node
{
	public:
		HoughTFPub() : Node("hough_tf_pub") {

			cable_yaw_publisher_ = this->create_publisher<iii_interfaces::msg::PowerlineDirection>(
				"cable_yaw_angle", 10);
						

			camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
			"/cable_camera/image_raw",	10,
			std::bind(&HoughTFPub::OnCameraMsg, this, std::placeholders::_1));

		}

		~HoughTFPub() {
			RCLCPP_INFO(this->get_logger(),  "Shutting down hough_tf_pub..");
		}


	private:

		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
		rclcpp::Publisher<iii_interfaces::msg::PowerlineDirection>::SharedPtr cable_yaw_publisher_;
		void OnCameraMsg(const sensor_msgs::msg::Image::SharedPtr _msg);
};




// mmwave message callback function
void HoughTFPub::OnCameraMsg(const sensor_msgs::msg::Image::SharedPtr _msg){

	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(_msg, _msg->encoding);
	cv::Mat img = cv_ptr->image;

	cv::Mat edge;
	cv::Canny(img, edge, 50, 200, 3); // edge detection

	// Standard Hough Line Transform
    std::vector<cv::Vec2f> lines; // will hold the results of the detection
    HoughLines(edge, lines, 1, PI/180, 150, 0, 0 ); // runs the actual detection

	float avg_theta = 0.0;

    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
		avg_theta = avg_theta + theta;
    }

	avg_theta = avg_theta / (float)lines.size();
	//RCLCPP_INFO(this->get_logger(),  "Theta avg %f:", avg_theta);

	iii_interfaces::msg::PowerlineDirection pl_msg;
	pl_msg.angle = avg_theta;
	cable_yaw_publisher_->publish(pl_msg);
}

			
int main(int argc, char *argv[])
{
	std::cout << "Starting hough_tf_pub node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<HoughTFPub>());

	rclcpp::shutdown();
	return 0;
}
