#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/debug_vect.hpp>
#include <px4_msgs/msg/vehicle_mocap_odometry.hpp>
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>
#include <px4_msgs/msg/timesync.hpp>

using namespace std::chrono_literals;

class VehicleMocapOdometryAdvertiser : public rclcpp::Node
{
public:
	VehicleMocapOdometryAdvertiser() : Node("vehicle_mocap_odometry_advertiser") {
#ifdef ROS_DEFAULT_API
		publisher_ = this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>("VehicleVisualOdometry_PubSubTopic", 10);
#else
		publisher_ = this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>("VehicleVisualOdometry_PubSubTopic");
#endif
		auto timer_callback =
		[this]()->void {
			auto vehicle_mocap_odometry = px4_msgs::msg::VehicleVisualOdometry();
			vehicle_mocap_odometry.timestamp = timesync_.timestamp; //std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
			vehicle_mocap_odometry.timestamp_sample = timesync_.timestamp;

			vehicle_mocap_odometry.local_frame = 0;
			vehicle_mocap_odometry.x = 5.0;
			vehicle_mocap_odometry.y = 0.0;
			vehicle_mocap_odometry.z = -1.0;
			vehicle_mocap_odometry.q = {1.0, 0.0, 0.0, 0.0};
			
			this->publisher_->publish(vehicle_mocap_odometry);
			std::cout << "============================="   << std::endl;
			std::cout << "TimeSync timestamp: " << timesync_.timestamp << std::endl;
		};
		
		subscription_ = this->create_subscription<px4_msgs::msg::Timesync>(
			"/Timesync_PubSubTopic", 10,
			[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
			timesync_.timestamp = msg->timestamp;
		});
		std::cout << "============================="   << std::endl;
		std::cout << "TimeSync timestamp: " << timesync_.timestamp << std::endl;
		timer_ = this->create_wall_timer(30ms, timer_callback);
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr publisher_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr subscription_;
	px4_msgs::msg::Timesync timesync_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting vehicle_mocap_odometry advertiser node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VehicleMocapOdometryAdvertiser>());

	rclcpp::shutdown();
	return 0;
}