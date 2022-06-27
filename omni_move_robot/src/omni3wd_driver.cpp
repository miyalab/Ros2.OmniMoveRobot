//-----------------------------
// include
//-----------------------------
// STL
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <thread>
#include <mutex>
#include <chrono>
#include <functional>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Boost
#include <boost/asio.hpp>

//-----------------------------
// Namespace
//-----------------------------


/**
 * @brief Project Name
 * 
 */
namespace OmniMoveRobot{
/**
 * @brief Component Definition
 * 
 */
class Omni3wdRobot: public rclcpp::Node {
public:
    Omni3wdRobot(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~Omni3wdRobot();
private:
    void Execute();

    // robot vel control
	void onSubscriptionCmdRobotVel(const geometry_msgs::msg::Twist::SharedPtr msg);
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdRobotVelSub;
    geometry_msgs::msg::Twist cmdRobotVel;
    std::mutex cmdRobotVelMutex;

    // robot pose
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr robotPosePub;
    nav_msgs::msg::Odometry robotPose;
    std::mutex robotPoseMutex;
};

/**
 * @brief Construct a new class object
 * 
 * @param options 
 */
Omni3wdRobot::Omni3wdRobot(rclcpp::NodeOptions options) : rclcpp::Node("Omni3wdDrive", options)
{
    // using placeholders
    using std::placeholders::_1;
    using std::placeholders::_2;

	// create topic
    cmdRobotVelSub = this->create_subscription<geometry_msgs::msg::Twist>("OmniMoveRobot/CmdRobotVel", 10,
	 		std::bind(&Omni3wdRobot::onSubscriptionCmdRobotVel, this, _1));
	robotPosePub = this->create_publisher<nav_msgs::msg::Odometry>("OmniMoveRobot/RobotPose", 10);

    // create thread
    std::thread{
        std::bind(&Omni3wdRobot::Execute, this)
    }.detach();
}

/**
 * @brief Destroy the class object
 * 
 */
Omni3wdRobot::~Omni3wdRobot()
{

}

/**
 * @brief Execute method
 * 
 */
void Omni3wdRobot::Execute()
{
    // serial port setting
    using namespace boost::asio;
    // io_service io;
    // serial_port serialDrive(io, "/dev/ttyACM0");
    // serialDrive.set_option(serial_port_base::baud_rate(9600));
    // serialDrive.set_option(serial_port_base::character_size(8));
    // serialDrive.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
    // serialDrive.set_option(serial_port_base::parity(serial_port_base::parity::none));
    // serialDrive.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));

    // Main process loop setting
    rclcpp::WallRate loop(1);		// loop freq.[Hz]

    // Main loop
    while(rclcpp::ok()){
        cmdRobotVelMutex.lock();
        RCLCPP_INFO(this->get_logger(), "(%.2lf, %.2lf, %.2lf)", cmdRobotVel.linear.x, cmdRobotVel.linear.y, cmdRobotVel.linear.z);
        cmdRobotVelMutex.unlock();

        robotPoseMutex.lock();
        robotPosePub->publish(robotPose);
        robotPoseMutex.unlock();
        loop.sleep();
    }

    // close serialport
    //serialDrive.close();
}

/**
 * @brief robot velosity control param. callback method
 * 
 * @param msg 
 */
void Omni3wdRobot::onSubscriptionCmdRobotVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(this->cmdRobotVelMutex);
    cmdRobotVel = *msg;
    //RCLCPP_INFO(this->get_logger(), "(%.2lf, %.2lf, %.2lf)", msg->linear.x, msg->linear.y, msg->linear.z);
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(OmniMoveRobot::Omni3wdRobot)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------