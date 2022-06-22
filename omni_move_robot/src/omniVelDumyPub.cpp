//-----------------------------
// include
//-----------------------------
// STL
#include <iostream>
#include <vector>
#include <string>
#include <cmath>

// ROS2 library
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

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
class OmniVelDumyPub: public rclcpp::Node {
public:
    OmniVelDumyPub(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~OmniVelDumyPub();
private:
    void Execute();
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdRobotVel;
};

/**
 * @brief Construct a new class object
 * 
 * @param options 
 */
OmniVelDumyPub::OmniVelDumyPub(rclcpp::NodeOptions options) : rclcpp::Node("OmniVelDumyPub", options)
{
    // create topic
    cmdRobotVel = this->create_publisher<geometry_msgs::msg::Twist>("OmniMoveRobot/CmdRobotVel", 10);

    // create thread
    std::thread{
        std::bind(&OmniVelDumyPub::Execute, this)
    }.detach();
}

/**
 * @brief Destroy the class object
 * 
 */
OmniVelDumyPub::~OmniVelDumyPub()
{
    
}

/**
 * @brief Execute method
 * 
 */
void OmniVelDumyPub::Execute()
{
    geometry_msgs::msg::Twist vel;

    // Main process loop setting
    rclcpp::WallRate loop(1);		// loop freq.[Hz]

    // Main loop
    while(rclcpp::ok()){
        //RCLCPP_INFO(this->get_logger(), "%s", "Hello world!");
        vel.linear.x += 1;
        vel.linear.y += 2;
        vel.linear.z += 1.5;
        cmdRobotVel->publish(vel);
        loop.sleep();
    }
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(OmniMoveRobot::OmniVelDumyPub)

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------