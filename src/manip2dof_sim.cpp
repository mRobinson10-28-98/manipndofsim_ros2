#include <array>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "ManipNDof.hpp"

using std::placeholders::_1;

class Manip2Dof_Sim : public rclcpp::Node
{
public:
    Manip2Dof_Sim() : Node("Manip2Dof_Sim") 
    {
        mJointPosSub = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "Manip2Dof_JointPos_Command", 10, std::bind(&Manip2Dof_Sim::position_command_callback, this, _1));
    }

private:
    void position_command_callback(const geometry_msgs::msg::Vector3Stamped& msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s", std::to_string(msg.vector.x).c_str());
    }

    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr mJointPosSub;

};
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Manip2Dof_Sim>());
  rclcpp::shutdown();
  return 0;
}