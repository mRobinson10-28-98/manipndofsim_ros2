#include <array>
#include <memory>
#include <thread>
#include <chrono>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// From maniptools library
#include "JointControllerSnap.hpp"
#include "ManipNDof.hpp"
#include "TwistJoint.hpp"

using std::placeholders::_1;

void StepSim(std::shared_ptr<ManipNDof> pManip)
{
    bool a = true;
    while (a)
    {
        pManip->StepJointController();
        std::cout << "Hello\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

class ManipNDofSim : public rclcpp::Node
{
public:
    ManipNDofSim(std::shared_ptr<ManipNDof> pManip) : Node("ManipNDofSim"), mManip(pManip)
    {
        mJointPosCmdSub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "ManipNDof_JointPos_Command", 10, std::bind(&ManipNDofSim::position_command_callback, this, _1));
        
        mJointPosPub = this->create_publisher<std_msgs::msg::Float64MultiArray>("ManipNDof_JointPos_Pub", 10);

        mPubTimer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ManipNDofSim::publish_state, this));
    }

private:
    void position_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s", std::to_string(msg->data[0]).c_str());

        std::vector<double> q;
        // Create joitn config vector and command joint config
        for (size_t i = 0; i < msg->data.size(); i++)
        {
            q.push_back(msg->data[i]);
        }

        mManip->CommandJointConfig(q);
    }

    void publish_state()
    {
        auto pos_msg = std_msgs::msg::Float64MultiArray();
        auto vel_msg = std_msgs::msg::Float64MultiArray();

        int dof = mManip->GetDof();

        pos_msg.data.resize(dof);
        vel_msg.data.resize(dof);

        Eigen::VectorXd q = mManip->GetTheta();
        Eigen::VectorXd q_dot = mManip->GetThetaDot();

        for(int i = 0; i < dof; i++)
        {
            pos_msg.data[i] = q(i);
        }

        mJointPosPub->publish(pos_msg);
    }

    std::shared_ptr<ManipNDof> mManip;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr mJointPosCmdSub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mJointPosPub;
    rclcpp::TimerBase::SharedPtr mPubTimer;

};
int main(int argc, char * argv[])
{
    // TODO use parameters
    std::vector<double> l {5,4};

    // Define twists
    Eigen::Vector3d z_axis {0, 0, 1};
    Eigen::Vector3d q_j1 {l.at(0), 0, 0};
    Eigen::Vector3d q_j2 {l.at(0) + l.at(1), 0, 0};
    TwistJoint j1(q_j1, z_axis);
    TwistJoint j2(q_j2, z_axis);

    std::vector<TwistJoint> manip_joints {j1, j2};

    // Initial pose
    Eigen::Matrix4d g_0 = Eigen::Matrix4d::Identity();
    g_0(0,3) = l.at(0) + l.at(1);

    JointControllerSnap jc;
    std::shared_ptr<ManipNDof> manip = std::make_shared<ManipNDof>(jc);
    manip->Initialize(manip_joints, g_0);

    // Sim thread
    std::thread stepsim(StepSim, manip);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManipNDofSim>(manip));
    rclcpp::shutdown();
    return 0;
}