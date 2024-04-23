#include <rclcpp/rclcpp.hpp>
#include <gary_msgs/msg/auto_aim.hpp>
#include "CommPort.h"
#include <geometry_msgs/msg/quaternion.hpp>

using namespace std::chrono_literals;

class ContactNode : public rclcpp::Node
{
public:
    ContactNode() : Node("contact")
    {
        comm.Start();
        quaternion_pub_ = this->create_publisher<geometry_msgs::msg::Quaternion>("/quaternion", 10);
        _gamestatus_sub_ = this->create_subscription<gary_msgs::msg::AutoAIM>("/autoaim/target", 1000, std::bind(&ContactNode::autoaim_callback, this, std::placeholders::_1));
        __timer__ = this->create_wall_timer(100ms, std::bind(&ContactNode::timer_callback, this));
    }

    void autoaim_callback(const gary_msgs::msg::AutoAIM::SharedPtr msg)
    {
        uint8_t packet[12];
        packet[0] = 0x5A; // 假设开始字节
        packet[1] = 0;    // 控制码，需要适当设置
        packet[2] = msg->target_id;

        union {
            float actual;
            uint8_t raw[4];
        } tx_x{}, tx_y{};

        float x_offset = 0.00; // 可调整的偏移量
        float y_offset = 0.00; // 可调整的偏移量

        tx_x.actual = pitch_temp + msg->pitch + x_offset;
        tx_y.actual = yaw_temp + msg->yaw + y_offset;

        RCLCPP_DEBUG(this->get_logger(), "send delta pitch is %f, send actual pitch is %f", msg->pitch, tx_x.actual);
        RCLCPP_DEBUG(this->get_logger(), "send delta yaw is %f, send actual yaw is %f", msg->yaw, tx_y.actual);

        for (int i = 0; i < 4; i++) {
            packet[3 + i] = tx_x.raw[i];  // 注意数组位置的更正
            packet[7 + i] = tx_y.raw[i];
        }
        Crc8Append(packet, sizeof(packet)); // 假设存在这样的函数
        comm.Write(packet, sizeof(packet), true);
    }

    void timer_callback()
    {
        pitch_temp = comm.get_Pitch();
        yaw_temp = comm.get_Yaw();

        auto q = comm.getQuaternion();
        geometry_msgs::msg::Quaternion quaternion_msg;
        quaternion_msg.x = q[0];
        quaternion_msg.y = q[1];
        quaternion_msg.z = q[2];
        quaternion_msg.w = q[3];

        quaternion_pub_->publish(quaternion_msg);
        RCLCPP_INFO(this->get_logger(), "Published quaternion x: %f, y: %f, z: %f, w: %f", q[0], q[1], q[2], q[3]);
    }

private:
    rclcpp::Subscription<gary_msgs::msg::AutoAIM>::SharedPtr _gamestatus_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr quaternion_pub_;
    rclcpp::TimerBase::SharedPtr __timer__;
    CommPort comm;
    float pitch_temp{};
    float yaw_temp{};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ContactNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}