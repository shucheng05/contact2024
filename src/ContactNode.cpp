
#include<rclcpp/rclcpp.hpp>
#include<gary_msgs/msg/auto_aim.hpp>
#include<CommPort.h>

using namespace std::chrono_literals;

class ContactNode : public rclcpp::Node
{
    public:
        ContactNode():Node("contact")
        {
            comm.Start();
            _gamestatus_sub_ = this->create_subscription<gary_msgs::msg::AutoAIM>("/autoaim/target",1000,std::bind(&ContactNode::autoaim_callback,this,std::placeholders::_1));
            __timer__ = this->create_wall_timer(100ms,std::bind(&ContactNode::timer_callback,this));
        }

        void autoaim_callback(const gary_msgs::msg::AutoAIM::SharedPtr msg)
        {
            TxPacket packet;
            packet[0] = 0x5A;
            packet[1] = 0;
            packet[2] = msg->target_id;
            union {
                float actual;
                uint8_t raw[4];
            } tx_x{}, tx_y{};

            float x_offset = 0.00;
            float y_offset = 0.00;

            tx_x.actual = pitch_temp + msg->pitch + x_offset;
            tx_y.actual = yaw_temp + msg->yaw + y_offset;
            RCLCPP_DEBUG(this->get_logger(),"send delta pitch is %f,send actual pitch is %f",msg->pitch,tx_x.actual);
            RCLCPP_DEBUG(this->get_logger(),"send delta yaw is %f,send actual yaw is %f",msg->yaw,tx_y.actual);

            for (int i = 0; i < 4; i++) {
                packet[2 + i] = tx_x.raw[i];  // x
                packet[6 + i] = tx_y.raw[i];  // y
            }
            Crc8Append(&packet, 12);
            //printf("  pitch %.2f yaw %.2f id %d cmd %d  time %.3f  \n", robotcmd.pitch_angle, robotcmd.yaw_angle, robotcmd.target_id, packet[1], robotstatus.timestamp);

            comm.Write(&packet, 12, true);
        }

        void timer_callback()
        {
            pitch_temp = comm.get_Pitch();
            yaw_temp = comm.get_Yaw();
        }
        
    private:
        rclcpp::Subscription<gary_msgs::msg::AutoAIM>::SharedPtr _gamestatus_sub_;

        rclcpp::TimerBase::SharedPtr __timer__;
        CommPort comm;
        float pitch_temp{};
        float yaw_temp{};
};

int main()
{

    return 0;
}

