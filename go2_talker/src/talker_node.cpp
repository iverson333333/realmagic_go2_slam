#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class TalkerNode : public rclcpp::Node
{
public:
    TalkerNode() : Node("talker_node"), count_(0)
    {
        // 创建发布器，发布到 /chatter 话题
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

        // 创建定时器，每500毫秒发布一次消息
        timer_ = this->create_wall_timer(
            500ms,
            [this]()
            { timer_callback(); });

        RCLCPP_INFO(this->get_logger(), "Talker node has been started.");
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, ROS2! Message count: " + std::to_string(count_++);

        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TalkerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
