#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SimplePublisher : public rclcpp::Node {
public:
    SimplePublisher() : Node("simple_publisher"), count_(0) {
        // Declare a parameter with default value
        this->declare_parameter("publish_rate", 500);
        this->declare_parameter("prefix", "Message");
        
        // Get parameter values
        int rate = this->get_parameter("publish_rate").as_int();
        prefix_ = this->get_parameter("prefix").as_string();
        
        publisher_ = this->create_publisher<std_msgs::msg::String>("my_topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(rate),
            std::bind(&SimplePublisher::timer_callback, this)
        );
        RCLCPP_INFO(this->get_logger(), "Publisher started with rate=%dms, prefix='%s'", rate, prefix_.c_str());
    }

private:
    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = prefix_ + " #" + std::to_string(count_);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
        count_++;
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string prefix_;
    int count_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePublisher>());
    rclcpp::shutdown();
    return 0;
}