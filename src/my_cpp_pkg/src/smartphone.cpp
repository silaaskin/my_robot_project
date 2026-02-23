#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std::placeholders;

class SmartphoneNode : public rclcpp:: Node
{
    public:
        SmartphoneNode() : Node("smartphone")
        {
            //sıralama: tip belirtme, dinleyeceğimiz kanalın (topic) adı, kuyruk boyutu, hangi fonksiyonun kimin üzerinde çalıştırılacağı paketi,
            // _1 boş sandalye, gelen mesajı yakalayıp fonksiyonun parantez içine argüman olarak gönderir.
            subscriber_ = this->create_subscription<example_interfaces::msg::String>(
                "robot_news", 10, std::bind(&SmartphoneNode::callbackRobotNews, this, _1)
            );
            RCLCPP_INFO(this->get_logger(), "Smartphone has been started.");
        }
    private:
    void callbackRobotNews(const example_interfaces::msg::String::SharedPtr msg)
    {
        //.c_str(): Modern C++ yazısını, logger'ın anlayacağı eski tip dile çevirir.
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
    }
    //Bunu sınıfın en altında private olarak tanımladık ki düğüm yaşadığı sürece bu abonelik de hayatta kalsın.
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SmartphoneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}