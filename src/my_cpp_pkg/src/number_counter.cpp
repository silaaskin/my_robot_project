#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"
using namespace std::placeholders;

class NumberCounterNode : public rclcpp:: Node
{
    public:
        NumberCounterNode() : Node("number_counter")
        {
            counter_ = 0;
            subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
                "number", 10, std::bind(&NumberCounterNode::callbackNumber, this, _1)
                //std::bind argümanları: 1. argüman: fonksiyonun adresi, 2. argüman nesnenin kendisi
                //this diyerek ROS2'ye bu fonksiyonu şu an çalışan nesne üzerinden çalıştır demiş oluruz
                //Eğer bunu yazmazsan, fonksiyon counter_ değişkenine ulaşamaz çünkü hangi nesnenin sayacını artıracağını bilemez.

                
            );

            server_ = this->create_service<example_interfaces::srv::SetBool>(
                "reset_counter", // 1. Parametre: Servisin ismi (ROS ağında görünen ad)

                // 2. Parametre: Servis çağrıldığında çalışacak fonksiyon (Callback)
                std::bind(&NumberCounterNode::callbackResetCounter, // Hangi fonksiyon çalışacak?
                     this, _1, // 1. argüman (Request) için bir yer tutucu.
                      _2)      // 2. Argüman (Response) için yer tutucu.

                // std::bind burada bir "bağlayıcı" görevi görür.

            );
            
            publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
            RCLCPP_INFO(this->get_logger(), "number_counter has been started.");
        }
    private:
    void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        counter_ += msg->data;
        auto new_msg = example_interfaces::msg::Int64();
        new_msg.data = counter_;
        publisher_->publish(new_msg);
        RCLCPP_INFO(this->get_logger(), "Guncel Toplam: %ld", counter_);
    }

    void callbackResetCounter(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                            example_interfaces::srv::SetBool::Response::SharedPtr response)
        {
            if(request->data == 1)
            {
                counter_ = 0;
                response->success = 1;
                response->message = "Counter has been reset zero";
                RCLCPP_INFO(this->get_logger(), "Counter has been reset zero!");
                
            }
            else 
            {
                response->success = false; // Eğer false gönderilirse sıfırlama yapma
                response->message = "Reset denied (request data was false).";
            }
        }
    
    int64_t counter_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}