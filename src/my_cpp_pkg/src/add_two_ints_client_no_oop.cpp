#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;
int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("add_two_int_client_no_oop");
    //create_client fonksiyonu, bize doğrudan bir nesne değil, o nesnenin SharedPtr (Akıllı İşaretçi) halini döndürür.
    auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    while(!client->wait_for_service(1s))
    {
        RCLCPP_WARN(node->get_logger(), "Waiting for the server...");
    }

    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = 6;
    request->b = 8;
    
    // 2. async_send_request: Bu 'kumandayı' (request) ROS'a veririz. 
   // ROS bu kumandayı kullanarak bellekteki sayıları okur ve server'a gönderir.
    auto future = client->async_send_request(request);
    rclcpp::spin_until_future_complete(node,future);
    
    //Python'da future.result() demiştik, burada get() diyoruz.
    // 4. future.get(): 'future' bir sözdür. .get() diyerek o sözün içindeki gerçek veriyi (Response) çekip alırız.
    auto response = future.get();
    RCLCPP_INFO(node->get_logger(), "%d + %d = %d", 
                (int)request->a, (int)request->b, (int)response->sum);


    rclcpp::shutdown();
    return 0;
}