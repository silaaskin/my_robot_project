#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;
class AddTwoIntsClient : public rclcpp:: Node
{
    public:
        AddTwoIntsClient() : Node("add_two_ints_client")
        {
            // Tıpkı Python'daki gibi servis ismini ve tipini ağa tanıtıyoruz.
            client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        }

        void call_add_two_ints(int a, int b)
        {
            while(!this->client_->wait_for_service(1s))
            {
                RCLCPP_WARN(this->get_logger(), "Wait for the server...");
            }

            // Paylaşımlı (Shared) bir istek paketi oluşturup içini dolduruyoruz.
            auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();  
            request->a = a;
            request->b = b;    

            // İsteği gönderiyoruz ve cevap gelince 'callback' fonksiyonunu çalıştır diyoruz.
            client_->async_send_request(
                request, 
                std::bind(&AddTwoIntsClient::callback_CallAddTwoInts, this, std::placeholders::_1)
            );
        }

    private:
    // ROS2, cevap geldiğinde sonucu bu 'future' kutusunun içine koyup bize getirir.
    void callback_CallAddTwoInts(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
    {
        // Kutuyu (future) açıp içindeki gerçek yanıt paketini (response) alıyoruz.
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Sum: %d", (int)response->sum);
    }

    // İstemci kumandası; düğüm yaşadığı sürece bellekte kalması için SharedPtr kullanıyoruz.
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    // Sınıfımızdan bir kopya üretip güvenli bir bellek alanına yerleştiriyoruz.
    auto node = std::make_shared<AddTwoIntsClient>();
    node->call_add_two_ints(2,7);
    // Düğümü çalıştırıp gelecek olan cevapları (callback) dinlemeye başlıyoruz.
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}