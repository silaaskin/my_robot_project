#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using namespace std::chrono_literals;
class ResetCounterClient : public rclcpp:: Node
{
    public:
        ResetCounterClient() : Node("reset_counter_client")
        {
            // Tıpkı Python'daki gibi servis ismini ve tipini ağa tanıtıyoruz.
            client_ = this->create_client<example_interfaces::srv::SetBool>("reset_counter");
        }

        void call_reset_counter(bool a)
        {
            while(!this->client_->wait_for_service(1s))
            {
                RCLCPP_WARN(this->get_logger(), "Wait for the server...");
            }

            // Paylaşımlı (Shared) bir istek paketi oluşturup içini dolduruyoruz.
            auto request = std::make_shared<example_interfaces::srv::SetBool::Request>();  
            request->data = a   ;

            // İsteği gönderiyoruz ve cevap gelince 'callback' fonksiyonunu çalıştır diyoruz.
            client_->async_send_request(
                request, 
                std::bind(&ResetCounterClient::callback_ResetCounterClient, this, std::placeholders::_1)
            );
        }

    private:
    // ROS2, cevap geldiğinde sonucu bu 'future' kutusunun içine koyup bize getirir.
    void callback_ResetCounterClient(rclcpp::Client<example_interfaces::srv::SetBool>::SharedFuture future)
    {
        // Kutuyu (future) açıp içindeki gerçek yanıt paketini (response) alıyoruz.
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(),"Server Response -> Success: %d", (int)response->success);
        RCLCPP_INFO(this->get_logger(),"Server Response -> Message: %s", response->message.c_str());

    }

    // İstemci kumandası; düğüm yaşadığı sürece bellekte kalması için SharedPtr kullanıyoruz.
    rclcpp::Client<example_interfaces::srv::SetBool>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    // Sınıfımızdan bir kopya üretip güvenli bir bellek alanına yerleştiriyoruz.
    auto node = std::make_shared<ResetCounterClient>();
    node->call_reset_counter(true);
    // Düğümü çalıştırıp gelecek olan cevapları (callback) dinlemeye başlıyoruz.
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}