#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

//Bir yere fonksiyon vereceğin zaman (Timer veya Subscriber gibi): std::bind( &SınıfAdı::FonksiyonAdı, this )
using namespace std::chrono_literals;
class RobotNewsStationNode : public rclcpp:: Node
{
    public:
        RobotNewsStationNode() : Node("robot_news_stations"), robot_name_("R2D2")
        {
            //Bana bir yayıncı makinesi üret, taşıyacağı mesaj tipi şu <...> olsun.
            publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
            //std::bind = paketleme yapma komutu
            // &RobotNesStatiınNode::publishNews = hangi fonksiyon paketlensin sorusunun cevabı
            //this = bu fonksiyon kimin üzerinde çalışsının cevabı, çalıştırma sadece adresini al (&)
            timer_ = this->create_wall_timer(0.5s, std::bind(&RobotNewsStationNode::publishNews, this));
            RCLCPP_INFO(this->get_logger(), "Robot news Station has been started");
            
        }
    private:
    void publishNews()
    {
        auto msg = example_interfaces::msg::String();
        msg.data = std::string("Hi, this is ") + robot_name_ + "from the robot news stations";
        publisher_->publish(msg);
    }
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;

    //DEĞİŞKEN OLUŞTURMA FORMÜLÜ: KütüphaneAdı :: NesneTürü < VeriTipi > :: HafızaTürü DeğişkenAdı ;
    std::string robot_name_;

    //rclcpp::Publisher = bu değişken bir ROS2 yayıncısı (Publisher) olacak
    // <> içine yazdığımız şey o yayıncının kapasitesini ve sınırını belirler, buna tip belirleme denilir.
    // SharedPtr = nesnenin hafıza biçimidir

    //süreyi tutacak saatin kumandası:
    rclcpp::TimerBase::SharedPtr timer_;

    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    // std::make_shared = bana bu sınıftan bir tane üret ve bellekte güvenli bir yere (heap) koy ve bana akıllı bir kumanda ver.
    auto node = std::make_shared<RobotNewsStationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}