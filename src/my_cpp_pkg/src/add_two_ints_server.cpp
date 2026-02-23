#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::placeholders;

class AddTwoIntsServerNode : public rclcpp:: Node
{
    public:
        AddTwoIntsServerNode() : Node("add_two_ints_server")
        {
            server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
                "add_two_ints", // 1. Parametre: Servisin ismi (ROS ağında görünen ad)

                // 2. Parametre: Servis çağrıldığında çalışacak fonksiyon (Callback)
                std::bind(&AddTwoIntsServerNode::callbackAddTwoInts, // Hangi fonksiyon çalışacak?
                     this, _1, // 1. argüman (Request) için bir yer tutucu.
                      _2)      // 2. Argüman (Response) için yer tutucu.

                // std::bind burada bir "bağlayıcı" görevi görür.

            );
            RCLCPP_INFO(this->get_logger(), "Add two ints Service has been started.");
        }
    private:
    void callbackAddTwoInts(example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                            example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
                            {
                                // Pointer (->) kullandığın için direkt bellekteki 'response' kutusuna yazıyorsun.
                                // Fonksiyon 'void' olduğu için bir şey döndürmez (return yok).
                                // Sen bu satırı yazdığın an, ana bellekteki yanıt güncellenmiş olur.
                                response->sum = request->a + request->b;
                                RCLCPP_INFO(this->get_logger(), "%d + %d = %d", 
                                        (int)request->a, (int)request->b, (int)response->sum );
                            }
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<AddTwoIntsServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}