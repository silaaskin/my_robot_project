#include "rclcpp/rclcpp.hpp"

class TemplateNode : public rclcpp:: Node
{
    public:
        TemplateNode() : Node("template_node")//node nesnesinin ros2 ağındaki adıdır
        {
        }
    private:
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TemplateNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}