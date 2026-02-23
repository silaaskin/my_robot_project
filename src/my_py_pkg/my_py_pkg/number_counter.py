import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool

class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.counter_ = 0
        self.subscriptions_ = self.create_subscription(
            Int64,"number", self.callbackNumber, 10)
        self.get_logger().info("Callback Number has been started. ")
        self.publisher_ = self.create_publisher(Int64, "number_counter", 10)
        self.server = self.create_service(SetBool, "reset_counter", self.callbackResetCounter)
        

    def callbackNumber(self, msg: Int64):
        self.counter_ += msg.data
        # yeni bir mesaj paketi oluştur ve fırlat
        new_msg = Int64()
        # Sayıyı paketin içindeki .data kısmına yerleştir
        new_msg.data = self.counter_
        # Paketi fırlat
        self.publisher_.publish(new_msg)
        self.get_logger().info(f"Guncel Toplam: {self.counter_}")
    
    def callbackResetCounter(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            self.counter_ = 0
            response.success = True
            response.message = "Reset counter has been reseted. "
            self.get_logger().info("Reset counter has been reseted!") # Server terminalinde de görelim
        else:
            response.success = False
            response.message = "Reset denied (request data was false)."
        return response




def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()