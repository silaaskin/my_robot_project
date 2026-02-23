#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed
from my_robot_interfaces.msg import LedStateArray


class LedPanelNode(Node):
    def __init__(self):
        super().__init__("led_panel")
        # 1. Hafızada LED durumlarını tut (3 LED: Hepsi sönük başlasın)
        self.led_states_ = [0, 0, 0]
        
        # Servis Tanımı (Tipini eklemeyi unutma!)
        self.server = self.create_service(SetLed, "set_led", self.callback_set_led)
        self.publisher_ = self.create_publisher(LedStateArray, "led_panel_state", 10)
        self.timer_ = self.create_timer(0.5, self.pub_led_state)
        self.get_logger().info("LED Panel Server has been started.")
    def pub_led_state(self):
        msg = LedStateArray()
        msg.led_states = self.led_states_
        self.publisher_.publish(msg)

    def callback_set_led(self, request, response):
        # 2. Gelen veriyi kontrol et (Doğrulama)
        # Eğer istenen LED numarası dizimizin dışındaysa hata dön
        if request.led_number > len(self.led_states_) or request.led_number <= 0:
            response.success = False 
            
        else:
            self.led_states_[request.led_number - 1] = request.state
            response.success = True 
            self.get_logger().info(f"Succes: LED {request.led_number} state: {request.state}.")            
        return response # Python'da return şart!
def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()