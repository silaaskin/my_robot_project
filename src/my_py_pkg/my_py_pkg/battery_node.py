#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed

class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery_node")
        self.battery_state_ = "full"
        self.counter_ = 0
        self.timer_ = self.create_timer(1.0, self.check_status_battery)
        self.client_ = self.create_client(SetLed, "set_led")

    def check_status_battery(self):
        self.counter_+= 1
        self.get_logger().info(f"Saniye: {self.counter_} - Durum: {self.battery_state_}")
        if self.battery_state_ == "full" and self.counter_ >= 4:
            self.battery_state_ = "empty"
            self.get_logger().warn("Battery is empt! Led Yakiliyor")
            self.call_set_led_service(3, 1)

        elif self.battery_state_ == "empty" and self.counter_ >= 10:
            self.battery_state_ = "full"
            self.counter_ = 0
            self.get_logger().info("Battery is full! Led Sonduruluyor")
            self.call_set_led_service(3, 0)

    def call_set_led_service(self, led_number, state):
        while not self.client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Set Led Server...")
        
        request = SetLed.Request()
        request.led_number = led_number
        request.state = state

        future = self.client_.call_async(request)
        future.add_done_callback(self.callback_set_led)

    def callback_set_led(self, future):
        response = future.result()
        self.get_logger().info(f"Succes: {response.success}")


   
def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()