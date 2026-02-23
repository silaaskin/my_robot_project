#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

#benim sınıfım ana sınıfın bir çocuğu, içindeki bütün özellikleri kullanmabilmem için kendini hazırla
class RobotNewsStationNode(Node):
    def __init__(self):
        #buraya yazılan isim ROS2 ağında rqt_graph'ta görülecek isim
        self.robot_name_ = "C3PO"
        super().__init__("robot_news_stations")
        #create_publisher argümanları: (hangi mesaj tipinde konuşacağız, hangi topic üzerinden konuşacağız, kuyruk boyutu)
        self.publisher_ = self.create_publisher(String, "robot_news", 10)
        #create_timerda publish_news parantezli yazılmaz çünkü şu an çalıştırmayacağız, zamanı gelince çalıştıracağız 
        self.timer_ = self.create_timer(0.5, self.publish_news)
        self.get_logger().info("Robot News Station has been started")     

    def publish_news(self):
        msg = String()
        msg.data = "Hi, this is " + self.robot_name_ + " from the robot news statiton"
        self.publisher_.publish(msg) 

def main(args=None):
    rclpy.init(args = args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()