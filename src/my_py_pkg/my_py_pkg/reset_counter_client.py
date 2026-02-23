#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
from functools import partial

class ResetCounterClient(Node):
    def __init__(self):
        super().__init__("reset_counter_client")
        self.client_ = self.create_client(SetBool, "reset_counter")

    def call_reset_counter(self, a):
        while not self.client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Reset counter Server...")

        request = SetBool.Request()
        request.data = a
        # call_async: İsteği gönderir ve hemen alt satıra geçer (kod donmaz).
        # Bize bir 'future' (takip numarası) döndürür.
        future = self.client_.call_async(request)
        # future.add_done_callback: Bu bir "Beni Ara" talimatıdır.
        # şu an başka işlerle ilgileniyorum (spin içindeyim). 
        # Cevap geldiği an, hiç vakit kaybetmeden şu aşağıdaki fonksiyonu (callback) çalıştır!"
        future.add_done_callback(partial(self.callback_call_reset_counter, request = request))
        # request= request İsimlendirilen_Bölme = Elimizdeki_Veri
        """
        Soldaki request: "Callback fonksiyonunun içindeki parametre ismi bu olsun" diyoruz.

        Sağdaki request: "O bölmeye, şu an elimde tuttuğum bu paketi koy" diyoruz.

        partial kelime anlamı olarak "kısmi" demektir. Şöyle düşün:
        Birine bir paket teslim edeceksin ama o kişi sadece zarf taşıyabiliyor. Sen zarfın içine (yani fonksiyonun içine) gizlice bir not 
        (bizim request verimiz) iliştiriyorsun ve zarfı öyle teslim ediyorsun.

        """

    def callback_call_reset_counter(self, future, request):
        response = future.result()
        self.get_logger().info(f"Server Response -> Succes: {response.success}")
        self.get_logger().info(f"Server Message: {response.message}")

def main(args=None):
    rclpy.init(args=args)
    node = ResetCounterClient()
    node.call_reset_counter(True)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()