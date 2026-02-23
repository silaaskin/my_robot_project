#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


def main(args=None):
    rclpy.init(args=args)
    node = Node("Add_two_ints_client_no_oop")

    client = node.create_client(AddTwoInts, "add_two_ints")

    # client.wait_for_service: bu fonksiyon ortalıkta bu servisi sunan bir server var mı kontrol eder
    # parametresi: zaman aşımı süresi, o saniye içinde bulamazsa false döndürür, bulursa true döndürür
    # servis hazır oluncaya kadar 1 saniye aralıklarsa mesajı bastırır
    while not client.wait_for_service(1.0):
        node.get_logger().warn("Waiting for Add Two Ints Server...")

    # request sınıfından yeni bir örnek yaratılır 
    #  bu satır çalıştırıldığında example_interface'in AddTwoInts tipinde üst kısmında tanımlanan değişkenleri 
    # yani a ve b yi içeren bir veri yapısı açılır
    # fonksiyon olmamasına rağmen neden parantez var?

    request = AddTwoInts.Request()
    request.a = 3
    request.b = 8

    # client: oluşturduğumuz haberleşme cihazı
    # .call_async bu paketi gönder ama karşıdan cevap gelene kadar bu satırda bekleme, sonraki satıra geçebilirsin
    # request: içini doldurduğumuz sipariş formu (veri paketi)
    # future: bu işlemin takip numarası
    future = client.call_async(request)

    # future tamamlanana kadar (cevap gelene kadar) çarkı döndür
    rclpy.spin_until_future_complete(node, future)

    # result() future paketindedir, eğer işlem bittiyse gelen veriyi (response) paketinden çıkar ve kulanıcıya teslim et  
    response = future.result()
    node.get_logger().info(str(request.a) + " + " + str(request.b) + " = " + 
                               str(response.sum))

    rclpy.shutdown()

if __name__ == "__main__":
    main()