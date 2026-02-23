#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServerNode(Node):
    def __init__(self):
        super().__init__("add_two_int_server")
        # create_service(Servis Türü, Servis İsmi, Callback)
        # Sunucuya bir istek göndermek için bir istemciye ihtiyacımız var,
        # bu yüzden talepleri aldıkça işleme koyacağız: callback'in amacı
        self.server_ = self.create_service(AddTwoInts, "add_two_ints", self.callback_add_two_ints)
        self.get_logger().info("Add Two Ints server has been started.")


    # ROS2de bir servis callback fonksiyonu her zaman 2 parametre alır: request, response
    # : AddTwoInts.Request: aslında zorunlu değildir,Python'a şunu dersin: "Bu request nesnesi, AddTwoInts servisinin içindeki Request tipindedir.
    def callback_add_two_ints(self, request: AddTwoInts.Request, response:AddTwoInts.Response):
        response.sum = request.a + request.b
        self.get_logger().info(str(request.a) + " + " + 
                                 str(request.b) + " = " + str(response.sum))
        # ROS2 bunu alıp client'e iletir
        return response
def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

""""
1. Adım: İstemci (Client) Hazırlanır

Sen Client terminalinde düğümü başlattığında, kodun içindeki a=2, b=7 sayıları henüz senin bilgisayarının RAM'inde bekleyen küçük kutucuklardır.
2. Adım: İstek (Request) Yola Çıkar

Sen call_async(request) dediğin an, Client bu sayıları bir zarfa koyar (Request paketi) ve ROS 2 ağına bağırır: "Hey, adı add_two_ints olan servis nerede? Sana iki sayı getirdim!"
3. Adım: Sunucu (Server) Yakalar

Server düğümün zaten terminalde açık ve "dinleme" modundadır. Bu mesajı havada yakalar.

    Zarfı açar.

    İçinden a ve byi çıkarır.

    Kendi içindeki o callback_add_two_ints fonksiyonunu çalıştırır.

4. Adım: Mutfakta İşlem Yapılır (Server)

Server senin verdiğin sayıları toplar: 2+7=9.
Sonra bu 9 sonucunu, zarfın Response (Yanıt) kısmına yazar.
5. Adım: Yanıt (Response) Geri Döner

Server, üzerine 9 yazdığı zarfı kapatır ve Client'a geri fırlatır.
6. Adım: İstemci Paketi Teslim Alır (Callback)

İşte burası senin anlamadığın o "Callback" kısmının devreye girdiği yerdir:

    Client'ın spin fonksiyonu sayesinde kulağı kapıdadır.

    Cevap paketinin (Response) geldiğini görünce, senin add_done_callback ile bağladığın o fonksiyonu (yani callback_call_add_two_ints) hemen çalıştırır.

    Senin fonksiyonun da future.result() diyerek zarfı açar ve içindeki 9 sayısını ekrana basar.

"""