import os
# ament_index_python: ROS 2'ye "Benim paketim bilgisayarın neresinde?" diye sormak için kullanılan kütüphanedir.
from ament_index_python.packages import get_package_share_path
# LaunchDescription: Çalıştırılacak tüm programları içine koyduğumuz "ana liste"dir.
from launch import LaunchDescription
# ParameterValue: Bir veriyi ROS 2'ye "Bu bir ayardır, tipi de metindir" diyerek paketlemek için kullanılır.
from launch_ros.parameter_descriptions import ParameterValue
# Command: Python içinden terminal komutu (mesela xacro) çalıştırmayı sağlar.
from launch.substitutions import Command
# Node: ROS 2'deki her bir bağımsız programa (düğüme) verilen isimdir.
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. ADIM: DOSYANIN YERİNİ BUL
    # 'my_robot_description' paketini bul, içine gir, 'urdf' klasörünü bul ve 'my_robot.urdf' dosyasının adresini al.
    # Bu satır sayesinde kod, senin bilgisayarında da arkadaşının bilgisayarında da hatasız çalışır.
    urdf_path = os.path.join(get_package_share_path('my_robot_description'), 'urdf' , 'my_robot.urdf.xacro')
    rviz_config_path= os.path.join(get_package_share_path('my_robot_description'), 'rviz', 'urdf_config.rviz')
    
    # 2. ADIM: ROBOT TARİFİNİ HAZIRLA
    # Command: "Git terminale 'xacro [dosya_yolu]' yaz ve çıkan sonucu bana getir" der.
    # ParameterValue: "Gelen o sonucu al ve 'bu bir metindir (str)' diye etiketle" der. 
    # Sonuçta 'robot_description' değişkeni içinde robotun tüm XML kodu hazır bekler.
    #xacrodan sonra boşluk koyman lazım
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # 3. ADIM: ROBOT STATE PUBLISHER (İSKELETİ YAYINLAYAN PROGRAM)
    # Bu program, robotun hangi parçası nerede (TF) hesaplar ve tüm dünyaya ilan eder.
    robot_state_publisher_node = Node(
        package='robot_state_publisher', # Hangi paketin içinde?
        executable='robot_state_publisher', # Paketin içindeki hangi dosyayı çalıştırayım?
        parameters=[{'robot_description': robot_description}] # Hazırladığımız robot tarifini buna veriyoruz.
    )

    # 4. ADIM: JOINT STATE PUBLISHER GUI (HAREKET ÇUBUKLARI)
    # Robotun eklemlerini (mesela tekerleklerini) fareyle oynatabilmen için o küçük pencereyi açar.
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # 5. ADIM: RVIZ2 (GÖRSEL EKRAN)
    # Robotu 3 boyutlu bir dünyada, lazerleri, kameraları ve iskeletiyle göreceğimiz ana ekran.
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # 6. ADIM: HEPSİNİ AYNI ANDA ATEŞLE
    # Yukarıda tanımladığımız 3 programı (düğümü) bir listeye koyup ROS 2'ye teslim ediyoruz.
    # 'ros2 launch ...' dediğin an bu 3 program aynı anda pıt diye açılır.
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
    ])