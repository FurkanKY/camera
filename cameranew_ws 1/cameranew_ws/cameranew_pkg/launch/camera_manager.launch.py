from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    # =========================================================
    # KAMERA KURULUMU: 4 WEBCAM + 1 IP KAMERA
    #
    # Kameralar dogrudan bu launch dosyasindan degil,
    # camera_manager node'u uzerinden yonetilir.
    #
    # camera_manager:
    #   - acilista 4 webcam + IP kamerayi otomatik acar
    #     (AUTO_START_CAMERAS, camera_manager.py icinde)
    #   - viewer'daki 1-2-3-4-5 tuslarindan gelen komutlarla
    #     kameralari tek tek ac/kapat yapar
    #
    # Boylece donma olursa yer istasyonundan birkac kamera
    # kapatilarak kalan kameralarla devam edilebilir.
    #
    # ZED kameralari bu kurulumda kullanilmiyor.
    # =========================================================
    camera_manager_node = Node(
        package='cameranew_pkg',
        executable='camera_manager',
        name='camera_manager',
        output='screen'
    )

    # =========================================================
    # SCIENCE NODE
    # (Bu node can2 CAN arayuzu gerektiriyor; sadece AGX uzerinde calisir.)
    # (Yer istasyonu / gelistirme PC'de CAN donanimi olmadigi icin
    #  bu launch'tan kaldirildi. AGX tarafindaki launch'a ekle.)
    # =========================================================
    # science_node = Node(
    #     package='cameranew_pkg',
    #     executable='science',
    #     name='agx_web_alici_node',
    #     output='screen'
    # )

    return LaunchDescription([
        camera_manager_node,
        # science_node,  # <- AGX launch'ina tasinmali
    ])
