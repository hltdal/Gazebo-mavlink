# launcher_functions.py
import sys
import subprocess
import time
import threading
import rospy
from nav_msgs.msg import Odometry
from design import Ui_MainWindow
from PyQt5.QtWidgets import *
from geometry_msgs.msg import PoseStamped
# Import mavutil
from pymavlink import mavutil



class LauncherAppFunctions(QMainWindow):
    def __init__(self):
        super().__init__()
        self.main = Ui_MainWindow()
        self.main.setupUi(self)
        # Dizini ayarla
        self.working_directory = "~"
        
        # Butonlara tıklama olaylarını bağla
        self.main.run_simulation_button.clicked.connect(self.launch_simulation)
        self.main.start_multisitl_button.clicked.connect(self.start_multisitl)
        self.main.arm_button.clicked.connect(self.arm_function)
        self.main.takeoff_button.clicked.connect(self.get_takeoff_text)
        self.main.takeoff_button.clicked.connect(self.take_off_function)
        self.main.land_button.clicked.connect(self.land_function)
        self.main.disarm_button.clicked.connect(self.disarm_function)
        self.main.swarm_move_button.clicked.connect(self.swarm_move_function)

    def launch_simulation(self):
        command = "roslaunch iq_sim multi_drone.launch"
        self.run_in_new_terminal(command)

    def start_multisitl(self):
        command = "./multi_sitl.sh"
        self.run_in_new_terminal(command)
        self.drone1 = mavutil.mavlink_connection('127.0.0.1:14550')
        self.drone1.wait_heartbeat()
        print("Drone1 bağlandı")
        self.drone2 = mavutil.mavlink_connection('127.0.0.1:14560')
        self.drone2.wait_heartbeat()
        print("Drone2 bağlandı")
        self.drone3 = mavutil.mavlink_connection('127.0.0.1:14570')
        self.drone3.wait_heartbeat()
        print("Drone3 bağlandı")
        self.continually_update_position()

    def arm_function(self):
        try:
            for drone in [self.drone1, self.drone2, self.drone3]:
                # Arm the vehicle
                drone.mav.command_long_send(
                    drone.target_system,
                    drone.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    1, 0, 0, 0, 0, 0, 0)
                # wait until arming confirmed
                drone.motors_armed_wait()
                print('Drone armed!')
            for drone in [self.drone1, self.drone2, self.drone3]:
                drone.set_mode("GUIDED")  # GUIDED modunu seç
                print("Drone GUIDED moduna geçiyor...")
        except Exception as e:
            print(f"Komut çalıştırılırken bir hata oluştu: {e}")
    
    def get_takeoff_text(self):
        self.takeoff_value=self.main.takeoff_lineEdit.text()

    def take_off_function(self):
        try:
            altitude = int(self.takeoff_value)
            for drone in [self.drone1, self.drone2, self.drone3]:
            # TAKEOFF komutunu gönder
                drone.mav.command_long_send(
                    drone.target_system,  # Hedef sistem (drone)
                    drone.target_component,  # Hedef bileşen
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # TAKEOFF komutu
                    0,  # Confirmation
                    0, 0, 0, 0,  # Parametreler (ilk dört boş geçiliyor)
                    0, 0, altitude  # Latitude, Longitude (boş geç), altitude (hedef irtifa)
                )
                print(f"Drone {altitude} metreye kalkıyor...")
        
        except Exception as e:
            print(f"Komut çalıştırılırken bir hata oluştu: {e}")

    def swarm_move_function(self):
        try:
            # Drone'ların mevcut pozisyonunu al ve 5 metre ileri hedefini ayarla
            lat_change = 5 / 111320 # 5 metre ileri hareket

            x1, y1, z1 = self.drone1_lat, self.drone1_lon, self.drone1_alt
            target_x1 = x1 + lat_change
            self.send_position_target(self.drone1, target_x1, y1, z1)

            x2, y2, z2 = self.drone2_lat, self.drone2_lon, self.drone2_alt
            target_x2 = x2 + lat_change
            self.send_position_target(self.drone2, target_x2, y2, z2)

            x3, y3, z3 = self.drone3_lat, self.drone3_lon, self.drone3_alt
            target_x3 = x3 + lat_change
            self.send_position_target(self.drone3, target_x3, y3, z3)

        except Exception as e:
            print(f"Drone'lar ileri hareket ettirilirken bir hata oluştu: {e}")
   
    def land_function(self):
        try:
            for drone in [self.drone1, self.drone2, self.drone3]:
                # LAND komutunu gönder
                drone.mav.command_long_send(
                    drone.target_system,  # Hedef sistem (drone)
                    drone.target_component,  # Hedef bileşen
                    mavutil.mavlink.MAV_CMD_NAV_LAND,  # LAND komutu
                    0,  # Confirmation
                    0, 0, 0, 0,  # Parametreler (ilk dört boş geçiliyor)
                    0, 0, 0  # Latitude, Longitude, Altitude (boş geçiliyor)
                )
            print("Drone iniş moduna geçti...")
        
        except Exception as e:
            print(f"Komut çalıştırılırken bir hata oluştu: {e}")

    def disarm_function(self):
        try:
            for drone in [self.drone1, self.drone2, self.drone3]:
                # Disarm the vehicle
                drone.mav.command_long_send(
                    drone.target_system,
                    drone.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    0, 0, 0, 0, 0, 0, 0)
                # wait until disarming confirmed
                drone.motors_disarmed_wait()
                print('Drone disarmed!')
        except Exception as e:
            print(f"Komut çalıştırılırken bir hata oluştu: {e}")
    
    def continually_update_position(self):
        # Drone pozisyonlarını güncellemek için iş parçacıkları oluştur
        thread1 = threading.Thread(target=self.update_position_drone1)
        thread2 = threading.Thread(target=self.update_position_drone2)
        thread3 = threading.Thread(target=self.update_position_drone3)

        # İş parçacıklarını başlat
        thread1.start()
        thread2.start()
        thread3.start()

    def update_position_drone1(self):
        try:
            while True:
                # 1. Drone için pozisyon bilgisi al
                msg = self.drone1.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                if msg:
                    self.drone1_lat = msg.lat / 1e7  # Enlem
                    self.drone1_lon = msg.lon / 1e7  # Boylam
                    self.drone1_alt = msg.alt / 1000  # Yükseklik (metre cinsinden)
                    
                    # PyQt5 arayüzündeki etiketi güncelle
                    self.main.drone1_position_label.setText(f"1. Drone Pozisyonu: \nLatitude={self.drone1_lat}, \nLongitude={self.drone1_lon}, \nAltitude={self.drone1_alt} m")
                time.sleep(0.01)  # 1 saniye bekle
        except Exception as e:
            print(f"Drone1 konumu alınırken hata oluştu: {e}")

    def update_position_drone2(self):
        try:
            while True:
                # 2. Drone için pozisyon bilgisi al
                msg = self.drone2.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                if msg:
                    self.drone2_lat = msg.lat / 1e7  # Enlem
                    self.drone2_lon = msg.lon / 1e7  # Boylam
                    self.drone2_alt = msg.alt / 1000  # Yükseklik (metre cinsinden)
                    
                    # PyQt5 arayüzündeki etiketi güncelle
                    self.main.drone2_position_label.setText(f"2. Drone Pozisyonu: \nLatitude={self.drone2_lat}, \nLongitude={self.drone2_lon}, \nAltitude={self.drone2_alt} m")
                time.sleep(0.01)  # 1 saniye bekle
        except Exception as e:
            print(f"Drone2 konumu alınırken hata oluştu: {e}")

    def update_position_drone3(self):
        try:
            while True:
                # 3. Drone için pozisyon bilgisi al
                msg = self.drone3.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                if msg:
                    self.drone3_lat = msg.lat / 1e7  # Enlem
                    self.drone3_lon = msg.lon / 1e7  # Boylam
                    self.drone3_alt = msg.alt / 1000  # Yükseklik (metre cinsinden)
                    
                    # PyQt5 arayüzündeki etiketi güncelle
                    self.main.drone3_position_label.setText(f"3. Drone Pozisyonu: \nLatitude={self.drone3_lat}, \nLongitude={self.drone3_lon}, \nAltitude={self.drone3_alt} m")
                time.sleep(0.01)  # 1 saniye bekle
        except Exception as e:
            print(f"Drone3 konumu alınırken hata oluştu: {e}")

    def send_position_target(self, drone, lat, lon, alt):
        try:
            drone.mav.set_position_target_global_int_send(
                0,  # time_boot_ms (ignored)
                drone.target_system, drone.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Hedef çerçeve (GLOBAL_RELATIVE_ALT)
            int(0b110111111000),  # Pozisyon alanlarını kullan
            int(lat * 1e7), int(lon * 1e7), alt,  # Latitude, Longitude, Altitude
            0, 0, 0,  # X, Y, Z hızları (ignor)
            0, 0, 0,  # X, Y, Z ivmeleri (ignor)
            0, 0  # yaw, yaw_rate (ignor)
            )
        except Exception as e:
            print(f"Pozisyon gönderilirken bir hata oluştu: {e}")
    
    def run_in_new_terminal(self, command):
        try:
            subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'cd {self.working_directory} && {command}; exec bash'])
        except Exception as e:
            print(f"Komut çalıştırılırken bir hata oluştu: {e}")

def main():
    app = QApplication(sys.argv)
    launcher = LauncherAppFunctions()
    launcher.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()