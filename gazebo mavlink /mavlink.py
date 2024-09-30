# launcher_functions.py
import sys
import subprocess
import time
import threading
from nav_msgs.msg import Odometry
from design import Ui_MainWindow
from PyQt5.QtWidgets import *
from geometry_msgs.msg import PoseStamped
# Import mavutil
from pymavlink import mavutil
import math


class LauncherAppFunctions(QMainWindow):
    def __init__(self):
        super().__init__()
        self.main = Ui_MainWindow()
        self.main.setupUi(self)
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
        self.main.emergency_button.clicked.connect(self.emergency_function)

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
        #self.continually_check_drone_state()

    def arm_function(self):
        try:
            for drone in [self.drone1, self.drone2, self.drone3]:
                drone.set_mode("GUIDED")  # GUIDED modunu seç
                print("Drone GUIDED moduna geçiyor...")
            for drone in [self.drone1, self.drone2, self.drone3]:
                # Arm the vehicle
                drone.mav.command_long_send(
                    drone.target_system,
                    drone.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    1, 0, 0, 0, 0, 0, 0)
                # wait until arming confirmed
                #drone.motors_armed_wait()
                #print('Drone armed!')
        except Exception as e:
            print(f"Komut çalıştırılırken bir hata oluştu: {e}")
    
    def get_takeoff_text(self):
        self.takeoff_value=self.main.takeoff_altitude_lineEdit.text()
        #self.takeoff_velocity=self.main.takeoff_velocity_lineEdit.text()

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

    def check_drone_state(self):
        while True:
            for i, drone in enumerate([self.drone1, self.drone2, self.drone3], 1):
                print(f"{i}. Drone durumu kontrol ediliyor...")
                # Her türlü mesajı bekle
                msg = drone.recv_match(type='HEARTBEAT', blocking=True)
                if msg:
                    base_mode = msg.base_mode
                    # MAV_MODE_FLAG_SAFETY_ARMED = 128 (0x80) -> Silahlanmış mı kontrol et
                    if base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                        print("Drone şu an silahlanmış durumda.")
                        # Havadaki durumu kontrol etmek için diğer modlara da bakabilirsiniz
                        if base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED:
                            print("Drone 'Guided' modunda ve muhtemelen havada.")
                        else:
                            print("Drone 'Guided' modunda değil.")
                    else:
                        print("Drone şu an yerde veya silahlanmamış.")
                else:
                    print(f"{i}. Drone'dan mesaj alınmadı.")
            time.sleep(1)  # 1 saniye bekle

    def swarm_move_function(self):
        try:
            # Drone'ların mevcut pozisyonunu al ve 100 metre ileri hedefini ayarla
            self.move_position_x_value = int(self.main.move_position_x_lineEdit.text())
            self.move_position_y_value = int(self.main.move_position_y_lineEdit.text())
            self.move_position_z_value = int(self.main.move_position_z_lineEdit.text())

            self.move_velocity_value = int(self.main.move_velocity_lineEdit.text())

            x1, y1, z1 = self.drone1_lat, self.drone1_lon, self.drone1_alt
            target_x1 = x1 + (self.move_position_x_value / 6378137.0) * (180 / math.pi)
            target_y1 = y1 + (self.move_position_y_value / 6378137.0) * (180 / math.pi)
            target_z1 = z1 + self.move_position_z_value 
            self.send_position_target(self.drone1, target_x1, target_y1, target_z1,self.move_velocity_value)

            x2, y2, z2 = self.drone2_lat, self.drone2_lon, self.drone2_alt
            target_x2 = x2 + (self.move_position_x_value / 6378137.0) * (180 / math.pi)
            target_y2 = y2 + (self.move_position_y_value / 6378137.0) * (180 / math.pi)
            target_z2 = z2 + self.move_position_z_value 
            self.send_position_target(self.drone2, target_x2, target_y2, target_z2,self.move_velocity_value)

            x3, y3, z3 = self.drone3_lat, self.drone3_lon, self.drone3_alt
            target_x3 = x3 + (self.move_position_x_value / 6378137.0) * (180 / math.pi)
            target_y3 = y3 + (self.move_position_y_value / 6378137.0) * (180 / math.pi)
            target_z3 = z3 + self.move_position_z_value 
            self.send_position_target(self.drone3, target_x3, target_y3, target_z3,self.move_velocity_value)
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
    
    def emergency_function(self):
        for drone in [self.drone1, self.drone2, self.drone3]:
            # BRAKE moduna geçiş yaparak drone'u acilen durdur
            drone.mav.set_mode_send(
                drone.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                drone.mode_mapping()['BRAKE']
            )
            print("Acil durdurma komutu gönderildi: BRAKE moduna geçiliyor.")

    def continually_update_position(self):
        # Drone pozisyonlarını güncellemek için iş parçacıkları oluştur
        thread1 = threading.Thread(target=self.update_position_drone1)
        thread2 = threading.Thread(target=self.update_position_drone2)
        thread3 = threading.Thread(target=self.update_position_drone3)
        thread4 = threading.Thread(target=self.check_drone_state)
        thread5 = threading.Thread(target=self.get_velocity_and_acceleration)

        # İş parçacıklarını başlat
        thread1.start()
        thread2.start()
        thread3.start()
        thread4.start()
        thread5.start()

    def get_velocity_and_acceleration(self):
        while True:
            for drone in [self.drone1, self.drone2, self.drone3]:
                # LOCAL_POSITION_NED mesajını al
                msg = drone.recv_match(type='LOCAL_POSITION_NED', blocking=True)
                if msg is not None:
                    vx = msg.vx  # X eksenindeki hız (cm/s, metre/s'ye çevir)
                    vy = msg.vy  # Y eksenindeki hız
                    vz = msg.vz  # Z eksenindeki hız
                    print(f"Velocity X: {vx} m/s, Y: {vy} m/s, Z: {vz} m/s")
            time.sleep(0.1)  # 0.1 saniye bekle

    def update_position_drone1(self):
        try:
            while True:
                """1. Drone için attitude bilgisi al
                msg=self.drone1.recv_match(type='ATTITUDE',blocking=True)
                if msg:
                    self.drone1_roll = msg.roll
                    self.drone1_pitch = msg.pitch
                    self.drone1_yaw = msg.yaw
                    print(f"1. Drone Attitude: \nRoll={self.drone1_roll}, \nPitch={self.drone1_pitch}, \nYaw={self.drone1_yaw}")
                """
                # 1. Drone için pozisyon bilgisi al
                msg = self.drone1.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                if msg:
                    self.drone1_lat = msg.lat / 1e7  # Enlem
                    self.drone1_lon = msg.lon / 1e7  # Boylam
                    self.drone1_alt = msg.relative_alt / 1000  # Yükseklik (metre cinsinden)
                    
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
                    self.drone2_alt = msg.relative_alt / 1000  # Yükseklik (metre cinsinden)
                    
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
                    self.drone3_alt = msg.relative_alt / 1000  # Yükseklik (metre cinsinden)
                    
                    # PyQt5 arayüzündeki etiketi güncelle
                    self.main.drone3_position_label.setText(f"3. Drone Pozisyonu: \nLatitude={self.drone3_lat}, \nLongitude={self.drone3_lon}, \nAltitude={self.drone3_alt} m")
                time.sleep(0.01)  # 1 saniye bekle
        except Exception as e:
            print(f"Drone3 konumu alınırken hata oluştu: {e}")

    def send_position_target(self, drone, lat, lon, alt,v):
        try:
            # Hedef pozisyon ve hız komutu gönderme
            drone.mav.send(
                drone.mav.set_position_target_global_int_encode(
                    0,  # time_boot_ms (geçerli zaman)
                    #drone.target_system,  # hedef sistem
                    0,
                    #drone.target_component,  # hedef komponent
                    0,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Küresel çerçeve (irtifa göreceli)
                    0b0000111111111000,
                    int(lat*1e7), int(lon*1e7), alt,  # Hedef enlem (10^7 ölçeğinde), boylam (10^7 ölçeğinde) ve irtifa (m)
                    v, v, v,  # Hedef hız (X, Y, Z eksenlerinde m/s cinsinden)
                    0, 0, 0,  # Hızlanma hedefi yok
                    0, 0  # Yaw ve Yaw hız hedefi yok
                )
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