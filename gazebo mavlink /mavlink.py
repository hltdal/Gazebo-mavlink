import sys
import subprocess
import time
import threading
from nav_msgs.msg import Odometry
from design import Ui_MainWindow
from PyQt5.QtWidgets import *
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
import math
from pymavlink_helper import MavlinkHelper
from drone_class import Drone

class LauncherAppFunctions(QMainWindow):
    def __init__(self):
        super().__init__()
        self.main = Ui_MainWindow()
        self.pymavlink_helper = MavlinkHelper()
        
        self.main.setupUi(self)
        self.working_directory = "~"

        self.drone1=Drone(udpin='127.0.0.1:14550')
        self.drone2=Drone(udpin='127.0.0.1:14560')
        self.drone3=Drone(udpin='127.0.0.1:14570')
        self.drones=[self.drone1,self.drone2,self.drone3]

        self.main.init_environment_button.clicked.connect(self.init_environment)
        self.main.arm_button.clicked.connect(self.arm)
        self.main.force_arm_button.clicked.connect(self.force_arm)
        self.main.disarm_button.clicked.connect(self.disarm)
        self.main.force_disarm_button.clicked.connect(self.force_disarm)
        self.main.takeoff_button.clicked.connect(self.take_off)
        self.main.land_button.clicked.connect(self.land)
        self.main.swarm_move_button.clicked.connect(self.swarm_move)
        self.main.emergency_button.clicked.connect(self.emergency)

    def init_environment(self):
        command = "roslaunch iq_sim multi_drone.launch"
        self.run_in_new_terminal(command)
        command = "./multi_sitl.sh"
        self.run_in_new_terminal(command)
        for drone in self.drones:
            drone.mavlink_connection=self.pymavlink_helper.connection(drone.mavlink_connection, drone.udpin)
        print("Drone'lar bağlandı")
        self.continually_update_position()

    def arm(self):
        try:
            for drone in self.drones:
                self.pymavlink_helper.arm(drone.mavlink_connection, is_force=False)
        except Exception as e:
            print(f"Arm komutu çalıştırılırken bir hata oluştu: {e}")
    
    def force_arm(self):
        try:
            for drone in self.drones:
                self.pymavlink_helper.arm(drone.mavlink_connection, is_force=True)
        except Exception as e:
            print(f"Force arm komutu çalıştırılırken bir hata oluştu: {e}")
                  
    def disarm(self):
        try:
            for drone in self.drones:
                self.pymavlink_helper.disarm(drone.mavlink_connection, is_force=False)
        except Exception as e:
            print(f"Disarm komutu çalıştırılırken bir hata oluştu: {e}")
    
    def force_disarm(self):
        try:
            for drone in self.drones:
                self.pymavlink_helper.disarm(drone.mavlink_connection, is_force=True)
        except Exception as e:
            print(f"Force disarm komutu çalıştırılırken bir hata oluştu: {e}")

    def take_off(self):
        try:
            altitude = float(self.main.takeoff_altitude_lineEdit.text())
            for drone in self.drones:
                self.pymavlink_helper.takeoff(drone.mavlink_connection, altitude)
        except Exception as e:
            print(f"Takeoff komutu çalıştırılırken bir hata oluştu: {e}")

    def land(self):
        try:
            for drone in self.drones:
                self.pymavlink_helper.land(drone.mavlink_connection)
        except Exception as e:
            print(f"Land komutu çalıştırılırken bir hata oluştu: {e}")

    def emergency(self):
        try:
            for drone in self.drones:
                self.pymavlink_helper.brake(drone.mavlink_connection)
            time.sleep(4) #Tercih edilebilir
            for drone in self.drones:
                self.pymavlink_helper.land(drone.mavlink_connection)
        except Exception as e:
            print(f"Emergency komutu çalıştırılırken bir hata oluştu: {e}")

    def swarm_move(self):
        try:
            self.move_position_x_value = int(self.main.move_position_x_lineEdit.text())
            self.move_position_y_value = int(self.main.move_position_y_lineEdit.text())
            self.move_position_z_value = int(self.main.move_position_z_lineEdit.text())
            self.move_velocity_value = int(self.main.move_velocity_lineEdit.text())

            for drone in self.drones:
                target_x1=drone.lat + (self.move_position_x_value / 6378137.0) * (180 / math.pi)
                target_y1=drone.lon + (self.move_position_y_value / 6378137.0) * (180 / math.pi)
                target_z1=drone.alt + self.move_position_z_value
                self.send_position_target(drone.mavlink_connection, target_x1, target_y1, target_z1, self.move_velocity_value)
        except Exception as e:
            print(f"Swarm move çalıştırılırken bir hata oluştu: {e}")

    def get_velocity(self):
        try:
            while True:
                for drone in self.drones:
                    drone.vx, drone.vy, drone.vz=self.pymavlink_helper.get_velocity_mavlink(drone.mavlink_connection)
                    if (drone.vx, drone.vy, drone.vz) is not None:
                        if drone == self.drone1:
                            self.main.drone1_velocity_label.setText(f"  1. Drone Hızı:  \nX={drone.vx:.3f} m/s  \nY={drone.vy:.2f} m/s  \nZ={drone.vz:.2f} m/s  ")
                        elif drone == self.drone2:
                            self.main.drone2_velocity_label.setText(f"  2. Drone Hızı:  \nX={drone.vx:.2f} m/s  \nY={drone.vy:.2f} m/s  \nZ={drone.vz:.2f} m/s  ")
                        elif drone == self.drone3:
                            self.main.drone3_velocity_label.setText(f"  3. Drone Hızı:  \nX={drone.vx:.2f} m/s  \nY={drone.vy:.2f} m/s  \nZ={drone.vz:.2f} m/s  ")
        except Exception as e:
            print(f"Drone hızı alınırken hata oluştu: {e}")
    
    def update_position(self):
        try:
            while True:
                for drone in self.drones:
                    drone.lat, drone.lon, drone.alt = self.pymavlink_helper.update_position_mavlink(drone.mavlink_connection)
                    if drone == self.drone1:
                        self.main.drone1_position_label.setText(f"1. Drone Pozisyonu: \nLatitude={drone.lat}, \nLongitude={drone.lon}, \nAltitude={drone.alt} m")
                    elif drone == self.drone2:
                        self.main.drone2_position_label.setText(f"2. Drone Pozisyonu: \nLatitude={drone.lat}, \nLongitude={drone.lon}, \nAltitude={drone.alt} m")
                    elif drone == self.drone3:
                        self.main.drone3_position_label.setText(f"3. Drone Pozisyonu: \nLatitude={drone.lat}, \nLongitude={drone.lon}, \nAltitude={drone.alt} m")
        except Exception as e:
            print(f"Drone konumu alınırken hata oluştu: {e}")

    def send_position_target(self, drone, lat, lon, alt, v):
        try:
            self.pymavlink_helper.position_target(drone, lat, lon, alt)
            self.pymavlink_helper.set_velocity(drone, v)
        except Exception as e:
            print(f"Pozisyon gönderilirken bir hata oluştu: {e}")
    
    def continually_update_position(self):
        thread1 = threading.Thread(target=self.update_position)
        thread2 = threading.Thread(target=self.get_velocity)

        thread1.start()
        thread2.start()

    def run_in_new_terminal(self, command):
        try:
            subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'cd {self.working_directory} && {command}; exec bash'])
        except Exception as e:
            print(f"Terminal komutu çalıştırılırken bir hata oluştu: {e}")

def main():
    app = QApplication(sys.argv)
    launcher = LauncherAppFunctions()
    launcher.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()