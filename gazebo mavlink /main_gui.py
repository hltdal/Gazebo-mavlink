import sys
import subprocess
import time
import threading
from PyQt5 import QtWidgets
from design import Ui_MainWindow
from PyQt5.QtWidgets import *
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import xml.etree.ElementTree as ET
from pymavlink_helper import MavlinkHelper
from drone_class import Drone
from individual_design import Ui_Dialog as Ui_Secondwindow
from functions import drone_functions

class LauncherAppFunctions(QMainWindow):
    def __init__(self):
        super().__init__()
        self.main = Ui_MainWindow()
        self.pymavlink_helper = MavlinkHelper()
        self.appointments = drone_functions()

        self.main.setupUi(self)
        self.working_directory = "~"

        self.drone1=Drone(name="drone1", udpin='127.0.0.1:14550')
        self.drone2=Drone(name="drone2", udpin='127.0.0.1:14560')
        self.drone3=Drone(name="drone3", udpin='127.0.0.1:14570')
        self.drones=[self.drone1,self.drone2,self.drone3]
        self.initUI()

        self.main.init_environment_button.clicked.connect(self.init_environment)
        self.main.arm_button.clicked.connect(self.arm_all)
        self.main.force_arm_button.clicked.connect(self.force_arm)
        self.main.disarm_button.clicked.connect(self.disarm)
        self.main.force_disarm_button.clicked.connect(self.force_disarm)
        self.main.takeoff_button.clicked.connect(self.take_off)
        self.main.land_button.clicked.connect(self.land)
        self.main.swarm_move_button.clicked.connect(self.swarm_move)
        self.main.emergency_button.clicked.connect(self.emergency)
        self.main.command_window_button.clicked.connect(self.open_second_window)

    def initUI(self):
        self.main.horizontalLayout_6.addWidget(CoordinatePlane(self.drones))

    def init_environment(self):
        command = "roslaunch iq_sim multi_drone.launch"
        self.run_in_new_terminal(command)
        command = "./multi_sitl.sh"
        self.run_in_new_terminal(command)
        for drone in self.drones:
            drone.mavlink_connection=self.pymavlink_helper.connection(drone.mavlink_connection, drone.udpin)
        print("Drone'lar bağlandı")
        self.continually_update_position()

    def arm_all(self):
        self.appointments.arm(self.drones)

    def force_arm(self):
        self.appointments.force_arm(self.drones)

    def disarm(self):
        self.appointments.disarm(self.drones)

    def force_disarm(self):
        self.appointments.force_disarm(self.drones)

    def take_off(self):
        altitude = float(self.main.takeoff_altitude_lineEdit.text())
        self.appointments.takeoff(self.drones, altitude)

    def land(self):
        self.appointments.land(self.drones)

    def emergency(self):
        self.appointments.emergency(self.drones)

    def swarm_move(self):
        try:
            move_position_x_value = float(self.main.move_position_x_lineEdit.text())
            move_position_y_value = float(self.main.move_position_y_lineEdit.text())
            move_position_z_value = float(self.main.move_position_z_lineEdit.text())
            move_velocity_value = float(self.main.move_velocity_lineEdit.text())

            self.appointments.move(self.drones, move_position_x_value, move_position_y_value, move_position_z_value, move_velocity_value)

        except Exception as e:
            print(f"Swarm move çalıştırılırken bir hata oluştu: {e}")

    def update_velocity(self):
        try:
            while True:
                for drone in self.drones:
                    drone.vx, drone.vy, drone.vz=self.pymavlink_helper.get_velocity(drone.mavlink_connection)
                    if (drone.vx, drone.vy, drone.vz) is not None:
                        if drone == self.drone1:
                            self.main.drone1_velocity_label.setText(f"  1. Drone Hızı:  \nX={drone.vx:.3f} m/s  \nY={drone.vy:.2f} m/s  \nZ={drone.vz:.2f} m/s  ")
                        elif drone == self.drone2:
                            self.main.drone2_velocity_label.setText(f"  2. Drone Hızı:  \nX={drone.vx:.2f} m/s  \nY={drone.vy:.2f} m/s  \nZ={drone.vz:.2f} m/s  ")
                        elif drone == self.drone3:
                            self.main.drone3_velocity_label.setText(f"  3. Drone Hızı:  \nX={drone.vx:.2f} m/s  \nY={drone.vy:.2f} m/s  \nZ={drone.vz:.2f} m/s  ")
                time.sleep(0.01)
        except Exception as e:
            print(f"Drone hızı alınırken hata oluştu: {e}")

    def update_position(self):
        try:
            while True:
                for drone in self.drones:
                    drone.lat, drone.lon, drone.alt = self.pymavlink_helper.get_position(drone.mavlink_connection)
                    if drone == self.drone1:
                        self.main.drone1_position_label.setText(f"1. Drone Pozisyonu: \nLatitude={drone.lat}, \nLongitude={drone.lon}, \nAltitude={drone.alt} m")
                    elif drone == self.drone2:
                        self.main.drone2_position_label.setText(f"2. Drone Pozisyonu: \nLatitude={drone.lat}, \nLongitude={drone.lon}, \nAltitude={drone.alt} m")
                    elif drone == self.drone3:
                        self.main.drone3_position_label.setText(f"3. Drone Pozisyonu: \nLatitude={drone.lat}, \nLongitude={drone.lon}, \nAltitude={drone.alt} m")
                time.sleep(0.01)
        except Exception as e:
            print(f"Drone konumu alınırken hata oluştu: {e}")

    def continually_update_position(self):
        thread1 = threading.Thread(target=self.update_position)
        thread2 = threading.Thread(target=self.update_velocity)

        thread1.start()
        thread2.start()

    def run_in_new_terminal(self, command):
        try:
            subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'cd {self.working_directory} && {command}; exec bash'])
        except Exception as e:
            print(f"Terminal komutu çalıştırılırken bir hata oluştu: {e}")

    def open_second_window(self):
        try:
            selected_index = self.main.comboBox.currentIndex()
            selected_drone = self.drones[selected_index]
            self.second_window=SecondWindow(selected_drone, self.pymavlink_helper)
            self.second_window.exec_()
        except Exception as e:
            print(f"İkinci pencere açılırken bir hata oluştu: {e}")

class SecondWindow(QDialog):
    def __init__(self, selected_drone, pymavlink_helper):
        super().__init__()
        self.main=Ui_Secondwindow()
        self.appointments=drone_functions()
        self.main.setupUi(self)

        self.selected_drone=[]
        self.selected_drone.append(selected_drone)
        self.pymavlink_helper=pymavlink_helper
        self.main.individual_arm_button.clicked.connect(self.arm_individual)
        self.main.individual_force_arm_button.clicked.connect(self.force_arm_individual)
        self.main.individual_disarm_button.clicked.connect(self.disarm_individual)
        self.main.individual_force_disarm_button.clicked.connect(self.force_disarm_individual)
        self.main.individual_takeoff_button.clicked.connect(self.takeoff_individual)
        self.main.individual_land_button.clicked.connect(self.land_individual)
        self.main.individual_emergency_button.clicked.connect(self.emergency_individual)
        self.main.individual_move_button.clicked.connect(self.move)

    def arm_individual(self):
        self.appointments.arm(self.selected_drone)

    def force_arm_individual(self):
        self.appointments.force_arm(self.selected_drone)

    def disarm_individual(self):
        self.appointments.disarm(self.selected_drone)

    def force_disarm_individual(self):
        self.appointments.force_disarm(self.selected_drone)

    def takeoff_individual(self):
        altitude = float(self.main.individual_takeoff_altitude_lineEdit.text())
        self.appointments.takeoff(self.selected_drone, altitude)

    def land_individual(self):
        self.appointments.land(self.selected_drone)

    def emergency_individual(self):
        self.appointments.emergency(self.selected_drone)

    def move(self):
        try:
            move_position_x_value=float(self.main.individual_move_position_x_lineEdit.text())
            move_position_y_value=float(self.main.individual_move_position_y_lineEdit.text())
            move_position_z_value=float(self.main.individual_move_position_z_lineEdit.text())
            move_velocity_value = float(self.main.individual_move_velocity_lineEdit.text())

            self.appointments.move(self.selected_drone, move_position_x_value, move_position_y_value, move_position_z_value, move_velocity_value)

        except Exception as e:
            print(f"Individual move emri verilirken hata oluştu{e}")

class CoordinatePlane(FigureCanvas):
    def __init__(self, drones):
        # Matplotlib figürü oluştur
        self.fig = Figure()
        self.ax = self.fig.add_subplot(111)
        self.drones=drones

        # Ekseni ortalamak için çizgiler
        self.ax.axhline(0, color='black', lw=1)
        self.ax.axvline(0, color='black', lw=1)

        # Eksen sınırları
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)

        super().__init__(self.fig)

        # Tıklanan noktalar ve tıklama sayısı
        self.click_points = []
        self.max_clicks = 3

        # Tıklama olayını bağla
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)

    def on_click(self, event):
        try:
            # Eksen üzerinde tıklanmış mı kontrol et
            if event.inaxes == self.ax and len(self.click_points) < self.max_clicks:
                x, y = event.xdata, event.ydata
                self.click_points.append((x, y))

                # Noktayı çizin
                self.ax.plot(x, y, 'ro')  # Kırmızı noktalar
                self.draw()  # Grafiği güncelle

                print(f"Tıklama {len(self.click_points)}: ({x:.2f}, {y:.2f})")

                # Tıklama sınırına ulaşıldığında mesaj
                if len(self.click_points) == self.max_clicks:
                    print("Üç tıklama tamamlandı!")
                    self.spawn_points(self.click_points)

            elif len(self.click_points) >= self.max_clicks:
                print("Tıklama sınırına ulaşıldı!")
        except Exception as e:
            print(f"Paint area'ya tıklama sırasında bir hata oluştu: {e}")

    def spawn_points(self, points):
        try:
            for drone in self.drones:
                drone.spawn_position_x=round(points[self.drones.index(drone)][0],2)
                drone.spawn_position_y=round(points[self.drones.index(drone)][1],2)
                print(f"{drone.name} noktası: ({drone.spawn_position_x}, {drone.spawn_position_y})")
            print("Drone'lar belirlenen konumlara yerleştirildi")
            file_path="/home/halit/catkin_ws/src/iq_sim/worlds/multi_drone.world"
            tree=ET.parse(file_path)
            root=tree.getroot()
            # Drone'ların pozisyonlarını drone nesnelerinden alarak güncelle
            for model in root.findall(".//model"):
                name = model.get("name")
                for drone in self.drones:
                    if name == drone.name:  # Drone adı eşleşiyorsa pozisyon güncellenir
                        pose = model.find("pose")
                        if pose is not None:
                            # Güncellenmiş pozisyonları drone nesnesinden al
                            new_position = f"{drone.spawn_position_x} {drone.spawn_position_y} 0 0 0 0"
                            pose.text = new_position
            # Değişiklikleri kaydet
            tree.write(file_path)
            print("Pozisyonlar .world dosyasına kaydedildi.")
        except Exception as e:
            print(f"Spawn noktaları belirlenirken bir hata oluştu: {e}")
def main():
    app = QApplication(sys.argv)
    launcher = LauncherAppFunctions()
    launcher.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()