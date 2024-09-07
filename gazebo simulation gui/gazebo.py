# launcher_functions.py
import sys
import subprocess
import time
import rospy
from nav_msgs.msg import Odometry
from design import Ui_MainWindow
from PyQt5.QtWidgets import *
from geometry_msgs.msg import PoseStamped


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
        self.main.launch_mavros_button.clicked.connect(self.launch_mavros)
        self.main.run_file_button.clicked.connect(self.run_file)
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

    def launch_mavros(self):
        command = "roslaunch iq_sim multi-apm.launch"
        self.run_in_new_terminal(command)
        
        # MAVROS düğümünü başlattıktan sonra ROS düğümünü çalıştır
        time.sleep(1)  # MAVROS'un tamamen başlatılması için bekleyin
        self.start_ros_node()

    def run_file(self):
        command = "roslaunch iq_gnc multi_square.launch"
        self.run_in_new_terminal(command)

    def arm_function(self):
        try:
            # Modu 'guided' yap
            for i in range(3):
                tmux_command = f"tmux send-keys -t sitl_session.{i} 'mode guided' C-m"
                subprocess.run(tmux_command, shell=True, check=True)

            time.sleep(1)  # Droneların moda geçmesi için zaman tanıyın
            
            # Throttle arm komutu gönder
            for i in range(3):
                tmux_command = f"tmux send-keys -t sitl_session.{i} 'arm throttle' C-m"
                subprocess.run(tmux_command, shell=True, check=True)
        except Exception as e:
            print(f"Komut çalıştırılırken bir hata oluştu: {e}")
    
    def get_takeoff_text(self):
        self.takeoff_value=self.main.takeoff_lineEdit.text()

    def take_off_function(self):
        try:
            time.sleep(1)
            # Takeoff komutu gönder
            for i in range(3):
                tmux_command = f"tmux send-keys -t sitl_session.{i} 'takeoff {self.takeoff_value}' C-m"
                subprocess.run(tmux_command, shell=True, check=True)
        
        except Exception as e:
            print(f"Komut çalıştırılırken bir hata oluştu: {e}")
    
    def swarm_move_function(self):
        if (self.drone1_position is None or self.drone2_position is None or self.drone3_position is None):
            print("Konumu alinmayan drone var.")
            return

        try:
            target_position_1 = PoseStamped()
            target_position_1.pose.position.x = self.drone1_position.x + 5.0  # X ekseninde 5 birim ileri
            target_position_1.pose.position.y = self.drone1_position.y
            target_position_1.pose.position.z = self.drone1_position.z  # Aynı irtifayı koru
            target_position_1.pose.orientation = self.drone1_orientation  # Orientation doğru şekilde atanıyor

            target_position_2=PoseStamped()
            target_position_2.pose.position.x = self.drone2_position.x + 5.0  # X ekseninde 5 birim ileri
            target_position_2.pose.position.y = self.drone2_position.y
            target_position_2.pose.position.z = self.drone2_position.z  # Aynı irtifayı koru
            target_position_2.pose.orientation = self.drone2_orientation  # Orientation doğru şekilde atanıyor

            target_position_3=PoseStamped()
            target_position_3.pose.position.x = self.drone3_position.x + 5.0  # X ekseninde 5 birim ileri
            target_position_3.pose.position.y = self.drone3_position.y
            target_position_3.pose.position.z = self.drone3_position.z  # Aynı irtifayı koru
            target_position_3.pose.orientation = self.drone3_orientation  # Orientation doğru şekilde atanıyor

            print(f"Drone 1 hedef pozisyon: x={target_position_1.pose.position.x}, y={target_position_1.pose.position.y}")
            print(f"Drone 2 hedef pozisyon: x={target_position_2.pose.position.x}, y={target_position_2.pose.position.y}")
            print(f"Drone 3 hedef pozisyon: x={target_position_3.pose.position.x}, y={target_position_3.pose.position.y}")
            
            for _ in range(100):  # Setpoint'i sürekli olarak gönder, drone harekete geçene kadar
                self.drone1_setpoint_pub.publish(target_position_1)
                self.drone2_setpoint_pub.publish(target_position_2)
                self.drone3_setpoint_pub.publish(target_position_3)
                time.sleep(0.1)

        except Exception as e:
            print(f"Drone 1 ileri hareket ettirilirken hata oluştu: {e}")
    
    def land_function(self):
        try:
            time.sleep(1)
            # Takeoff komutu gönder
            for i in range(3):
                tmux_command = f"tmux send-keys -t sitl_session.{i} 'mode land' C-m"
                subprocess.run(tmux_command, shell=True, check=True)
        
        except Exception as e:
            print(f"Komut çalıştırılırken bir hata oluştu: {e}")

    def disarm_function(self):
        try:
            time.sleep(1)
            # Takeoff komutu gönder
            for i in range(3):
                tmux_command = f"tmux send-keys -t sitl_session.{i} 'disarm' C-m"
                subprocess.run(tmux_command, shell=True, check=True)
        
        except Exception as e:
            print(f"Komut çalıştırılırken bir hata oluştu: {e}")

    def start_ros_node(self):
        # ROS düğümünü başlat
        try:
            rospy.init_node('drone_position_node', anonymous=True)
            rospy.Subscriber('/drone1/mavros/local_position/odom', Odometry, self.update_position_drone1)
            rospy.Subscriber('/drone2/mavros/local_position/odom', Odometry, self.update_position_drone2)
            rospy.Subscriber('/drone3/mavros/local_position/odom', Odometry, self.update_position_drone3)

            self.drone1_setpoint_pub = rospy.Publisher('/drone1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
            self.drone2_setpoint_pub = rospy.Publisher('/drone2/mavros/setpoint_position/local', PoseStamped, queue_size=10)
            self.drone3_setpoint_pub = rospy.Publisher('/drone3/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        except rospy.ROSInterruptException as e:
            print(f"ROS düğümü başlatılamadı: {e}")

    def update_position_drone1(self, data):
        # Drone1 pozisyonunu güncelle
        self.drone1_position = data.pose.pose.position
        self.drone1_orientation = data.pose.pose.orientation  # Orientation bilgisini ekle
        self.main.drone1_position_label.setText(f"1. Drone Pozisyonu: x={self.drone1_position.x:.2f}, y={self.drone1_position.y:.2f}, z={self.drone1_position.z:.2f}")

    def update_position_drone2(self, data):
        # Drone2 pozisyonunu güncelle
        self.drone2_position = data.pose.pose.position
        self.drone2_orientation = data.pose.pose.orientation  # Orientation bilgisini ekle
        self.main.drone2_position_label.setText(f"2. Drone Pozisyonu: x={self.drone2_position.x:.2f}, y={self.drone2_position.y:.2f}, z={self.drone2_position.z:.2f}")

    def update_position_drone3(self, data):
        # Drone3 pozisyonunu güncelle
        self.drone3_position = data.pose.pose.position
        self.drone3_orientation = data.pose.pose.orientation  # Orientation bilgisini ekle
        self.main.drone3_position_label.setText(f"3. Drone Pozisyonu: x={self.drone3_position.x:.2f}, y={self.drone3_position.y:.2f}, z={self.drone3_position.z:.2f}")

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