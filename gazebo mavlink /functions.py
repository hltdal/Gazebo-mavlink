import time
from pymavlink_helper import MavlinkHelper

class drone_functions:
    def __init__(self):
        self.pymavlink_helper= MavlinkHelper()
        pass

    def arm(self, drones):
        try:
            for drone in drones:
                self.pymavlink_helper.arm_disarm(drone.mavlink_connection, is_arm=True, is_force=False)
        except Exception as e:
            print(f"Arm komutu çalıştırılırken bir hata oluştu: {e}")

    def force_arm(self, drones):
        try:
            for drone in drones:
                self.pymavlink_helper.arm_disarm(drone.mavlink_connection, is_arm=True, is_force=True)
        except Exception as e:
            print(f"Force arm komutu çalıştırılırken bir hata oluştu: {e}")

    def disarm(self, drones):
        try:
            for drone in drones:
                self.pymavlink_helper.arm_disarm(drone.mavlink_connection, is_arm=False, is_force=False)
        except Exception as e:
            print(f"Disarm komutu çalıştırılırken bir hata oluştu: {e}")

    def force_disarm(self, drones):
        try:
            for drone in drones:
                self.pymavlink_helper.arm_disarm(drone.mavlink_connection, is_arm=False, is_force=True)
        except Exception as e:
            print(f"Force disarm komutu çalıştırılırken bir hata oluştu: {e}")

    def takeoff(self, drones, altitude):
        try:
            for drone in drones:
                self.pymavlink_helper.takeoff(drone.mavlink_connection, altitude)
        except Exception as e:
            print(f"Takeoff komutu çalıştırılırken bir hata oluştu: {e}")

    def land(self, drones):
        try:
            for drone in drones:
                self.pymavlink_helper.land(drone.mavlink_connection)
        except Exception as e:
            print(f"Land komutu çalıştırılırken bir hata oluştu: {e}")

    def emergency(self, drones):
        try:
            for drone in drones:
                self.pymavlink_helper.brake(drone.mavlink_connection)
            time.sleep(4)  # Tercih edilebilir
            for drone in drones:
                self.pymavlink_helper.land(drone.mavlink_connection)
        except Exception as e:
            print(f"Emergency komutu çalıştırılırken bir hata oluştu: {e}")
