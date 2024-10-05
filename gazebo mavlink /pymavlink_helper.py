from pymavlink import mavutil



class MavlinkHelper:
    def __init__(self):
        pass

    """def connection(self,drone,udpin):
        # Connect to the drone
        drone = mavutil.mavlink_connection(udpin)
        drone.wait_heartbeat()"""

    def arm(self, drone):
        # Set mode to guided
        drone.set_mode("GUIDED")
        # Arm command
        drone.mav.command_long_send(
                    drone.target_system,
                    drone.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1, 0, 0, 0, 0, 0, 0) 

    def disarm(self, drone):
        # Disarm command
        drone.mav.command_long_send(
                    drone.target_system,
                    drone.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 0, 0, 0, 0, 0, 0, 0)

    def takeoff(self, drone, altitude):
        # Takeoff command
        drone.mav.command_long_send(
                    drone.target_system,
                    drone.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0, 0, 0, 0, 0, 0, 0, altitude)
    
    def land(self, drone):
        # Land command
        drone.mav.command_long_send(
                    drone.target_system,
                    drone.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_LAND,
                    0, 0, 0, 0, 0, 0, 0, 0)
    
    def brake(self, drone):
        # Set mode to brake
        drone.mav.set_mode_send(
                drone.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                drone.mode_mapping()['BRAKE']
            )

    def local_position_ned(self, drone):
        # Get local position ned
        msg = drone.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        if msg is not None:
            vx = msg.vx  # X eksenindeki hız (cm/s, metre/s'ye çevir)
            vy = msg.vy  # Y eksenindeki hız
            vz = msg.vz  # Z eksenindeki hız
            print(f"Velocity X: {vx} m/s, Y: {vy} m/s, Z: {vz} m/s")

