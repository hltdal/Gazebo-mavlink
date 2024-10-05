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

    def get_velocity_mavlink(self, drone):
        # Get local position ned
        msg = drone.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        if msg is not None:
            vx = msg.vx  # X eksenindeki hız (cm/s, metre/s'ye çevir)
            vy = msg.vy  # Y eksenindeki hız
            vz = msg.vz  # Z eksenindeki hız
            return vx, vy, vz

    def position_target(self, drone, lat, lon, alt):
        # Send position target
        drone.mav.send(
                drone.mav.set_position_target_global_int_encode(
                    0,  # time_boot_ms (geçerli zaman)
                    drone.target_system,  # hedef sistem
                    drone.target_component,  # hedef komponent
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Küresel çerçeve (irtifa göreceli)
                    0b0000111111111000,
                    int(lat*1e7), int(lon*1e7), alt,  # Hedef enlem (10^7 ölçeğinde), boylam (10^7 ölçeğinde) ve irtifa (m)
                    0, 0, 0,  # Hedef hız (X, Y, Z eksenlerinde m/s cinsinden)
                    0, 0, 0,  # Hızlanma hedefi yok
                    0, 0  # Yaw ve Yaw hız hedefi yok
                )
            )

    def set_velocity(self, drone, v):
        drone.mav.command_long_send(
                drone.target_system,
                drone.target_component,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                0,
                0,  
                v, 
                0,  
                0, 0, 0, 0 
            )