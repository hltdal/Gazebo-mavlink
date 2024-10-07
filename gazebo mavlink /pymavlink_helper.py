from pymavlink import mavutil



class MavlinkHelper:
    def __init__(self):
        pass

    def connection(self,drone,udpin):
        drone = mavutil.mavlink_connection(udpin)
        drone.wait_heartbeat()
        return drone

    def arm(self, drone, is_force=False):
        drone.set_mode("GUIDED")
        if is_force:
            drone.mav.command_long_send(
                    drone.target_system,
                    drone.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1, 21196, 0, 0, 0, 0, 0)
        else:
            drone.mav.command_long_send(
                    drone.target_system,
                    drone.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1, 0, 0, 0, 0, 0, 0)

    def disarm(self, drone, is_force=False):
        if is_force:
            drone.mav.command_long_send(
                    drone.target_system,
                    drone.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 0, 21196, 0, 0, 0, 0, 0)
        else:
            drone.mav.command_long_send(
                    drone.target_system,
                    drone.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 0, 0, 0, 0, 0, 0, 0)

    def takeoff(self, drone, altitude):
        drone.mav.command_long_send(
                    drone.target_system,
                    drone.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0, 0, 0, 0, 0, 0, 0, altitude)
    
    def land(self, drone):
        drone.mav.command_long_send(
                    drone.target_system,
                    drone.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_LAND,
                    0, 0, 0, 0, 0, 0, 0, 0)
    
    def brake(self, drone):
        drone.mav.set_mode_send(
                drone.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                drone.mode_mapping()['BRAKE']
            )

    def get_velocity_mavlink(self, drone):
        msg = drone.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        if msg is not None:
            vx = msg.vx
            vy = msg.vy
            vz = msg.vz
        return vx, vy, -vz
        
    def update_position_mavlink(self, drone):
        msg = drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg is not None:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000
        return lat, lon, alt

    def position_target(self, drone, lat, lon, alt):
        drone.mav.send(
                drone.mav.set_position_target_global_int_encode(
                    0,
                    drone.target_system,
                    drone.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    0b0000111111111000,
                    int(lat*1e7), int(lon*1e7), alt,
                    0, 0, 0,
                    0, 0, 0,
                    0, 0  
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
