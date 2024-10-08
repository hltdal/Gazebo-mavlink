from abc import ABC, abstractmethod
from pymavlink import mavutil
import pymavlink
import pymavlink.mavutil


class MavlinkHelper:
    def __init__(self):
        pass

    @abstractmethod
    def connection(self,drone: None , udpin: str) -> pymavlink.mavutil.mavudp:
        drone = mavutil.mavlink_connection(udpin)
        drone.wait_heartbeat()
        return drone

    @abstractmethod
    def arm_disarm(self, drone: pymavlink.mavutil.mavudp ,is_arm: bool, is_force: bool) -> None:
        drone.set_mode("GUIDED")
        if is_arm:
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
        else:
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

    @abstractmethod
    def takeoff(self, drone: pymavlink.mavutil.mavudp , altitude: float) -> None:
        drone.mav.command_long_send(
                    drone.target_system,
                    drone.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0, 0, 0, 0, 0, 0, 0, altitude)
    
    @abstractmethod
    def land(self, drone: pymavlink.mavutil.mavudp ) -> None:
        drone.mav.command_long_send(
                    drone.target_system,
                    drone.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_LAND,
                    0, 0, 0, 0, 0, 0, 0, 0)

    @abstractmethod
    def brake(self, drone: pymavlink.mavutil.mavudp ) -> None:
        drone.mav.set_mode_send(
                drone.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                drone.mode_mapping()['BRAKE']
            )

    @abstractmethod
    def get_velocity_mavlink(self, drone: pymavlink.mavutil.mavudp) -> tuple:
        msg = drone.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        if msg is not None:
            vx = msg.vx
            vy = msg.vy
            vz = msg.vz
        return vx, vy, -vz

    @abstractmethod
    def update_position_mavlink(self, drone: pymavlink.mavutil.mavudp) -> tuple:
        msg = drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg is not None:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000
        return lat, lon, alt

    @abstractmethod
    def position_target(self, drone: pymavlink.mavutil.mavudp, lat: float, lon: float, alt: float) -> None:
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

    @abstractmethod
    def set_velocity(self, drone: pymavlink.mavutil.mavudp, v: float) -> None:
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