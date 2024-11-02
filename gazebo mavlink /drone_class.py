class Drone:
    def __init__(self,name, udpin):
        self.mavlink_connection = None
        self.udpin = udpin
        self.name = name
        self.lat = None
        self.lon = None
        self.alt = None
        self.vx = None
        self.vy = None
        self.vz = None
        self.spawn_position_x = None
        self.spawn_position_y = None