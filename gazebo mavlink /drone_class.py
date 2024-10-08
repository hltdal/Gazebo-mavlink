class Drone:
    def __init__(self, udpin):
        self.mavlink_connection = None
        self.udpin = udpin
        self.lat = None
        self.lon = None
        self.alt = None
        self.vx = None
        self.vy = None
        self.vz = None