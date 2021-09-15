class Radar_Data():
    def __init__(self):
        self.header = 0
        self.radar_id = 0
        self.time_unix_sec = 0
        self.radarLat_deg = 0
        self.radarLong_deg = 0
        self.radarAlt_mMSL = 0
        self.numTgts = 0
        self.drones = []

    def store_info(self, locs):
        self.header = locs[0]
        self.radar_id = locs[1]
        self.time_unix_sec = locs[2]
        self.radarLat_deg = locs[3]
        self.radarLong_deg = locs[4]
        self.radarAlt_mMSL = locs[5]
        self.numTgts = locs[6]
        for j in range(int(self.numTgts)):
            drone_info = locs[7+(3*j):10+(3*j)]
            new_drone = Drone_Radar()
            new_drone.create_drone(drone_info, self.time_unix_sec)
            self.drones.append(new_drone)
        return self


class Drone_Radar():
    def __init__(self):
        self.time_unix_sec = 0
        self.range_m = 0
        self.azimuth_deg = 0
        self.elevation_deg = 0
        self.grid_x = 0
        self.grid_y = 0

    def create_drone(self, info, time):
        self.range_m = info[0]
        self.azimuth_deg = info[1]
        self.elevation_deg = info[2]
        self.time_unix_sec = time
        return self

    def store_grid_loc(self, x, y):
        self.grid_x = x
        self.grid_y = y