import math


def ecef(latitude, longitude, altitude):
    latitude, longitude = map(math.radians, (latitude, longitude))
    eccentricity_squared = 6.6943799901413165e-3
    radius = 6378137.0 / math.sqrt(
        1.0 - eccentricity_squared * math.sin(latitude) ** 2
    )
    return (
        (radius + altitude) * math.cos(latitude) * math.cos(longitude),
        (radius + altitude) * math.cos(latitude) * math.sin(longitude),
        (radius * (1.0 - eccentricity_squared) + altitude) * math.sin(latitude),
    )


class GpsProjection:
    def __init__(self, latitude, longitude, altitude, heading_deg=0.0):
        self.origin = ecef(latitude, longitude, altitude)
        self.latitude = math.radians(latitude)
        self.longitude = math.radians(longitude)
        self.heading = math.radians(heading_deg)
        self.altitude = altitude

    def to_world(self, latitude, longitude, altitude=None):
        position = ecef(
            latitude, longitude, self.altitude if altitude is None else altitude
        )
        dx, dy, dz = (p - o for p, o in zip(position, self.origin))
        east = -math.sin(self.longitude) * dx + math.cos(self.longitude) * dy
        north = (
            -math.sin(self.latitude) * math.cos(self.longitude) * dx
            - math.sin(self.latitude) * math.sin(self.longitude) * dy
            + math.cos(self.latitude) * dz
        )
        return (
            -math.cos(self.heading) * east + math.sin(self.heading) * north,
            -math.sin(self.heading) * east - math.cos(self.heading) * north,
        )
