from collections import namedtuple
import math

utmCoord_t = namedtuple("utmCoord_t", "UTMNorthing UTMEasting UTMZoneNumber UTMZoneLatDes")
map_coordinate = namedtuple("map_coordinate", "x y")

PI = math.pi
FOURTHPI = PI / 4
deg2rad = PI / 180.0
rad2deg = 180.0 / PI
equatorialRadius = 6378137.0
eccentricitySquared = 0.00669438
maxWGS84Val = 10000000


def UTMLetterDesignator (Lat):

    LetterDesignator = None

    if 72 <= Lat <= 84: LetterDesignator = 'X'
    elif 72 > Lat and Lat >= 64: LetterDesignator = 'W'
    elif 64 > Lat and Lat >= 56: LetterDesignator = 'V'
    elif 56 > Lat and Lat >= 48: LetterDesignator = 'U'
    elif 48 > Lat and Lat >= 40: LetterDesignator = 'T'
    elif 40 > Lat and Lat >= 32: LetterDesignator = 'S'
    elif 32 > Lat and Lat >= 24: LetterDesignator = 'R'
    elif 24 > Lat and Lat >= 16: LetterDesignator = 'Q'
    elif 16 > Lat and Lat >= 8: LetterDesignator = 'P'
    elif 8 > Lat and Lat >= 0: LetterDesignator = 'N'
    elif 0 > Lat and Lat >= -8: LetterDesignator = 'M'
    elif -8 > Lat and Lat >= -16:  LetterDesignator = 'L'
    elif -16 > Lat and Lat >= -24:  LetterDesignator = 'K'
    elif -24 > Lat and Lat >= -32:  LetterDesignator = 'J'
    elif -32 > Lat and Lat >= -40:  LetterDesignator = 'H'
    elif -40 > Lat and Lat >= -48:  LetterDesignator = 'G'
    elif -48 > Lat and Lat >= -56:  LetterDesignator = 'F'
    elif -56 > Lat and Lat >= -64:  LetterDesignator = 'E'
    elif -64 > Lat and Lat >= -72:  LetterDesignator = 'D'
    elif -72 > Lat and Lat >= -80:  LetterDesignator = 'C'
    else: LetterDesignator = 'Z'

    return LetterDesignator


def utmZoneNumber(Lat, Long):

    zone_number = 0
    # Make sure the longitude is between -180 .. 179.9
    longtemp = (Long + 180) - ((Long + 180) / 360) * 360 - 180
    zone_number = ((longtemp + 180) / 6) + 1

    if 56 <= Lat < 64 and 3.0 <= longtemp < 12.0:
        zone_number = 32

    if 72.0 <= Lat < 84.0:
        if 0.0 <= longtemp < 9.0:
            zone_number = 31
        elif 9.0 <= longtemp < 21.0:
            zone_number = 33
        elif 21.0 <= longtemp < 33.0:
            zone_number = 35
        elif 33.0 <= longtemp < 42.0:
            zone_number = 37

    return zone_number


def LLtoUTM_Radians(Lat, Long):
    return LLtoUTM_Degrees(Lat * rad2deg, Long * rad2deg)


def LLtoUTM_Degrees(Lat, Long):
    utm = utmCoord_t
    a = equatorialRadius
    eccSquared = eccentricitySquared
    k0 = 0.9996
    LongOrigin = 0
    eccPrimedSquared = 0
    N, T, C, A, M = 0, 0, 0, 0, 0

    LongTemp = (Long + 180.0) - ((Long + 180.0) / 360.0) * 360.0 - 180.0

    LatRad = Lat * deg2rad
    LongRad = LongTemp * deg2rad
    LongOriginRad = 0.0
    ZoneNumber = utmZoneNumber(Lat, Long)

    LongOrigin = (ZoneNumber - 1.0) * 6.0 - 180.0 + 3.0
    LongOriginRad = LongOrigin * deg2rad

    utm.UTMZoneNumber = ZoneNumber
    utm.UTMZoneLatDes = UTMLetterDesignator(Lat)

    eccPrimeSquared = eccSquared / (1.0 - eccSquared)

    N = a / math.sqrt(1.0 - eccSquared * math.sin(LatRad) * math.sin(LatRad))
    T = math.tan(LatRad) * math.tan(LatRad)
    C = eccPrimedSquared * math.cos(LatRad) * math.cos(LatRad)
    A = math.cos(LatRad) * (LongRad - LongOriginRad)

    M = a * ((1.0 - eccSquared / 4.0 - 3.0 * eccSquared * eccSquared / 64.0 - 5.0 * eccSquared * eccSquared * eccSquared / 256.0)
             * LatRad - (3.0 * eccSquared / 8.0 + 3.0 * eccSquared * eccSquared / 32.0 + 45.0 * eccSquared * eccSquared * eccSquared / 1024.0)
             * math.sin(2.0 * LatRad) + (15.0 * eccSquared * eccSquared / 256.0 + 45.0 * eccSquared * eccSquared * eccSquared / 1024.0)
             * math.sin(4.0 * LatRad) - (35.0 * eccSquared * eccSquared * eccSquared / 3072.0) * math.sin(6.0 * LatRad))

    utm.UTMEasting = (k0 * N * (A + (1.0 - T + C) * A * A * A / 6.0 +
                                (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * eccPrimeSquared)
                                * A * A * A * A * A / 120.0) + 500000.0)

    utm.UTMNorthing = (k0 * (M + N * math.tan(LatRad) * (A * A / 2.0 +
                                                         (5.0 - T + 9.0 * C + 4.0 * C * C) * A * A * A * A / 24.0 +
                                                         (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * eccPrimeSquared)
                                                         * A * A * A * A * A * A / 720.0)))
    if Lat < 0:
        utm.UTMNorthing += 10000000.0

    return utm


