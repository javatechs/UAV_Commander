import math
import numpy
import TCS_util

DBL_EPSILON = 2.2204460492503131e-16
CONSTANTS_RADIUS_OF_EARTH = 6371000

def globalToECEF(latitude, longitude, altitude):
    """constants description WGS84:
    http://www.arsitech.com/mapping/geodetic_datum/

    Formulae: https://en.wikipedia.org/wiki/Geographic_coordinate_conversion
    """
    phi = math.radians(latitude)
    lam = math.radians(longitude)
    h = altitude
    e = 0.08181919092890624
    a = 6378137.0
    N = a/math.sqrt(1-math.pow(e,2)*math.pow(math.sin(phi),2))
    K = (N + h)*math.cos(phi)
    X = K*math.cos(lam)
    Y = K*math.sin(lam)
    Z = (N*(1-e*e)+h)*math.sin(phi)
    return TCS_util.vector3(X,Y,Z)

def ECEFtoENU(ECEFp,ECEFo,latitude,longitude):
    """converts from ECEF coordinate to local coordinate
    Formulae: https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ECEF_to_ENU
    """
    phi = numpy.radians(latitude)
    lam = numpy.radians(longitude)
    r1 = (-math.sin(lam), math.cos(lam), 0.0)
    r2 = (-math.sin(phi) * math.cos(lam), -math.sin(phi)*math.sin(lam), math.cos(phi))
    r3 = (math.cos(phi)*math.cos(lam), math.cos(phi)*math.sin(lam), math.sin(phi))
    M = numpy.matrix((r1,r2,r3))
    X = ECEFp.x - ECEFo.x
    Y = ECEFp.y - ECEFo.y
    Z = ECEFp.z - ECEFo.z
    V = numpy.array([X,Y,Z]).transpose()
    R = M.dot(V)
    return TCS_util.vector3(R[0,0],R[0,1],R[0,2])

def ECEFtoNED(ECEFp,ECEFo,latitude,longitude):
    """converts from ECEF coordinate to local coordinate
    """
    phi = numpy.radians(latitude)
    lam = numpy.radians(longitude)
    r1 = (-math.sin(phi)*math.cos(lam), -math.sin(phi)*math.sin(lam), math.cos(phi))
    r2 = (-math.sin(lam), math.cos(lam), 0.0)
    r3 = (-math.cos(phi)*math.cos(lam), -math.cos(phi)*math.sin(lam), -math.sin(phi))
    M = numpy.matrix((r1,r2,r3))
    X = ECEFp.x - ECEFo.x
    Y = ECEFp.y - ECEFo.y
    Z = ECEFp.z - ECEFo.z
    V = [X,Y,Z]
    R = M.dot(V)
    return TCS_util.vector3(R[0,0],R[0,1],R[0,2])

def globalToNED_PX4(GEOp, GEOo):
    lat_rad = numpy.radians(GEOp.x)
    lon_rad = numpy.radians(GEOp.y)
    sin_lat = numpy.sin(lat_rad)
    cos_lat = numpy.cos(lat_rad)
    lat_rado = numpy.radians(GEOo.x)
    lon_rado = numpy.radians(GEOo.y)
    sin_lato = numpy.sin(lat_rado)
    cos_lato = numpy.cos(lat_rado)
    cos_d_lon = numpy.cos(lon_rad - lon_rado)
    arg = sin_lato*sin_lat + cos_lato * cos_lat * cos_d_lon
    if arg > 1.0:
        arg = 1.0
    elif arg < -1.0:
        arg = -1.0

    c = numpy.arccos(arg)
    global DBL_EPSILON
    global CONSTANTS_RADIUS_OF_EARTH
    if(numpy.fabs(c) < DBL_EPSILON):
        k = 1.0
    else:
        k = c/numpy.sin(c)
    x = k * (cos_lato*sin_lat - sin_lato * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH
    y = k * cos_lat * numpy.sin(lon_rad - lon_rado) * CONSTANTS_RADIUS_OF_EARTH
    return TCS_util.vector3(x,y,GEOp.z) 

