import math
import numpy
import TCS_util

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
    Formulae: https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ENU_to_ECEF
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
    return M.dot(V)
