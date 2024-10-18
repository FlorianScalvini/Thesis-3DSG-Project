//
// Created by Florian on 14/09/22.
//

#include "geographic_coordinate.h"
#include <math.h>

#define PI   3.14159265358979323846264338327950288
#define degToRad(angleInDegrees) ((angleInDegrees) * PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / PI)
#define radiusEarth  6371000.0


GeographicCoordinate::GeographicCoordinate(double longitude, double latitude) {
    this->longitude = longitude;
    this->latitude = latitude;
}

/*
 * Return the distance between two geographic coordinate in meter
 */
double GeographicCoordinate::toDistance(GeographicCoordinate pointA, GeographicCoordinate pointB) {
    double dLon = degToRad(pointA.longitude - pointB.longitude);
    double phiA = degToRad(pointA.latitude);
    double phiB = degToRad(pointB.latitude);
    double dst = radiusEarth * acos(sin(phiA) * sin(phiB) + cos(phiA) * cos(phiB) * cos(dLon));
    return dst;
}


/*
 * Return the direction from pointA to pointB in degree
 */
double GeographicCoordinate::toBearing(GeographicCoordinate pointA, GeographicCoordinate pointB) {
    double dLon = degToRad(pointB.longitude - pointA.longitude);
    double radA = degToRad(pointA.latitude);
    double radB = degToRad(pointB.latitude);
    double X = cos(radB) * sin(dLon);
    double Y = cos(radA) * sin(radB) - sin(radA) * cos(radB) * cos(dLon);
    double bearing = atan2(X, Y);
    return radToDeg(bearing);
}

GeographicCoordinate GeographicCoordinate::toEastPosition(GeographicCoordinate point, float distance) {
    float  new_longitude = point.longitude + (distance / radiusEarth) * (180 / PI) / cos(point.latitude * PI / 180);
    return {new_longitude, point.latitude};
}

GeographicCoordinate GeographicCoordinate::toNorthPosition(GeographicCoordinate point, float distance) {
    float new_latitude = point.latitude + (distance / radiusEarth) * (180 / PI);
    return {point.longitude, new_latitude};
}