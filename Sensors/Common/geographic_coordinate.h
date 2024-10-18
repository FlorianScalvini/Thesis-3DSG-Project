//
// Created by Florian on 14/09/22.
//

#ifndef OUTDOORNAV_GEOGRAPHIC_COORDINATE_H
#define OUTDOORNAV_GEOGRAPHIC_COORDINATE_H


class GeographicCoordinate {
public:
    GeographicCoordinate()= default;
    GeographicCoordinate(double longitude, double latitude);
    bool inBound();
    static double toDistance(GeographicCoordinate pointA, GeographicCoordinate pointB);
    static double toBearing(GeographicCoordinate pointA, GeographicCoordinate pointB);
    static GeographicCoordinate toNorthPosition(GeographicCoordinate point, float distance);
    static GeographicCoordinate toEastPosition(GeographicCoordinate point, float distance);
    double longitude = 0;
    double latitude = 0;
};


#endif //OUTDOORNAV_GEOGRAPHIC_COORDINATE_H
