#include <stdio.h>
#include <stdlib.h>
#include <math.h>

const double m=1.;
const double km=1000.*m;
const double miles=1609.34*m;

double great_circle(double lng1, double lat1, double lng2, double lat2) {
    const double R = 6371.*km;           // Earth's mean radius in km
    const double TO_RAD = M_PI / 180.0; // degrees to radians conversion

    // Convert latitudes and longitudes to radians
    double φ1 = lat1 * TO_RAD;
    double φ2 = lat2 * TO_RAD;
    double Δφ = (lat2 - lat1) * TO_RAD;
    double Δλ = (lng2 - lng1) * TO_RAD;

    // Haversine formula
    double a = sin(Δφ / 2.0) * sin(Δφ / 2.0) +
               cos(φ1) * cos(φ2) *
               sin(Δλ / 2.0) * sin(Δλ / 2.0);

    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    return R * c;
}

#include <math.h>

double euclidean_polar_distance(double lat1, double lon1,
                                double lat2, double lon2) {
    const double MERIDIAN_LENGTH = 40000.*km;

    double colat1_deg = 90.0 - fabs(lat1);
    double colat2_deg = 90.0 - fabs(lat2);

    double r1 = MERIDIAN_LENGTH * (colat1_deg / 180.0);
    double r2 = MERIDIAN_LENGTH * (colat2_deg / 180.0);

    double theta1 = lon1 * M_PI / 180.0;
    double theta2 = lon2 * M_PI / 180.0;

    double x1 = r1 * cos(theta1);
    double y1 = r1 * sin(theta1);
    double x2 = r2 * cos(theta2);
    double y2 = r2 * sin(theta2);

    // Euclidean distance
    double dx = x2 - x1;
    double dy = y2 - y1;

    return sqrt(dx * dx + dy * dy);
}

double fancy_distance(double lng1, double lat1, double lng2, double lat2) {
    int DistanceToTravel = 1000;    // miles
    int Skip = 1;                   // starting skip value
    
    for (int i = Skip; i < DistanceToTravel; i++) {
        // Increase the skip amount by 50 each iteration
        Skip += 50;
        euclidean_polar_distance(lng1, lat1, lng2, lat2);
    }
    printf("Done: %d\n", Skip);
    return 0;
}

int main(void) {
    fancy_distance(0, 0,1,1);
    return 0;
}

