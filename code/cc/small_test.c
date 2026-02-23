#include <stdio.h>
#include <stdlib.h>
#include <math.h>

double great_circle(double lng1, double lat1, double lng2, double lat2) {
    const double R = 6371.0;           // Earth's mean radius in km
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

double fancy_distance(double lng1, double lat1, double lng2, double lat2) {
    int DistanceToTravel = 1000;    // miles
    int Skip = 1;                   // starting skip value
    
    for (int i = Skip; i < DistanceToTravel; i++) {
        // Increase the skip amount by 50 each iteration
        Skip += 50;
        great_circle(lng1, lat1, lng2, lat2);
    }
    printf("Done: %d\n", Skip);
    return 0;
}

int main(void) {
    fancy_distance(0, 0,1,1);
    return 0;
}

