#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

const double m=1.;
const double km=1000.*m;
const double mile=1609.34*m;

const double TO_RAD = M_PI / 180.0;
const double R = 6371.*km;
const double MERIDIAN_LENGTH = 40000.*km;


// Structure for latitude/longitude pairs
typedef struct {
    const char* name;
    double latitude;
    double longitude;
} Location;


static const Location locations[] = {
    // Northern Hemisphere
    {"New York",          40.7128,   -74.0060},
    {"London",            51.5074,    -0.1278},
    {"Tokyo",             35.6762,   139.6503},
    {"Paris",             48.8566,     2.3522},
    {"Moscow",            55.7558,    37.6173},
    {"Cairo",             30.0444,    31.2357},
    {"Los Angeles",       34.0522,  -118.2437},
    {"Beijing",           39.9042,   116.4074},

    // Southern Hemisphere
    {"Sydney",           -33.8688,   151.2093},
    {"Melbourne",        -37.8136,   144.9631},
    {"Cape Town",        -33.9249,    18.4241},
    {"Buenos Aires",     -34.6037,   -58.3816},
    {"Rio de Janeiro",   -22.9068,   -43.1729},
    {"Auckland",         -36.8485,   174.7633},
    {"Santiago",         -33.4489,   -70.6693},
    {"Perth",            -31.9505,   115.8605},

    // Near equator
    {"Singapore",          1.3521,   103.8198},
    {"Nairobi",           -1.2921,    36.8219},
    {"Quito",             -0.1807,   -78.4678}
};


double great_circle(double lng1, double lat1, double lng2, double lat2) {
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

double euclidean_polar_distance(double lat1, double lon1,
                                double lat2, double lon2) {
    double colat1_deg = 90.0 - fabs(lat1);
    double colat2_deg = 90.0 - fabs(lat2);

    double r1 = MERIDIAN_LENGTH * (colat1_deg / 180.0);
    double r2 = MERIDIAN_LENGTH * (colat2_deg / 180.0);

    double theta1 = lon1 * TO_RAD;
    double theta2 = lon2 * TO_RAD;

    double x1 = r1 * cos(theta1);
    double y1 = r1 * sin(theta1);
    double x2 = r2 * cos(theta2);
    double y2 = r2 * sin(theta2);

    // Euclidean distance
    double dx = x2 - x1;
    double dy = y2 - y1;

    return sqrt(dx * dx + dy * dy);
}

bool polar_euclidean_interpolate(
    double latA, double lonA,
    double latB, double lonB,
    double step,
    double* latC_out,
    double* lonC_out)
{
    const double R_MAX = MERIDIAN_LENGTH / 2.0;

    // Co-latitudes (angular distance from north pole in degrees)
    double colatA_deg = 90.0 - fabs(latA);
    double colatB_deg = 90.0 - fabs(latB);

    // Radial distances from center
    double rA = R_MAX * (colatA_deg / 90.0);
    double rB = R_MAX * (colatB_deg / 90.0);

    double thetaA = lonA * M_PI / 180.0;
    double thetaB = lonB * M_PI / 180.0;

    // Cartesian coordinates
    double xA = rA * cos(thetaA);
    double yA = rA * sin(thetaA);
    double xB = rB * cos(thetaB);
    double yB = rB * sin(thetaB);

    // Vector from A to B
    double dx = xB - xA;
    double dy = yB - yA;

    // Total Euclidean distance A→B
    double total_dist = sqrt(dx*dx + dy*dy);

    // Handle degenerate case: points are the same
    if (total_dist < 1 * m ) {
        *latC_out = latA;
        *lonC_out = lonA;
        return (step <= 1 * m);
    }

    // Fraction along the line segment
    double t;
    if (step <= 0.0) {
        t = 0.0;
    } else if (step >= total_dist) {
        t = 1.0;
    } else {
        t = step / total_dist;
    }

    // Interpolated position
    double xC = xA + t * dx;
    double yC = yA + t * dy;

    // Convert back to polar (r, theta)
    double rC = sqrt(xC*xC + yC*yC);
    double thetaC;

    if (rC < 1 * m) {
        // Very close to north pole → latitude = 90°, longitude arbitrary
        *latC_out = 90.0;
        *lonC_out = lonA;  // or 0.0 — doesn't matter
        return true;
    }

    thetaC = atan2(yC, xC);
    double lonC = thetaC * 180.0 / M_PI;

    // Normalize longitude to [-180, +180]
    if (lonC > 180.0)  lonC -= 360.0;
    if (lonC < -180.0) lonC += 360.0;

    // Convert radius back to co-latitude → latitude
    double colatC_deg = (rC / R_MAX) * 90.0;
    double latC = 90.0 - colatC_deg;

    *latC_out = latC;
    *lonC_out = lonC;

    return true;
}


double fancy_distance(double lng1, double lat1, double lng2, double lat2) {
    double sum_dist = 0;
    double step=50*mile;

    while (1) {
        double latC, lonC;
        if (!polar_euclidean_interpolate(lat1, lng1, lat2, lng2, step, &latC, &lonC)) {
            break;
        }
        sum_dist += step;
        lat1 = latC;
        lng1 = lonC;
    }
    
    return sum_dist;
}



int main(void) {
    // Melbourne
    char city1[256];
    strcpy(city1, locations[8].name);
    double lat1 = locations[8].latitude;
    double lng1 = locations[8].longitude;
    // Sydney
    char city2[256];
    strcpy(city2, locations[9].name);
    double lat2 = locations[9].latitude;
    double lng2 = locations[9].longitude;

    double dist = euclidean_polar_distance(lat1, lng1, lat2, lng2);
    printf("Euclidean distance between %s and %s: %f km\n", city1, city2, dist / km);

    dist = great_circle(lng1, lat1, lng2, lat2);
    printf("Great circle distance between %s and %s: %f km\n", city1, city2, dist / km);

    dist = fancy_distance(lng1, lat1, lng2, lat2);
    printf("Fancy distance between %s and %s: %f km\n", city1, city2, dist / km);
}

