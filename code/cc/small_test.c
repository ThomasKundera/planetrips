#include <stdio.h>
#include <stdlib.h>

double pos2dist(double longitude, double latitude) {
    return 0;
}

double whatever(double lng, double lat) {
    int DistanceToTravel = 1000;    // miles
    int Skip = 1;                   // starting skip value
    
    for (int i = Skip; i < DistanceToTravel; i++) {
        // Increase the skip amount by 50 each iteration
        Skip += 50;
        pos2dist(lng, lat);
    }
    printf("Done: %d\n", Skip);
    return 0;
}

int main(void) {
    whatever(0, 0);
    return 0;
}