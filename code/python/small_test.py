#!/usr/bin/env python3

def pos2dist(long, lat):
    return 0

def whatever(long, lat):
    DistanceToTravel = 1000
    Skip = 1

    for i in range(Skip,DistanceToTravel):
        Skip = Skip + 50
        pos2dist(long, lat)

    print (f"Done: {Skip}")


whatever(0,0)