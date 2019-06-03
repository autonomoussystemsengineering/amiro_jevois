#ifndef AMIRO_ARUCO_GLOBAL_HPP
#define AMIRO_ARUCO_GLOBAL_HPP

//Statemaschine
enum plannerStates{
    IDLE,
    SEARCH,
    START_SEARCH_TURN,
    SEARCH_TURN,
    MIDLLE_TURN,
    DRIVE_TO_TARGET,
    TARGET,
    STOP
    };

//Define Marker
typedef struct {
    int id;
    double x;
    double y;
    double z;
    double markerlen;
} Marker_t;


const char delimiter[] = " ";

#endif