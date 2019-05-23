#ifndef ECSAM_DEFINITIONS_H
#define ECSAM_DEFINITIONS_H

#define ECSAM_TOPIC_COMMAND "ecsam/command"
#define ECSAM_TOPIC_STATUS "ecsam/status"

#define ECSAM_COMMAND_PICK "pick"
#define ECSAM_COMMAND_PLACE "place"

#define ECSAM_INFO_PICKING_SUCCESSFUL "picking successful"
#define ECSAM_INFO_PLACING_SUCCESSFUL "placing successful"

#define ECSAM_ERROR_GAZE_NOT_CORRELATED "gaze not correlated"
#define ECSAM_ERROR_NO_GRASP_DETECTED "no grasp detected"
#define ECSAM_ERROR_PICKING_FAILED "picking failed"
#define ECSAM_ERROR_PLACING_FAILED "placing failed"

#define PICK_CORRELATION_LINE_POINT_DISTANCE_THRESHOLD  0.05
#define PLACE_CORRELATION_LINE_POINT_DISTANCE_THRESHOLD 0.005


//#define EXECUTION_TIME_FILE_PATH "/home/jesper/ecsam_execution_time.txt"
//#define GAZE_CORRELATION_TABLE_FILE_PATH "/home/jesper/gaze_correlation.txt"
//#define GAZE_CORRELATION_OBJECT
#include <fstream>

#endif