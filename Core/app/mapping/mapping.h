/*
 * mapping.h
 *
 *  Created on: Dec 9, 2021
 *      Author: onias
 */

#ifndef APP_MAPPING_MAPPING_H_
#define APP_MAPPING_MAPPING_H_

#include "main.h"
#include "../robot.h"

#define MAP_LIMIT_X 100
#define MAP_LIMIT_Y 100
#define MAP_ODOMETRY_TAIL 25

typedef struct {
	float x;
	float y;
	uint32_t timeStamp;
	uint32_t index;
} displacement_t;

typedef enum {
	MAP_NOT_EXPLORED,
	MAP_EXPLORED,
	MAP_OBSTACLE,
	MAP_EDGE,
	MAP_CURRENT,
} map_e;

extern map_e map[][MAP_LIMIT_Y];
extern displacement_t odometry[];

void mapping_updateMap(robot_t * const robot, map_e map[][MAP_LIMIT_Y], displacement_t odometry[]);

void mapping_insertObstacleMap(robot_t * const robot, map_e map[][MAP_LIMIT_Y], displacement_t odometry[]);

void mapping_insertEdgeMap(robot_t * const robot, map_e map[][MAP_LIMIT_Y], displacement_t odometry[]);


#endif /* APP_MAPPING_MAPPING_H_ */
