/*
 * mapping.c
 *
 *  Created on: Dec 11, 2021
 *      Author: onias
 */

#include "mapping.h"

map_e map[MAP_LIMIT_X][MAP_LIMIT_Y];
displacement_t odometry[MAP_ODOMETRY_TAIL];

void mapping_updateMap(robot_t * const robot, map_e map[][MAP_LIMIT_Y], displacement_t odometry[])
{
	map[0][0] = MAP_CURRENT;
}

void mapping_insertObstacleMap(robot_t * const robot, map_e map[][MAP_LIMIT_Y], displacement_t odometry[]);

void mapping_insertEdgeMap(robot_t * const robot, map_e map[][MAP_LIMIT_Y], displacement_t odometry[]);
