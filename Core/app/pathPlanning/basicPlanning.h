/*
 * basicPlanning.h
 *
 *  Created on: Dec 11, 2021
 *      Author: onias
 */

#ifndef APP_CONTROL_PATHPLANNING_BASICPLANNING_H_
#define APP_CONTROL_PATHPLANNING_BASICPLANNING_H_

#include "main.h"
#include "../robot.h"
#include "../control/control.h"
#include "../mapping/mapping.h"
#include "../mapping/floorDetection.h"
#include "../mapping/wallDetection.h"

typedef enum{
	PATH_NO_OBSTRUCTION,
	PATH_OBSTACLE_GENERIC,
	PATH_OBSTACLE_FRONT,
	PATH_OBSTACLE_RIGHT,
	PATH_OBSTACLE_LEFT,
	PATH_OBSTACLE_EDGE_RIGHT,
	PATH_OBSTACLE_EDGE_LEFT,
	PATH_OBSTACLE_EDGE_BACK
} pathReturn_e;

extern map_e map[][MAP_LIMIT_Y];
extern displacement_t odometry[];

void dumbPathPlanning(robot_t * const robot);

pathReturn_e moveForward(robot_t * const robot, float speedForward);

pathReturn_e moveBackward(robot_t * const robot, float speedBackward, uint32_t timeMilliSec);

pathReturn_e moveRotateCCW(robot_t * const robot, float speedCCW, uint32_t timeMilliSec);

pathReturn_e moveRotateCW(robot_t * const robot, float speedCW);


#endif /* APP_CONTROL_PATHPLANNING_BASICPLANNING_H_ */
