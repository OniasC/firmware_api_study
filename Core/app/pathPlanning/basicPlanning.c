/*
 * basicPlanning.c
 *
 *  Created on: Dec 11, 2021
 *      Author: onias
 */

#include "basicPlanning.h"

pathReturn_e moveForward(robot_t * const robot, float speedForward)
{
	float desiredRobotSpeed[] = {0.0, speedForward};
	bumpSensor_e obstacle = 0U;
	edgeSensor_e edge = 0U;

	while((obstacle == 0) && (edge == 0))
	{

		robot_controlRobotSpeed(robot, desiredRobotSpeed);
		mapping_updateMap(robot, map, odometry);
		obstacle = detectObstacle(robot->bumpSensor, ROBOT_NUM_BUMP);
		edge = detectEdge(robot->infraRed, ROBOT_NUM_IR);
		/*check here if there is an update to bump sensors and edge sensor*/
		if(obstacle != NO_BUMP)
		{
			desiredRobotSpeed[1] = 0.0;
			robot_controlRobotSpeed(robot, desiredRobotSpeed);
			//mapping_addMapFeature(robot, map, obstacle);
			return PATH_OBSTACLE_GENERIC;
		}
		if(edge != NO_EDGE)
		{
			desiredRobotSpeed[1] = 0.0;
			robot_controlRobotSpeed(robot, desiredRobotSpeed);
			//mapping_addMapFeature(robot, map, edge);
			return PATH_OBSTACLE_GENERIC;
		}
	}

	return PATH_NO_OBSTRUCTION;
}

pathReturn_e moveBackward(robot_t * const robot, float speedBackward)
{
	float desiredRobotSpeed[] = {0.0, 0.0};

	/*if (detectEdge())
	{
		return detectEdge();
	}
	if (detectObstacle())
	{
		return detectObstacle();
	}*/

	desiredRobotSpeed[0] = 0.0;
	desiredRobotSpeed[1] = 0.5;
	robot_controlRobotSpeed(robot, desiredRobotSpeed);
	return PATH_NO_OBSTRUCTION;
}

pathReturn_e moveRotateCCW(robot_t * const robot, float speedCCW)
{
	return PATH_NO_OBSTRUCTION;
}

pathReturn_e moveRotateCW(robot_t * const robot, float speedCW)
{
	return PATH_NO_OBSTRUCTION;
}
