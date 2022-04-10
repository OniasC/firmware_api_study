/*
 * basicPlanning.c
 *
 *  Created on: Dec 11, 2021
 *      Author: onias
 */

#include "basicPlanning.h"

void dumbPathPlanning(robot_t * const robot)
{
	pathReturn_e path 	  = PATH_NO_OBSTRUCTION;
	bumpSensor_e obstacle = 0U;
	edgeSensor_e edge     = 0U;
	uint8_t 	 cont 	  = 1;

	while(cont)
	{
		obstacle = detectObstacle(robot->bumpSensor, ROBOT_NUM_BUMP);
		edge = detectEdge(robot->infraRed, ROBOT_NUM_IR);
		if ((obstacle != NO_BUMP) || (edge != NO_EDGE))
		{
			path = PATH_OBSTACLE_GENERIC;
		}
		switch(path)
		{
		case PATH_NO_OBSTRUCTION:
			path = moveForward(robot, 0.5);
			//cont = 1;
			break;
		case PATH_OBSTACLE_GENERIC:
		case PATH_OBSTACLE_FRONT:
		case PATH_OBSTACLE_RIGHT:
		case PATH_OBSTACLE_LEFT:
		case PATH_OBSTACLE_EDGE_RIGHT:
		case PATH_OBSTACLE_EDGE_LEFT:
		case PATH_OBSTACLE_EDGE_BACK:
			path = moveBackward(robot, 0.5, 1000);
			path = moveRotateCCW(robot, 0.5, 1000);
			break;
		}
	}
}

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

pathReturn_e moveBackward(robot_t * const robot, float speedBackward, uint32_t timeMilliSec)
{
	float desiredRobotSpeed[] = {0.0, -speedBackward};
	bumpSensor_e obstacle = 0U;
	edgeSensor_e edge = 0U;
	uint32_t timeInit = HAL_GetTick();

	while((obstacle == 0) && (edge == 0) && (HAL_GetTick() - timeInit < timeMilliSec))
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

pathReturn_e moveRotateCCW(robot_t * const robot, float speedCCW, uint32_t timeMilliSec)
{
	float desiredRobotSpeed[] = {speedCCW, 0.0};
	bumpSensor_e obstacle = 0U;
	edgeSensor_e edge = 0U;
	uint32_t timeInit = HAL_GetTick();

	while((obstacle == 0) && (edge == 0) && (HAL_GetTick() - timeInit < timeMilliSec))
	{
		robot_controlRobotSpeed(robot, desiredRobotSpeed);
		mapping_updateMap(robot, map, odometry);
		obstacle = detectObstacle(robot->bumpSensor, ROBOT_NUM_BUMP);
		edge = detectEdge(robot->infraRed, ROBOT_NUM_IR);
		/*check here if there is an update to bump sensors and edge sensor*/
		if(obstacle != NO_BUMP)
		{
			desiredRobotSpeed[0] = 0.0;
			robot_controlRobotSpeed(robot, desiredRobotSpeed);
			//mapping_addMapFeature(robot, map, obstacle);
			return PATH_OBSTACLE_GENERIC;
		}
		if(edge != NO_EDGE)
		{
			desiredRobotSpeed[0] = 0.0;
			robot_controlRobotSpeed(robot, desiredRobotSpeed);
			//mapping_addMapFeature(robot, map, edge);
			return PATH_OBSTACLE_GENERIC;
		}
	}
	return PATH_NO_OBSTRUCTION;
}

pathReturn_e moveRotateCW(robot_t * const robot, float speedCW)
{
	return PATH_NO_OBSTRUCTION;
}
