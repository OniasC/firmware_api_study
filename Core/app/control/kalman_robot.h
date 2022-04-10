/*
 * kalman_robot.h
 *
 *  Created on: 29 Mar 2022
 *      Author: onias
 */

#ifndef APP_CONTROL_KALMAN_ROBOT_H_
#define APP_CONTROL_KALMAN_ROBOT_H_

#include "../app/thirdparties/kalman/kalman.h"

void control_kalman_init(const kalman_t *kf, kalman_measurement_t *kfmEncoder, kalman_measurement_t *kfmIMU);


#endif /* APP_CONTROL_KALMAN_ROBOT_H_ */
