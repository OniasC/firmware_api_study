/*
 * kalman_robot.c
 *
 *  Created on: 29 Mar 2022
 *      Author: onias
 */
// create the filter structure
#define KALMAN_NAME robot
#define KALMAN_NUM_STATES 4
#define KALMAN_NUM_INPUTS 2
#include "../app/thirdparties/kalman/kalman_factory_filter.h"

// create the 1st measurement structure
#define KALMAN_MEASUREMENT_NAME encoder
#define KALMAN_NUM_MEASUREMENTS 2
#include "../app/thirdparties/kalman/kalman_factory_measurement.h"

// create the 2nd measurement structure
#define KALMAN_MEASUREMENT_NAME imu
#define KALMAN_NUM_MEASUREMENTS 2
#include "../app/thirdparties/kalman/kalman_factory_measurement.h"

// clean up
#include "../app/thirdparties/kalman/kalman_factory_cleanup.h"

#define matrix_set matrix_set_ONIAS

#include "string.h"
/*!
* \brief Initializes the gravity Kalman filter
*/
void control_kalman_init(const kalman_t *kf, kalman_measurement_t * kfmEncoder, kalman_measurement_t *kfmIMU)
{
    /************************************************************************/
    /* initialize the filter structures                                     */
    /************************************************************************/
	kalman_t *kf_temp = kalman_filter_robot_init();
	kalman_measurement_t *kfmEncoder_temp = kalman_filter_robot_measurement_encoder_init();
	kalman_measurement_t *kfmIMU_temp = kalman_filter_robot_measurement_imu_init();

    /************************************************************************/
    /* set initial state                                                    */
    /************************************************************************/
    matrix_t *x = kalman_get_state_vector(kf_temp);
    x->data[0] = 0; // v_i
    x->data[1] = 0; // w_i
    x->data[2] = 0; // a_i
    x->data[3] = 0; // alfa_i

    /************************************************************************/
    /* set state transition                                                 */
    /************************************************************************/
    matrix_t *A = kalman_get_state_transition(kf_temp);

    // set time constant
    const matrix_data_t T = 0.001;

    // transition of _ to _
    matrix_set(A, 0, 0, 1);   // 1
    matrix_set(A, 0, 1, 0);   // 0
    matrix_set(A, 0, 2, (matrix_data_t)T); // T
    matrix_set(A, 0, 3, 0); // 0

    // transition of _ to _
    matrix_set(A, 1, 0, 0);   // 0
    matrix_set(A, 1, 1, 1);   // 1
    matrix_set(A, 1, 2, 0);   // 0
    matrix_set(A, 1, 3, (matrix_data_t)T);   // T

    // transition of _ to _
    matrix_set(A, 2, 0, 0);   // 0
    matrix_set(A, 2, 1, 0);   // 0
    matrix_set(A, 2, 2, 1);   // 1
    matrix_set(A, 2, 3, 0);   // 0

    // transition of _ to _
    matrix_set(A, 3, 0, 0);   // 0
    matrix_set(A, 3, 1, 0);   // 0
    matrix_set(A, 3, 2, 0);   // 0
    matrix_set(A, 3, 3, 1);   // 1

    matrix_t *B = kalman_get_input_transition(kf_temp);
    // transition of _ to _
	matrix_set(B, 0, 0, 1);   // 0
	matrix_set(B, 0, 1, 0);   // 0

	// transition of _ to _
	matrix_set(B, 1, 0, 0);   // 0
	matrix_set(B, 1, 1, 1);   // 0

	// transition of _ to _
	matrix_set(B, 2, 0, 0);   // 0
	matrix_set(B, 2, 1, 0);   // 0

	// transition of _ to _
	matrix_set(B, 3, 0, 0);   // 0
	matrix_set(B, 3, 1, 0);   // 0

	//TODO: finish populating the matrixes!
    /************************************************************************/
    /* set covariance                                                       */
    /************************************************************************/
    matrix_t *P = kalman_get_system_covariance(kf_temp);

    matrix_set_symmetric(P, 0, 0, (matrix_data_t)0.1);   // var(s)
    matrix_set_symmetric(P, 0, 1, 0);   // cov(s,v)
    matrix_set_symmetric(P, 0, 2, 0);   // cov(s,g)

    matrix_set_symmetric(P, 1, 1, 1);   // var(v)
    matrix_set_symmetric(P, 1, 2, 0);   // cov(v,g)

    matrix_set_symmetric(P, 2, 2, 1);   // var(g)

    /************************************************************************/
    /* set measurement transformation                                       */
    /************************************************************************/
    matrix_t *H_encoder = kalman_get_measurement_transformation(kfmEncoder_temp);

    matrix_set(H_encoder, 0, 0, 1);     // z = 1*s
    matrix_set(H_encoder, 0, 1, 0);     //   + 0*v
    matrix_set(H_encoder, 0, 2, 0);     //   + 0*g

    /************************************************************************/
    /* set process noise                                                    */
    /************************************************************************/
    matrix_t *R_encoder = kalman_get_process_noise(kfmEncoder_temp);

    matrix_set(R_encoder, 0, 0, (matrix_data_t)0.5);     // var(s)

    /************************************************************************/
    /* set measurement transformation                                       */
    /************************************************************************/
    matrix_t *H_imu = kalman_get_measurement_transformation(kfmIMU_temp);

    matrix_set(H_imu, 0, 0, 1);     // z = 1*s
    matrix_set(H_imu, 0, 1, 0);     //   + 0*v
    matrix_set(H_imu, 0, 2, 0);     //   + 0*g

    /************************************************************************/
    /* set process noise                                                    */
    /************************************************************************/
    matrix_t *R_imu = kalman_get_process_noise(kfmIMU_temp);

    matrix_set(R_imu, 0, 0, (matrix_data_t)0.5);     // var(s)

    kf = kf_temp;
    memcpy(kfmEncoder, kfmEncoder_temp, sizeof(&kfmEncoder));
    memcpy(kfmIMU, kfmIMU_temp, sizeof(&kfmIMU));

}
