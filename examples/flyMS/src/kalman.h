#ifndef KALMAN_H
#define KALMAN_H

#pragma once
#include "flyMS.h"

void update_rot_matrix(matrix_t* Rot_matrix);
void* kalman_filter(void *ptr);
vector_t* get_lat_state();
vector_t* get_lon_state();


#endif