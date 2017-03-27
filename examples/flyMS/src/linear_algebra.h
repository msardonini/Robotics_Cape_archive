/*******************************************************************************
* linear_algebra.h
*
* A couple of functions that either weren't included in the original or needed adjustments
*******************************************************************************/
#pragma once
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h> // for memset
#include <unistd.h>
#include <robotics_cape.h>
	
#ifndef LINEAR_ALGEBGRAMS_H
#define LINEAR_ALGEBGRAMS_H
	
// Basic Matrix creation, modification, and access
int add_vectors(vector_t A, vector_t B, vector_t *out);
void vector_times_scalar2(vector_t* v, float s, vector_t* o);
int copy_matrix(matrix_t A, matrix_t *out);

#endif

