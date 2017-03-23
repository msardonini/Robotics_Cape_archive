#ifndef FILTER_H
#define FILTER_H

#define MAX_ARRAY_SIZE 32

typedef struct discrete_filter{
	int order;
	float dt;
	// input scaling factor usually =1, useful for fast controller tuning
	float prescaler; 
	float numerator[MAX_ARRAY_SIZE];	// points to array of numerator constants
	float denominator[MAX_ARRAY_SIZE];	// points to array 
	float inputs[MAX_ARRAY_SIZE];
	float outputs[MAX_ARRAY_SIZE];
	float last_input;
	float current_output;
}discrete_filter;


/* 
--- March Filter ---
march the filter forward in time one step with new input data
returns new output which could also be accessed with filter.current_output
*/
float marchFilter(discrete_filter* filter, float new_input);


/* 
--- Saturate Filter ---
limit the output of filter to be between min&max
returns 1 if saturation was hit 
returns 0 if output was within bounds
*/
float saturateFilter(float filter, float min, float max);


/*
--- Zero Filter ---
reset all input and output history to 0
*/
int zeroFilter(discrete_filter* filter);

/*
--- PreFill Filter ---
fill the past inputs with the curent input
use before marchFilter when starting to avoid ugly step input
*/
int preFillFilter(discrete_filter* filter, float input);


/*
--- Generate Filter ---
Dynamically allocate memory for a filter of specified order
and set transfer function constants.

Note: A normalized transfer function should have a leading 1 
in the denominator but can be !=1 in this library
*/
discrete_filter generateFilter(int order, float dt,float numerator[],float denominator[]);

// discrete-time implementation of a parallel PID controller with derivative filter
// similar to Matlab pid command
//
// N is the pole location for derivative filter. Must be greater than 2*DT
// smaller N gives faster filter decay
discrete_filter generatePID(float kp, float ki, float kd, float Tf, float dt);

// print order, numerator, and denominator constants
int printFilterDetails(discrete_filter* filter);




#endif