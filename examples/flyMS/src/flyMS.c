	// flyMS.c Control Program to fly quadcopter
// By Michael Sardonini, with help from James Strawson



#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include <math.h>
#include <time.h>
#include "../include/filter.h"
#include "../include/flight_defs.h"
#include "../include/flyMS.h"
#include "../include/kalman.h"
#include "../include/gps.h"
#include "../include/logger.h"


#define DEBUG

#define LAT_ACCEL_BIAS -0.0177
#define LON_ACCEL_BIAS  0.0063
#define ALT_ACCEL_BIAS  0.0000
#define YAW_OFFSET	  	1.106

 

int flight_core(void * ptr);
//void* kalman_filter(void*ptr);
//int initialize_structs(accel_data_t *tmp, GPS_data_t *tmp2);


/************************************************************************
* 	Global Variables				
************************************************************************/
control_variables_t		control;			//Structure to contain all system states
setpoint_t 				setpoint; 			//Structure to store all Setpoints
GPS_data_t				GPS_data;			//Structure to store data from GPS
function_control_t 		function_control;	//Structure to store variables which control functions
filters_t				filters;			//Struct to contain all the filters
accel_data_t 			accel_data;			//A struct which is given to kalman.c
logger_t				logger;
tranform_matrix_t		transform;

imu_data_t				imu_data;			//Struct to relay all IMU info from driver to here
float 					accel_bias[3] = {LAT_ACCEL_BIAS, LON_ACCEL_BIAS, ALT_ACCEL_BIAS};
float 					yaw_offset[3] = {0, 0, YAW_OFFSET};



 
int flight_core(void * ptr){
	
	control_variables_t *STATE = (control_variables_t*)ptr;
	
	//printf("pointer value %f\n", STATE->pitch);
	
	//static vector_t *X_state_Lat1, *X_state_Lon1;
	
	//Keep an i for loops
	static uint8_t i=0;
	
	//Variables to Initialize things upon startup
	static uint8_t First_Iteration=1, First_Iteration_GPS=1;
	
	//static float initial_alt = 0;

	//Initialize some variables if it is the first iteration
	if(First_Iteration){
		//memset(&control,0,sizeof(control));
		//memset(&setpoint,0,sizeof(setpoint));
		//memset(&GPS_data,0,sizeof(GPS_data));
		////memset(&function_control,0,sizeof(function_control));
		//memset(&filters,0,sizeof(filters));
		//memset(&accel_data,0,sizeof(accel_data));
		//memset(&logger,0,sizeof(logger));
		//memset(&transform,0,sizeof(transform));
		i=0;
		clock_gettime(CLOCK_MONOTONIC, &function_control.start_time); //Set the reference time to the first iteration
		setpoint.Aux = 1; control.kill_switch[0]=1;
		function_control.dsm2_timeout=0;
		//X_state_Lat1 = get_lat_state();
		//X_state_Lon1 = get_lon_state();	
		read_barometer();
		//initial_alt = bmp_get_altitude_m();
		set_state(RUNNING);
		//fprintf(logger.GPS_logger,"time,deg_lon,min_lon,deg_lat,min_lat,speed,direction,gps_alt,hdop,fix\n");
		printf("First Iteration ");
		}

		//Keep all other threads from interfering from this point until unlock
		pthread_mutex_lock(&function_control.lock);
	/**********************************************************
	*    Read the IMU for Rotational Position and Velocity    *
	**********************************************************/
	
	//Bring 3 axes of accel, gyro and angle data in to this program
	for (i=0;i<3;i++) 
	{	
		transform.dmp_imu.data[i] = imu_data.fused_TaitBryan[i] + yaw_offset[i];
		transform.gyro_imu.data[i] = imu_data.gyro[i] * DEG_TO_RAD;
		transform.accel_imu.data[i] = imu_data.accel[i];
	}
	//Convert from IMU coordinate system to drones
	matrix_times_col_vec(transform.IMU_to_drone_dmp, transform.dmp_imu, &transform.dmp_drone);
	matrix_times_col_vec(transform.IMU_to_drone_gyro, transform.gyro_imu, &transform.gyro_drone);
	matrix_times_col_vec(transform.IMU_to_drone_accel, transform.accel_imu, &transform.accel_drone);


	//Subtract the gravity vector component from lat/lon accel
	transform.accel_drone.data[0]+= 9.8 * sin(transform.dmp_drone.data[0]);
	transform.accel_drone.data[1]+= 9.8 * sin(transform.dmp_drone.data[1]);
	
	//lowpass the accel data and subtract the biases
	//accel_data.accel_Lat	= marchFilter(&filters.LPF_Accel_Lat,transform.accel_drone.data[0]-accel_bias[0]);
	//accel_data.accel_Lon	= marchFilter(&filters.LPF_Accel_Lon,transform.accel_drone.data[1]-accel_bias[1]);
	accel_data.accelz		= transform.accel_drone.data[2]-accel_bias[2];

	//control.pitch 			= marchFilter(&filters.LPF_pitch,transform.dmp_drone.data[0]);
	//control.pitch 			= dmp_drone.data[0];
	//control.roll 				= marchFilter(&filters.LPF_roll,transform.dmp_drone.data[1]);
	control.yaw[1] 			= control.yaw[0];	
	control.yaw[0] 			= transform.dmp_drone.data[2] + control.num_wraps*2*M_PI;

	if(fabs(control.yaw[0] - control.yaw[1])  > 5)
	{
		if(control.yaw[0] > control.yaw[1]) control.num_wraps--;
		if(control.yaw[0] < control.yaw[1]) control.num_wraps++;
	}
	control.yaw[0]= transform.dmp_drone.data[2] + control.num_wraps*2*M_PI;	
	
	control.d_pitch			= transform.gyro_drone.data[0];
	control.d_roll			= transform.gyro_drone.data[1];
	control.d_yaw			= transform.gyro_drone.data[2];
		
		
	//Store some info in the accel_data struct to send to kalman filer
	accel_data.pitch=control.pitch;
	accel_data.roll=control.roll;
	accel_data.yaw[0]=control.yaw[0];
	
		
	if(First_Iteration){
		setpoint.yaw_ref[0]=control.yaw[0];
		First_Iteration=0;
		control.yaw_ref_offset = control.yaw[0];
		printf("Started \n");
	}
	

		
	/**********************************************************
	*           Read the RC Controller for Commands           *
	**********************************************************/
	
	if(is_new_dsm2_dataMS()){
		//printf("DSM2 In\n");
		
		//Reset the timout counter back to zero
		function_control.dsm2_timeout=0;
		
		//Set the throttle
		control.throttle=(get_dsm2_ch_normalizedMS(1)+1)*0.5*(MAX_THROTTLE-MIN_THROTTLE)+MIN_THROTTLE;
		
		//Keep the aircraft at a constant height while making manuevers 
		control.throttle *= 1/(cos(control.pitch)*cos(control.roll));
		
		//Set Yaw, RC Controller acts on Yaw velocity, save a history for integration
		setpoint.yaw_rate_ref[1]=setpoint.yaw_rate_ref[0];		
		setpoint.yaw_rate_ref[0]=get_dsm2_ch_normalizedMS(4)*MAX_YAW_RATE;
		
		//Apply a deadzone to keep integrator from wandering
		if(fabs(setpoint.yaw_rate_ref[0])<0.05) {
			setpoint.yaw_rate_ref[0]=0;
		}
		
		//Kill Switch
		control.kill_switch[0]=get_dsm2_ch_normalizedMS(5)/2;
		
		//Auxillary Switch
		setpoint.Aux=get_dsm2_ch_normalizedMS(6); 
		
		if(setpoint.Aux>0 || 1){ //Remote Controlled Flight
			
			setpoint.roll_ref=-get_dsm2_ch_normalizedMS(2)*MAX_ROLL_RANGE;	//DSM2 Receiver is inherently positive to the left
			setpoint.pitch_ref=get_dsm2_ch_normalizedMS(3)*MAX_PITCH_RANGE;
			
			//Convert from Drone Coordinate System to User Coordinate System
			float P_R_MAG=pow(pow(setpoint.roll_ref,2)+pow(setpoint.pitch_ref,2),0.5);
			float Theta_Ref=atan2f(setpoint.pitch_ref,setpoint.roll_ref);
			setpoint.roll_ref =P_R_MAG*cos(Theta_Ref-control.yaw[0]+control.yaw_ref_offset);
			setpoint.pitch_ref=P_R_MAG*sin(Theta_Ref-control.yaw[0]+control.yaw_ref_offset);
		}
		else{ //Flight by GPS and/or Lidar
			//control.alt_rate_ref=get_dsm2_ch_normalizedMS(1)*MAX_ALT_SPEED;
		}
	}
	else{ //check to make sure too much time hasn't gone by since hearing the RC
		
		#ifndef DEBUG
		function_control.dsm2_timeout=function_control.dsm2_timeout+1;
		
		if(function_control.dsm2_timeout>1.5/DT) {
			printf("\nLost Connection with Remote!! Shutting Down Immediately \n");	
			fprintf(Error_logger,"\nLost Connection with Remote!! Shutting Down Immediately \n");	
			set_state(EXITING);
		}
		#endif
	
	}
		
	
	#ifdef DEBUG
		control.throttle = MIN_THROTTLE;
		setpoint.Aux = 0;
		setpoint.roll_ref = 0;
		setpoint.pitch_ref = 0;
		setpoint.yaw_rate_ref[0] = 0;
		setpoint.yaw_rate_ref[1] = 0;
		setpoint.roll_ref = 0;
	#endif
	
		
	/************************************************************************
	*                   	Throttle Controller                             *
	************************************************************************/
	
 
//	float throttle_compensation = 1 / cos(control.roll);
//	throttle_compensation *= 1 / cos(control.pitch);		

	
		
	/************************************************************************
	* 	                  Pitch and Roll Controllers                        *
	************************************************************************/
	if(setpoint.Aux>=0 || 1){
		//Using Remote Control
		function_control.gps_pos_mode=0;
	}
	else{
		//Using GPS Control
		
		if(function_control.gps_pos_mode==0){
			printf("gps position mode\n");
			setpoint.lat_setpoint=GPS_data.pos_lat;
			setpoint.lon_setpoint=GPS_data.pos_lon;
			function_control.gps_pos_mode=1;
			control.standing_throttle=control.throttle;
			control.alt_ref=GPS_data.gps_altitude+1;
		}
		control.lat_error=setpoint.lat_setpoint-GPS_data.pos_lat;
		control.lon_error=setpoint.lon_setpoint-GPS_data.pos_lon;
		
		setpoint.pitch_ref=0.14*marchFilter(&filters.Outer_Loop_TF_pitch,control.lat_error);
		setpoint.roll_ref=-0.14*marchFilter(&filters.Outer_Loop_TF_roll,control.lon_error);
		
		setpoint.pitch_ref=saturateFilter(setpoint.pitch_ref,-0.2,0.2);
		setpoint.roll_ref=saturateFilter(setpoint.roll_ref,-0.2,0.2);
		
		//Convert to Drone Coordinate System from User Coordinate System
		float P_R_MAG=pow(pow(setpoint.roll_ref,2)+pow(setpoint.pitch_ref,2),0.5);
		float Theta_Ref=atan2f(setpoint.pitch_ref,setpoint.roll_ref);
		setpoint.roll_ref=P_R_MAG*cos(Theta_Ref-control.yaw[0]);
		setpoint.pitch_ref=P_R_MAG*sin(Theta_Ref-control.yaw[0]);
		
		control.alt_error=control.alt_ref-GPS_data.gps_altitude;
		//control.throttle=0.12*marchFilter(&filters.Throttle_controller,control.alt_error);
		
		//control.throttle=saturateFilter(control.throttle,-0.15,0.15)+control.standing_throttle;
	}
	
	//Filter out any high freq noise coming from yaw in the CS translation
	setpoint.filt_pitch_ref = marchFilter(&filters.LPF_Yaw_Ref_P,setpoint.pitch_ref);
	setpoint.filt_roll_ref = marchFilter(&filters.LPF_Yaw_Ref_R,setpoint.roll_ref);
	
	//Filter out high frequency noise in Raw Gyro data
	control.d_pitch_f = marchFilter(&filters.LPF_d_pitch,control.d_pitch);			
	control.d_roll_f = marchFilter(&filters.LPF_d_roll,control.d_roll);

	//Set the Pitch reference Setpoint
	control.dpitch_setpoint=((setpoint.filt_pitch_ref - control.pitch)*  
					6 - control.d_pitch_f);
	//Set the Roll reference Setpoint
	control.droll_setpoint=((setpoint.filt_roll_ref - control.roll)* 
					6 - control.d_roll_f);
	
	//Apply the PD Controllers 
	marchFilter(&filters.pitch_PD,control.dpitch_setpoint);
	marchFilter(&filters.roll_PD,control.droll_setpoint);				
	

	
	/************************************************************************
	*                        	Yaw Controller                              *
	************************************************************************/	
	control.d_yaw_f = marchFilter(&filters.LPF_d_yaw,control.d_yaw);
	
	setpoint.yaw_ref[1]=setpoint.yaw_ref[0];
	setpoint.yaw_ref[0]=setpoint.yaw_ref[1]+(setpoint.yaw_rate_ref[0]+setpoint.yaw_rate_ref[1])*DT/2;
	
	marchFilter(&filters.yaw_PD,setpoint.yaw_ref[0]-control.yaw[0]);
	

	
	/************************************************************************
	*                   	Apply the Integrators                           *
	************************************************************************/	
		
	if(control.throttle<MIN_THROTTLE+.01){	
		function_control.integrator_reset++;
		function_control.integrator_start=0;
	}else{
		function_control.integrator_reset=0;
		function_control.integrator_start++;
	}
	
	if(function_control.integrator_reset==300){// if landed, reset integrators and Yaw error
		setpoint.yaw_ref[0]=control.yaw[0];
		control.droll_err_integrator=0; 
		control.dpitch_err_integrator=0;
		control.dyaw_err_integrator=0;
	}
	
		
	//only use integrators if airborne (above minimum throttle for > 1.5 seconds)
	if(function_control.integrator_start >  400){
		control.droll_err_integrator  += control.uroll  * DT;
		control.dpitch_err_integrator += control.upitch * DT;
		control.dyaw_err_integrator += control.uyaw * DT;		
		
		control.upitch+=PITCH_ROLL_KI * control.dpitch_err_integrator;
		control.uroll +=PITCH_ROLL_KI * control.droll_err_integrator;
		control.uyaw+=YAW_KI * control.dyaw_err_integrator;
	}
	
	//Apply a saturation filter
	control.upitch=saturateFilter(filters.pitch_PD.current_output,-MAX_PITCH_COMPONENT,MAX_PITCH_COMPONENT);
	control.uroll=saturateFilter(filters.roll_PD.current_output,-MAX_ROLL_COMPONENT,MAX_ROLL_COMPONENT);
	control.uyaw=saturateFilter(filters.yaw_PD.current_output,-MAX_YAW_COMPONENT,MAX_YAW_COMPONENT);
	
	/************************************************************************
	*  Mixing
	*           	      black				yellow
	*                          CCW 1	  2 CW			
	*                          	   \ /				Y
	*	                           / \            	|_ X
	*                         CW 3	  4 CCW
	*                 	  yellow       	    black
	************************************************************************/
	
	control.u[0]=control.throttle+control.uroll-control.upitch+control.uyaw;
	control.u[1]=control.throttle-control.uroll-control.upitch-control.uyaw;
	control.u[2]=control.throttle+control.uroll+control.upitch-control.uyaw;
	control.u[3]=control.throttle-control.uroll+control.upitch+control.uyaw;		

	
	float largest_value = 1;
	float smallest_value = 0;

	for(i=0;i<4;i++){ 
		if(control.u[i]>largest_value)largest_value=control.u[i];
		
		if(control.u[i]<smallest_value)control.u[i]=0;
	}
			
	// if upper saturation would have occurred, reduce all outputs evenly
	if(largest_value>1){
		float offset = largest_value - 1;
		for(i=0;i<4;i++) control.u[i]-=offset;
	}

	
	#ifndef DEBUG
	//Send Commands to Motors
	if(get_state()!=EXITING){
		for(i=0;i<4;i++){
			send_esc_pulse_normalized(i+1,control.u[i]);
		}
	}
	else{
		for(i=0;i<4;i++){
			control.u[i] = 0;
			send_esc_pulse_normalized(i+1,control.u[i]);
		}	
	}
	if(control.kill_switch[0] < .5) {
		printf("\nKill Switch Hit! Shutting Down\n");
		fprintf(Error_logger,"\nKill Switch Hit! Shutting Down\n");	
		set_state(EXITING);
	}		
	#endif
	/*
	i1++;
	if (i1 == 8) // Only read the barometer at 25Hz
	{
		// perform the i2c reads to the sensor, this takes a bit of time
		if(read_barometer()<0){
			printf("\rERROR: Can't read Barometer");
			fflush(stdout);
		}
		i1=0;
	}
	
	baro_alt = bmp_get_altitude_m() - initial_alt;

	fflush(stdout);
	*/
	
	clock_gettime(CLOCK_MONOTONIC, &function_control.log_time);
	control.time=(float)(function_control.log_time.tv_sec - function_control.start_time.tv_sec) + 
						((float)(function_control.log_time.tv_nsec - function_control.start_time.tv_nsec) / 1000000000) ;
	
	/*
	logger.new_entry.time			= control.time;	
	logger.new_entry.pitch			= control.pitch;	
	logger.new_entry.roll			= control.roll;
	logger.new_entry.yaw			= control.yaw[0];
	logger.new_entry.d_pitch		= control.d_pitch;	
	logger.new_entry.d_roll			= control.d_roll;
	logger.new_entry.d_yaw			= control.d_yaw;
	logger.new_entry.u_1			= control.u[0];
	logger.new_entry.u_2			= control.u[1];
	logger.new_entry.u_3			= control.u[2];
	logger.new_entry.u_4			= control.u[3];
	logger.new_entry.throttle		= control.throttle;
	logger.new_entry.upitch			= control.upitch;	
	logger.new_entry.uroll			= control.uroll;
	logger.new_entry.uyaw			= control.uyaw;
	logger.new_entry.pitch_ref		= setpoint.pitch_ref;
	logger.new_entry.roll_ref		= setpoint.roll_ref;
	logger.new_entry.yaw_ref		= setpoint.yaw_ref[0];
	logger.new_entry.yaw_rate_ref	= setpoint.yaw_rate_ref[0];
	logger.new_entry.Aux			= setpoint.Aux;
	logger.new_entry.lat_error		= control.lat_error;
	logger.new_entry.lon_error		= control.lon_error;
//	logger.new_entry.kalman_lat		= X_state_Lat1->data[0];
//	logger.new_entry.kalman_lon		= X_state_Lon1->data[0];
	logger.new_entry.accel_lat		= accel_data.accel_Lat;
	logger.new_entry.accel_lon		= accel_data.accel_Lon;
	logger.new_entry.baro_alt		= control.baro_alt;
	log_core_data(&logger.core_logger, &logger.new_entry);
	
	
	
	fprintf(logger,"%4.5f,",control.time);
	fprintf(logger,"%0.4f,%0.4f,%0.4f,%0.4f,",control.u[0],control.u[1],control.u[2],control.u[3]);
	fprintf(logger,"%1.3f,%1.3f,%2.4f,",control.pitch,control.roll,control.yaw[0]);
	fprintf(logger,"%0.4f,%0.4f,%0.4f,",control.d_pitch_f,control.d_roll_f,control.d_yaw);
	fprintf(logger,"%0.4f,%2.2f,%2.2f,%2.4f,%2.4f,",control.throttle,setpoint.pitch_ref,setpoint.roll_ref,setpoint.yaw_ref[0],setpoint.yaw_rate_ref[0]);
	fprintf(logger,"%1.3f,",setpoint.filt_pitch_ref);
	fprintf(logger,"%0.4f,%0.4f,%0.4f,",control.upitch,control.uroll,control.uyaw);
	fprintf(logger,"%1.1f,%f,%f,",setpoint.Aux,control.lon_error,control.lat_error);
	fprintf(logger,"%2.3f,",control.yaw[0]+control.initial_yaw);
	fprintf(logger,"%f,%f,",X_state_Lat1->data[0],X_state_Lon1->data[0]);
	fprintf(logger,"%f,%f,",accel_data.accel_Lat,accel_data.accel_Lon);
	fprintf(logger,"\n");	
	*/
	

	/*
		//Print some stuff
		printf("\r ");
		printf("time %3.3f ", control.time);
	//	printf("Alt %2.2f ",lidar_data.altitude[0]);
	//	printf("vel %2.2f ",lidar_data.d_altitude[0]);		
	//	printf("H_d %3.3f ", control.height_damping);
	//	printf("Alt_ref %3.1f ",control.alt_ref);
	//	printf(" U1:  %2.2f ",control.u[0]);
	//	printf(" U2: %2.2f ",control.u[1]);
	//	printf(" U3:  %2.2f ",control.u[2]);
	//	printf(" U4: %2.2f ",control.u[3]);	
	//	printf(" TH %2.2f ", control.throttle);
	//	printf("Aux %2.1f ", setpoint.Aux);
	//	printf("function: %f",get_dsm2_ch_normalizedMS(6));
	//	printf("num wraps %d ",control.num_wraps);
	//	printf(" Pitch_ref %2.2f ", setpoint.filt_pitch_ref);
	//	printf(" Roll_ref %2.2f ", setpoint.filt_roll_ref);
	//	printf(" Yaw_ref %2.2f ", setpoint.yaw_ref[0]);
	//	printf(" Pitch %1.2f ", control.pitch);
	//	printf(" Roll %1.2f ", control.roll);
		printf(" Yaw %2.3f ", control.yaw[0]); 
	//	printf(" DPitch %1.2f ", control.d_pitch_f); 
	//	printf(" DRoll %1.2f ", control.d_roll_f);
	//	printf(" DYaw %2.3f ", control.d_yaw); 	
	//	printf(" uyaw %2.3f ", control.upitch); 		
	//	printf(" uyaw %2.3f ", control.uroll); 		
	//	printf(" uyaw %2.3f ", control.uyaw);
	//	printf(" GPS pos lat: %2.2f", GPS_data.pos_lat);
	//	printf(" GPS pos lon: %2.2f", GPS_data.pos_lon);
	//	printf(" HDOP: %f", GPS_data.HDOP);
	//	printf(" Acc_Lat %2.3f ", accel_data.accel_Lat);
	//	printf(" Acc_Lon %2.3f ", accel_data.accel_Lon);
	//	printf(" Acc_z %2.3f", accel_data.accelz);
	//	printf(" Pos_Lat %2.3f ", X_state_Lat1->data[0]);	
	//	printf(" Pos_Lon %2.3f ", X_state_Lon1->data[0]);
	//	printf("control: %d",get_state());
		printf("Baro Alt: %f ",baro_alt);
		*/
		
		/***************** Get GPS Data if available *******************/
	if(is_new_GPS_data()){
		
		accel_data.GPS_kal_flag = 1;
		 //printf("\n new GPS data in \n");
		fprintf(logger.GPS_logger,"%4.5f,",control.time);
		fprintf(logger.GPS_logger,"%3.0f,%f,",GPS_data.deg_longitude,GPS_data.min_longitude);
		fprintf(logger.GPS_logger,"%3.0f,%f,",GPS_data.deg_latitude,GPS_data.min_latitude);
		fprintf(logger.GPS_logger,"%f,%f,",GPS_data.speed,GPS_data.direction);
		fprintf(logger.GPS_logger,"%f,",GPS_data.gps_altitude);
		fprintf(logger.GPS_logger,"%2.2f,%d",GPS_data.HDOP,GPS_data.GPS_fix);
		fprintf(logger.GPS_logger,"\n");
		//	printf("%s",GPS_data.GGAbuf);
		//	printf("%s",GPS_data.VTGbuf);

		if (First_Iteration_GPS==1  && GPS_data.GPS_fix==1){
			control.initial_pos_lat=GPS_data.meters_lat;
			control.initial_pos_lon=GPS_data.meters_lon;
			First_Iteration_GPS=0;
			GPS_data.GPS_fix_check=1;
			printf("First Iteration GPS\n");
		}
		if(GPS_data.HDOP<4 && GPS_data.GPS_fix==1){
			GPS_data.pos_lat=GPS_data.meters_lat-control.initial_pos_lat;
			GPS_data.pos_lon=GPS_data.meters_lon-control.initial_pos_lon;
		}
	}
	pthread_mutex_unlock(&function_control.lock);
		
	
		
	return 0;
}
	
	
int main(int argc, char *argv[]){

	
	//Define some threads
	//pthread_t led_thread;
	pthread_t kalman_thread;
	pthread_t core_logging_thread;
	//pthread_t barometer_alt_threat;
	
	//Initialize some cape and beaglebone hardware
	if(initialize_cape()){
		printf("ERROR: failed to initialize_cape\n");
		return -1;
	}	
	
	
	if(initialize_barometer(OVERSAMPLE, INTERNAL_FILTER)<0){
		printf("initialize_barometer failed\n");
		return -1;
	}
//	pthread_create(&barometer_alt_threat, NULL, barometer_monitor, (void*) NULL);
	
	
	// set up IMU configuration
	imu_config_t imu_config = get_default_imu_config();
	imu_config.dmp_sample_rate = SAMPLE_RATE;
	imu_config.orientation = ORIENTATION_Z_UP;
	imu_config.accel_fsr = A_FSR_2G;
	imu_config.enable_magnetometer=1;
	
	control_variables_t *ptr1; 
	ptr1 = malloc(sizeof(control_variables_t));
	ptr1->pitch = 20;
	
	// start imu
	if(initialize_imu_dmp(&imu_data, imu_config, (void*)ptr1)){
		printf("ERROR: can't talk to IMU, all hope is lost\n");
		blink_led(RED, 5, 5);
		return -1;
	}

	//Initialize the remote controller
	//	initialize_dsm2MS();
	
	#ifndef DEBUG
	if(ready_check()){
		printf("Exiting Program \n");
		return -1;
	} //Toggle the kill switch a few times to signal it's ready
	#endif
	
	
	// start a core_log and logging thread
	if(start_core_log(&logger)<0){
		printf("WARNING: failed to open a core_log file\n");
	}
	else{
		pthread_create(&core_logging_thread, NULL, core_log_writer, &logger.core_logger);
	}

	sleep(2); //wait for the IMU to level off	

	init_rotation_matrix(&transform); //Initialize the rotation matrix from IMU to drone
	initialize_filters(&filters);

	//Start the GPS thread, flash the LED's if GPS has a fix
	GPS_data.GPS_init_check=GPS_init(argc, argv,&GPS_data);
	//pthread_create(&led_thread, NULL, LED_thread, (void*) &GPS_data);
	
	
	//Spawn the Kalman Filter Thread
	pthread_create(&kalman_thread, NULL , kalman_filter, (void*) NULL);
	
	//Give the ESCs a zero command before starting to prevent going into calibration
	//init_esc_hardware();	 
	 
	//Start the mutex lock to prevent threads from interfering
	if(pthread_mutex_init(&function_control.lock,NULL))
	{
		printf("Error Lock init failed\n");
	}
		
	//Start the control program
	set_imu_interrupt_func(&flight_core);
	
	printf("Starting \n");
	set_state(RUNNING);
	while (get_state() != EXITING) {
		sleep(1);
	}
	
	
	//stop_core_log(&logger.core_logger);// finish writing core_log
	
	//Join the threads for a safe process shutdown
	join_GPS_thread(&GPS_data);
	printf("GPS thread joined\n");
	pthread_join(kalman_thread, NULL);
	printf("Kalman thread joined\n");
	//pthread_join(led_thread, NULL);
	//printf("LED thread joined\n");
	pthread_join(core_logging_thread, NULL);
	printf("Logging thread joined\n");
	 
	// Close the log files
	close(GPS_data.GPS_file);
	fclose(logger.GPS_logger);
	
	
	fflush(stdout);
	cleanup_cape();
	return 0;
	}
	
	
accel_data_t* get_accel_pointer(){
	return &accel_data;
}
	
GPS_data_t* get_GPS_pointer(){
	return &GPS_data;
}