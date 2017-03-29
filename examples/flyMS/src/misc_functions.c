#include <robotics_cape.h>
#include "flyMS.h"
#include <pthread.h>
#include "gps.h"

//Coordinate system transformations matrices



int ready_check(control_variables_t *control){
	//Toggle the kill switch to get going, to ensure controlled take-off
	//Keep kill switch down to remain operational
    int count=1;
	printf("Toggle the kill swtich twice and leave up to initialize\n");
	while(count<6 && get_state()!=EXITING){
		zero_escs();
		if(is_new_dsm2_dataMS()){
			control->kill_switch[1]=control->kill_switch[0];
			control->kill_switch[0]=get_dsm2_ch_normalizedMS(5);
			printf("Value: %f \r",control->kill_switch[0]);
			usleep(100000);
			if(control->kill_switch[0] < -0.75 && control->kill_switch[1] > 0.15){
			count++;
			} 
			if(control->kill_switch[0] > 0.75 && control->kill_switch[1] < 0.35){
			count++;
			}
			usleep(10000);
			
		}
		usleep(10000);
	}
	
	//make sure the kill switch is in the position to fly before starting
	while(control->kill_switch[0] < 0.5 && get_state()!=EXITING)
		{
		zero_escs();
		if(is_new_dsm2_dataMS()){
			control->kill_switch[0]=get_dsm2_ch_normalizedMS(5);	
			}
		usleep(10000);
		}
	
	if(get_state() == EXITING)
	{
		return -1;
	}
	
	printf("\nInitialized! Starting program\n");
	zero_escs(); 
	return 0;
}

void* LED_thread(void *ptr){
	
	led_thread_t *GPS_ready= (led_thread_t*)ptr;
	
	const char *filepath0 = "/sys/class/leds/beaglebone:green:usr0/brightness";
	const char *filepath1 = "/sys/class/leds/beaglebone:green:usr1/brightness";
	const char *filepath2 = "/sys/class/leds/beaglebone:green:usr2/brightness";
	const char *filepath3 = "/sys/class/leds/beaglebone:green:usr3/brightness";
	int i=0;
	FILE *file0 = NULL;
	FILE *file1 = NULL;
	FILE *file2 = NULL;
	FILE *file3 = NULL;
	
	for(i=0;i<50;i++){
		if(get_state() == EXITING) pthread_exit(NULL);
		
		if((file0 = fopen(filepath0, "r+")) != NULL){
			fwrite("1", sizeof(char), 1, file0);
			fclose(file0);
		}
		
		if((file1 = fopen(filepath1, "r+")) != NULL){
			fwrite("1", sizeof(char), 1, file1);
			fclose(file1);
		}
		
		if((file2 = fopen(filepath2, "r+")) != NULL){
			fwrite("1", sizeof(char), 1, file2);
			fclose(file2);
		}
		
		if((file3 = fopen(filepath3, "r+")) != NULL){
			fwrite("1", sizeof(char), 1, file3);
			fclose(file3);
		}
		
		usleep(500000-400000*GPS_ready->GPS_fix_check);
		if(get_state() == EXITING) pthread_exit(NULL);
		
		if(GPS_ready->GPS_init_check==-1){
			if((file0 = fopen(filepath0, "r+")) != NULL){
				fwrite("0", sizeof(char), 1, file0);
				fclose(file0);
			}
			
			if((file1 = fopen(filepath1, "r+")) != NULL){
				fwrite("0", sizeof(char), 1, file1);
				fclose(file1);
			}			
		
			if((file2 = fopen(filepath2, "r+")) != NULL){
			fwrite("0", sizeof(char), 1, file2);
			fclose(file2);
			}
		
			if((file3 = fopen(filepath3, "r+")) != NULL){
				fwrite("0", sizeof(char), 1, file3);
				fclose(file3);
			}
		}
		usleep(500000-400000*GPS_ready->GPS_fix_check);
	}
	return NULL;
  }
	



	
	
	
/************************************************************************
*	initialize_filters()
*	setup of feedback controllers used in flight core
************************************************************************/
int initialize_filters(filters_t *filters){

	//PD Controller (I is done manually)
	filters->pitch_PD = generatePID(PITCH_ROLL_KP, 0, PITCH_ROLL_KD, 0.15, 0.005);
	filters->roll_PD  = generatePID(PITCH_ROLL_KP, 0, PITCH_ROLL_KD, 0.15, 0.005);
	filters->yaw_PD   = generatePID(YAW_KP,		  0, YAW_KD,	    0.15, 0.005);
	
	//Gains on Low Pass Filter for raw gyroscope output
	
	// Butterworth Filter 4th order 0.3 cutoff 
	//float num[5] = {0.0186, 0.0743, 0.1114, 0.0743, 0.0186}; 
	//float den[5] = { 1.0000, -1.5704, 1.2756, -0.4844, 0.0762};
	
	//elliptic filter 4th order 1 dB passband ripple 60 dB min Cutoff 0.2 cutoff frq
	float num[5] = { 0.0059,  0.0053,  0.0096,  0.0053,  0.0059}; 
	float den[5] = { 1.0000, -3.0477,  3.8240, -2.2926,  0.5523};
	
	filters->LPF_d_pitch = initialize_filter(4, DT, num, den);		
	filters->LPF_d_roll= initialize_filter(4, DT, num, den);	
	filters->LPF_d_yaw = initialize_filter(4, DT, num, den);
	
	//ellip filter, 5th order .5 pass 70 stop .05 cutoff
	float num3[6] = { 0.00041175441,  -0.00087875614,   0.00053292744,   0.00053292744,  -0.00087875614,  0.00041175441};
	float den3[6] =	{ 1,  -4.65651332,   8.74687738,  -8.28155697,   3.95103526,  -0.75971050};
//	float num3[6] =	{0.0045,    0.0006,    0.0052,    0.0052,    0.0006,    0.0045};
//	float den3[6] =	{1.0000,   -3.6733,    5.8400,   -4.9357,    2.2049,   -0.4153};

	filters->LPF_Accel_Lat = initialize_filter(5, DT, num3, den3);							
	filters->LPF_Accel_Lon = initialize_filter(5, DT, num3, den3);		
	
	//Gains on Low Pass Filter for Yaw Reference		
	float num2[4] = {  0.0317,    0.0951,    0.0951,    0.0317};
	float den2[4] = { 1.0000,   -1.4590,    0.9104,   -0.1978};					
	filters->LPF_Yaw_Ref_P = initialize_filter(3, DT, num2, den2);							
	filters->LPF_Yaw_Ref_R = initialize_filter(3, DT, num2, den2);	
	
/*
	float num3[3] = {  0.0055 ,   0.0111 ,   0.0055};
	float den3[3] = {   1.0000 ,  -1.7786  ,  0.8008};					
	LPF_Height_Damping = initialize_filter(2, DT, num3, den3);				
*/

//	float num4[2] = { 2/(2+DT*DECAY_CONST),  -2/(2+DT*DECAY_CONST)+0.001};
//	float den4[2] = {   1.0000 , (DT*DECAY_CONST-2)/(DT*DECAY_CONST+2)};	
//	filters->Outer_Loop_TF_pitch = initialize_filter(1, DT, num4, den4);		
//	filters->Outer_Loop_TF_roll = initialize_filter(1, DT, num4, den4);
	//Throttle_controller = initialize_filter(1, DT, num4, den4);

	//4th order ellip .1 dp PB 60 dB SB 0.2 wn
	float num5[5] = {0.0088,    0.0144,    0.0197,    0.0144,    0.0088};
	float den5[5] = {1.0000,   -2.6537,    2.9740,   -1.5989,    0.3455};	
	filters->LPF_pitch = initialize_filter(4, DT, num5, den5);		
	filters->LPF_roll = initialize_filter(4, DT, num5, den5);	
	
	//zeroFilter(&core_state.yaw_ctrl);
	zeroFilter(&filters->LPF_d_pitch);
	zeroFilter(&filters->LPF_d_roll);
	zeroFilter(&filters->LPF_d_yaw);
	zeroFilter(&filters->LPF_Yaw_Ref_P);
	zeroFilter(&filters->LPF_Yaw_Ref_R);
	zeroFilter(&filters->Outer_Loop_TF_pitch);
	zeroFilter(&filters->Outer_Loop_TF_roll);
	return 0;
}

int init_rotation_matrix(tranform_matrix_t *transform){
	float pitch_offset, roll_offset, yaw_offset;
	int i,j;
	transform->IMU_to_drone_dmp = create_matrix(3,3);
	transform->IMU_to_drone_gyro = create_matrix(3,3);
	transform->IMU_to_drone_accel = create_matrix(3,3);
	
	transform->dmp_imu = create_vector(3);
	transform->gyro_imu = create_vector(3);
	transform->accel_imu = create_vector(3);
	transform->dmp_drone = create_vector(3);
	transform->gyro_drone = create_vector(3);
	transform->accel_drone = create_vector(3);
	
	
	pitch_offset = 0; roll_offset = M_PI; yaw_offset = - 5* M_PI/4;
	float ROTATION_MAT1[][3] = ROTATION_MATRIX1;
	for(i=0; i<3; i++){
		for(j=0; j<3; j++){
		transform->IMU_to_drone_dmp.data[i][j]=ROTATION_MAT1[i][j];
		}
	}
	
	
	pitch_offset = 0; roll_offset = M_PI; yaw_offset = -5 * M_PI/4;
	float ROTATION_MAT2[][3] = ROTATION_MATRIX1;
	for(i=0; i<3; i++){
		for(j=0; j<3; j++){
		transform->IMU_to_drone_gyro.data[i][j]=ROTATION_MAT2[i][j];
		}
	}
	
	
	pitch_offset = 0; roll_offset = M_PI; yaw_offset = M_PI/4;
	float ROTATION_MAT3[][3] = ROTATION_MATRIX1;
	for(i=0; i<3; i++){
		for(j=0; j<3; j++){
		transform->IMU_to_drone_accel.data[i][j]=ROTATION_MAT3[i][j];
		}
	}
	
	printf("Vectors and matrices initialiazed \n");
	return 0;
}

void zero_escs(){
	int i;
	for (i=0;i<8;i++){
		send_esc_pulse_normalized(i+1, 0);	
	}
}


void init_esc_hardware(){
	uint8_t p;
	for (p = 0; p < 100; p++)
	{
		send_esc_pulse_normalized_all(0); 
	usleep(1000000/100); //run at about 50 hz
	}
}


void* barometer_monitor(void* ptr){
	control_variables_t *control= (control_variables_t*)control;
		
	while(get_state()!=EXITING)
	{
		// perform the i2c reads to the sensor, this takes a bit of time
		if(read_barometer()<0){
			printf("\rERROR: Can't read Barometer");
			fflush(stdout);
			continue;
		}
		control->baro_alt = bmp_get_altitude_m();
		usleep(1000000/BMP_CHECK_HZ);
	}
	return NULL;
}
