/*******************************************************************************
* test_imu.c
*
* James Strawson 2016
* This serves as an example of how to read the IMU with direct reads to the
* sensor registers. To use the DMP or interrupt-driven timing see test_dmp.c
*******************************************************************************/
#include <useful_includes.h>
#include <robotics_cape.h>
int read_imu_data(); 
imu_data_t data;
FILE * logger;

#define cR cos(roll_offset)
#define sR sin(roll_offset)
#define cP cos(pitch_offset)
#define sP sin(pitch_offset)
#define cY cos(yaw_offset)
#define sY sin(yaw_offset)

#define ROTATION_MATRIX		{{cP*cY, -cR*sY+sR*sP*cY,  sR*sY+cR*sP*cY}, \
							 {cP*sY,  cR*cY+sR*sP*sY, -sR*cY+cR*sP*sY}, \
							 {-sP  ,  sR*cP,		   cR*cP		 }}


matrix_t IMU_to_drone_dmp, IMU_to_drone_gyro, IMU_to_drone_accel;
vector_t dmp_imu, gyro_imu, accel_imu;
vector_t dmp_drone, gyro_drone, accel_drone;

	int i, j;

int main(){

	float pitch_offset, roll_offset, yaw_offset;
	if(initialize_cape()){
		printf("ERROR: failed to initialize_cape\n");
		return -1;
	}
	
	
	IMU_to_drone_dmp = create_matrix(3,3);
	IMU_to_drone_gyro = create_matrix(3,3);
	IMU_to_drone_accel = create_matrix(3,3);
	
	dmp_imu = create_vector(3);
	gyro_imu = create_vector(3);
	accel_imu = create_vector(3);
	dmp_drone = create_vector(3);
	gyro_drone = create_vector(3);
	accel_drone = create_vector(3);
	
	
	pitch_offset = 0; roll_offset = M_PI; yaw_offset = -5 * M_PI/4;
	
	float ROTATION_MAT1[][3] = ROTATION_MATRIX;
	for(i=0; i<3; i++){
		for(j=0; j<3; j++){
		IMU_to_drone_dmp.data[i][j]=ROTATION_MAT1[i][j];
		}
	}
	
	pitch_offset = 0; roll_offset = M_PI; yaw_offset = -5 * M_PI/4;
	float ROTATION_MAT2[][3] = ROTATION_MATRIX;
	for(i=0; i<3; i++){
		for(j=0; j<3; j++){
		IMU_to_drone_gyro.data[i][j]=ROTATION_MAT2[i][j];
		}
	}
	
	
	pitch_offset = 0; roll_offset = M_PI; yaw_offset = M_PI/4;
	float ROTATION_MAT3[][3] = ROTATION_MATRIX;
	for(i=0; i<3; i++){
		for(j=0; j<3; j++){
		IMU_to_drone_accel.data[i][j]=ROTATION_MAT3[i][j];
		}
	}
	
	
	
	
	
	logger = fopen("logger.txt","w+");
		// set up IMU configuration
	imu_config_t imu_config = get_default_imu_config();
	imu_config.dmp_sample_rate = 200;
	imu_config.orientation = ORIENTATION_Z_UP;
	imu_config.enable_magnetometer=1;
	
	// start imu
	if(initialize_imu_dmp(&data, imu_config)){
		printf("ERROR: can't talk to IMU, all hope is lost\n");
		blink_led(RED, 5, 5);
		return -1;
	}
	
	// print a header
	printf("\n");
	printf("   Accel XYZ(m/s^2)  |");
	printf("   Angle XYZ (rad)  |");
	printf("  Mag Field XYZ(uT)  |");
	printf(" Temp (C)");
	printf("\n");
	
	
	//now just wait, print_data will run
	set_imu_interrupt_func(&read_imu_data);
	
	
	// chill until something exits the program
	while(get_state()!=EXITING){
		usleep(10000);
	}
	fclose(logger);
		power_off_imu();
	cleanup_cape();
	return 0;
}


int read_imu_data(){


	for (i=0;i<3;i++) 
	{	
		dmp_imu.data[i] = data.fused_TaitBryan[i];
		gyro_imu.data[i] = data.gyro[i];
		accel_imu.data[i] = data.accel[i];
	}
	
	dmp_drone = matrix_times_col_vec(IMU_to_drone_dmp, dmp_imu);
	gyro_drone = matrix_times_col_vec(IMU_to_drone_gyro, gyro_imu);
	accel_drone = matrix_times_col_vec(IMU_to_drone_accel, accel_imu);

	accel_drone.data[0]+= 9.8 * sin(dmp_drone.data[0]);
	accel_drone.data[1]+= 9.8 * sin(dmp_drone.data[1]);
	
		printf("\r");
		

		 printf("%6.2f %6.2f %6.2f |",		accel_drone.data[0],\
											accel_drone.data[1],\
											accel_drone.data[2]);
								

		 printf("%6.1f %6.1f %6.1f |",	gyro_drone.data[TB_PITCH_X],\
										gyro_drone.data[TB_ROLL_Y],\
										gyro_drone.data[TB_YAW_Z]);
								 

		printf("%2.3f %2.3f %2.3f |",	dmp_drone.data[0],\
											dmp_drone.data[1],\
											dmp_drone.data[2]);
											
											
			
/******************************** fprintf commands *****************************/
		fprintf(logger,"%f,%f,%f,",		accel_drone.data[0],\
										accel_drone.data[1],\
										accel_drone.data[2]);
										
		fprintf(logger,"%f,%f,%f\n",		gyro_drone.data[TB_PITCH_X],\
										gyro_drone.data[TB_ROLL_Y],\
										gyro_drone.data[TB_YAW_Z]);										
			
		if(read_imu_temp(&data)<0){
			printf("read temp data failed\n");
		}
		else printf(" %4.1f ", data.temp);
														
		fflush(stdout);

	

	return 0;
}
