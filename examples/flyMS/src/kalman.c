
#include <stdio.h>
#include <stdlib.h>

#include "../include/flight_defs.h"
#include "../include/linear_algebra.h"
#include <robotics_cape.h>
#include"../include/flight_defs.h"
//#include"flyMS.h"
#include"../include/gps.h"
#include"../include/kalman.h"


/************** Globals ********************/

static accel_data_t *accel_data_kal;
vector_t X_state_Lat, X_state_Lon;



vector_t* get_lat_state(){
	return &X_state_Lat;
}

vector_t* get_lon_state(){
	return &X_state_Lon;
}


void* kalman_filter(void *ptr){		
	int i,j;
	
	matrix_t A_mat, Q_mat, A_mat_tr, eye;
	vector_t B_mat, H_mat;
	

	//Variable Matrices Latitude
	matrix_t tmp33_Lat, Pk_Lat, tmp33_2_Lat;
	vector_t tmp31_Lat, Kalman_gain_Lat;

	//Variable Matrices Longitude
	matrix_t tmp33_Lon, Pk_Lon, tmp33_2_Lon;
	vector_t tmp31_Lon, Kalman_gain_Lon;
	
	
	//Other Variable Matrices
	matrix_t Rot_matrix;
	vector_t accel_mat_global, accel_mat_local;
	

	static GPS_data_t *GPS_data;

	/************************************
	Allocate Memory for all the Matrices 
	************************************/
	
	//Constant Matrices
	A_mat = create_matrix(3,3);
	B_mat = create_vector(3);
	A_mat_tr = create_matrix (3,3);
	eye = create_matrix(3,3);
	

	Q_mat = create_matrix(3,3);
	H_mat = create_vector(3);
	
	//Latitude Matrices 
	X_state_Lat = create_vector(3);
	tmp31_Lat =  create_vector(3);
	tmp33_Lat = create_matrix(3,3);
	Pk_Lat = create_matrix(3,3);
	Kalman_gain_Lat = create_vector(3);
	tmp33_2_Lat = create_matrix(3,3);

	
	//Longitude Matrices
	X_state_Lon = create_vector(3);
	tmp31_Lon =  create_vector(3);
	tmp33_Lon = create_matrix(3,3);
	Pk_Lon = create_matrix(3,3);
	Kalman_gain_Lon = create_vector(3);
	tmp33_2_Lon = create_matrix(3,3);
	
	//Other Matrices
	accel_mat_local = create_vector(3);
	accel_mat_global = create_vector(3);
	Rot_matrix = create_matrix(3,3);
	 
	float Q_matrix[][3] = Q_MATRIX;
	float A_matrix[][3] = A_MATRIX;
	float B_matrix[][1] = B_MATRIX;
	float eye_mat[][3] = EYE;
	float H_Mat[][1] = H_MATRIX;
	
	for(i=0; i<3; i++){
		X_state_Lat.data[i]=0;
		X_state_Lon.data[i]=0;
	}

	GPS_data = get_GPS_pointer();
	accel_data_kal = get_accel_pointer();

	i=0; j=0;
	for(i=0; i<3; i++){
		B_mat.data[i]=B_matrix[i][0];	
		H_mat.data[i]=H_Mat[i][0];
		for(j=0; j<3; j++){
			A_mat.data[i][j]=A_matrix[i][j];
			A_mat_tr.data[i][j]=A_matrix[i][j];
			Q_mat.data[i][j]=Q_matrix[i][j];
			eye.data[i][j]=eye_mat[i][j];
			Pk_Lon.data[i][j] = eye_mat[i][j];
			Pk_Lat.data[i][j] = eye_mat[i][j];
		}
	}
	transpose_matrix(&A_mat_tr);

	sleep(5); // wait for other things to start before running
	
	while (get_state()!=EXITING)
	{
		accel_mat_local.data[0]=accel_data_kal->accel_Lat;
		accel_mat_local.data[1]=accel_data_kal->accel_Lon;
		accel_mat_local.data[2]=accel_data_kal->accelz;
		update_rot_matrix(&Rot_matrix);
		matrix_times_col_vec(Rot_matrix, accel_mat_local, &accel_mat_global);

		/*********************** LATITUDE ESTIMATION STEP KALMAN FILTER ******************************/
			//print_vector(X_state_Lat);

		//time update step of Kalman Filter
		matrix_times_col_vec(A_mat, X_state_Lat, &X_state_Lat);
		vector_times_scalar2(&B_mat, accel_mat_global.data[0], &tmp31_Lat);
		add_vectors(X_state_Lat, tmp31_Lat,&X_state_Lat);
		//#ifdef aslkfhasdkfh
		 
		 //Calculate Estimated Covariance Matric Pk_Lat 
		multiply_matrices(A_mat, Pk_Lat, &tmp33_Lat);
		multiply_matrices(tmp33_Lat, A_mat_tr, &tmp33_Lat);
		add_matrices(tmp33_Lat, Q_mat, &Pk_Lat);
		
		 /*********************** LONGITUDE ESTIMATION STEP KALMAN FILTER ******************************/
		//time update step of Kalman Filter
		matrix_times_col_vec(A_mat, X_state_Lon, &X_state_Lon);
		vector_times_scalar2(&B_mat, accel_mat_global.data[1], &tmp31_Lon);
		add_vectors(X_state_Lon, tmp31_Lon, &X_state_Lon);
		
		 //Calculate Estimated Covariance Matric Pk_Lon 
		multiply_matrices(A_mat, Pk_Lon, &tmp33_Lon);
		multiply_matrices(tmp33_Lon, A_mat_tr, &tmp33_Lon);
		add_matrices(tmp33_Lon, Q_mat, &Pk_Lon);
		
		 /*********************** LAT/LON UPDATE STEP KALMAN FILTER ******************************/	
		if (accel_data_kal->GPS_kal_flag)
		{
			float temp_float_lat = 1/(Pk_Lat.data[0][0]+R_MATRIX);
			for (i=0;i<3;i++) Kalman_gain_Lat.data[i] = Pk_Lat.data[i][0] * temp_float_lat;
			 
			//Update the estimated state accounting for measurement
			vector_times_scalar2(&Kalman_gain_Lat, GPS_data->pos_lat-X_state_Lat.data[0], &tmp31_Lat);
			//tmp31_Lat = vector_times_scalar2(&Kalman_gain_Lat, 0-X_state_Lat.data[0]);
			add_vectors(tmp31_Lat, X_state_Lat, &X_state_Lat);
			 
			//Calculate the next Covariance Matric Pk_Lat
			vector_outer_product(Kalman_gain_Lat, H_mat, &tmp33_Lon);
			matrix_times_scalar(&tmp33_Lat, -1);
			add_matrices(eye,tmp33_Lat, &tmp33_Lat);
			copy_matrix(Pk_Lat, &tmp33_2_Lat);
			multiply_matrices(tmp33_Lat,tmp33_2_Lat, &Pk_Lat);

			//printf("Updating Measurement \n"); 
			float temp_float_lon = 1/(Pk_Lon.data[0][0]+R_MATRIX);
			for (i=0;i<3;i++) Kalman_gain_Lon.data[i] = Pk_Lon.data[i][0] * temp_float_lon;
			 
			 //Update the estimated state accounting for measurement
			vector_times_scalar2(&Kalman_gain_Lon, GPS_data->pos_lon-X_state_Lon.data[0], &tmp31_Lon);
			//tmp31_Lon = vector_times_scalar2(&Kalman_gain_Lon, 0-X_state_Lon.data[0]);
			add_vectors(tmp31_Lon, X_state_Lon, &X_state_Lon);

			 //Calculate the next Covariance Matric Pk_Lon
			vector_outer_product(Kalman_gain_Lon, H_mat, &tmp33_Lon);
			matrix_times_scalar(&tmp33_Lon, -1);
			add_matrices(eye,tmp33_Lon, &tmp33_Lon);
			copy_matrix(Pk_Lon, &tmp33_2_Lon);
			multiply_matrices(tmp33_Lon,tmp33_2_Lon, &Pk_Lon);	
			
			accel_data_kal->GPS_kal_flag = 0;
		}
		usleep(DT*1000000);
 	
	//	#endif
	}
	
	
//	printf("Q mat2 \n");
	// print_matrix(Q_mat);

	// printf("A mat2\n");
	// print_matrix(A_mat);

//	printf("A mat tr2 \n");
//	 print_matrix(A_mat_tr);	 

	 printf("\n P_k Latitude\n");
	 print_matrix(Pk_Lat);
   
   	 printf("\n P_k Longitude\n");
	 print_matrix(Pk_Lon);
   
   /*
   	printf("tmp33_Lat \n");
	print_matrix(tmp33_Lat);	

   	 printf("\n tmp33_Lon\n");
	print_matrix(tmp33_Lon);	
	 
	printf("\n tmp33_2_Lat\n");
	print_matrix (tmp33_2_Lat);
	
	printf("\n P_k tmp33_2_Lon\n");
	print_matrix(tmp33_2_Lon);
   */
   
	printf("\n X_state_Lat\n");	
	print_vector(X_state_Lat);
	
	printf("\n X_state_Lon\n");	
	print_vector(X_state_Lon);
   
  	printf("\n Kalman_gain_Lat\n");
	print_vector(Kalman_gain_Lat);
	
	printf("\n Kalman_gain_Lon\n");	
	print_vector(Kalman_gain_Lon);

	 
	//fclose(logger);
	
	//Unallocate all the vectors and matrices
	destroy_matrix(&A_mat);
	destroy_vector(&B_mat);
	destroy_matrix(&A_mat_tr);
	destroy_matrix(&eye);

	destroy_matrix(&Q_mat);
	destroy_vector(&H_mat);

	destroy_vector(&X_state_Lat);
	destroy_vector(&tmp31_Lat);
	destroy_matrix(&tmp33_Lat);
	destroy_matrix(&Pk_Lat);
	destroy_vector(&Kalman_gain_Lat);
	destroy_matrix(&tmp33_2_Lat);

	destroy_vector(&X_state_Lon);
	destroy_vector(&tmp31_Lon);
	destroy_matrix(&tmp33_Lon);
	destroy_matrix(&Pk_Lon);
	destroy_vector(&Kalman_gain_Lon);
	destroy_matrix(&tmp33_2_Lon);
	
	destroy_vector(&accel_mat_local);
	destroy_vector(&accel_mat_global);
	destroy_matrix(&Rot_matrix);
	
	return NULL;
}

void update_rot_matrix(matrix_t* Rot_matrix){
	int i, j;
	float ROTATION_MAT[][3] = ROTATION_MATRIX;
	for(i=0; i<3; i++){
		for(j=0; j<3; j++){
		Rot_matrix->data[i][j]=ROTATION_MAT[i][j];
		}
	}
}

