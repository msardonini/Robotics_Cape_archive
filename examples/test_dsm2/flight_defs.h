#define MAX_ARRAY_SIZE 32
#define DEFAULT_SAMPLE_RATE	200
#define DT 0.005
#define K 0.115
#define Integrator_TH 0.45
#define BATTERY_VOLTAGE_TH -1 // keep negative until voltage divider is implemented
#define MAX_PITCH_RANGE 0.666 // in radians
#define MAX_ROLL_RANGE 0.666 // in radians
#define MAX_YAW_RATE 2.0 //in Radians per second
#define ALT_DAMPING_GAIN .003 // in D.C. per cm/second
#define ALT_PROPORTIONAL_GAIN 0.002 // in D.C. per cm
#define MAX_ALT_SPEED 10
#define DECAY_CONST 2.2
#define MIN_THROTTLE 0.15
#define MAX_THROTTLE 0.9
#define MAX_PITCH_COMPONENT 0.25
#define MAX_ROLL_COMPONENT 0.25
#define MAX_YAW_COMPONENT 0.25
#define DEG_TO_RAD 180/3.14
#define DEGREE_TO_RAD 180/3.14


//// Spektrum DSM2 RC Radio
#define RC_CHANNELS 9
#define CONFIG_DIRECTORY "/root/robot_config/"
#define DSM2_CAL_FILE	"dsm2.cal"
#define UART4_PATH "/dev/ttyO4"

/*************************DSM2 Stuff ***************/
//// DSM2 Spektrum RC radio functions
int initialize_dsm2MS();
float get_dsm2_ch_normalizedMS(int channel);
int get_dsm2_ch_rawMS(int channel);
int is_new_dsm2_dataMS();
void* uart4_checkerMS(void *ptr); //background thread


typedef struct control_variables_t{
	float	dmp_offsetX, dmp_offsetY, dmp_offsetZ;	// Offset for Euler Angles due to imperfect mounting the IMU
	float	pitch, roll, yaw[2], yaw_tmp[2];		// Euler angles of aircraft
	float	d_pitch, d_roll, d_yaw; 			// First derivative of Euler Angles	
	int		mag0, mag1, mag2;					// Magnetometer Values
	float	dpitch_setpoint, droll_setpoint;	// Desired attitude
	float	d_pitch_f, d_roll_f, d_yaw_f; 		// Filtered First derivative of Eulter Angles
	int		num_wraps, num_wraps_h;				// Number of spins in Yaw
	float	unwrapped_yaw[2];					// Some Yaw Varibles
	float	initial_yaw, unwrapped_heading;
	float	heading[2];
	float 	throttle, throttle1;				// Throttles. I don't have a good reason for why there are two.
	float	droll_err_integrator;
	float	dpitch_err_integrator;
	float	dyaw_err_integrator;
	float 	uyaw, upitch, uroll;				// Controller effort for each state variable
	float	u[4]; 								// Duty Cycle to send to each motor
	float	time; 								// Time since execution of the program
	
	float 	alt_rate_ref, d_alt_filt, alt_ref;	//Height Variables for control with Lidar
	float	height_damping;
	
	double	initial_pos_lon, initial_pos_lat; 	// Lat & Long positions from GPS
	double	pos_lon, pos_lat;
	double	lat_error, lon_error;
 
	float	standing_throttle, alt_error;
}control_variables_t;

typedef struct setpoint_t{
	float	pitch_ref, roll_ref, yaw_ref[2];	// Reference (Desired) Position
	float	filt_pitch_ref, filt_roll_ref;		// LPF of pitch and roll (because they are a func of yaw)
	float	yaw_rate_ref[2];
	float	Aux;
	double	lat_setpoint, lon_setpoint;			// Controller Variables for Autonomous Flight
}setpoint_t;

typedef struct accel_data_t{
	vector_t  X_state_Lat, X_state_Lon;
	float accel_Lat, accel_Lon,accelz;// Accelerometer Values for Kalman use
	float	pitch, roll, yaw[2];
}accel_data_t;



typedef struct GPS_data_t{
	float deg_longitude;
	float deg_latitude;
	float gps_altitude;
	float meters_lat;
	float meters_lon;
	float speed;
	float direction;
	double min_longitude;
	double min_latitude;
	float HDOP;
	int GPS_fix;
	char GGAbuf[225];
	char VTGbuf[225];
}GPS_data_t;





typedef struct function_control_t{
	int 	Lidar_kill_counter;					//Kill the lidar thread if returns negative 20 times consecutively
	int 	dsm2_timeout;						//Shutdown the system if it lost communication with the RC
	int 	alt_pos_mode;						//Signal the first iteration of Altitude position mode
	int		gps_pos_mode;						//Signal the first iteration of GPS position mode
	int		yaw_err_timout;						//Reset Yaw Error if landed for more than 1 second
	int		integrator_reset;
	int		integrator_start;
	}function_control_t;