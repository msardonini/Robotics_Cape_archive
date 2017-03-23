#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <robotics_cape.h>

 float kill_switch[2];
 FILE *logger, *GPS_logger, *Error_logger;
int GPS_file, GPS_init_check, GPS_fix_check;

#include </root/Robotics_Cape/examples/flyMS/flight_defs.h>
#include </root/Robotics_Cape/examples/flyMS/flyMS.h>
int i2c_file, GPS_file;
FILE *logger, *GPS_logger;
float random_input[2];


int main(int argc, char *argv[]){
	GPS_data_t GPS_data; //declare structure for GPS data to do into
	GPS_init(argc, argv);//initialize the GPS
	
	memset(&GPS_data,0,sizeof(GPS_data));
	
	while(1){
		if(is_new_GPS_data()){
			GPS_data=get_GPS_data();
			if(/*GPS_data.GPS_fix*/ 1){
			printf("meters lon: %f, meters lat: %f",GPS_data.meters_lon,GPS_data.meters_lat);
			//Print the data from the structure which was just read
			//printf("lat: %2.0f°%f ", GPS_data.deg_latitude,GPS_data.min_latitude);
			//printf("lon: %2.0f°%f ", GPS_data.deg_longitude, GPS_data.min_longitude);
			//printf("speed: %2.2f, dir: %2.2f,",GPS_data.speed,GPS_data.direction);
			//printf("alt: %2.2f,",GPS_data.gps_altitude);
			//printf("HDOP: %2.2f, fix: %d",GPS_data.HDOP,GPS_data.GPS_fix);
			printf("\n");
			}
			else{
				printf("%s",GPS_data.GGAbuf);
				printf("%s",GPS_data.VTGbuf);
			}
		}
		usleep(100000);
	}
	return 0;
}