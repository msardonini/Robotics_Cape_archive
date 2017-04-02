/*******************************************************************************
* calibrate_escs.c
*
* James Strawson 2016
* Typical brushless speed controllers (ESCs) accept a variety of pulse widths,
* usually from 900-2100 microseconds. Before using them, you must calibrate the
* ESC so it knows what pulse widths correspond to minimum and maximum throttle.
* Typically ESCs go into calibration mode by applying any pulse width greater 
* than roughly 1000us when it is powered on. Once in calibration mode, the user
* then applies max and min pulse widths briefly before turning off the signal to
* exit calibration mode. This is typically done with the throttle stick on your 
* RC transmitter.
*
* The calibrate_escs example assists you with this process by sending the right 
* pulse widths. Follow the instructions that are displayed in the console when 
* you execute the program.
*******************************************************************************/

#include <useful_includes.h>
#include <robotics_cape.h>

float width; // global variable for normalized pulse width to send

// background thread to send pulses at 50hz to ESCs
void *send_pulses(void *params){
	while(get_state()!=EXITING){
		send_esc_pulse_normalized_all(width);
		usleep(20000);
	}
	return 0;
}

int main(){
	initialize_cape();
	
	//Send full throttle until the user hits enter
	width = 0;
	pthread_t  send_pulse_thread;
	pthread_create(&send_pulse_thread, NULL, send_pulses, (void*) NULL);


	printf("\nDISCONNECT PROPELLERS FROM MOTORS\n");
	printf("DISCONNECT POWER FROM ESCS\n");
	printf("press enter to start sending max pulse width\n");
	if(continue_or_quit()<1){
		printf("aborting calibrate_escs\n");
		goto END;
	}
	
	width = 1;
	printf("\n");
	printf("Now reapply power to the ESCs.\n");
	printf("Press enter again after the ESCs finish chirping\n");
	set_led(GREEN,1);
	if(continue_or_quit()<1){
		printf("aborting calibrate_escs\n");
		goto END;
	}
	
	// now set lower bound
	printf("\n");
	printf("Sending minimum width pulses\n");
	printf("Press enter again after the ESCs chirping to finish calibration\n");
	width = 0;
	if(continue_or_quit()<1){
		printf("aborting calibrate_escs\n");
		goto END;
	}
	

	// cleanup and close
	printf("\nCalibration complete, check with test_servos\n");
END:
	set_state(EXITING); // this tells the send_pulses thread to stop
	pthread_join(send_pulse_thread, NULL); // wait for it to stop
	
	cleanup_cape();
	return 0;
}
