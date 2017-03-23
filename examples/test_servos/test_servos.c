/*******************************************************************************
* test_servos.c
*
* James Strawson 2016
* Demonstrates use of pru to control servos and ESCs with pulses.
* This program operates in 4 different modes. See the option list below
* for how to select an operational mode from the command line.
*
* SERVO: uses send_servo_pulse_normalized() to set one or all servo positions
* to a value from -1.5 to 1.5 corresponding to their extended range. 
* -1 to 1 is considered the "safe" normal range as some servos will not go 
* beyond this. Test your servos incrementally to find their safe range.
*
* ESC: For unidirectional brushless motor speed controllers specify a range from
* 0 to 1 as opposed to the bidirectional servo range. Be sure to run the
* calibrate_esc example first to make sure the ESCs are calibrated to the right
* pulse range. This mode uses the send_esc_pulse_normalized() function.
*
* MICROSECONDS: You can also specify your own pulse width in microseconds (us).
* This uses the send_servo_pulse_us() function.
*
* SWEEP: This is intended to gently sweep a servo back and forth about the
* center position. Specify a range limit as a command line argument as described
* below. This also uses the send_servo_pulse_normalized() function.
* 
* 
* SERVO POWER RAIL: The robotics cape has a software-controlled 6V power
* regulator allowing controlled steady power to drive servos. This can be
* enabled at the command line with the -v option. It will not allow you to
* enable the power rail when using the ESC mode as sending 6V into an ESC
* may damage it. It is best to physically cut the center wire on ESC connections
* as the BEC function is not needed when using the Robotics Cape.
*******************************************************************************/

#include <robotics_cape.h>
#include <useful_includes.h>

typedef enum test_mode_t{
	DISABLED,
	SERVO,
	ESC,
	MICROSECONDS,
	SWEEP
}test_mode_t;

// printed if some invalid argument was given
void print_usage(){
	printf("\n");
	printf(" Options\n");
	printf(" -c {channel}   Specify one channel from 1-8.\n");
	printf("                Otherwise all channels will be driven equally\n");
	printf(" -f {hz}        Specify pulse frequency, otherwise 50hz is used\n");
	printf(" -v             Enable 6V servo power rail\n");
	printf("                DO NOT use power option with ESCs\n");
	printf(" -p {position}  Drive servos to a position between -1.5 & 1.5\n");
	printf(" -e {throttle}  Drive ESCs at normalized throttle from 0-1\n");
	printf(" -u {width_us}  Send pulse width in microseconds (us)\n");
	printf(" -s {limit}     Sweep servo back/forth between +- limit\n");
	printf("                Limit can be between 0 & 1.5\n");
	printf(" -h             Print this help messege \n\n");
	printf("sample use to center servo channel 1:\n");
	printf("   test_servos -v -c 1 -p 0.0\n\n");
}

int main(int argc, char *argv[]){
	float servo_pos, sweep_limit, esc_throttle;
	float direction = 1; // switches between 1 & -1 in sweep mode
	int ch, c, width_us;
	int all = 1;	// set to 0 if a channel -c  argument is given 
	test_mode_t mode = DISABLED; //start mode disabled
	int power_en = 0; // change to 1 if user wishes to enable power rail
	int frequency_hz = 50; // default 50hz frequency to send pulses
	
	// parse arguments
	opterr = 0;
	while ((c = getopt(argc, argv, "c:f:vp:e:u:s:h")) != -1){
		switch (c){
		case 'c': // servo/esc channel option
			ch = atoi(optarg);
			if(ch<=8 && ch>=1){
				all = 0;
			}
			else{
				printf("channel option must be from 1-4\n");
				return -1;
			}
			break;
		
		case 'f': // pulse frequency option
			frequency_hz = atoi(optarg);
			if(frequency_hz<1){
				printf("Frequency option must be >=1\n");
				return -1;
			}
			break;
		
		case 'v':
			power_en=1; //enable power rail later
			break;
			
		case 'p': // servo position option
			if(mode!=DISABLED) print_usage();
			servo_pos = atof(optarg);
			if(servo_pos<=1.5 && servo_pos >=-1.5){
				mode = SERVO;
			}
			else{
				printf("Servo position must be from -1.5 to 1.5\n");
				return -1;
			}
			break;
			
		case 'e': // esc throttle option
			if(mode!=DISABLED) print_usage();
			esc_throttle = atof(optarg);
			if(esc_throttle<=1 && esc_throttle>=0){
				mode = ESC;
			}
			else{
				printf("ESC throttle must be from 0 to 1\n");
				return -1;
			}
			break;
			
		case 'u': // width in microsecons option
			if(mode!=DISABLED) print_usage();
			width_us = atof(optarg);
			if(width_us >= 10){
				mode = MICROSECONDS;
			}
			else{
				printf("Width in microseconds must be >10\n");
				return -1;
			}
			break;
			
		case 's':
			if(mode!=DISABLED) print_usage();
			sweep_limit = atof(optarg);
			if(sweep_limit<=1.5 && sweep_limit>=-1.5){
				mode = SWEEP;
				servo_pos = 0;
			}
			else{
				printf("Sweep limit must be from -1.5 to 1.5\n");
				return -1;
			}
			break;
			
		case 'h':  // help mode
			print_usage();
			return 0;
			break;
			
		default:
			printf("\nInvalid Argument \n");
			print_usage();
			return -1;
			break;
		}
    }
	
	// if the user didn't give enough arguments, exit
	if(mode==DISABLED){
		printf("\nNot enough input arguments\n");
		printf("Run 'test_servos -h' to display help messege.\n\n");
		return -1;
	}
	
	// check user isn't trying to use power with ESCs
	if(mode==ESC && power_en==1){
		printf("can't use servo power rail when connected to ESCs\n");
		return -1;
	}
	
	// okay, off we go!
	if(initialize_cape()){
		printf("ERROR: failed to initialize_cape\n");
		return -1;
	}
	// turn on power if option was given
	if(power_en){
		printf("Turning On 6V Servo Power Rail\n");
		enable_servo_power_rail();
	}
	
	// if driving an ESC, send throttle of 0 first
	// otherwise it will go into calibration mode
	if(mode==ESC){
		if(all) send_esc_pulse_normalized_all(0);
		else send_esc_pulse_normalized(ch,0);
		usleep(50/1000000);
	}
	
	
	// print out what the program is doing
	printf("\n");
	if(all) printf("Sending on all channels.\n");
	else	printf("Sending only to channel %d.\n", ch);
	switch(mode){	
	case SERVO:
		printf("Using send_servo_pulse_normalized\n");
		printf("Normalized Signal: %f  Pulse Frequency: %d\n", \
												servo_pos, frequency_hz);
		break;
	case ESC:
		printf("Using send_esc_pulse_normalized\n");
		printf("Normalized Signal: %f  Pulse Frequency: %d\n", \
												esc_throttle, frequency_hz);
		break;
	case MICROSECONDS:
		printf("Using send_servo_pulse_microseconds\n");
		printf("Pulse_width: %d  Pulse Frequency: %d\n", \
												width_us, frequency_hz);
		break;
	case SWEEP:
		printf("Sweeping servos back/forth between +-%f\n", sweep_limit);
		printf("Pulse Frequency: %d\n", frequency_hz);
		break;
		
	default:
		set_state(EXITING); //should never actually get here
		break;
	}
	
	
	// Main loop runs at frequency_hz
	while(get_state()!=EXITING){
		switch(mode){
			
		case SERVO:
			if(all) send_servo_pulse_normalized_all(servo_pos);
			else send_servo_pulse_normalized(ch, servo_pos);
			break;
			
		case ESC:
			if(all) send_esc_pulse_normalized_all(esc_throttle);
			else send_esc_pulse_normalized(ch, esc_throttle);
			break;
			
		case MICROSECONDS:
			if(all) send_servo_pulse_us_all(width_us);
			else send_servo_pulse_us(ch, width_us);
			break;
			
		case SWEEP:
			// increase or decrease position each loop
			// scale with frequency
			servo_pos += direction * sweep_limit / frequency_hz;
			
			// reset pulse width at end of sweep
			if(servo_pos>sweep_limit){
				servo_pos = sweep_limit;
				direction = -1;
			}
			else if(servo_pos < (-sweep_limit)){
				servo_pos = -sweep_limit;
				direction = 1;
			}
			// send result
			if(all) send_servo_pulse_normalized_all(servo_pos);
			else send_servo_pulse_normalized(ch, servo_pos);
			break;
			
		default:
			set_state(EXITING); //should never actually get here
			break;
		}
		
		// sleep roughly enough to maintain frequency_hz
		usleep(1000000/frequency_hz);
	}
	
	cleanup_cape();
    return 0;
}
	
