#include <stdio.h>
#include <stdlib.h>
#include "config.h"
#include <string.h>

/************************************************************************
* 	print_core_config()
*	print configuration table to console
************************************************************************/
int print_core_config(core_config_t* config){	
	#define X(type, fmt, name, default) printf("%s " fmt "\n", #name, config->name);
	CORE_CONFIG_TABLE
	#undef X
	
	return 0;
}

/************************************************************************
* 	create_empty_core_config_file()
*	creates a new file from the robotics cape library definition for 
*	config directory and this file's definition for config file name
************************************************************************/
FILE* create_empty_core_config_file(char name[]){
	char core_config_path[100];
	FILE* f;
	
	// construct a new file path string
	strcpy (core_config_path, CONFIG_DIRECTORY);
	strcat (core_config_path, CORE_CONFIG_FILE);
	
	// open
	f = fopen(core_config_path, "w");
	if (f==NULL){
		printf("could not open config directory\n");
		printf(CONFIG_DIRECTORY);
		printf("\n");
		return NULL;
	}
	
	fprintf(f, CORE_FILE_HEADER);
	fprintf(f, "%s\n", name);
	return f;
}

/************************************************************************
* 	save_core_config()
*	write the config struct to disk
* 	should be called after writing the following header line to the file
* 	fprintf(f, "flight_core_config_name: your_name");
************************************************************************/
int save_core_config(FILE* f, core_config_t* config){
	// if the file isn't already open, open it
	if(f == NULL){
		printf("open core_config and write header first\n");
		return -1;
	}
	
	// write 
	#define X(type, fmt, name, default) fprintf(f, "%s," fmt "\n", #name, config->name);
    CORE_CONFIG_TABLE
	#undef X	
	
	fclose(f);
	return 0;
}

/************************************************************************
* 	load_core_config()
*	read from the disk. returns -1 if unsuccessful 
************************************************************************/
int load_core_config(core_config_t* config){

	char core_config_path[100];
	FILE* f;
	
	// construct a new file path string
	strcpy (core_config_path, CONFIG_DIRECTORY);
	strcat (core_config_path, CORE_CONFIG_FILE);
	
	// open
	f = fopen(core_config_path, "r");
	if (f==NULL){
		printf("could not open core_config file\n");
		printf(core_config_path);
		printf("\n");
		return -1;
	}
	// read from beginning
	rewind(f);
	
	char name[100];
	fscanf(f, CORE_FILE_HEADER);
	fscanf(f, "%s\n", name);
	printf("using flight_core_config: ");
	printf(name);
	printf("\n");
	
	#define X(type, fmt, name, default) fscanf(f, #name "," fmt"\n", &config->name);
    CORE_CONFIG_TABLE
	#undef X
	
	fclose(f);
	return 0;
}


/************************************************************************
* 	create_default_core_config_file()
*	creates a new file and returns a struct containing the default values
************************************************************************/
int create_default_core_config_file(core_config_t* config){
	FILE* f;
	// construct new struct from defaults
	#define X(type, fmt, name, default) default ,
	core_config_t defaults = { CORE_CONFIG_TABLE };
	#undef X
	
	// write defaults out to user's struct
	*config=defaults;
	
	//start a new file
	f = create_empty_core_config_file("default");
	
	// if we couldn't open file for writing, return error -1
	if(f == NULL){
		return -1;
	}

	save_core_config(f, &defaults);

	printf("created new default core_config file here:\n");
	printf(CONFIG_DIRECTORY);
	printf("\n");
	
	//fclose(f);
	return 0;
}