/*
 * default_main.cpp
 *
 *  Created on: Oct 24, 2014
 *      Author: parallels
 */
#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <libgen.h>
#include <signal.h>
#include <sndfile.h> // read wav
#include <getopt.h>
#include <Bela.h>

// ---Drums----
#include "drums.h"


using namespace std;

int gSensorInputFrequency = 0;
int gSensorInputAmplitude = 1;

int gSamplesLoaded = 0;


/* Drum samples are pre-loaded in these buffers. Length of each
 * buffer is given in gDrumSampleBufferLengths.
 */
float *gSampleBuffers[NUMBER_OF_SAMPLES];
int gSampleBufferLengths[NUMBER_OF_SAMPLES];

//--------------

using namespace std;

int returnCode = 0;

int initDrums() {
	/* Load drums from WAV files */
	SNDFILE *sndfile;
	SF_INFO sfinfo;
	char filename[64];
	

	for(int i = 0; i < NUMBER_OF_SAMPLES; i++) {
	    
	    //char* path = "/root/drums/drum0.wav";
	    
		snprintf(filename, 64, "sample%d.wav", i);

		if (!(sndfile = sf_open (filename, SFM_READ, &sfinfo))) {
			printf("Couldn't open file %s\n", filename);

			/* Free already loaded sounds */
			for(int j = 0; j < i; j++)
				free(gSampleBuffers[j]);
			return 1;
		}

		if (sfinfo.channels != 1) {
			printf("Error: %s is not a mono file\n", filename);

			/* Free already loaded sounds */
			for(int j = 0; j < i; j++)
				free(gSampleBuffers[j]);
			return 1;
		}

		gSampleBufferLengths[i] = sfinfo.frames;
		gSampleBuffers[i] = (float *)malloc(gSampleBufferLengths[i] * sizeof(float));
		if(gSampleBuffers[i] == NULL) {
			printf("Error: couldn't allocate buffer for %s\n", filename);

			/* Free already loaded sounds */
			for(int j = 0; j < i; j++)
				free(gSampleBuffers[j]);
			return 1;
		}

		int subformat = sfinfo.format & SF_FORMAT_SUBMASK;
		int readcount = sf_read_float(sndfile, gSampleBuffers[i], gSampleBufferLengths[i]);

		/* Pad with zeros in case we couldn't read whole file */
		for(int k = readcount; k < gSampleBufferLengths[i]; k++)
			gSampleBuffers[i][k] = 0;

		if (subformat == SF_FORMAT_FLOAT || subformat == SF_FORMAT_DOUBLE) {
			double	scale ;
			int 	m ;

			sf_command (sndfile, SFC_CALC_SIGNAL_MAX, &scale, sizeof (scale)) ;
			if (scale < 1e-10)
				scale = 1.0 ;
			else
				scale = 32700.0 / scale ;
			printf("Scale = %f\n", scale);

			for (m = 0; m < gSampleBufferLengths[i]; m++)
				gSampleBuffers[i][m] *= scale;
		}

		sf_close(sndfile);
	}

	return 0;
}

void cleanupDrums() {
	for(int i = 0; i < NUMBER_OF_SAMPLES; i++)
		free(gSampleBuffers[i]);
}

// Handle Ctrl-C by requesting that the audio rendering stop
void interrupt_handler(int var)
{
	gShouldStop = true;
	// allows other process to monitor why Bela has exited
	// var=15, returnCode = 143 if terminated with SIGTERM
	returnCode = var + 128;
}

// Print usage information
void usage(const char * processName)
{
	cerr << "Usage: " << processName << " [options]" << endl;

	Bela_usage();

	cerr << "   --help [-h]:                Print this menu\n";
}

int main(int argc, char *argv[])
{
	BelaInitSettings settings;	// Standard audio settings

	struct option customOptions[] =
	{
		{"help", 0, NULL, 'h'},
		{NULL, 0, NULL, 0}
	};

	// Set default settings
	Bela_defaultSettings(&settings);

	// Parse command-line arguments
	while (1) {
		int c;
		if ((c = Bela_getopt_long(argc, argv, "h", customOptions, &settings)) < 0)
				break;
		switch (c) {
		case 'h':
				usage(basename(argv[0]));
				exit(0);
		case '?':
		default:
				usage(basename(argv[0]));
				exit(1);
		}
	}

	// Initialise the PRU audio device
	if(Bela_initAudio(&settings, 0) != 0) {
		cout << "Error: unable to initialise audio" << endl;
		return -1;
	}

	// Start the audio device running
	if(Bela_startAudio()) {
		cout << "Error: unable to start real-time audio" << endl;
		return -1;
	}
	// Load the drum sounds and the patterns
    if(initDrums()) {
    	printf("Unable to load drum sounds. Check that you have all the WAV files!\n");
    	return -1;
    }
    printf("Drum sounds loaded correctly. Starting ...\n");
    usleep(200000);
    gSamplesLoaded = 1;
    

	// Set up interrupt handler to catch Control-C and SIGTERM
	signal(SIGINT, interrupt_handler);
	signal(SIGTERM, interrupt_handler);

	// Run until told to stop
	while(!gShouldStop) {
		usleep(100000);
	}

	// Stop the audio device
	Bela_stopAudio();

	// Clean up any resources allocated for audio
	Bela_cleanupAudio();

	// All done!
	return returnCode;
}


