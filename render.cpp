#include <Bela.h>
#include <cmath>
#include <Scope.h>
#include <helpers.hpp>
#include <WriteFile.h>
#include <pairs.h>
#include <time.h>
#include <drums.h>
#include <RampGenerator.h>

// DEFINITIONS AND EXTERNALS 
// ---------------------------------
#define BUFFER_SIZE 22050 // Delay Buffer Size (0.5 ms)
#define PRINT_INTERVAL 44100
uint64_t lastPrintFrames;
const int NUMBER_OF_VOICES = 24;
// const int NUMBER_OF_VOICES_PER_BOX = 8;

extern float *gSampleBuffers[NUMBER_OF_SAMPLES];
extern int gSampleBufferLengths[NUMBER_OF_SAMPLES];

float gInverseSampleRate;
int gAudioFramesPerAnalogFrame;
int gSamplesToWaitAtStartup = 10000; // This is the number of samples we wait when the system starts.
uint64_t gSampleCount = 0; // Number of frames since the audio began

// OBJECT DECLARATIONS 
// -----------------------------------
Scope scope;


// HELPER FUNCTIONS
// -----------------------------------

void calculateCoeffs();
float readPiezo(BelaContext *context, unsigned int, unsigned int);
double readZ(BelaContext *context, unsigned int, unsigned int);
int startPlayingSample(int sampleIndex, float sampleAmplitude, int latency, int boxNum, int z_state);

// FLAGS
// -----------------------------------
extern int gSamplesLoaded;  // wait for the wav files to be loaded before accessing
int gMuteIsOn = 1;


// BUTTONS, LIGHTS, POTS
// -----------------------------------
int gMuteButton = P8_07;
int gMuteLed = P8_08; 

int gPrevMuteButtonStatus = 0; // previous state was LOW, opposite of starting position (which is high because of pullup resistor)
int gMuteDebounceCounter = 0;


int gStatusLed = P8_09;
float gThreshPiezoTrigger = 0.6;// The threshold for any sound to trigger, POT!
float gOutputScaler = 0;  // POT variable for scaling the audio output.

// NO SHUT DOWN BUTTON VARIABLE - this is done through the pin being pulled to ground through the attached button.


// VARIABLES FOR FILTERS
// -----------------------------------

// LPF for acceleromter:
double Z_X[3][2];
double Z_LP_Y[3][2];
double Z_DC_Y[3][2];


// DC blocker for Z and Piezo:
double lastZDC_X[3];
double lastZDC_Y[3];
float lastPiezoDC_X[3] = {1, 1, 1};
float lastPiezoDC_Y[3] = {1, 1, 1};

// READ + WRITE POINTERS
// -----------------------------------

// Read pointers:
int gReadPointers[NUMBER_OF_VOICES] = { 0 }; // All available samples to play
int gReadPointerBufferID[NUMBER_OF_VOICES] = { 0 }; // The sample EACH is playing.
// Write pointers:
int gWritePointer = 0; // for 2-item filter arrays that update every time (for n-1 and n-2)

// PIEZO STATE MACHINE
// -----------------------------------

int gPiezoState[3] = { 0, 0, 0 };
float gFinalPiezoAmplitude[3] = { 0 }; // Stores highest value found over time
int gLookingForPiezoPeak_counter[3] = { 0 }; // Counts the 5ms needed to find the peak.
int gPiezoDebounce_counter[3] = { 0 }; // Counts the time that we stop the piezos so we don't re-trigger.
float gHighestPiezoValueFound[3] = { 0 }; // The highest value we found over time.

float gHoldAmplitudes[3] = { 0 };

// Z STATE MACHINE 
// -----------------------------------
int gZ_state[3] = { 0 }; // 0 at rest, 1 moving, 2 sudden stop (3 stopping?)

double gZ_peakValue[3]; // This is the value that finds the peak and stays there. It needs to start looking for a point when the Z reading is zero.
double gZ_peakFollower[3]; // This value varies up and down with the Z input.
int gZ_sampleCounter[3] = { 0 }; // this ticks up over and over. It resets to 0 if a highest point is found.
int gZ_sampleCounterTimeout = 8820; // If there's no Z reading for this many samples, we assume we're stopped.
// double zReading_processed[3] = { 0 }; // input from accelerometer that has LPF and DC blocking applied.

int gZ_peaks[3][2];
int gZ_bounceRate[3];
int gZ_peakCounter[3];

// VOICE STATE
// -----------------------------------

int gVoiceStates[NUMBER_OF_VOICES] = { 0 };
int gVoiceBoxAssignment[NUMBER_OF_VOICES];


// PLAYBACK VARIABLES
// -----------------------------------
// These control the audio playback, and the voice stealing.
// Buffer pointers:
int gVoiceAge[NUMBER_OF_VOICES] = { 0 }; // Buffer to store the sample count at which the voice was allocated for voice stealing.
// int gPointerState[NUMBER_OF_VOICES] = { 0 }; // The flag that shows if EACH voice is active or silent.
int gBoxStopTimer[3];
int gBoxStopMax = 44100; // Stop over 250ms.

// -----------------------------------
// START IT UP
// -----------------------------------

bool setup(BelaContext *context, void *userData)
{
    // Start the scope.
    scope.setup(4, context->audioSampleRate);

    // Scrub all buffers    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            Z_X[i][j] = 0.0;
            Z_LP_Y[i][j] = 0.0;
            Z_DC_Y[i][j] = 0.0;
            gZ_peaks[i][j] = 0;
        }
        gBoxStopTimer[i] = 0;
        lastZDC_X[i] = 0.0;
        lastZDC_Y[i] = 0.0;
        lastPiezoDC_Y[i] = 0.0;
    	lastPiezoDC_X[i] = 0.0;
        gZ_peakValue[i] = 0.0; // This is the value that finds the peak and stays there. It needs to start looking for a point when the Z reading is zero.
        gZ_peakFollower[i] = 0.0;
        gZ_bounceRate[i] = 0;

    }
    for (int j = 0; j < NUMBER_OF_VOICES; j++) {
        gVoiceBoxAssignment[j] = -1;
    }

    // Calculate filter coefficients (defined in helpers.hpp):
    calculateCoeffs();
	

	gAudioFramesPerAnalogFrame = context->audioFrames / context->analogFrames;
    
    // set up LED and button pins:
    pinMode(context, 0, gMuteButton, INPUT);
    pinMode(context, 0, gMuteLed, OUTPUT);

    pinMode(context, 0, gStatusLed, OUTPUT);
    digitalWrite(context, 0, P8_09, HIGH); // Turn the status light on when setup has executed, showing it's online and ready.

    rt_printf("Number of samples: %d\n", NUMBER_OF_SAMPLES);
    rt_printf("System is online and ready to go.\n");
    
	return true;
}



void render(BelaContext *context, void *userData)
{
    // GET CRASH THRESHOLD FOR PIEZOS AND AUDIO SCALER OUTSIDE OF BLOCK:
    float crashThresholdReading = 0.0f;
    float outputScalerReading = 0.0f;
    
    crashThresholdReading = analogRead(context, 0, 6);
    outputScalerReading = analogRead(context, 0, 7);
    gOutputScaler = map(outputScalerReading, 0.001, 0.825, 0.001, 1.25);
    gThreshPiezoTrigger = map(crashThresholdReading, 0.001, 0.825, 0.001, 1.25);
    
    // BUTTON HANDLING
    
    /*
    Read the button. 
    It is always high. If it goes low and stays there for 11025 samples (1/4 second), toggle the mute.
    Set the LED to whatever the state is.
    */
    gMuteDebounceCounter++;
    if (gMuteDebounceCounter >= 300) {
        int muteStatus = digitalRead(context, 0, gMuteButton);
        
        if (muteStatus == 0 && gPrevMuteButtonStatus != 0) {
            gMuteIsOn = !gMuteIsOn;
            gMuteDebounceCounter = 0;
            rt_printf("MUTE STATUS: %d\n", gMuteIsOn);
        }
        
        gPrevMuteButtonStatus = muteStatus;
    }
    
    // Turn on the LED according to the mute state: 
    digitalWrite(context, 0, P8_08, gMuteIsOn);
    
    // AUDIO LOOP:
    for (unsigned int n = 0; n < context->audioFrames; n++) {
        // Make vars for piezo, z, and out.
        float piezoReading_processed[3] = { 0 };
        double zReading_processed[3] = { 0 }; // input from accelerometer that has LPF and DC blocking applied.
        float out = 0.0; // audio out.

        // Go through each box, and get the Piezo and Z states for each.
        for (int i = 0; i < 3; i++) {
            zReading_processed[i] = readZ(context, n, i); // Z in
            piezoReading_processed[i] = readPiezo(context, n, i); // piezo in

            
            // Z STATES
            if (gZ_state[i] == 0) { // Z STATE 0: Resting, waiting to move
                if (zReading_processed[i] > 0.02) {
                    gZ_state[i] = 1;
                    gZ_peakValue[i] = 0.0; // reset peak value for next state
                    rt_printf("Z state on Box %d went from rest to moving!\n", i);
                    gZ_sampleCounter[i] = 0;
                }
            } else if (gZ_state[i] == 1) { // Z STATE 1: Moving, waiting to STOP or REST
                if (zReading_processed[i] > gZ_peakFollower[i]) {
                    gZ_sampleCounter[i] = 0;
                    gZ_peakFollower[i] = zReading_processed[i];
                    gZ_peakValue[i] = zReading_processed[i];
                } else {
                    gZ_peakFollower[i] *= 0.9998;
                    gZ_sampleCounter[i]++;
                }
                // Look for a stop:
                if (gZ_sampleCounter[i] > gZ_sampleCounterTimeout
                    && gZ_peakValue[i] > 0.01 
                    && zReading_processed[i] < 0.01) {
                        gZ_state[i] = 2;
                    }
            } else if (gZ_state[i] == 2) {
                rt_printf("Sudden stop on Z! Return to 0\n");
                gZ_state[i] = 0;
                // Tell the voices on that box that we're now stopping:
                for (int q = 0; q < NUMBER_OF_VOICES; q++) {
                    if (gVoiceBoxAssignment[q] == i) {
                        gVoiceStates[q] = 2;
                    }
                }
            } // END Z STATES
            
            
            // PIEZO STATES
            // STATE 0: WAITING FOR TRIGGER    
            if (gPiezoState[i] == 0) {
                if(piezoReading_processed[i] > 0.09) { // 0.09 is the trigger threshold so we're not triggered by noise alone
                // rt_printf("PIEZO TRIGGERED: Entering State 1\n");
                    gPiezoState[i] = 1;
                    gLookingForPiezoPeak_counter[i] = 0;
                    gHighestPiezoValueFound[i] = 0;
                    gZ_sampleCounter[i] = 0; // Reset Z counter to 0 to prevent wiggling under the last peak value causing the timer to expire.
                }
            } // End state 0
            
            // STATE 1: TRIGGERED, GATHERING HIGHEST VALUE.
            else if (gPiezoState[i] == 1) {
                gZ_sampleCounter[i] = 0; // reset Z sample to keep Z state in 1, or "moving"
                if (gLookingForPiezoPeak_counter[i] < 440) { // wait for 6ms and collect the biggest value.
                    gLookingForPiezoPeak_counter[i]++;
                    if (piezoReading_processed[i] > gHighestPiezoValueFound[i]) {
	  	                gHighestPiezoValueFound[i] = piezoReading_processed[i];
                    }
                } else { // The piezo peak counter has elapsed, ready to return the value
                    // rt_printf("Done looking for a peak on piezo %d! Highest was: %f\n", i, gHighestPiezoValueFound[i]);
                    gFinalPiezoAmplitude[i] = gHighestPiezoValueFound[i]; // Store the highest value found
                    gPiezoState[i] = 2;
                    gPiezoDebounce_counter[i] = 0;
                    
                }
            } // end state 1
            else if (gPiezoState[i] == 2) {
                
                // start buffers playing
                // // Voice allocation
                
                //gHoldAmplitudes[i] = gSampleAmps[i];
                if (gSamplesLoaded) { // Check to make sure the samples are loaded!
                    if (startPlayingSample(i, gFinalPiezoAmplitude[i], -100, i, gZ_state[i]) == 0) { // If no voice was free, there's one free now ...
                        startPlayingSample(i, gFinalPiezoAmplitude[i], -100, i, gZ_state[i]); // ... so start another sample playing.
                        }
                    }
                gPiezoState[i] = 3;
            }  // end piezo state 2
            else if (gPiezoState[i] == 3) {
                if (gPiezoDebounce_counter[i] < 3500) {
                    gPiezoDebounce_counter[i]++;
                } else {
                    gPiezoDebounce_counter[i] = 0;
                    gPiezoState[i] = 0;
                    // rt_printf("Box %d Back to 0!\n", i);
                }
            } // END PIEZO STATES
            
        } // end i loop - going through each box and getting the Z and piezo info.
        
        float outputSample;
    	
    	for(int j = 0; j < NUMBER_OF_VOICES; j++) {
    	    if (gVoiceStates[j] == 0) { // if we're waiting ...
    	       gReadPointers[j] = 0; // make sure the read pointer is 0.
    	    } // end voice state 0
    	    
    	    else if (gVoiceStates[j] == 1) { // if we're playing ...
    	        outputSample = gSampleBuffers[gReadPointerBufferID[j]][gReadPointers[j]] * gHoldAmplitudes[j];
    	        out += outputSample;

                // Check if we're out of buffer.
                if (gReadPointers[j] < gSampleBufferLengths[gReadPointerBufferID[j]]) {
    			    gReadPointers[j]++; // if not, advance the read pointer.
                } else { // if we're out of buffer ...
                    gVoiceStates[j] = 0; // go to state 0
    				gVoiceBoxAssignment[j] = -1; // unassign from a box.
    				gReadPointers[j] = 0;
                }
            }  // end voice state 1
            else if (gVoiceStates[j] == 2) { // If we're on a sudden stop ...
                // If we're not at the end of the timer ...
                if(gVoiceBoxAssignment[j] < 0 || gVoiceBoxAssignment[j] >= 3) {
                    rt_printf("oops! for voice %d in state 2, box assignment was %d\n", gVoiceBoxAssignment[j]);
                }
                else {
                    if (gBoxStopTimer[gVoiceBoxAssignment[j]] < gBoxStopMax) {
                        gHoldAmplitudes[j] *= 0.99;
                        outputSample = gSampleBuffers[gReadPointerBufferID[j]][gReadPointers[j]] * gHoldAmplitudes[j];
        	            out += outputSample; // add the current sample to the output
                    
                        // Then advance the read if we still have buffer.
                        if (gReadPointers[j] < gSampleBufferLengths[gReadPointerBufferID[j]]) {
            			    gReadPointers[j]++; // if we do, advance the read pointer.
                        } else { // if we're out of buffer ...
                            gVoiceStates[j] = 0; // go to state 0
            				gVoiceBoxAssignment[j] = -1; // unassign from a box.
            				gReadPointers[j] = 0;
                        }
                    }
                    else { // if we're at the end of the timer, go back to 0, reset pointer, unallocate.
                        gVoiceStates[j] = 0; // go to state 0
            		    gVoiceBoxAssignment[j] = -1; // unassign from a box.
            			gReadPointers[j] = 0;
                    }
                }
    	   } // end voice states
    	}// end voice for loop
    	        
    		// Handle output
    		if (!gMuteIsOn) { // if mute isn't on ...
                out *= gOutputScaler; // Scale the amplitude down over time.
            } else { // if mute is on ...
                out *= 0; // zero everything.
            }
            
    		for(unsigned int channel = 0; channel < context->audioOutChannels; channel++){
    			audioWrite(context, n, channel, out);
    		}
    		
    		gSampleCount = context->audioFramesElapsed;
    		
    	  	scope.log(zReading_processed[0], zReading_processed[1], zReading_processed[2]);
            
            // ADVANCE THE WRITE POINTERS:
    		gWritePointer++;
    		if (gWritePointer >= 2) {
    			gWritePointer = 0;
    		}

    } // END AUDIO FOR LOOP
} // END OF RENDER();



float readPiezo(BelaContext *context, unsigned int frame, unsigned int boxNum) {

    float piezoInput = analogRead(context, frame/gAudioFramesPerAnalogFrame, boxNum);    
    float piezoOut = (piezoInput - lastPiezoDC_X[boxNum] + 0.95 * lastPiezoDC_Y[boxNum]);

    // DC blocking:
    lastPiezoDC_Y[boxNum] = piezoOut;
    lastPiezoDC_X[boxNum] = piezoInput;
    
    // Rectify:
    if (piezoOut < 0.0) {
        piezoOut *= -1.0;
    }
    
    return piezoOut;
}

double readZ(BelaContext *context, unsigned int frame, unsigned int boxNum) {
    int pinNum = boxNum + 3;
    double zIn = analogRead(context, frame/gAudioFramesPerAnalogFrame, pinNum);
        
    // LPF: y[n] = (B0 * x[n] + B1 * x[n-1] + B2 x[n-2] - A1 * x[n-1] - A2 * x[n-2])
    double z_LP = (gLowB0 * zIn) + (gLowB1 * Z_X[boxNum][(gWritePointer + 2 - 1) % 2]) + (gLowB2 * Z_X[boxNum][gWritePointer]) - (gLowA1 * Z_LP_Y[boxNum][(gWritePointer + 2 - 1) % 2]) - gLowA2 * Z_LP_Y[boxNum][gWritePointer];
    
    // Print statement for debugging:
    // if(context->audioFramesElapsed - lastPrintFrames > PRINT_INTERVAL) {
    //     lastPrintFrames = context->audioFramesElapsed;
    // }
    
    Z_X[boxNum][gWritePointer] = zIn;
    Z_LP_Y[boxNum][gWritePointer] = z_LP;

    // DC blocking:
    double z_dc = (z_LP - lastZDC_X[boxNum] + 0.9998 * lastZDC_Y[boxNum]);
    lastZDC_X[boxNum] = z_LP;
    lastZDC_Y[boxNum] = z_dc;
    
    // Rectification:
    if (z_dc < 0) {
        z_dc *= -1;
    }
    
    return z_dc;
}

int startPlayingSample(int sampleIndex, float sampleAmplitude, int latency, int boxNum, int z_state) {
    // rt_printf("Incoming amplitude: %f\n", sampleAmplitude);
	// Assigns the next free voice to play the drum sample
    
	// Go through each voice. 
	for(int j=0; j < NUMBER_OF_VOICES; j++){
		if(gVoiceStates[j] == 0) { // If the voice's state is 0 (unassigned):
		    gVoiceBoxAssignment[j] = boxNum; // Store the box that the voice was triggered by
			gReadPointers[j] = 0;
			if (sampleAmplitude > gThreshPiezoTrigger // If the box hit was hard enough ... 
			    && z_state == 1) { // and the box was moving ...
			    sampleIndex += 3; // play the sample with the high partials
			 //   rt_printf("Hard enough hit! Playing sample %d\n", sampleIndex);
			}
		    else if (z_state == 0) { // If the box was at rest ...
		        sampleIndex += 6; // play the sample with all the bass
		        rt_printf("Box was hit while stopped! Play that crazy bass but no partials.\n");
		    }

    		gReadPointerBufferID[j] = sampleIndex; // keep track of which sample the pointer is playing
    		gVoiceStates[j] = 1; // mark voice as active
    		gVoiceAge[j] = gSampleCount; // Keep track of how old the pointer is
            
            gHoldAmplitudes[j] = sampleAmplitude; // Set the amplitude according to incoming level.
    
     		rt_printf("Box: %d Voice: %d Sample: %d Amplitude: %f Age: %d\n", boxNum, j, sampleIndex, gHoldAmplitudes[j], gVoiceAge[j]);
            // storeVoiceForEachBox(boxNum, j);
    		return 1;
		}
	}
		
		
		
	

	// IF IT DOESN'T FIND A VOICE WITH THE STATUS 0, then it finds the oldest pointer and changes its state to 0, and then when we run this again it's 
	// free to be reassigned.
	
	
	int oldest = gVoiceAge[0];
	int oldestPointer = 0;
	for(int i=0;i<NUMBER_OF_VOICES;i++){
	    if(gVoiceAge[i] < oldest){
	        oldest = gVoiceAge[i];
	        oldestPointer = i;
	    }
	    
	}
	gVoiceStates[oldestPointer] = 0;
	rt_printf("** releasing voice %d\n", oldestPointer);
	
	return 0;
}



void cleanup(BelaContext *context, void *userData)
{
}