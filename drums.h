/*** drums.h ***/
/*
 * assignment2_drums
 *
 * Second assignment for ECS732 RTDSP, to create a sequencer-based
 * drum machine which plays sampled drum sounds in loops.
 *
 * This code runs on BeagleBone Black with the BeagleRT environment.
 *
 * 2015
 * Andrew McPherson and Victor Zappi
 * Queen Mary University of London
 */

#define NUMBER_OF_SAMPLES 9
#ifndef _DRUMS_H
#define _DRUMS_H


#define NUMBER_OF_PATTERNS 9
#define FILL_PATTERN 5

/* Start playing a particular drum sound */
int startPlayingSample(int sampleIndex, float sampleAmplitude, int latency);
// int startPlayingDrum(int drumIndex, float drumAmplitude, int drumLatency);

/* Start playing the next event in the pattern */
void startNextEvent();

/* Returns whether the given event contains the given drum sound */
int eventContainsDrum(int event, int drum);

#endif /* _DRUMS_H */
