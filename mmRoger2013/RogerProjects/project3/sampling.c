/*************************************************************************/
/* File:        sampling.c                                               */
/* Description: Methods for sampling from distributions                  */
/* Date:        10-30-2012                                               */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

int init_random_seed = TRUE;


//specify how many samples it should take before giving up
#define NUM_SAMPLES 5

int sample_count = 0;


int sample_heading(heading)
double *heading;
{
	//only initialize random number generator with seed once
	if (init_random_seed == TRUE)
	{
		//printf("------------------------------\n");
		srand(time(NULL));
		init_random_seed = FALSE;
	}

	//uniformly sample from -PI to PI
	*heading = (double) rand() / (double) RAND_MAX * M_PI * 2.0 - M_PI;
	
	if (sample_count++ > NUM_SAMPLES) 
	{
		sample_count = 0;
		return FALSE;
	}

	return TRUE;
	
}

//reset the sample count to 0
reset_sampling_count()
{
	sample_count = 0;
}

//randomly place ball inside map
place_object_random()
{
	double rand_pos[2];
	double range[2];

	//only initialize random number generator with seed once
	if (init_random_seed == TRUE)
	{
		//printf("------------------------------\n");
		srand(time(NULL));
		init_random_seed = FALSE;
	}

	//assume center at 0.0 and stay radius of ball distance away from walls
	//           range       -   radius      -   wall thickness
	range[X] = MAX_X - MIN_X - (2.0 * R_OBJ) - 2 * XDELTA;
	range[Y] = MAX_Y - MIN_Y - (2.0 * R_OBJ) - 2 * YDELTA;

	//generate random x, y
	rand_pos[X] = ((double) rand() / (double) RAND_MAX) * range[X] - range[X]/2.0;	
	rand_pos[Y] = ((double) rand() / (double) RAND_MAX) * range[Y] - range[Y]/2.0;	
	
	printf("Placing ball at random location: %4.3f, %4.3f \n", 	rand_pos[X], rand_pos[Y]);

	//check if too close to robot?

	//insert object
	place_object(rand_pos[X], rand_pos[Y]);
		
}


