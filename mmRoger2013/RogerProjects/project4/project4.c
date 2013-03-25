/*************************************************************************/
/* File:        project4.c                                               */
/* Description: User project #4                                          */
/* Date:        11-13-2012                                               */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "Xkw/Xkw.h"

#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"


/*************************************************************************/
/* project development interface - called by GUI                         */
/*************************************************************************/
// macro0() - station keeping SEARCHTRACK red 
// primitive2() - APPROACH
// primitive3() - STRIKE
project4_control(roger, time)
Robot* roger;
double time;
{}


project4_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
project4_enter_params() 
{
  printf("Project 4 enter_params called. \n");
}

/* do not alter */
//function called when the 'visualize' button on the gui is pressed
project4_visualize(roger)
Robot* roger;
{ }


