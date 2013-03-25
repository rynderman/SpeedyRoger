/*************************************************************************/
/* File:        project3.c                                               */
/* Description: User project #3                                          */
/* Date:        11-25-2012                                               */
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
project3_control(roger,time)
Robot * roger;
double time;
{}

/*************************************************************************/
project3_reset(roger)
Robot* roger;
{ }

/*************************************************************************/
// prompt for and read user customized input values
/*************************************************************************/
project3_enter_params() 
{
  printf("Project 3 enter_params called. \n");
}

/*************************************************************************/
/* DO NOT ALTER */
// function called when the 'visualize' button on the gui is pressed
project3_visualize(roger)
Robot* roger;
{ }
