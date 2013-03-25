/*************************************************************************/
/* File:        object.c                                                 */
/* Description: all structures and dynamics specific to the objects      */
/* Author:      Rod Grupen                                               */
/* Date:        11-1-2009                                                */
/*************************************************************************/
#include <math.h>
#include "include/Roger.h"
#include "include/simulate.h"
#include "include/control.h"

Obj object; // initialized in xrobot.c

simulate_object(obj)
Obj * obj;
{
  int i;
  double acc[2];
  double mag, fnorm[2];
/*
  printf("net force on object = %6.4lf %6.4lf\n",
	 obj->extForce[0], obj->extForce[1]);*/
  mag = sqrt(SQR(obj->extForce[0])+SQR(obj->extForce[1]));
  fnorm[0] = fnorm[1] = 0.0;

  if (mag < STATIC_FORCE_THRESHOLD) mag=0.0;
  else {
    fnorm[0] = obj->extForce[0]/mag; fnorm[1] = obj->extForce[1]/mag;
    mag -= STATIC_FORCE_THRESHOLD;
  }

  acc[X] = (mag*fnorm[0] - VISCOSITY*obj->velocity[X])/obj->mass;
  acc[Y] = (mag*fnorm[1] - VISCOSITY*obj->velocity[Y])/obj->mass;

  //    acc[0] = (obj->extForce[0] - VISCOSITY*obj->vel[0])/obj->mass;
  //    acc[1] = (obj->extForce[1] - VISCOSITY*obj->vel[1])/obj->mass;

  //acc[0] = 0.0;
  //acc[1] = 0.0;

  obj->velocity[X] += acc[X] * DT;
  obj->velocity[Y] += acc[Y] * DT;

  obj->position[X] += 0.5*acc[X]*SQR(DT) + obj->velocity[X]*DT;
  obj->position[Y] += 0.5*acc[Y]*SQR(DT) + obj->velocity[Y]*DT;

  //  if ((obj->position[X] < MIN_X + R_OBJ) && (obj->velocity[X] < 0.0))
  //    obj->velocity[X] *= -1.0;
  //  if ((obj->position[X] > MAX_X - R_OBJ) && (obj->velocity[X] > 0.0))
  //    obj->velocity[X] *= -1.0;
  //  if ((obj->position[Y] < MIN_Y + R_OBJ) && (obj->velocity[Y] < 0.0))
  //    obj->velocity[Y] *= -1.0;
  //  if ((obj->position[Y] > MAX_Y - R_OBJ) && (obj->velocity[Y] > 0.0))
  //    obj->velocity[Y] *= -1.0;
}







