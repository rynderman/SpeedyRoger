/*************************************************************************/
/* File:        eye.c                                                    */
/* Description: all structures and dynamics specific to the eye          */
/* Author:      Rod Grupen                                               */
/* Date:        11-1-2009                                                */
/*************************************************************************/
#include <math.h>
#include "include/Roger.h"
#include "include/simulate.h"
#include "include/control.h"

extern int acq_count;
extern Obj obj;

// uses M_EYE, L_EYE, I_EYE(=M_EYE*L_EYE^2), GRAVITY in Roger.h

// the triangulation result
// Estimate observation[3]; // centroid, left_edge, right_edge

Eye eyes[NEYES]; // initialized in xrobot.c

simulate_eyes(xform, eyes)
double xform[4][4];
Eye eyes[NEYES];
{
  int i;
  double acc;

  for (i=0;i<NEYES;++i) {
    acc = (eyes[i].torque - M_EYE*GRAVITY*L_EYE*xform[0][0])/I_EYE;
    eyes[i].theta += 0.5*acc*DT*DT + eyes[i].theta_dot*DT;

    /* ==== Changed by Dan ===== */
    if (eyes[i].theta > M_PI/2.0) {
      eyes[i].theta = M_PI/2.0;
    }
    else if (eyes[i].theta < -M_PI/2.0) {
      eyes[i].theta = -M_PI/2.0;
    }
    /*if (eyes[i].theta > M_PI) {
      eyes[i].theta -= 2.0*M_PI;
    }
    else if (eyes[i].theta < -M_PI) {
      eyes[i].theta += 2.0*M_PI;
    }*/
    /* ========================= */
    
    eyes[i].theta_dot += acc*DT;
  }
  //  printf("          new actual = %6.4lf\n", eyes[RIGHT].theta);

}

