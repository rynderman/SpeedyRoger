/*************************************************************************/
/* File:        mobile_base.c                                            */
/* Description: all structures and dynamics specific to the mobile base  */
/* Author:      Rod Grupen                                               */
/* Date:        7-25-09                                                  */
/*************************************************************************/
#include <math.h>
#include "include/Roger.h"
#include "include/simulate.h"
#include "include/control.h"

static double Mbase = (0.5+MARM_1+MARM_2);    /* kg  ( note: I = m*l^2) */
static double Ibase = ((0.5+MARM_1+MARM_2)*SQR(R_BASE));      /* kg m^2 */

Base mobile_base_home =
  { { {  0.0, 1.0, 0.0,  0.0 },  /* wTbase */
      { -1.0, 0.0, 0.0,  0.0 },  
      {  0.0, 0.0, 1.0,  0.0 },
      {  0.0, 0.0, 0.0,  1.0 } },
    0.0, /* x */
    0.0, /* x_dot */
    0.0, /* y */
    0.0, /* y_dot */
    -M_PI/2.0, /* theta */
    0.0, /* theta_dot */
    { 0.0, 0.0 }, /* wheel_torque */
    0.0, /* contact_theta */
    { 0.0, 0.0 } /* extForce (fx, fy) */
  };    

// numerically integrate dynamic equations and update mobile_base state
simulate_base(mobile_base)
Base * mobile_base;
{
  int i, j;
  double dtheta, vmag, tx, ty, fx, fy;
  double s_ddot, theta_ddot;

  // the (positive) heading of the mobile base
  tx = cos(mobile_base->theta);
  ty = sin(mobile_base->theta);

  fx = mobile_base->extForce[X];
  fy = mobile_base->extForce[Y];

  //  s_ddot = (mobile_base->wheel_torque[LEFT] +  
  //	    mobile_base->wheel_torque[RIGHT] +
  //	    mobile_base->extForce[X]*tx + mobile_base->extForce[Y]*ty) / Mbase;

  s_ddot = (mobile_base->wheel_torque[LEFT] +  
	    mobile_base->wheel_torque[RIGHT]) / Mbase;

  //  printf("tangential acceleration of the base = %6.4lf\n", s_ddot);
  //  printf("                  tangent direction = %6.4lf %6.4lf\n", tx, ty);
  //  printf("                           extForce = %6.4lf %6.4lf\n", fx, fy);

  theta_ddot = R_BASE * (mobile_base->wheel_torque[RIGHT] - 
			 mobile_base->wheel_torque[LEFT]) / Ibase;

	//TODO: is this right?
  dtheta = 0.5*theta_ddot*SQR(DT) + mobile_base->theta_dot*DT;
  mobile_base->theta_dot += theta_ddot* DT;

  vmag = cos(mobile_base->theta)*mobile_base->x_dot + 
    sin(mobile_base->theta)*mobile_base->y_dot;

  mobile_base->theta += dtheta;

  while (mobile_base->theta > M_PI)
	mobile_base->theta -= 2.0 * M_PI;
  while (mobile_base->theta < -M_PI)
	mobile_base->theta += 2.0 * M_PI;

  mobile_base->x += (0.5*s_ddot*SQR(DT)+vmag*DT)*cos(mobile_base->theta);
  mobile_base->y += (0.5*s_ddot*SQR(DT)+vmag*DT)*sin(mobile_base->theta);


  //  mobile_base->x_dot += s_ddot*cos(mobile_base->theta) * DT;
  //  mobile_base->y_dot += s_ddot*sin(mobile_base->theta) * DT;
  mobile_base->x_dot = (vmag + s_ddot*DT)*cos(mobile_base->theta);
  mobile_base->y_dot = (vmag + s_ddot*DT)*sin(mobile_base->theta);

  // update permanent state
  mobile_base->wTb[0][0] = mobile_base->wTb[1][1] = cos(mobile_base->theta);
  mobile_base->wTb[1][0] = sin(mobile_base->theta);
  mobile_base->wTb[0][1] = -mobile_base->wTb[1][0];
  mobile_base->wTb[0][3] =   mobile_base->x;
  mobile_base->wTb[1][3] =   mobile_base->y;
}

