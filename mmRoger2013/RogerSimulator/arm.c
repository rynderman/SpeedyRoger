/*************************************************************************/
/* File:        arm.c                                                    */
/* Description: all data structures and dynamics specific to the arm     */
/* Author:      Rod Grupen                                               */
/* Date:        11-1-2009                                                */
/*************************************************************************/
#include <math.h>
#include "include/Roger.h"
#include "include/simulate.h"
#include "include/control.h"

static double l[2] = {LARM_1, LARM_2};
static double m[2] = {MARM_1, MARM_2};

Arm arms_home[NARMS][NARM_FRAMES] =
{ { { { { 1.0, 0.0, 0.0, 0.0 },         /* LEFT ARM         */
	{ 0.0, 1.0, 0.0, ARM_OFFSET },  /* mobile base to frame 0 */
	{ 0.0, 0.0, 1.0, 0.0 },
	{ 0.0, 0.0, 0.0, 1.0 } }, NOTYPE,NOAXIS, 0.0,0.0,
                                  0.0, {0.0,0.0}},
    { { { 1.0, 0.0, 0.0, 0.0 },  /* frame 0 to frame 1 */
	{ 0.0, 1.0, 0.0, 0.0  },
	{ 0.0, 0.0, 1.0, 0.0 },
	{ 0.0, 0.0, 0.0, 1.0 } }, REVOLUTE,ZAXIS, (11.0*M_PI/24.0),0.0,
                                  0.0, {0.0,0.0}},
    { { { 1.0, 0.0, 0.0, LARM_1 },  /* frame 1 to frame 2 */
	{ 0.0, 1.0, 0.0, 0.0 },
	{ 0.0, 0.0, 1.0, 0.0 },
	{ 0.0, 0.0, 0.0, 1.0 } }, REVOLUTE,ZAXIS, (-5.0*M_PI/6.0),0.0,
                                  0.0, {0.0,0.0}},
    { { { 1.0, 0.0, 0.0, LARM_2 },  /* frame 2 to frame 3 */
	{ 0.0, 1.0, 0.0, 0.0 },
	{ 0.0, 0.0, 1.0, 0.0 },
	{ 0.0, 0.0, 0.0, 1.0 } }, NOTYPE,NOAXIS, 0.0,0.0,
                                  0.0, {0.0,0.0}} },
  { { { { 1.0, 0.0, 0.0, 0.0 },           /* RIGHT ARM        */
	{ 0.0, 1.0, 0.0, -ARM_OFFSET },  /* world to frame 0 */
	{ 0.0, 0.0, 1.0, 0.0 },
	{ 0.0, 0.0, 0.0, 1.0 } }, NOTYPE,NOAXIS, 0.0,0.0,
                                  0.0, {0.0,0.0}},
    { { { 1.0, 0.0, 0.0, 0.0 },  /* frame 0 to frame 1 */
	{ 0.0, 1.0, 0.0, 0.0 },
	{ 0.0, 0.0, 1.0, 0.0 },
	{ 0.0, 0.0, 0.0, 1.0 } }, REVOLUTE,ZAXIS, (-11.0*M_PI/24.0),0.0,
                                  0.0, {0.0,0.0}},
    { { { 1.0, 0.0, 0.0, LARM_1 },  /* frame 1 to frame 2 */
	{ 0.0, 1.0, 0.0, 0.0 },
	{ 0.0, 0.0, 1.0, 0.0 },
	{ 0.0, 0.0, 0.0, 1.0 } }, REVOLUTE,ZAXIS, (5.0*M_PI/6.0),0.0,
                                  0.0, {0.0,0.0}},
    { { { 1.0, 0.0, 0.0, LARM_2 },  /* frame 2 to frame 3 */
	{ 0.0, 1.0, 0.0, 0.0 },
	{ 0.0, 0.0, 1.0, 0.0 },
	{ 0.0, 0.0, 0.0, 1.0 } }, NOTYPE,NOAXIS, 0.0,0.0,
                                  0.0, {0.0,0.0}} } };

simulate_arm(xform, arms)
double xform[4][4];
Arm arms[NARMS][NARM_FRAMES];
{
  int i;
  double M[NARM_JOINTS][NARM_JOINTS], V[NARM_JOINTS], G[NARM_JOINTS], F[NARM_JOINTS];
  double arm_acc[NARM_JOINTS];

  for (i=0;i<NARMS;++i) {
    arm_dynamics(arms[i], M,V,G,F);
    arm_accelerations(M,arms[i],V,G,F, arm_acc);
    euler_arm(arm_acc, arms[i]);
  }
}

arm_dynamics(arm,M,V,G,F)
Arm arm[NARM_FRAMES];
double M[NARM_JOINTS][NARM_JOINTS],V[NARM_JOINTS], G[NARM_JOINTS],F[NARM_JOINTS];
{
  double s1, c1, s2, c2, s12, c12;

  s1 = sin(arm[1].theta);
  c1 = cos(arm[1].theta);
  s2 = sin(arm[2].theta);
  c2 = cos(arm[2].theta);
  s12 = sin(arm[1].theta + arm[2].theta);
  c12 = cos(arm[1].theta + arm[2].theta);

  /* fill in non-zero terms for M */
  M[0][0] = SQR(l[1])*m[1] + 2.0*l[0]*l[1]*m[1]*c2 + SQR(l[0])*(m[0]+m[1]);
  M[1][1] = SQR(l[1])*m[1];
  M[1][0] = M[1][1] + l[0]*l[1]*m[1]*c2;
  M[0][1] = M[1][0];

  V[0] = -m[1]*l[0]*l[1]*s2*SQR(arm[2].theta_dot) - 
          2.0*m[1]*l[0]*l[1]*s2*arm[1].theta_dot*arm[2].theta_dot;
  V[1] = m[1]*l[0]*l[1]*s2*SQR(arm[1].theta_dot);

  G[1] = m[1]*l[1]*GRAVITY*c12;
  G[0] = G[1] + (m[0]+m[1])*l[0]*GRAVITY*c1;
  //  G[1] = 0.0;
  //  G[0] = 0.0;

  /* torques due to endpoint forces tau = J^T force */
  F[0] = -(l[0]*s1+l[1]*s12)*arm[NARM_FRAMES-1].extForce[0] +
          (l[0]*c1+l[1]*c12)*arm[NARM_FRAMES-1].extForce[1];
  F[1] = -(l[0]*s12)*arm[NARM_FRAMES-1].extForce[0] +
          (l[1]*c12)*arm[NARM_FRAMES-1].extForce[1];
}

arm_accelerations(M,arm,V,G,F, theta_ddot)
Arm arm[NARM_FRAMES];
double M[NARM_JOINTS][NARM_JOINTS],V[NARM_JOINTS],G[NARM_JOINTS],F[NARM_JOINTS];
double theta_ddot[NARM_JOINTS];
{
  int i;
  double arg[NARM_JOINTS], Minv[NARM_JOINTS][NARM_JOINTS];;

  invert(M,Minv);
  for (i=0; i<NARM_JOINTS; ++i) {
    arg[i] = arm[i+1].torque - V[i] - G[i] - F[i];
  }
  matrix_mult(Minv,arg,theta_ddot);
}

euler_arm(acc, arm)
double acc[NARM_JOINTS];
Arm arm[NARM_FRAMES];
{
  int i;
  int joint = 0;

  /* update positions and velocities of roger's manipulator */
  for (i=0;i<NARM_FRAMES;++i) {
    if (arm[i].dof_type != NOTYPE) {
      if (arm[i].dof_type == REVOLUTE) {
	arm[i].theta += 0.5*acc[joint]*DT*DT + arm[i].theta_dot*DT;
	arm[i].theta_dot += acc[joint]*DT;
	++joint;
      }
      else if (arm[i].dof_type == PRISMATIC) {
	arm[i].theta += 0.5*acc[joint]*DT*DT + arm[i].theta_dot*DT;
	arm[i].theta_dot += acc[joint]*DT;
	if (arm[i].theta < 0.0) { 
	  arm[i].theta = 0.0; arm[i].theta_dot = 0.0;
	}
	if (arm[i].theta > l[joint+1]) {
	  arm[i].theta = l[joint+1];
	  arm[i].theta_dot = 0.0;
	}
	++joint;
      }
    }
  }
  rectify_theta(arm);
  update_state(arm);
}

update_state(arm)
Arm arm[NARM_FRAMES];
{
  int i;

  /* update transforms that describe position of roger's manipulator */
  for(i=1;i<NARM_FRAMES;++i) {
    switch(arm[i].axis) {
      case(XAXIS):   /* dof axis is local x axis */
	if (arm[i].dof_type == REVOLUTE) {
	  arm[i].iTj[1][1] =  cos(arm[i].theta);
	  arm[i].iTj[1][2] = -sin(arm[i].theta);
	  arm[i].iTj[2][1] = -arm[i].iTj[1][2];
	  arm[i].iTj[2][2] = 	arm[i].iTj[1][1];
	}
	else if (arm[i].dof_type == PRISMATIC) {
	  arm[i].iTj[0][3] = arm[i].theta;
	}
      break;

      case(YAXIS):   /* dof axis is local y axis */
	if (arm[i].dof_type == REVOLUTE) {
	  arm[i].iTj[0][0] = cos(arm[i].theta);
	  arm[i].iTj[0][2] = sin(arm[i].theta);
	  arm[i].iTj[2][0] = -arm[i].iTj[0][2];
	  arm[i].iTj[2][2] =  arm[i].iTj[0][0];
	}
	else if (arm[i].dof_type == PRISMATIC) {
	  arm[i].iTj[1][3] = arm[i].theta;
	}
      break;

      case(ZAXIS):   /* dof axis is local z axis */
	if (arm[i].dof_type == REVOLUTE) {
	  arm[i].iTj[0][0] = cos(arm[i].theta);
	  arm[i].iTj[0][1] = -sin(arm[i].theta);
	  arm[i].iTj[1][0] = -arm[i].iTj[0][1];
	  arm[i].iTj[1][1] =  arm[i].iTj[0][0];
	}
	else if (arm[i].dof_type == PRISMATIC) {
	  arm[i].iTj[2][3] = l[i] + arm[i].theta;
	}
      break;
    }
  }
}

invert(A,Ainv)
double A[NARM_JOINTS][NARM_JOINTS], Ainv[NARM_JOINTS][NARM_JOINTS];
{
   double det;

   switch(NARM_JOINTS) {
   case (1):
     Ainv[0][0] = 1.0/A[0][0];
     break;
   case (2):
     det = 1.0 / (A[0][0]*A[1][1] - A[1][0]*A[0][1]);
      
     Ainv[0][0] = det * A[1][1];
     Ainv[1][0] = -1.0 * det * A[1][0];
     Ainv[0][1] = -1.0 * det * A[0][1];
     Ainv[1][1] = det * A[0][0];
     break;
   
   case(3):
     det = 1.0 / (A[0][0]*A[1][1]*A[2][2] + A[0][1]*A[1][2]*A[2][0] +
		  A[0][2]*A[1][0]*A[2][1] - A[2][0]*A[1][1]*A[0][2] -
		  A[2][1]*A[1][2]*A[0][0] - A[2][2]*A[1][0]*A[0][1]) ;
     
     Ainv[0][0] = det * (A[1][1]*A[2][2]-A[2][1]*A[1][2]);
     Ainv[0][1] = det * (A[2][1]*A[0][2]-A[0][1]*A[2][2]);
     Ainv[0][2] = det * (A[0][1]*A[1][2]-A[1][1]*A[0][2]);
     Ainv[1][0] = det * (A[2][0]*A[1][2]-A[1][0]*A[2][2]);
     Ainv[1][1] = det * (A[0][0]*A[2][2]-A[2][0]*A[0][2]);
     Ainv[1][2] = det * (A[1][0]*A[0][2]-A[0][0]*A[1][2]);
     Ainv[2][0] = det * (A[1][0]*A[2][1]-A[2][0]*A[1][1]);
     Ainv[2][1] = det * (A[2][0]*A[0][1]-A[0][0]*A[2][1]);
     Ainv[2][2] = det * (A[0][0]*A[1][1]-A[1][0]*A[0][1]);
     break;
   }
 }

matrix_mult(A,x,y)
double A[NARM_JOINTS][NARM_JOINTS],x[NARM_JOINTS],y[NARM_JOINTS];
{
  int i,j;
  for (i=0; i<NARM_JOINTS; ++i) {
    y[i] = 0.0;
    for (j=0; j<NARM_JOINTS; ++j) {
      y[i] += A[i][j] * x[j];
    }
  }
}

// HACK
rectify_theta(arm)
Arm arm[NARM_FRAMES]; 
{
  while (arm[1].theta < (-M_PI))
    arm[1].theta += 2.0 * M_PI;
  while (arm[1].theta > M_PI)
    arm[1].theta -= 2.0 * M_PI;
  while (arm[2].theta < (-M_PI))
    arm[2].theta += 2.0 * M_PI;
  while (arm[2].theta > M_PI)
    arm[2].theta -= 2.0 * M_PI;
}





