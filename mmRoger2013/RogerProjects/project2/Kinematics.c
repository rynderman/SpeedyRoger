/*************************************************************************/
/* File:        Kinematics.c                                             */
/* Description: Kinematic functions for Cartesian control:               */
/*              inv_arm_kinematics(); arm_Jacobian();
/* FUNCTION LIBRARY (provided/attached at bottom):                       */
/*    construct_wTb(roger->base_position, wTb) computes the homogeneous  */
/*          transform (double wTb[4][4]) relating the base frame to the  */
/*          to the world frame given input robot state (double x,y,theta)*/
/*    inverse_homogeneous_xform(aTb, bTa) - inverts a homogeneous xform  */
/*    matrix_times_vector(A,x,y) - multiplies double A[4][4] and input   */
/*          double x[4] to yield output double y[4] (y=Ax) - can be used */
/*         to apply homogeneous transforms to homogeneous vectors        */
/* Date:        11-17-2012                                               */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"

#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

/**********************************************************************/
/************* CARTESIAN ARM CONTROL **********************************/
/**********************************************************************/

/* code development stub called by simulator */
CartesianArmControl(roger, limb, x, y)
Robot * roger;
double x, y;
int limb;
{
  double px, py;
  double wTb[4][4], bTw[4][4], ref_b[4], ref_w[4];

  // input (x,y) is in world frame coordinates - map it into the base frame

  // modify setpoints for the arms accordingly
  inv_arm_kinematics(roger, limb, ref_b[X], ref_b[Y]);
}

/* Inverse Kinematics for the ARM - THIS PROCEDURE CAN BE CALLED BY ANY    */
/*       CONTROL TASK THAT SUBMITS CARTESIAN REFERENCES FOR THE HAND       */
/*       (x,y) in base coordinates, if in reach it modifies cspace         */
/*       setpoints for arm[limb], else it leaves the setpoints unchanged   */
int inv_arm_kinematics(roger, limb, x, y)
Robot * roger;
int limb;
double x, y;
{
  double r2, c2, s2_plus, s2_minus, theta2_plus, theta2_minus;
  double k1, k2_plus, k2_minus, alpha_plus, alpha_minus;
  double theta1_plus, theta1_minus;

  if (limb==LEFT) y -= ARM_OFFSET;
  else y += ARM_OFFSET;

  //roger->arm_setpoint[limb][0] = ...
  //roger->arm_setpoint[limb][1] = ...
  
  return FALSE;
}

/* === DOESN'T need to be done in Project 2 === */
arm_Jacobian(theta1,theta2, Jacobian)
double theta1,theta2;
double Jacobian[2][2];
{
}


/****** LIBRARY FUNCTIONS FOR YOUR USE  *************************************/
construct_wTb(base_pos, wTb)
double base_pos[3]; // (x,y,theta)
double wTb[4][4];
{
  double s0, c0;
  s0 = sin(base_pos[2]);
  c0 = cos(base_pos[2]);

  wTb[0][0] = c0;  wTb[0][1] = -s0; wTb[0][2] = 0.0; wTb[0][3] = base_pos[X];
  wTb[1][0] = s0;  wTb[1][1] = c0;  wTb[1][2] = 0.0; wTb[1][3] = base_pos[Y];
  wTb[2][0] = 0.0; wTb[2][1] = 0.0; wTb[2][2] = 1.0; wTb[2][3] = 0.0;
  wTb[3][0] = 0.0; wTb[3][1] = 0.0; wTb[3][2] = 0.0; wTb[3][3] = 1.0;
}

/******************* HOMOGENEOUS TRANSFORMS **************************/
inverse_homogeneous_xform(in, out)
double in[4][4], out[4][4];
{
  int i,j;
  for (i=0; i<3; ++i) {
    for (j=0; j<3; ++j) {
      out[i][j] = in[j][i];
    }
  }
  out[3][0] = out[3][1] = out[3][2] = 0.0; out[3][3] = 1.0;

  out[0][3] = -in[0][3]*in[0][0] - in[1][3]*in[1][0] - in[2][3]*in[2][0];
  out[1][3] = -in[0][3]*in[0][1] - in[1][3]*in[1][1] - in[2][3]*in[2][1];
  out[2][3] = -in[0][3]*in[0][2] - in[1][3]*in[1][2] - in[2][3]*in[2][2];
}

/*********************************************************************/
matrix_times_vector(mat, in, out)    // out = mat x in
double mat[4][4], in[4], out[4];
{
  int i,j;
  double sum;
  for (i=0; i<4; ++i) {
    sum = 0.0;
    for (j=0; j<4; ++j) {
      sum += mat[i][j] * in[j];
    }
    out[i] = sum;
  }
}
