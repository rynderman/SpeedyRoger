/*************************************************************************/
/* File:        MotorUnits.c                                             */
/* Description: motor units execute every simulated millisecond and are  */
/*              never disengaged, applications control Roger by altering */
/*              setpoints for each degree of freedom                     */
/* Date:        11-2012                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
// #include "Xkw/Xkw.h"

#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

/*************************************************************************/
/*       THE SIMULATOR EXECUTES control_roger() EVERY CONTROL CYCLE      */
/*                        *** DO NOT ALTER ***                           */
/*************************************************************************/
control_roger(roger, time)
Robot * roger;
double time;
{
    update_setpoints(roger);
    
    // turn setpoint references into torques
    PDController_eyes(roger, time);
    PDController_arms(roger, time);
    PDController_base(roger,time);
}

/*************************************************************************/
/* PROJECT #1 - COMPLETE THE FOLLOWING CONTROL PROCEDURES                */
/*************************************************************************/

// gains for the PD controllers for eyes
double kp_eye = 1.0; //KP_EYE
double kd_eye = (sqrt(4.0*1.0*I_EYE)); //(sqrt(4.0*kp_eye*I_EYE)); //KD_EYE;

/* PROJECT #1.1 - PD CONTROLLER FOR THE EYES                             */
/* setpoints are joint angle values in radians for the eyes              */
PDController_eyes(roger, time)
Robot * roger;
double time;
{
    int i;
    double theta_error, theta_ddot_des;
    
    for (i = 0; i < NEYES; i++) {
        theta_error = roger->eyes_setpoint[i] - roger->eye_theta[i];
        
        theta_ddot_des = kp_eye*theta_error - kd_eye*roger->eye_theta_dot[i];
        
        // feedforward compensator
        // roger->eye_torque[i] = (M_EYE*SQR(L_EYE)) * theta_ddot_des +
        //       M_EYE*GRAVITY*L_EYE*cos(roger->eye_theta[i]);
        roger->eye_torque[i] = theta_ddot_des;
    }
}

// gains for the PD controllers for arms
double kp_arm = 400; //KP_ARM;
double kd_arm = 17; //KD_ARM;

/* PROJECT #1.2 - PD CONTROLLER FOR THE ARMS                             */
/* setpoints - joint angles in radians for the shoulders and elbows      */
PDController_arms(roger, time)
Robot * roger;
double time;
{
    int i;
    double theta_error[2], acc[2], M[2][2], V[2], G[2];
    
    for (i=LEFT; i<=RIGHT; ++i) {
        // PDcontrol - desired accelerations
        theta_error[0] = roger->arm_setpoint[i][0] - roger->arm_theta[i][0];
        theta_error[1] = roger->arm_setpoint[i][1] - roger->arm_theta[i][1];
        
        //shoulder - bound error -pi < error < +pi
        while (theta_error[0] > M_PI) theta_error[0] -= 2.0 * M_PI;
        while (theta_error[0] < -M_PI) theta_error[0] += 2.0 * M_PI;
        //elbow - bound error -pi < error < +pi
        while (theta_error[1] > M_PI) theta_error[1] -= 2.0 * M_PI;
        while (theta_error[1] < -M_PI) theta_error[1] += 2.0 * M_PI;
        
        // tune kp_arm and kd_arm by changing their value using enter_params()
        acc[0] = kp_arm * (theta_error[0]) +
        kd_arm * (0.0 - roger->arm_theta_dot[i][0]);
        acc[1] = kp_arm * (theta_error[1]) +
        kd_arm * (0.0 - roger->arm_theta_dot[i][1]);
        
        roger->arm_torque[i][0] = acc[0];
        roger->arm_torque[i][1] = acc[1];
    }
}

/* PROJECT #1.3 - PD CONTROLLER FOR THE BASE                             */
/* setpoints - (xy) location for translation heading in radians          */

/*   the base differential drive Jacobian:                               */
/*    |tau_l|     |Fx|      |  x_dot  |     |theta_dot_left |            */
/*    |     |= JT |  |      |         | = J |               |            */
/*    |tau_r| =   |Mz|      |theta_dot|     |theta_dot_right|            */

double baseJT[2][2] = {{(1.0/2.0), -(1.0/(2.0*R_AXLE))},
    {(1.0/2.0),  (1.0/(2.0*R_AXLE))} };

PDController_base(roger, time)
Robot * roger;
double time;
{
    double Fx, Mz, PDBase_translate(), PDBase_rotate();
    
    Fx = PDBase_translate(roger,time);
    Mz = PDBase_rotate(roger,time);
    
    // integrated wheel torque control
    roger->wheel_torque[LEFT] = baseJT[0][0]*Fx + baseJT[0][1]*Mz;
    roger->wheel_torque[RIGHT] = baseJT[1][0]*Fx + baseJT[1][1]*Mz;
}

// gains for base translational controller
double kp_base_trans = 175.0; // KP_BASE      M=0.9 kg //mass of eyes?
double kd_base_trans = 25.0;  // KD_BASE  should be critical sqrt(4KM)

/* Base PD controller, Cartesian reference */
double PDBase_translate(roger, time)
Robot * roger;
double time;
{
    double Fx, error[2], trans_error, trans_vel;
    
    error[X] = roger->base_setpoint[X] - roger->base_position[X];
    error[Y] = roger->base_setpoint[Y] - roger->base_position[Y];
    // position error between setpoint and yoke tip
    
    trans_error = error[X]*cos(roger->base_position[THETA]) +
    error[Y]*sin(roger->base_position[THETA]);
    
    trans_vel = roger->base_velocity[X]*cos(roger->base_position[THETA]) +
    roger->base_velocity[Y]*sin(roger->base_position[THETA]);
    
    Fx = kp_base_trans * trans_error - kd_base_trans * trans_vel;
    
    return(Fx);
}

// gains for base rotational controller
double kp_base_rot = 100.0;   //KP_BASE                I=0.0234 kg m^2
double kd_base_rot = 3.059;   //KD_BASE   should be critical sqrt(4KI)

/* Base PD controller, Cartesian reference */
double PDBase_rotate(roger, time)
Robot * roger;
double time;
{
    double theta_error, Mz;
    
    theta_error = roger->base_setpoint[THETA] - roger->base_position[THETA];
    
    while (theta_error > M_PI) theta_error -= 2.0 * M_PI;
    while (theta_error < -M_PI) theta_error += 2.0 * M_PI;
    
    Mz = kp_base_rot * theta_error - kd_base_rot * roger->base_velocity[THETA];
    
    return(Mz);
}

/*************************************************************************/
/* PROMPT FOR AND READ USER CUSTOMIZED INPUT VALUES                      */
/*************************************************************************/
enter_params()
{
    // put anything in here that you would like to change at run-time
    // without re-compiling user project codes
    
    //  printf(" K = %6.4lf   B = %6.4lf\n", kp_eye, kd_eye);
    //  printf("enter K\n"); fflush(stdout);
    //  scanf("%lf", &kp_eye);
    //	kd_eye = sqrt(4.0*kp_eye*I_EYE);
    
    printf("ARM: K=%6.4lf  B=%6.4lf\n", kp_arm, kd_arm);
    printf("ARM: enter 'K B'\n"); fflush(stdout);
    scanf("%lf %lf", &kp_arm, &kd_arm);
}
