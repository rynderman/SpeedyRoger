/*************************************************************************/
/* File:        MotorUnits.c                                             */
/* Description: motor units execute every simulated millisecond and are  */
/*              never disengaged, applications control Roger by altering */
/*              setpoints for each degree of freedom                     */
/* Date:        11-2012                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"

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
    //PDController_arms(roger, time);
    PDController_base(roger,time);


	double velocity = sqrt((roger->base_velocity[X]*roger->base_velocity[X])+(roger->base_velocity[Y]*roger->base_velocity[Y]));
	//printf("%f %f \n" , time , velocity );
	
}

/*************************************************************************/
/* PROJECT #1 - COMPLETE THE FOLLOWING CONTROL PROCEDURES                */
/*************************************************************************/

// gains for the PD controllers for eyes
double kp_eye = 4.0; //KP_EYE
double kd_eye = (sqrt(4.0*4.0*I_EYE)); //(sqrt(4.0*kp_eye*I_EYE)); //KD_EYE;

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
        roger->eye_torque[i] = kp_eye*theta_error - kd_eye*roger->eye_theta_dot[i];
    }
}

// gains for the PD controllers for arms
double kp_arm = 400; //KP_ARM;
double kd_arm = 17; //KD_ARM;

/* PROJECT #1.2 - PD CONTROLLER FOR THE ARMS                             */
/* setpoints - joint angles in radians for the shoulders and elbows      */
/*
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
        
        //roger->arm_torque[i][0] = ...
        //roger->arm_torque[i][1] = ...
    }
}
*/

/* PROJECT #2.1 - PD CONTROLLER FOR THE BASE                             */
/* setpoints - (xy) location for translation heading in radians          */


//double baseJT[2][2] = ...

PDController_base(roger, time)
Robot * roger;
double time;
{
    double Fx, Mz, PDBase_translate(), PDBase_rotate();
    
    Fx = PDBase_translate(roger,time);
    Mz = PDBase_rotate(roger,time);
    
    // integrated wheel torque control
    roger->wheel_torque[LEFT] = Fx/2 - Mz/(2*0.2);
    roger->wheel_torque[RIGHT] = Fx/2 + Mz/(2*0.2);
}

// gains for base translational controller
//double kp_base_trans = 175.0; // KP_BASE      M=0.9 kg //mass of eyes?
//double kd_base_trans = 25.0;  // KD_BASE  should be critical sqrt(4KM)

double kp_base_trans = 130.0; // KP_BASE      M=0.9 kg //mass of eyes?220
double kd_base_trans = 24.0;  // KD_BASE  should be critical sqrt(4KM)

/* Base PD controller, Cartesian reference */
double PDBase_translate(roger, time)
Robot * roger;
double time;
{
    double Fx;
    
    /*
    int i, j;
    double wTb[4][4], bTw[4][4], e[2], e_dot[2];
    double ref_b[4], ref_w[4], v_b[4], v_w[4];
    
    // construct the homogeneous transform from the world to the mobile base
    construct_wTb(roger->base_position, wTb);
    
    // position feedback
    inverse_homogeneous_xform(wTb, bTw);
    
    ref_w[0] = roger->base_setpoint[X];
    ref_w[1] = roger->base_setpoint[Y];
    ref_w[2] = 0.0;
    ref_w[3] = 1.0;
    
    matrix_times_vector(bTw, ref_w, ref_b);
    
    // "control yoke" - the control offset of the mobile base in base coordinates
    e[X] = ref_b[X] - BASE_CONTROL_OFFSET;
    e[Y] = ref_b[Y];
    
    v_w[0] = roger->base_velocity[X];
    v_w[1] = roger->base_velocity[Y];
    v_w[2] = 0.0;
    v_w[3] = 0.0; // not a position vector
    matrix_times_vector(bTw, v_w, v_b);
    
    v_b[Y] = v_b[Y] + roger->base_velocity[2] * BASE_CONTROL_OFFSET;
    Fx = kp_base_trans*e[X] - kd_base_trans*v_b[X];
     */
     
    return(Fx);
}

// gains for base rotational controller
double kp_base_rot = 150;   //KP_BASE                I=0.0234 kg m^2 250
double kd_base_rot = 65.0;   //KD_BASE   should be critical sqrt(4KI)

/* Base PD controller, Cartesian reference */
double PDBase_rotate(roger, time)
Robot * roger;
double time;
{
    double Mz;
    //Mz = ...
    // mz = heading error * moment stuff?

/*
    int i, j;
    double wTb[4][4], bTw[4][4], e[2], e_dot[2];
    double ref_b[4], ref_w[4], v_b[4], v_w[4];
    
    construct_wTb(roger->base_position, wTb);
    
    // position feedback
    inverse_homogeneous_xform(wTb, bTw);
    
    ref_w[0] = roger->base_setpoint[X];
    ref_w[1] = roger->base_setpoint[Y];
    ref_w[2] = 0.0;
    ref_w[3] = 1.0;
    
    matrix_times_vector(bTw, ref_w, ref_b);
    
    // "control yoke" - the control offset of the mobile base in base coordinates
    e[X] = ref_b[X] - BASE_CONTROL_OFFSET;
    e[Y] = ref_b[Y];
    
    v_w[0] = roger->base_velocity[X];
    v_w[1] = roger->base_velocity[Y];
    v_w[2] = 0.0;
    v_w[3] = 0.0; // not a position vector
    matrix_times_vector(bTw, v_w, v_b);
    
    //add influence of angular velocity of body
    v_b[Y] = v_b[Y] + roger->base_velocity[2] * BASE_CONTROL_OFFSET;
    
    double force_y = kp_base_rot*e[Y] - kd_base_rot*v_b[Y];
    Mz = force_y*BASE_CONTROL_OFFSET;
    */
    
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
