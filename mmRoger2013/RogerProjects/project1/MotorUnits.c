/*************************************************************************/
/* File:        MotorUnits.c                                             */
/* Description: motor units execute every simulated millisecond and are  */
/*              never disengaged, applications control Roger by altering */
/*              setpoints for each degree of freedom                     */
/* Date:        11-2012                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>

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
}

// gains for the PD controllers for eyes
double kp_eye = 1.0; //KP_EYE
double kd_eye = (sqrt(4.0*1.0*I_EYE)); //(sqrt(4.0*kp_eye*I_EYE)); //KD_EYE;
double commandVel = 0.0;

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
        roger->eye_torque[i] = theta_ddot_des;
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

float max = 0.0f;

PDController_base(roger, time)
Robot * roger;
double time;
{
    double Fx, Mz, PDBase_translate(), PDBase_rotate();
    
	 //Fx = PDBase_translate(roger,time);
    //Fx = PDBase_translate(roger, time);

	 Fx = commandVel;
    
	Mz = PDBase_rotate(roger,time);

	float velocity = roger->base_velocity[X];
	if(velocity > max){
		//printf("%f %f \n" , velocity, time);
	max = velocity;
	}
	
    // integrated wheel torque control
    roger->wheel_torque[LEFT] = ( baseJT[0][0]*Fx + baseJT[0][1]*Mz );
    roger->wheel_torque[RIGHT] = ( baseJT[1][0]*Fx + baseJT[1][1]*Mz );
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
    //printf("ARM: K=%6.4lf  B=%6.4lf\n", kp_arm, kd_arm);
    //printf("ARM: enter 'K B'\n"); fflush(stdout);
    //scanf("%lf %lf", &kp_arm, &kd_arm);
}
