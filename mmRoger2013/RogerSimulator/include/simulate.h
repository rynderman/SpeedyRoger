 /**************************************************************************/
/* File:        simulate.h                                                */
/* Description: all the compile time constants for use in the simulator   */
/* Author:      Rod Grupen                                                */
/* Date:        11-1-2009
/**************************************************************************/

/********************* simulator X display **********************/
// simulator panel
#define WIDTH              1100//1060
#define HEIGHT	           480

//cartesian space
#define MIN_X             -2.0
#define MIN_Y             -2.0
#define MAX_X              2.0
#define MAX_Y              2.0
#define NBINS              64
#define XDELTA   	   ((MAX_X-MIN_X)/NBINS)
#define YDELTA   	   ((MAX_Y-MIN_Y)/NBINS)
#define R_OBSTACLE         (XDELTA/2.0)


#define T1                0
#define T2                1
#define T1_MIN      -(M_PI)
#define T2_MIN      -(M_PI)
#define T1_MAX       (M_PI)
#define T2_MAX       (M_PI)
#define TDELTA   ((2.0*M_PI)/NBINS)

// button geometry
#define BOXH	           18
#define BOXW	           ((int) WIDTH/6 - 6) //five buttons will fit

// sub-panel geometry and placement
#define LEFT_MAP_CENTER_X  170
#define LEFT_MAP_CENTER_Y  220
#define CENTER_X           530
#define CENTER_Y           220
#define RIGHT_MAP_CENTER_X 890
#define RIGHT_MAP_CENTER_Y 220

#define ZOOM_SCALE     (100.0)
#define MAP_SCALE          (50.0)
/* world to pixmap panels (drawing) transformations */
#define W2DR(scale,num)          (((int)(scale*1000.0*(num))/1000))
#define W2DX(scale,num)	   (CENTER_X+((int)(scale*1000.0*((num)))/1000))
#define W2DY(scale,num)	   (CENTER_Y-((int)(scale*1000.0*((num)))/1000))
/* pixmap to world */
#define D2WR(scale,num)	   ((double)(num*1000)/(scale*1000.0))
#define D2WX(scale,num)	   ((double)((num-CENTER_X)*1000)/(scale*1000.0))
#define D2WY(scale,num)	   ((double)((CENTER_Y-num)*1000)/(scale*1000.0))

// world to display theta
#define T2DR(num)   (((int)(MAP_SCALE*1000.0*(num))/1000))
#define T12LD(num)  (LEFT_MAP_CENTER_X+((int)(MAP_SCALE*1000.0*((num)))/1000))
#define T22LD(num)  (LEFT_MAP_CENTER_Y-((int)(MAP_SCALE*1000.0*((num)))/1000))
#define T12RD(num)  (RIGHT_MAP_CENTER_X+((int)(MAP_SCALE*1000.0*((num)))/1000))
#define T22RD(num)  (RIGHT_MAP_CENTER_Y-((int)(MAP_SCALE*1000.0*((num)))/1000))

#define LEFT_IMAGE_X       (CENTER_X - (IMAGE_WIDTH+5))
#define RIGHT_IMAGE_X      (CENTER_X + 5)
#define IMAGE_Y            440
#define PIXEL_WIDTH        (IMAGE_WIDTH/NPIXELS)
#define PIXEL_HEIGHT       20

/********************* simulation constants **********************/
#define TIMEOUT            60        /* seconds worth of simulation       */
#define DT                 0.001     /* the time increment between frames */
#define RENDER_RATE        20        /* render every twentieth state      */
#define SERVO_RATE         1         /* servo rate at 1000Hz (1 msec)     */
#define TIMER_UPDATE       5

#define NOFILL	           0
#define FILL               1

//#define RED          -3
//#define GREEN        -2
//#define BLUE         -1

// indices into world_color array (xrobot.c x_init_colors() )
#define DARKRED      101
#define RED          102
#define LIGHTRED     103
#define DARKBLUE     104
#define BLUE         105
#define LIGHTBLUE    106
#define DARKGREEN    107
#define GREEN        108
#define LIGHTGREEN   109
#define DARKYELLOW   110
#define YELLOW       111
#define LIGHTYELLOW  112

#define GAZE_COLOR   65
#define OBJECT_COLOR RED
#define ARM_COLOR    BLUE
#define EYE_COLOR	   DARKGREEN
#define GOAL_COLOR   GREEN 

/***********************************************************************/
/* CONSTANTS AND STRUCTURES THAT DEFINE MECHANISMS FOR THE SIMULATOR   */
#define NOTYPE       -1              /* KINEMATIC SPECIFICATIONS       */
#define NOAXIS       -1
#define REVOLUTE      0
#define PRISMATIC     1
#define XAXIS         0
#define YAXIS         1
#define ZAXIS         2

/***********************************************************************/
// STRUCTURES FOR THE STATE OF ALL THE DEVICES THAT COMPRISE ROGER
typedef struct _base {
  double wTb[4][4];
  double x;
  double x_dot;
  double y;
  double y_dot;
  double theta;
  double theta_dot;
  double wheel_torque[2];
  double contact_theta;
  double extForce[2];          // net (fx,fy) force on the base
} Base;

typedef struct _arm {
  double iTj[4][4];
  int dof_type;                // revolute or prismatic type
  int axis;                    // XAXIS, YAXIS, ZAXIS
  double theta;
  double theta_dot;
  double torque;
  double extForce[2];          // (fx,fy) force on distal endpoint of this link
} Arm;

typedef struct _eye {
  double position[2];          // position of the eye in world coordinates
  double theta;                // eye pan angle relative to world frame
  double theta_dot;            // angular velocity
  int    image[NPIXELS];       // afferent ONLY
  double torque;               // efferent ONLY - command torque
} Eye;

typedef struct _object {
  double mass;                 // intrinsic parameters 
  double position[2];          // position of the centroid of the object
  double velocity[2];          // velocity of the centroid of the object
  double extForce[2];          // written by the simulator: endpoint load
} Obj;

/*************************************************************************/
//typedef struct _world {
//  int occupancy_map[NBINS][NBINS]; // ground truth world geometry
//  int color_map[NBINS][NBINS];     // index into SimColors for "light" colors
//} World;
/*************************************************************************/

typedef struct _simcolor {
  char name[32];
  int display_color;
  int red;
  int green;
  int blue;
} SimColor;

/*************************************************************************/

#define MAX_HISTORY   1000

typedef struct _history {
  double arm_pos[NARMS][2];    /* (theta1, theta2) */
  double base_pos[3];          /* (x,y,theta) */
} History;


