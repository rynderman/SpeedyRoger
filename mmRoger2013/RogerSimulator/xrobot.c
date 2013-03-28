/****************************************************************/
/** xrobot.c: simulates and renders mobile manipulator         **/
/**           version of Roger-the-Crab                        **/
/** author:   Grupen                                           **/
/** date:     April, 2010                                      **/
/****************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "Xkw/Xkw.h"

#include "include/Roger.h"
#include "include/simulate.h"
#include "include/control.h"
#include "include/modes.h"

// simulator data structures that comprise Roger
Base mobile_base;
Base mobile_base_home;
Eye eyes[NEYES];
Eye eyes_home[NEYES];
Arm arms[NARMS][NARM_FRAMES];
Arm arms_home[NARMS][NARM_FRAMES];
Obj object;
Robot Roger;// the user application interface data structure

SimColor world_colors[113];

History history[MAX_HISTORY]; // a diagnostic tool to illustrate a trajectory
int history_ptr=0;

//extern init_control_flag;
int init_control_flag = TRUE;

// global Boolean reset flag - eliminates user defined obstacles and goals
// reconfigures Roger and single object to starting configuration
int reset;

// global simulator clock
double simtime = 0.0;

void x_canvas_proc(), x_start_proc(), x_params_proc(), x_input_mode_proc(),
  x_control_mode_proc();
void x_quit_proc(), x_timer_proc(), x_visualize_proc();

Display          *display;
Window           window;
Pixmap           pixmap;
XtAppContext     app_con;
GC               gc;
int              screen;
Widget           canvas_w, input_mode_w, control_mode_w, params_w,
  start_w, popup_w = NULL;
Widget           intensity_w, stream_w;
XtIntervalId     timer = 0;
int              width = WIDTH, height = HEIGHT, depth;
unsigned long    foreground, background;

int zoom = ZOOM_SCALE; 

main(argc, argv)
int argc;char **argv; 
{
  int i;
  static String fallback_resources[] = { 
    "*title:	Roger-the-Crab",
    "*Roger-the-Crab*x:	100", 
    "*Roger-the-Crab*y:	100",
    NULL,
  };
  Widget toplevel, form, widget;
  void x_clear();
  
  toplevel = XtAppInitialize(&app_con, "Roger-the-Crab", NULL, ZERO, &argc,
			     argv, fallback_resources, NULL, ZERO);
  form = XkwMakeForm(toplevel);
  widget = NULL;
  start_w = widget = XkwMakeCommand(form, NULL, widget, x_start_proc, 
				    "Start", BOXW, BOXH);
  input_mode_w = widget = XkwMakeCommand(form, NULL, widget, x_input_mode_proc,
					 "Input: Joint angles", BOXW, BOXH);
  control_mode_w = widget = XkwMakeCommand(form, NULL, widget,
					   x_control_mode_proc,
					   "Teleoperation", BOXW, BOXH);
  params_w = widget = XkwMakeCommand(form, NULL, widget, x_params_proc,
				     "Enter Params", BOXW, BOXH);
  stream_w = widget = XkwMakeCommand(form, NULL, widget, x_visualize_proc,
				     "Visualize",  BOXW, BOXH);
  widget = XkwMakeCommand(form, NULL, widget, x_quit_proc,
			  "Quit", BOXW,	BOXH);
  canvas_w = widget = XkwMakeCanvas(form, widget, NULL,
				    x_canvas_proc, width, height);
  XtRealizeWidget(toplevel);
  display = XtDisplay(canvas_w);
  window = XtWindow(canvas_w);
  screen = DefaultScreen(display);
  depth = DefaultDepth(display, screen);
  foreground = BlackPixel(display, screen);
  background = WhitePixel(display, screen);
	
  gc = XCreateGC(display, window, 0, NULL);  
  XSetFunction(display, gc, GXcopy);
  XSetForeground(display, gc, foreground);
  XSetBackground(display, gc, background);
	
  pixmap = XCreatePixmap(display, window, width, height, depth);
  x_clear();
	
  x_init_colors();
	
  reset=TRUE;
  initialize_simulator(reset); // initializes world boundaries,
  // mobile_base, eyes[2], arms[2], and 
  // Roger interface structure
	
  simulate_base(&mobile_base);
  //simulate_arm(mobile_base.wTb, arms);
  simulate_eyes(mobile_base.wTb, eyes);
  simulate_object(&object);
	printf("Test 6 \n");
	
  make_images();
    	printf("Test 7 \n");

  draw_all();
	    	printf("Test 8 \n");

  XtAppMainLoop(app_con);
        	    	printf("Test 9 \n");

}

x_init_colors() 
{
  int i;
	
  //  printf("initializing grey scale colors..."); fflush(stdout);
  for (i = 0; i <= 100; i++) { // 0 => black; 100 => white
    sprintf(world_colors[i].name, "grey%d", i);
    world_colors[i].display_color = 
      XkwParseColor(display, world_colors[i].name);
    world_colors[i].red = (int) i*2.55;
    world_colors[i].green = (int) i*2.55;
    world_colors[i].blue = (int) i*2.55;
  }
  strcpy(world_colors[101].name, "dark red");
  world_colors[101].display_color = XkwParseColor(display, "dark red");
  world_colors[101].red = 139;
  world_colors[101].green = 0;
  world_colors[101].blue = 0;
	
  strcpy(world_colors[102].name, "red");
  world_colors[102].display_color = XkwParseColor(display, "red");
  world_colors[102].red = 255;
  world_colors[102].green = 0;
  world_colors[102].blue = 0;
	
  strcpy(world_colors[103].name, "hot pink");
  world_colors[103].display_color = XkwParseColor(display, "hot pink");
  world_colors[103].red = 255;
  world_colors[103].green = 105;
  world_colors[103].blue = 180;
	
  strcpy(world_colors[104].name, "navy");
  world_colors[104].display_color = XkwParseColor(display, "navy");
  world_colors[104].red = 65;
  world_colors[104].green = 105;
  world_colors[104].blue = 225;
  
  strcpy(world_colors[105].name, "blue");
  world_colors[105].display_color = XkwParseColor(display, "blue");
  world_colors[105].red = 0;
  world_colors[105].green = 0;
  world_colors[105].blue = 255;
	
  strcpy(world_colors[106].name, "light sky blue");
  world_colors[106].display_color = XkwParseColor(display, "light sky blue");
  world_colors[106].red = 250;
  world_colors[106].green = 128;
  world_colors[106].blue = 114;
	
  strcpy(world_colors[107].name, "dark green");
  world_colors[107].display_color = XkwParseColor(display, "dark green");
  world_colors[107].red = 244;
  world_colors[107].green = 164;
  world_colors[107].blue = 96;
	
  strcpy(world_colors[108].name, "green");
  world_colors[108].display_color = XkwParseColor(display, "green");
  world_colors[108].red = 0;
  world_colors[108].green = 255;
  world_colors[108].blue = 0;
	
  strcpy(world_colors[109].name, "light green");
  world_colors[109].display_color = XkwParseColor(display, "light green");
  world_colors[109].red = 46;
  world_colors[109].green = 139;
  world_colors[109].blue = 87;
	
  strcpy(world_colors[110].name, "gold");
  world_colors[110].display_color = XkwParseColor(display, "gold");
  world_colors[110].red = 160;
  world_colors[110].green = 82;
  world_colors[110].blue = 45;
	
  strcpy(world_colors[111].name, "yellow");
  world_colors[111].display_color = XkwParseColor(display, "yellow");
  world_colors[111].red = 255;
  world_colors[111].green = 255;
  world_colors[111].blue = 0;
	
  strcpy(world_colors[112].name, "light goldenrod");
  world_colors[112].display_color = XkwParseColor(display, "light goldenrod");
  world_colors[112].red = 192;
  world_colors[112].green = 192;
  world_colors[112].blue = 192;
}

initialize_simulator(rst) 
int rst;
{
  int i,j,k,l;
	
  if (rst) {
    /************************************************************************/
    // MOBILE BASE
    for (i=0;i<4;++i){
      for (j=0;j<4;++j) {
	mobile_base.wTb[i][j] = mobile_base_home.wTb[i][j];
      }
    }
    mobile_base.x = mobile_base_home.x;
    mobile_base.x_dot = mobile_base_home.x_dot;
    mobile_base.y = mobile_base_home.y;
    mobile_base.y_dot = mobile_base_home.y_dot;
    mobile_base.theta = mobile_base_home.theta;
    mobile_base.theta_dot = mobile_base_home.theta_dot;
    for (i=0;i<2;++i) {
      mobile_base.wheel_torque[i] = mobile_base_home.wheel_torque[i];
    }
    mobile_base.contact_theta = mobile_base_home.contact_theta;
    for (i=0;i<2;++i) {
      mobile_base.extForce[i] = mobile_base_home.extForce[i];
    }
		
    /************************************************************************/
    // LEFT AND RIGHT EYE
    for (i = 0; i < NPIXELS; ++i) {
      eyes[LEFT].image[i] = eyes[RIGHT].image[i] = 99;
    }
		
    // initialize LEFT eye
    eyes[LEFT].position[X] = 0.0;
    eyes[LEFT].position[Y] = BASELINE;
    eyes[LEFT].theta = 0.0;
    eyes[LEFT].theta_dot = 0.0;
    eyes[LEFT].torque = 0.0;
    
    // RIGHT eye
    eyes[RIGHT].position[X] = 0.0;
    eyes[RIGHT].position[Y] = -BASELINE;
    eyes[RIGHT].theta = 0.0;
    eyes[RIGHT].theta_dot = 0.0;
    eyes[RIGHT].torque = 0.0;
		
    /************************************************************************/
    // LEFT AND RIGHT ARM
    /*
    for (i=0;i<NARMS;++i) {
      for (j=0;j<NARM_FRAMES;++j) {
	for (k=0;k<4;++k) {
	  for (l=0;l<4;++l) {
	    arms[i][j].iTj[k][l] = arms_home[i][j].iTj[k][l];
	  }
	}
	arms[i][j].dof_type = arms_home[i][j].dof_type;
	arms[i][j].axis = arms_home[i][j].axis;
	arms[i][j].theta = arms_home[i][j].theta;
	arms[i][j].theta_dot = arms_home[i][j].theta_dot;
	arms[i][j].torque = arms_home[i][j].torque;
	arms[i][j].extForce[0] = arms_home[i][j].extForce[0];
	arms[i][j].extForce[1] = arms_home[i][j].extForce[1];
      }
    }
		*/
    /************************************************************************/
    // INITIALIZE THE WORLD GEOMETRY
    for (i = 0; i < NYBINS; ++i) {   // rows
      for (j = 0; j < NXBINS; ++j) {   // cols
      
	Roger.world_map.occupancy_map[i][j] = FREESPACE;
	Roger.world_map.potential_map[i][j] = 1.0;
	//Roger.arm_map[LEFT].occupancy_map[i][j] = FREESPACE;
	//Roger.arm_map[LEFT].potential_map[i][j] = 1.0;
	//Roger.arm_map[RIGHT].occupancy_map[i][j] = FREESPACE;
	//Roger.arm_map[RIGHT].potential_map[i][j] = 1.0;
	
	// left and right walls
	if ((j <= 0) || (j >= (NXBINS - 1))) {
	  Roger.world_map.occupancy_map[i][j] = OBSTACLE;
	  Roger.world_map.potential_map[i][j] = 1.0;
	  Roger.world_map.color_map[i][j] = LIGHTGREEN;
	}
	// top and bottom walls
	if ((i <= 0) || (i >= (NYBINS - 1))) {
	  Roger.world_map.occupancy_map[i][j] = OBSTACLE;
	  Roger.world_map.potential_map[i][j] = 1.0;
	  Roger.world_map.color_map[i][j] = DARKBLUE;
	}
      }
    }
    printf("Hello Test Finished Initializing World Geometry\n");
		
    //     // ***** the three room geometry *****
    //     // add room partitions to Cartesian map - horizonal wall
    //     for (j=(NBINS/2-NBINS/3); j < (NBINS/2+NBINS/3); ++j) {
    //        for (i=(NBINS/2-1); i<(NBINS/2+1); ++i) {
    //          Roger.world_map.occupancy_map[i][j] = OBSTACLE;
    //          Roger.world_map.potential_map[i][j] = 1.0;
    //          if (rst) add_bin_bumper(i,j);
    //        }
    //      }
    //	// vertical wall
    //      for (j=(NBINS/2-1); j<(NBINS/2+1); ++j) {
    //        for (i=(NBINS/2+1); i<(NBINS); ++i) {
    //          Roger.world_map.occupancy_map[i][j] = OBSTACLE;
    //          Roger.world_map.potential_map[i][j] = 1.0;
    //          if (rst) add_bin_bumper(i,j);
    //        }
    //      }
    // // **********************************************
    //  // ***** the soccer goal geometry *****
    //  i=56;
    //  for (j=0; j<22; ++j) {
    //    Roger.world_map.occupancy_map[i][j] = 
    //      Roger.world_map.occupancy_map[i][j+42] = OBSTACLE;
    //    Roger.world_map.potential_map[i][j] = 
    //      Roger.world_map.potential_map[i][j+42] = 1.0;
    //    Roger.world_map.color_map[i][j] = 
    //      Roger.world_map.color_map[i][j+42] = obstacle_color[4];
    //
    //  }
    //
    //  for (i=56;i<NBINS;++i) {
    //    Roger.world_map.occupancy_map[i][21] = 
    //      Roger.world_map.occupancy_map[i][42] = OBSTACLE;
    //    Roger.world_map.potential_map[i][21] = 
    //      Roger.world_map.potential_map[i][42] = 1.0;
    //    Roger.world_map.color_map[i][21] = 
    //      Roger.world_map.color_map[i][42] = obstacle_color[4];
    //  }
    //  // ****************************************************
		
    // in the new code organization, this is the user responsibility
    //  if (ARM_CSPACE_MAP) update_cspace_arms(&Roger);
    //  //  sor(&Roger);
    
    /************************************************************************/
    // initialize the volatile elements (afferents and efferents) 
    // of the applications interface for user control code
    write_interface(rst);
  }
}

// #ifndef MAX_RAND
// #define MAX_RAND   2147483647.0
// #endif

#define R_GOAL 0.45

initialize_random_object()
{
  // ORIGINAL (SINGLE) OBJECT
  object.mass = 0.2;
  // srand((unsigned int)(time(NULL)));
  double N1 = ((double)(rand() % 1000 + 1))/1000.0;
  double N2 = ((double)(rand() % 1000 + 1))/1000.0;
  
  double dx = MAX_X - MIN_X - 2.0*XDELTA - 2.0*R_OBJ;
  double dy = MAX_Y - MIN_Y - 2.0*YDELTA - 2.0*R_OBJ - R_GOAL;
	
  object.position[X] = (MIN_X + XDELTA + R_OBJ) + N1*dx;
  object.position[Y] = (MIN_Y + YDELTA + R_OBJ + R_GOAL) + N2*dy;
	
  // printf("%lf %lf %lf %lf\n", 
  //        N1, N2, object.position[X], object.position[Y]);
	
  object.velocity[X] = object.velocity[Y] = 0.0;
  object.extForce[X] = object.extForce[Y] = 0.0;
}

place_object(x,y)
double x,y;
{
  object.mass = 0.2;
  object.position[X] = x;
  object.position[Y] = y;
  object.velocity[X] = object.velocity[Y] = 0.0;
  object.extForce[X] = object.extForce[Y] = 0.0;
}

void x_start_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  if (!timer) {
    XkwSetWidgetLabel(start_w, "Stop");
    timer = XtAppAddTimeOut(app_con, TIMER_UPDATE, x_timer_proc,
			    (XtPointer) NULL);
    //    initialize_random_object();
  } 
  else {
    XkwSetWidgetLabel(start_w, "Start");
    XtRemoveTimeOut(timer);
    timer = 0;
  }
}

void x_params_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  switch (Roger.control_mode) {
  case PROJECT3: project3_enter_params(); break;
  case PROJECT4: project4_enter_params(); break;
  case PROJECT5: project5_enter_params(); break;
  case PROJECT6: project6_enter_params(); break;
  default:
    //called general enter_params() in UserIO.c
    enter_params();
  }
}
//remove / edit?
void x_input_mode_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  Roger.input_mode = change_input_mode();

  switch (Roger.input_mode) {
  case JOINT_ANGLE_INPUT: 
    XkwSetWidgetLabel(input_mode_w, "Input: Joint angles"); break;
  case BASE_GOAL_INPUT:
    XkwSetWidgetLabel(input_mode_w, "Input: Base goal"); break;
  case ARM_GOAL_INPUT: 
    XkwSetWidgetLabel(input_mode_w, "Input: Arm goals"); break;
  case BALL_INPUT:
    XkwSetWidgetLabel(input_mode_w, "Input: Ball position"); break;
  case MAP_INPUT:
    XkwSetWidgetLabel(input_mode_w, "Input: Map Editor"); break;
  default: break;
  }	
}

int change_input_mode()
{
  static int input_mode;

  input_mode = (input_mode + 1) % N_INPUT_MODES;
  //init_input_flag = TRUE;
  return (input_mode);
}

void x_control_mode_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  Roger.control_mode = change_control_mode();
	
  switch (Roger.control_mode) {
  case TELEOPERATION:
    XkwSetWidgetLabel(control_mode_w, "Teleoperation"); break;
  case PROJECT3:
    XkwSetWidgetLabel(control_mode_w, "User Project 3"); break;
  case PROJECT4:
    XkwSetWidgetLabel(control_mode_w, "User Project 4"); break;
  case PROJECT5:
    XkwSetWidgetLabel(control_mode_w, "User Project 5"); break;
  case PROJECT6:
    XkwSetWidgetLabel(control_mode_w, "User Project 6"); break;
  default: break;
  }
  //call init here makes it independent of timer running
  initialize_control_mode(&Roger);
}

int change_control_mode()
{
  static int control_mode;

  control_mode = (control_mode + 1) % N_CONTROL_MODES;
  init_control_flag = TRUE;
  return (control_mode);
}

void x_quit_proc(w, client_data, call_data)
Widget w;
XtPointer call_data, client_data;
{
  XFreePixmap(display, pixmap);
  XFreeGC(display, gc);
  XtDestroyApplicationContext(app_con);
  exit(0);
}

void x_canvas_proc(w, client_data, call_data)
Widget w;
XtPointer call_data, client_data; 
{
  XEvent *event = ((XEvent *) call_data);
  char text[10];
  KeySym key;
  double x, y, theta1, theta2;
  int i, j, xbin, ybin;
  int c;
  static Dimension nwidth = WIDTH, nheight = HEIGHT;
  void x_expose(), x_clear();
	
  switch (event->type) {
  case ConfigureNotify:
    nwidth = event->xconfigure.width;
    nheight = event->xconfigure.height;
    break;
  case Expose:
    if (nwidth == width && nheight == height)
      x_expose();
    else {
      /* width = nwidth; height = nheight; */
      x_expose();
    }
    break;
			
  case ButtonPress:
    Roger.button_reference[X] = D2WX(zoom,event->xbutton.x);
    Roger.button_reference[Y] = D2WY(zoom,event->xbutton.y);
			
    Roger.button_event = event->xbutton.button;
    if (event->xbutton.button == LEFT_BUTTON) { }
    break;
			
  case ButtonRelease:
    break;
			
  case KeyPress:
    c = XLookupString((XKeyEvent *) event, text, 10, &key, 0);
    if (c == 1)
      switch (text[0]) {
      case 'h':
	break;
      case 'c':
	x_clear();
	x_expose();
	break;
      case 'q':
	x_quit_proc(w, client_data, call_data);
      }
  }
}

void x_draw_line(color, start_x, start_y, end_x, end_y)
int color;
double start_x, start_y, end_x, end_y; 
{
  XSetForeground(display, gc, world_colors[color].display_color);
  XDrawLine(display, pixmap, gc,  W2DX(zoom, start_x), W2DY(zoom, start_y), 
	    W2DX(zoom, end_x), W2DY(zoom, end_y));
}

/*
void x_draw_circle(color, center_x, center_y, radius, fill)
int color, fill;
double center_x, center_y, radius; 
{
  XSetForeground(display, gc, world_colors[color].display_color);
  draw_circle(W2DX(zoom,center_x),W2DY(zoom,center_y),W2DR(zoom,radius),fill);
}
*/

void x_expose()
{
  XCopyArea(display, pixmap, window, gc, 0, 0, width, height, 0, 0);
}

void x_clear()
{
  XSetForeground(display, gc, background);
  XFillRectangle(display, pixmap, gc, 0, 0, width, height);
}

//#define STEP         0.01
#define STREAM_SPACE 0

void x_visualize_proc(w,client_data,call_data)
{
  static int draw_visual = FALSE;
  void draw_roger(), draw_object(), draw_frames(), draw_history();

  if (draw_visual == TRUE) {
    draw_visual = FALSE;
    XkwSetWidgetLabel(start_w, "Stop");
    timer = XtAppAddTimeOut(app_con, TIMER_UPDATE, x_timer_proc,
			    (XtPointer) NULL);
    return;
  }
  else {
    if (timer) {
      XkwSetWidgetLabel(start_w, "Start");
      XtRemoveTimeOut(timer);
    }
    timer = 0;
    draw_visual = TRUE;
      
    switch (Roger.control_mode) {
    case PROJECT3:
      project3_visualize(&Roger);
      break;
    case PROJECT4:
      project4_visualize(&Roger);
      break;
    case PROJECT5:
      project5_visualize(&Roger);
      break;
    case PROJECT6:
      project6_visualize(&Roger);
      break;
    default:
      break;
    }

    draw_object(object);
    draw_roger(mobile_base, arms, eyes);
    draw_frames();
      
    if (HISTORY) {
      draw_history();
    }
    x_expose();
  }
}

void mark_used(ii,jj,aux)
int ii, jj;
int aux[NYBINS][NXBINS];
{
  int j,k;
  double dist;
	
  for (j=-STREAM_SPACE; j<=STREAM_SPACE; ++j) {
    for (k=-STREAM_SPACE; k<=STREAM_SPACE; ++k) {
      dist = sqrt(SQR((double)j) + SQR((double)k));
      if ((dist < (2.0*STREAM_SPACE + 1.0)) &&
	  ((ii+j) >= 0) && ((ii+j) < NYBINS) &&
	  ((jj+k) >= 0) && ((jj+k) < NXBINS))
	aux[ii+j][jj+k] = TRUE;
    }
  }
    printf("Test 5 \n");

}

void x_timer_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  int i, j, reset;
  static int render = RENDER_RATE, servo = SERVO_RATE;
  void goal_detection();
  //  static double simtime=0.0;
	
  if (servo++ == SERVO_RATE) {
		
    reset=FALSE;
    write_interface(reset); // reset eliminates user goals and obstacles
    // from occupancy grids
		
    control_roger(&Roger, simtime);
		
    read_interface();
    servo = 1;
  }
	
  /* writes collision forces into respective data structures */
  /* obstacle data structure is global */
  compute_external_forces(&mobile_base, arms, &object);
	
  simulate_base(&mobile_base);
  //simulate_arm(mobile_base.wTb, arms);
  simulate_eyes(mobile_base.wTb, eyes);
  simulate_object(&object);
	
  if (render++ == RENDER_RATE) {
    if ((HISTORY) && (history_ptr < MAX_HISTORY)) {
     // history[history_ptr].arm_pos[LEFT][0] = arms[LEFT][1].theta;
     // history[history_ptr].arm_pos[LEFT][1] = arms[LEFT][2].theta;
      
     // history[history_ptr].arm_pos[RIGHT][0] = arms[RIGHT][1].theta;
     // history[history_ptr].arm_pos[RIGHT][1] = arms[RIGHT][2].theta;
			
      history[history_ptr].base_pos[0] = mobile_base.x;
      history[history_ptr].base_pos[1] = mobile_base.y;
      history[history_ptr].base_pos[2] = mobile_base.theta;
			
      ++history_ptr;
    }
		
    make_images();
    draw_all();
    render = 1;
  }
  simtime += DT;
  
  timer = XtAppAddTimeOut(app_con, TIMER_UPDATE, x_timer_proc,
			  (XtPointer) NULL);
  // goal_detection(); // for shooting goals in soccer app
}

write_interface(reset)
int reset;
{
  int i,j;
	
  // pass in afferents (read only)
  Roger.eye_theta[0] = eyes[0].theta;
  Roger.eye_theta_dot[0] = eyes[0].theta_dot;
  Roger.eye_theta[1] = eyes[1].theta;
  Roger.eye_theta_dot[1] = eyes[1].theta_dot;
  for (i=0;i<NPIXELS;++i) {
      Roger.image[LEFT][i][RED_CHANNEL] = 
	world_colors[eyes[LEFT].image[i]].red;
      Roger.image[LEFT][i][GREEN_CHANNEL]=
	world_colors[eyes[LEFT].image[i]].green;
      Roger.image[LEFT][i][BLUE_CHANNEL] = 
	world_colors[eyes[LEFT].image[i]].blue;
      Roger.image[RIGHT][i][RED_CHANNEL] = 
	world_colors[eyes[RIGHT].image[i]].red;
      Roger.image[RIGHT][i][GREEN_CHANNEL] = 
	world_colors[eyes[RIGHT].image[i]].green;
      Roger.image[RIGHT][i][BLUE_CHANNEL] = 
	world_colors[eyes[RIGHT].image[i]].blue;
  }
  /*
  Roger.arm_theta[0][0] = arms[0][1].theta;
  Roger.arm_theta[0][1] = arms[0][2].theta;
  Roger.arm_theta[1][0] = arms[1][1].theta;
  Roger.arm_theta[1][1] = arms[1][2].theta;
  Roger.arm_theta_dot[0][0] = arms[0][1].theta_dot;
  Roger.arm_theta_dot[0][1] = arms[0][2].theta_dot;
  Roger.arm_theta_dot[1][0] = arms[1][1].theta_dot;
  Roger.arm_theta_dot[1][1] = arms[1][2].theta_dot;
  Roger.ext_force[0][X] = arms[0][NARM_FRAMES - 1].extForce[X];
  Roger.ext_force[0][Y] = arms[0][NARM_FRAMES - 1].extForce[Y];
  Roger.ext_force[1][X] = arms[1][NARM_FRAMES - 1].extForce[X];
  Roger.ext_force[1][Y] = arms[1][NARM_FRAMES - 1].extForce[Y];
	*/
  Roger.base_position[0] = mobile_base.x;
  Roger.base_position[1] = mobile_base.y;
  Roger.base_position[2] = mobile_base.theta;
  Roger.base_velocity[0] = mobile_base.x_dot;
  Roger.base_velocity[1] = mobile_base.y_dot;
  Roger.base_velocity[2] = mobile_base.theta_dot;
	
  // zero efferents (write only)
  Roger.eye_torque[0] = Roger.eye_torque[1] = 0.0;
 // Roger.arm_torque[0][0]=Roger.arm_torque[0][1]=Roger.arm_torque[1][0]=
  //  Roger.arm_torque[1][1] = 0.0;
  Roger.wheel_torque[0] = Roger.wheel_torque[1] = 0.0;
}  

read_interface()
{
  // pass back torques (write only)
  /*
  arms[0][1].torque = Roger.arm_torque[0][0];
  arms[1][1].torque = Roger.arm_torque[1][0];
  arms[0][2].torque = Roger.arm_torque[0][1];
  arms[1][2].torque = Roger.arm_torque[1][1];
	*/
  eyes[0].torque = Roger.eye_torque[0];
  eyes[1].torque = Roger.eye_torque[1];
	
  mobile_base.wheel_torque[0] = Roger.wheel_torque[0];
  mobile_base.wheel_torque[1] = Roger.wheel_torque[1];
}

#define NBODY 4 // single ball, two hands, roger's body

compute_external_forces(base, arms, obj)
Base * base;
Arm arms[NARMS][NARM_FRAMES];
Obj * obj; 
{
  int i, j, row, col;
	
  double x, y, p_b[4], p_w[4], v_b[4], v_w[4];
  double r[NBODY][2], mag, v[NBODY][2], R[NBODY], J[2][2], dr[2];
  double vi_proj, vj_proj, force;
  double F[NBODY][2], dij, sum[2];
  
  // initialize forces on dynamic bodies: base, hands, object
  base->extForce[X] = base->extForce[Y] = 0.0;
  obj->extForce[X] = obj->extForce[Y] = 0.0;
 /*
  for (i = 0; i < NARMS; ++i) {
    arms[i][NARM_FRAMES - 1].extForce[X] = 0.0;
    arms[i][NARM_FRAMES - 1].extForce[Y] = 0.0;
  }
	*/
  // define the position and radius of the NBODY dynamic bodies
  // BASE #0
  r[0][X] = base->x;
  r[0][Y] = base->y;
  v[0][X] = base->x_dot;
  v[0][Y] = base->y_dot;
  R[0] = R_BASE;
	
  // ARM #1
  /*
  sim_fwd_kinematics(LEFT, arms[LEFT][1].theta, arms[LEFT][2].theta, &x, &y);
  p_b[0] = x; p_b[1] = y; p_b[2] = 0.0; p_b[3] = 1.0;
  SIMmatXvec(base->wTb, p_b, p_w);
  r[1][X] = p_w[X];
  r[1][Y] = p_w[Y];
	*/
  // hand velocity relative to base written in base coordinates
  /*
  sim_arm_Jacobian(arms[LEFT][1].theta, arms[LEFT][2].theta, J);
  v_b[X] = J[0][0]*arms[LEFT][1].theta_dot + J[0][1]*arms[LEFT][2].theta_dot -
    R_BASE*base->theta_dot;
  v_b[Y] = J[1][0]*arms[LEFT][1].theta_dot + J[1][1]*arms[LEFT][2].theta_dot;
  v_b[2] = 0.0;
  v_b[3] = 0.0; // homogeneous vector
	
  SIMmatXvec(base->wTb, v_b, v_w);
  v[1][X] = base->x_dot + v_w[X];
  v[1][Y] = base->y_dot + v_w[Y];
  R[1] = R_TACTILE;
	*/
  // ARM #2
  
  /*
  sim_fwd_kinematics(RIGHT, arms[RIGHT][1].theta, arms[RIGHT][2].theta, &x,&y);
  p_b[0] = x; p_b[1] = y; p_b[2] = 0.0; p_b[3] = 1.0;
  SIMmatXvec(base->wTb, p_b, p_w);
  r[2][X] = p_w[X];
  r[2][Y] = p_w[Y];
	
  // hand velocity relative to base written in base coordinates
  sim_arm_Jacobian(arms[RIGHT][1].theta, arms[RIGHT][2].theta, J);
  v_b[X] = J[0][0]*arms[RIGHT][1].theta_dot + J[0][1]*arms[RIGHT][2].theta_dot
    + R_BASE*base->theta_dot;
  v_b[Y] = J[1][0]*arms[RIGHT][1].theta_dot + J[1][1]*arms[RIGHT][2].theta_dot;
  v_b[2] = 0.0;
  v_b[3] = 0.0; // homogeneous vector
	
  SIMmatXvec(base->wTb, v_b, v_w);
  v[2][X] = base->x_dot + v_w[X];
  v[2][Y] = base->y_dot + v_w[Y];
  R[2] = R_TACTILE;
	*/
  // OBJ #3
  r[3][X] = obj->position[X];
  r[3][Y] = obj->position[Y];
  v[3][X] = obj->velocity[X];
  v[3][Y] = obj->velocity[Y];
  R[3] = R_OBJ;
	
  //  sum multi-body collision forces on the every body:
  //     base #0, arm #1 arm #2, object #3
  for (i = 0; i < NBODY; ++i) {
    sum[X] = sum[Y] = 0.0;
    for (j = 0; j < NBODY; ++j) {
      if (j != i) {
	dr[X] = r[i][X] - r[j][X];
	dr[Y] = r[i][Y] - r[j][Y];
	mag = sqrt(SQR(dr[X]) + SQR(dr[Y]));
	
	dij = MAX(0.0, (R[i]+R[j]-mag));
				
	if (dij > 0.0) { // body i is in compression
	  // compute dij_dot
	  vi_proj = v[i][X]*dr[X]/mag + v[i][Y]*dr[Y]/mag;
	  vj_proj = v[j][X]*dr[X]/mag + v[j][Y]*dr[Y]/mag;
	  
	  force = K_COLLIDE*dij + B_COLLIDE*(MAX(0.0, (vi_proj - vj_proj)));
	  // force = K_COLLIDE*dij - B_COLLIDE*(vi_proj - vj_proj);
	  sum[X] += force * dr[X] / mag;
	  sum[Y] += force * dr[Y] / mag;
	}
      }
    }
    for (row=0; row<NYBINS; ++row) {
      for (col=0; col<NXBINS; ++col) {
	if (Roger.world_map.occupancy_map[row][col] == OBSTACLE) {
	  dr[X] = r[i][X] - (MIN_X + (col+0.5)*XDELTA);
	  dr[Y] = r[i][Y] - (MAX_Y - (row+0.5)*YDELTA);
	  mag = sqrt(SQR(dr[X]) + SQR(dr[Y]));
					
	  dij = MAX(0.0, (R[i]+R_OBSTACLE-mag));
					
	  if (dij > 0.0) {
	    //	base->contact_theta = theta = atan2(ry, rx);
	    vi_proj = v[i][X]*dr[X]/mag + v[i][Y]*dr[Y]/mag;
	    force = K_COLLIDE*dij - B_COLLIDE*(vi_proj);
	    sum[X] += force * dr[X] / mag;
	    sum[Y] += force * dr[Y] / mag;
	  }
	}
      }
    }
        printf("Hello Test 2 \n");

    F[i][X] = sum[X];
    F[i][Y] = sum[Y];
  }
	
  // BASE
  base->extForce[X] = F[0][X];
  base->extForce[Y] = F[0][Y];
	
  // ARM #1
 // arms[LEFT][NARM_FRAMES - 1].extForce[X] = F[1][X];
 // arms[LEFT][NARM_FRAMES - 1].extForce[Y] = F[1][Y];
	
  // ARM #2
  //arms[RIGHT][NARM_FRAMES - 1].extForce[X] = F[2][X];
  //arms[RIGHT][NARM_FRAMES - 1].extForce[Y] = F[2][Y];
	
  // OBJ
  obj->extForce[X] = F[3][X];
  obj->extForce[Y] = F[3][Y];
}

//void compute_reflection(Obj * obj)
//{
//  static int bump_flag = 0;
//  double gridSize = 2*MAX_X/NBINS;
//  int gx = (obj->position[X]-MIN_X)/gridSize;
//  int gy = (MAX_Y-obj->position[Y])/gridSize;
//  int x,y,xmin=0,ymin=0;
//  double dmin = 100000;
//  if (Roger.world_map.occupancy_map[gy][gx] == OBSTACLE)
//    printf("INSIDE RED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
//  
//  if(Roger.world_map.occupancy_map[gy][gx] == OBSTACLE) {
//    if (bump_flag == 0) {
//      int window = 40;
//      for(x=gx-window; x<gx+window;x++) {
//	for(y=gy-window;y<gy+window;y++) {
//	  if(y<0 || y>NBINS || x<0 || x>NBINS)
//	    continue;
//	  if(Roger.world_map.occupancy_map[gy][gx] != OBSTACLE)
//	    continue;
//		
//	  double dist = sqrt((gx-x) * (gx-x) + (gy-y) * (gy-y));
//	  if(dist < dmin) {
//	    dmin = dist;
//	    xmin = x;
//	    ymin = y;
//	    printf("============dist =%f===========\n",dist);
//	    printf("============dmin =%f===========\n",dmin);
//	    printf("xmin=%d,ymin = %d\n",xmin,ymin);
//	  }
//	}
//      }
//      double n_alpha = atan2(gy-ymin,gx-xmin);
//      double v_theta = atan2(obj->velocity[Y],obj->velocity[X]);
//      double out_beta = 2*n_alpha - v_theta + 3.1415926;
//      double v_mag = sqrt((obj->velocity[X]) * obj->velocity[X] + 
//			  obj->velocity[Y] *obj->velocity[Y]);
//      printf("============n_alpha =%f===========\n",n_alpha);
//      obj->velocity[X] = v_mag * cos(out_beta);
//      obj->velocity[Y] = v_mag * sin(out_beta);
//    }
//    bump_flag = 1;
//  }
//  if(Roger.world_map.occupancy_map[gy][gx] == FREESPACE) {
//    bump_flag = 0;
//  }
//}

#define NSAMPLES 10

// Roger is now being treated as a global variable and passed in as 
// a local argument

update_cspace_arms(roger)
Robot * roger;
{ 
/*
  int i,j, arm, xbin, ybin, dl, link, collide;
  double t1, t2, x, y, dx[3], dy[3];
  double wTb[4][4], bTw[4][4], b[4], w[4];
  
  // MAP CARTESIAN OBSTACLES TO OBSTACLES IN ARM CSPACES
  construct_wTb(roger->base_position, wTb);
	
  for (arm=0; arm < NARMS; ++arm) {
    for (i=0;i<NBINS;++i) {
      t2 = T2_MAX - (i+0.5)*TDELTA;
      for (j=0;j<NBINS;++j) {
	t1 = T1_MIN + (j+0.5)*TDELTA;
	
	// default status
	roger->arm_map[arm].occupancy_map[i][j] = FREESPACE;
	collide=FALSE;
	if ((j <= 0) || (j >= (NBINS - 1))) {
	  Roger.arm_map[arm].occupancy_map[i][j] = OBSTACLE;
	  Roger.arm_map[arm].potential_map[i][j] = 1.0;
	}
	else if ((i <= 0) || (i >= (NBINS - 1))) {
	  Roger.arm_map[arm].occupancy_map[i][j] = OBSTACLE;
	  Roger.arm_map[arm].potential_map[i][j] = 1.0;
	}
	else {
	  // in base coordinates
	  if (arm==LEFT) {x = 0.0; y = ARM_OFFSET;}
	  else  {x = 0.0; y = -ARM_OFFSET;}
	  dx[1] = LARM_1*cos(t1)/NSAMPLES;
	  dx[2] = LARM_2*cos(t1+t2)/NSAMPLES;
	  dy[1] = LARM_1*sin(t1)/NSAMPLES;
	  dy[2] = LARM_2*sin(t1+t2)/NSAMPLES;
					
	  // walk the length of the manipulator in Cartesian space
	  for (link=1; ((link <= 2) && (!collide)); ++link) {
	    for (dl=0; (dl<NSAMPLES); ++dl) {
	      x += dx[link]; y += dy[link];
	      
	      b[0]=x; b[1]=y; b[2]=0.0; b[3]=1.0;
	      
	      SIMmatXvec(wTb, b, w);
							
	      xbin = (int)((w[0]-MIN_X)/XDELTA);
	      ybin = (int)((MAX_Y-w[1])/YDELTA);

	      if (roger->world_map.occupancy_map[ybin][xbin] == OBSTACLE)
		collide = TRUE;
	    }
	  }
	  if (collide) {
	    roger->arm_map[arm].occupancy_map[i][j] = OBSTACLE;
	    roger->arm_map[arm].potential_map[i][j] = 1.0;
	  }
	}
	//map goals
	sim_fwd_kinematics(arm,t1,t2,&x,&y);
	b[0]=x; b[1]=y; b[2]=0.0; b[3]=1.0;
	SIMmatXvec(wTb, b, w);
				
	xbin = (int)((w[0]-MIN_X)/XDELTA);
	ybin = (int)((MAX_Y-w[1])/YDELTA);
	if (roger->world_map.occupancy_map[ybin][xbin] == GOAL) {
	  roger->arm_map[arm].occupancy_map[i][j] = GOAL;
	  roger->arm_map[arm].potential_map[i][j] = 0.0;
	}
      }
    }
  }

*/
}

//update_cspace_base(roger)
//Robot * roger;
//{ 
//  int i,j, xbin, ybin, dl, link, collide;
//  double t1, t2, x, y, dx[3], dy[3];
//  double wTb[4][4], bTw[4][4], b[4], w[4];
//	
//  // MAP CARTESIAN OBSTACLES TO DIRICHLET CONSTRAINTS IN CSPACE
//  // for each CSpace bin:
//  //	printf("\n Updating CSPACE for BASE");
//  //construct_wTb(roger->base_position, wTb);
//	
//  for (i=0;i<NBINS;++i) {
//    for (j=0;j<NBINS;++j) {
//      cspace[BASE].boundary_map[i][j] = FREESPACE;
//      if (CartesianMap[i][j] == DIRICHLET) {
//	  cspace[BASE].boundary_map[i][j] = DIRICHLET;
//	  cspace[BASE].potential_map[i][j] = 1.0;
//	}
//      if (CartesianMap[i][j] == GOAL) {
//	  cspace[BASE].boundary_map[i][j] = GOAL;
//	  cspace[BASE].potential_map[i][j] = 0.0;
//	}
//      cspace[BASE].boundary_map[0][j] = 
//	cspace[BASE].boundary_map[(NBINS-1)][j] = DIRICHLET;
//      cspace[BASE].potential_map[0][j] = 
//	   cspace[BASE].potential_map[(NBINS-1)][j] = 1.0;
//    }
//		
//    // joint angle limits:
//    // theta1: is toroidal   theta2: -3.1 rad < theta2 < +3.1 rad
//    cspace[BASE].boundary_map[i][0] = 
//       cspace[BASE].boundary_map[i][(NBINS-1)] = DIRICHLET;
//    cspace[BASE].potential_map[i][0] = 
//       cspace[BASE].potential_map[i][(NBINS-1)] = 1.0;
//  }
//  //	printf("\n Updating CSPACE for BASE");
//  fflush(stdout);
//}

draw_all() 
{
  int n;
  char buffer[64];
  void draw_potential_maps(),  draw_roger();
	//draw_object()
	
  x_clear();
		printf("Test 10 \n");

  draw_potential_maps(); 
  		printf("Test 11 \n");

  //draw_object(object);
	
  //draw_streamline()
	
  draw_roger(mobile_base,arms, eyes);
  		printf("Test 12 \n");
	
  //  draw_ellipse(manipulator(LEFT));
  //  draw_ellipse(manipulator(RIGHT));
  
  // XSetForeground(display, gc, goal_color);
  //  draw_ellipse(observation);
  //  draw_ellipse(spatial_goals[CENTROID]);
  //  draw_ellipse(spatial_goals[LEFT_EDGE]);
  //  draw_ellipse(spatial_goals[RIGHT_EDGE]);
	
  //  if (p_index==6) draw_ellipse(grasp_goal);
  //  XSetForeground(display, gc, target_color);
  //  draw_ellipse(target);
	
  n = sprintf(buffer, "total elapsed time = %6.3lf", simtime);
  XSetForeground(display, gc, foreground);
  XDrawString(display, pixmap, gc,
	      W2DX(zoom,3.0), W2DY(zoom,1.8), buffer, n);
	
  x_expose();
}

draw_circle(cu, cv, r, fill)
int cu, cv, r, fill;
{
  if (fill == NOFILL)
    XDrawArc(display, pixmap, gc, cu - r, cv - r, 2* r , 2* r , 0, 64*360);
  else
    XFillArc(display, pixmap, gc, cu-r, cv-r, 2*r, 2*r, 0, 64*360);
}

void draw_frames()
{
#define FRAME_L 0.04
#define FRAME_T 0.045
	
  XSetForeground(display, gc, foreground);
	
  // the Cartesian frame
  /* x-axis */
  XDrawLine(display, pixmap, gc, 
	    W2DX(zoom,0.0), W2DY(zoom,0.0),
	    W2DX(zoom,(FRAME_L*4.0)), W2DY(zoom,0.0));
  XDrawString(display, pixmap, gc, 
	      W2DX(zoom,FRAME_T*4.0), W2DY(zoom,0.0), "x", 1);
  
  /* y-axis */
  XDrawLine(display, pixmap, gc, W2DX(zoom,0.0), W2DY(zoom,0.0),
	    W2DX(zoom,0.0), W2DY(zoom,FRAME_L*4.0));
  XDrawString(display, pixmap, gc, 
	      W2DX(zoom,0.0), W2DY(zoom,FRAME_T*4.0), "y", 1);
	
  // the LEFT CSpace frame
  /* q1-axis */
  /*
  XDrawLine(display, pixmap, gc, T12LD(0.0), T22LD(0.0),
	    T12LD(FRAME_L*2.0*M_PI), T22LD(0.0));
  XDrawString(display, pixmap, gc, 
	      T12LD(FRAME_T*2.0*M_PI), T22LD(0.0), "q1", 2);
	
  /* q2-axis */
  /*
  XDrawLine(display, pixmap, gc, T12LD(0.0), T22LD(0.0), T12LD(0.0),
	    T22LD(FRAME_L*2.0*M_PI));
  XDrawString(display, pixmap, gc, 
	      T12LD(0.0), T22LD(FRAME_T*2.0*M_PI), "q2", 2);
	
  XDrawString(display, pixmap, gc, T12LD(-0.75), T22LD(-3.5), "left    /", 9);
  XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
  XDrawString(display, pixmap, gc, T12LD(-0.2), T22LD(-3.5), "arm", 3);
  XSetForeground(display, gc, world_colors[EYE_COLOR].display_color);
  XDrawString(display, pixmap, gc, T12LD(0.35), T22LD(-3.5), "eye", 3);
		
  XSetForeground(display, gc, foreground);
  */
  // the RIGHT CSpace frame
  /* q1-axis */
  
  /*
  XDrawLine(display, pixmap, gc, T12RD(0.0), T22RD(0.0),
	    T12RD(FRAME_L*2.0*M_PI), T22RD(0.0));
  XDrawString(display, pixmap, gc, 
	      T12RD(FRAME_T*2.0*M_PI), T22RD(0.0), "q1", 2);
	*/
  /* q2-axis */
  /*
  XDrawLine(display, pixmap, gc, T12RD(0.0), T22RD(0.0), T12RD(0.0),
	    T22RD(FRAME_L*2.0*M_PI));
  XDrawString(display, pixmap, gc, 
	      T12RD(0.0), T22RD(FRAME_T*2.0*M_PI), "q2", 2);
  
  XDrawString(display, pixmap, gc, T12RD(-0.85), T22RD(-3.5), "right    /", 10);
  XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
  XDrawString(display, pixmap, gc, T12RD(-0.15), T22RD(-3.5), "arm", 3);
  XSetForeground(display, gc, world_colors[EYE_COLOR].display_color);
  XDrawString(display, pixmap, gc, T12RD(0.4), T22RD(-3.5), "eye", 3);
	*/

#undef FRAME_L // 0.04
#undef FRAME_T // 0.045
}

void draw_frame(xform)
double xform[4][4]; {
#define FRAME_L 0.04
#define FRAME_T 0.045
	
  XSetForeground(display, gc, foreground);
	
  /* x-axis */
  XDrawLine(display, pixmap, gc, W2DX(zoom,xform[0][3]),
	    W2DY(zoom,xform[1][3]),
	    W2DX(zoom,xform[0][3]+FRAME_L*xform[0][0]),
	    W2DY(zoom,xform[1][3]+FRAME_L*xform[1][0]));
  XDrawString(display, pixmap, gc,
	      W2DX(zoom,xform[0][3]+FRAME_T*xform[0][0]),
	      W2DY(zoom,xform[1][3]+FRAME_T*xform[1][0]), "x", 1);
  
  /* y-axis */
  XDrawLine(display, pixmap, gc, W2DX(zoom,xform[0][3]),
	    W2DY(zoom,xform[1][3]),
	    W2DX(zoom,xform[0][3]+FRAME_L*xform[0][1]),
	    W2DY(zoom,xform[1][3]+FRAME_L*xform[1][1]));
  XDrawString(display, pixmap, gc,
	      W2DX(zoom,xform[0][3]+FRAME_T*xform[0][1]),
	      W2DY(zoom,xform[1][3]+FRAME_T*xform[1][1]), "y", 1);
#undef FRAME_L // 0.04
#undef FRAME_T // 0.045
}

// draw the NBINSxNBINS potential maps in their respective areas of the canvas
void draw_potential_maps() {
  int i, j, Cart_grey_index, left_arm_grey_index, right_arm_grey_index;
  double x, y, t1, t2;
  double Cart_bin_potential, left_arm_bin_potential, right_arm_bin_potential;
  int sum;	
  for (i = 0; i < NYBINS; ++i) {
    y = MAX_Y - i*YDELTA;
   // t2 = T2_MAX - i*TDELTA;
    for (j = 0; j < NXBINS; ++j) {
      x = MIN_X + j*XDELTA;
      //t1 = T1_MIN + j*TDELTA;
      // user map grey level fill
      
     // printf("%d " , i+j);

      Cart_bin_potential = Roger.world_map.potential_map[i][j];
     // left_arm_bin_potential = Roger.arm_map[LEFT].potential_map[i][j];
     // right_arm_bin_potential = Roger.arm_map[RIGHT].potential_map[i][j];
      
      // 0 <= grey indices <= 100
      
      Cart_grey_index = (int) (Cart_bin_potential * 100.0);
     // left_arm_grey_index = (int) (left_arm_bin_potential * 100.0);
     // right_arm_grey_index = (int) (right_arm_bin_potential * 100.0);
			
      // Cartesian Map
      // fill is either:
      //   a grey level depicting the user defined potential
      XSetForeground(display, gc, world_colors[Cart_grey_index].display_color);
      //   a user map perceived obstacle color, or
      if (Roger.world_map.occupancy_map[i][j] == OBSTACLE)
	XSetForeground(display, gc, 
		world_colors[Roger.world_map.color_map[i][j]].display_color);
	   else if (Roger.world_map.occupancy_map[i][j] == DILATED_OBSTACLE)
	XSetForeground(display, gc, 
		world_colors[Roger.world_map.color_map[i][j]].display_color);
      //   a user defined goal
      else if (Roger.world_map.occupancy_map[i][j] == GOAL)
	XSetForeground(display, gc, world_colors[GOAL_COLOR].display_color);
      XFillRectangle(display, pixmap, gc, W2DX(zoom,x), W2DY(zoom,y),
		     (W2DR(zoom,XDELTA) + 1), (W2DR(zoom,YDELTA) + 1));
      
      //      // each real obstacle should be outlined in the obstacle color
      //      if (real_world.occupancy_map[i][j] == OBSTACLE) {
      //	XSetForeground(display, gc, 
      //	       world_colors[real_world.color_map[i][j]].display_color);
      //	x = MIN_X + j*XDELTA; y = MAX_Y - i*YDELTA;
      //	XDrawRectangle(display, pixmap, gc, W2DX(zoom,x), W2DY(zoom,y),
      //		     (W2DR(zoom,XDELTA)), (W2DR(zoom,YDELTA)));
      //      }
			
      // Left Arm Map
      /*
      XSetForeground(display, gc, 
		     world_colors[left_arm_grey_index].display_color);
      if (Roger.arm_map[LEFT].occupancy_map[i][j] == OBSTACLE)
	XSetForeground(display, gc, world_colors[RED].display_color);
      else if (Roger.arm_map[LEFT].occupancy_map[i][j] == GOAL)
	XSetForeground(display, gc, world_colors[GOAL_COLOR].display_color);
      XFillRectangle(display, pixmap, gc, T12LD(t1), T22LD(t2),
		     (T2DR(TDELTA) + 1), (T2DR(TDELTA) + 1));
			
      // Right Arm Map
      XSetForeground(display, gc, 
		     world_colors[right_arm_grey_index].display_color);
      if (Roger.arm_map[RIGHT].occupancy_map[i][j] == OBSTACLE)
	XSetForeground(display, gc, world_colors[RED].display_color);
      else if (Roger.arm_map[RIGHT].occupancy_map[i][j] == GOAL)
	XSetForeground(display, gc, world_colors[GOAL_COLOR].display_color);
      XFillRectangle(display, pixmap, gc, T12RD(t1), T22RD(t2),
		     (T2DR(TDELTA) + 1), (T2DR(TDELTA) + 1));
		     */
    }
  }
      printf("Hello Test 3 \n");
	  fflush(stdout);
	  
  draw_boundaries();
  draw_frames();
}

draw_boundaries()
{
  /******************************************************************/
  /**  draw world                                                  **/
  XSetForeground(display, gc, foreground);
  XDrawLine(display, pixmap, gc, W2DX(zoom,MIN_X), W2DY(zoom,MAX_Y),
	    W2DX(zoom,MAX_X), W2DY(zoom,MAX_Y));
  XDrawLine(display, pixmap, gc, W2DX(zoom,MAX_X), W2DY(zoom,MAX_Y),
	    W2DX(zoom,MAX_X), W2DY(zoom,MIN_Y));
  XDrawLine(display, pixmap, gc, W2DX(zoom,MAX_X), W2DY(zoom,MIN_Y),
	    W2DX(zoom,MIN_X), W2DY(zoom,MIN_Y));
  XDrawLine(display, pixmap, gc, W2DX(zoom,MIN_X), W2DY(zoom,MIN_Y),
	    W2DX(zoom,MIN_X), W2DY(zoom,MAX_Y));
	
  /* draw LEFT boundaries */
  /*
  XSetForeground(display, gc, foreground);
  XDrawLine(display, pixmap, gc, T12LD(T1_MIN), T22LD(T2_MAX), T12LD(T1_MAX),
	    T22LD(T2_MAX));
  XDrawLine(display, pixmap, gc, T12LD(T1_MAX), T22LD(T2_MAX), T12LD(T1_MAX),
	    T22LD(T2_MIN));
  XDrawLine(display, pixmap, gc, T12LD(T1_MAX), T22LD(T2_MIN), T12LD(T1_MIN),
	    T22LD(T2_MIN));
  XDrawLine(display, pixmap, gc, T12LD(T1_MIN), T22LD(T2_MIN), T12LD(T1_MIN),
	    T22LD(T2_MAX));
	
  /* draw RIGHT boundaries */
  /*
  XSetForeground(display, gc, foreground);
  XDrawLine(display, pixmap, gc, T12RD(T1_MIN), T22RD(T2_MAX), T12RD(T1_MAX),
	    T22RD(T2_MAX));
  XDrawLine(display, pixmap, gc, T12RD(T1_MAX), T22RD(T2_MAX), T12RD(T1_MAX),
	    T22RD(T2_MIN));
  XDrawLine(display, pixmap, gc, T12RD(T1_MAX), T22RD(T2_MIN), T12RD(T1_MIN),
	    T22RD(T2_MIN));
  XDrawLine(display, pixmap, gc, T12RD(T1_MIN), T22RD(T2_MIN), T12RD(T1_MIN),
	    T22RD(T2_MAX));
	    */
}

void draw_object(obj)
Obj obj;
{
  XSetForeground(display, gc, world_colors[OBJECT_COLOR].display_color);
  draw_circle(W2DX(zoom,obj.position[X]), W2DY(zoom,obj.position[Y]),
	      W2DR(zoom,R_OBJ), FILL);
}

void draw_roger(mobile_base, arms, eyes)
Base mobile_base;
Arm arms[NARMS][NARM_FRAMES];
Eye eyes[NEYES];
{
  register i, j;
  double r_b[4], r_w[4], fhat[2];
  double theta1, theta2, mag;
  double temp0[4][4], temp1[4][4];
  XPoint rect[4];
  void draw_history();
	
  /******************************************************************/
  /* draw mobile base */
  XSetForeground(display, gc, foreground);
  draw_circle(W2DX(zoom,mobile_base.wTb[0][3]),
	      W2DY(zoom,mobile_base.wTb[1][3]), W2DR(zoom,R_BASE), NOFILL);
	
  // draw contact forces on object from body
  mag = sqrt(SQR(mobile_base.extForce[X]) + SQR(mobile_base.extForce[Y]));
	
  if (mag > 0.0) {
    fhat[X] = mobile_base.extForce[X] / mag;
    fhat[Y] = mobile_base.extForce[Y] / mag;
    
    XDrawLine(display, pixmap, gc,
	      W2DX(zoom, mobile_base.x - R_BASE*fhat[X]),
	      W2DY(zoom, mobile_base.y - R_BASE*fhat[Y]),
	      W2DX(zoom, mobile_base.x - (R_BASE+0.08)*fhat[X]),
	      W2DY(zoom, mobile_base.y - (R_BASE+0.08)*fhat[Y]));
		
    //    XDrawLine(display, pixmap, gc,
    //      W2DX(zoom, mobile_base.x + R_BASE*cos(mobile_base.contact_theta)),
    //      W2DY(zoom, mobile_base.y + R_BASE*sin(mobile_base.contact_theta)),
    //      W2DX(zoom, mobile_base.x + R_BASE*cos(mobile_base.contact_theta)
    //	   + 0.08*mobile_base.extforce[Y]/mag),
    //      W2DY(zoom, mobile_base.y + R_BASE*sin(mobile_base.contact_theta)
    //	   - 0.08*mobile_base.extforce[X]/mag));
  }
	
  //  draw_wheels();
  r_b[0] = R_BASE/2.0; r_b[1] = R_BASE+R_WHEEL; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  draw_circle(W2DX(zoom, r_w[0]), W2DY(zoom, r_w[1]), W2DR(zoom,R_WHEEL),
	      FILL);
  r_b[0] = -R_BASE/2.0; r_b[1] = R_BASE+R_WHEEL; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  draw_circle(W2DX(zoom, r_w[0]), W2DY(zoom, r_w[1]), W2DR(zoom,R_WHEEL),
	      FILL);
	
  r_b[0] = R_BASE/2.0; r_b[1] = R_BASE; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[0].x = (short) (W2DX(zoom, r_w[0]));
  rect[0].y = (short) (W2DY(zoom, r_w[1]));
	
  r_b[0] = R_BASE/2.0; r_b[1] = (R_BASE+2*R_WHEEL); r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[1].x = (short) (W2DX(zoom, r_w[0]));
  rect[1].y = (short) (W2DY(zoom, r_w[1]));
	
  r_b[0] = -R_BASE/2.0; r_b[1] = (R_BASE+2*R_WHEEL); r_b[2]=0.0; r_b[3]=1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[2].x = (short) (W2DX(zoom, r_w[0]));
  rect[2].y = (short) (W2DY(zoom, r_w[1]));
	
  r_b[0] = -R_BASE / 2.0; r_b[1] = R_BASE; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[3].x = (short) (W2DX(zoom, r_w[0]));
  rect[3].y = (short) (W2DY(zoom, r_w[1]));
	
  XFillPolygon(display, pixmap, gc, rect, 4, Convex, CoordModeOrigin);
	
  r_b[0] = R_BASE/2.0; r_b[1] = -R_BASE-R_WHEEL; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  draw_circle(W2DX(zoom, r_w[0]), W2DY(zoom, r_w[1]), W2DR(zoom,R_WHEEL),FILL);
  r_b[0] = -R_BASE/2.0; r_b[1] = -R_BASE-R_WHEEL; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  draw_circle(W2DX(zoom, r_w[0]), W2DY(zoom, r_w[1]), W2DR(zoom,R_WHEEL),FILL);
	
  r_b[0] = R_BASE/2.0; r_b[1] = -R_BASE; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[0].x = (short) (W2DX(zoom, r_w[0]));
  rect[0].y = (short) (W2DY(zoom, r_w[1]));
	
  r_b[0] = R_BASE/2.0; r_b[1] = -(R_BASE+2*R_WHEEL); r_b[2] = 0.0; r_b[3]=1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[1].x = (short) (W2DX(zoom, r_w[0]));
  rect[1].y = (short) (W2DY(zoom, r_w[1]));
	
  r_b[0] = -R_BASE/2.0; r_b[1] = -(R_BASE+2*R_WHEEL); r_b[2] = 0.0; r_b[3]=1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[2].x = (short) (W2DX(zoom, r_w[0]));
  rect[2].y = (short) (W2DY(zoom, r_w[1]));
	
  r_b[0] = -R_BASE/2.0; r_b[1] = -R_BASE; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[3].x = (short) (W2DX(zoom, r_w[0]));
  rect[3].y = (short) (W2DY(zoom, r_w[1]));
	
  XFillPolygon(display, pixmap, gc, rect, 4, Convex, CoordModeOrigin);
  /******************************************************************/
  /* draw eyes */
  for (i = 0; i < NEYES; i++)
  {
    draw_eye(mobile_base, eyes[i]);

    /******************************************************************/
    /* draw displays **************************************************/
    /* draw coordinate in configuration space for left and right eyes */
    XSetForeground(display, gc, world_colors[EYE_COLOR].display_color);
    if (i == LEFT)
      XFillRectangle(display, pixmap, gc, T12LD(eyes[i].theta),
		     T22LD(0.0), (T2DR(TDELTA) + 1), (T2DR(TDELTA) + 1));
    else if (i == RIGHT)
      XFillRectangle(display, pixmap, gc, T12RD(eyes[i].theta),
		     T22RD(0.0), (T2DR(TDELTA) + 1), (T2DR(TDELTA) + 1));
  }
  /******************************************************************/
  /* draw arms */
  /*
  for (j=0; j<NARMS; j++) {
    XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
    SIMmatXmat(mobile_base.wTb, arms[j][0].iTj, temp0);
		
    for (i=1; i<NARM_FRAMES; i++) {
      SIMmatXmat(temp0, arms[j][i].iTj, temp1);
      XDrawLine(display, pixmap, gc, W2DX(zoom,temp0[0][3]),
		W2DY(zoom,temp0[1][3]), W2DX(zoom,temp1[0][3]),
		W2DY(zoom,temp1[1][3]));
      if (i==(NARM_FRAMES-1))
	draw_circle(W2DX(zoom,temp1[0][3]), W2DY(zoom,temp1[1][3]),
		    W2DR(zoom,R_TACTILE), FILL);
      else {
	draw_circle(W2DX(zoom,temp1[0][3]), W2DY(zoom,temp1[1][3]),
		    W2DR(zoom,R_JOINT), NOFILL);
	SIMcopy_matrix(temp1, temp0);
      }
    }
		*/
    // draw endpoint forces
    /*
    mag = sqrt(SQR(arms[j][NARM_FRAMES-1].extForce[X]) +
	       SQR(arms[j][NARM_FRAMES-1].extForce[Y]));
    if (mag>0.0) {
      XDrawLine(display, pixmap, gc,
		W2DX(zoom,temp1[0][3]),	W2DY(zoom,temp1[1][3]),
          W2DX(zoom,temp1[0][3] - 0.08*arms[j][NARM_FRAMES-1].extForce[X]/mag),
         W2DY(zoom,temp1[1][3] - 0.08*arms[j][NARM_FRAMES-1].extForce[Y]/mag));
      */
      // XDrawLine(display, pixmap, gc,
      //   W2DX(zoom,temp1[0][3]), W2DY(zoom,temp1[1][3]),
      //   W2DX(zoom,temp1[0][3] + 0.125*arms[j][NARM_FRAMES-1].extForce[Y]/mag),
      //   W2DY(zoom,temp1[1][3] - 0.125*arms[j][NARM_FRAMES-1].extForce[X]/mag));
    //}
		
    /******************************************************************/
    /* draw displays **************************************************/
    /* draw coordinate in configuration space for left and right arms */
    /*
    if (j == LEFT)
      XFillRectangle(display, pixmap, gc, T12LD(arms[j][1].theta),
		     T22LD(arms[j][2].theta), (T2DR(TDELTA) + 1),
		     (T2DR(TDELTA) + 1));
    else if (j == RIGHT)
      XFillRectangle(display, pixmap, gc, T12RD(arms[j][1].theta),
		     T22RD(arms[j][2].theta), (T2DR(TDELTA) + 1),
	
		     (T2DR(TDELTA) + 1));
		     */
  //}
  
  if (HISTORY) draw_history();
  
  /* visual images *************************************************/
  draw_image(LEFT);
  draw_image(RIGHT);
}

void draw_history()
{
  int h;
	
  // draw history of all Cartesian arm postures
 // XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
	
  for (h = 0; h < history_ptr; ++h) {
    // draw Cartesian history of the mobile platform
    XFillRectangle(display, pixmap, gc, 
		   W2DX(zoom, (history[h].base_pos[0]-XDELTA/4.0)),
		   W2DY(zoom, (history[h].base_pos[1]-YDELTA/4.0)),
		   W2DR(zoom,XDELTA/2.0),
		   W2DR(zoom,YDELTA/2.0));
  }
}

void draw_ellipse(est)
Estimate est; 
{
  double m[2][2], a, b, c, root[2];
  double dx, dy, mag, eigenvalue[2], eigenvector[2][2];
  double theta, dx0, dy0, dx1, dy1;
	
  //  printf("observation time = %lf  time = %lf\n", est.t, simtime);
  
  //if ((est.t == simtime)) {
		
    // DRAW THE CURRENT ESTIMATED STATE VARIABLES AND THEIR VELOCITIES
    draw_circle(W2DX(zoom,est.position[X]), W2DY(zoom,est.position[Y]),
		W2DR(zoom,R_JOINT), FILL);
		
    m[0][0] = est.covariance[0][0];
    m[0][1] = est.covariance[0][1];
    m[1][0] = est.covariance[1][0];
    m[1][1] = est.covariance[1][1];
		
    // cov = [A  B] => det |JJt| = a(lambda)^2 + b(lambda) +c
    //       [B  C]
    a = 1.0;
    b = -(m[0][0] + m[1][1]);
    c = m[0][0] * m[1][1] - m[1][0] * m[0][1];
		
    root[0] = (-b + sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);
    root[1] = (-b - sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);
    
    // the eigenvector for root 0
    dy = 1.0;
    dx = -m[0][1] / (m[0][0] - root[0]);
    mag = sqrt(SQR(dx) + SQR(dy));
    eigenvalue[0] = sqrt(root[0]);
    eigenvector[0][0] = dx / mag;
    eigenvector[0][1] = dy / mag;
    
    // the eigenvector for root 1
    dy = 1.0;
    dx = -m[0][1] / (m[0][0] - root[1]);
    mag = sqrt(SQR(dx) + SQR(dy));
    eigenvalue[1] = sqrt(root[1]);
    eigenvector[1][0] = dx / mag;
    eigenvector[1][1] = dy / mag;
		
    dx0 = eigenvalue[0] * eigenvector[0][X];
    dy0 = eigenvalue[0] * eigenvector[0][Y];
    for (theta = M_PI / 20.0; theta < 2*M_PI; theta += M_PI / 20.0) {
      dx1 = (eigenvalue[0]*cos(theta))*eigenvector[0][X] +
	(eigenvalue[1]*sin(theta))*eigenvector[1][X];
      dy1 = (eigenvalue[0]*cos(theta))*eigenvector[0][Y] +
	(eigenvalue[1]*sin(theta))*eigenvector[1][Y];
      XDrawLine(display, pixmap, gc, W2DX(zoom,(est.position[X] + dx0)),
		W2DY(zoom,(est.position[Y] + dy0)),
		W2DX(zoom,(est.position[X] + dx1)),
		W2DY(zoom,(est.position[Y] + dy1)));
      dx0 = dx1;
      dy0 = dy1;
    }
 // }
}

draw_eye(base, eye)
Base base;
Eye eye; 
{
  double px, py;
  double rx, ry, from_x, from_y, to_x, to_y;
  double lambda_x, lambda_y;
  int xbin, ybin;

  px = base.wTb[0][0]*eye.position[0] + base.wTb[0][1]*eye.position[1] +
    base.wTb[0][3];
  py = base.wTb[1][0]*eye.position[0] + base.wTb[1][1]*eye.position[1] +
    base.wTb[1][3];
  
  from_x = px; from_y = py;
  rx = cos(base.theta + eye.theta);
  ry = sin(base.theta + eye.theta);
  /*
  //old code to cut off eye gaze at map boundary
  lambda_x = lambda_y = 1000.0;
  if (rx > 0.1)	lambda_x = (MAX_X - px) / rx;
  else if (rx < -0.1) lambda_x = (MIN_X - px) / rx;
  if (ry > 0.1) lambda_y = (MAX_Y - py) / ry;
  else if (ry < -0.1) lambda_y = (MIN_Y - py) / ry;
	
  if (lambda_x < lambda_y) {
    if (rx > 0.0) to_x = MAX_X - XDELTA;
    else to_x = MIN_X + XDELTA;
    to_y = py + lambda_x * ry;
  } 
  else {
    if (ry > 0.0) to_y = MAX_Y - YDELTA;
    else to_y = MIN_Y + YDELTA;
    to_x = px + lambda_y * rx;
  }
  */


  //trace the eye direction till you hit an obstacle
  to_x = from_x;
  to_y = from_y;		

  while (to_x < MAX_X && to_x > MIN_X && to_y < MAX_Y && to_y > MIN_Y)
  {
		//get bin for location
		ybin = (int)((MAX_Y - to_y)/YDELTA);
		xbin = (int)((to_x - MIN_X)/XDELTA);

		//check for obstacle collision
  		if (Roger.world_map.occupancy_map[ybin][xbin] == OBSTACLE) 
		{			
			break;
		}
		to_x += rx * 0.001;
		to_y += ry * 0.001;
  } 	
	
  XSetForeground(display, gc, world_colors[GAZE_COLOR].display_color);
  XDrawLine(display, pixmap, gc, W2DX(zoom,from_x), W2DY(zoom,from_y),
	    W2DX(zoom,to_x), W2DY(zoom,to_y));
  
  XSetForeground(display, gc, foreground);
  draw_circle(W2DX(zoom,px), W2DY(zoom,py), W2DR(zoom,R_EYE), NOFILL);
  draw_circle(W2DX(zoom, px+(R_EYE-R_PUPIL)*rx),
	      W2DY(zoom, py+(R_EYE-R_PUPIL)*ry), W2DR(zoom,R_PUPIL),
	      FILL);
}

draw_image(eye)
int eye; 
{
  register i, color, dx;
	
  XSetForeground(display, gc, foreground);
  if (eye == LEFT)
    XDrawRectangle(display, pixmap, gc, LEFT_IMAGE_X - 1, IMAGE_Y - 1,
		   IMAGE_WIDTH + 1, PIXEL_HEIGHT + 1);
  else if (eye == RIGHT)
    XDrawRectangle(display, pixmap, gc, RIGHT_IMAGE_X - 1, IMAGE_Y - 1,
		   IMAGE_WIDTH + 1, PIXEL_HEIGHT + 1);
	
  for (i = 0, dx = 0; i < NPIXELS; i++, dx += PIXEL_WIDTH) {
    color = eyes[eye].image[i];
    //    XSetForeground(display, gc, color);
		
    // Commented out by Dan.    
    //printf("color=%d\n", color);
    
    //    if (color > 99) XSetForeground(display, gc, background);
    //    else if (intensity >= 0) 
    //      XSetForeground(display, gc, image_color[intensity]);
    //    if (color == ARM_COLOR)
    //      XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
    //    else if (color == ARM_COLOR)
    //      XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
    //    else if (color == OBJECT_COLOR)
    //      XSetForeground(display, gc, 
    //         world_colors[OBJECT_COLOR].display_color);
    XSetForeground(display, gc, world_colors[color].display_color);
    
    if (eye == LEFT)
      XFillRectangle(display, pixmap, gc, LEFT_IMAGE_X + dx, IMAGE_Y,
		     PIXEL_WIDTH, PIXEL_HEIGHT);
    else if (eye == RIGHT)
      XFillRectangle(display, pixmap, gc, RIGHT_IMAGE_X + dx, IMAGE_Y,
		     PIXEL_WIDTH, PIXEL_HEIGHT);
  }
}

typedef struct _visible_object {
  double dx, dy;
  double radius;
  int color;
} VisibleObject;

make_images()
{
  int i, j, eye, o_index; // intensity[3];
  double x, y;
  double p_b[4], p_w[4], bTw[4][4];
  //  int feature_id, 
	
  int sort[NXBINS*NYBINS];
  VisibleObject vobject[NXBINS*NYBINS];
	
  /* initialize image white */
  // make sure eye's images keep changing when roger is moving
  for (i = 0; i < NPIXELS; i++)
    eyes[LEFT].image[i] = eyes[RIGHT].image[i] = 100;
  
  for (eye = LEFT; eye <= RIGHT; ++eye) {
    
    //    first 3 elements in the range array are the hands and the ball
    /* LEFT_ARM - in body frame */
   // sim_fwd_kinematics(LEFT, arms[LEFT][1].theta, arms[LEFT][2].theta, &x, &y);
    // convert to eye coordinates
    /*
    vobject[0].dx = x - eyes[eye].position[X];
    vobject[0].dy = y - eyes[eye].position[Y];
    vobject[0].radius = R_JOINT;
    vobject[0].color = ARM_COLOR;
		
    /* RIGHT_ARM - in body frame */
    /*
    sim_fwd_kinematics(RIGHT,arms[RIGHT][1].theta,arms[RIGHT][2].theta,&x,&y);
    vobject[1].dx = x - eyes[eye].position[X];
    vobject[1].dy = y - eyes[eye].position[Y];
    vobject[1].radius = R_JOINT;
    vobject[1].color = ARM_COLOR;
		
    /* OBJECT */
    SIMinv_transform(mobile_base.wTb, bTw);
    p_w[0] = object.position[X]; p_w[1] = object.position[Y]; 
    p_w[2] = 0.0; p_w[3] = 1.0;
    SIMmatXvec(bTw, p_w, p_b);
    vobject[2].dx = p_b[X] - eyes[eye].position[X];
    vobject[2].dy = p_b[Y] - eyes[eye].position[Y];
    vobject[2].radius = R_OBJ;
    vobject[2].color = OBJECT_COLOR;
		
    // after the first three, the rest are colored obstacles in the occupancy
    // grid
    o_index = 3; // points at the next empty element of the range array
    for (i=0;i<NYBINS;++i) {
      y = MAX_Y - i*YDELTA;
      for (j=0;j<NXBINS; ++j) {
	if (Roger.world_map.occupancy_map[i][j] == OBSTACLE) {
	  p_w[0] = MIN_X + j*XDELTA; p_w[1] = y;
	  p_w[2] = 0.0; p_w[3] = 1.0;
	  SIMmatXvec(bTw, p_w, p_b);
	  vobject[o_index].dx = p_b[X] - eyes[eye].position[X];
	  vobject[o_index].dy = p_b[Y] - eyes[eye].position[Y];
	  vobject[o_index].radius = R_OBSTACLE;
	  vobject[o_index++].color = Roger.world_map.color_map[i][j];
	}
      }
    }
    insertion_sort(vobject, sort, o_index);
    for (i=0; i<o_index; ++i)
      pinhole_camera(vobject[sort[i]], eye);
  }
  printf("Test 4 \n");
}

insertion_sort(vob, sort, num)
VisibleObject *vob;
int *sort, num;
{
  int i,j, temp;
	
  for(i=0; i<num; i++) sort[i] = i;
	
  for (i=1; i<num; i++) {
    j = i - 1;
    temp = sort[i];
    while ((j>=0) && (sqrt(SQR(vob[sort[j]].dx) + SQR(vob[sort[j]].dy)) <=
		      sqrt(SQR(vob[temp].dx) + SQR(vob[temp].dy)))) {
      sort[j+1] = sort[j];
      j--;
    }
    sort[j+1] = temp;
  }
}

// NEEDS TO BE FIXED...DOESN'T SEEM TO ACCOUNT FOR EYE ANGLE
// x,y in eye coordinate frame base coordinates
pinhole_camera(vob, i)
VisibleObject vob;
int i;
{
  int j, low_bin_index, high_bin_index;
  double phi, beta, alpha;
	
  phi = atan2(vob.dy, vob.dx) - eyes[i].theta; // eye frame heading eye->object
  //  printf("      phi for eye %d = %6.4lf\n", i, phi);
  if (fabs(phi) < FOV) { /* feature projects onto image plane */
    alpha = atan2(vob.radius, sqrt(SQR(vob.dx)+SQR(vob.dy)));
    low_bin_index = (int) (NPIXELS / 2.0 * (1.0 + tan(phi - alpha)));
    low_bin_index = MAX(low_bin_index,0);
    high_bin_index = (int) (NPIXELS / 2.0 * (1.0 + tan(phi + alpha)));
    high_bin_index = MIN(high_bin_index,(NPIXELS-1));
    for (j = low_bin_index; j <= high_bin_index; j++) {
      eyes[i].image[j] = vob.color;
    }
  }
  //  else {
  //     printf("FEATURE OUTSIDE FIELD OF VIEW \n");
  //  }
}

/* forward kinematics in base frame **************************************/
sim_fwd_kinematics(arm_id, theta1, theta2, x, y)
int arm_id;
double theta1, theta2;
double *x, *y; 
{
/*
  *x = LARM_1 * cos(theta1) + LARM_2 * cos(theta1 + theta2);
  *y = LARM_1 * sin(theta1) + LARM_2 * sin(theta1 + theta2);
	
  if (arm_id == LEFT) *y += ARM_OFFSET;
  else *y -= ARM_OFFSET;
  */
}

sim_arm_Jacobian(theta1,theta2, Jacobian)
double theta1,theta2;
double Jacobian[2][2];
{
/*
  Jacobian[0][0] = -LARM_1*sin(theta1) - LARM_2*sin(theta1 + theta2);
  Jacobian[0][1] = -LARM_2*sin(theta1 + theta2);
  Jacobian[1][0] =  LARM_1*cos(theta1) + LARM_2*cos(theta1 + theta2);
  Jacobian[1][1] =  LARM_2*cos(theta1 + theta2);
  */
}

void goal_detection()
{
  char *temp, buf[20];
	
  if (object.position[Y] < (MIN_Y + R_GOAL)) {
    printf("Scored at: %f\n",simtime);
		
    temp = gcvt(simtime,5,buf);
    printf("temp=%s\n",temp);
		
    XSetForeground(display, gc, foreground);
    XDrawString(display, pixmap, gc,
		W2DX(zoom,3.0), W2DY(zoom,1.8), temp, 5);
    //XDrawString(display, pixmap, gc, T12RD(-0.45), T22RD(-3.6), "time=", 5);
    
    XkwSetWidgetLabel(start_w, "Start");
    XtRemoveTimeOut(timer);
    timer = 0;
  }
}

/***********************************************************************/
/** Input modes --- update the setpoint data structure:            *****/ 
/**    JOINT_ANGLE_INPUT mode: mouse(q1,q2) -> arm (q1, q2) ref    *****/
/**      BASE_GOAL_INPUT mode: mouse(x,y) -> base (x,y) ref	   *****/
/**      ARM_GOALS_INPUT mode: mouse(x,y) -> arm (x,y) ref  	   *****/
/**        BALL_POSITION mode: mouse(x,y) -> red ball (x,y)        *****/
/**           MAP_EDITOR mode: left button(x,y) -> obstacle        *****/
/**                            right button(x,y) -> goal           *****/
/***********************************************************************/
update_setpoints(roger)
Robot * roger;
{
  int xbin, ybin;
  //void rrt_planning();
  double wTb[4][4], bTw[4][4], ref_b[4], ref_w[4], theta0, theta1, avg_theta;
  //cartesian coordinates
  double x, y;
  //configuration coordinates
  double q1, q2;
  //body side
  int body_side = 0;
  double range = 0.0;	

  //*******************************************************************
  // initialize the configuration of the robot at each mode change
  //*******************************************************************
  if (init_control_flag) initialize_control_mode(roger);  
  init_control_flag = FALSE;

  //*******************************************************************
  //***** process button input based on selected input mode ***********
  //*******************************************************************
  // Mode dependent button interfaces  
  if (roger->button_event) {
    switch(roger->input_mode) {
    case JOINT_ANGLE_INPUT:
      //check if inputs are valid
      if (isConfigurationInput(roger->button_reference[X], 
			       roger->button_reference[Y],
			       &q1, &q2, &body_side) == FALSE)
	break;

      if (roger->control_mode == TELEOPERATION) {
	//left mouse button -> arms
	if (roger->button_event == LEFT_BUTTON) {		
	  printf("q1=%6.4lf q2=%6.4lf\n", q1, q2);
	  
	 // roger->arm_setpoint[body_side][0] = q1;
	 // roger->arm_setpoint[body_side][1] = q2;
	} 
	//right mouse button -> eyes
	else if (roger->button_event == RIGHT_BUTTON) {
	  printf("q1=%6.4lf \n", q1);
	  roger->eyes_setpoint[body_side] = q1;
	}
      }
      break;

    case BASE_GOAL_INPUT:
      //check if inputs are valid
      if (isCartesianInput(roger->button_reference[X], 
			   roger->button_reference[Y], &x, &y) == FALSE)
	break;

      if (roger->control_mode == TELEOPERATION)
	Teleoperation_Cartesian_input_base(roger, x, y, roger->button_event);
      break;

    case ARM_GOAL_INPUT:
      // BUTTON INTERFACE - ARM TELEOPERATOR - CARTESIAN CONTROL
      //check if inputs are valid
      if (isCartesianInput(roger->button_reference[X],
			   roger->button_reference[Y],&x,&y) == FALSE)
	break;
      
      if (roger->control_mode == TELEOPERATION)
	Teleoperation_Cartesian_input_arms(roger, x, y, roger->button_event);
      break;

    case BALL_INPUT:
      //check if inputs are valid
      if (isCartesianInput(roger->button_reference[X],
			   roger->button_reference[Y], &x, &y) == FALSE) 
	break;
      else {
	place_object(x, y);
	break;
      }
    case MAP_INPUT:
	  if (isCartesianInput(roger->button_reference[X],
			   roger->button_reference[Y], &x, &y) == FALSE) 
	  	break;
	  
	  int xbin, ybin;

	  printf("Map editor input - x: %4.3f, y: %4.3f - button: %d\n", x, y, roger->button_event);

	  xbin = (x - MIN_X) / XDELTA;
	  ybin = NYBINS - (y - MIN_Y) / YDELTA;
	  if ((xbin<0) || (xbin>(NXBINS-1)) || (ybin<0) || (ybin>(NYBINS-1))) {
		printf("Out of the boundary!!!\n");
	  }
	  else {
		if (roger->button_event == LEFT_BUTTON) { // obstacles in Cartesian space
		  if (roger->world_map.occupancy_map[ybin][xbin] == OBSTACLE) {
		printf("deleting an obstacle xbin=%d  ybin=%d\n", xbin, ybin);
		fflush(stdout);
		roger->world_map.occupancy_map[ybin][xbin] = FREESPACE;
		//	    delete_bin_bumper(xbin,ybin);
		  } 
		  else if (roger->world_map.occupancy_map[ybin][xbin] == FREESPACE) {
		printf("inserting an obstacle xbin=%d  ybin=%d\n", xbin, ybin);
		fflush(stdout);
		roger->world_map.occupancy_map[ybin][xbin] = OBSTACLE;
		roger->world_map.potential_map[ybin][xbin] = 1.0;
		roger->world_map.color_map[ybin][xbin] = DARKYELLOW;
		  }
		} 
		else if (roger->button_event == MIDDLE_BUTTON) { } 
		else if (roger->button_event == RIGHT_BUTTON) {
		  if (roger->world_map.occupancy_map[ybin][xbin] == GOAL) {
		printf("deleting an goal xbin=%d  ybin=%d\n", xbin, ybin);
		fflush(stdout);
		roger->world_map.occupancy_map[ybin][xbin] = FREESPACE;
		  } 
		  else if (roger->world_map.occupancy_map[ybin][xbin] == FREESPACE) {
		printf("inserting an goal xbin=%d  ybin=%d\n", xbin, ybin);
		fflush(stdout);
		roger->world_map.occupancy_map[ybin][xbin] = GOAL;
		roger->world_map.potential_map[ybin][xbin] = 0.0;
		  }			
		}
		//update harmonic map						
		//sor(roger);
	  }		


      break;
    default:
      break;
    }
    roger->button_event = FALSE;
  }

  //*******************************************************************
  //******** perform control based on selected control mode ***********
  //*******************************************************************

  switch(roger->control_mode) {

  case PROJECT3:
    project3_control(roger, simtime);
    break;
    
  case PROJECT4:
    project4_control(roger,simtime);
    break;
			
  case PROJECT5:
    project5_control(roger, simtime);
    break;
    
  case PROJECT6:
    project6_control(roger, simtime);
    break;
  default: 
    break;
  }
}

//check if input is in configuration space area and return angles

int isConfigurationInput(x, y, q1, q2, side) 
double x, y;
double *q1, *q2;
int *side;
{
  double range = 0.0;		

  *q1 = 0.0;
  *q2 = 0.0;
  *side = LEFT;

  //printf("Conf space check: %6.4lf, %6.4lf \n", x, y);

  //check if click is in configuration space area
  //X-Direction
  if (x < D2WX(100.0, T12LD(T1_MIN)) || 
      (x > D2WX(100.0, T12LD(T1_MAX)) && x < D2WX(100.0, T12RD(T1_MIN))) || 
      x > D2WX(100.0, T12RD(T1_MAX))) {
    //printf("x-location out of bounds!!! %6.4lf \n", x);
    return FALSE;
  }
  //Y-Direction
  else if (y < D2WY(100.0, T22LD(T2_MIN)) || y > D2WY(100.0, T22LD(T2_MAX)) ) {
    //printf("y-location out of bounds!!! %6.4lf\n", y);
    return FALSE;
  }	

  //calculate joint angles from click locations
  //left arm
  /*
  if (x < D2WX(100.0, T12LD(T1_MAX))) {
    *side = LEFT;
    range = D2WX(100.0, T12LD(T1_MAX)) - D2WX(100.0, T12LD(T1_MIN));
    *q1 = (x - (D2WX(100.0, T12LD(T1_MIN)) + range/2.0)) / (range/2.0) * M_PI;
    range = D2WY(100.0, T22LD(T2_MAX)) - D2WY(100.0, T22LD(T2_MIN));
    *q2 = (y - (D2WY(100.0, T22LD(T2_MIN)) + range/2.0)) / (range/2.0) * M_PI;
  }
  */
  //right arm
  /*
  else { 
    *side = RIGHT;
    range = D2WX(100.0, T12RD(T1_MAX)) - D2WX(100.0, T12RD(T1_MIN));
    *q1 = (x - (D2WX(100.0, T12RD(T1_MIN)) + range/2.0)) / (range/2.0) * M_PI;
    range = D2WY(100.0, T22RD(T2_MAX)) - D2WY(100.0, T22RD(T2_MIN));
    *q2 = (y - (D2WY(100.0, T22RD(T2_MIN)) + range/2.0)) / (range/2.0) * M_PI;
  }
  */
  return TRUE;
}
//make everything in here to make button interface work
//check if input is in cartesian space area
int isCartesianInput(x_input, y_input, x, y) 
double x_input, y_input;
double *x, *y;
{
  if (x_input < MIN_X || x_input > MAX_X ||
      y_input < MIN_Y || y_input > MAX_Y) {
    //printf("Location out of bounds!!!\n");
    return FALSE;
  }
  *x = x_input;
  *y = y_input;
  
  return TRUE;
}

Teleoperation_Cartesian_input_arms(roger, x, y, button)
Robot* roger;
double x;
double y;
int button;
{
/*
  int limb = 0;

  printf("Arm goal input - x: %4.3f, y: %4.3f - button: %d\n", x, y, button);

  limb = 0;
  if (button == LEFT_BUTTON) {
    limb = LEFT;
    printf("world frame input for LEFT arm x=%6.4lf y=%6.4lf\n", x, y);
  }
  else if (button == RIGHT_BUTTON) {
    limb = RIGHT;
    printf("world frame input for RIGHT arm x=%6.4lf y=%6.4lf\n", x, y);
  }
  else return;
  CartesianArmControl(roger, limb, x,y); //PROJECT #2 - Kinematics.c
*/
}

Teleoperation_Cartesian_input_base(roger, x, y, button)
Robot* roger;
double x;
double y;
int button;
{
  double dx, dy, theta;

  roger->base_setpoint[X] = x;
  roger->base_setpoint[Y] = y;
  dx = x - roger->base_position[X];
  dy = y - roger->base_position[Y];
  theta = roger->base_setpoint[THETA] = atan2(dy,dx);
  printf("world frame Base goal: x=%6.4lf y=%6.4lf theta=%6.4lf\n", 
	 x, y, theta);
}

// initialize all setpoints to reflect the current posture, set the Cartesian
//    button_reference as appropriate for the new mode (base, left arm,
//       right arm, stereo head, harmonic function, integrated app)
initialize_control_mode(roger)
Robot * roger;
{
  double wTb[4][4],ref_w[4],ref_b[4];
  int i,j;

  // define all setpoints to current positions and zero velocities
  roger->base_setpoint[X] = roger->base_position[X];// + BASE_CONTROL_OFFSET*cos(roger->base_position[THETA]);
  roger->base_setpoint[Y] = roger->base_position[Y];// + BASE_CONTROL_OFFSET*sin(roger->base_position[THETA]);
  roger->base_setpoint[THETA] = roger->base_position[THETA];

/*
  roger->arm_setpoint[LEFT][0] = roger->arm_theta[LEFT][0];
  roger->arm_setpoint[LEFT][1] = roger->arm_theta[LEFT][1];
  roger->arm_setpoint[RIGHT][0] = roger->arm_theta[RIGHT][0];
  roger->arm_setpoint[RIGHT][1] = roger->arm_theta[RIGHT][1];
*/
  roger->eyes_setpoint[LEFT] = roger->eye_theta[LEFT];
  roger->eyes_setpoint[RIGHT] = roger->eye_theta[RIGHT];

  //call the reset method of the last mode --> clear all thing you modified
  switch ((roger->control_mode + N_CONTROL_MODES - 1) % N_CONTROL_MODES) 
  // "+ N_CONTROL_MODES" needed as modulo of negative numbers incorrect
  {
  case PROJECT3: 
    project3_reset(roger); 
    break;
  case PROJECT4: 
    project4_reset(roger); 
    break;
  case PROJECT5: 
    project5_reset(roger); 
    break;
  case PROJECT6: 
    project6_reset(roger); 
    break;
  default:
    break;
  }
  init_control_flag = FALSE;  
}
