/*************************************************************************/
/* File:        project5.c                                               */
/* Description: User project #5                                          */
/* Date:        11-20-2012                                               */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "Xkw/Xkw.h"

#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"


int project5_initialized = FALSE;
double extern commandVel;
float xlocations[1000];
float ylocations[1000];
float error = 0.0f;
int pathLen =0;
float  distTraveled = 0.0f;
float saveX =0.0f;
float saveY = 0.0f;
project5_control(roger)
Robot* roger;
{
	static int state = 0;
    
	//make sure walls, dilation, and goals are inserted
	if (project5_initialized == FALSE)
	{
		project5_init(roger);
		project5_initialized = TRUE;
		
	}
    
	sor(roger);
    
	
    // tells the velocity controller what velocity to set, calls sor(roger)
    control_velocity(roger);
    
	state = primitive4(roger);

}


/*
 / Given two grid coordinates (indeces) in the occupancy grid, calculate and return the distance between the closest points of both grid cells.
 / The size of each cell is XDELTA x YDELTA.
 */
	 double cell_distance(xbin1, ybin1, xbin2, ybin2)
	 int xbin1, ybin1, xbin2, ybin2;
	 {
	 	double dist[2] = {0.0, 0.0};
    
	 	//calculate distance in x direction
	     dist[X] = xbin2 - xbin1;
    
	     if (fabs(dist[X]) <= 1) {
	         dist[X] = 0;
	     }
	     else{
	         dist[X] -= 1*SGN(dist[X]);
	         dist[X] = dist[X]*XDELTA;
	     }
	 	//calculate distance in y direction
	     dist[Y] = ybin2 - ybin1;
    
	     if (fabs(dist[Y]) <= 1) {
	         dist[Y] = 0;
	     }
	     else{
	         dist[Y] -= 1*SGN(dist[Y]);
	         dist[Y] = dist[Y]*YDELTA;
	     }
	     return sqrt( SQR(dist[X]) + SQR(dist[Y]) );
	 }


#define ROBOT_DILATE_RADIUS 0.2


dilate_obstacles(roger)
Robot* roger;
{
	int w, i, j;
	int xbin, ybin;
	double x,y, dist;
    
	int num_obs = 0;
	int num_dilate = 0;
    
	//printf("Dilating obstacles...\n");
    
	//remove previous dilated obstacles
	for (i = 0; i < NYBINS; ++i) {   // rows
		for (j = 0; j < NXBINS; ++j) {   // cols
			if (roger->world_map.occupancy_map[i][j] == DILATED_OBSTACLE) {
				roger->world_map.occupancy_map[i][j] = FREESPACE;
				roger->world_map.potential_map[i][j] = 1.0;
				roger->world_map.color_map[i][j] = 0;
			}
		}
	}
    
    //-------------------------------
    //PROJECT 5: Iterate over the occupancy grid. Whenever you encounter an OBSTACLE, marks all FREESPACE cell in
    //distance < ROBOT_DILATE_RADIUS as DILATED_OBSTACLE with potential = 1.0 and color = LIGHTYELLOW (see example below
    //You need to complete function "cell_distance(xbin1,ybin1, xbin2, ybin2)" first which will return the distance between the closest
    //points of two grid cells. Adjust constant ROBOT_DILATE_RADIUS above to match the circle size that matches your robot.
    
	
    //create dilated obstacles
    for (i = 0; i < NYBINS; ++i) {   // rows y
		for (j = 0; j < NXBINS; ++j) {   // col x
			if (roger->world_map.occupancy_map[i][j] == OBSTACLE) {
                num_obs++;
                // go through all freespaces within radius distance and mark it as dilated
                // in this case, go through every square and if in distance, mark as dialated
                // To do: make this efficient
                for (ybin = 0; ybin < NYBINS; ++ybin) {   // rows y
                    for (xbin = 0; xbin < NXBINS; ++xbin) {   // cols x
                        if (roger->world_map.occupancy_map[ybin][xbin] == FREESPACE) {
                            if (cell_distance(j, i, xbin, ybin) <= ROBOT_DILATE_RADIUS) {
                                roger->world_map.occupancy_map[ybin][xbin] = DILATED_OBSTACLE;
                                roger->world_map.potential_map[ybin][xbin] = 1.0;
                                roger->world_map.color_map[ybin][xbin] = LIGHTYELLOW;
                                num_dilate++;
                            }
                        }
                    }
                }
            }
		}
	}
    //printf("Total: %d obstacles, %d freespaces marked as dilated obstacles.\n", num_obs, num_dilate);
}



/*
 This will be replaced with a goal
 */
#define NUM_SEARCH_LOCATIONS 1
double search_locations[NUM_SEARCH_LOCATIONS][2] = { {3.5, 0.0}};
/*
 / Initializes a set a search locations in the occupancy grid / potentential map
 / Can be called whenever you want to set/renew the search goals
 */
init_search_locations(roger)
Robot* roger;
{
	int i;
	int xbin ,ybin;
    
	for (i=0; i<NUM_SEARCH_LOCATIONS; i++)
	{
		ybin = (int)((MAX_Y - search_locations[i][Y])/YDELTA);
		xbin = (int)((search_locations[i][X] - MIN_X)/XDELTA);
        
		//printf("%d : %4.3f, %4.3f -> %d, %d \n", i, search_locations[i][X], search_locations[i][Y], xbin, ybin );
		roger->world_map.occupancy_map[ybin][xbin] = GOAL;
		roger->world_map.potential_map[ybin][xbin] = 0.0;
		roger->world_map.color_map[ybin][xbin] = GREEN;
	}
}


/*
 / DO NOT ALTER
 / Run SOR numerical relaxation of the harmonic map
 */
 sor(roger)
 Robot * roger;
 {
 	int i, j, sor_count=0, converged = FALSE;
 	double sor_once();
    
 	while (!converged && (sor_count < 10000)) {
 		++sor_count;
 		if (sor_once(roger) < THRESHOLD)
 			converged = TRUE;
 	}
     // printf("Sor called\n");
    
 	//if (sor_count > 1)
 		//printf("completed harmonic function --- %d iterations\n", sor_count);
 }

/*
 / one complete backup, only dirichlet boundary conditions
 */
 double sor_once(roger)
 Robot * roger;
 {
 	int i, j, ipos, ineg, jpos, jneg;
 	double residual, max, front, back, up, down;
    
 	max = 0.0;
    
     // iterate over entire map once
     // return the  maximum change in the potential value over the entire
     // occupancy map as a means of determining convergence
     //
     // 0.0 <= roger->world_map.potential_map[ybin][xbin] <= 1.0
    
    
     for (i = 0; i < NYBINS; ++i) {   // rows y
        
 		for (j = 0; j < NXBINS; ++j) {   // col x
            
             if (roger->world_map.occupancy_map[i][j] == FREESPACE) {
                
                 ipos = i+1;
                 ineg = i-1;
                 jpos = j+1;
                 jneg = j-1;
                
                 front = roger->world_map.potential_map[i][jpos];
                 back = roger->world_map.potential_map[i][jneg];
                 up = roger->world_map.potential_map[ipos][j];
                 down = roger->world_map.potential_map[ineg][j];
                
                 residual = (0.46/4)*(front + back + up + down - 4*roger->world_map.potential_map[i][j]);
                
                 if (residual > max) {
                     max = residual;
                 }
                 roger->world_map.potential_map[i][j] += residual;
             }
         }
     }
 	return max;
 }


//----------------------------------------------------------------------------------------------------------------------------------------------
//--------------Primitive4 - follow gradient of harmonic function-------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------

#define CONTROL_STEP 3.5

/*
 / Harmonic function path planner / follower. It will follow a gradient in the potential map
 / to a goal. Once reached, it will delete the goal and return CONVERGED. If no gradient is
 / available (no goals or gradient too shallow) it will return NO_REFERENCE. In all other cases
 / it will return UNCONVERGED.
 */

int primitive4(roger)
Robot* roger;
{
	int xbin, ybin;
	double x, y, bb, mag, grad[2], compute_gradient();
	int state;
	
    x = roger->base_position[X];
    y = roger->base_position[Y];
    
    ybin = (int)((MAX_Y - y)/YDELTA);
    xbin = (int)((x - MIN_X)/XDELTA);
    
    if(roger->world_map.occupancy_map[ybin][xbin] == GOAL){
        
        //state = CONVERGED;
        //delete current goal from harmonic map
        roger->world_map.occupancy_map[ybin][xbin] = FREESPACE;
        roger->world_map.potential_map[ybin][xbin] = 0.0;
        //relax map again to account for changes
        sor(roger);
        
    }
    else{
        compute_gradient(x, y, roger, grad);
        mag = sqrt(SQR(grad[0])+SQR(grad[1]));
        
        if (fabs(mag) < THRESHOLD) {
            //state = NO_REFERENCE;
        }
        else{
            //state = UNCONVERGED;
            roger->base_setpoint[X] = roger->base_position[X] - CONTROL_STEP*(grad[X]/mag);
            roger->base_setpoint[Y] = roger->base_position[Y] - CONTROL_STEP*(grad[Y]/mag);
        }
    }
    //printf("prim 4\n");
	return state;
}

/*
 / DO NOT ALTER
 / Given a x,y location, it will use the potential map to compute a gradient returned in 'grad'
 /
 */
double compute_gradient(x, y, roger, grad)
double x, y;
Robot * roger;
double grad[2]; // grad = [ d(phi)/dx  d(phi)/dy ] ~ [ d(phi)/dj  -d(phi)/di ]
{
	int i0,i1,j0,j1;
	double mag, dphi_di, dphi_dj, del_x, del_y;
    
	j0 = (int) ((x-MIN_X)/XDELTA);
	j1 = (j0+1);
	i1 = NYBINS - (int) ((y - MIN_Y)/YDELTA);
	i0 = (i1-1);
    
	del_x = (x-MIN_X)/XDELTA - j0;
	del_y = (NYBINS - (y - MIN_Y)/YDELTA) - i0;
    
	dphi_dj = ((1.0-del_y)*(roger->world_map.potential_map[i0][j1] -
                            roger->world_map.potential_map[i0][j0] ) +
               (del_y)*(roger->world_map.potential_map[i1][j1] -
                        roger->world_map.potential_map[i1][j0]  ) );
	dphi_di = ((1.0-del_x)*(roger->world_map.potential_map[i1][j0] -
                            roger->world_map.potential_map[i0][j0] ) +
               (del_x)*(roger->world_map.potential_map[i1][j1] -
                        roger->world_map.potential_map[i0][j1]  ) );
    
	grad[0] = dphi_dj; grad[1] = -dphi_di;
    
	mag = sqrt(SQR(grad[0])+SQR(grad[1]));
    
	if (mag>THRESHOLD) {
		grad[0] /= mag; grad[1] /= mag;
	}
	else {
		grad[0] = grad[1] = 0;
	}
	return(mag);
}

/*
 / Clean environment from project 5 specific things.
 */
project5_reset(roger)
Robot* roger;
{
	int i, j;
    
	//clear walls, dilated obstacles, goals
	for (i = 1; i < NYBINS-1; ++i) {   // rows
		for (j = 1; j < NXBINS-1; ++j) {   // cols
			roger->world_map.occupancy_map[i][j] = FREESPACE;
			roger->world_map.potential_map[i][j] = 1.0;
            
		}
	}
	project5_initialized = FALSE;
}

/*
 / Setup environment for project 5
 */
project5_init(roger)
Robot* roger;
{
	
	
	//insert the walls
	draw_room(roger);
	
	//dilate the obstacles
	dilate_obstacles(roger);
	
	//init the search locations
	init_search_locations(roger);
	
	
}

#define VSTEP 0.1 // STEP in meters along path
#define SIZE 1000000
#define savePathStep 0.01
save_path(roger)
Robot* roger;
{
    int xbin, ybin, already_used[NXBINS][NYBINS];
	double compute_gradient(), mag, grad[2], x, y;
    
    
    // create gradients towards goal
    sor(roger);

    // get position
    x = -3.5;
    y = 0;
    
    // find which bin we're in
    ybin = (int)((MAX_Y - y)/YDELTA);
    xbin = (int)((x - MIN_X)/XDELTA);
	
	xlocations[0] = x;
	ylocations[0] = y;
    
    // get the gradient at that point
    mag = compute_gradient(x, y, roger, grad);
    
    // find the bins with the goal
    int gybin = (int)((MAX_Y-0.0)/YDELTA);
    int gxbin = (int)((3.5-MIN_X)/XDELTA);
    
    // Compute headings by following gradients until you reach the goal
    while ((mag > THRESHOLD) && (roger->world_map.occupancy_map[ybin][xbin] != GOAL)) {
                
        // go along the path
        x -= savePathStep*grad[0];
        y -= savePathStep*grad[1];
        
		xlocations[pathLen] = x;
		ylocations[pathLen] = y;
		
        // find the bin we're in
        ybin = (int)((MAX_Y-y)/YDELTA);
        xbin = (int)((x-MIN_X)/XDELTA);
        
        // go to next point in path
        pathLen++;
	  //  printf("MAKE SIZE BIGGER %d \n", pathLen);
		
        if (pathLen >= SIZE) {
            //printf("MAKE SIZE BIGGER %d \n", pathLen);
            break;
        }
        // get the gradient
        mag = compute_gradient(x, y, roger, grad);

        // At goal
        if (cell_distance(xbin, ybin, gxbin, gybin) < 1) {
            break;
        }
    }
	
}



/*
 / draws the rooms and init potential map
 */
draw_room(roger)
Robot *roger;
{
	int w, i, j;
	int xbin, ybin;
	double x,y, dist;
    
	//remove dilated obstacles
	for (i = 0; i < NYBINS; ++i) {   // rows
		for (j = 0; j < NXBINS; ++j) {   // cols
			if (roger->world_map.occupancy_map[i][j] == DILATED_OBSTACLE) {
				roger->world_map.occupancy_map[i][j] = FREESPACE;
				roger->world_map.potential_map[i][j] = 1.0;
			}
		}
	}
    
	//insert walls
	//wall segments: {start_x, start_y, stop_x, stop_y}
	double walls[3][4] = {{0.7, -2.0, 0.0, 2.8}, {-0.7, -0.5, 1.4, 0.0}, {-0.7, -0.5, 0, 0.8 }};
    
	for (w=0; w<3; w++)
	{
		if (walls[w][2] != 0.0)
		{
			y = walls[w][1];
			for (x=walls[w][0]; x<walls[w][0] + walls[w][2]; x+=XDELTA/4.0)
			{
				ybin = (int)((MAX_Y - y)/YDELTA);
				xbin = (int)((x - MIN_X)/XDELTA);
				roger->world_map.occupancy_map[ybin][xbin] = OBSTACLE;
				roger->world_map.potential_map[ybin][xbin] = 1.0;
			}
		}
		else
		{
			x = walls[w][0];
			for (y=walls[w][1]; y<walls[w][1] + walls[w][3]; y+=YDELTA/4.0)
			{
				ybin = (int)((MAX_Y - y)/YDELTA);
				xbin = (int)((x - MIN_X)/XDELTA);
				roger->world_map.occupancy_map[ybin][xbin] = OBSTACLE;
				roger->world_map.potential_map[ybin][xbin] = 1.0;
			}
		}
	}
    
	//make sure potential map is initialized everywhere
	for (i = 0; i < NYBINS; ++i) {   // rows
		for (j = 0; j < NXBINS; ++j) {   // cols
			switch (roger->world_map.occupancy_map[i][j]) {
				case FREESPACE:
					roger->world_map.potential_map[i][j] = 1.0;
					break;
				case OBSTACLE:
					roger->world_map.potential_map[i][j] = 1.0;
					break;
				case GOAL:
					roger->world_map.potential_map[i][j] = 0.0;
					break;
				default:
					break;
			}
		}
	}
}

// Max curve is the sharpest turn you can do and still have longitudinal velocity
//const double MAX_CURVE = 0.08727f; // radians, 5 degrees
const double MAX_CURVE = 3.14159/4; // radians, 5 degrees

// Min curve is the sharpest turn you can do at max velocity
//const double MIN_CURVE = 0.01745f; // radians, 1 degree
const double MIN_CURVE = 0.0; // radians, 1 degree

// Some selected max v
const double MAX_V = 8.0f; // meters/second

//18 , 111.11  , 111.11
//8 , 11.11 , 10

// Best safe performance for motors
const double MAX_A = 10; // not in m/s^2

// Our awesome smoothing algorithm
void smooth(double *vel_g_cu, int size, double a){
    int i;
    for (i = 0; i < size-1; i++){
        if (vel_g_cu[i]+a < vel_g_cu[i+1]){
            vel_g_cu[i+1] = vel_g_cu[i]+a;
		}else{
			//vel_g_cu[i+1] = vel_g_cu[i];
		}
    }
    for (i = size-1; i > 1; i--){
        if (vel_g_cu[i-1] > vel_g_cu[i]+a){
            vel_g_cu[i-1] = vel_g_cu[i]+a;
		}else{
			//vel_g_cu[i-1] = vel_g_cu[i];
		}
    }
}

#define ROBOT_MASS 1.0
// Our awesome function. Needs to take into account 1/r^2 relationship with velocity
control_velocity(roger)
Robot* roger;
{
    int xbin, ybin, already_used[NXBINS][NYBINS];
	double compute_gradient(), mag, grad[2], x, y;
	double headings[SIZE];
    
    int numOfPointsInPath = 0;
    
    // create gradients towards goal
    sor(roger);
	
	float velocity = roger->base_velocity[X];
	//if( velocity < .001  ){
		save_path(roger);
		//}
		
    // get position
    x = roger->base_position[X];
    y = roger->base_position[Y];
    
    // find which bin we're in
    ybin = (int)((MAX_Y - y)/YDELTA);
    xbin = (int)((x - MIN_X)/XDELTA);
    
    // get the gradient at that point
    mag = compute_gradient(x, y, roger, grad);
    
    // find the bins with the goal
    int gybin = (int)((MAX_Y-0.0)/YDELTA);
    int gxbin = (int)((3.5-MIN_X)/XDELTA);
    
    // Compute headings by following gradients until you reach the goal
    while ((mag > THRESHOLD) && (roger->world_map.occupancy_map[ybin][xbin] != GOAL)) {
        
        // set heading
        headings[numOfPointsInPath] = fabs(atan2f(grad[1], grad[0]));
        
        // go along the path
        x -= VSTEP*grad[0];
        y -= VSTEP*grad[1];
        
        // find the bin we're in
        ybin = (int)((MAX_Y-y)/YDELTA);
        xbin = (int)((x-MIN_X)/XDELTA);
        
        // go to next point in path
        numOfPointsInPath++;
        if (numOfPointsInPath >= SIZE) {
            //printf("MAKE SIZE BIGGER %d \n", numOfPointsInPath);
            break;
        }
        // get the gradient
        mag = compute_gradient(x, y, roger, grad);

        // At goal
        if (cell_distance(xbin, ybin, gxbin, gybin) < 1) {
            break;
        }
    }
    
    // Compute velocities based on headings
    if ((mag > THRESHOLD)) {
        // Compute change in headings
        double change[numOfPointsInPath];
        int i=0;
        change[0] = 0.0f;
        for (i = 1; i < numOfPointsInPath; i++) {
            change[i] = fabs(headings[i] - headings[i-1]);
        }
        // Compute the Max allowed velocity
        double velocity[numOfPointsInPath];
         for (i = 0; i < numOfPointsInPath; i++) {
         // Linear increase should be the square of change
         velocity[i] = MAX_V*((3.1415/4)-change[i]) / (3.1415/4);
         }

        // Set the current longitudinal velocity
        velocity[0] = roger->base_velocity[X];
        
        // set final velocity to 0
        if (numOfPointsInPath > 1) {
            velocity[numOfPointsInPath-1] = 0.0f;
        }
        
        // Smooth the velocities
        smooth(velocity, numOfPointsInPath, MAX_A);
 
		
	    // get position
	    x = roger->base_position[X];
	    y = roger->base_position[Y];
    
	    // find which bin we're in
	    ybin = (int)((MAX_Y - y)/YDELTA);
	    xbin = (int)((x - MIN_X)/XDELTA);
    
	    // get the gradient at that point
	    mag = compute_gradient(x, y, roger, grad);
		
		roger->base_setpoint[THETA] = atan2(-grad[1] , -grad[0]) ;


		distTraveled = sqrt( (x-saveX)*(x-saveX) + (y-saveY)*(y-saveY));

		if(distTraveled > VSTEP){
			saveX = x;
			saveY = y;

       	 	if (cell_distance(xbin, ybin, gxbin, gybin) > 0.1f) {
        
				find_min_error(roger,x,y);
				printf("%f\n" , error );
			}
		}
        
        // If there is a path
        if (numOfPointsInPath > 0) {
            if( velocity[0] < velocity[1]){
              //  printf("accel\n");
				 commandVel = 10;
            } else if(velocity[0] > velocity[1]){
               // printf("deccel\n");
                commandVel = -10;
            }else{}
        // Stand still
        }else{
            //printf("don't move\n");
		     commandVel = -roger->base_velocity[X]*100;
        }
    }
}


find_min_error(roger , x , y)
Robot* roger;
float x;
float y;
{
	
	int ct = 0;
	float currX;
	float currY;
	float currX1;
	float currY1;
	float d;
	float d1;
	float minE = 1000000.0f;
    int ybin = (int)((MAX_Y-y)/YDELTA);
    int xbin = (int)((x-MIN_X)/XDELTA);
    int currXBin;
    int currYBin ;
	
	for(ct  = 0 ; ct < pathLen-1 ; ct++){
		currX = xlocations[ct];
		currY = ylocations[ct];
		
	    int currXBin = (int)((MAX_Y-currY)/YDELTA);
	    int currYBin = (int)((currY-MIN_X)/XDELTA);
		
		
		//d =  cell_distance(xbin, ybin, currXBin, currYBin);
		d = sqrt( (x-currX)*(x-currX) + (y-currY)*(y-currY) );
		if(d < minE){
			minE = d;
		}		
	}

	if(pathLen != 0){
		error += (minE*VSTEP);
	}
}


read_map(roger)
Robot* roger;
{
    int xbin, ybin;
    
    int x, y = 0;
    // Read in x and y
    //printf("Read in - x: %4.3f, y: %4.3f - button: %d\n", x, y, roger->button_event);
    
    xbin = (x - MIN_X) / XDELTA;
    ybin = NYBINS - (y - MIN_Y) / YDELTA;
    if ((xbin<0) || (xbin>(NXBINS-1)) || (ybin<0) || (ybin>(NYBINS-1))) {
        printf("Out of the boundary!!!\n");
    }
    else{
        // First set of coordinates is the goal
        roger->world_map.occupancy_map[ybin][xbin] = GOAL;
        roger->world_map.potential_map[ybin][xbin] = 0.0;
        
        // The rest are obstacles defined x1 y1 x2 y2 (line from p1 to p2) Loop until end of file adding obstacles
        roger->world_map.occupancy_map[ybin][xbin] = OBSTACLE;
        roger->world_map.potential_map[ybin][xbin] = 1.0;
        roger->world_map.color_map[ybin][xbin] = DARKYELLOW;
    }
}

//use cartesian space input from mouse including mouse button info
project5_cartesian_input(roger, x, y, button)
Robot* roger;
double x;		//x value
double y;		//y value
int button;		//mouse button
{
    
	//printf("Project 5 input - x: %4.3f, y: %4.3f - button: %d\n", x, y, button);
    
}

// this procedure can be used to prompt for and read user customized input values
project5_enter_params()
{
	//printf("Project 5 enter_params called. \n");
    
}

#define STEP         0.01

/* do not alter */
//function called when the 'visualize' button on the gui is pressed
project5_visualize(roger)
Robot* roger;
{
    
	int i, j, xbin, ybin, already_used[NYBINS][NXBINS];
	double compute_gradient(), mag, grad[2], x, y;
	//void draw_roger(), draw_object(), draw_frames(), mark_used(), draw_history();
	
	//printf("Project 5 visualize called. \n");
    
	// make sure it converged
	sor(roger);
	
	// initialize auxilliary structure for controlling the
	// density of streamlines rendered
	for (i=0; i<NYBINS; ++i) {
		for (j=0; j<NXBINS; ++j) {
			already_used[i][j] = FALSE;
		}
	}
	
    
	// If [row,col] is FREESPACE and at least one of its neighbors
	// is OBSTACLE, then draw a streamline
	for (i=1;i<(NYBINS-1);i+=1) {
		for (j=1;j<(NXBINS-1);j+=1) {
			if ((roger->world_map.occupancy_map[i][j] == FREESPACE) &&
				((roger->world_map.occupancy_map[i-1][j-1] == OBSTACLE) ||
				 (roger->world_map.occupancy_map[i-1][j] == OBSTACLE)   ||
				 (roger->world_map.occupancy_map[i-1][j+1] == OBSTACLE) ||
				 (roger->world_map.occupancy_map[i][j-1] == OBSTACLE)   ||
				 (roger->world_map.occupancy_map[i][j+1] == OBSTACLE)   ||
				 (roger->world_map.occupancy_map[i+1][j-1] == OBSTACLE) ||
				 (roger->world_map.occupancy_map[i+1][j] == OBSTACLE)   ||
				 (roger->world_map.occupancy_map[i+1][j+1] == OBSTACLE) ||
				 
				 (roger->world_map.occupancy_map[i-1][j-1] == DILATED_OBSTACLE) ||
				 (roger->world_map.occupancy_map[i-1][j] == DILATED_OBSTACLE)   ||
				 (roger->world_map.occupancy_map[i-1][j+1] == DILATED_OBSTACLE) ||
				 (roger->world_map.occupancy_map[i][j-1] == DILATED_OBSTACLE)   ||
				 (roger->world_map.occupancy_map[i][j+1] == DILATED_OBSTACLE)   ||
				 (roger->world_map.occupancy_map[i+1][j-1] == DILATED_OBSTACLE) ||
				 (roger->world_map.occupancy_map[i+1][j] == DILATED_OBSTACLE)   ||
				 (roger->world_map.occupancy_map[i+1][j+1] == DILATED_OBSTACLE) ) )
			{
				
				// follow a stream line
				x = MIN_X + (j+0.5)*XDELTA;
				y = MAX_Y - (i+0.5)*YDELTA;
				ybin = i; xbin = j;
				
				if (!already_used[ybin][xbin]) {
					int loops = 0;
					while ((roger->world_map.occupancy_map[ybin][xbin] != GOAL) &&
						   (loops++ < 1000)) {
						mag = compute_gradient(x, y, roger, grad);
						if (mag < THRESHOLD) {
							//TODO: Prevent uninitialized harmonic map to try to print stream
							//printf("gradient magnitude is too small %6.4lf\n", mag);
						}
						else {
							x_draw_line(GOAL_COLOR, x, y, x-STEP*grad[0], y-STEP*grad[1]);
                            
							x -= STEP*grad[0];
							y -= STEP*grad[1];
							
							ybin = (int)((MAX_Y-y)/YDELTA);
							xbin = (int)((x-MIN_X)/XDELTA);
						}
					}
					mark_used((i+1), (j+1), already_used);
				}
			}
		}
	}
}


