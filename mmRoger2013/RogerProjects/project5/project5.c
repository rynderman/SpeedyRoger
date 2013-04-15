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
    
    //state = primitive4(roger);
    
    
    
    // tells the velocity controller what velocity to set, calls sor(roger)
    control_velocity(roger);
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
    
	printf("Dilating obstacles...\n");
    
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
    printf("Total: %d obstacles, %d freespaces marked as dilated obstacles.\n", num_obs, num_dilate);
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
        
		printf("%d : %4.3f, %4.3f -> %d, %d \n", i, search_locations[i][X], search_locations[i][Y], xbin, ybin );
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
    
	if (sor_count > 1)
		printf("completed harmonic function --- %d iterations\n", sor_count);
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

#define CONTROL_STEP 2.0

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
//	draw_room(roger);
	
	//dilate the obstacles
	dilate_obstacles(roger);
	
	//init the search locations
	init_search_locations(roger);
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
#define STEP 0.3 // STEP in meters along path
//#define GOAL_X 20.0 // meters
//#define GOAL_Y 20.0 //sin(GOAL_X) // meters

// Represents the starting velocity at which this algorithm is run
//const double CURRENT_V = 0.0;

// Max curve is the sharpest turn you can do and still have longitudinal velocity
//const double MAX_CURVE = 0.08727f; // radians, 5 degrees
const double MAX_CURVE = 3.14159; // radians, 5 degrees

// Min curve is the sharpest turn you can do at max velocity
//const double MIN_CURVE = 0.01745f; // radians, 1 degree
const double MIN_CURVE = 0.0; // radians, 1 degree

// Some selected max v
const double MAX_V = 10.0f; // meters/second

// Best safe performance for motors
const double MAX_A = 0.33f; // m/s^2


void smooth(double *vel_g_cu, int size, double a){
    int i;
    for (i = 0; i < size-1; i++){
        if (vel_g_cu[i]+a < vel_g_cu[i+1])
            vel_g_cu[i+1] = vel_g_cu[i]+a;
    }
    for (i = size-1; i > 0; i--){
        if (vel_g_cu[i-1] > vel_g_cu[i]+a)
            vel_g_cu[i-1] = vel_g_cu[i]+a;
    }
}

control_velocity(roger)
Robot* roger;
{
    int xbin, ybin, already_used[NXBINS][NYBINS];
	double compute_gradient(), mag, grad[2], x, y;
	double headings[1000];
    
    int index = 0;
    
    // Find stat position
    sor(roger);
	
	// initialize auxilliary structure for controlling the
	// density of streamlines rendered
    x = roger->base_position[X];
    y = roger->base_position[Y];
    
    ybin = (int)((MAX_Y - y)/YDELTA);
    xbin = (int)((x - MIN_X)/XDELTA);
    
    // Compute Headings
    while (roger->world_map.occupancy_map[ybin][xbin] != GOAL) {
        
        // get the gradient
        mag = compute_gradient(x, y, roger, grad);
        printf("mag: %f\n", mag);

        
        if (mag < THRESHOLD) {
            break;
        }
        
        // set heading
        headings[index] = atan2(grad[0], grad[1]);
        
        // go along the path
        x -= STEP*grad[0];
        y -= STEP*grad[1];
        
        // find the bin we're in
        ybin = (int)((MAX_Y-y)/YDELTA);
        xbin = (int)((x-MIN_X)/XDELTA);
        
        index++;

        printf("grad: %f\t%f\n", grad[0], grad[1]);
        printf("bin: %d\t%d\n", xbin, ybin);
        printf("pos: %f\t%f\n\n", x, y);
    }

    if (mag) {
        // Compute change in headings
        double change[index];
        int i=0;
        for (i = 0; i < index - 1; i++) {
            change[i] = fabs(headings[i+1] - headings[i]);
        }
        // Compute the Max allowed velocity
        double velocity[index];
        for (i = 0; i < index; i++) {
            if (change[i] < MIN_CURVE) {
                velocity[i] = MAX_V;
            }else if(change[i] > MAX_CURVE){
                velocity[i] = 0.0f;
            }else{
                // Linear increase should be the square of change
                velocity[i] = MAX_V*(change[i]/MAX_CURVE);
            }
        }
        // Set the current longitudinal velocity
        velocity[0] = sqrt((roger->base_velocity[X]*roger->base_velocity[X])+(roger->base_velocity[Y]*roger->base_velocity[Y]));
        // Set the ending velocity
        velocity[index-1] = 0.0f;
        
        // Smooth the velocities
        smooth(velocity, index, MAX_A);
        
        // set the velocity to velocity[0];
        printf("Go %f m/s, thanks\n", velocity[0]);
    }
}

//use cartesian space input from mouse including mouse button info
project5_cartesian_input(roger, x, y, button)
Robot* roger;
double x;		//x value
double y;		//y value
int button;		//mouse button
{
    
	printf("Project 5 input - x: %4.3f, y: %4.3f - button: %d\n", x, y, button);
    
}
 
// this procedure can be used to prompt for and read user customized input values
project5_enter_params()
{
	printf("Project 5 enter_params called. \n");
    
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
	
	printf("Project 5 visualize called. \n");

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


