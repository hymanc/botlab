#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <math.h>
#include <lcm/lcm.h>

#include "vx/vx.h"
#include "vx/vxo_drawables.h"
#include "vx/vx_remote_display_source.h"

#include "common/getopt.h"
#include "common/timestamp.h"

#include "math/matd.h"
#include "math/math_util.h"
#include "math/gsl_util_vector.h"
#include "math/gsl_util_matrix.h"
#include "math/gsl_util_eigen.h"
#include "math/gsl_util_blas.h"
#include "math/homogenous.h"

#include "imagesource/image_util.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "lcmtypes/maebot_diff_drive_t.h"
#include "lcmtypes/maebot_laser_t.h"
#include "lcmtypes/maebot_leds_t.h"
#include "lcmtypes/maebot_sensor_data_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"
#include "lcmtypes/pose_xyt_t.h"
#include "lcmtypes/rplidar_laser_t.h"

#include "xyt.h"

#define JOYSTICK_REVERSE_SPEED1 -0.25f
#define JOYSTICK_FORWARD_SPEED1  0.35f

#define JOYSTICK_REVERSE_SPEED2 -0.35f
#define JOYSTICK_FORWARD_SPEED2  0.45f

#define MAX_REVERSE_SPEED -0.35f
#define MAX_FORWARD_SPEED  0.35f

#define VXO_GRID_SIZE 0.25 // [m]

#define SLIDING_TIME_WINDOW 10000000 // 10 s

#define GOAL_RADIUS 0.10 // [m]

#define dmax(A,B) A < B ? B : A
#define dmin(A,B) A < B ? A : B

//#define NO_ELLIPSE

typedef struct ellipse ellipse_t;
struct ellipse
{
	double x;
	double y;
	double t;
	double axis1;  // 
	double axis2;  // 
	double tcovar; // Theta covariance
};

typedef struct state state_t;

struct state 
{
    bool running;
    getopt_t *gopt;
    char *url;
    image_source_t *isrc;
    int fidx;
    lcm_t *lcm;

    // pose
    pose_xyt_t *pose;
    zarray_t* past_poses;

    // lidar
    rplidar_laser_t *lidar;

    // covariance ellipse
    ellipse_t* ellipse;
    zarray_t* past_ellipses;
    uint32_t last_poop_idx;
    uint8_t render_ellipses;

    pthread_t command_thread;
    maebot_diff_drive_t cmd;
    bool manual_control;

    pthread_t render_thread;
    bool have_goal;
    double goal[3];

	// vx
    vx_world_t *vw;
    vx_application_t app;
    vx_event_handler_t veh;
    zhash_t *layer_map; // <display, layer>

	// mutex
    pthread_mutex_t mutex;
};


void compute_sigma_ellipse(state_t *state);



/**
 * @brief Display stop handler
 */
static void display_finished (vx_application_t *app, vx_display_t *disp)
{
    state_t *state = app->impl;

    pthread_mutex_lock (&state->mutex);
    {
        vx_layer_t *layer = NULL;
        zhash_remove (state->layer_map, &disp, NULL, &layer);
        vx_layer_destroy (layer);
    }
    pthread_mutex_unlock (&state->mutex);
}

/**
 * @brief Display startup handler
 */
static void display_started (vx_application_t *app, vx_display_t *disp)
{
    state_t *state = app->impl;

    vx_layer_t *layer = vx_layer_create (state->vw);
    vx_layer_set_display (layer, disp);
    vx_layer_add_event_handler (layer, &state->veh);

    vx_layer_camera_op (layer, OP_PROJ_PERSPECTIVE);
    float eye[3]    = {  0,  0,  5};
    float lookat[3] = {  0,  0,  0 };
    float up[3]     = {  0,  1,  0 };
    vx_layer_camera_lookat (layer, eye, lookat, up, 1);

    pthread_mutex_lock (&state->mutex);
    {
        zhash_put (state->layer_map, &disp, &layer, NULL, NULL);
    }
    pthread_mutex_unlock (&state->mutex);
}

/**
 * @brief Vx touch handler
 */
static int touch_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_touch_event_t *mouse)
{
    return 0;
}

/**
 * @brief Vx mouse handler
 */
static int mouse_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
{
    state_t *state = vh->impl;

    // Button state
    bool m1 = mouse->button_mask & VX_BUTTON1_MASK;
    bool ctrl = mouse->modifiers & VX_CTRL_MASK;
    //bool alt = mouse->modifiers & VX_ALT_MASK;
    //bool shift = mouse->modifiers & VX_SHIFT_MASK;

    pthread_mutex_lock (&state->mutex);
    {
        if (m1 && ctrl) {
            // Ray cast to find click point
            vx_ray3_t ray;
            vx_camera_pos_compute_ray (pos, mouse->x, mouse->y, &ray);

            double ground[3];
            vx_ray3_intersect_xy (&ray, 0, ground);

            printf ("Mouse clicked at coords: [%8.3f, %8.3f]  Ground clicked at coords: [%6.3f, %6.3f]\n",
                    mouse->x, mouse->y, ground[0], ground[1]);

            state->goal[0] = ground[0];
            state->goal[1] = ground[1];
            state->have_goal = true;
        }
    }
    pthread_mutex_unlock (&state->mutex);

    return 0;
}

/**
 * @brief Vx Key Handler
 */
static int key_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_key_event_t *key)
{
    state_t *state = vh->impl;

    static bool key_shift=0, key_up=0, key_down=0, key_left=0, key_right=0;

    switch (key->key_code) {
        case VX_KEY_SHIFT:
            key_shift = !key->released;
            break;
        case VX_KEY_UP:
            key_up = !key->released;
            break;
        case VX_KEY_DOWN:
            key_down = !key->released;
            break;
        case VX_KEY_LEFT:
            key_left = !key->released;
            break;
        case VX_KEY_RIGHT:
            key_right = !key->released;
            break;
        case VX_KEY_CTRL:
            state->manual_control = !key->released;
            if (key->released)
                state->cmd.motor_left_speed = state->cmd.motor_right_speed = 0.0;
            break;
        default:
            break;
    }

    if (state->manual_control) {
        pthread_mutex_lock (&state->mutex);
        {
            // default to zero
            state->cmd.motor_left_speed = state->cmd.motor_right_speed = 0.0;

            float fwd_speed = JOYSTICK_FORWARD_SPEED1;
            float rev_speed = JOYSTICK_REVERSE_SPEED1;
            if (key_shift) { // speed boost
                fwd_speed = JOYSTICK_FORWARD_SPEED2;
                rev_speed = JOYSTICK_REVERSE_SPEED2;
            }

            if (key_up) { // forward
                state->cmd.motor_left_speed = fwd_speed;
                state->cmd.motor_right_speed = fwd_speed;
                if (key_left) {
                    state->cmd.motor_left_speed -= 0.1;
                    state->cmd.motor_right_speed += 0.1;
                }
                else if (key_right) {
                    state->cmd.motor_left_speed += 0.1;
                    state->cmd.motor_right_speed -= 0.1;
                }
            }
            else if (key_down) { // reverse
                state->cmd.motor_left_speed = rev_speed;
                state->cmd.motor_right_speed = rev_speed;
                if (key_left) {
                    state->cmd.motor_left_speed += 0.1;
                    state->cmd.motor_right_speed -= 0.1;
                }
                else if (key_right) {
                    state->cmd.motor_left_speed -= 0.1;
                    state->cmd.motor_right_speed += 0.1;
                }
            }
            else if (key_left) { // turn left
                state->cmd.motor_left_speed =  rev_speed;
                state->cmd.motor_right_speed = -rev_speed;
            }
            else if (key_right) { // turn right
                state->cmd.motor_left_speed = -rev_speed;
                state->cmd.motor_right_speed = rev_speed;
            }
        }
        pthread_mutex_unlock (&state->mutex);
    }

    return 0;
}

static void destroy (vx_event_handler_t *vh)
{
    // do nothing, since this event handler is statically allocated.
}

static state_t *global_state;
static void handler (int signum)
{
    switch (signum) {
        case SIGINT:
        case SIGQUIT:
            global_state->running = 0;
            break;
        default:
            break;
    }
}


// This thread continuously publishes command messages to the maebot
static void * command_thread (void *data)
{
    state_t *state = data;
    const uint32_t Hz = 20;
    const char *channel = getopt_get_string (state->gopt, "maebot-diff-drive-channel");

    while (state->running) {
        pthread_mutex_lock (&state->mutex);
        {
            if (!state->manual_control && state->have_goal) {
                // TODO: IMPLEMENT ME

            }
            // Publish
            state->cmd.utime = utime_now ();
            maebot_diff_drive_t_publish (state->lcm, channel, &(state->cmd));
        }
        pthread_mutex_unlock (&state->mutex);

        usleep (1000000/Hz);
    }

    return NULL;
}

// TODO: don't add new poses if they're not far enough away or theta isn't different enough

/*
 * @brief absolute value fn for int64_t
 */
 int64_t abs_64(int64_t A)
 {
	if ( A < 0 )
	{
		A *= -1;
	}

	return A;
 }

// This thread continously renders updates from the robot
/**
 * @brief Rendering Thread
 * @param data State struct argument
 * @return NULL
 */
static void * render_thread (void *data)
{
    state_t *state = data;
    float ellipse_color[4] = {0.0, 0.0, 1.0, 0.5};
    // Grid
        vx_buffer_t *vbgrid = vx_world_get_buffer (state->vw, "grid");
        vx_buffer_set_draw_order (vbgrid, 0);
        vx_buffer_add_back (vbgrid,
                            vxo_chain (vxo_mat_scale (VXO_GRID_SIZE),
                                       vxo_grid ()));
        vx_buffer_swap (vbgrid);

    // Axes
        vx_buffer_t *vbaxes = vx_world_get_buffer (state->vw, "axes");
        vx_buffer_set_draw_order (vbaxes, 0);
        vx_buffer_add_back (vbaxes,
                            vxo_chain (vxo_mat_scale3 (0.10, 0.10, 0.0),
                                       vxo_mat_translate3 (0.0, 0.0, -0.005),
                                       vxo_axes_styled (vxo_mesh_style (vx_red),
                                                        vxo_mesh_style (vx_green),
                                                        vxo_mesh_style (vx_black))));
        vx_buffer_swap (vbaxes);

    const int fps = 30;
    while (state->running) 
    {
        pthread_mutex_lock (&state->mutex);

            // Goal
            if (state->have_goal) 
	    	{
                float color[4] = {0.0, 1.0, 0.0, 0.5};
                vx_buffer_t *vbgoal = vx_world_get_buffer (state->vw, "goal");
                vx_buffer_set_draw_order (vbgoal, -1);
                vx_buffer_add_back (vbgoal,
                                    vxo_chain (vxo_mat_translate2 (state->goal[0], state->goal[1]),
                                               vxo_mat_scale (GOAL_RADIUS),
                                               vxo_circle (vxo_mesh_style (color))));
                vx_buffer_swap (vbgoal);
            }

            // Robot

                vx_buffer_t *vbrobot = vx_world_get_buffer (state->vw, "robot");
                //vx_buffer_set_draw_order (vbrobot, 1);
                enum {ROBOT_TYPE_TRIANGLE, ROBOT_TYPE_DALEK};
                vx_object_t *robot = NULL;
                switch (ROBOT_TYPE_DALEK) 
				{
                    case ROBOT_TYPE_DALEK: 
		    		{
                        float nose[6] = {0.0, 0.0, 0.151, 0.104, 0.0, 0.151};
                        robot = vxo_chain (	vxo_lines (	vx_resc_copyf (nose, 6),
														2,
														GL_LINES,
														vxo_lines_style (vx_red, 3.0f)),
											vxo_mat_scale3 (0.104, 0.104, 0.151),
                                           	vxo_mat_translate3 (0.0, 0.0, 0.5),
                                           	vxo_cylinder (vxo_mesh_style (vx_blue)));
                        break;
                    }
                    case ROBOT_TYPE_TRIANGLE:
                    default:
                        robot = vxo_chain (vxo_mat_scale (0.104),
                                           vxo_mat_scale3 (1, 0.5, 1),
                                           vxo_triangle (vxo_mesh_style (vx_blue)));
                        break;
                }

					// Line
					int length = 2 * 3 * zarray_size(state->past_poses);
					float line[length];
					if(length >= 3)
					{
						line[0] = 0.0; // initial x
						line[1] = 0.0; // inital y
						line[2] = 0.0; // initial z
					}
					pose_xyt_t* last_pose = (pose_xyt_t*)malloc(sizeof(pose_xyt_t));
					int i, j=3;
					for (i=0; i < zarray_size(state->past_poses); i++)
					{
						zarray_get(state->past_poses, i, last_pose);
						line[j] = (last_pose->xyt)[0]; // next x
						j++;
						line[j] = (last_pose->xyt)[1]; // next y
						j++;
						line[j] = 0.0; // all z's are 0
						j++;
						// double up the end points
						if(i != (zarray_size(state->past_poses) - 1))
						{
							line[j] = (last_pose->xyt)[0];
							j++;
							line[j] = (last_pose->xyt)[1];
							j++;
							line[j] = 0;
							j++;
						}
					}
					free(last_pose);

                if (state->pose)
				{
                    vx_buffer_add_back (vbrobot, vxo_chain (vxo_lines (vx_resc_copyf (line, length),
                                                      length/3,
                                                      GL_LINES,
                                                      vxo_lines_style (vx_purple, 3.0f))));
					vx_buffer_add_back (vbrobot, vxo_mat_from_xyt (state->pose->xyt));
					vx_buffer_add_back (vbrobot, robot);
					vx_buffer_swap(vbrobot);
					printf("x: %lf, y: %lf, t: %lf\n", state->pose->xyt[0], state->pose->xyt[1], state->pose->xyt[2]);
				}
                //else
                    //vx_buffer_add_back(vbrobot, robot);
               // vx_buffer_swap(vbrobot);

               
		compute_sigma_ellipse(state);

        //Ellipse
		vx_buffer_t *vbellipse = vx_world_get_buffer(state->vw, "Ellipse");
		//vx_buffer_set_draw_order(vbellipse, 1);

		vx_object_t *ellipse = vxo_chain(vxo_mat_scale3 
                                ( state->ellipse->axis1, 
                                  state->ellipse->axis2, 
                                  state->ellipse->tcovar), //
						 vxo_circle (vxo_mesh_style (ellipse_color)));

        float covtline[6] = { 0.0, 0.0, 0.0, 
                              0.0, 0.0, state->ellipse->t};
        vx_object_t *covt  = vxo_chain (	vxo_lines (	vx_resc_copyf (covtline, 6),
														2,
														GL_LINES,
														vxo_lines_style (vx_orange, 3.0f)));
		if(state->pose)
		{
		    vx_buffer_add_back(vbellipse, vxo_chain
                                            ( vxo_mat_translate2
                                                ( state->ellipse->x, state->ellipse->y ),
                                              vxo_mat_rotate_z( state->ellipse->t ),
                                              ellipse, covt));
		    vx_buffer_swap(vbellipse);
		}
	
        // Ellipse trail
        #ifndef NO_ELLIPSE
        vx_buffer_t *vbellipsetrail = vx_world_get_buffer(state->vw, "ellipsetrail");
        ellipse_t* cur_ellipse = malloc(sizeof(ellipse_t));
        int n;
        for(n = 0; n < zarray_size(state->past_ellipses); n++)
        {
           zarray_get(state->past_ellipses, n, cur_ellipse);
           vx_object_t *cur_ellipse_obj = vxo_chain ( vxo_mat_scale3
                                                        ( cur_ellipse->axis1,
                                                          cur_ellipse->axis2,
                                                          cur_ellipse->tcovar),
                                                      vxo_circle 
                                                        ( vxo_lines_style (vx_black, 3.0f )));
           vx_buffer_add_back (vbellipsetrail, vxo_chain
                                                ( vxo_mat_translate2
                                                    ( cur_ellipse->x, cur_ellipse->y ),
                                                  vxo_mat_rotate_z ( cur_ellipse->t ),
                                                  cur_ellipse_obj));
        }
        vx_buffer_swap(vbellipsetrail);
        free(cur_ellipse);
        #endif
		// Current Lidar Scan
		// Lidar
		int num_points = state->lidar->nranges;
		if (num_points)
		{
		// Lidar for one pose
			int64_t diff = INT_MAX;
			// find the pose closest to the lidar acquisition
			int k, pose_idx = zarray_size(state->past_poses) - 1;
			pose_xyt_t* cur_p = (pose_xyt_t*)malloc(sizeof(pose_xyt_t));
			printf("%d stored poses\n", zarray_size(state->past_poses));
			for (k = zarray_size(state->past_poses) - 1; k >= 0; k--)
			{
				zarray_get(state->past_poses, k, cur_p);
				int64_t d = abs_64(state->lidar->utime - cur_p->utime);
				if ( d < diff )
				{
					diff = d;
					pose_idx = k;
				}
				else
				{
					break;
				}
			}
					
			float *points = malloc ( (3 * num_points) * sizeof(float));

			if (pose_idx >= 0)
			{
				printf("using pose_idx: %d\n", pose_idx);
				zarray_get( state->past_poses, pose_idx, cur_p);
				
				//Set x,y positions of each point based off the one pose chosen
				int l, m=0;
				printf("number of lidar points: %d\n", num_points);
				for(l = 0; l < num_points; l++)
				{
					points[m] = state->lidar->ranges[l] * cos(state->lidar->thetas[l]);
					m++;
					points[m] = -state->lidar->ranges[l] * sin(state->lidar->thetas[l]);
					m++;
					points[m] = 0;
					m++;
				}

			//render lidar dots
			vx_buffer_t *vblidaronepose = vx_world_get_buffer (state->vw, "lidaronepose");
			vx_buffer_add_back (vblidaronepose,
							vxo_chain ( vxo_mat_translate3 (cur_p->xyt[0],
															cur_p->xyt[1], 0),
										vxo_mat_rotate_z ( cur_p->xyt[2] ),
										vxo_points( vx_resc_copyf (points, 3*num_points),
													num_points, 
													vxo_points_style (vx_red, 3.0f))));
			vx_buffer_swap (vblidaronepose);
			}
            free(cur_p);

		// Lidar for interpolated poses
			int end_idx = zarray_size(state->past_poses) - 1;
			int start_idx = zarray_size(state->past_poses) - 1;
			pose_xyt_t* last_p = (pose_xyt_t*)malloc(sizeof(pose_xyt_t));
			pose_xyt_t* first_p = (pose_xyt_t*)malloc(sizeof(pose_xyt_t));
			printf("%d stored poses\n", zarray_size(state->past_poses));

			diff = INT_MAX;
			// find the pose closest to last point acquired
			for (k = zarray_size(state->past_poses) - 1; k >= 0; k--)
			{
				zarray_get(state->past_poses, k, last_p);
				int64_t d = abs_64(state->lidar->times[num_points - 1] - last_p->utime);
				if ( d < diff )
				{
					diff = d;
					end_idx = k;
				}
				else
				{
					break;
				}
			}
			// find the pose closest to first point acquired
			diff = INT_MAX;
			for (k = end_idx; k >= 0; k--)
			{
				zarray_get(state->past_poses, k, first_p);
				int64_t d = abs_64(state->lidar->times[0] - first_p->utime);
				if ( d < diff )
				{
					diff = d;
					start_idx = k;
				}
				else
				{
					break;
				}
			}

			if (end_idx >= 0 && start_idx >= 0)
			{
				printf("using end_idx: %d\n", end_idx);
				printf("using start_idx: %d\n", start_idx);
				zarray_get( state->past_poses, end_idx, last_p);
				zarray_get( state->past_poses, start_idx, first_p);
				
				double end_t= last_p->xyt[2];
				double start_t = first_p->xyt[2];
				double theta_diff = end_t - start_t;
				double t_slope = theta_diff / num_points;

				double end_x = last_p->xyt[0];
				double start_x = first_p->xyt[0];
				double x_diff = end_x - start_x;
				double x_slope = x_diff / num_points;

				double end_y = last_p->xyt[1];
				double start_y = first_p->xyt[1];
				double y_diff = end_y - start_y;
				double y_slope = y_diff / num_points;

				float *points = malloc ( (3 * num_points) * sizeof(float));
				//Set x,y positions of each point based off linear interpolation of poses
				int l, m=0;
				printf("number of lidar points: %d\n", num_points);
				for(l = 0; l < num_points; l++)
				{
					double xlid = state->lidar->ranges[l] * cos(state->lidar->thetas[l]);
					double ylid = -state->lidar->ranges[l] * sin(state->lidar->thetas[l]);
					
					double xinterp = start_x + ( l * x_slope);
					double yinterp = start_y + ( l * y_slope);
					double tinterp = start_t + ( l * t_slope);
					
					// x
					points[m] = (cos(tinterp) * xlid)
									- (sin(tinterp) * ylid) + xinterp;
					m++;
					// y
					points[m] = (sin(tinterp) * xlid)
									+ (cos(tinterp) * ylid) + yinterp;
					m++;
					// z
					points[m] = 0;
					m++;
				}

			//render lidar dots
			vx_buffer_t *vblidarinterp = vx_world_get_buffer (state->vw, "lidarinterp");
			vx_buffer_add_back (vblidarinterp,
							vxo_chain ( vxo_points( vx_resc_copyf (points, 3*num_points),
													num_points, 
													vxo_points_style (vx_green, 3.0f))));
			vx_buffer_swap (vblidarinterp);
			}
            free(last_p);
            free(first_p);
		}

        pthread_mutex_unlock (&state->mutex);
        usleep (1000000/fps);
    }

    return NULL;
}

// === LCM Handlers =================

/**
 * @brief Maebot Motor Encoder Handler
 */
static void maebot_motor_feedback_handler (const lcm_recv_buf_t *rbuf, const char *channel, const maebot_motor_feedback_t *msg, void *user)
{
    state_t *state = user;

    pthread_mutex_lock (&state->mutex);
    {
        // TODO: IMPLEMENT ME
    }
    pthread_mutex_unlock (&state->mutex);
}

/**
 * @brief Maebot IMU Data handler
 */
static void maebot_sensor_data_handler (const lcm_recv_buf_t *rbuf, const char *channel, const maebot_sensor_data_t *msg, void *user)
{
    state_t *state = user;

    pthread_mutex_lock (&state->mutex);
    {
        // TODO: IMPLEMENT ME
    }
    pthread_mutex_unlock (&state->mutex);
}

/**
 * @brief X-Y-Theta Odometry/Pose Handler
 */
static void pose_xyt_handler (const lcm_recv_buf_t *rbuf, const char *channel, const pose_xyt_t *msg, void *user)

{
    state_t *state = user;

    pthread_mutex_lock (&state->mutex);
    {
	*(state->pose) = *msg; // Update current pose
	zarray_add(state->past_poses, state->pose);
	printf("X:%.3f, Y:%.3f, T:%.3f\n",msg->xyt[0], msg->xyt[1], msg->xyt[2]);
    }
    pthread_mutex_unlock (&state->mutex);
}

/**
 * @brief RP LIDAR LCM Handler
 */
static void rplidar_laser_handler (const lcm_recv_buf_t *rbuf, const char *channel, const rplidar_laser_t *msg, void *user)
{
    state_t *state = user;

    pthread_mutex_lock (&state->mutex);
    {
		*(state->lidar) = *msg; // update current lidar reading
    }
    pthread_mutex_unlock (&state->mutex);
}

/**
 * @brief State initialization routine
 * @return Initialized default state struct
 */
state_t *state_create (void)
{
    state_t *state = calloc (1, sizeof (*state));

    state->running = 1;
    state->gopt = getopt_create ();
    state->lcm = lcm_create (NULL);

	// goal
	state->have_goal = false;

	// pose
	state->pose = calloc(1, sizeof(pose_xyt_t));
	state->past_poses = zarray_create(sizeof(pose_xyt_t));

	// ellipse
	state->ellipse = calloc(1, sizeof(ellipse_t));
	state->last_poop_idx = 0;
	state->past_ellipses = zarray_create(sizeof(ellipse_t));

	// lidar
	state->lidar = calloc(1, sizeof(rplidar_laser_t));

    state->vw = vx_world_create ();
    state->app.display_finished = display_finished;
    state->app.display_started = display_started;
    state->app.impl = state;
    state->veh.dispatch_order = -10;
    state->veh.touch_event = touch_event;
    state->veh.mouse_event = mouse_event;
    state->veh.key_event = key_event;
    state->veh.destroy = destroy;
    state->veh.impl = state;
    state->layer_map = zhash_create (sizeof(vx_display_t*), sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);

    // note, pg_sd() family of functions will trigger their own callback of my_param_changed(),
    // hence using a recursive mutex avoids deadlocking when using pg_sd() within my_param_changed()
    pthread_mutexattr_t attr;
    pthread_mutexattr_init (&attr);
    pthread_mutexattr_settype (&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init (&state->mutex, &attr);

    return state;
}

/**
 * @brief Free memory used by state
 * @param state is the current state struct
 */
void state_destroy(state_t *state)
{
	zarray_destroy(state->past_poses);
	zarray_destroy(state->past_ellipses);
	free(state->pose);
	free(state->lidar);
	free(state->ellipse);
	//TODO: Everything else...
}

/*
 * @brief Computes the total distance (m) travelled between two pose indices
 *			Return the distance if the indices are valid
 *			Else returns -1
 * @param state is the current state struct
 * @param idx_1 is the index of the start pose
 * @param idx_2 is the index of the end pose
 */
 double compute_distance_travelled(state_t *state, int idx_1, int idx_2)
 {
	if ((idx_2 < idx_1) || (idx_1 < 0) || (idx_2 > zarray_size(state->past_poses) - 1))
	{
		return -1;
	}

	pose_xyt_t* cur_pose = (pose_xyt_t*)malloc(sizeof(pose_xyt_t));
	pose_xyt_t* next_pose = (pose_xyt_t*)malloc(sizeof(pose_xyt_t));
	zarray_get(state->past_poses, idx_1, cur_pose);
	double dist = 0;
	int i;
	for(i = idx_1 + 1; i < zarray_size(state->past_poses); i++)
	{
		zarray_get(state->past_poses, i, next_pose);
		double x = next_pose->xyt[0] - cur_pose->xyt[0];
		double y = next_pose->xyt[1] - cur_pose->xyt[1];
		dist += sqrt(pow(x, 2) + pow(y, 2));
	}

	return (dist);
 }

/**
 * @brief Computes the uncertainty ellipse parameters for the given state
 * @param state Current state struct
 */
void compute_sigma_ellipse(state_t *state)
{
    pthread_mutex_lock(&state->mutex);
    if(state->pose->Sigma != NULL)
    {
	gsl_matrix *sig = gsl_matrix_alloc(3,3);// Reconstruct covariance matrix
	memcpy(sig->data, state->pose->Sigma, 9*sizeof(double));
	gslu_eigen *sig_eigs = gslu_eigen_decomp_alloc (sig);// Compute Eigenvalues/vectors of covariance

	printf("Eigenstuff\n");
	gslu_vector_printf(sig_eigs->D,"Evals");
	gslu_matrix_printf(sig_eigs->V,"Evecs");
	
	gsl_vector *u1 = gsl_vector_calloc(2);
	gsl_vector_set(u1, 0, gsl_matrix_get(sig_eigs->V, 0, 1));
	gsl_vector_set(u1, 1, gsl_matrix_get(sig_eigs->V, 1, 1));
	gsl_vector *u2 = gsl_vector_calloc(2);
	gsl_vector_set(u2, 0, gsl_matrix_get(sig_eigs->V, 0, 2));
	gsl_vector_set(u2, 1, gsl_matrix_get(sig_eigs->V, 1, 2));
	
	// Store the values in state->ellipse
	// If current pose is 10cm away from last time stored, 
		// add this ellipse to the zarray state->past_ellipses
		// Should probably have a field that stores the last pose 
			// of the last ellipse that was stored or its index
			// in state->past poses or something like that

	ellipse_t* cur_ellipse = malloc(sizeof(ellipse_t));
	// Fill in it's values
	// Set state's ellipse equal to its values
	
	cur_ellipse->x = state->pose->xyt[0];
	cur_ellipse->y = state->pose->xyt[1];
	cur_ellipse->t = atan2(gsl_vector_get(u2, 0), gsl_vector_get(u2, 1));
	cur_ellipse->axis1 = 2 * sqrt(gsl_vector_get(sig_eigs->D, 1));
	cur_ellipse->axis2 = 2 * sqrt(gsl_vector_get(sig_eigs->D, 2));
	cur_ellipse->tcovar = sqrt(gsl_vector_get(sig_eigs->D, 0));

	*(state->ellipse) = *(cur_ellipse);

	// Add this ellipse to the zarray of ellipses if this pose is >= 10cm from the last poop
	double dist = compute_distance_travelled(state, state->last_poop_idx, zarray_size(state->past_poses) - 1);
	if (dist >= 0.1)
	{
		zarray_add(state->past_ellipses, cur_ellipse);
		state->last_poop_idx = zarray_size(state->past_poses) - 1;
	}
	else if (dist == -1)
	{
		printf ("Bad pose index comparison in compute_sigma_ellipse\n");
	}

	// TODO: 
	gslu_vector_free(u1);
	gslu_vector_free(u2);
	gslu_matrix_free(sig);
	gslu_eigen_free(sig_eigs);
    }
    pthread_mutex_unlock(&state->mutex);
}

/**
 * @brief main
 */
int main (int argc, char *argv[])
{
    printf("===Botlab===\n");
    printf("Section 1, Group 4\n");
    printf("Cody Hyman, Jon Kurzer, and Sarah Spitzer\n");
    
    
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    vx_global_init ();

    state_t *state = NULL;
    global_state = state = state_create ();

    // Clean up on Ctrl+C
    signal (SIGINT, handler);
    signal (SIGQUIT, handler);

    getopt_add_bool (state->gopt, 'h', "help", 0, "Show this help");
    getopt_add_int (state->gopt, 'l', "limitKBs", "-1", "Remote display bandwith limit in KBs. < 0: unlimited.");
    getopt_add_int (state->gopt, 'p', "port", "15151", "Vx display port");
    getopt_add_string (state->gopt, '\0', "maebot-motor-feedback-channel", "MAEBOT_MOTOR_FEEDBACK", "LCM channel name");
    getopt_add_string (state->gopt, '\0', "maebot-sensor-data-channel", "MAEBOT_SENSOR_DATA", "LCM channel name");
    getopt_add_string (state->gopt, '\0', "maebot-diff-drive-channel", "MAEBOT_DIFF_DRIVE", "LCM channel name");
    getopt_add_string (state->gopt, '\0', "odometry-channel", "BOTLAB_ODOMETRY", "LCM channel name");
    getopt_add_string (state->gopt, '\0', "rplidar-laser-channel", "RPLIDAR_LASER", "LCM channel name");
    //getopt_add_string (state->gopt, 'e', "no-ellipses", 0, "Ellipse Flag");

    if (!getopt_parse (state->gopt, argc, argv, 0)) 
    {
        getopt_do_usage (state->gopt);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (state->gopt,"help"))
    {
        getopt_do_usage (state->gopt);
        exit (EXIT_SUCCESS);
    }

    /*
    if (getopt_get_bool (state->gopt,"no-ellipses")) 
    {
        printf("Not rendering ellipsepoops\n");
	state->render_ellipses = 0;
    }
    else
    {
	state->render_ellipses = 1;
    }*/
    
    // Set up Vx remote display
    vx_remote_display_source_attr_t remote_attr;
    vx_remote_display_source_attr_init (&remote_attr);
    remote_attr.max_bandwidth_KBs = getopt_get_int (state->gopt, "limitKBs");
    remote_attr.advertise_name = "Maebot App";
    remote_attr.connection_port = getopt_get_int (state->gopt, "port");
    vx_remote_display_source_t *remote = vx_remote_display_source_create_attr (&state->app, &remote_attr);

    // Video stuff?

    // LCM subscriptions
    maebot_motor_feedback_t_subscribe (state->lcm,
                                       getopt_get_string (state->gopt, "maebot-motor-feedback-channel"),
                                       maebot_motor_feedback_handler, state);
    maebot_sensor_data_t_subscribe (state->lcm,
                                    getopt_get_string (state->gopt, "maebot-sensor-data-channel"),
                                    maebot_sensor_data_handler, state);
    pose_xyt_t_subscribe (state->lcm,
                          getopt_get_string (state->gopt, "odometry-channel"),
                          pose_xyt_handler, state);
    rplidar_laser_t_subscribe (state->lcm,
                               getopt_get_string (state->gopt, "rplidar-laser-channel"),
                               rplidar_laser_handler, state);

    // Launch worker threads
    pthread_create (&state->command_thread, NULL, command_thread, state);
    pthread_create (&state->render_thread, NULL, render_thread, state);

    // Loop forever
    while (state->running)
        lcm_handle_timeout (state->lcm, 500);

    pthread_join (state->command_thread, NULL);
    pthread_join (state->render_thread, NULL);

    printf ("waiting vx_remote_display_source_destroy...");
    vx_remote_display_source_destroy (remote);
    printf ("done\n");

	state_destroy(state);
}
