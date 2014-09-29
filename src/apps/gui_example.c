#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <math.h>

// core api
#include "vx/vx.h"
#include "vx/vx_util.h"
#include "vx/vx_remote_display_source.h"
#include "vx/gtk/vx_gtk_display_source.h"

// drawables
#include "vx/vxo_drawables.h"

// common
#include "common/getopt.h"
#include "common/pg.h"
#include "common/zarray.h"

// imagesource
#include "imagesource/image_u32.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "rob550_util.h"    // This is where a lot of the internals live

// It's good form for every application to keep its state in a struct.
typedef struct state state_t;
struct state {
    bool running;

    getopt_t        *gopt;
    parameter_gui_t *pg;

    // image stuff
    char *img_url;
    int   img_height;
    int   img_width;

    // vx stuff
    vx_application_t    vxapp;
    vx_world_t         *vxworld;      // where vx objects are live
    vx_event_handler_t *vxeh; // for getting mouse, key, and touch events
    vx_mouse_event_t    last_mouse_event;

    // threads
    pthread_t animate_thread;

    // for accessing the arrays
    pthread_mutex_t mutex;
};


// === Parameter listener =================================================
// This function is handed to the parameter gui (via a parameter listener)
// and handles events coming from the parameter gui. The parameter listener
// also holds a void* pointer to "impl", which can point to a struct holding
// state, etc if need be.
static void
my_param_changed (parameter_listener_t *pl, parameter_gui_t *pg, const char *name)
{
    if (0==strcmp ("sl1", name))
        printf ("sl1 = %f\n", pg_gd (pg, name));
    else if (0==strcmp ("sl2", name))
        printf ("sl2 = %d\n", pg_gi (pg, name));
    else if (0==strcmp ("cb1", name) || 0==strcmp ("cb2", name))
        printf ("%s = %d\n", name, pg_gb (pg, name));
    else
        printf ("%s changed\n", name);
}

static int
mouse_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
{
    state_t *state = vxeh->impl;

    // vx_camera_pos_t contains camera location, field of view, etc
    // vx_mouse_event_t contains scroll, x/y, and button click events

    if ((mouse->button_mask & VX_BUTTON1_MASK) &&
        !(state->last_mouse_event.button_mask & VX_BUTTON1_MASK)) {

        vx_ray3_t ray;
        vx_camera_pos_compute_ray (pos, mouse->x, mouse->y, &ray);

        double ground[3];
        vx_ray3_intersect_xy (&ray, 0, ground);

        printf ("Mouse clicked at coords: [%8.3f, %8.3f]  Ground clicked at coords: [%6.3f, %6.3f]\n",
                mouse->x, mouse->y, ground[0], ground[1]);
    }

    // store previous mouse event to see if the user *just* clicked or released
    state->last_mouse_event = *mouse;

    return 0;
}

static int
key_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_key_event_t *key)
{
    //state_t *state = vxeh->impl;
    return 0;
}

static int
touch_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_touch_event_t *mouse)
{
    return 0; // Does nothing
}

// === Your code goes here ================================================
// The render loop handles your visualization updates. It is the function run
// by the animate_thread. It periodically renders the contents on the
// vx world contained by state
void *
animate_thread (void *data)
{
    const int fps = 60;
    state_t *state = data;

    // Set up the imagesource
    image_source_t *isrc = image_source_open (state->img_url);

    if (isrc == NULL)
        printf ("Error opening device.\n");
    else {
        // Print out possible formats. If no format was specified in the
        // url, then format 0 is picked by default.
        // e.g. of setting the format parameter to format 2:
        //
        // --url=dc1394://bd91098db0as9?fidx=2
        for (int i = 0; i < isrc->num_formats (isrc); i++) {
            image_source_format_t ifmt;
            isrc->get_format (isrc, i, &ifmt);
            printf ("%3d: %4d x %4d (%s)\n",
                    i, ifmt.width, ifmt.height, ifmt.format);
        }
        isrc->start (isrc);
    }

    // Continue running until we are signaled otherwise. This happens
    // when the window is closed/Ctrl+C is received.
    while (state->running) {

        // Get the most recent camera frame and render it to screen.
        if (isrc != NULL) {
            image_source_data_t *frmd = calloc (1, sizeof(*frmd));
            int res = isrc->get_frame (isrc, frmd);
            if (res < 0)
                printf ("get_frame fail: %d\n", res);
            else {
                // Handle frame
                image_u32_t *im = image_convert_u32 (frmd);
                if (im != NULL) {
                    vx_object_t *vim = vxo_image_from_u32(im,
                                                          VXO_IMAGE_FLIPY,
                                                          VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);

                    // render the image centered at the origin and at a normalized scale of +/-1 unit in x-dir
                    const double scale = 2./im->width;
                    vx_buffer_add_back (vx_world_get_buffer (state->vxworld, "image"),
                                        vxo_chain (vxo_mat_scale3 (scale, scale, 1.0),
                                                   vxo_mat_translate3 (-im->width/2., -im->height/2., 0.),
                                                   vim));
                    vx_buffer_swap (vx_world_get_buffer (state->vxworld, "image"));
                    image_u32_destroy (im);
                }
            }
            fflush (stdout);
            isrc->release_frame (isrc, frmd);
        }

        // Example rendering of vx primitives
        double rad = (vx_util_mtime () % 5000) * 2. * M_PI / 5e3;   // [ 0, 2PI]
        double osc = ((vx_util_mtime () % 5000) / 5e3) * 2. - 1;    // [-1, 1]

        // Creates a blue box and applies a series of rigid body transformations
        // to it. A vxo_chain applies its arguments sequentially. In this case,
        // then, we rotate our coordinate frame by rad radians, as determined
        // by the current time above. Then, the origin of our coordinate frame
        // is translated 0 meters along its X-axis and 0.5 meters along its
        // Y-axis. Finally, a 0.1 x 0.1 x 0.1 cube (or box) is rendered centered at the
        // origin, and is rendered with the blue mesh style, meaning it has
        // solid, blue sides.
        vx_object_t *vxo_sphere = vxo_chain (vxo_mat_rotate_z (rad),
                                             vxo_mat_translate2 (0, 0.5),
                                             vxo_mat_scale (0.1),
                                             vxo_sphere (vxo_mesh_style (vx_blue)));

        // Then, we add this object to a buffer awaiting a render order
        vx_buffer_add_back (vx_world_get_buffer (state->vxworld, "rot-sphere"), vxo_sphere);

        // Now we will render a red box that translates back and forth. This
        // time, there is no rotation of our coordinate frame, so the box will
        // just slide back and forth along the X axis. This box is rendered
        // with a red line style, meaning it will appear as a red wireframe,
        // in this case, with lines 2 px wide at a scale of 0.1 x 0.1 x 0.1.
        vx_object_t *vxo_square = vxo_chain (vxo_mat_translate2 (osc, 0),
                                             vxo_mat_scale (0.1),
                                             vxo_box (vxo_lines_style (vx_red, 2)));

        // We add this object to a different buffer so it may be rendered
        // separately if desired
        vx_buffer_add_back (vx_world_get_buffer (state->vxworld, "osc-square"), vxo_square);


        // Draw a default set of coordinate axes
        vx_object_t *vxo_axe = vxo_chain (vxo_mat_scale (0.1), // 10 cm axes
                                          vxo_axes ());
        vx_buffer_add_back (vx_world_get_buffer (state->vxworld, "axes"), vxo_axe);


        // Now, we update both buffers
        vx_buffer_swap (vx_world_get_buffer (state->vxworld, "rot-sphere"));
        vx_buffer_swap (vx_world_get_buffer (state->vxworld, "osc-square"));
        vx_buffer_swap (vx_world_get_buffer (state->vxworld, "axes"));

        usleep (1000000/fps);
    }

    if (isrc != NULL)
        isrc->stop (isrc);

    return NULL;
}

state_t *
state_create (void)
{
    state_t *state = calloc (1, sizeof(*state));

    state->vxworld = vx_world_create ();
    state->vxeh = calloc (1, sizeof(*state->vxeh));
    state->vxeh->key_event = key_event;
    state->vxeh->mouse_event = mouse_event;
    state->vxeh->touch_event = touch_event;
    state->vxeh->dispatch_order = 100;
    state->vxeh->impl = state; // this gets passed to events, so store useful struct here!

    state->vxapp.display_started = rob550_default_display_started;
    state->vxapp.display_finished = rob550_default_display_finished;
    state->vxapp.impl = rob550_default_implementation_create (state->vxworld, state->vxeh);

    state->running = 1;

    return state;
}

void
state_destroy (state_t *state)
{
    if (!state)
        return;

    free (state->vxeh);
    getopt_destroy (state->gopt);
    pg_destroy (state->pg);
    free (state);
}

// This is intended to give you a starting point to work with for any program
// requiring a GUI. This handles all of the GTK and vx setup, allowing you to
// fill in the functionality with your own code.
int
main (int argc, char *argv[])
{
    rob550_init (argc, argv);
    state_t *state = state_create ();

    // Parse arguments from the command line, showing the help
    // screen if required
    state->gopt = getopt_create ();
    getopt_add_bool   (state->gopt,  'h', "help", 0, "Show help");
    getopt_add_bool   (state->gopt,  'l', "list", 0, "Lists available camera URLs and exit");
    getopt_add_string (state->gopt, '\0', "url", "", "Camera URL");

    if (!getopt_parse (state->gopt, argc, argv, 1) || getopt_get_bool (state->gopt, "help")) {
        printf ("Usage: %s [--url=CAMERAURL] [other options]\n\n", argv[0]);
        getopt_do_usage (state->gopt);
        exit (EXIT_FAILURE);
    }

    // Set up the imagesource. This looks for a camera url specified on
    // the command line and, if none is found, enumerates a list of all
    // cameras imagesource can find and picks the first url it finds.
    if (strncmp (getopt_get_string (state->gopt, "url"), "", 1)) {
        state->img_url = strdup (getopt_get_string (state->gopt, "url"));
        printf ("URL: %s\n", state->img_url);
    }
    else {
        // No URL specified. Show all available and then use the first
        zarray_t *urls = image_source_enumerate ();
        printf ("Cameras:\n");
        for (int i = 0; i < zarray_size (urls); i++) {
            char *url;
            zarray_get (urls, i, &url);
            printf ("  %3d: %s\n", i, url);
        }

        if (0==zarray_size (urls)) {
            printf ("Found no cameras.\n");
            return -1;
        }

        zarray_get (urls, 0, &state->img_url);
    }

    if (getopt_get_bool (state->gopt, "list"))
        exit (EXIT_SUCCESS);

    // Initialize this application as a remote display source. This allows
    // you to use remote displays to render your visualization. Also starts up
    // the animation thread, in which a render loop is run to update your display.
    vx_remote_display_source_t *cxn = vx_remote_display_source_create (&state->vxapp);

    // Initialize a parameter gui
    state->pg = pg_create ();
    pg_add_double_slider (state->pg, "sl1", "Slider 1", 0, 100, 50);
    pg_add_int_slider    (state->pg, "sl2", "Slider 2", 0, 100, 25);
    pg_add_check_boxes (state->pg,
                        "cb1", "Check Box 1", 0,
                        "cb2", "Check Box 2", 1,
                        NULL);
    pg_add_buttons (state->pg,
                    "but1", "Button 1",
                    "but2", "Button 2",
                    "but3", "Button 3",
                    NULL);

    parameter_listener_t *my_listener = calloc (1, sizeof(*my_listener));
    my_listener->impl = state;
    my_listener->param_changed = my_param_changed;
    pg_add_listener (state->pg, my_listener);

    // Launch our worker threads
    pthread_create (&state->animate_thread, NULL, animate_thread, state);

    // This is the main loop
    rob550_gui_run (&state->vxapp, state->pg, 1024, 768);

    // Quit when GTK closes
    state->running = 0;
    pthread_join (state->animate_thread, NULL);

    // Cleanup
    free (my_listener);
    state_destroy (state);
    vx_remote_display_source_destroy (cxn);
    vx_global_destroy ();
}
