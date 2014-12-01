#ifndef __ROB550_UTIL_H__
#define __ROB550_UTIL_H__

#include "vx/vx.h"
#include "common/pg.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct rob550_default_implementation rob550_default_implementation_t;

rob550_default_implementation_t *
rob550_default_implementation_create (vx_world_t *world, vx_event_handler_t *vxeh);

void
rob550_default_display_started (vx_application_t *app, vx_display_t *disp);

void
rob550_default_display_finished (vx_application_t *app, vx_display_t *disp);

void
rob550_init (int argc, char *argv[]);

void
rob550_gui_run (vx_application_t *app, parameter_gui_t *pg, int w, int h);

int
rob550_set_camera (vx_application_t *app, const float eye[3], const float lookat[3], const float up[3]);

#ifdef __cplusplus
}
#endif

#endif // __ROB550_UTIL_H__
