#ifndef __MATH_GPS_LINEARIZE_H__
#define __MATH_GPS_LINEARIZE_H__

typedef struct _gps_linearize_t gps_linearize_t;
struct _gps_linearize_t
{
    double lon0_deg, lat0_deg;
    double radius_ns, radius_ew;
};

void gps_linearize_init(gps_linearize_t *gl, const double ll_deg[2]);
int gps_linearize_to_xy(gps_linearize_t *gl, const double ll_deg[2], double xy[2]);
int gps_linearize_to_lat_lon(gps_linearize_t *gl, const double xy[2], double ll_deg[2]);

#endif // __MATH_GPS_LINEARIZE_H__
