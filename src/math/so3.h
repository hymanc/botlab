#ifndef __MATH_SO3_H__
#define __MATH_SO3_H__

#include <string.h>
#include <assert.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
% ROTX  Compute a rotation matrix about the X-axis.
%   R = ROTX(PHI) returns [3x3] rotation matrix R.  Note: PHI
%   is measured in radians.  PHI measures orientation of coordinate
%   frame 2 relative to coordinate frame 1.  Multiplication by rotation
%   matrix R rotates a vector in coordinate frame 1 into coordinate
%   frame 2.
*/
static inline void
so3_rotx (double Rx[3*3], double phi)
{
#ifdef _GNU_SOURCE
    double s, c;
    sincos (phi, &s, &c);
#else
    double s = sin (phi), c = cos (phi);
#endif

    double _Rx[3*3] = { 1,  0, 0,
                        0,  c, s,
                        0, -s, c };
    memcpy (Rx, _Rx, sizeof _Rx);
}

static inline void
so3_rotx_gsl (gsl_matrix *Rx, double phi)
{
    assert (Rx->size1==3 && Rx->size2==3 && Rx->tda==3);
    so3_rotx (Rx->data, phi);
}

/*
% ROTY  Compute a rotation matrix about the Y-axis.
%   R = ROTY(THETA) returns [3x3] rotation matrix R.  Note: THETA
%   is measured in radians.  THETA measures orientation of coordinate
%   frame 2 relative to coordinate frame 1.  Multiplication by rotation
%   matrix R rotates a vector in coordinate frame 1 into coordinate
%   frame 2.
*/
static inline void
so3_roty (double Ry[3*3], double theta)
{
#ifdef _GNU_SOURCE
    double s, c;
    sincos (theta, &s, &c);
#else
    double s = sin (theta), c = cos (theta);
#endif

    double _Ry[3*3] = { c, 0, -s,
                        0, 1,  0,
                        s, 0,  c };
    memcpy (Ry, _Ry, sizeof _Ry);
}

static inline void
so3_roty_gsl (gsl_matrix *Ry, double theta)
{
    assert (Ry->size1==3 && Ry->size2==3 && Ry->tda==3);
    so3_roty (Ry->data, theta);
}


/*
% ROTZ  Compute a rotation matrix about the Z-axis.
%   R = ROTZ(PSI) returns [3x3] rotation matrix R.  Note: PSI
%   is measured in radians.  PSI measures orientation of coordinate
%   frame 2 relative to coordinate frame 1.  Multiplication by rotation
%   matrix R rotates a vector in coordinate frame 1 into coordinate
%   frame 2.
*/
static inline void
so3_rotz (double Rz[3*3], double psi)
{
#ifdef _GNU_SOURCE
    double s, c;
    sincos (psi, &s, &c);
#else
    double s = sin (psi), c = cos (psi);
#endif

    double _Rz[3*3] = {  c, s, 0,
                        -s, c, 0,
                         0, 0, 1 };
    memcpy (Rz, _Rz, sizeof _Rz);
}

static inline void
so3_rotz_gsl (gsl_matrix *Rz, double psi)
{
    assert (Rz->size1==3 && Rz->size2==3 && Rz->tda==3);
    so3_rotz (Rz->data, psi);
}

/*
% ROTXYZ  Compute a rotation matrix about the XYZ-axes.
%   R = ROTXYZ(RPH) returns [3x3] rotation matrix R where RPH
%   is a 3-vector of Euler angles [roll,pitch,heading] measured in
%   radians.  RPH measures orientation of coordinate frame 2
%   relative to coordinate frame 1.  Multiplication by rotation
%   matrix R rotates a vector in coordinate frame 2 into coordinate
%   frame 1.
*/
static inline void
so3_rotxyz (double R[3*3], const double rph[3])
{
#ifdef _GNU_SOURCE
    double sr, sp, sh, cr, cp, ch;
    sincos (rph[0], &sr, &cr);
    sincos (rph[1], &sp, &cp);
    sincos (rph[2], &sh, &ch);
#else
    double sr = sin (rph[0]), cr = cos (rph[0]);
    double sp = sin (rph[1]), cp = cos (rph[1]);
    double sh = sin (rph[2]), ch = cos (rph[2]);
#endif

    double _R[3*3] = { ch*cp, -sh*cr + ch*sp*sr,  sh*sr + ch*sp*cr,
                       sh*cp,  ch*cr + sh*sp*sr, -ch*sr + sh*sp*cr,
                         -sp,             cp*sr,             cp*cr };
    memcpy (R, _R, sizeof _R);
}

static inline void
so3_rotxyz_gsl (gsl_matrix *R, const gsl_vector *rph)
{
    assert (R->size1==3 && R->size2==3 && rph->size==3);

    // when rph is vector view, stride might not be 1
    double rph_data[3] = {rph->data[0], rph->data[rph->stride], rph->data[2*rph->stride]};

    if (R->tda==3)
        so3_rotxyz (R->data, rph_data);
    else {
        double rot[9];
        so3_rotxyz (rot, rph_data);
        for (int i=0; i < 3; ++i)
            for (int j=0; j < 3; ++j)
                gsl_matrix_set (R, i, j, rot[i*3 + j]);
    }
}

/*
%ROT2RPH  Convert rotation matrix into Euler roll,pitch,heading.
%   RPH = ROT2RPH(R) computes 3-vector of Euler angles
%   [roll,pitch,heading] from [3x3] rotation matrix R.  Angles are
%   measured in radians.
%
*/
static inline void
so3_rot2rph (const double R[3*3], double rph[3])
{
#define R(i,j) (R[i*3+j])
    // heading
    rph[2] = atan2 (R(1,0), R(0,0));
#ifdef _GNU_SOURCE
    double sh, ch;
    sincos (rph[2], &sh, &ch);
#else
    double sh = sin (rph[2]), ch = cos (rph[2]);
#endif

    // pitch
    rph[1] = atan2 (-R(2,0), R(0,0)*ch + R(1,0)*sh);

    // roll
    rph[0] = atan2 (R(0,2)*sh - R(1,2)*ch, -R(0,1)*sh + R(1,1)*ch);
#undef R
}

static inline void
so3_rot2rph_gsl (const gsl_matrix *R, gsl_vector *rph)
{
    assert (R->size1==3 && R->size2==3 && R->tda==3 && rph->size==3);
    so3_rot2rph (R->data, rph->data);
}

/*
% DROTX  Compute the derivative of a rotation matrix about the X-axis.
%   dRx = DROTZ(PHI) returns [3x3] rotation matrix dRx.  Note: PHI
%   is measured in radians.  PHI measures orientation of coordinate
%   frame 2 relative to coordinate frame 1.
*/
static inline void
so3_drotx (double dRx[3*3], double phi)
{
#ifdef _GNU_SOURCE
    double s, c;
    sincos (phi, &s, &c);
#else
    double s = sin (phi), c = cos (phi);
#endif

    double _dRx[3*3] = { 0,  0,  0,
                         0, -s,  c,
                         0, -c, -s };
    memcpy (dRx, _dRx, sizeof _dRx);
}

static inline void
so3_drotx_gsl (gsl_matrix *dRx, double phi)
{
    assert (dRx->size1==3 && dRx->size2==3 && dRx->tda==3);
    so3_drotx (dRx->data, phi);
}

/*
% DROTY  Compute the derivative of a rotation matrix about the Y-axis.
%   dRy = DROTY(THETA) returns [3x3] rotation matrix dRy.  Note: THETA
%   is measured in radians.  THETA measures orientation of coordinate
%   frame 2 relative to coordinate frame 1.
*/
static inline void
so3_droty (double dRy[3*3], double theta)
{
#ifdef _GNU_SOURCE
    double s, c;
    sincos (theta, &s, &c);
#else
    double s = sin (theta), c = cos (theta);
#endif

    double _dRy[3*3] = { -s,  0, -c,
                          0,  0,  0,
                          c,  0, -s };
    memcpy (dRy, _dRy, sizeof _dRy);
}

static inline void
so3_droty_gsl (gsl_matrix *dRy, double theta)
{
    assert (dRy->size1==3 && dRy->size2==3 && dRy->tda==3);
    so3_droty (dRy->data, theta);
}

/*
% DROTZ  Compute the derivative of a rotation matrix about the Z-axis.
%   dRz = DROTZ(PSI) returns [3x3] rotation matrix dRz.  Note: PSI
%   is measured in radians.  PSI measures orientation of coordinate
%   frame 2 relative to coordinate frame 1.
*/
static inline void
so3_drotz (double dRz[3*3], double psi)
{
#ifdef _GNU_SOURCE
    double s, c;
    sincos (psi, &s, &c);
#else
    double s = sin (psi), c = cos (psi);
#endif

    double _dRz[3*3] = { -s,  c,  0,
                         -c, -s,  0,
                          0,  0,  0 };
    memcpy (dRz, _dRz, sizeof _dRz);
}

static inline void
so3_drotz_gsl (gsl_matrix *dRz, double psi)
{
    assert (dRz->size1==3 && dRz->size2==3 && dRz->tda==3);
    so3_drotz (dRz->data, psi);
}

/*
% QUAT2ROT quaternion to rotation matrix.
%   R = QUAT2ROT(q) takes a 4-vector unit quaternion reprsented by q,
%   (i.e. q = [q0;qx;qy;qz]) and returns the corresponding [3 x 3]
%   orthonormal rotation matrix R.
*/
static inline void
so3_quat2rot (const double q[4], double R[3*3])
{
    double q0 = q[0], qx = q[1], qy = q[2], qz = q[3];

    double q0q0 = q0*q0;
    double qxqx = qx*qx;
    double qyqy = qy*qy;
    double qzqz = qz*qz;

    double q0qx = q0*qx;
    double q0qz = q0*qz;
    double q0qy = q0*qy;
    double qxqy = qx*qy;
    double qxqz = qx*qz;
    double qyqz = qy*qz;

    double _R[3*3] =
        { q0q0 + qxqx - qyqy - qzqz,  2*(qxqy - q0qz),            2*(qxqz + q0qy),
          2*(qxqy + q0qz),            q0q0 - qxqx + qyqy - qzqz,  2*(qyqz - q0qx),
          2*(qxqz - q0qy),            2*(qyqz + q0qx),            q0q0 - qxqx - qyqy + qzqz };
    memcpy (R, _R, sizeof _R);
}

static inline void
so3_quat2rot_gsl (const gsl_vector *q, gsl_matrix *R)
{
    assert (q->size==4 && R->size1==3 && R->size2==3 && R->tda==3);
    so3_quat2rot (q->data, R->data);
}


/*
% ROT2QUAT rotation matrix to quaternion.
%   q = ROT2QUAT(R) returns a 4-vector unit quaternion reprsented by q,
%   (i.e. q = [q0;qx;qy;qz]) corresponding to the [3 x 3]
%   orthonormal rotation matrix R.
*/
static inline void
so3_rot2quat (const double R[3*3], double q[4])
{
    // read
    double r00 = R[0], r01 = R[1], r02 = R[2];
    double r10 = R[3], r11 = R[4], r12 = R[5];
    double r20 = R[6], r21 = R[7], r22 = R[8];

    double tr = r00 + r11 + r22;

    double qw, qx, qy, qz;
    if (tr > 0) {
        const double S = sqrt (tr+1.0) * 2; // S=4*qw
        qw = 0.25 * S;
        qx = (r21 - r12) / S;
        qy = (r02 - r20) / S;
        qz = (r10 - r01) / S;
    }
    else if ((r00 > r11) && (r00 > r22)) {
        double S = sqrt (1.0 + r00 - r11 - r22) * 2; // S=4*qx
        qw = (r21 - r12) / S;
        qx = 0.25 * S;
        qy = (r01 + r10) / S;
        qz = (r02 + r20) / S;
    }
    else if (r11 > r22) {
        double S = sqrt (1.0 + r11 - r00 - r22) * 2; // S=4*qy
        qw = (r02 - r20) / S;
        qx = (r01 + r10) / S;
        qy = 0.25 * S;
        qz = (r12 + r21) / S;
    }
    else {
        double S = sqrt (1.0 + r22 - r00 - r11) * 2; // S=4*qz
        qw = (r10 - r01) / S;
        qx = (r02 + r20) / S;
        qy = (r12 + r21) / S;
        qz = 0.25 * S;
    }

    // write
    q[0] = qw; q[1] = qx; q[2] = qy; q[3] = qz;
}

static inline void
so3_rot2quat_gsl (const gsl_matrix *R, gsl_vector *q)
{
    assert (q->size==4 && R->size1==3 && R->size2==3 && R->tda==3);
    so3_rot2quat (R->data, q->data);
}


/*
% QUAT2RPH converts unit quaternion to Euler RPH.
%   rph = QUAT2RPH(q) returns a [3 x 1] Euler xyz representation
%   equivalent to the [4 x 1] unit quaternion (provided q is
%   not near an Euler singularity).
*/
static inline void
so3_quat2rph (const double q[4], double rph[3])
{
    double R[3*3];
    so3_quat2rot (q, R);
    so3_rot2rph (R, rph);
}

static inline void
so3_quat2rph_gsl (const gsl_vector *q, gsl_vector *rph)
{
    assert (q->size==4 && rph->size==4);
    so3_quat2rph (q->data, rph->data);
}

/*
% RPH2QUAT converts Euler RPH to unit quaternion.
%   q = RPH2QUAT(R) converts [3 x 1] Euler xyz representation
%   to an equivalent [4 x 1] unit quaternion (provided R is
%   not near an Euler singularity).
*/
static inline void
so3_rph2quat (const double rph[3], double q[4])
{
    double R[3*3];
    so3_rotxyz (R, rph);
    so3_rot2quat (R, q);
}

static inline void
so3_rph2quat_gsl (const gsl_vector *rph, gsl_vector *q)
{
    assert (q->size==4 && rph->size==4);
    so3_rph2quat (rph->data, q->data);
}


/*
% function [rph_dot,J] = body2euler(pqr,rph)
% BODY2EULER converts body frame angular rates to Euler rates.
%   RPH_DOT = BODY2EULER(PQR,RPH) converts body frame angular rates PQR to
%   Euler angular rates RPH_DOT given Euler angles RPH.
%
%   [RPH_DOT,J] = BODY2EULER(PQR,RPH) also returns the [3 x 6] Jacobian J
%   associated with the transformation where J = [J_pqr, J_rph].
%   see body2euler.m for cleaner notation
*/
static inline void
so3_body2euler (const double pqr[3], const double rph[3],
                double rph_dot[3], double J[3*6]) {

    double p = pqr[0];
    double q = pqr[1];
    double r = pqr[2];

#ifdef _GNU_SOURCE
    double sr, cr;
    sincos (rph[0], &sr, &cr);
    double sp, cp;
    sincos (rph[1], &sp, &cp);
#else
    double sr = sin (rph[0]), cr = cos (rph[0]);
    double sp = sin (rph[1]), cp = cos (rph[1]);
#endif
    double tp = tan (rph[1]);
    double kp = 1.0/cp;

    //Euler angular rates
    rph_dot[0] = p+sr*tp*q+cr*tp*r;
    rph_dot[1] = cr*q-sr*r;
    rph_dot[2] = sr*kp*q+cr*kp*r;

    if (J != NULL) {
        // Jacobian wrt
        // first row
        J[0] = 1;                   J[1] = sr*tp;                   J[2] = cr*tp;
        J[3] = cr*tp*q-sr*tp*r;     J[4] = sr*kp*kp*q+cr*kp*kp*r;   J[5] = 0;
        // second row
        J[6] = 0;                   J[7] = cr;                      J[8] = -sr;
        J[9] = -sr*q-cr*r;          J[10] = 0;                      J[11] = 0;
        // third row
        J[12] = 0;                  J[13] = sr*kp;                  J[14] = cr*kp;
        J[15] = cr*kp*q-sr*kp*r;    J[16] = sr*kp*tp*q+cr*kp*tp*r;  J[17] = 0;
    }
}

/*
% function [pqr,J] = euler2body(rph_dot,rph)
%   converts euler rates to body frame angular rates
%
%   if not null also returns the [3 x 6] Jacobian J
%   associated with the transformation where J = [rph_dot, J_rph].
%   see body2euler.m for cleaner notation
*/
static inline void
so3_euler2body (const double rph_dot[3], const double rph[3],
                double pqr[3], double J[3*6]) {

    double rd = rph_dot[0];
    double pd = rph_dot[1];
    double hd = rph_dot[2];

#ifdef _GNU_SOURCE
    double sr, cr;
    sincos (rph[0], &sr, &cr);
    double sp, cp;
    sincos (rph[1], &sp, &cp);
#else
    double sr = sin (rph[0]), cr = cos (rph[0]);
    double sp = sin (rph[1]), cp = cos (rph[1]);
#endif

    //body rates
    pqr[0] = -sp*hd + rd;
    pqr[1] = sr*cp*hd + cr*pd;
    pqr[2] = cr*cp*hd - sr*pd;

    if (J != NULL) {
        // Jacobian wrt
        // first row
        J[0] = 1;                   J[1] = 0;                   J[2] = -sp;
        J[3] = 0;                   J[4] = -cp*hd;              J[5] = 0;
        // second row
        J[6] = 0;                   J[7] = cr;                  J[8] = sr*cp;
        J[9] = cr*cp*hd-sr*pd;      J[10] = sr*-sp*hd;          J[11] = 0;
        // third row
        J[12] = 0;                  J[13] = sr;                 J[14] = cr*cp;
        J[15] = -sr*cp*hd-cr*pd;    J[16] = cr*+sp*hd;          J[17] = 0;
    }
}

static inline void
so3_body2euler_gsl (const gsl_vector *pqr, const gsl_vector *rph,
                    gsl_vector *rph_dot, gsl_matrix *J) {

    assert (pqr->size==3 && rph->size==3 && rph_dot->size==3
            && J->size1==3 && J->size2==6 && J->tda==6);

    so3_body2euler (pqr->data, rph->data, rph_dot->data, J->data);

}

#ifdef __cplusplus
}
#endif

#endif // __MATH_SO3_H__
