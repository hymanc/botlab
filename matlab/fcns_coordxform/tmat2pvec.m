function x_ij = tmat2pvec(Tij);
%TMAT2PVEC  maps a transformation matrix to a 6 DOF pose vector
%    x_ij = TMAT2PVEC(Tij) computes the [6 x 1] pose vector x_ij from the
%    [4 x 4] homogenous coordinate transformation matrix Tij.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-14-2004      rme         Created and written.

x_ij = [Tij(1:3,4); rot2rph(Tij(1:3,1:3))];
