function Tij = pvec2tmat(x_ij);
%PVEC2TMAT  maps a 6 DOF pose vector to a transformation matrix.
%    Tij = PVEC2TMAT(x_ij) computes a [4 x 4] homogenous coordinate transformation
%    matrix Tij based upon the [6 x 1] pose vector x_ij.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-14-2004      rme         Created and written.

Tij = [rotxyz(x_ij(4:6)), x_ij(1:3); 0 0 0 1];
  
