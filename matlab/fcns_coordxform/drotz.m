function dRz = drotz(h)
% DROTZ  Compute the derivative of a rotation matrix about the Z-axis.
%   dRz = DROTZ(PHI) returns [3x3] rotation matrix dRz.  Note: PHI
%   is measured in radians.  PHI measures orientation of coordinate
%   frame 2 relative to coordinate frame 1.
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    08/2003         rme         Created from rotz.m

c = cos(h);
s = sin(h);
dRz = [-s  c  0; ...
       -c -s  0; ...
        0  0  0];
