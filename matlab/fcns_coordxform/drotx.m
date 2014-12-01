function dRx = drotx(r)
% DROTX  Compute the derivative of a rotation matrix about the X-axis.
%   dRx = DROTZ(PHI) returns [3x3] rotation matrix dRx.  Note: PHI
%   is measured in radians.  PHI measures orientation of coordinate
%   frame 2 relative to coordinate frame 1.
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    08/2003         rme         Created from rotx.m
  
c = cos(r);
s = sin(r);
dRx = [ 0  0  0; ...
        0 -s  c; ...
	0 -c -s ];
