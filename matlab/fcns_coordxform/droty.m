function dRy = droty(p)
% DROTY  Compute the derivative of a rotation matrix about the Y-axis.
%   dRy = DROTY(PHI) returns [3x3] rotation matrix dRy.  Note: PHI
%   is measured in radians.  PHI measures orientation of coordinate
%   frame 2 relative to coordinate frame 1.
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    08/2003         rme         Created from roty.m
  
c = cos(p);
s = sin(p);
dRy = [ -s  0 -c; ...
	 0  0  0; ...
	 c  0 -s];
