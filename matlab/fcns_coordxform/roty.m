function C_y_theta = roty(theta)
% ROTY  Compute a rotation matrix about the Y-axis.
%   R = ROTY(THETA) returns [3x3] rotation matrix R.  Note: THETA
%   is measured in radians.  THETA measures orientation of coordinate
%   frame 2 relative to coordinate frame 1.  Multiplication by rotation
%   matrix R rotates a vector in coordinate frame 1 into coordinate
%   frame 2.
%
%   If THETA is a N-vector then R will be a 3x3xN array where each 3x3xn
%   "slice" corresponds to a rotation matrix based upon THETA(n).
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    17 March 2001   LLW         Created and Written
%    12-02-2002      rme         Added matlab help text and changed
%                                rotation matrix to be consistent with Fossen.
%    04-24-2006      rme         Added capability to stack into 3rd dimension.

c = reshape(cos(theta),1,1,[]);
s = reshape(sin(theta),1,1,[]);
I = ones(size(c));
O = zeros(size(c));

C_y_theta = [ c,  O,  -s; ...
              O,  I,   O; ...
              s,  O,   c  ];
