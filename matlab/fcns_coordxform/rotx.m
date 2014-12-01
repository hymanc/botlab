function C_x_phi = rotx(phi)
% ROTX  Compute a rotation matrix about the X-axis.
%   R = ROTX(PHI) returns [3x3] rotation matrix R.  Note: PHI
%   is measured in radians.  PHI measures orientation of coordinate
%   frame 2 relative to coordinate frame 1.  Multiplication by rotation
%   matrix R rotates a vector in coordinate frame 1 into coordinate
%   frame 2.
%
%   If PHI is a N-vector then R will be a 3x3xN array where each 3x3xn
%   "slice" corresponds to a rotation matrix based upon PHI(n).
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    17 March 2001   LLW         Created and Written
%    12-02-2002      rme         Added matlab help text and changed
%                                rotation matrix to be consistent with Fossen.
%    04-24-2006      rme         Added capability to stack into 3rd dimension.

c = reshape(cos(phi),1,1,[]);
s = reshape(sin(phi),1,1,[]);
I = ones(size(c));
O = zeros(size(c));

C_x_phi = [ I,   O,  O; ...
            O,   c,  s; ...
            O,  -s,  c  ];
