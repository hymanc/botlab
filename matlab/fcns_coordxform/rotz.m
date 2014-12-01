function C_z_psi = rotz(psi)
% ROTZ  Compute a rotation matrix about the Z-axis.
%   R = ROTZ(PSI) returns [3x3] rotation matrix R.  Note: PSI
%   is measured in radians.  PSI measures orientation of coordinate
%   frame 2 relative to coordinate frame 1.  Multiplication by rotation
%   matrix R rotates a vector in coordinate frame 1 into coordinate
%   frame 2.
%
%   If PSI is a N-vector then R will be a 3x3xN array where each 3x3xn
%   "slice" corresponds to a rotation matrix based upon PSI(n).
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    17 March 2001   LLW         Created and Written
%    12-02-2002      rme         Added matlab help text and changed
%                                rotation matrix to be consistent with Fossen.
%    04-24-2006      rme         Added capability to stack into 3rd dimension.

c = reshape(cos(psi),1,1,[]);
s = reshape(sin(psi),1,1,[]);
I = ones(size(c));
O = zeros(size(c));

C_z_psi = [ c,  s,  O; ...
           -s,  c,  O; ... 
            O,  O,  I  ];
