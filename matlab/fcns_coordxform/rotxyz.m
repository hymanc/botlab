function R = rotxyz(varargin)
% ROTXYZ  Compute a rotation matrix about the XYZ-axes.
%   R = ROTXYZ(RPH) returns [3x3] rotation matrix R where RPH
%   is a 3-vector of Euler angles [roll,pitch,heading] measured in
%   radians.  RPH measures orientation of coordinate frame 2
%   relative to coordinate frame 1.  Multiplication by rotation
%   matrix R rotates a vector in coordinate frame 2 into coordinate
%   frame 1.
%
%   R = ROTXYZ(R,P,H) computes the same result as above, though, if R,P,H
%   are each N-vectors, then R will be a 3x3xN array where each 3x3xn
%   "slice" corresponds to a rotation matrix based upon R(n),P(n),H(n).
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    17 March 2001   LLW         Created and Written
%    12-02-2002      rme         Added matlab help text and changed
%                                rotation matrix to be consistent with Fossen.
%    07-20-2003      rme         Fixed a typo in rotmatrix code comment
%    08-14-2003      rme         Explicitly wrote out rotation matrix to
%                                speed up the calculation.
%    04-27-2006      rme         Added capability to stack into 3rd dimension.

if (nargin == 1);
  rph = varargin{1};
  r = rph(1);
  p = rph(2);
  h = rph(3);
elseif (nargin == 3);
  r = reshape(varargin{1},1,1,[]);
  p = reshape(varargin{2},1,1,[]);
  h = reshape(varargin{3},1,1,[]);
else;
  error('Incorrect number of arguments');
end;

% R = yaw_matrix' * pitch_matrix' * roll_matrix'
% R = rotz(h)' * roty(p)' * rotx(r)';

% note: the above 3 principle rotations are equivalent to:
cr = cos(r); sr = sin(r);
cp = cos(p); sp = sin(p);
ch = cos(h); sh = sin(h);

R = [ ch.*cp,   (-sh.*cr + ch.*sp.*sr),   ( sh.*sr + ch.*sp.*cr); ...
      sh.*cp,   ( ch.*cr + sh.*sp.*sr),   (-ch.*sr + sh.*sp.*cr); ...
      -sp,          cp.*sr,                  cp.*cr        ];

