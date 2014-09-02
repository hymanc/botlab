function q = aa2quat(r)
%AA2QUAT axis-angle rotation to unit quaternion.
%   q = AA2QUAT(r) returns a [4 x 1] unit quaternion
%   equivalent to the [3 x 1] axis angle representation.
%
%   This algorithm is based upon:
%   Horn, B.K.P.  Relative Orientation, MIT A.I. Memo #994 September 1987
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-12-2003      rme         Created and written.
%    09-20-2003      rme         Renamed to aa2quat.m

if size(r,2) > 1
  % reshape into a column vector
  r = r'; 
end

theta = norm(r);

rotaxis = r/theta;

q = [cos(theta/2); sin(theta/2)*rotaxis];
