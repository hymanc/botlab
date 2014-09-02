function [t_hat,J] = unitize(t)
% UNITIZE normalizes a vector to magnitude 1.
%    T_HAT = UNITIZE(T) returns a unit vector in the same direction as
%    the vector T where T is a column vector.
%
%    [T_HAT,J] = UNITIZE(T) also returns the analytical Jacobian evaluated at T.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-03-2003      rme         Created and written.
%    10-02-2003      rme         Added return of analytical Jacobian.
%    11-20-2003      rme         Added check for zero magnitude.

mag = sqrt(t'*t);
if mag == 0
  mag = mag+eps;
end
t_hat = t/mag;

if nargout == 2
  % the expression below is the analytical jacobian for the unitize
  % function which has two equal and one zero singular values.
  % Jt_anal = 
  %[ (y^2+z^2)/(x^2+y^2+z^2)^(3/2),      -x/(x^2+y^2+z^2)^(3/2)*y,      -x/(x^2+y^2+z^2)^(3/2)*z]
  %[      -x/(x^2+y^2+z^2)^(3/2)*y, (x^2+z^2)/(x^2+y^2+z^2)^(3/2),      -y/(x^2+y^2+z^2)^(3/2)*z]
  %[      -x/(x^2+y^2+z^2)^(3/2)*z,      -y/(x^2+y^2+z^2)^(3/2)*z, (x^2+y^2)/(x^2+y^2+z^2)^(3/2)]
  x = t(1);
  y = t(2);
  z = t(3);
  alpha = mag^3;
  J = (1/alpha)*[(y^2+z^2),      -x*y,      -x*z; ...
		      -x*y, (x^2+z^2),      -y*z; ...
		      -x*z,      -y*z, (x^2+y^2)];
end
