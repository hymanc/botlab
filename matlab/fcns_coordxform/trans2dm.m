function [b,J] = trans2dm(t)
%TRANS2DM translation vector to direction-magnitude.
%
% b = TRANS2DM(t) converts a a [3 x 1] translation vector 
% (i.e. t = [tx ty tz]') to a  [3 x 1] direction-magnitude vector 
% (i.e. b = [azimuth, elevation, magnitude]').
%
% [b,J] = TRANS2DM(t) also returns the Jacobian of this transformation
% which is useful for propogating 1st order covariances e.g.
% Cov_b = J*Cov_t*J';
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    2003            op          Created and written.
%    09-20-2003      rme         Rewrote jacobian_t2b and renamed parent
%                                function to trans2dm.m
%    04-06-2004      rme         Added alpha = 0 check in jacobian_t2b

b = t2b(t);
if nargout > 1
  J = jacobian_t2b(t);
end

%**************************************************
function b = t2b(t)

x = t(1); % x = mag*cos(elev)*cos(azim)
y = t(2); % y = mag*cos(elev)*sin(azim)
z = t(3); % z = mag*sin(elev)

azim = atan2(y,x);
elev = atan2(z,sqrt(x^2+y^2));
mag  = norm(t);

b = [azim;elev;mag];

%**************************************************
function J = jacobian_t2b(t)
x = t(1);
y = t(2);
z = t(3);
mag = sqrt(x^2+y^2+z^2);
alpha = sqrt(x^2+y^2); 
if alpha == 0
  alpha = alpha+eps; % avoid division by zero
end

% analytical expression for jacobian
J = [-y/alpha^2,          x/alpha^2,         0; ...
     -z*x/(alpha*mag^2), -z*y/(alpha*mag^2), alpha/mag^2; ...
      x/mag,              y/mag,             z/mag];
