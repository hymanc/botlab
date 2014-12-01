function [mag,J] = magnitude(t);
% MAGNITUDE computes the L2 norm of a vector
%    MAG = MAGNITUDE(T) returns L2 norm of a vector, i.e. MAG = (T'*T)^0.5
%
%    [MAG,J] = MAGNITUDE(T) also returns the analytical Jacobian evaluated at T.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-22-2004      rme         Created and written.

mag = sqrt(t'*t);
if mag == 0
  mag = mag+eps;
end

if nargout == 2
  % the expression below is the analytical jacobian
  % Ji = d( (t't)^0.5 )/dxi = 0.5*(t't)*2*xi = (t't)*xi
  J = mag^2*t';
end
