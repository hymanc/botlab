function [dij,Jij] = euclideanDistance(xi,xj)
%euclideanDistance computes Euclidean distance between two vectors.
%   D = euclideanDistance(Xi,Xj) returns the Euclidean distance D between the 
%   two column vectors Xi & Xj.
%
%   [D,J] = euclideanDistance(Xi,Xj) also computes the Jacobian J of the distance,
%   i.e. J = [dD/dXi, dD/dXj].
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    04-02-2004      rme         Created and written.
%    10-28-2004      rme         Renamed edist.m to euclideanDistance.m


% Euclidean distance between two vectors xi and xj is defined as:
% dij = [ SUM_k (xj(k)-xi(k))^2 ]^0.5
dx = xj - xi;
dij = sqrt(dx'*dx);

if nargout > 1
  % Jacobian associated with Euclidean distance is:
  % Jij = [d(dij)/dxi, d(dij)/dxj]
  %
  % where d(dij)/dxi = -(1/dij)*dx'
  %       d(dij)/dxj =  (1/dij)*dx';
  Jij = 1/dij*[-dx', dx'];
end
