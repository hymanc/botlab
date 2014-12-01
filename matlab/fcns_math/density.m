function rho = density(S,tol)
%DENSITY calculates a matrix's density.
%   rho = DENSITY(S) calculates the density of a sparse matrix as
%   rho = nnz(S)/prod(size(S)).
%
%   rho = DENISTY(S,TOL) only counts elements with a magnitude greater
%   than TOL, rho = nnz(abs(S)>TOL)/prod(size(S)).
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    07-05-2004      rme         Created and written.

if nargin == 1
  rho = nnz(S)/prod(size(S));
elseif nargin == 2
  rho = nnz(abs(S)>tol)/prod(size(S));
end
