function A = symmetrize(A)
%SYMMETRIZE forces a matrix to be symmetric.
%   A = SYMMETRIZE(A) forces the matrix A to be symmetric by setting 
%   A = (A+A')/2;
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    06-24-2004      rme         Created and written.

A = (A+A')/2;
