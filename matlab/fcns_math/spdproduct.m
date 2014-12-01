function C = spdproduct(A,B)
%SYMMETRIC POSITIVE DEFINITE PRODUCT
%    C = spdproduct(A,B) computes C = B'*A*B using Cholesky decomposition
%    of the positive definite matrix A so that symmetry of C is preserved.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    10-26-2004      rme         Created and written.
%    11-03-2004      rme         Added sparse check of matrix B
%    11-27-2004      rme         Renamed to spdproduct.m

if issparse(B)
  W = sparse(chol(A))*B;
else
  W = chol(A)*B;
end
C = W'*W;
