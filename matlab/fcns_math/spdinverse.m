function B = spdinverse(A)
%SYMMETRIC POSITIVE DEFINITE INVERSE
%    B = spdinverse(A) uses Cholesky decomposition of A to
%    preserve numerical symmetry in it's inverse B.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-01-2004      rme         Created and written.
%    11-27-2004      rme         Renamed to spdinverse.m
%    11-27-2004      rme         Added call to full() in Rinv calc.

% Cholesky decomposition
R = chol(A); % A = R'*R
Rinv = full(R^-1);

B = Rinv*Rinv';
