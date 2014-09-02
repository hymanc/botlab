function x = spdmldivide(A,b);
%SYMMETRIC POSITIVE DEFINITE MATRIX LEFT DIVIDE
%    x = spdmldivide(A,b) uses Cholesky decomposition of the symmetric
%    positive definite matrix A to efficiently solve the system A*x=b via
%    M = chol(A);
%    x = M \ (M' \ b);
%
%    Note that Matlab's builtin mldivide.m does the same thing but first
%    must run some checks to determine that A is SPD.  Therefore calling
%    spdldivide.m directly is faster.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-27-2004      rme         Created and written.

p = symmmd(A);
M = chol(A(p,p));
x(p,1) = M \ (M' \ b(p));
