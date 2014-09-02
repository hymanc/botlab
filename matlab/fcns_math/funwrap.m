function x_fix = funwrap(x,DIM,MOD)
%FUNWRAP  fast unwrapping of phase angles.
%   THETA_UNWRAP = FUNWRAP(THETA) unwraps the vector THETA by changing
%   absolute jumps greater than pi to their 2*pi complement.
%
%   THETA_UNWRAP = FUNWRAP(THETA,DIM) unwraps the matrix THETA along the
%   dimension DIM.
%
%   X_UNWRAP = FUNWRAP(X,DIM,MOD) unwraps the matrix X along the dimension
%   DIM by jumps greater than MOD.  i.e., in the above MOD=2*pi
%
%   For vectors and matrices FUNWRAP is approximately 10x faster on my
%   1.8GHz machine than MATLAB's UNWRAP function.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    03-29-2004      rme         Created and written.
%    04-12-2004      rme         Renamed to funwrap.m and made DIM
%                                argument optional for vectors.
%    01-10-2009      rme         Added MOD argument.

% check DIM argument condition
if ~exist('DIM','var')
  if numel(x) == length(x)
    if size(x,1) > size(x,2)
      DIM = 1;
    else
      DIM = 2;
    end
  else
    error('you must specify DIM for matrices');
  end
end

if ~exist('MOD','var')
    MOD = 2*pi;
end

% unwrap algorithm
if DIM == 1
  dx = [zeros(1,size(x,2)); diff(x,1,DIM)];  
elseif DIM == 2
  dx = [zeros(size(x,1),1), diff(x,1,DIM)];
else
  error('DIM must be either 1 or 2');
end
x_fix = x - cumsum(round(dx./MOD)*MOD,DIM);
