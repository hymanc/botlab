function x_fix = funwrapd(x,DIM)
%FUNWRAPD  Fast unwrapping of phase angles in degrees.
%    See FUNWRAP for syntax.  
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    10-28-2006      rme         Created and written.

if nargin == 1;
  x_fix = 180/pi * funwrap(x*pi/180);
elseif nargin == 2;
  x_fix = 180/pi * funwrap(x*pi/180, DIM);
else;
  error('wrong number of arguments');
end;
