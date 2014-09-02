function y = roundodd(x)
%ROUNDODD  rounds towards the nearest odd integer.
%   Y = ROUNDODD(X) rounds the elements of X towards the nearest odd integer.
%   
%   Examples:
%   y = roundodd(1.9); % y = 1
%   y = roundodd(2.0); % y = 3
%   y = roundodd(2.1); % y = 3  
%   y = roundodd(3.9); % y = 3
%   y = roundodd(4.0); % y = 5
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    04-30-2006      rme         Created and written.

y = round(x/2)*2;

dy = x-y;
if (dy < 0);
  y = y + floor(dy);
else;
  y = y + ceil(dy+eps);
end;
