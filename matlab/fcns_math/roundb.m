function y = roundb(x,base)
%ROUNDB rounds to specified precision.
%    Y = ROUNDB(X,BASE) rounds X to the specified level of precision
%    declared in BASE.
%
%    Example:
%    roundb(3.14159,0.1)  returns 3.1
%    roundb(3.14159,0.2)  returns 3.2
%    roundb(3.14159,1)    returns 3
%    roundb(3.14159,10)   returns 0
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-22-2004      rme         Created and written.
%    10-28-2006      rme         Renamed from roundunit.m to roundb.m

y = round(x/base)*base;
