function y = roundeven(x)
%ROUNDEVEN  rounds towards the nearest even integer.
%   Y = ROUNDEVEN(X) rounds the elements of X towards the nearest even integer.
%
%   Examples:
%   y = roundeven(0.9); % y = 0
%   y = roundeven(1.9); % y = 2
%   y = roundeven(2.0); % y = 2
%   y = roundeven(2.1); % y = 2  
%   y = roundeven(3.0); % y = 4
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    04-30-2006      rme         Created and written.

y = round(x/2)*2;

