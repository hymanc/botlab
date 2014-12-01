function S = skewsym(s)
%SKEWSYM  Generates a skew-symmetric matrix.
%  S = SKEWSYM(s) returns a [3x3] skew-symmetric matrix S based
%  upon the [3x1] vector s.  
%  i.e.
%  S = [  0    -s(3)  s(2); ...
%         s(3)  0    -s(1); ...
%	 -s(2)  s(1)  0   ];
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-18-2002      rme         Created and written.


S = [  0    -s(3)  s(2); ...
       s(3)  0    -s(1); ...
      -s(2)  s(1)  0   ];
