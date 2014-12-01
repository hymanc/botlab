function [X] = homogenize(varargin)
%HOMOGENIZE  Convert to homogenous representation.
%  [X] = HOMOGENIZE(x,y) returns [3xN] homogenous matrix computed from
%  nonhomgenous [Nx1] vectors x and y.
%
%  [X] = HOMOGENIZE(x,y,z) returns [4xN] homgenous matrix computed from
%  nonhomgenous [Nx1] vectors x, y, and z.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-19-2002      rme         Created and written.
%    04-18-2004      rme         Fixed missing transpose on varargin{3}
  
switch nargin
 case 2
  X = [varargin{1}'; varargin{2}'; ones(1,length(varargin{1}))];
 case 3
  X = [varargin{1}'; varargin{2}'; varargin{3}'; ones(1,length(varargin{1}))];
 otherwise
  error('Inappropriate number of arguments');
end
