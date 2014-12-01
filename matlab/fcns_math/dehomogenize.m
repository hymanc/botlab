function [varargout] = dehomogenize(X)
%DEHOMOGENIZE  Convert to nonhomogenous representation.
%  [x,y] = DEHOMOGENIZE(X) converts [3xN] homogenous vector into
%  nonhomgenous [Nx1] vectors x and y.
%
%  [x,y,z] = DEHOMOGENIZE(X) converts [4xN] homgenous vector into
%  nonhomgenous [Nx1] vectors x, y, and z.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-19-2002      rme         Created and written.

switch size(X,1)
 case 3
  varargout{1} = (X(1,:)./X(3,:))';
  varargout{2} = (X(2,:)./X(3,:))';
 case 4
  varargout{1} = (X(1,:)./X(4,:))';
  varargout{2} = (X(2,:)./X(4,:))';
  varargout{3} = (X(3,:)./X(4,:))';
 otherwise
  error('Inappropriate input size, should be [3xN]');
end
