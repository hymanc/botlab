function P = permutation_matrix(varargin)
%PERMUTATION_MATRIX
%    P = PERMUTATION_MATRIX(ii) generates a [n x n] permutation matrix, P,
%    where n = length(ii), such that when multiplied by a [n x 1] vector,
%    P rearranges the elements according to the order specified by index ii
%    (e.g., b = P*a).  Similarly, the permutation matrix can be used to reorder
%    a [n x n] covariance matrix as B = P*A*P'.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    07-09-2004      rme         Created and written.
%    06-16-2006      rme         Simplified input to one argument.

if (nargin==1);
  ii = varargin{1};
  n  = length(ii);
else; % the obsolete 2 argument format is included here for backwards compatibility
  n = varargin{1};
  ii = varargin{2};
end;

P = speye(n);
P = P(ii,:);
