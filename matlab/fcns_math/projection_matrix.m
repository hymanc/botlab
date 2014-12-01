function S = projection_matrix(n,ii)
%PROJECTION_MATRIX
%    S = PROJECTION_MATRIX(n,ii) returns a projection matrix S over the
%    state space of size n which extracts elements defined by indicies ii.
%    S's transpose multiplied by a n-vector extracts the elements
%    specified by the index ii,  e.g. b = S'*a.  Similarly the projection
%    matrix can be used to extract a subset of elements from a [n x n]
%    covariance matrix, e.g. B = S'*A*S.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    07-09-2004      rme         Created and written.
%    12-14-2004      rme         Updated help file.

% organize ii as a row vector
ni = length(ii);  
ii = reshape(ii,1,ni);

% generate a sparse projection matrix
S = spalloc(n,ni,ni);
S(sub2ind([n,ni],ii,1:ni)) = 1;
