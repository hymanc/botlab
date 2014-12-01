function D =  scalar_triple_product(A,B,C)
% SCALAR_TRIPLE_PRODUCT
%    D = SCALAR_TRIPLE_PRODUCT(A,B,C) returns D = A dot (B cross C)
%    where A, B, and C are 3-vectors.  If instead A, B, and C are
%    3xN matrices then D is a 1xN matrix of the result for each triplet
%    of column vectors for A, B, and C.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-10-2003      rme         Created and written.

D = dot(A,cross(B,C));
