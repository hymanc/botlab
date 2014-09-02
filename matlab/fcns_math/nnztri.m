function nz = nnztri(n,k)
%NNZTRI number of nonzero triangular matrix elements.
%   NZ = NNZTRI(N,K) computes the number of elements on and above the K-th
%   diagonal of a [N x N] matrix.  Note that K = 0 is the default corresponding
%   to the main diagonal.
%
%   See also TRIL, TRIU.
%
%   Ryan M. Eustice 08-15-2003
%   Woods Hole Oceanographic Institution, MS 7
%   Woods Hole, MA 02543
%   508.289.3269   ryan@whoi.edu
%
% History
% DATE          WHO                      WHAT
%----------    ------------------        ------------------------------
%2004-04-08    Ryan Eustice              Created & written.
%2004-11-02    rme                       Make K=0 default

if ~exist('k','var') || isempty(k)
  k = 0;
end

nz = n*(n+1)/2 - sum(n:-1:(n-k+1));
