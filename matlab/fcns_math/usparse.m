function S = usparse(S);
%USPARSE ultimate sparse matrix.
%  S = USPARSE(S) returns a minimally sparse matrix.
%
% History
% DATE          WHO                      WHAT
%----------    ------------------        ------------------------------
%2004-12-03    Ryan Eustice              Created & written.

[i,j,s] = find(S);
[m,n] = size(S);
S = sparse(i,j,s,m,n);
