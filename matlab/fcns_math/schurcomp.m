function B = schurcomp(A,ii);
%SCHURCOMP computes the Schur complement for a SPD block matrix.
%   B = SCHURCOMP(A,ii) returns the Schur complement, B, for a
%   block of the symmetric positive definite matrix, A, where 
%   ii is an index vector specifying which block.  Recall that the
%   Schur complement is given by:
%   B = Aii - Aij*Ajj^-1*Aji
%   where j is the complement of the index set i.
%
%   Example:
%   A = randcov(5)
%   B = schurcomp(A,1:2) % returns the Schur complement for A(1:2,1:2)
%   B = schurcomp(A,4:5) % returns the Schur complement for A(4:5,4:5)
%   B = schurcomp(A,2:3) % returns the Schur complement for A(2:3,2:3)
%
%   Note that the last example is equivalent to pre and post multiplying
%   A by a permutation matrix, P, and then taking the Schur complement of
%   the upper left [2 x 2] block as:
%   P = permutation_matrix([2,3,1,4,5]);
%   B = schurcomp(P*A*P',1:2)
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    06-16-2006      rme         Created and written.

jj = [1:ii(1)-1,ii(end)+1:size(A,1)];

if isnumeric(A);
  % B = Aii - Aij*Ajj^-1*Aji
  %   = Aii - Aij*(R'*R)^-1*Aji       % where R = chol(Ajj)
  %   = Aii - Aij*(R^-1*R'^-1)*Aji
  %   = Aii - (Aij*R^-1)*(R'^-1*Aji)
  %   = Aij - (Aij*R^-1)*(R'^-1*Aij') % since Aij = Aji' for symmetric A
  %   = Aij - W*W'                    % where W = Aij*R^-1
  if issparse(A);
    W = A(ii,jj)*sparse(inv(chol(A(jj,jj))));
  else;
    W = A(ii,jj)*inv(chol(A(jj,jj)));
  end;
  B = A(ii,ii) - W*W';
else; % sym object
  B = A(ii,ii) - A(ii,jj)*A(jj,jj)^-1*A(jj,ii);
end;
