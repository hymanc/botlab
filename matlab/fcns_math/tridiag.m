function B = tridiag(A,n)
%TRIDIAG  extracts block tridiagonal elements from a matrix
%    B = TRIDIAG(A,n) returns the matrix B containing the block
%    tridiagonal elements from the square matrix A where n is the size
%    of each submatrix.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    10-25-2004      rme         Created and written.

N = size(A,1);
Nblks = N/n;

B = spalloc(N,N,Nblks*(3*n^2));

ii = 1:n;
jj = 1:n;
kk = jj+n;
for k=1:Nblks-1
  B(jj,ii) = A(jj,ii);
  B(jj,jj) = A(jj,jj);
  B(jj,kk) = A(jj,kk);
  ii = jj;
  jj = kk;
  kk = kk + n;
end
B(jj,ii) = A(jj,ii);
B(jj,jj) = A(jj,jj);

