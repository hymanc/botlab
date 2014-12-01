function x = jacobi_iterate(A,x,b,K)

N = length(x);

if ~exist('K','var') || isempty(K), K = N; end

% create a diagonal matrix Ainvdiag containing the the inverse of the 
% diagonal elements of A
Ainvdiag = spdiags(diag(A).^-1,0,spalloc(N,N,N));

% set the main diagonal of A to zero
A = spdiags(spalloc(N,1,0),0,A);

%keyboard
for k=1:K
  x = Ainvdiag*(b - A*x);
end
