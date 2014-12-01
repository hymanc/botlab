function LambdaInv = blkTriDiagInv(Lambda,index_t)

% shorthand
Xf_ii = index_t.Xf_ii;

% number of blocks
N = index_t.Nf+1;

% allocate output storage
LambdaInv = zeros(size(Lambda));

% allocate local variables
Delta = zeros(index_t.Naug,index_t.Nv);
Sigma = zeros(index_t.Naug,index_t.Nv);
U = zeros(index_t.Naug,index_t.Nv);
V = zeros(index_t.Nv,index_t.Naug);

Delta(Xf_ii{1},:) = Lambda(Xf_ii{1},Xf_ii{1});   % Delta(1)
Sigma(Xf_ii{N},:) = Lambda(Xf_ii{N},Xf_ii{N});   % Sigma(N)
for k=2:N
  % populate Delta via forward recursion
  D =  full(Lambda(Xf_ii{k},Xf_ii{k}));                     % D(k)
  A = -full(Lambda(Xf_ii{k},Xf_ii{k-1}));                   % A(k)
  Delta(Xf_ii{k},:) = D - A * Delta(Xf_ii{k-1},:)^-1 * A';  % Delta(k)
  
  % populate Sigma via backwards recursion
  n = N-k+1;
  D =  full(Lambda(Xf_ii{n},Xf_ii{n}));                     % D(n)
  A = -full(Lambda(Xf_ii{n+1},Xf_ii{n}));                   % A(n+1)
  Sigma(Xf_ii{n},:) = D - A' * Sigma(Xf_ii{n+1},:)^-1 * A;  % Sigma(n)
end


A = -full(Lambda(Xf_ii{2},Xf_ii{1}));
U(Xf_ii{1},:) = (A^-1)'*Delta(Xf_ii{1},:);  % U1
V(:,Xf_ii{1}) = Sigma(Xf_ii{1},:)^-1*A'*Sigma(Xf_ii{1},:)^-1;
for k=2:N
  % populate U via forward recursion
  A = -full(Lambda(Xf_ii{k},Xf_ii{k-1}));
  U(Xf_ii{k},:) = (A^-1)'*Delta(Xf_ii{k-1},:)*U(Xf_ii{k-1},:);
  
  % populate V via forward recursion
  V(:,Xf_ii{k}) = V(:,Xf_ii{k-1})*A'*Sigma(Xf_ii{k},:)^-1;
  
end

LambdaInv = U*V;

U1V1 = U(Xf_ii{1},:)*V(:,Xf_ii{1});

keyboard;
