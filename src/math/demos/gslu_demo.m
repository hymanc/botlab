function gslu_demo()

fprintf('===================================================================\n');
disp('gsl_util_vector')
fprintf('===================================================================\n');

x = [0 0 1 0]'

xx = x

y = x

isel = 3:4;

x_sub = x(isel)

y(1:2) = x_sub

sum_ = sum (y)

dist = sqrt (sum (y-x).^2)

a = [1, 2, 3]';
b = [6, 5, 4]';
c = cross (a, b)

stp = a'* cross (b, c)

vtp = cross (a, cross (b, c))



fprintf('===================================================================\n');
disp('gsl_util_matrix')
fprintf('===================================================================\n');

A = [4, 10, 7, 4; 0, 1, 7, 6; 7, 5, 7, 1; 8, 6, 7, 1]
A_transpose = A'
B = [9 1 8 2; 10 3 2 3; 5 8 9 6; 1 3 3 5]
C = reshape (0:15, 4, 4)'

D = C(isel,:)

E = C(:,isel)

Aequal = all(all(A == A))

Adet = det(A)

Atrace = trace(A)

Ainv = A^-1

C(:,3:4) = E;
C(3:4,:) = D

C_2x8 = reshape (C, 2, 8)

Ctrans_2x8 = reshape (C', 2, 8)

c = C(:)

Cprime = reshape (c, 4, 4)


t = [1, 2, 3]';
skew = skewsym (t)



fprintf('===================================================================\n');
disp('gsl_util_blas')
fprintf('===================================================================\n');

b = A*x

btrans = x'*A

scalar = x'*A*y

C1 = A*B
C2 = A*B'
C3 = A'*B
C4 = A'*B'

D1 = A*B*C
D2 = A*B*C'
D3 = A*B'*C
D4 = A'*B*C
D5 = A*B'*C'
D6 = A'*B'*C
D7 = A'*B*C'
D8 = A'*B'*C'



fprintf('===================================================================\n');
disp('gsl_util_linalg')
fprintf('===================================================================\n');

disp('QR factorization of A')
[Q,R] = qr (A)

disp('SVD decomposition of A (econ) for thin or square matrices')
[U,S,V] = svd (A)

disp('SVD decomposition of A_fat (full)');
A_fat = A(1:3,1:4)
[U2,S2,V2] = svd (A_fat)

disp('Reduced row echelon form of A');
A_rref = rref(A)


if 0
fprintf('===================================================================\n');
disp('gsl_util_rand')
fprintf('===================================================================\n');

w = zeros(5,1)
W = zeros(5,5)


% mean and covariance
rho = 0.75;
mu = [0, 0, 200, 300, 400]'
Sigma = [   1, -rho,   0, 0,     0; ...
         -rho,    1,   0, 0,     0; ...
            0,    0, 0.1, 0,     0; ...
            0,    0,   0, 1,     0; ...
            0,    0,   0, 0, 0.001 ]
end
