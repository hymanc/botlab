function Ainv = inv4x4 (A)

a = A(1,1); b = A(1,2); c = A(1,3); d = A(1,4);
e = A(2,1); f = A(2,2); g = A(2,3); h = A(2,4);
i = A(3,1); j = A(3,2); k = A(3,3); l = A(3,4);
m = A(4,1); n = A(4,2); o = A(4,3); p = A(4,4);

det = ...
              a*(f*(k*p - l*o) - j*(g*p - h*o) + n*(g*l - h*k)) - ...
              e*(b*(k*p - l*o) - j*(c*p - d*o) + n*(c*l - d*k)) + ...
              i*(b*(g*p - h*o) - f*(c*p - d*o) + n*(c*h - d*g)) - ...
              m*(b*(g*l - h*k) - f*(c*l - d*k) + j*(c*h - d*g));

Ainv = zeros(4);

Ainv(1,1) =  (f*(k*p - l*o) - j*(g*p - h*o) + n*(g*l - h*k)) / det;
Ainv(1,2) = -(b*(k*p - l*o) - j*(c*p - d*o) + n*(c*l - d*k)) / det;
Ainv(1,3) =  (b*(g*p - h*o) - f*(c*p - d*o) + n*(c*h - d*g)) / det;
Ainv(1,4) = -(b*(g*l - h*k) - f*(c*l - d*k) + j*(c*h - d*g)) / det;

Ainv(2,1) = -(e*(k*p - l*o) - i*(g*p - h*o) + m*(g*l - h*k)) / det;
Ainv(2,2) =  (a*(k*p - l*o) - i*(c*p - d*o) + m*(c*l - d*k)) / det;
Ainv(2,3) = -(a*(g*p - h*o) - e*(c*p - d*o) + m*(c*h - d*g)) / det;
Ainv(2,4) =  (a*(g*l - h*k) - e*(c*l - d*k) + i*(c*h - d*g)) / det;

Ainv(3,1) =  (e*(j*p - l*n) - i*(f*p - h*n) + m*(f*l - h*j)) / det;
Ainv(3,2) = -(a*(j*p - l*n) - i*(b*p - d*n) + m*(b*l - d*j)) / det;
Ainv(3,3) =  (a*(f*p - h*n) - e*(b*p - d*n) + m*(b*h - d*f)) / det;
Ainv(3,4) = -(a*(f*l - h*j) - e*(b*l - d*j) + i*(b*h - d*f)) / det;

Ainv(4,1) = -(e*(j*o - k*n) - i*(f*o - g*n) + m*(f*k - g*j)) / det;
Ainv(4,2) =  (a*(j*o - k*n) - i*(b*o - c*n) + m*(b*k - c*j)) / det;
Ainv(4,3) = -(a*(f*o - g*n) - e*(b*o - c*n) + m*(b*g - c*f)) / det;
Ainv(4,4) =  (a*(f*k - g*j) - e*(b*k - c*j) + i*(b*g - c*f)) / det;
