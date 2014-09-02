function Ainv = inv3x3 (A)

a = A(1,1); b = A(1,2); c = A(1,3);
d = A(2,1); e = A(2,2); f = A(2,3);
g = A(3,1); h = A(3,2); i = A(3,3);

det = a*(e*i-f*h) -b*(d*i-f*g) + c*(d*h-e*g);

Ainv = 1/det * ...
    [e*i-f*h, c*h-b*i, b*f-c*e; ...
     f*g-d*i, a*i-c*g, c*d-a*f; ...
     d*h-e*g, b*g-a*h, a*e-b*d];
