function [plane_coeff, min_in] = fit_plane_RANSAC(xyz)
% given xyz point fit a plane using RANSAC
%
% xyz = 4 x num matrix

x = xyz(1,:); y = xyz(2,:); z = xyz(3,:);

THRESH = 1E-4; %6e-5;
MIN_ITERATIONS = 100;
MAX_ITERATIONS = 500;

p = length(x); % set size
q = 3;          % sample size (for 6-point algorithm)

iterations = 0; min_in = 0;
while (iterations < MIN_ITERATIONS) || (iterations < MAX_ITERATIONS)
    rsel = randsample(p,q);
    x_sample = x(rsel); y_sample = y(rsel); z_sample = z(rsel);

    a = -det([ones(q,1) y_sample' z_sample']);
    b = -det([x_sample' ones(q,1) z_sample']);
    c = -det([x_sample' y_sample' ones(q,1)]);
    d =  det([x_sample' y_sample' z_sample']);

    coeff = [a b c d]'/sqrt(a^2+b^2+c^2);
    error = xyz'*coeff;
    n_in = sum(abs(error) < THRESH);

    if n_in > min_in
        min_in = n_in;
        plane_coeff = coeff;
    end
    iterations = iterations +1;
end
