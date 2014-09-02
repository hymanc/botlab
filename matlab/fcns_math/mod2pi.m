function angles = mod2pi(angles)
%MOD2PI  Wraps angles between [-pi,pi].
%   WRAPPED = MOD2PI(ANGLES) wraps the entries of ANGLES between [-pi,pi].

while any(angles < -pi)
    angles(angles < -pi) = angles(angles < -pi) + 2*pi;
end

while any(angles > pi)
    angles(angles > pi) = angles(angles > pi) - 2*pi;
end
