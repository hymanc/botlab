function vout = mod2pi(vin)
%MOD2PI  Wraps angles between [-pi,pi].
%   WRAPPED = MOD2PI(ANGLES) wraps the entries of ANGLES.

vout = mod2pi_positive (vin+pi)-pi;
