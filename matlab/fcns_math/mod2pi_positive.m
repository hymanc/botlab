function vout = mod2pi_positive(vin)
%MOD2PI_POSITIVE  Wraps angles between [0,2*pi].
%   WRAPPED = MOD2PI_POSITIVE(ANGLES) wraps the entries of ANGLES.

vout = vin - 2*pi*floor(vin/(2*pi));
